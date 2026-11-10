#!/usr/bin/env python3
"""
zros_driver.py — ROS 2 <-> ZoneRobotics (ZRCar) bridge node
===========================================================

Bridges a ZRCar (ESP32 bot) into ROS 2 using the zrsense LAN SDK:

  ROS topic (in)   ->  bot action
  --------------------------------------------------------------
  /cmd_vel  (Twist)      linear.x  -> throttle  (bot.car.raw)
                         angular.z -> steering  (bot.car.set_steering)
  /servo_angle (Int32)   -> an AUX hobby servo (bot.servo.set_angle)

  bot stream       ->  ROS topic (out)
  --------------------------------------------------------------
  JPEG video       ->  /camera/image_raw  (sensor_msgs/Image, bgr8)
  MPU6050 telem    ->  /imu/data          (sensor_msgs/Imu)
  ToF distance     ->  /range             (sensor_msgs/Range)
  swept-ToF scan   ->  /scan              (sensor_msgs/LaserScan)

  ROS service      ->  action
  --------------------------------------------------------------
  /scan_room (std_srvs/Trigger)   sweep the steering servo and publish one
                                  /scan LaserScan. The rangefinder is geared
                                  3:1 to the steering servo, so a ±60° servo
                                  sweep traces a full 360° room cross-section.
                                      ros2 service call /scan_room std_srvs/srv/Trigger

Run:
    python3 zros_driver.py --ros-args -p robot_ip:=192.168.1.46

Notes / gotchas (from the zrsense SDK — ZROS/PythonAPI):
  * websocket-client MUST be 1.6.4 (newer versions reject the ESP32's frames).
  * ONE WebSocket client at a time — close the browser /car tab before running.
  * The bot's camera must be enabled for video, and the rangefinder/IMU
    enabled for /range and /imu (all toggle in the bot's /console -> Components).
  * This is a CAR: throttle and steering are driven together (Ackermann-style),
    not tank-style. angular.z steers the front wheels; it does NOT spin in place.
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu, Range, LaserScan
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

# zrsense.py lives one directory up (ZROS/PythonAPI/zrsense.py). run.sh puts it
# on PYTHONPATH; if you run this by hand, add that dir to PYTHONPATH first.
from zrsense import Robot

# OpenCV + numpy only needed for the video path (JPEG -> bgr8). Import lazily so
# a sensors-only run still works on a box without OpenCV.
try:
    import cv2
    import numpy as np
    _HAVE_CV = True
except Exception:  # pragma: no cover
    _HAVE_CV = False

# Static TF lets rviz place the camera/imu/range frames in 3D. Guarded so the
# bridge still runs if tf2 isn't installed.
try:
    from tf2_ros import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    _HAVE_TF = True
except Exception:  # pragma: no cover
    _HAVE_TF = False


def euler_deg_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """Convert roll/pitch/yaw in DEGREES to a (x, y, z, w) quaternion."""
    r = math.radians(roll_deg) * 0.5
    p = math.radians(pitch_deg) * 0.5
    y = math.radians(yaw_deg) * 0.5
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class ZRosDriver(Node):
    def __init__(self):
        super().__init__("zros_driver")

        # ---- Parameters (override with --ros-args -p name:=value) ----
        self.robot_ip     = self.declare_parameter("robot_ip", "192.168.1.46").value
        # linear.x == 1.0 maps to this % throttle (-100..100 after clamping).
        self.linear_scale = float(self.declare_parameter("linear_scale", 100.0).value)
        # angular.z == 1.0 maps to this many degrees of steering offset (0..90).
        self.steer_scale  = float(self.declare_parameter("steer_scale", 90.0).value)
        # Pin for the optional /servo_angle aux servo (e.g. a camera pan servo).
        self.aux_servo_pin = int(self.declare_parameter("aux_servo_pin", 4).value)
        # Publish decoded raw video? (needs OpenCV). Turn off for sensors-only.
        self.publish_image = bool(self.declare_parameter("publish_image", True).value)
        # QQVGA/QVGA/VGA... Smaller = lighter over ROS. QVGA (320x240) is a good default.
        self.framesize     = str(self.declare_parameter("framesize", "QVGA").value)
        # If ROS stops sending /cmd_vel for this long, coast the throttle to 0.
        self.cmd_timeout   = float(self.declare_parameter("cmd_timeout", 0.6).value)
        # Motors need a minimum PWM to overcome stiction — below it they just
        # buzz. Any nonzero throttle is bumped up to at least this % (keeps sign).
        # tester 2026-11-04: "had to turn speed up before it moved". 0 = off.
        self.min_throttle  = int(self.declare_parameter("min_throttle", 0).value)
        # Steering centre trim (degrees added to 90) — nudge if the wheels sit
        # off-centre when going straight. tester: "not centered".
        self.steer_trim    = int(self.declare_parameter("steer_trim", 0).value)

        # ---- Room-scan (/scan_room service -> /scan LaserScan) ----
        # The VL53 rangefinder is geared to the STEERING servo: scan_gear deg of
        # beam per 1 deg of servo (36T->12T = 3.0). Sweep scan_servo_min..max,
        # read the straight-ahead distance at each step, map to a beam angle.
        # Flip scan_gear's SIGN (-3.0) if the published scan comes out mirrored.
        self.scan_servo_min = int(self.declare_parameter("scan_servo_min", 30).value)
        self.scan_servo_max = int(self.declare_parameter("scan_servo_max", 150).value)
        self.scan_center    = int(self.declare_parameter("scan_center", 90).value)
        self.scan_gear      = float(self.declare_parameter("scan_gear", 3.0).value)
        self.scan_step      = int(self.declare_parameter("scan_step", 1).value)
        self.scan_settle    = float(self.declare_parameter("scan_settle", 0.20).value)
        self.scan_frame     = str(self.declare_parameter("scan_frame", "laser").value)

        self.get_logger().info(f"Connecting to ZRCar at {self.robot_ip} ...")
        # auto_reconnect keeps the node alive across bot reboots / WiFi blips.
        self.bot = Robot(self.robot_ip, auto_reconnect=True)
        self.bot.connect()
        self.bot.wait(0.5)  # SDK rule: settle before the first command
        try:
            self.bot.camera.framesize(self.framesize)  # optional: pick a sane res
        except Exception as e:
            self.get_logger().warning(f"framesize({self.framesize}) failed: {e}")
        # Hold-to-drive: let the firmware's own watchdog stop the motor if our
        # keepalive stops arriving (safer than disabling auto-stop).
        self.bot.car.center()

        # ---- Publishers ----
        self.img_pub   = self.create_publisher(Image, "camera/image_raw", 10)
        self.imu_pub   = self.create_publisher(Imu, "imu/data", 10)
        self.range_pub = self.create_publisher(Range, "range", 10)
        self.scan_pub  = self.create_publisher(LaserScan, "scan", 10)

        # Room-scan runs on a worker thread so the sweep (~30 s) does not block
        # the node spin. _scanning gates /cmd_vel steering so teleop can't fight
        # the sweep while it's running.
        self._scanning = False
        self.create_service(Trigger, "scan_room", self.on_scan_room)

        # ---- Static TF (base_link -> camera/imu/tof) so rviz positions them ----
        if _HAVE_TF:
            self._tf = StaticTransformBroadcaster(self)
            self._publish_static_tf()

        # ---- Subscriptions ----
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Int32, "servo_angle", self.on_servo, 10)

        # ---- SDK callbacks (fire on the SDK's recv thread) ----
        self.bot.on_telemetry(self.on_telemetry)
        if self.publish_image and _HAVE_CV:
            self.bot.on_video(self.on_video)
        elif self.publish_image and not _HAVE_CV:
            self.get_logger().warning("publish_image set but OpenCV missing — video disabled")

        # ---- Drive keepalive (classic keepalive pattern) ----
        # /cmd_vel sources (teleop) often publish sparsely. Re-send the last
        # throttle at 10 Hz so the motor keeps running while a key is held, and
        # coast to 0 if ROS goes quiet (cmd_timeout) — so a dead publisher or a
        # crashed teleop stops the car instead of running it away.
        self._throttle = 0
        self._steer    = 90
        self._last_cmd = self.get_clock().now()
        self.create_timer(0.1, self._keepalive)

        self.get_logger().info("ZRos bridge up: /cmd_vel /servo_angle -> bot ; "
                               "video+imu+range -> ROS. (Ctrl-C to stop)")

    # ---------- OUT: ROS -> bot ----------
    def on_cmd_vel(self, msg: Twist):
        # Throttle from forward/back speed; clamp to the bot's -100..100 range.
        throttle = int(max(-100.0, min(100.0, msg.linear.x * self.linear_scale)))
        # Bump a small nonzero throttle past the motor's stiction threshold.
        if throttle != 0 and self.min_throttle > 0 and abs(throttle) < self.min_throttle:
            throttle = self.min_throttle if throttle > 0 else -self.min_throttle
        self._throttle = throttle
        # Steering from yaw rate. REP-103: +angular.z = turn LEFT. The car's
        # servo is 0..180 with 90 = straight and <90 = left, so subtract.
        # steer_trim nudges the centre if the wheels sit off-centre.
        angle = 90.0 + self.steer_trim - (msg.angular.z * self.steer_scale)
        steer = int(max(0.0, min(180.0, angle)))
        self._last_cmd = self.get_clock().now()
        # While a room scan is running the steering servo IS the scanner — don't
        # let teleop move it (and keep the car still: throttle is forced 0 below).
        if steer != self._steer and not self._scanning:
            self._steer = steer
            try:
                self.bot.car.set_steering(steer)
            except Exception as e:
                self.get_logger().warning(f"set_steering failed: {e}")

    def _keepalive(self):
        # Coast to 0 if no fresh /cmd_vel within cmd_timeout. Also hold at 0
        # while a room scan is in progress so the car stays put.
        age = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        throttle = 0 if self._scanning else (self._throttle if age < self.cmd_timeout else 0)
        try:
            self.bot.car.raw(throttle)
        except Exception as e:
            self.get_logger().warning(f"car.raw failed: {e}")

    def on_servo(self, msg: Int32):
        angle = max(0, min(180, int(msg.data)))
        try:
            self.bot.servo.set_angle(pin=self.aux_servo_pin, angle=angle)
        except Exception as e:
            self.get_logger().warning(f"servo.set_angle failed: {e}")

    # ---------- IN: bot -> ROS ----------
    def on_video(self, jpeg_bytes, latency_ms):
        try:
            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                return
            m = Image()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "camera_link"
            m.height = img.shape[0]
            m.width = img.shape[1]
            m.encoding = "bgr8"
            m.is_bigendian = False
            m.step = img.shape[1] * 3
            m.data = img.tobytes()
            self.img_pub.publish(m)
        except Exception as e:
            self.get_logger().error(f"video frame failed: {e}")

    def on_telemetry(self, data: dict):
        now = self.get_clock().now().to_msg()

        # MPU6050 — short keys: iax/iay/iaz accel, ih heading, ipt pitch, irl roll.
        if "iax" in data:
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = "imu_link"
            imu.linear_acceleration.x = float(data.get("iax", 0.0))
            imu.linear_acceleration.y = float(data.get("iay", 0.0))
            imu.linear_acceleration.z = float(data.get("iaz", 0.0))
            if "ih" in data or "ipt" in data or "irl" in data:
                qx, qy, qz, qw = euler_deg_to_quaternion(
                    float(data.get("irl", 0.0)),   # roll
                    float(data.get("ipt", 0.0)),   # pitch
                    float(data.get("ih", 0.0)),    # yaw / heading
                )
                imu.orientation.x = qx
                imu.orientation.y = qy
                imu.orientation.z = qz
                imu.orientation.w = qw
            else:
                imu.orientation.w = 1.0
            self.imu_pub.publish(imu)

        # ToF rangefinder — short key 'd' = straight-ahead distance in mm
        # (centre-2x2 min of the VL53 grid; 'dmin' would be nearest-anywhere).
        if "d" in data and data.get("d") is not None:
            rng = Range()
            rng.header.stamp = now
            rng.header.frame_id = "tof_link"
            rng.radiation_type = Range.INFRARED
            rng.field_of_view = 0.785   # VL53L5CX ~45 deg total FoV
            rng.min_range = 0.02
            rng.max_range = 4.0
            rng.range = float(data.get("d")) / 1000.0  # mm -> m
            self.range_pub.publish(rng)

    # ---------- Room scan: ROS trigger -> servo sweep -> /scan LaserScan ----------
    def on_scan_room(self, request, response):
        """Trigger a room scan. The sweep runs on a worker thread, so this
        returns immediately — watch /scan for the result."""
        if self._scanning:
            response.success = False
            response.message = "scan already in progress"
            return response
        self._scanning = True
        threading.Thread(target=self._do_scan, daemon=True).start()
        response.success = True
        response.message = "room scan started — watch /scan"
        return response

    def _do_scan(self):
        """Sweep the steering servo across its range, read the straight-ahead
        distance at each step, and publish one LaserScan on /scan. scan_gear
        maps servo angle -> beam angle; beam 0 rad = straight ahead."""
        try:
            lo = min(self.scan_servo_min, self.scan_servo_max)
            hi = max(self.scan_servo_min, self.scan_servo_max)
            step = max(1, self.scan_step)
            sweep = list(range(lo, hi + 1, step))
            self.get_logger().info(
                f"Room scan: servo {lo}..{hi} step {step} "
                f"({len(sweep)} beams, ~{len(sweep) * (self.scan_settle + 0.05):.0f}s)")

            self.bot.car.set_steering(self.scan_center)
            time.sleep(0.4)

            beams = []  # (beam_rad, range_m)
            t0 = time.time()
            for servo_angle in sweep:
                self.bot.car.set_steering(servo_angle)
                time.sleep(self.scan_settle)  # servo move + a fresh ranging frame
                reply = self.bot.wait_for(lambda m: "distanceMm" in m,
                                          ask=self.bot.tof.get_distance, timeout=0.5)
                d_mm = reply.get("distanceMm", -1) if reply else -1
                beam_rad = math.radians((servo_angle - self.scan_center) * self.scan_gear)
                if d_mm is None or d_mm < 40 or d_mm > 4000:
                    rng_m = float("inf")  # invalid / out of VL53 range -> ignored by rviz
                else:
                    rng_m = d_mm / 1000.0
                beams.append((beam_rad, rng_m))
            scan_time = time.time() - t0

            self.bot.car.center()  # steering back to straight

            if len(beams) < 2:
                self.get_logger().warning("scan produced too few beams")
                return
            # LaserScan wants ranges ordered by increasing angle with a uniform
            # increment. A negative gear sweeps the beam the other way, so flip.
            if beams[-1][0] < beams[0][0]:
                beams.reverse()

            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = self.scan_frame
            scan.angle_min = beams[0][0]
            scan.angle_increment = math.radians(abs(self.scan_gear) * step)
            scan.angle_max = scan.angle_min + scan.angle_increment * (len(beams) - 1)
            scan.range_min = 0.04
            scan.range_max = 4.0
            scan.scan_time = float(scan_time)
            scan.time_increment = float(scan_time / max(1, len(beams)))
            scan.ranges = [r for (_, r) in beams]
            self.scan_pub.publish(scan)
            valid = sum(1 for r in scan.ranges if math.isfinite(r))
            self.get_logger().info(
                f"Published /scan: {len(scan.ranges)} beams "
                f"({math.degrees(scan.angle_min):.0f}..{math.degrees(scan.angle_max):.0f} deg), "
                f"{valid} valid, in {scan_time:.1f}s")
        except Exception as e:
            self.get_logger().error(f"room scan failed: {e}")
        finally:
            self._scanning = False

    def _publish_static_tf(self):
        """Publish fixed base_link -> camera/imu/tof transforms (identity
        rotation = each sensor faces +x/forward). Lets rviz show the Range cone
        and position the camera/imu frames. Positions are rough — tweak to match
        your chassis."""
        def tf(child, x, y, z):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            t.child_frame_id = child
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = float(z)
            t.transform.rotation.w = 1.0
            return t
        self._tf.sendTransform([
            tf("camera_link", 0.10, 0.0, 0.06),
            tf("imu_link",    0.0,  0.0, 0.02),
            tf("tof_link",    0.12, 0.0, 0.04),
            tf(self.scan_frame, 0.12, 0.0, 0.04),  # LaserScan pivot (rangefinder)
        ])

    def destroy_node(self):
        try:
            self.bot.car.stop()
        except Exception:
            pass
        try:
            self.bot.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZRosDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
