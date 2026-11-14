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
                   ->  TF odom -> base_link (heading/pitch/roll: the car model
                                            turns and tilts in rviz)
  ToF distance     ->  /range             (sensor_msgs/Range)
  swept-ToF scan   ->  /scan              (sensor_msgs/LaserScan, last sweep)
                   ->  /scan_cloud        (sensor_msgs/PointCloud2, last sweep, 3D)
                   ->  /map_cloud         (sensor_msgs/PointCloud2, all sweeps)
                   ->  /map               (nav_msgs/OccupancyGrid floor map)
  car outline      ->  /robot_marker      (visualization_msgs/MarkerArray)

  ROS service      ->  action
  --------------------------------------------------------------
  /scan_room  (std_srvs/Trigger)  sweep the steering servo once and publish
                                  /scan + /scan_cloud, and add to /map_cloud
                                  and /map. The rangefinder is geared 3:1 to
                                  the steering servo, so a ±60° servo sweep
                                  traces a full 360° room cross-section.
  /scan_loop  (std_srvs/SetBool)  true = keep sweeping, false = stop after the
                                  current sweep.
  /clear_map  (std_srvs/Trigger)  forget all sweeps (map + clouds) and zero the
                                  heading, so a new map starts facing forward.

Run:
    python3 zros_driver.py --ros-args -p robot_ip:=192.168.1.46

Notes / gotchas (from the zrsense SDK — ZROS/PythonAPI):
  * websocket-client MUST be 1.6.4 (newer versions reject the ESP32's frames).
  * ONE WebSocket client at a time — close the browser /car tab before running.
  * The bot's camera must be enabled for video, and the rangefinder/IMU enabled
    for /range, /imu and the scan (bot main page -> Info -> Components).
  * This is a CAR: throttle and steering are driven together (Ackermann-style),
    not tank-style. angular.z steers the front wheels; it does NOT spin in place.
  * The steering servo IS the scanner: the car can't steer while it sweeps, and
    with no wheel encoders the bridge can't tell how far it drove — so the map
    stays lined up when the car turns in place, not when it drives somewhere new.
"""
import array
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu, Range, LaserScan, PointCloud2, PointField
from std_msgs.msg import Int32
from std_srvs.srv import Trigger, SetBool

# zrsense.py lives one directory up (ZROS/PythonAPI/zrsense.py). run.sh puts it
# on PYTHONPATH; if you run this by hand, add that dir to PYTHONPATH first.
from zrsense import Robot

import scan_geometry as sg   # pure-Python scan maths, next to this file

# Floor map + car outline. Part of ROS common_interfaces (in the image); guarded
# so the bridge still runs without them.
try:
    from nav_msgs.msg import OccupancyGrid
    _HAVE_MAP = True
except Exception:  # pragma: no cover
    _HAVE_MAP = False
try:
    from visualization_msgs.msg import Marker, MarkerArray
    _HAVE_MARKER = True
except Exception:  # pragma: no cover
    _HAVE_MARKER = False

# OpenCV + numpy only needed for the video path (JPEG -> bgr8). Import lazily so
# a sensors-only run still works on a box without OpenCV.
try:
    import cv2
    import numpy as np
    _HAVE_CV = True
except Exception:  # pragma: no cover
    _HAVE_CV = False

# TF lets rviz place the car and its sensors in 3D. Guarded so the bridge still
# runs if tf2 isn't installed.
try:
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    _HAVE_TF = True
except Exception:  # pragma: no cover
    _HAVE_TF = False

# Where the rangefinder's pivot sits on base_link (metres) — keep in sync with
# the static TF "laser" frame below. Rough; tweak to match your chassis.
LASER_XYZ = (0.12, 0.0, 0.04)
MAX_CLOUD_POINTS = 80000        # /map_cloud keeps the newest points up to this


def euler_to_quaternion(roll, pitch, yaw):
    """ROS roll/pitch/yaw in RADIANS -> (x, y, z, w) quaternion."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
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
        # Camera mounted upside down? Set both to 1 (it's done on the sensor
        # itself, so it costs nothing). -1 = leave the bot's setting alone.
        self.camera_vflip   = int(self.declare_parameter("camera_vflip", -1).value)
        self.camera_hmirror = int(self.declare_parameter("camera_hmirror", -1).value)
        # If ROS stops sending /cmd_vel for this long, coast the throttle to 0.
        self.cmd_timeout   = float(self.declare_parameter("cmd_timeout", 0.6).value)
        # Motors need a minimum PWM to overcome stiction — below it they just
        # buzz. Any nonzero throttle is bumped up to at least this % (keeps sign).
        # tester 2026-11-04: "had to turn speed up before it moved". 0 = off.
        self.min_throttle  = int(self.declare_parameter("min_throttle", 0).value)
        # Steering centre trim (degrees added to 90) — nudge if the wheels sit
        # off-centre when going straight. tester: "not centered".
        self.steer_trim    = int(self.declare_parameter("steer_trim", 0).value)

        # ---- Room scan (/scan_room, /scan_loop, /clear_map) ----
        # The VL53 rangefinder is geared to the STEERING servo: scan_gear deg of
        # beam per 1 deg of servo (36T->12T = 3.0). Each servo step reads the
        # rangefinder's whole zone grid; every zone becomes one ray.
        self.scan_servo_min = int(self.declare_parameter("scan_servo_min", 30).value)
        self.scan_servo_max = int(self.declare_parameter("scan_servo_max", 150).value)
        self.scan_center    = int(self.declare_parameter("scan_center", 90).value)
        self.scan_gear      = float(self.declare_parameter("scan_gear", 3.0).value)
        # Calibration — which way the rangefinder points when the steering is at
        # scan_center (+ = left). Depends on how the gears were meshed. Put a
        # wall straight ahead, scan: wall shows up on the LEFT -> -90, on the
        # RIGHT -> 90, behind -> 180. Map mirrored -> flip scan_gear's sign.
        self.scan_offset_deg = float(self.declare_parameter("scan_offset_deg", 0.0).value)
        # Mirror the zone columns if walls look smeared/doubled even after the
        # offset is right (sensor heatmap left/right swapped).
        self.scan_zone_flip = bool(self.declare_parameter("scan_zone_flip", False).value)
        self.scan_fov_deg   = float(self.declare_parameter("scan_fov_deg", 45.0).value)
        self.scan_step      = int(self.declare_parameter("scan_step", 1).value)
        # Wait after each servo step so a FRESH ranging frame (5 Hz) lands.
        self.scan_settle    = float(self.declare_parameter("scan_settle", 0.25).value)
        # Ignore returns closer than this — the car's own chassis (it ranges
        # itself when the beam swings backwards) — and further than the sensor
        # can really see.
        self.scan_min_range = float(self.declare_parameter("scan_min_range", 0.12).value)
        self.scan_max_range = float(self.declare_parameter("scan_max_range", 3.5).value)
        self.scan_bin_deg   = float(self.declare_parameter("scan_bin_deg", 1.0).value)
        self.scan_frame     = str(self.declare_parameter("scan_frame", "laser").value)
        self.scan_loop_pause = float(self.declare_parameter("scan_loop_pause", 1.0).value)
        self.map_resolution = float(self.declare_parameter("map_resolution", 0.05).value)
        self.map_size       = float(self.declare_parameter("map_size", 8.0).value)
        self.clear_resets_heading = bool(self.declare_parameter("clear_resets_heading", True).value)

        self.get_logger().info(f"Connecting to ZRCar at {self.robot_ip} ...")
        # auto_reconnect keeps the node alive across bot reboots / WiFi blips.
        self.bot = Robot(self.robot_ip, auto_reconnect=True)
        self.bot.connect()
        self.bot.wait(0.5)  # SDK rule: settle before the first command
        try:
            self.bot.camera.framesize(self.framesize)  # optional: pick a sane res
        except Exception as e:
            self.get_logger().warning(f"framesize({self.framesize}) failed: {e}")
        if self.camera_vflip >= 0 or self.camera_hmirror >= 0:
            try:
                self.bot.camera.flip(
                    h=bool(self.camera_hmirror) if self.camera_hmirror >= 0 else None,
                    v=bool(self.camera_vflip) if self.camera_vflip >= 0 else None)
            except Exception as e:
                self.get_logger().warning(f"camera flip failed: {e}")
        # Hold-to-drive: let the firmware's own watchdog stop the motor if our
        # keepalive stops arriving (safer than disabling auto-stop).
        self.bot.car.center()

        # ---- Publishers ----
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.img_pub   = self.create_publisher(Image, "camera/image_raw", 10)
        self.imu_pub   = self.create_publisher(Imu, "imu/data", 10)
        self.range_pub = self.create_publisher(Range, "range", 10)
        self.scan_pub  = self.create_publisher(LaserScan, "scan", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "scan_cloud", 10)
        self.map_cloud_pub = self.create_publisher(PointCloud2, "map_cloud", latched)
        self.map_pub   = self.create_publisher(OccupancyGrid, "map", latched) if _HAVE_MAP else None
        self.marker_pub = (self.create_publisher(MarkerArray, "robot_marker", 10)
                           if _HAVE_MARKER else None)

        # ---- Map state (built up by sweeps, emptied by /clear_map) ----
        self._map_lock = threading.Lock()
        self._map_points = []                                  # [(x, y, z)] in odom
        self._grid = sg.OccupancyGrid2D(self.map_resolution, self.map_size)
        self._zone_angle_cache = {}

        # Room-scan runs on a worker thread so a sweep (~30 s) doesn't block the
        # node. _scanning gates /cmd_vel: while the steering servo is the
        # scanner, teleop can't steer and the throttle is held at 0.
        self._scanning = False
        self._scan_loop = False
        self.create_service(Trigger, "scan_room", self.on_scan_room)
        self.create_service(SetBool, "scan_loop", self.on_scan_loop)
        self.create_service(Trigger, "clear_map", self.on_clear_map)

        # ---- TF: static sensor frames + odom -> base_link from the IMU ----
        # Latest orientation in ROS convention (radians): the firmware reports
        # pitch NOSE-UP positive, ROS (REP-103) is nose-DOWN positive -> negate.
        self._rpy = (0.0, 0.0, 0.0)
        self._tf_dyn = None
        if _HAVE_TF:
            self._tf = StaticTransformBroadcaster(self)
            self._publish_static_tf()
            self._tf_dyn = TransformBroadcaster(self)
            self.create_timer(0.1, self._publish_odom_tf)   # keep TF fresh even without IMU
        if self.marker_pub is not None:
            self.create_timer(1.0, self._publish_robot_marker)
        self._publish_map()   # an empty map/cloud so rviz has something to show at once

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
                               "video+imu+range+scan -> ROS. (Ctrl-C to stop)")

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
                # Heading: firmware integrates gz (counter-clockwise +, like ROS
                # yaw). Pitch: firmware nose-up +, ROS nose-down + -> negate.
                roll  = math.radians(float(data.get("irl", 0.0)))
                pitch = -math.radians(float(data.get("ipt", 0.0)))
                yaw   = math.radians(float(data.get("ih", 0.0)))
                self._rpy = (roll, pitch, yaw)
                qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
                imu.orientation.x = qx
                imu.orientation.y = qy
                imu.orientation.z = qz
                imu.orientation.w = qw
                self._publish_odom_tf()
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

    # ---------- Room scan: services ----------
    def on_scan_room(self, request, response):
        """Run ONE sweep on a worker thread; returns at once — watch /scan + /map."""
        if self._scanning:
            response.success = False
            response.message = "a scan is already running"
            return response
        self._start_scan(loop=False)
        response.success = True
        response.message = "room scan started — watch /scan, /map_cloud and /map"
        return response

    def on_scan_loop(self, request, response):
        """data=true: keep sweeping until data=false (stops after the current sweep)."""
        if request.data:
            self._scan_loop = True
            if not self._scanning:
                self._start_scan(loop=True)
            response.success = True
            response.message = "continuous scanning ON — ./run.sh scan stop to end"
        else:
            self._scan_loop = False
            response.success = True
            response.message = ("stopping after the current sweep" if self._scanning
                                else "continuous scanning was not running")
        return response

    def on_clear_map(self, request, response):
        """Forget every sweep: empty /map, /map_cloud, /scan_cloud. Also zeroes
        the heading (clear_resets_heading) so the new map starts facing forward."""
        with self._map_lock:
            self._map_points = []
            self._grid.clear()
        if self.clear_resets_heading:
            try:
                self.bot.imu.reset_heading()
            except Exception as e:
                self.get_logger().warning(f"reset_heading failed: {e}")
        self._publish_map()
        self.cloud_pub.publish(self._make_cloud([]))
        response.success = True
        response.message = "map cleared" + (" and heading zeroed" if self.clear_resets_heading else "")
        return response

    def _start_scan(self, loop):
        self._scanning = True
        self._scan_loop = loop
        threading.Thread(target=self._scan_worker, daemon=True).start()

    def _scan_worker(self):
        try:
            while True:
                self._sweep_once()
                if not self._scan_loop:
                    break
                time.sleep(max(0.0, self.scan_loop_pause))
        except Exception as e:
            self.get_logger().error(f"room scan failed: {e}")
        finally:
            try:
                self.bot.car.center()      # steering back to straight
            except Exception:
                pass
            self._scan_loop = False
            self._scanning = False

    # ---------- Room scan: one sweep ----------
    def _zone_angles(self, side):
        key = (side, self.scan_fov_deg, self.scan_zone_flip)
        if key not in self._zone_angle_cache:
            self._zone_angle_cache[key] = sg.zone_angles(
                side, self.scan_fov_deg, self.scan_fov_deg, self.scan_zone_flip)
        return self._zone_angle_cache[key]

    def _read_grid(self):
        """One rangefinder frame as (side, [distance_mm per zone]) in the bot's
        DISPLAY frame, or None if the grid isn't available."""
        reply = self.bot.wait_for(lambda m: "zones" in m,
                                  ask=self.bot.tof.get_grid, timeout=0.8)
        if not reply:
            return None
        zones = reply.get("zones") or []
        n = int(reply.get("zoneCount") or len(zones))
        side = sg.grid_side(n)
        if side is None:
            return None
        dist = [-1] * n
        for z in zones:
            try:
                i = int(z.get("i", -1))
                if 0 <= i < n:
                    dist[i] = int(z.get("distanceMm", -1))
            except (TypeError, ValueError):
                continue
        return side, dist

    def _read_center(self):
        """Fallback when the grid isn't available: one straight-ahead reading."""
        reply = self.bot.wait_for(lambda m: "distanceMm" in m,
                                  ask=self.bot.tof.get_distance, timeout=0.6)
        d = reply.get("distanceMm", -1) if reply else -1
        return 1, [d if isinstance(d, (int, float)) else -1]

    def _sweep_once(self):
        """Sweep the steering servo, read every zone at every step, then publish
        /scan + /scan_cloud and add the sweep to /map_cloud and /map."""
        lo = min(self.scan_servo_min, self.scan_servo_max)
        hi = max(self.scan_servo_min, self.scan_servo_max)
        step = max(1, self.scan_step)
        sweep = list(range(lo, hi + 1, step))
        self.get_logger().info(
            f"Room scan: servo {lo}..{hi} step {step} ({len(sweep)} steps, "
            f"~{len(sweep) * (self.scan_settle + 0.08):.0f}s), offset {self.scan_offset_deg:+.0f} deg")

        self.bot.car.set_steering(lo)        # go to the start, let it settle fully
        time.sleep(0.6)

        flat_rays = []      # (bearing_rad, horizontal_range_m) for /scan
        floor_rays = []     # (x0, y0, x1, y1) in odom for /map
        points = []         # (x, y, z) in odom for the clouds
        grid_ok = 0
        t0 = time.time()
        for servo_angle in sweep:
            self.bot.car.set_steering(servo_angle)
            time.sleep(self.scan_settle)       # servo move + a fresh ranging frame
            frame = self._read_grid() or self._read_center()
            side, dist = frame
            if side > 1:
                grid_ok += 1
                angles = self._zone_angles(side)
                flat_rows = sg.horizontal_rows(side)
            else:
                angles, flat_rows = [(0.0, 0.0)], {0}
            beam = math.radians((servo_angle - self.scan_center) * self.scan_gear
                                + self.scan_offset_deg)
            roll, pitch, yaw = self._rpy          # car orientation right now
            ox, oy, oz = sg.rotate_rpy(*LASER_XYZ, roll, pitch, yaw)
            for i, d_mm in enumerate(dist):
                if d_mm is None or d_mm <= 0:
                    continue
                r = d_mm / 1000.0
                if r < self.scan_min_range or r > self.scan_max_range:
                    continue
                az, el = angles[i]
                bearing = beam + az
                horiz = r * math.cos(el)
                bx = LASER_XYZ[0] + horiz * math.cos(bearing)
                by = LASER_XYZ[1] + horiz * math.sin(bearing)
                bz = LASER_XYZ[2] + r * math.sin(el)
                px, py, pz = sg.rotate_rpy(bx, by, bz, roll, pitch, yaw)
                points.append((px, py, pz))
                if (i // side if side > 1 else 0) in flat_rows:
                    flat_rays.append((bearing, horiz))
                    floor_rays.append((ox, oy, px, py))
        scan_time = time.time() - t0
        self.bot.car.set_steering(self.scan_center)

        if grid_ok == 0:
            self.get_logger().warning(
                "rangefinder grid not available — used the single centre distance "
                "(older firmware?). Walls will look curved.")

        # /scan — the last sweep as a 2D LaserScan, in the laser frame.
        n_bins = max(36, int(round(360.0 / max(0.25, self.scan_bin_deg))))
        a_min, inc, ranges = sg.bin_scan(flat_rays, n_bins, self.scan_min_range, self.scan_max_range)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.scan_frame
        scan.angle_min = a_min
        scan.angle_increment = inc
        scan.angle_max = a_min + inc * (n_bins - 1)
        scan.range_min = self.scan_min_range
        scan.range_max = self.scan_max_range
        scan.scan_time = float(scan_time)
        scan.time_increment = float(scan_time / max(1, n_bins))
        scan.ranges = ranges
        self.scan_pub.publish(scan)

        # /scan_cloud — this sweep in 3D; /map_cloud + /map — all sweeps so far.
        self.cloud_pub.publish(self._make_cloud(points))
        with self._map_lock:
            self._map_points.extend(points)
            if len(self._map_points) > MAX_CLOUD_POINTS:
                self._map_points = self._map_points[-MAX_CLOUD_POINTS:]
            for x0, y0, x1, y1 in floor_rays:
                self._grid.add_ray(x0, y0, x1, y1, hit=True)
        self._publish_map()

        valid = sum(1 for r in ranges if math.isfinite(r))
        self.get_logger().info(
            f"Published sweep: {len(points)} points ({len(flat_rays)} on the floor plane), "
            f"{valid}/{n_bins} scan bins, {grid_ok}/{len(sweep)} grid frames, {scan_time:.1f}s")

    # ---------- Room scan: messages ----------
    def _make_cloud(self, points):
        """PointCloud2 (x, y, z float32) in the odom frame."""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.height = 1
        msg.width = len(points)
        msg.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                      for k, n in enumerate(("x", "y", "z"))]
        msg.is_bigendian = sys.byteorder != "little"
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        buf = array.array("f")
        for p in points:
            buf.extend(p)
        msg.data = buf.tobytes()
        return msg

    def _publish_map(self):
        with self._map_lock:
            pts = list(self._map_points)
            cells = self._grid.values()
            n, res, origin = self._grid.n, self._grid.res, self._grid.origin
        self.map_cloud_pub.publish(self._make_cloud(pts))
        if self.map_pub is None:
            return
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "odom"
        g.info.resolution = float(res)
        g.info.width = n
        g.info.height = n
        g.info.origin.position.x = float(origin)
        g.info.origin.position.y = float(origin)
        g.info.origin.orientation.w = 1.0
        g.data = array.array("b", cells)
        self.map_pub.publish(g)

    # ---------- TF + car outline ----------
    def _publish_odom_tf(self):
        if self._tf_dyn is None:
            return
        roll, pitch, yaw = self._rpy
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        # No wheel encoders -> no position, only orientation (turn + tilt).
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_dyn.sendTransform(t)

    def _publish_robot_marker(self):
        """A simple car (body + 4 wheels + a nose arrow) on base_link, so the
        car is visible in rviz and turns/tilts with the IMU. Swap for a proper
        URDF + robot_state_publisher when one exists."""
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def mk(mid, mtype, pos, scale, rgba, quat=(0.0, 0.0, 0.0, 1.0)):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = stamp
            m.ns = "zrcar"
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.frame_locked = True
            m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = quat
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            return m

        arr.markers.append(mk(0, Marker.CUBE, (0.0, 0.0, 0.055), (0.20, 0.11, 0.05),
                              (0.36, 0.55, 0.42, 0.9)))
        wheel_q = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))   # axle along y
        for k, (wx, wy) in enumerate(((0.07, 0.065), (0.07, -0.065), (-0.07, 0.065), (-0.07, -0.065))):
            arr.markers.append(mk(1 + k, Marker.CYLINDER, (wx, wy, 0.03), (0.06, 0.06, 0.025),
                                  (0.1, 0.1, 0.1, 1.0), wheel_q))
        arr.markers.append(mk(5, Marker.ARROW, (0.0, 0.0, 0.09), (0.16, 0.015, 0.03),
                              (0.0, 0.85, 1.0, 1.0)))
        self.marker_pub.publish(arr)

    def _publish_static_tf(self):
        """Publish fixed base_link -> camera/imu/tof/laser transforms (identity
        rotation = each sensor faces +x/forward). Lets rviz show the Range cone
        and position the camera/imu frames. Positions are rough — tweak to match
        your chassis (and LASER_XYZ above)."""
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
            tf("tof_link",    *LASER_XYZ),
            tf(self.scan_frame, *LASER_XYZ),   # LaserScan pivot (rangefinder)
        ])

    def destroy_node(self):
        self._scan_loop = False
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
