#!/usr/bin/env python3
"""
obstacle_stop.py — tiny autonomous behaviour demo
==================================================

A pure ROS 2 node (it never touches the bot directly — only the bridge does):
it SUBSCRIBES to /range and PUBLISHES /cmd_vel. That's the whole point of the
demo — show how, in ROS, a behaviour is just a node wired between a sensor
topic and a command topic. Swap this node for a fancier planner later and the
bridge/teleop don't change.

Behaviour: cruise forward while the path is clear; brake when the wall gets
close. By DEFAULT cruise_speed is 0.0 (brake-only / watch mode) so running it
can't drive your car off the table. Opt into the full auto-cruise with:

    ... obstacle_stop.py --ros-args -p cruise_speed:=0.3

Params:
    stop_distance (m, default 0.30)  brake when /range is nearer than this
    cruise_speed  (0..1, default 0.0)  forward linear.x when clear (0 = watch only)

!!!  SAFETY: put the car up on blocks (wheels off the ground) the first time,
     or run it on a wide open floor. It WILL drive forward when cruise_speed>0.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class ObstacleStop(Node):
    def __init__(self):
        super().__init__("obstacle_stop")
        self.stop_distance = float(self.declare_parameter("stop_distance", 0.30).value)
        self.cruise_speed  = float(self.declare_parameter("cruise_speed", 0.0).value)

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Range, "range", self.on_range, 10)
        self._clear = False

        mode = "WATCH-ONLY (cruise_speed=0)" if self.cruise_speed <= 0 else \
               f"AUTO-CRUISE at {self.cruise_speed:.2f}"
        self.get_logger().info(
            f"obstacle_stop up: brake under {self.stop_distance:.2f} m — {mode}. "
            f"Publishing /cmd_vel from /range. (Ctrl-C to stop)")

    def on_range(self, msg: Range):
        d = msg.range
        twist = Twist()
        if 0.0 < d < self.stop_distance:
            # Too close — stop. (twist defaults to all-zero = stop.)
            if self._clear:
                self.get_logger().warn(f"obstacle at {d:.2f} m — STOP")
            self._clear = False
        else:
            # Clear — cruise forward (or nothing, in watch-only mode).
            if not self._clear:
                self.get_logger().info(f"clear ({d:.2f} m) — go")
            self._clear = True
            twist.linear.x = self.cruise_speed
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the car stopped.
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
