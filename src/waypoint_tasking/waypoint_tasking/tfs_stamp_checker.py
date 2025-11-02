#!/usr/bin/env python3
"""
tf_stamp_checker_specific.py

ROS2 Humble node that watches /tf and reports timing (age) for a single transform
specified by parent and child frame names.

Features:
 - Accepts ROS parameters `parent` and `child` (defaults: map, odom).
 - Prints concise, human-readable timing lines showing:
    * node `now()` (sim seconds) and ISO epoch conversion (UTC)
    * transform stamp (sim seconds) and ISO epoch conversion (UTC)
    * age = now - stamp in seconds (positive = transform in the past)
 - Uses `self.get_clock().now()` (so it works for both sim and wall time). It
   heuristically labels the output as SIM-TIME vs WALL-TIME by comparing to
   system time.
 - Only logs the age for the specified transform; if the transform is not
   present in a /tf message the node logs a short informative line.

Usage examples:
  ros2 run waypoint_tasking tf_stamp_checker_specific
  ros2 run waypoint_tasking tf_stamp_checker_specific --ros-args --param parent:=map --param child:=odom

Output example (concise):
  [TF-AGE] map->odom | mode=SIM | now=1668.408000000s (2025-09-28T15:51:20.855000000Z) |
            stamp=1667.996000000s (1970-01-01T00:27:47.996000000Z) | age=0.412000s

"""

import time
from datetime import datetime, timezone
from typing import Optional

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def human_time_pair(sec: int, nanosec: int) -> str:
    """Return a compact representation: "{sim_seconds}s ({ISO UTC})".

    If the epoch conversion fails, the ISO part will be replaced with "invalid-iso".
    """
    sim_s = float(sec) + float(nanosec) * 1e-9
    try:
        dt = datetime.fromtimestamp(sim_s, tz=timezone.utc)
        iso = dt.strftime('%Y-%m-%dT%H:%M:%S') + f'.{nanosec:09d}Z'
    except Exception:
        iso = 'invalid-iso'
    return f"{sim_s:.9f}s ({iso})"


class TFChecker(Node):
    def __init__(self):
        super().__init__('tf_stamp_checker_specific')

        # Parameters
        self.declare_parameter('parent', 'map')
        self.declare_parameter('child', 'odom')
        self.declare_parameter('log_every_n', 1)

        self.parent = self.get_parameter('parent').get_parameter_value().string_value
        self.child = self.get_parameter('child').get_parameter_value().string_value
        self.log_every_n = int(self.get_parameter('log_every_n').get_parameter_value().integer_value)

        # Subscribe to /tf topic
        self.sub = self.create_subscription(TFMessage, '/tf', self.tf_cb, 10)

        self.counter = 0
        self.get_logger().info(f"tf_stamp_checker_specific started. Watching: {self.parent} -> {self.child}")

    def _is_sim_mode(self, now_s: float) -> bool:
        """Heuristic to label sim vs wall time.

        If the node clock (`now_s`) differs from system wall time by more than
        ~1e6 seconds (~11.5 days) we assume sim-time. This is a safe heuristic
        for typical Gazebo simulations (sim seconds are small).
        """
        try:
            wall = time.time()
            return abs(now_s - wall) > 1e6
        except Exception:
            return False

    def tf_cb(self, msg: TFMessage):
        self.counter += 1
        if (self.counter % self.log_every_n) != 0:
            return

        # authoritative node time (respects use_sim_time if configured)
        now_rcl = self.get_clock().now()
        now_sec, now_nsec = now_rcl.seconds_nanoseconds()
        now_s = float(now_sec) + float(now_nsec) * 1e-9

        sim_mode = self._is_sim_mode(now_s)
        mode_str = 'SIM' if sim_mode else 'WALL'

        found = False
        for t in msg.transforms:
            if t.header.frame_id == self.parent and t.child_frame_id == self.child:
                found = True
                sec = t.header.stamp.sec
                nsec = t.header.stamp.nanosec
                stamp_s = float(sec) + float(nsec) * 1e-9
                age = now_s - stamp_s

                now_h = human_time_pair(now_sec, now_nsec)
                stamp_h = human_time_pair(sec, nsec)

                # concise, single-line summary
                summary = (
                    f"[TF-AGE] {self.parent}->{self.child} | mode={mode_str} | "
                    f"now={now_h} | stamp={stamp_h} | age={age:.6f}s"
                )

                # Log: info for normal, warn if age magnitude is large
                if abs(age) > 0.5:
                    # >0.5s is large for many robotics uses — warn the user
                    self.get_logger().warn(summary + '  [WARNING: large age]')
                else:
                    self.get_logger().info(summary)

        if not found:
            self.get_logger().info(f'No transform {self.parent}->{self.child} in /tf message')


def main(args=None):
    rclpy.init(args=args)
    node = TFChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
