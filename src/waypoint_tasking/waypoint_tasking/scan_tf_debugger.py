#!/usr/bin/env python3
"""
scan_tf_debugger.py

ROS2 Humble node to help debug scan vs TF timestamp / transform mismatches.

This variant logs human-readable timestamps (ISO 8601, UTC) for scan stamps,
TF stamps and `now`, while still printing numeric ages and deltas useful for
diagnosis.

Usage: same as the original file in the canvas.
"""

import math
import sys
import traceback
from typing import Optional
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
import tf2_ros


def time_msg_to_seconds(t):
    return float(t.sec) + float(t.nanosec) * 1e-9


def rclpy_time_to_seconds(t: Time) -> float:
    return float(t.seconds_nanoseconds()[0]) + float(t.seconds_nanoseconds()[1]) * 1e-9


def stamp_to_iso(sec, nanosec):
    # Produce an ISO8601 UTC timestamp with full nanosecond precision.
    # Example: 2025-09-28T12:34:56.123456789Z
    dt = datetime.fromtimestamp(float(sec) + float(nanosec) * 1e-9, tz=timezone.utc)
    # dt.strftime doesn't include nanoseconds, so format manually
    base = f"{dt.strftime('%Y-%m-%dT%H:%M:%S')}.{dt.microsecond // 1000:03d}"
    return base


def quat_to_yaw(qx, qy, qz, qw):
    # yaw (z-axis rotation) from quaternion (x, y, z, w)
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def yaw_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class ScanTFDebugger(Node):
    def __init__(self):
        super().__init__('scan_tf_debugger')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_timeout_sec', 2.0)
        self.declare_parameter('log_every_n', 1)  # log every N scans

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').get_parameter_value().double_value)
        self.log_every_n = int(self.get_parameter('log_every_n').get_parameter_value().integer_value)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 1)

        self.counter = 0

        self.get_logger().info('scan_tf_debugger started. Subscribing to %s' % self.scan_topic)
        self.get_logger().info('map_frame=%s odom_frame=%s base_frame=%s' % (self.map_frame, self.odom_frame, self.base_frame))

    def _lookup_transform(self, target_frame: str, source_frame: str, time_rcl: Time) -> Optional[tf2_ros.TransformStamped]:
        try:
            t = self.tf_buffer.lookup_transform(target_frame, source_frame, time_rcl, timeout=Duration(seconds=self.tf_timeout))
            return t
        except Exception as e:
            return None

    def scan_cb(self, scan: LaserScan):
        self.counter += 1
        if (self.counter % self.log_every_n) != 0:
            return

        now = self.get_clock().now()
        now_msg = now.to_msg()

        scan_time_msg = scan.header.stamp
        scan_time = time_msg_to_seconds(scan_time_msg)
        now_time = rclpy_time_to_seconds(now)
        scan_age = now_time - scan_time

        scan_frame = scan.header.frame_id if scan.header.frame_id != '' else 'laser'

        # Human-readable timestamps
        scan_iso = stamp_to_iso(scan_time_msg.sec, scan_time_msg.nanosec)
        now_iso = stamp_to_iso(now_msg.sec, now_msg.nanosec)

        log_lines = []
        log_lines.append('---- ScanTFDebugger sample #%d ----' % self.counter)
        log_lines.append('scan.header.frame_id: %s' % scan_frame)
        log_lines.append('scan.header.stamp: %s (%.9f) ; now: %s (%.9f) ; age: %.6fs' % (scan_iso, scan_time, now_iso, now_time, scan_age))
        log_lines.append('scan ranges: n=%d min=%.3f max=%.3f' % (len(scan.ranges), float(scan.range_min), float(scan.range_max)))

        # Try to get transform map <- scan_frame at scan time
        try:
            t_map_scan_at_scan = self._lookup_transform(self.map_frame, scan_frame, Time.from_msg(scan_time_msg))
            if t_map_scan_at_scan is None:
                log_lines.append('map->%s @ scan_time: lookup failed (extrapolation/timeout?)' % scan_frame)
            else:
                tf_stamp = time_msg_to_seconds(t_map_scan_at_scan.header.stamp)
                tf_iso = stamp_to_iso(t_map_scan_at_scan.header.stamp.sec, t_map_scan_at_scan.header.stamp.nanosec)
                tf_age = now_time - tf_stamp
                dt_tf_scan = tf_stamp - scan_time
                tx = t_map_scan_at_scan.transform.translation.x
                ty = t_map_scan_at_scan.transform.translation.y
                tz = t_map_scan_at_scan.transform.translation.z
                q = t_map_scan_at_scan.transform.rotation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                log_lines.append('map->%s @ scan_time: stamp=%s (%.9f) ; tf_age=%.6fs ; (tf_stamp - scan_stamp)=%.6fs' % (scan_frame, tf_iso, tf_stamp, tf_age, dt_tf_scan))
                log_lines.append('  t=(%.3f, %.3f, %.3f) yaw=%.3fdeg' % (tx, ty, tz, math.degrees(yaw)))
        except Exception as e:
            log_lines.append('Exception while looking up map->%s @ scan_time: %s' % (scan_frame, str(e)))
            log_lines.append(traceback.format_exc())

        # Try to get transform map <- scan_frame at now
        try:
            t_map_scan_at_now = self._lookup_transform(self.map_frame, scan_frame, now)
            if t_map_scan_at_now is None:
                log_lines.append('map->%s @ now: lookup failed (extrapolation/timeout?)' % scan_frame)
            else:
                tf_stamp_now = time_msg_to_seconds(t_map_scan_at_now.header.stamp)
                tf_iso_now = stamp_to_iso(t_map_scan_at_now.header.stamp.sec, t_map_scan_at_now.header.stamp.nanosec)
                txn = t_map_scan_at_now.transform.translation.x
                tyn = t_map_scan_at_now.transform.translation.y
                tzn = t_map_scan_at_now.transform.translation.z
                qn = t_map_scan_at_now.transform.rotation
                yawn = quat_to_yaw(qn.x, qn.y, qn.z, qn.w)
                log_lines.append('map->%s @ now: stamp=%s (%.9f)' % (scan_frame, tf_iso_now, tf_stamp_now))
                log_lines.append('  t=(%.3f, %.3f, %.3f) yaw=%.3fdeg' % (txn, tyn, tzn, math.degrees(yawn)))
        except Exception as e:
            log_lines.append('Exception while looking up map->%s @ now: %s' % (scan_frame, str(e)))
            log_lines.append(traceback.format_exc())

        # If both available, compare them
        if 't_map_scan_at_scan' in locals() and t_map_scan_at_scan is not None and 't_map_scan_at_now' in locals() and t_map_scan_at_now is not None:
            dx = txn - tx
            dy = tyn - ty
            dz = tzn - tz
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            yaw_now = yawn
            yaw_then = yaw
            dyaw = yaw_diff(yaw_now, yaw_then)
            log_lines.append('Difference between map->%s @ now and @ scan_time: |dpos|=%.4fm dyaw=%.3fdeg' % (scan_frame, dist, math.degrees(dyaw)))

        # Also check odom->scan_frame at scan time and compare
        try:
            t_odom_scan_at_scan = self._lookup_transform(self.odom_frame, scan_frame, Time.from_msg(scan_time_msg))
            if t_odom_scan_at_scan is None:
                log_lines.append('odom->%s @ scan_time: lookup failed' % scan_frame)
            else:
                tf_stamp_o = time_msg_to_seconds(t_odom_scan_at_scan.header.stamp)
                tf_iso_o = stamp_to_iso(t_odom_scan_at_scan.header.stamp.sec, t_odom_scan_at_scan.header.stamp.nanosec)
                to_tx = t_odom_scan_at_scan.transform.translation.x
                to_ty = t_odom_scan_at_scan.transform.translation.y
                to_tz = t_odom_scan_at_scan.transform.translation.z
                qo = t_odom_scan_at_scan.transform.rotation
                yaw_o = quat_to_yaw(qo.x, qo.y, qo.z, qo.w)
                log_lines.append('odom->%s @ scan_time: stamp=%s (%.9f) t=(%.3f, %.3f, %.3f) yaw=%.3fdeg' % (scan_frame, tf_iso_o, tf_stamp_o, to_tx, to_ty, to_tz, math.degrees(yaw_o)))
        except Exception as e:
            log_lines.append('Exception while looking up odom->%s @ scan_time: %s' % (scan_frame, str(e)))
            log_lines.append(traceback.format_exc())

        # If map->scan_at_scan and odom->scan_at_scan both exist, try to compute map->odom at scan_time by composition
        if 't_map_scan_at_scan' in locals() and t_map_scan_at_scan is not None and 't_odom_scan_at_scan' in locals() and t_odom_scan_at_scan is not None:
            try:
                mx, my = tx, ty
                ox, oy = to_tx, to_ty
                myaw = yaw
                oyaw = yaw_o
                ddx = mx - ox
                ddy = my - oy
                ddist = math.sqrt(ddx * ddx + ddy * ddy)
                dyaw_mo = yaw_diff(myaw, oyaw)
                log_lines.append('Approx map vs odom (2D approx) at scan_time: dpos=%.4fm dyaw=%.3fdeg' % (ddist, math.degrees(dyaw_mo)))
            except Exception:
                pass
                
        log_lines.append('-------------------------------------')

        for l in log_lines:
            self.get_logger().info(l)


def main(args=None):
    rclpy.init(args=args)
    node = ScanTFDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
