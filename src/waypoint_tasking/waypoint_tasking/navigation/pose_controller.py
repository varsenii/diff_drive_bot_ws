#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class SemiClosedLoopPoseController(Node):
    """Simple semi-closed-loop controller using odometry."""

    def __init__(self):
        super().__init__('semi_closed_loop_pose_controller')

        # Publisher and subscriber
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        # Control parameters
        self.linear_kp = 0.5
        self.angular_kp = 1.0
        self.distance_tolerance = 0.05
        self.yaw_tolerance = math.radians(2.0)

        self.get_logger().info("Semi-closed-loop PoseController node initialized.")

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.odom_received = True

    def stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def rotate_to_angle(self, target_yaw):
        rate = self.create_rate(20)
        twist = Twist()

        self.get_logger().info(f"Rotating to {math.degrees(target_yaw):.2f} deg")

        while rclpy.ok():
            rclpy.spin_once(self)
            if not self.odom_received:
                continue

            error = self.normalize_angle(target_yaw - self.yaw)
            if abs(error) < self.yaw_tolerance:
                break

            twist.angular.z = max(min(self.angular_kp * error, 0.5), -0.5)
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            rate.sleep()

        self.stop()

    def move_straight_to_point(self, target_x, target_y):
        rate = self.create_rate(20)
        twist = Twist()

        self.get_logger().info(f"Moving straight to ({target_x:.2f}, {target_y:.2f})")

        while rclpy.ok():
            rclpy.spin_once(self)
            if not self.odom_received:
                continue

            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.hypot(dx, dy)
            if distance < self.distance_tolerance:
                break

            desired_yaw = math.atan2(dy, dx)
            yaw_error = self.normalize_angle(desired_yaw - self.yaw)

            twist.linear.x = min(self.linear_kp * distance, 0.3)
            twist.angular.z = max(min(1.0 * yaw_error, 0.5), -0.5)
            self.cmd_pub.publish(twist)
            rate.sleep()

        self.stop()

    def move_to_pose(self, target_x, target_y, target_yaw_deg):
        target_yaw = math.radians(target_yaw_deg)
        self.get_logger().info(f"Target pose: x={target_x}, y={target_y}, yaw={target_yaw_deg}°")

        # Wait for odometry
        self.get_logger().info("Waiting for /odom...")
        start_time = time.time()
        while not self.odom_received and rclpy.ok():
            rclpy.spin_once(self)
            if time.time() - start_time > 5.0:
                self.get_logger().warn("No /odom received after 5 s. Proceeding anyway.")
                break
            time.sleep(0.1)

        # Step 1: Rotate toward target
        desired_yaw = math.atan2(target_y - self.y, target_x - self.x)
        self.rotate_to_angle(desired_yaw)

        # Step 2: Move straight
        self.move_straight_to_point(target_x, target_y)

        # Step 3: Align with final orientation
        self.rotate_to_angle(target_yaw)

        self.get_logger().info("✅ Goal pose reached successfully!")
        self.stop()

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = SemiClosedLoopPoseController()

    try:
        # Example target pose
        node.move_to_pose(1.0, 0.5, 90)  # x=1m, y=0.5m, yaw=90°
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
