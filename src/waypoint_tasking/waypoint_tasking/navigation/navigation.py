import rclpy
import rclpy.time
from geometry_msgs.msg import TransformStamped, Twist, Vector3
import math


class Navigator:
    def __init__(self, node, tf_buffer):
        self.speed = 0.25
        self.xy_tolerance = 0.10
        self.yaw_tolerance = 0.1
        self.rate = self.node.create_rate(1)  # Control loop frequency

        self.node = node
        self.tf_buffer = tf_buffer
        self.logger = self.node.get_logger()

        self.global_frame = "map"
        self.base_link_frame = "base_link"
    
    def move_to_position(self, x: float, y: float, z: float):
        distance = (x**2 + y**2 + z**2) ** 0.5
        self.logger.info(f"Moving translation by ({x}, {y}, {z}), distance: {distance}")

        twist = Twist()
        twist.linear = Vector3(x=self.speed, y=0.0, z=0.0)
        twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        # Get the initial postion
        try:
            x_init, y_init = self.get_current_position()
        except RuntimeError:
            if distance:
                self.logger.error(
                    "Aborting the navigation due to unknown initial position"
                )
                return

        # Navigte in closed-loop
        while rclpy.ok():
            # Compute the traveled distance
            x_current, y_current = self.get_current_position()

            distance_remained = math.sqrt(
                (x - x_current) ** 2 + (y - y_current) ** 2
            )

            self.logger.debug(f"Remained distance: {distance_remained}/{distance}")

            # Stop if the robot is close enough to the target
            if distance_remained <= self.xy_tolerance:
                self.logger.info("The target position is reached")
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            # Send the velocity command
            self.cmd_vel_pub.publish(twist)

            # Wait according to the target Hz
            self.rate.sleep()
    
    def move_rotation(self, yaw: float):
        self.logger.info(f"Rotating by yaw: {yaw}")

        twist = Twist()
        twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        twist.angular = Vector3(x=0.0, y=0.0, z=self.speed)

        # Get the initial postion
        try:
            x_init, y_init = self.get_current_position()
        except RuntimeError:
            if yaw:
                self.logger.error(
                    "Aborting the navigation due to unknown initial position"
                )
                return

        # Navigte in closed-loop
        while rclpy.ok():
            # Compute the traveled distance
            x_current, y_current = self.get_current_position()

            distance_remained = math.sqrt(
                (x - x_current) ** 2 + (y - y_current) ** 2
            )

            self.logger.debug(f"Remained distance: {distance_remained}/{distance}")

            # Stop if the robot is close enough to the target
            if distance_remained <= self.xy_tolerance:
                self.logger.info("The target position is reached")
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            # Send the velocity command
            self.cmd_vel_pub.publish(twist)

            # Wait according to the target Hz
            self.rate.sleep()

    def get_current_pose(self) -> TransformStamped:
        try:
            return self.tf_buffer.lookup_transform(
                self.global_frame, self.base_link_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.logger.warn(f"Failed to get current pose: {e}")
            raise

    def get_current_position(self) -> tuple[float, float]:
        try:
            current_pose = self.get_current_pose()
            return (
                current_pose.transform.translation.x,
                current_pose.transform.translation.y,
            )
        except Exception as e:
            self.logger.error(f"Failed to get current position: {e}")
            raise RuntimeError(f"Failed to get current position: {e}")

    def get_current_yaw(self) -> float:
        try:
            current_pose = self.get_current_pose()
            q = current_pose.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return yaw
        except Exception as e:
            self.logger.error(f"Failed to get current yaw: {e}")
            raise RuntimeError(f"Failed to get current yaw: {e}")