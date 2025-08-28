import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math

from waypoint_tasking.navigation.navigation import Navigator
from waypoint_tasking.navigation.obsactle_detector import ObstacleDetector


class TwistCommander(Navigator):
    def __init__(self, node, tf_buffer):
        super().__init__(node=node, tf_buffer=tf_buffer)

        self.speed = 0.25
        self.rate = self.node.create_rate(1)  # Control loop frequency

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)

        self.cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", qos)

        self.obstacle_detector = ObstacleDetector(node=node)

    def move_by_command(self, distance=None, direction=None):
        # Create twist message
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

            distance_traveled = math.sqrt(
                (x_current - x_init) ** 2 + (y_current - y_init) ** 2
            )

            distance_log = f"Traveled distance: {distance_traveled}"
            if distance:
                distance_log += f"/{distance}"
            self.logger.debug(distance_log)

            # Check whether there're obstacles
            if self.obstacle_detector.has_obstacles_linear(
                longitudinal_dist=0.5, traversal_dist=0.25
            ):
                pass
                # return

            # Stop if the target distance has been traveled
            if distance and distance_traveled >= distance:
                self.logger.info("The desired distance has been traveled")
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            # Send the velocity command
            # self.cmd_vel_pub.publish(twist)

            # Wait according to the target Hz
            self.rate.sleep()
