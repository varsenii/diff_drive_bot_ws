import math
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.time
from tf_transformations import quaternion_from_euler, quaternion_multiply

from waypoint_tasking.navigation.utils import transform_to_goal


class NavigationManager:
    def __init__(self, node, tf_buffer):
        self.node = node
        self.tf_buffer = tf_buffer
        self.callback_group = ReentrantCallbackGroup()
        self.nav2_client = ActionClient(
            self.node,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self.callback_group,
        )
        self.current_goal_handle = None  # To track the current goal handle
        self.logger = self.node.get_logger()

        self.global_frame = "map"
        self.base_link_frame = "base_link"

    def move_by_command(self, distance, direction):
        # Get the current pose TF
        transform = self.get_current_pose()
        self.logger.info(f"Current pose: {transform}")

        # Convert to the goal
        goal = transform_to_goal(transform)

        # Adjust the goal to reflect the desired target
        goal.pose.pose.position.x += distance * math.cos(goal.pose.pose.orientation.z)
        goal.pose.pose.position.y += distance * math.sin(goal.pose.pose.orientation.z)
        self.logger.info(f"Target goal: {goal}")

        # Send the goal
        self.send_goal(goal)

    def rotate_by_command(self, angle, direction):
        # Get the current orientation quaternion
        transform = self.get_current_pose()
        orientation = transform.transform.rotation

        orientation_quat = (orientation.x, orientation.y, orientation.z, orientation.w)

        # Create the quaternion representing the desired rotation
        desired_rotation = quaternion_from_euler(0, 0, math.radians(angle))

        # Compute the target orientation quaternion
        target_quaternion = quaternion_multiply(desired_rotation, orientation_quat)

        # Convert the TF to goal
        goal = transform_to_goal(transform=transform)

        # Adjust the goal to reflect the desired target rotation
        goal.pose.pose.orientation.x = target_quaternion[0]
        goal.pose.pose.orientation.y = target_quaternion[1]
        goal.pose.pose.orientation.z = target_quaternion[2]
        goal.pose.pose.orientation.w = target_quaternion[3]

        # Send the goal
        self.send_goal(goal)

    def move_to_waypoint(self, goal_msg):
        if self.current_goal_handle:
            self.logger.debug("Cancelling current goal...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda future: self.send_goal(goal_msg))
        else:
            self.send_goal(goal_msg)

    def send_goal(self, goal_msg):
        if self.nav2_client.wait_for_server(timeout_sec=10.0):
            self.logger.debug("Sending goal...")
            future = self.nav2_client.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)
        else:
            self.logger.error("Action server not available.")

    def goal_response_callback(self, future):
        try:
            self.current_goal_handle = future.result()
            if not self.current_goal_handle.accepted:
                self.logger.error("Goal was rejected by the action server.")
                return

            self.logger.debug("Goal accepted by action server. Waiting for result...")
            result_future = self.current_goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

        except Exception as e:
            self.logger.error(f"Exception occurred: {e}")

    def result_callback(self, future):
        try:
            result = future.result()
            if result:
                self.logger.debug(f"Goal result received: {result}")
                self.parent_tasker.on_nav_goal_succeed()
            else:
                self.logger.error("Failed to receive result from action server.")
        except Exception as e:
            self.logger.error(f"Exception occurred while retrieving result: {e}")

    def set_parent_tasker(self, tasker):
        self.parent_tasker = tasker

    def get_current_pose(self):
        return self.tf_buffer.lookup_transform(
            self.global_frame, self.base_link_frame, rclpy.time.Time()
        )
