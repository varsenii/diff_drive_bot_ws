import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class Nav2Client:

    def __init__(self):
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    
    async def send_goal(self, goal_pose):
        self.logger
        if not self.navigation_client.wait_for_server(timeout_sec=10.0):
            # self.logger.error('Navigation action server not available')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = goal_pose

        # self.logger.info('Sending navigation goal...')
        future = self.navigation_client.send_goal_async(goal_msg)
        result = await future
        if result.status == rclpy.action.GoalStatus.SUCCEEDED:
            # self.get_logger().info('Reached waypoint successfully!')
            return True
        else:
            # self.get_logger().warn('Failed to reach waypoint.')
            return False

