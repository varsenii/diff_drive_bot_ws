from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup

class NavigationManager:

    def __init__(self, node):
        self.node = node
        self.callback_group = ReentrantCallbackGroup()
        self.nav2_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group)
        self.current_goal_handle = None  # To track the current goal handle
        self.logger = self.node.get_logger()

    def move_to_waypoint(self, goal_msg):
        if self.current_goal_handle:
            self.logger.debug('Cancelling current goal...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda future: self.send_goal(goal_msg))
        else:
            self.send_goal(goal_msg)
    
    def send_goal(self, goal_msg):
        if self.nav2_client.wait_for_server(timeout_sec=10.0):
            self.logger.debug('Sending goal...')
            future = self.nav2_client.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)
        else:
            self.logger.error('Action server not available.')

    def goal_response_callback(self, future):
        try:
            self.current_goal_handle = future.result()
            if not self.current_goal_handle.accepted:
                self.logger.error('Goal was rejected by the action server.')
                return

            self.logger.debug('Goal accepted by action server. Waiting for result...')
            result_future = self.current_goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

        except Exception as e:
            self.logger.error(f'Exception occurred: {e}')

    def result_callback(self, future):
        try:
            result = future.result()
            if result:
                self.parent_tasker.perform_tasks_or_terminate()
            else:
                self.logger.error('Failed to receive result from action server.')
        except Exception as e:
            self.logger.error(f'Exception occurred while retrieving result: {e}')

    def set_parent_tasker(self, tasker):
        self.parent_tasker = tasker
