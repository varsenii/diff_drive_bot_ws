import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visualization_msgs.msg import MarkerArray, Marker
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from diff_drive_bot_interfaces.srv import FaceIdentification, StartWaypointTask


class WaypointTasker(Node):

    def __init__(self):
        super().__init__('waypoint_tasker')
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscription to MarkerArray with a callback group for concurrency
        self.waypoint_subscription = self.create_subscription(
            MarkerArray,
            '/waypoints',
            self.waypoint_callback,
            10,
            callback_group=self.callback_group
        )
        self.start_navigation_service = self.create_service(StartWaypointTask, 'start_waypoint_task', self.start_waypoint_task_callback)

        self.face_recognition_client = self.create_client(FaceIdentification, 'identify_person_by_face')
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group)
        
        self.logger = self.get_logger()
        self.waypoints = []
        self.current_goal_handle = None  # To track the current goal handle
        self.current_waypoint_index = 0

    def waypoint_callback(self, msg):
        self.logger.info(f'Received {len(msg.markers)} markers')      
        self.waypoints = [marker for i, marker in enumerate(msg.markers) if i % 3 == 0]
        self.logger.info(f'Derived {len(self.waypoints)} waypoints')

    def start_waypoint_task_callback(self, request, response):
        self.current_waypoint_index = 0
        if self.waypoints:
            self.move_to_waypoint(self.waypoints[self.current_waypoint_index])
        else:
            self.logger.error('No waypoints available to navigate.')

        response.success = True
        return response

    
    def move_to_waypoint(self, marker):
        goal_msg = self.marker_to_goal(marker)
        
        if self.current_goal_handle:
            self.logger.info('Cancelling current goal...')
            self.current_goal_handle.cancel_goal_async()

        self.send_goal(goal_msg)
    
    def send_goal(self, goal_msg):
        if self.nav2_client.wait_for_server(timeout_sec=10.0):
            self.logger.info('Sending goal...')
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

            self.logger.info('Goal accepted by action server. Waiting for result...')
            result_future = self.current_goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

        except Exception as e:
            self.logger.error(f'Exception occurred: {e}')

    def result_callback(self, future):
        try:
            result = future.result()
            if result:
                self.logger.info(f'Goal result: {result.result}')
                self.identify_by_face()
            else:
                self.logger.error('Failed to receive result from action server.')
        except Exception as e:
            self.logger.error(f'Exception occurred while retrieving result: {e}')

    def marker_to_goal(self, marker: Marker) -> NavigateToPose.Goal:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = marker.header.stamp
        goal_msg.pose.header.frame_id = marker.header.frame_id
        goal_msg.pose.pose.position = marker.pose.position
        goal_msg.pose.pose.orientation = marker.pose.orientation
        return goal_msg

    def identify_by_face(self):
        self.logger.info('Performing face recognition...')
        request = FaceIdentification.Request()
        
        future = self.face_recognition_client.call_async(request)
        future.add_done_callback(self.face_recognition_callback)

    def face_recognition_callback(self, future):
        try:
            result = future.result()
            self.logger.info(f'Face recognition result: {result}')
            
            # Move to the next waypoint if any
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.move_to_waypoint(self.waypoints[self.current_waypoint_index])
            else:
                self.logger.info('All waypoints have been visited.')
        except Exception as e:
            self.logger.error(f'Exception occurred during face recognition: {e}')

def main():
    rclpy.init()

    waypoint_tasker = WaypointTasker()

    # Use MultiThreadedExecutor to handle concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_tasker)

    try:
        executor.spin()
    finally:
        waypoint_tasker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
