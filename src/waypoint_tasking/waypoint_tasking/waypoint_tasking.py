import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import rclpy.time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from diff_drive_bot_interfaces.srv import StartWaypointTask
from waypoint_tasking.tasks.face_recognition import FaceRecognitionManager
from waypoint_tasking.waypoint_manager import WaypointManager
from waypoint_tasking.navigation import NavigationManager
from waypoint_tasking.utils import marker_to_goal



class WaypointTasker(Node):

    def __init__(self):
        super().__init__('waypoint_tasker')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
            depth=10,
        )

        self.start_navigation_service = self.create_service(StartWaypointTask, 'start_waypoint_task', self.start_waypoint_task_callback, qos_profile=qos_profile)
        
        self.waypoint_manager = WaypointManager(self)
        self.navigation_manager = NavigationManager(self)
        self.face_recognition_manager = FaceRecognitionManager(self)

        self.navigation_manager.set_parent_tasker(self)
        self.face_recognition_manager.set_parent_tasker(self)

        self.logger = self.get_logger()
        self.current_waypoint_index = 0
        self.is_coming_back_to_starting_point = False

        self.supported_tasks = {
            'face_recognition': self.face_recognition_manager
        }
        self.executed_tasks_per_waypoint = 0
        self.tasks = []


    def start_waypoint_task_callback(self, request, response):
        self.tasks = request.tasks
        self.logger.info(f'Received tasks: {self.tasks}')

        self.current_waypoint_index = 0
        self.executed_tasks_per_waypoint = 0

        waypoints = self.waypoint_manager.get_waypoints()

        if waypoints:
            goal_msg = marker_to_goal(waypoints[self.current_waypoint_index])
            self.move_to_waypoint(goal_msg)
        else:
            self.logger.error('No waypoints available to navigate.')

        response.success = True
        return response

    def move_to_waypoint(self, marker):
        self.navigation_manager.move_to_waypoint(marker)

    def perform_tasks_or_terminate(self):
        if self.is_coming_back_to_starting_point is True:
            self.logger.info('The robot came back to the starting point')
            self.is_coming_back_to_starting_point = False
            return

        for task in self.tasks:
            self.logger.info(f'Performing "{task}" task...')
            self.supported_tasks[task].perform_task()

    def on_task_complete(self, task):
        self.executed_tasks_per_waypoint += 1

        self.logger.info(f'"{task}" task completed')
        self.logger.info(f'Executed tasks: {self.executed_tasks_per_waypoint}/{len(self.tasks)}')

        if self.executed_tasks_per_waypoint < len(self.tasks):
            return
        
        self.logger.info('All tasks are completed')

        self.current_waypoint_index += 1
        waypoints = self.waypoint_manager.get_waypoints()

        if self.current_waypoint_index < len(waypoints):
            goal_msg = marker_to_goal(waypoints[self.current_waypoint_index])
            self.move_to_waypoint(goal_msg)
        else:
            self.logger.info('All waypoints have been visited.')
            self.is_coming_back_to_starting_point = True
            self.logger.info('Coming back to the starting point...')
            self.navigation_manager.move_to_waypoint(self.waypoint_manager.starting_pose_goal)
    

def main():
    rclpy.init()

    waypoint_tasker = WaypointTasker()

    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_tasker)

    try:
        executor.spin()
    finally:
        waypoint_tasker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
