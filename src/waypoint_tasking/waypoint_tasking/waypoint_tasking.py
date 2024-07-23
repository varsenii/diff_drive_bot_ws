import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import rclpy.time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.logging import LoggingSeverity

from diff_drive_bot_interfaces.msg import Intent, Slot
from diff_drive_bot_interfaces.srv import StartWaypointTasks
from waypoint_tasking.tasks.face_recognition import FaceRecognitionManager
from waypoint_tasking.waypoint_manager import WaypointManager
from waypoint_tasking.navigation import NavigationManager
from waypoint_tasking.utils import marker_to_goal, ReportManager


class WaypointTasker(Node):

    def __init__(self):
        super().__init__('waypoint_tasker')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
            depth=10,
        )

        self.start_navigation_service = self.create_service(StartWaypointTasks, 'start_waypoint_tasks', self.start_waypoint_tasks_callback, qos_profile=qos_profile)
        self.sub_intent = self.create_subscription(Intent, '/intent', self.intent_callback, 10)

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

        self.report_manager = ReportManager()

        self.logger.set_level(LoggingSeverity.DEBUG)


    def start_waypoint_tasks_callback(self, request, response):
        intialized = self.init_navigation_and_task_execution(request.waypoints, request.tasks, request.return_to_start)
        
        if intialized is False:
            self.logger.error('No waypoints available to navigate.')
            response.success = False
            return response

        response.success = True
        return response

    def intent_callback(self, intent):
        self.logger.info(f'Received intent: {intent}')

        if intent.name == 'faceRecognition':
            tasks = ['face_recognition']
        elif intent.name == 'waypointNavigation':
            tasks = []
        else:
            self.logger.warning(f'Unsupported intent: {intent.name}')
            return
        
        for slot in intent.slots:
            self.logger.debug(f'Slot: {slot.name}')

            if slot.name == 'waypointOrdinal':
                waypoints = [int(slot.value[:-2]) - 1]
                return_to_start = False
            elif slot.name == 'waypointInteger':
                waypoints = [int(slot.value) - 1]
                return_to_start = False
            elif slot.name == 'allWaypoints':
                waypoints = []
                return_to_start = True
        
        self.init_navigation_and_task_execution(waypoints, tasks, return_to_start)
                    
    def init_navigation_and_task_execution(self, waypoint_indexes, tasks, return_to_start):
        self.logger.info(f'Tasks to perform: {tasks}')
        self.logger.info(f'Waypoints to navigate: {"all" if len(waypoint_indexes) == 0 else waypoint_indexes}')
        self.logger.info(f'Return to start: {return_to_start}')

        self.current_waypoint_index = 0
        self.executed_tasks_per_waypoint = 0
        self.tasks = tasks
        self.waypoints_to_visit = self.waypoint_manager.get_waypoints(waypoint_indexes)
        self.return_to_start = return_to_start

        self.report_manager.create_report()

        if self.waypoints_to_visit:
            self.move_to_waypoint(self.waypoints_to_visit[self.current_waypoint_index])
            return True

    def move_to_waypoint(self, waypoint):
        self.logger.info(f'Moving to the waypoint {self.current_waypoint_index}...')
        goal_msg = marker_to_goal(waypoint)
        self.navigation_manager.move_to_waypoint(goal_msg)

    def perform_tasks_or_terminate(self):
        if self.is_coming_back_to_starting_point is True:
            self.logger.info('The robot came back to the starting point')
            self.is_coming_back_to_starting_point = False
            return

        for task in self.tasks:
            self.logger.info(f'Performing "{task}" task...')
            self.supported_tasks[task].perform_task()

    def on_task_complete(self, task, result):
        self.executed_tasks_per_waypoint += 1

        self.logger.info(f'"{task}" task completed')
        self.logger.info(f'Executed tasks: {self.executed_tasks_per_waypoint}/{len(self.tasks)} at waypoint {self.current_waypoint_index}')

        self.report_manager.log_task_result(waypoint_index=self.current_waypoint_index, task=task, result=result)

        # Proceed to the next waypoint only if all tasks have been executed
        if self.executed_tasks_per_waypoint < len(self.tasks):
            return
        
        self.logger.info(f'All tasks are completed at waypoint {self.current_waypoint_index}')

        self.current_waypoint_index += 1
        self.executed_tasks_per_waypoint = 0

        if self.current_waypoint_index < len(self.waypoints_to_visit):
            self.move_to_waypoint(self.waypoints_to_visit[self.current_waypoint_index])
        else:
            self.logger.info('All waypoints have been visited.')

            if self.return_to_start is False:
                return

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
