import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from diff_drive_bot_interfaces.srv import StartWaypointTask
from waypoint_tasking.tasks.face_recognition import FaceRecognitionManager
from waypoint_tasking.waypoint_manager import WaypointManager
from waypoint_tasking.navigation import NavigationManager



class WaypointTasker(Node):

    def __init__(self):
        super().__init__('waypoint_tasker')
        
        self.waypoint_manager = WaypointManager(self)
        self.navigation_manager = NavigationManager(self)
        self.face_recognition_manager = FaceRecognitionManager(self)

        self.navigation_manager.set_parent_tasker(self)
        self.face_recognition_manager.set_parent_tasker(self)

        self.start_navigation_service = self.create_service(StartWaypointTask, 'start_waypoint_task', self.start_waypoint_task_callback)

        self.logger = self.get_logger()
        self.current_waypoint_index = 0

    def start_waypoint_task_callback(self, request, response):
        self.current_waypoint_index = 0
        waypoints = self.waypoint_manager.get_waypoints()
        if waypoints:
            self.move_to_waypoint(waypoints[self.current_waypoint_index])
        else:
            self.logger.error('No waypoints available to navigate.')

        response.success = True
        return response

    def move_to_waypoint(self, marker):
        self.navigation_manager.move_to_waypoint(marker)

    def identify_by_face(self):
        self.face_recognition_manager.identify_by_face()

    def on_face_recognition_complete(self):
        self.current_waypoint_index += 1
        waypoints = self.waypoint_manager.get_waypoints()
        if self.current_waypoint_index < len(waypoints):
            self.move_to_waypoint(waypoints[self.current_waypoint_index])
        else:
            self.logger.info('All waypoints have been visited.')

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