from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler
from time import sleep


class Navigtor():
    def __init__(self, logger):
        self.logger = logger
        self.nav_client = BasicNavigator()
    
    def go_to_pose(self, x: float, y: float, yaw: float) -> TaskResult:
        self.nav_client.waitUntilNav2Active()

        pose = self._build_pose(x=x, y=y, yaw=yaw)

        self.nav_client.goToPose(pose)

        while not self.nav_client.isTaskComplete():
            feedback = self.nav_client.getFeedback()
            
            if feedback:
                self.logger.info(f'Navigating... Distance remaining: {feedback.distance_remaining:.2f} meters')
            
            sleep(0.5)

        result = self.nav_client.getResult()
        if result == TaskResult.SUCCEEDED:
            self.logger.info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.logger.warn('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.logger.error('Goal failed!')

        return self.nav_client.getResult()

    def _build_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav_client.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self._convert_yaw_to_quaternion(yaw)
        return pose

    def _convert_yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])