import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import json

from waypoint_tasking.tasks import NavigateToPose, SavePose


class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')

        self.logger = self.get_logger()

        db_path = '/home/varsenii/Robotics/ros2_workspaces/diff_drive_bot_ws/src/waypoint_tasking/waypoint_tasking/database/poses.yaml'
        
        self.ai_command_sub = self.create_subscription(
            String,
            'ai_command',
            self.ai_command_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

        self.navigate_task = NavigateToPose(logger=self.logger, db_path=db_path)
        self.save_position_task = SavePose(logger=self.logger, db_path=db_path, buffer=self.tf_buffer)

    def ai_command_callback(self, msg):
        try:
            self.logger.info(f'Received AI command: {msg.data}')
            
            # Deserialize the command
            command = msg.data
            json_command = json.loads(command)
            
            # Execute the command
            match json_command['type']:
                case 'navigate':
                    self.navigate_task.execute(json_command)
                case 'save':
                    self.save_position_task.execute(json_command)
                case _:
                    self.logger.error(f"Unknown command type: {json_command['type']}")
                    return
        except Exception as e:
            self.logger.error(f"Error processing AI command: {e}")

def main(args=None):
    rclpy.init(args=args)

    task_manager = TaskManager()

    rclpy.spin(task_manager)

    task_manager.destroy_node()
    rclpy.shutdown()