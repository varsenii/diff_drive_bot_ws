import rclpy
import rclpy.time
from geometry_msgs.msg import TransformStamped


class Navigator:
    def __init__(self, node, tf_buffer):
        self.node = node
        self.tf_buffer = tf_buffer
        self.logger = self.node.get_logger()

        self.global_frame = "map"
        self.base_link_frame = "base_link"

    def get_current_pose(self) -> TransformStamped:
        try:
            return self.tf_buffer.lookup_transform(
                self.global_frame, self.base_link_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.logger.warn(f"Failed to get current pose: {e}")
            raise

    def get_current_position(self) -> tuple[float, float]:
        try:
            current_pose = self.get_current_pose()
            return (
                current_pose.transform.translation.x,
                current_pose.transform.translation.y,
            )
        except Exception as e:
            self.logger.error(f"Failed to get current position: {e}")
            raise RuntimeError(f"Failed to get current position: {e}")
