import rclpy
from geometry_msgs.msg import TransformStamped, Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class TFManager:
    def __init__(self, buffer):
        self.tf_buffer = buffer

    def get_current_pose_with_yaw(self, global_frame="map", robot_frame="base_footprint") -> dict[str, float]:
        transform = self.get_tf(global_frame, robot_frame)

        q = transform.transform.rotation
        quaternion = (q.x, q.y, q.z, q.w)
        _, _, yaw = euler_from_quaternion(quaternion)

        return {
            "x": transform.transform.translation.x,
            "y": transform.transform.translation.y,
            "yaw": yaw
        }

    def get_tf(self, target_frame: str, source_frame: str) -> TransformStamped:
        return self.tf_buffer.lookup_transform(
            target_frame, source_frame, rclpy.time.Time()
        )
