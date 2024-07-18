from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Set CUDA_VISIBLE_DEVICES to an empty string to force TensorFlow to use CPU
    # This is necessary to avoid compatibility issues related to GPU usage
    cuda_env_var = SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '')

    face_recognition_node = Node(
        package='face_recognition',
        executable='face_recognition',
        name='face_recognition'
    )

    return LaunchDescription([
        cuda_env_var,
        face_recognition_node
    ])