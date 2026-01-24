from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="voskros",
            executable="vosk",
            namespace="speech/stt",
            name="voskros"
        )
    ])