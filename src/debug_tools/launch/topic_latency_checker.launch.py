from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='debug_tools',
            executable='topic_latency_checker',
            name='topic_latency_checker',
            parameters=[{
                'topic': '/scan',
                'duration': 10.0,
                'verbose': True
            }]
        )
    ])