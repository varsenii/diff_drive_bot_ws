from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='debug_tools',
            executable='topic_ts_checker',
            name='topic_ts_checker',
            parameters=[{
                'topic1': '/diff_cont/odom',
                'topic2': '/scan',
                'slop': 0.05,
                'queue_size': 10,
                'duration': 10.0
            }]
        )
    ])