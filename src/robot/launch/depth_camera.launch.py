from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth', '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/camera/scan')
            ],
            parameters=[{
                'output_frame': 'laser_frame',
                'scan_height': 80,
                'scan_time': 0.033,
                'range_min': 0.4,
                'range_max': 8.0
            }]
        )
    ])
