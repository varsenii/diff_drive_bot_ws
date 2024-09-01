from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Whether to use the simulated time'
        ),
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
                'output_frame': 'camera_link',
                'scan_height': 60,
                'scan_time': 0.1,
                'range_min': 0.4,
                'range_max': 2.0
            }]
        ),
        Node(
            package='sensor_fusion',
            executable='scan_fusion_node',
            name='scan_fusion_node',
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        )
    ])
