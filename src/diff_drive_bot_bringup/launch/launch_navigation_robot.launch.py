from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory

import os


def generate_launch_description():
    map = LaunchConfiguration('map')

    map_arg = DeclareLaunchArgument('map', default_value='', description='Full path to map file to load')


    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('diff_drive_bot'), 'launch', 'launch_robot.launch.py')
        )
    )
    
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={'map': map, 'use_sim_time': 'false'}.items(),
        condition=IfCondition(PythonExpression(['"', map, '" != ""']))
    )

    return LaunchDescription([
        map_arg,
        localization_launch
    ])