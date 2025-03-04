import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    planner_dir = get_package_share_directory('planner')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': planner_dir + '/pddl/monitor.pddl'}.items()
    )

    move_node = Node(
        package='planner',
        executable='move_action_node',
        name='move_action_node',
        output='screen'
    )

    scan_node = Node(
        package='planner',
        executable='scan_action_node',
        name='scan_action_node',
        output='screen'
    )

    return LaunchDescription([
        plansys2_cmd,
        move_node,
        scan_node
    ])
    