from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    world_file_name = LaunchConfiguration('world', default=PathJoinSubstitution([FindPackageShare('robot'), 'worlds', 'obstacles.world']))
    map_file_name = LaunchConfiguration('map', default=PathJoinSubstitution([FindPackageShare('robot'), 'config', 'course_map.yaml']))
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')

    def print_resolved_paths(context):
        world = context.launch_configurations['world']
        map = context.launch_configurations['map']
        return [
            LogInfo(msg=f"Resolved World file: {os.path.abspath(world)}"),
            LogInfo(msg=f"Resolved Map file: {os.path.abspath(map)}")
        ]

    debug_log = OpaqueFunction(function=print_resolved_paths)

    # Launch diff_drive_bot simulation
    diff_drive_bot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('diff_drive_bot'), 'launch', 'launch_sim.launch.py'
        ])),
        launch_arguments={
            'world': world_file_name,
            'use_ros2_control': use_ros2_control
        }.items()
    )

    # Launch nav2_bringup localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('diff_drive_bot'), 'launch', 'localization_launch.py'
        ])),
        launch_arguments={
            'map': map_file_name,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Launch nav2_bringup navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('diff_drive_bot'), 'launch', 'navigation_launch.py'
        ])),
        launch_arguments={
            'use_sim_time': 'false',  # Explicitly set to false
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    return LaunchDescription([
        debug_log,
        diff_drive_bot_launch,
        localization_launch,
        navigation_launch
    ])
