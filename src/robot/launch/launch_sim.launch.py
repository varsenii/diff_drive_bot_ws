import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():
    package_name = 'diff_drive_bot'
    robot_name = 'robot'

    use_ros2_control = LaunchConfiguration('use_ros2_control')

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': use_ros2_control}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' +  gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', robot_name],
                        output='screen')
    
    # Run the spawner node to start the diff_cont controller
    diff_drive_spawner = Node(
        package='controller_manager', 
        executable='spawner',
        arguments=['diff_cont'],
        condition=IfCondition(use_ros2_control)
    )

    delayed_diff_drive_spawner = TimerAction(period=3.0, actions=[diff_drive_spawner])

    # Run the spawner node to start the joint_broad controller
    joint_broad_spawner = Node(
        package='controller_manager', 
        executable='spawner',
        arguments=['joint_broad'],
        condition=IfCondition(use_ros2_control)
    )

    delayed_joint_broad_spawner = TimerAction(period=3.0, actions=[joint_broad_spawner])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Specifyies whether to use ros2_control'),
        rsp,
        gazebo,
        spawn_entity,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])