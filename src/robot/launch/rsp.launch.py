import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

motor_controller_device = 'usb-FTDI_FT232R_USB_UART_AQ029HJR-if00-port0';


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Check whether to use ros2_control, gazebo_control otherwise
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    motor_controller_path = find_motor_controller_path(motor_controller_device)

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('diff_drive_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, 
                                        ' use_ros2_control:=', use_ros2_control,
                                        ' sim_mode:=', use_sim_time, 
                                        ' device:=', motor_controller_path])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'
        ),

        node_robot_state_publisher
    ])


def find_motor_controller_path(motor_controller_device):
    symbolic_link_path = f'/dev/serial/by-id/{motor_controller_device}'

    if not os.path.exists(symbolic_link_path):
        return ''

    try:
        actual_file_path = os.path.realpath(symbolic_link_path)
    except OSError as e:
        raise RuntimeError(f'Error resolving symbolic link: {e}')

    if not os.path.exists(actual_file_path):
        raise RuntimeError(f'The resolved path "{actual_file_path}" does not exist')

    return actual_file_path
