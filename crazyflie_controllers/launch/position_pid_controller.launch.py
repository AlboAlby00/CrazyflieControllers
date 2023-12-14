from launch_ros.actions import Node
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument

import os


def generate_launch_description():

    use_simulation_param = DeclareLaunchArgument("mode", default_value="sim", choices=["sim", "real"])

    position_controller = Node(
        package='crazyflie_controllers',
        executable='position_pid_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    attitude_controller = Node(
        package='crazyflie_controllers',
        executable='attitude_pid_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        condition=LaunchConfigurationEquals('mode', 'sim')
    )

    return LaunchDescription([
        use_simulation_param,
        attitude_controller,
        position_controller
    ])
