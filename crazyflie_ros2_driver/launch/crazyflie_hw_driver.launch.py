
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    enable_motor_level_control_param = DeclareLaunchArgument(
        'motor_level_control',
        default_value='false',
        description='If true, then the drone will be controlled on motor level'
    )
    target_publisher = Node(
        package='crazyflie_ros2_driver',
        executable='crazyflie_hw_attitude_driver',
        arguments=['--ros-args', '--log-level', 'info'],
        condition=UnlessCondition(LaunchConfiguration('motor_level_control'))
    )

    target_publisher_2 = Node(
        package='crazyflie_ros2_driver',
        executable='crazyflie_hw_motor_vel_driver',
        arguments=['--ros-args', '--log-level', 'info'],
        condition=IfCondition(LaunchConfiguration('motor_level_control'))
    )


    return LaunchDescription([
        enable_motor_level_control_param,
        target_publisher,
        target_publisher_2
    ])
