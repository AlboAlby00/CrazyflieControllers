from launch_ros.actions import Node
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch import LaunchDescription
from ament_index_python import get_package_share_directory 


def generate_launch_description():
    """
    Launch file that starts the attitude PID controller for the simulation
    (On the real Crazyflie, the attitude controller runs on the Crazyflie itself)
    """

    use_joy_param = DeclareLaunchArgument('use_joy', default_value='false')

    use_joy_param = DeclareLaunchArgument(
        'use_joy',
        default_value='false',
        description='Use joystick control (True/False)'
    )

    controller = Node(
        package='crazyflie_controllers',
        executable='attitude_pid_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    simulation = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([get_package_share_directory(
            'crazyflie_ros2_driver') + '/launch/crazyflie_webots_driver.launch.py']))

    joystick_driver = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([get_package_share_directory(
                'crazyflie_teleop') + '/launch/joystick.launch.py']),
        condition=IfCondition(LaunchConfiguration('use_joy')))

    joystick_to_attitude = Node(
        package='crazyflie_teleop',
        executable='joystick_to_attitude',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joy'))
    )

    return LaunchDescription([
        use_joy_param,
        simulation,
        controller,
        joystick_to_attitude,
        joystick_driver
    ])
