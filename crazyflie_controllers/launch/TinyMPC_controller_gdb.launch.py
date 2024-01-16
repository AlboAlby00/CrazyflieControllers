from launch_ros.actions import Node
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python import get_package_share_directory


def generate_launch_description():

    tinyMPC_controller = Node(
        package='crazyflie_controllers',
        executable='TinyMPC_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )



    simulation = IncludeLaunchDescription(launch_description_source=
                                          PythonLaunchDescriptionSource([get_package_share_directory(
                                              'crazyflie_ros2_driver') + '/launch/crazyflie_webots_driver.launch.py']))

    return LaunchDescription([
        simulation,
        tinyMPC_controller,
    ])