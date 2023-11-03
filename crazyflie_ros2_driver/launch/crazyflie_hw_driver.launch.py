import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix

def generate_launch_description():

    target_publisher = Node(
        package='crazyflie_ros2_driver',
        executable='crazyflie_hw_driver',
        arguments=['--ros-args', '--log-level', 'info']
    )


    return LaunchDescription([
        target_publisher
    ])
