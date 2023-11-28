from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    joystick_to_attitude_node = Node(
        package='crazyflie_teleop',
        executable='joystick_to_attitude',
        output='screen'
    )

    joystick_node = IncludeLaunchDescription(launch_description_source =
        PythonLaunchDescriptionSource([get_package_share_directory(
            'teleop_twist_joy') + '/launch/teleop-launch.py']))

<<<<<<< HEAD
<<<<<<< HEAD
    return LaunchDescription([
        joystick_to_attitude_node,
        joystick_node
=======


    return LaunchDescription([
        joystick_to_attitude_node,
        joystick_node

>>>>>>> 0d9b7e9efb5d1d9244a28c6c8e5649b79eaf1379
=======
    return LaunchDescription([
        joystick_to_attitude_node,
        joystick_node
>>>>>>> main
    ])