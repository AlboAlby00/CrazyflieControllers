from launch_ros.actions import Node
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory 



def generate_launch_description():

    controller = Node(
        package='crazyflie_controllers',
        executable='test_controller.py',
        output='screen'
    )

    simulation = IncludeLaunchDescription(launch_description_source=
        PythonLaunchDescriptionSource([get_package_share_directory(
            'crazyflie_ros2_driver') + '/launch/crazyflie_ros2_driver.launch.py']))

    return LaunchDescription([
        simulation,
        controller
    ])