from launch_ros.actions import Node
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    target_publisher = Node(
        package='crazyflie_detectors',
        executable='target_publisher.py',
        output='screen'
    )


    simulation = IncludeLaunchDescription(launch_description_source=
        PythonLaunchDescriptionSource([get_package_share_directory(
            'crazyflie_ros2_driver') + '/launch/crazyflie_ros2_driver.launch.py']),
    )

    return LaunchDescription([
        simulation,
        target_publisher
    ])