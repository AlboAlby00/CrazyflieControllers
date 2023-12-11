from launch_ros.actions import Node
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory 
from launch.actions import DeclareLaunchArgument
# import LaunchConfiguration
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    target_publisher = Node(
        package='crazyflie_detectors',
        executable='sim_target_publisher.py',
        output='screen'
    )


    simulation = IncludeLaunchDescription(launch_description_source=
        PythonLaunchDescriptionSource([get_package_share_directory(
            'crazyflie_ros2_driver') + '/launch/crazyflie_ros2_driver.launch.py']),
        launch_arguments={
            'simulation_world': 'crazyflie_arena_atag.wbt',
        }.items()
    )

    return LaunchDescription([
        simulation,
        target_publisher
    ])