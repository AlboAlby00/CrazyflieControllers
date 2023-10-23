from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    simulation = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('crazyflie_ros2_driver') + '/launch/crazyflie_ros2_driver.launch.py'
        ]),
        launch_arguments={
            'simulation_world': 'complete_apartment.wbt',
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    visual_odometry_node = Node(
        package='crazyflie_localization',
        executable='visual_odomerty_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        simulation,
        rviz_node,
        visual_odometry_node
    ])