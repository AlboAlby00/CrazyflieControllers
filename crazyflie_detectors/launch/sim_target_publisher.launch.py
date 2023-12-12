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
        executable='sim_target_publisher.py',
        output='screen'
    )

    attitude_controller = Node(
        package='crazyflie_controllers',
        executable='attitude_pid_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        #arguments=['-d', package_dir+"/config/camera.rviz"]
    )

    simulation = IncludeLaunchDescription(launch_description_source=
        PythonLaunchDescriptionSource([get_package_share_directory(
            'crazyflie_ros2_driver') + '/launch/crazyflie_webots_driver.launch.py']),
        launch_arguments={
            'simulation_world': 'apriltag_apartment.wbt'
        }.items()
    )

    return LaunchDescription([
        simulation,
        attitude_controller,
        rviz_node,
        target_publisher
    ])