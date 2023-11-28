from launch_ros.actions import Node
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument

import os


def generate_launch_description():

    localization_dir = get_package_share_directory('crazyflie_localization')
    driver_dir = get_package_share_directory('crazyflie_ros2_driver')
    orb3_dir = get_package_share_directory('orbslam3')

    use_simulation_param = DeclareLaunchArgument("mode", default_value="sim", choices=["sim", "real"])

    position_controller = Node(
        package='crazyflie_controllers',
        executable='position_pid_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    attitude_controller = Node(
        package='crazyflie_controllers',
        executable='attitude_pid_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        condition=LaunchConfigurationEquals('mode', 'sim')
    )

    simulation = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            driver_dir + '/launch/crazyflie_webots_driver.launch.py']),
        launch_arguments=[("simulation_world", "complete_apartment.wbt")],
        condition=LaunchConfigurationEquals('mode', 'sim')
    )

    vocabulary_path = os.path.join(
        orb3_dir, 'vocabulary', 'ORBvoc.txt')
    camera_info_path = os.path.join(
        localization_dir, 'config', 'orb3_webots_camera.yaml')

    orbslam3_odometry_node = Node(
        package='orbslam3',
        executable='mono-inertial',
        output='screen',
        arguments=[vocabulary_path, camera_info_path],
        remappings=[('/camera', '/crazyflie/camera'),
                    ('/imu', '/crazyflie/imu'),
                    ('/camera_position', '/crazyflie/camera_position')]
    )

    return LaunchDescription([
        use_simulation_param,
        simulation,
        attitude_controller,
        position_controller,
        orbslam3_odometry_node
    ])
