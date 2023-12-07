from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():

    localization_dir = get_package_share_directory('crazyflie_localization')
    teleop_dir = get_package_share_directory('crazyflie_teleop')
    driver_dir = get_package_share_directory('crazyflie_ros2_driver')
    orb3_dir = get_package_share_directory('orbslam3')

    simulation = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            driver_dir + '/launch/crazyflie_webots_driver.launch.py'
        ]),
        launch_arguments={
            'simulation_world': 'complete_apartment.wbt',
        }.items()
    )

    joystick = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            teleop_dir + '/launch/joystick.launch.py']))

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
        arguments=['-d', localization_dir+"/rviz/visual_odometry.rviz"]
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
        simulation,
        rviz_node,
        orbslam3_odometry_node,
        joystick,
        attitude_controller
    ])
