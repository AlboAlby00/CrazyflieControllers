from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():

    package_dir = get_package_share_directory('crazyflie_localization')

    simulation = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('crazyflie_ros2_driver') + '/launch/crazyflie_webots_driver.launch.py'
        ]),
        launch_arguments={
            'simulation_world': 'complete_apartment.wbt',
        }.items()
    )

    joystick = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource([
            get_package_share_directory('crazyflie_teleop') + '/launch/joystick.launch.py']))
    
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
        arguments=['-d', package_dir+"/rviz/visual_odometry.rviz"]
    )

    intrinsics = os.path.join(
        get_package_share_directory('crazyflie_localization'),
        'config',
        'webots_camera_intrinsics.yaml'
        )

    visual_odometry_node = Node(
        package='crazyflie_localization',
        executable='visual_odometry_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[intrinsics]
    )

    tf_broadcaster_node = Node(
        package="crazyflie_localization",
        executable="tf_broadcaster_node",
        name="tf_broadcaster_node",
        output='screen',
    )

    return LaunchDescription([
        simulation,
        rviz_node,
        visual_odometry_node,
        tf_broadcaster_node,
        joystick,
        attitude_controller
    ])
