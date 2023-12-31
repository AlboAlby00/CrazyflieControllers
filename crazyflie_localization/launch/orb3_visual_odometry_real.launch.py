from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    localization_dir = get_package_share_directory('crazyflie_localization')
    orb3_dir = get_package_share_directory('orbslam3')

    conf_orb_mode = LaunchConfiguration("orb_mode", default="mono")
    esp_ip_conf = LaunchConfiguration("ip", default="192.168.45.169")

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
        localization_dir, 'config', 'orb3_esp32_640_480_camera.yaml')

    orbslam3_odometry_node = Node(
        package='orbslam3',
        executable=conf_orb_mode,
        output='screen',
        arguments=[vocabulary_path, camera_info_path],
        remappings=[('/camera', '/crazyflie/camera'),
                    ('/imu', '/crazyflie/imu')]
    )

    esp_32_driver = Node(
        package="crazyflie_ros2_driver",
        executable="esp32_driver",
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[("esp_32/camera", "crazyflie/camera")],
        parameters=[
            {"ip": esp_ip_conf}
        ]
    )

    camera_aligner = Node(
        package="crazyflie_localization",
        executable="camera_aligner.py",
        parameters=[{
            "sim_camera": False
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('orb_mode', default_value=conf_orb_mode, choices=['mono', 'mono-inertial']),
        DeclareLaunchArgument("ip", default_value=esp_ip_conf),
        rviz_node,
        orbslam3_odometry_node,
        esp_32_driver,
        camera_aligner
    ])
