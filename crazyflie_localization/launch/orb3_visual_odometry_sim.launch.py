from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():

    localization_dir = get_package_share_directory('crazyflie_localization')
    orb3_dir = get_package_share_directory('orbslam3')

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
        executable='mono',
        output='screen',
        arguments=[vocabulary_path, camera_info_path],
        remappings=[('/camera', '/crazyflie/camera'),
                    ('/imu', '/crazyflie/imu')]
    )

    return LaunchDescription([
        rviz_node,
        orbslam3_odometry_node
    ])
