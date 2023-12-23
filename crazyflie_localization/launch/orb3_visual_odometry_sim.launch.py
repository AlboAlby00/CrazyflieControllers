from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os


def generate_launch_description():

    localization_dir = get_package_share_directory('crazyflie_localization')
    orb3_dir = get_package_share_directory('orbslam3')

    conf_orb_mode = LaunchConfiguration("orb_mode", default="mono")
    ekf_conf = LaunchConfiguration("ekf", default="true")

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', localization_dir+"/rviz/visual_odometry.rviz"]
    )

    ekf_wrapper_node = Node(
        package="crazyflie_localization",
        executable="ekf_wrapper.py",
        output='screen',
        condition=IfCondition(ekf_conf)
    )

    full_ekf_params = [localization_dir, "/config/ekf.yaml"]
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output="screen",
        emulate_tty=True,
        remappings=[
            ("odometry/filtered", "odometry/filtered/local"),
            ("set_pose", "ekf/set_pose"),  # Useful for debugging
        ],
        parameters=[full_ekf_params],
        condition=IfCondition(ekf_conf)
    )

    vocabulary_path = os.path.join(
        orb3_dir, 'vocabulary', 'ORBvoc.txt')
    camera_info_path = os.path.join(
        localization_dir, 'config', 'orb3_webots_camera.yaml')

    orbslam3_odometry_node = Node(
        package='orbslam3',
        executable=conf_orb_mode,
        output='screen',
        arguments=[vocabulary_path, camera_info_path],
        remappings=[('/camera', '/crazyflie/camera'),
                    ('/imu', '/crazyflie/imu')]
    )

    camera_aligner = Node(
        package="crazyflie_localization",
        executable="camera_aligner.py",
        parameters=[{
            "sim_camera": True
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('orb_mode', default_value=conf_orb_mode, choices=['mono', 'mono-inertial']),
        DeclareLaunchArgument('ekf', default_value=ekf_conf),
        rviz_node,
        orbslam3_odometry_node,
        camera_aligner,
        ekf_node,
        ekf_wrapper_node
    ])
