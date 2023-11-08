from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():

    package_dir = get_package_share_directory('crazyflie_localization')

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
        'esp32_intrinsics.yaml'
        )

    visual_odometry_node = Node(
        package='crazyflie_localization',
        executable='visual_odometry_node',
        name="visual_odometry_node",
        output='screen',
        parameters=[intrinsics]
    )

    esp_32_driver = Node(
        package="crazyflie_ros2_driver",
        executable="esp32_driver",
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[("esp_32/camera", "crazyflie/camera")]
    )

    return LaunchDescription([
        rviz_node,
        esp_32_driver,
        visual_odometry_node
    ])
