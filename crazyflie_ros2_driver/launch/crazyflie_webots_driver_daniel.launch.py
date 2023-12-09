import os
import pathlib
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('crazyflie_ros2_driver')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'crazyflie.urdf')).read_text()

    simulation_world_arg = DeclareLaunchArgument('simulation_world', default_value='crazyflie_arena.wbt')
    simulation_world_arg = DeclareLaunchArgument('simulation_world_daniel', default_value='crazyflie_arena_daniel.wbt')

    webots = WebotsLauncher(
        world = PathJoinSubstitution([package_dir,"worlds",LaunchConfiguration("simulation_world")])
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'crazyflie'},
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return LaunchDescription([
        simulation_world_arg,
        webots,
        my_robot_driver,
        ros2_supervisor,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
