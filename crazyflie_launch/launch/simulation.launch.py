from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    localization_dir = get_package_share_directory('crazyflie_localization')
    controllers_dir = get_package_share_directory('crazyflie_controllers')
    driver_dir = get_package_share_directory('crazyflie_ros2_driver')

    world_conf = LaunchConfiguration("world", default="complete_apartment.wbt")
    use_joy_conf = LaunchConfiguration("use_joy", default="false")

    simulation = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            driver_dir + '/launch/crazyflie_webots_driver.launch.py'
        ]),
        launch_arguments=[
            ('simulation_world', LaunchConfiguration('world'))
        ]
    )

    orbslam = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            localization_dir + '/launch/orb3_visual_odometry_sim.launch.py'
        ])
    )

    joystick = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            controllers_dir + '/launch/joystick.launch.py']),
        condition=IfCondition(LaunchConfiguration('use_joy'))
    )

    position_controller = Node(
        package='crazyflie_controllers',
        executable='position_pid_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        condition=UnlessCondition(LaunchConfiguration('use_joy'))
    )

    attitude_controller = Node(
        package='crazyflie_controllers',
        executable='attitude_pid_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    rqt_gui = Node( 
        package='rqt_gui',
        executable='rqt_gui',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_joy'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_conf),
        DeclareLaunchArgument('use_joy', default_value=use_joy_conf),
        simulation,
        orbslam,
        joystick,
        position_controller,
        attitude_controller,
        rqt_gui
    ])
