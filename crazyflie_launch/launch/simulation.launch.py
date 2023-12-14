from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals


def generate_launch_description():

    localization_dir = get_package_share_directory('crazyflie_localization')
    controllers_dir = get_package_share_directory('crazyflie_controllers')
    driver_dir = get_package_share_directory('crazyflie_ros2_driver')

    world_conf = LaunchConfiguration("world", default="complete_apartment.wbt")
    use_joy_conf = LaunchConfiguration("use_joy", default="false")
    state_est_conf = LaunchConfiguration("state_est", default="camera")
    orb_mode_conf = LaunchConfiguration("orb_mode", default="mono")

    simulation = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            driver_dir + '/launch/crazyflie_webots_driver.launch.py'
        ]),
        launch_arguments=[
            ('simulation_world', world_conf)
        ]
    )

    orbslam = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            localization_dir + '/launch/orb3_visual_odometry_sim.launch.py'
        ]),
        condition=LaunchConfigurationEquals('state_est', 'camera'),
        launch_arguments=[
            ('orb_mode', orb_mode_conf),
        ]
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
        parameters=[{'state_est': state_est_conf}],
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
        DeclareLaunchArgument('state_est', default_value=state_est_conf, choices=["camera", "gps"]),
        DeclareLaunchArgument('orb_mode', default_value=orb_mode_conf, choices=["mono", "mono-inertial"]),
        simulation,
        orbslam,
        joystick,
        position_controller,
        attitude_controller,
        rqt_gui
    ])
