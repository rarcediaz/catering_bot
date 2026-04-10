import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'my_bot'
    use_joystick = LaunchConfiguration('use_joystick')
    use_battery_monitor = LaunchConfiguration('use_battery_monitor')
    use_stop_debug_monitor = LaunchConfiguration('use_stop_debug_monitor')
    stop_debug_log_path = LaunchConfiguration('stop_debug_log_path')
    stop_debug_log_hz = LaunchConfiguration('stop_debug_log_hz')
    obstacle_stop_distance_m = LaunchConfiguration('obstacle_stop_distance_m')
    obstacle_stop_distance_max_m = LaunchConfiguration('obstacle_stop_distance_max_m')
    obstacle_stop_speed_mps = LaunchConfiguration('obstacle_stop_speed_mps')
    obstacle_slowdown_margin_m = LaunchConfiguration('obstacle_slowdown_margin_m')
    front_stop_start_x_m = LaunchConfiguration('front_stop_start_x_m')
    front_stop_width_m = LaunchConfiguration('front_stop_width_m')

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'true'
        }.items()
    )

    # JoyStick


    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]),
                condition=IfCondition(use_joystick)
    )

    battery_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'safety.launch.py'
            )
        ),
        launch_arguments={
            'obstacle_stop_distance_m': obstacle_stop_distance_m,
            'obstacle_stop_distance_max_m': obstacle_stop_distance_max_m,
            'obstacle_stop_speed_mps': obstacle_stop_speed_mps,
            'obstacle_slowdown_margin_m': obstacle_slowdown_margin_m,
            'front_stop_start_x_m': front_stop_start_x_m,
            'front_stop_width_m': front_stop_width_m,
        }.items(),
        condition=IfCondition(use_battery_monitor)
    )

    stop_debug_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'stop_debug.launch.py'
            )
        ),
        launch_arguments={
            'log_path': stop_debug_log_path,
            'log_hz': stop_debug_log_hz,
        }.items(),
        condition=IfCondition(use_stop_debug_monitor)
    )


    # YDLidar Launch
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'ydlidar.launch.py'
            )
        )
    )

    # Controllers
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params_file],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen'
    )

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Twist mux
    twist_mux_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )

    scan_filter_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'scan_filter.yaml'
    )

    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter_chain',
        output='screen',
        parameters=[scan_filter_config],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ]
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
        parameters=[
            {'use_sim_time': False},
            twist_mux_config
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'use_joystick',
            default_value='false',
            description='Launch local joystick teleop on this machine if true.'
        ),
        DeclareLaunchArgument(
            'use_battery_monitor',
            default_value='true',
            description='Launch the battery and safety monitoring node if true.'
        ),
        DeclareLaunchArgument(
            'use_stop_debug_monitor',
            default_value='false',
            description='Launch the stop debug monitor if true.'
        ),
        DeclareLaunchArgument(
            'stop_debug_log_path',
            default_value='~/stop_debug_log.csv',
            description='CSV file path for the stop debug monitor.'
        ),
        DeclareLaunchArgument(
            'stop_debug_log_hz',
            default_value='5.0',
            description='Stop debug logging frequency in Hz.'
        ),
        DeclareLaunchArgument(
            'obstacle_stop_distance_m',
            default_value='0.50',
            description='Stop if an obstacle is within this forward distance in meters.'
        ),
        DeclareLaunchArgument(
            'obstacle_stop_distance_max_m',
            default_value='0.60',
            description='Maximum stop distance used at higher forward speeds.'
        ),
        DeclareLaunchArgument(
            'obstacle_stop_speed_mps',
            default_value='0.60',
            description='Forward speed that maps to the maximum stop distance.'
        ),
        DeclareLaunchArgument(
            'obstacle_slowdown_margin_m',
            default_value='0.15',
            description='Additional distance ahead of the stop threshold where forward speed is scaled down.'
        ),
        DeclareLaunchArgument(
            'front_stop_start_x_m',
            default_value='0.0508',
            description='Distance from lidar to the robot front edge in meters.'
        ),
        DeclareLaunchArgument(
            'front_stop_width_m',
            default_value='0.8596',
            description='Width of the forward stop corridor in meters.'
        ),
        rsp,
        joystick,
        battery_monitor,
        stop_debug_monitor,
        ydlidar,
        scan_filter,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        twist_mux,
    ])