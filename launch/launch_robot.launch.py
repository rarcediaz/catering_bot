import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'my_bot'
    package_share = get_package_share_directory(package_name)
    cyclonedds_uri = f'file://{os.path.join(package_share, "config", "cyclonedds.xml")}'
    use_joystick = LaunchConfiguration('use_joystick')
    obstacle_stop_distance_m = LaunchConfiguration('obstacle_stop_distance_m')
    obstacle_stop_distance_max_m = LaunchConfiguration('obstacle_stop_distance_max_m')
    obstacle_stop_speed_mps = LaunchConfiguration('obstacle_stop_speed_mps')
    obstacle_slowdown_margin_m = LaunchConfiguration('obstacle_slowdown_margin_m')
    front_stop_start_x_m = LaunchConfiguration('front_stop_start_x_m')
    rear_stop_start_x_m = LaunchConfiguration('rear_stop_start_x_m')
    front_stop_width_m = LaunchConfiguration('front_stop_width_m')
    front_obstacle_confirmation_scans = LaunchConfiguration(
        'front_obstacle_confirmation_scans'
    )
    front_obstacle_pending_speed_mps = LaunchConfiguration(
        'front_obstacle_pending_speed_mps'
    )
    rear_obstacle_confirmation_scans = LaunchConfiguration(
        'rear_obstacle_confirmation_scans'
    )
    rear_obstacle_pending_speed_mps = LaunchConfiguration(
        'rear_obstacle_pending_speed_mps'
    )
    side_stop_distance_m = LaunchConfiguration('side_stop_distance_m')
    side_hard_stop_distance_m = LaunchConfiguration('side_hard_stop_distance_m')
    side_min_speed_scale = LaunchConfiguration('side_min_speed_scale')
    side_obstacle_confirmation_scans = LaunchConfiguration(
        'side_obstacle_confirmation_scans'
    )
    side_obstacle_pending_angular_rps = LaunchConfiguration(
        'side_obstacle_pending_angular_rps'
    )
    side_stop_start_y_m = LaunchConfiguration('side_stop_start_y_m')
    turn_in_place_linear_threshold_mps = LaunchConfiguration(
        'turn_in_place_linear_threshold_mps'
    )
    turn_in_place_angular_threshold_radps = LaunchConfiguration(
        'turn_in_place_angular_threshold_radps'
    )
    nav_timeout_sec = LaunchConfiguration('nav_timeout_sec')
    nav_stop_hold_sec = LaunchConfiguration('nav_stop_hold_sec')
    scan_timeout_sec = LaunchConfiguration('scan_timeout_sec')
    startup_quiet_sec = LaunchConfiguration('startup_quiet_sec')
    motor_device = LaunchConfiguration('motor_device')
    lidar_device = LaunchConfiguration('lidar_device')

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
            'use_ros2_control': 'true',
            'motor_device': motor_device,
        }.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'joystick.launch.py')
        ),
        condition=IfCondition(use_joystick)
    )

    safety_node = IncludeLaunchDescription(
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
            'rear_stop_start_x_m': rear_stop_start_x_m,
            'front_stop_width_m': front_stop_width_m,
            'front_obstacle_confirmation_scans': front_obstacle_confirmation_scans,
            'front_obstacle_pending_speed_mps': front_obstacle_pending_speed_mps,
            'rear_obstacle_confirmation_scans': rear_obstacle_confirmation_scans,
            'rear_obstacle_pending_speed_mps': rear_obstacle_pending_speed_mps,
            'side_stop_distance_m': side_stop_distance_m,
            'side_hard_stop_distance_m': side_hard_stop_distance_m,
            'side_min_speed_scale': side_min_speed_scale,
            'side_obstacle_confirmation_scans': side_obstacle_confirmation_scans,
            'side_obstacle_pending_angular_rps': side_obstacle_pending_angular_rps,
            'side_stop_start_y_m': side_stop_start_y_m,
            'turn_in_place_linear_threshold_mps': turn_in_place_linear_threshold_mps,
            'turn_in_place_angular_threshold_radps': turn_in_place_angular_threshold_radps,
            'nav_timeout_sec': nav_timeout_sec,
            'nav_stop_hold_sec': nav_stop_hold_sec,
            'scan_timeout_sec': scan_timeout_sec,
            'startup_quiet_sec': startup_quiet_sec,
        }.items(),
    )
    # YDLidar Launch
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'ydlidar.launch.py'
            )
        ),
        launch_arguments={
            'port': lidar_device,
        }.items(),
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
        output='screen',
    )

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_cont",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--service-call-timeout", "60",
            "--switch-timeout", "60",
        ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_broad",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--service-call-timeout", "60",
            "--switch-timeout", "60",
        ],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Do not issue concurrent load requests while ros2_control is still
    # initializing the serial hardware. On a busy Pi, diff_cont previously
    # timed out, completed loading in the background, then failed its retry as
    # "already loaded" and remained unconfigured forever. Finish and activate
    # the joint-state broadcaster first, then load diff_cont with service and
    # switch timeouts long enough for a cold boot.
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    controller_manager_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager,
            on_exit=[
                LogInfo(
                    msg=(
                        'controller_manager exited; shutting down the hardware '
                        'launch for a clean systemd retry.'
                    )
                ),
                EmitEvent(
                    event=Shutdown(reason='controller_manager exited')
                ),
            ],
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

    # The YDLidar driver publishes both -pi and +pi as readings. Those are the
    # same ray, and Karto otherwise registers this as a non-360-degree laser.
    scan_canonicalizer = Node(
        package=package_name,
        executable='scan_canonicalizer.py',
        name='scan_canonicalizer',
        output='screen',
        remappings=[
            ('scan', '/scan'),
            ('scan_canonical', '/scan_canonical'),
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter_chain',
        output='screen',
        parameters=[scan_filter_config],
        remappings=[
            ('scan', '/scan_canonical'),
            ('scan_filtered', '/scan_filtered'),
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
        parameters=[
            {'use_sim_time': False},
            twist_mux_config
        ],
        respawn=True,
        respawn_delay=1.0,
    )

    robot_health = Node(
        package=package_name,
        executable='robot_health_node.py',
        name='robot_health_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI',
            EnvironmentVariable('CYCLONEDDS_URI', default_value=cyclonedds_uri),
        ),
        DeclareLaunchArgument(
            'use_joystick',
            default_value='false',
            description='Launch local joystick teleop on this machine if true.'
        ),
        DeclareLaunchArgument(
            'obstacle_stop_distance_m',
            default_value='0.25',
            description=(
                'Immediate Pi-local hard-stop clearance in meters; motion is '
                'progressively slowed before reaching this boundary.'
            )
        ),
        DeclareLaunchArgument(
            'obstacle_stop_distance_max_m',
            default_value='0.60',
            description='Clearance used to derive the progressive speed limit.'
        ),
        DeclareLaunchArgument(
            'obstacle_stop_speed_mps',
            default_value='0.60',
            description='Forward speed that maps to the maximum stop distance.'
        ),
        DeclareLaunchArgument(
            'obstacle_slowdown_margin_m',
            default_value='0.15',
            description=(
                'Additional distance ahead of the stop threshold where forward '
                'speed is scaled down.'
            )
        ),
        DeclareLaunchArgument(
            'front_stop_start_x_m',
            default_value='0.12',
            description='Distance from lidar to the robot front edge in meters.'
        ),
        DeclareLaunchArgument(
            'rear_stop_start_x_m',
            default_value='0.88',
            description='Distance from lidar to the robot rear edge in meters.'
        ),
        DeclareLaunchArgument(
            'front_stop_width_m',
            default_value='0.8596',
            description='Width of the forward stop corridor in meters.'
        ),
        DeclareLaunchArgument(
            'front_obstacle_confirmation_scans',
            default_value='3',
            description=(
                'Consecutive filtered scans required to confirm a front hard stop.'
            )
        ),
        DeclareLaunchArgument(
            'front_obstacle_pending_speed_mps',
            default_value='0.10',
            description=(
                'Forward crawl cap while the first front detection awaits confirmation.'
            )
        ),
        DeclareLaunchArgument(
            'rear_obstacle_confirmation_scans',
            default_value='3',
            description=(
                'Consecutive filtered scans required to confirm a rear hard stop.'
            )
        ),
        DeclareLaunchArgument(
            'rear_obstacle_pending_speed_mps',
            default_value='0.10',
            description=(
                'Reverse crawl cap while a rear detection awaits confirmation.'
            )
        ),
        DeclareLaunchArgument(
            'side_stop_distance_m',
            default_value='0.25',
            description=(
                'Begin progressively slowing turns toward an obstacle within '
                'this side clearance in meters.'
            )
        ),
        DeclareLaunchArgument(
            'side_hard_stop_distance_m',
            default_value='0.08',
            description=(
                'Stop a turn toward an obstacle only inside this imminent '
                'side-collision clearance in meters.'
            )
        ),
        DeclareLaunchArgument(
            'side_min_speed_scale',
            default_value='0.25',
            description=(
                'Minimum progressive turn scale immediately outside the side '
                'hard-stop boundary.'
            )
        ),
        DeclareLaunchArgument(
            'side_obstacle_confirmation_scans',
            default_value='2',
            description=(
                'Consecutive filtered scans required to confirm a side obstacle.'
            )
        ),
        DeclareLaunchArgument(
            'side_obstacle_pending_angular_rps',
            default_value='0.15',
            description=(
                'Turn-speed cap while the first side detection awaits confirmation.'
            )
        ),
        DeclareLaunchArgument(
            'side_stop_start_y_m',
            default_value='0.34',
            description='Distance from lidar centerline to the robot side edge in meters.'
        ),
        DeclareLaunchArgument(
            'turn_in_place_linear_threshold_mps',
            default_value='0.05',
            description=(
                'Maximum Nav2 linear speed converted to zero during a '
                'rotation-dominant command.'
            )
        ),
        DeclareLaunchArgument(
            'turn_in_place_angular_threshold_radps',
            default_value='0.20',
            description=(
                'Minimum Nav2 angular speed required to normalize a '
                'near-zero-linear command as an in-place turn.'
            )
        ),
        DeclareLaunchArgument(
            'nav_timeout_sec',
            default_value='0.50',
            description=(
                'Keep the latest navigation command through a short DDS/Wi-Fi '
                'gap while fresh Pi-local lidar safety remains active.'
            )
        ),
        DeclareLaunchArgument(
            'nav_stop_hold_sec',
            default_value='0.50',
            description=(
                'High-priority zero-command hold after a moving navigation '
                'stream is lost, not after an intentional fresh zero.'
            )
        ),
        DeclareLaunchArgument(
            'scan_timeout_sec',
            default_value='0.50',
            description='Block all motion when filtered scans are older than this.'
        ),
        DeclareLaunchArgument(
            'startup_quiet_sec',
            default_value='5.0',
            description=(
                'Seconds of quiet raw motion commands required before the '
                'startup gate opens automatically.'
            )
        ),
        DeclareLaunchArgument(
            'motor_device',
            default_value=EnvironmentVariable(
                'ROBOT_MOTOR_DEVICE',
                default_value='/dev/ttyACM0',
            ),
            description='Arduino serial device; prefer /dev/serial/by-id/... on the Pi.'
        ),
        DeclareLaunchArgument(
            'lidar_device',
            default_value=EnvironmentVariable(
                'ROBOT_LIDAR_DEVICE',
                default_value='/dev/ttyUSB0',
            ),
            description='YDLidar serial device; prefer /dev/serial/by-id/... on the Pi.'
        ),
        rsp,
        joystick,
        safety_node,
        ydlidar,
        scan_canonicalizer,
        scan_filter,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        controller_manager_exit,
        twist_mux,
        robot_health,
    ])
