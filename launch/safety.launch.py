from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    obstacle_stop_distance_m = LaunchConfiguration('obstacle_stop_distance_m')
    obstacle_stop_distance_max_m = LaunchConfiguration('obstacle_stop_distance_max_m')
    obstacle_stop_speed_mps = LaunchConfiguration('obstacle_stop_speed_mps')
    obstacle_slowdown_margin_m = LaunchConfiguration('obstacle_slowdown_margin_m')
    front_stop_start_x_m = LaunchConfiguration('front_stop_start_x_m')
    rear_stop_start_x_m = LaunchConfiguration('rear_stop_start_x_m')
    front_stop_width_m = LaunchConfiguration('front_stop_width_m')
    side_stop_distance_m = LaunchConfiguration('side_stop_distance_m')
    side_hard_stop_distance_m = LaunchConfiguration('side_hard_stop_distance_m')
    side_min_speed_scale = LaunchConfiguration('side_min_speed_scale')
    side_stop_start_y_m = LaunchConfiguration('side_stop_start_y_m')
    nav_timeout_sec = LaunchConfiguration('nav_timeout_sec')
    nav_stop_hold_sec = LaunchConfiguration('nav_stop_hold_sec')
    scan_timeout_sec = LaunchConfiguration('scan_timeout_sec')
    startup_quiet_sec = LaunchConfiguration('startup_quiet_sec')
    scan_topic = LaunchConfiguration('scan_topic')

    safety_node = Node(
        package='my_bot',
        executable='safety_node.py',
        output='screen',
        parameters=[{
            'obstacle_stop_enabled': True,
            'obstacle_stop_distance_m': obstacle_stop_distance_m,
            'obstacle_stop_distance_max_m': obstacle_stop_distance_max_m,
            'obstacle_stop_speed_mps': obstacle_stop_speed_mps,
            'obstacle_slowdown_margin_m': obstacle_slowdown_margin_m,
            'front_stop_start_x_m': front_stop_start_x_m,
            'rear_stop_start_x_m': rear_stop_start_x_m,
            'front_stop_width_m': front_stop_width_m,
            'side_stop_distance_m': side_stop_distance_m,
            'side_hard_stop_distance_m': side_hard_stop_distance_m,
            'side_min_speed_scale': side_min_speed_scale,
            'side_stop_start_y_m': side_stop_start_y_m,
            'nav_timeout_sec': nav_timeout_sec,
            'nav_stop_hold_sec': nav_stop_hold_sec,
            'scan_timeout_sec': scan_timeout_sec,
            'startup_quiet_sec': startup_quiet_sec,
        }],
        remappings=[
            ('/scan', scan_topic),
        ],
        respawn=True,
        respawn_delay=1.0,
    )

    return LaunchDescription([
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
            default_value='0.09',
            description='Distance from lidar to the robot front edge in meters.'
        ),
        DeclareLaunchArgument(
            'rear_stop_start_x_m',
            default_value='0.91',
            description='Distance from lidar to the robot rear edge in meters.'
        ),
        DeclareLaunchArgument(
            'front_stop_width_m',
            default_value='0.8596',
            description='Width of the forward stop corridor in meters.'
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
            'side_stop_start_y_m',
            default_value='0.34',
            description='Distance from lidar centerline to the robot side edge in meters.'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_filtered',
            description='LaserScan topic consumed by the safety node.'
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
                'Require this many seconds without an active raw motion command '
                'before the startup safety gate opens automatically.'
            )
        ),
        safety_node,
    ])
