import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'my_bot'
    robot_id = LaunchConfiguration('robot_id')
    mission_control_url = LaunchConfiguration('mission_control_url')
    use_heartbeat = LaunchConfiguration('use_heartbeat')
    use_safety_node = LaunchConfiguration('use_safety_node')
    obstacle_stop_distance_m = LaunchConfiguration('obstacle_stop_distance_m')
    obstacle_stop_distance_max_m = LaunchConfiguration('obstacle_stop_distance_max_m')
    obstacle_stop_speed_mps = LaunchConfiguration('obstacle_stop_speed_mps')
    obstacle_slowdown_margin_m = LaunchConfiguration('obstacle_slowdown_margin_m')
    front_stop_start_x_m = LaunchConfiguration('front_stop_start_x_m')
    rear_stop_start_x_m = LaunchConfiguration('rear_stop_start_x_m')
    front_stop_width_m = LaunchConfiguration('front_stop_width_m')
    side_stop_distance_m = LaunchConfiguration('side_stop_distance_m')
    side_stop_start_y_m = LaunchConfiguration('side_stop_start_y_m')
    nav_stop_hold_sec = LaunchConfiguration('nav_stop_hold_sec')

    robot_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'launch_robot.launch.py'
            )
        ),
        launch_arguments={
            'use_joystick': 'false',
            'robot_id': robot_id,
            'mission_control_url': mission_control_url,
            'use_heartbeat': use_heartbeat,
            'use_safety_node': use_safety_node,
            'obstacle_stop_distance_m': obstacle_stop_distance_m,
            'obstacle_stop_distance_max_m': obstacle_stop_distance_max_m,
            'obstacle_stop_speed_mps': obstacle_stop_speed_mps,
            'obstacle_slowdown_margin_m': obstacle_slowdown_margin_m,
            'front_stop_start_x_m': front_stop_start_x_m,
            'rear_stop_start_x_m': rear_stop_start_x_m,
            'front_stop_width_m': front_stop_width_m,
            'side_stop_distance_m': side_stop_distance_m,
            'side_stop_start_y_m': side_stop_start_y_m,
            'nav_stop_hold_sec': nav_stop_hold_sec,
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'robot_id',
            default_value='IntelliTrolley-01',
            description='Stable robot identity shown in Mission Control.'
        ),
        DeclareLaunchArgument(
            'mission_control_url',
            default_value='http://127.0.0.1:8000',
            description='Mission Control server base URL used by the heartbeat node.'
        ),
        DeclareLaunchArgument(
            'use_heartbeat',
            default_value='true',
            description='Send periodic robot telemetry to the mission control server.'
        ),
        DeclareLaunchArgument(
            'use_safety_node',
            default_value='true',
            description='Launch the obstacle safety node on the robot computer.'
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
            'rear_stop_start_x_m',
            default_value='1.016',
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
            description='Block left/right turns when an obstacle is within this side distance in meters.'
        ),
        DeclareLaunchArgument(
            'side_stop_start_y_m',
            default_value='0.34',
            description='Distance from lidar centerline to the robot side edge in meters.'
        ),
        DeclareLaunchArgument(
            'nav_stop_hold_sec',
            default_value='0.50',
            description='High-priority zero-command hold time after Nav2 commands stop.'
        ),
        robot_base,
    ])
