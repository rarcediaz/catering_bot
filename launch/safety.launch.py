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
    side_stop_start_y_m = LaunchConfiguration('side_stop_start_y_m')
    nav_stop_hold_sec = LaunchConfiguration('nav_stop_hold_sec')
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
            'side_stop_start_y_m': side_stop_start_y_m,
            'nav_stop_hold_sec': nav_stop_hold_sec,
        }],
        remappings=[
            ('/scan', scan_topic),
        ],
    )

    return LaunchDescription([
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
            description='Block left/right turns when an obstacle is within this side distance in meters.'
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
            'nav_stop_hold_sec',
            default_value='0.50',
            description='High-priority zero-command hold time after Nav2 commands stop.'
        ),
        safety_node,
    ])
