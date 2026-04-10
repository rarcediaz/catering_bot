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
    front_stop_width_m = LaunchConfiguration('front_stop_width_m')
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
            'front_stop_width_m': front_stop_width_m,
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
            default_value='0.0508',
            description='Distance from lidar to the robot front edge in meters.'
        ),
        DeclareLaunchArgument(
            'front_stop_width_m',
            default_value='0.8596',
            description='Width of the forward stop corridor in meters.'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_filtered',
            description='LaserScan topic consumed by the safety node.'
        ),
        safety_node,
    ])