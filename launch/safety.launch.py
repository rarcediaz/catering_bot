from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    obstacle_stop_distance_m = LaunchConfiguration('obstacle_stop_distance_m')
    obstacle_slow_distance_m = LaunchConfiguration('obstacle_slow_distance_m')
    front_stop_start_x_m = LaunchConfiguration('front_stop_start_x_m')
    front_stop_width_m = LaunchConfiguration('front_stop_width_m')

    safety_node = Node(
        package='my_bot',
        executable='safety_node.py',
        output='screen',
        parameters=[{
            'obstacle_stop_enabled': True,
            'obstacle_stop_distance_m': obstacle_stop_distance_m,
            'obstacle_slow_distance_m': obstacle_slow_distance_m,
            'front_stop_start_x_m': front_stop_start_x_m,
            'front_stop_width_m': front_stop_width_m,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacle_stop_distance_m',
            default_value='0.20',
            description='Stop if an obstacle is within this forward distance in meters.'
        ),
        DeclareLaunchArgument(
            'obstacle_slow_distance_m',
            default_value='0.60',
            description='Begin reducing forward speed within this distance in meters.'
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
        safety_node,
    ])