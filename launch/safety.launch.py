from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    obstacle_stop_distance_m = LaunchConfiguration('obstacle_stop_distance_m')
    front_stop_half_angle_deg = LaunchConfiguration('front_stop_half_angle_deg')

    safety_node = Node(
        package='my_bot',
        executable='safety_node.py',
        output='screen',
        parameters=[{
            'obstacle_stop_enabled': True,
            'obstacle_stop_distance_m': obstacle_stop_distance_m,
            'front_stop_half_angle_deg': front_stop_half_angle_deg,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacle_stop_distance_m',
            default_value='0.10',
            description='Stop if an obstacle is within this front distance in meters.'
        ),
        DeclareLaunchArgument(
            'front_stop_half_angle_deg',
            default_value='15.0',
            description='Half-angle of the front stop cone in degrees.'
        ),
        safety_node,
    ])