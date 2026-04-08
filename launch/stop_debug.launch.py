from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_path = LaunchConfiguration('log_path')
    log_hz = LaunchConfiguration('log_hz')

    debug_node = Node(
        package='my_bot',
        executable='stop_debug_monitor.py',
        output='screen',
        parameters=[{
            'log_path': log_path,
            'log_hz': log_hz,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_path',
            default_value='~/stop_debug_log.csv',
            description='CSV file path for stop debug logging.'
        ),
        DeclareLaunchArgument(
            'log_hz',
            default_value='5.0',
            description='Logging frequency in Hz.'
        ),
        debug_node,
    ])