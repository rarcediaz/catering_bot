from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    xbox_controller = Node(
        package='my_bot',
        executable='xbox_controller.py',
        output='screen',
    )

    return LaunchDescription([
        xbox_controller,
    ])