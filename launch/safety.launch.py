from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    safety_node = Node(
        package='my_bot',
        executable='safety_node.py',
        output='screen',
    )

    return LaunchDescription([
        safety_node,
    ])