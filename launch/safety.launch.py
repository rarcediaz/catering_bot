import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_name = 'my_bot'
    safety_script = os.path.join(
        get_package_share_directory(package_name),
        'launch',
        'safety_node.py'
    )

    safety_node = ExecuteProcess(
        cmd=['python3', safety_script],
        output='screen',
    )

    return LaunchDescription([
        safety_node,
    ])