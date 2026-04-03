import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_name = 'my_bot'
    xbox_script = os.path.join(
        get_package_share_directory(package_name),
        'launch',
        'xbox_controller.py'
    )

    xbox_controller = ExecuteProcess(
        cmd=['python3', xbox_script],
        output='screen',
    )

    return LaunchDescription([
        xbox_controller,
    ])