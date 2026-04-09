import os

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    xbox_controller = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(
                get_package_prefix('my_bot'),
                'lib',
                'my_bot',
                'xbox_controller.py',
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        xbox_controller,
    ])