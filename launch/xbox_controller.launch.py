import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    cyclonedds_uri = (
        f'file://{os.path.join(get_package_share_directory("my_bot"), "config", "cyclonedds.xml")}'
    )
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
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', cyclonedds_uri),
        xbox_controller,
    ])
