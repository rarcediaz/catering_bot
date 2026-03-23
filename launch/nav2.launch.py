import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    package_name = 'my_bot'
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2_params.yaml'
    )

    nav2 = GroupAction([
        SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_nav'),
        SetRemap(src='smoothed_cmd_vel', dst='cmd_vel_nav'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'map': map_file,
                'params_file': params_file,
            }.items()
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true.'
        ),
        DeclareLaunchArgument(
            'map',
            description='Full path to the saved map YAML file.'
        ),
        nav2,
    ])