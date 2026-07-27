import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'my_bot'
    package_share = get_package_share_directory(package_name)
    cyclonedds_uri = f'file://{os.path.join(package_share, "config", "cyclonedds.xml")}'

    robot_id = LaunchConfiguration('robot_id')
    mission_control_url = LaunchConfiguration('mission_control_url')
    use_heartbeat = LaunchConfiguration('use_heartbeat')
    map_file = LaunchConfiguration('map')
    keepout_mask_file = LaunchConfiguration('keepout_mask')
    display_map_file = LaunchConfiguration('display_map')
    params_file = LaunchConfiguration('params_file')
    use_keepout = LaunchConfiguration('use_keepout')
    use_display_map = LaunchConfiguration('use_display_map')
    navigation_use_composition = LaunchConfiguration('navigation_use_composition')
    isolate_localization = LaunchConfiguration('isolate_localization')
    nav_start_delay_s = LaunchConfiguration('nav_start_delay_s')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'rpi_robot.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id,
            'mission_control_url': mission_control_url,
            'use_heartbeat': use_heartbeat,
            'use_joystick': 'false',
            'use_safety_node': 'true',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_file,
            'use_keepout': use_keepout,
            'keepout_mask': keepout_mask_file,
            'use_display_map': use_display_map,
            'display_map': display_map_file,
            'params_file': params_file,
            'navigation_use_composition': navigation_use_composition,
            'isolate_localization': isolate_localization,
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', cyclonedds_uri),
        DeclareLaunchArgument(
            'robot_id',
            default_value='IntelliTrolley-01',
            description='Stable robot identity shown in Mission Control.',
        ),
        DeclareLaunchArgument(
            'mission_control_url',
            default_value='http://127.0.0.1:8000',
            description='Mission Control server URL for optional heartbeat telemetry.',
        ),
        DeclareLaunchArgument(
            'use_heartbeat',
            default_value='false',
            description=(
                'The local ROS adapter supplies telemetry; '
                'HTTP heartbeat is normally disabled.'
            ),
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(package_share, 'maps', 'atrium_navigation.yaml'),
            description='Navigation map YAML.',
        ),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=os.path.join(package_share, 'maps', 'atrium_keepout.yaml'),
            description='Keepout mask YAML.',
        ),
        DeclareLaunchArgument(
            'display_map',
            default_value=os.path.join(package_share, 'maps', 'atrium_display.yaml'),
            description='UI display map YAML.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters tuned for the physical robot.',
        ),
        DeclareLaunchArgument(
            'use_keepout',
            default_value='true',
            description='Apply the keepout mask in local and global costmaps.',
        ),
        DeclareLaunchArgument(
            'use_display_map',
            default_value='true',
            description='Publish the UI-only display map.',
        ),
        DeclareLaunchArgument(
            'navigation_use_composition',
            default_value='True',
            description=(
                'Compose the Pi navigation servers to reduce process and DDS load.'
            ),
        ),
        DeclareLaunchArgument(
            'isolate_localization',
            default_value='true',
            description=(
                'Keep map_server and AMCL in separate processes so localization '
                'remains available while navigation waits for map-to-odom.'
            ),
        ),
        DeclareLaunchArgument(
            'nav_start_delay_s',
            default_value='8.0',
            description='Allow lidar, controllers, odometry, and TF to start before Nav2.',
        ),
        robot,
        TimerAction(period=nav_start_delay_s, actions=[navigation]),
    ])
