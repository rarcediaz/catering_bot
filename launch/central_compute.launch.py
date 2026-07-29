import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_central_mode(context):
    use_slam = LaunchConfiguration('use_slam').perform(context).lower() in {
        '1', 'true', 'yes', 'on',
    }
    use_nav2 = LaunchConfiguration('use_nav2').perform(context).lower() in {
        '1', 'true', 'yes', 'on',
    }
    if use_slam and use_nav2:
        raise RuntimeError(
            'Central mapping and navigation modes are mutually exclusive: '
            'SLAM and AMCL/Nav2 cannot run together.'
        )
    return []


def generate_launch_description():
    package_name = 'my_bot'
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joystick = LaunchConfiguration('use_joystick')
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_rviz = LaunchConfiguration('use_rviz')
    map_file = LaunchConfiguration('map')
    keepout_mask_file = LaunchConfiguration('keepout_mask')
    display_map_file = LaunchConfiguration('display_map')
    use_keepout = LaunchConfiguration('use_keepout')
    use_display_map = LaunchConfiguration('use_display_map')

    package_share = get_package_share_directory(package_name)
    cyclonedds_uri = f'file://{os.path.join(package_share, "config", "cyclonedds.xml")}'

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'joystick.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_joystick),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_slam),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'use_keepout': use_keepout,
            'keepout_mask': keepout_mask_file,
            'use_display_map': use_display_map,
            'display_map': display_map_file,
        }.items(),
        condition=IfCondition(use_nav2),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    default_map = os.path.join(package_share, 'maps', 'atrium_navigation.yaml')
    default_keepout_mask = os.path.join(package_share, 'maps', 'atrium_keepout.yaml')
    default_display_map = os.path.join(package_share, 'maps', 'atrium_display.yaml')

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', cyclonedds_uri),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true.'
        ),
        DeclareLaunchArgument(
            'use_joystick',
            default_value='false',
            description='Launch joystick teleop on the central computer.'
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='Launch slam_toolbox on the central computer.'
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Launch Nav2 on the central computer.'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 on the central computer.'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the navigation map YAML file used when use_nav2 is true.'
        ),
        DeclareLaunchArgument(
            'use_keepout',
            default_value='true',
            description='Apply the keepout mask while navigating.'
        ),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=default_keepout_mask,
            description='Full path to the keepout mask YAML file.'
        ),
        DeclareLaunchArgument(
            'use_display_map',
            default_value='true',
            description='Publish the UI-only display map on /display_map.'
        ),
        DeclareLaunchArgument(
            'display_map',
            default_value=default_display_map,
            description='Full path to the UI display map YAML file.'
        ),
        OpaqueFunction(function=validate_central_mode),
        joystick,
        slam,
        nav2,
        rviz,
    ])
