import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_name = 'my_bot'
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    keepout_mask_file = LaunchConfiguration('keepout_mask')
    display_map_file = LaunchConfiguration('display_map')
    use_keepout = LaunchConfiguration('use_keepout')
    use_display_map = LaunchConfiguration('use_display_map')
    params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2_params.yaml'
    )
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'local_costmap.local_costmap.ros__parameters.keepout_filter.enabled': use_keepout,
            'global_costmap.global_costmap.ros__parameters.keepout_filter.enabled': use_keepout,
        },
        convert_types=True,
    )

    nav2 = GroupAction([
        SetRemap(src='cmd_vel_nav', dst='cmd_vel_nav_internal'),
        SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_nav_raw'),
        SetRemap(src='smoothed_cmd_vel', dst='cmd_vel_nav_raw'),
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
                'params_file': configured_params,
            }.items()
        ),
    ])

    keepout_servers = GroupAction(
        condition=IfCondition(use_keepout),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'yaml_filename': keepout_mask_file,
                    'topic_name': 'keepout_filter_mask',
                    'frame_id': 'map',
                }],
            ),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'type': 0,
                    'filter_info_topic': 'costmap_filter_info',
                    'mask_topic': '/keepout_filter_mask',
                    'base': 0.0,
                    'multiplier': 1.0,
                }],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_keepout',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': [
                        'filter_mask_server',
                        'costmap_filter_info_server',
                    ],
                }],
            ),
        ],
    )

    display_server = GroupAction(
        condition=IfCondition(use_display_map),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='display_map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'yaml_filename': display_map_file,
                    'topic_name': 'display_map',
                    'frame_id': 'map',
                }],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_display_map',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['display_map_server'],
                }],
            ),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true.'
        ),
        DeclareLaunchArgument(
            'map',
            description='Full path to the saved map YAML file.'
        ),
        DeclareLaunchArgument(
            'use_keepout',
            default_value='false',
            description='Apply the keepout mask to the local and global costmaps.'
        ),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value='',
            description='Full path to the keepout mask YAML file.'
        ),
        DeclareLaunchArgument(
            'use_display_map',
            default_value='false',
            description='Publish a UI-only map on /display_map.'
        ),
        DeclareLaunchArgument(
            'display_map',
            default_value='',
            description='Full path to the UI display map YAML file.'
        ),
        keepout_servers,
        display_server,
        nav2,
    ])
