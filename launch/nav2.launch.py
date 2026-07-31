import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node, SetRemap
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_name = 'my_bot'
    package_share = get_package_share_directory(package_name)
    cyclonedds_uri = f'file://{os.path.join(package_share, "config", "cyclonedds.xml")}'
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    keepout_mask_file = LaunchConfiguration('keepout_mask')
    display_map_file = LaunchConfiguration('display_map')
    use_keepout = LaunchConfiguration('use_keepout')
    use_display_map = LaunchConfiguration('use_display_map')
    navigation_use_composition = LaunchConfiguration('navigation_use_composition')
    isolate_localization = LaunchConfiguration('isolate_localization')
    default_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2_params.yaml'
    )
    params_file = LaunchConfiguration('params_file')
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'local_costmap.local_costmap.ros__parameters.keepout_filter.enabled': use_keepout,
            'global_costmap.global_costmap.ros__parameters.keepout_filter.enabled': use_keepout,
        },
        convert_types=True,
    )

    nav2_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    navigation_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    composed_navigation = LoadComposableNodes(
        condition=IfCondition(navigation_use_composition),
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=nav2_remappings,
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=nav2_remappings,
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=nav2_remappings,
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=nav2_remappings,
            ),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=nav2_remappings,
            ),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=nav2_remappings + [
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel'),
                ],
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': False,
                    'node_names': navigation_lifecycle_nodes,
                    # Fast DDS discovery on the Pi can exceed Nav2's 4-second
                    # default while all costmaps and BT plugins activate.
                    'bond_timeout': 15.0,
                }],
            ),
        ],
    )

    standard_nav2 = GroupAction(
        condition=UnlessCondition(isolate_localization),
        actions=[
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
                    'use_composition': navigation_use_composition,
                }.items()
            ),
        ],
    )

    split_nav2 = GroupAction(
        condition=IfCondition(isolate_localization),
        actions=[
            SetRemap(src='cmd_vel_nav', dst='cmd_vel_nav_internal'),
            SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_nav_raw'),
            SetRemap(src='smoothed_cmd_vel', dst='cmd_vel_nav_raw'),
            Node(
                condition=IfCondition(navigation_use_composition),
                package='rclcpp_components',
                executable='component_container_isolated',
                name='nav2_container',
                output='screen',
                parameters=[configured_params, {'autostart': False}],
                arguments=['--ros-args', '--log-level', 'info'],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'localization_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'true',
                    'map': map_file,
                    'params_file': configured_params,
                    'use_composition': 'False',
                    'container_name': 'nav2_container',
                }.items()
            ),
            composed_navigation,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    )
                ),
                condition=UnlessCondition(navigation_use_composition),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'false',
                    'params_file': configured_params,
                    'use_composition': 'False',
                    'container_name': 'nav2_container',
                }.items()
            ),
            Node(
                package=package_name,
                executable='nav2_startup_gate.py',
                name='nav2_startup_gate',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'localization_topic': '/amcl_pose',
                    'manager_service': '/lifecycle_manager_navigation/manage_nodes',
                }],
            ),
        ],
    )

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
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', cyclonedds_uri),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true.'
        ),
        DeclareLaunchArgument(
            'navigation_use_composition',
            default_value='True',
            description=(
                'Run the navigation servers in a component container.'
            ),
        ),
        DeclareLaunchArgument(
            'isolate_localization',
            default_value='true',
            description=(
                'Run map_server and AMCL outside the navigation component '
                'container so initial localization and retries always remain '
                'reachable while navigation waits for the first AMCL pose.'
            ),
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
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the Nav2 parameter YAML file.'
        ),
        keepout_servers,
        display_server,
        standard_nav2,
        split_nav2,
    ])
