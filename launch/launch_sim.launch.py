import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_bot'
    pkg_share = get_package_share_directory(package_name)
    headless = LaunchConfiguration('headless')

    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    gazebo_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py',
        )
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'false',
            'sim_mode': 'true',
        }.items(),
    )

    gazebo_gui = IncludeLaunchDescription(
        gazebo_launch,
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(headless),
    )

    gazebo_headless = IncludeLaunchDescription(
        gazebo_launch,
        launch_arguments={
            'gz_args': f'-r -s --headless-rendering {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(headless),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-z', '0.15',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'qos_overrides./clock.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run only the Gazebo server with headless rendering.',
        ),
        robot_state_publisher,
        gazebo_gui,
        gazebo_headless,
        spawn_robot,
        bridge,
    ])
