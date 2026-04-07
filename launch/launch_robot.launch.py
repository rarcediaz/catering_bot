import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'my_bot'
    use_joystick = LaunchConfiguration('use_joystick')
    use_battery_monitor = LaunchConfiguration('use_battery_monitor')
    obstacle_stop_distance_m = LaunchConfiguration('obstacle_stop_distance_m')
    obstacle_slow_distance_m = LaunchConfiguration('obstacle_slow_distance_m')
    front_stop_start_x_m = LaunchConfiguration('front_stop_start_x_m')
    front_stop_width_m = LaunchConfiguration('front_stop_width_m')

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'true'
        }.items()
    )

    # JoyStick


    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]),
                condition=IfCondition(use_joystick)
    )

    battery_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'safety.launch.py'
            )
        ),
        launch_arguments={
            'obstacle_stop_distance_m': obstacle_stop_distance_m,
            'obstacle_slow_distance_m': obstacle_slow_distance_m,
            'front_stop_start_x_m': front_stop_start_x_m,
            'front_stop_width_m': front_stop_width_m,
        }.items(),
        condition=IfCondition(use_battery_monitor)
    )


    # YDLidar Launch
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'ydlidar.launch.py'
            )
        )
    )

    # Controllers
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params_file],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen'
    )

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Twist mux
    twist_mux_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
        parameters=[
            {'use_sim_time': False},
            twist_mux_config
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joystick',
            default_value='true',
            description='Launch local joystick teleop on this machine if true.'
        ),
        DeclareLaunchArgument(
            'use_battery_monitor',
            default_value='true',
            description='Launch the battery and safety monitoring node if true.'
        ),
        DeclareLaunchArgument(
            'obstacle_stop_distance_m',
            default_value='0.20',
            description='Stop if an obstacle is within this forward distance in meters.'
        ),
        DeclareLaunchArgument(
            'obstacle_slow_distance_m',
            default_value='0.60',
            description='Begin reducing forward speed within this distance in meters.'
        ),
        DeclareLaunchArgument(
            'front_stop_start_x_m',
            default_value='0.0508',
            description='Distance from lidar to the robot front edge in meters.'
        ),
        DeclareLaunchArgument(
            'front_stop_width_m',
            default_value='0.8596',
            description='Width of the forward stop corridor in meters.'
        ),
        rsp,
        joystick,
        battery_monitor,
        ydlidar,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        twist_mux,
    ])