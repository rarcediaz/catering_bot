import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_bot'
    pkg_share = get_package_share_directory(package_name)

    # -----------------------
    # File paths
    # -----------------------
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    tmp_urdf = '/tmp/my_bot.urdf'

    # -----------------------
    # Robot State Publisher
    # -----------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # -----------------------
    # Step 1: Xacro -> URDF
    # -----------------------
    generate_urdf = ExecuteProcess(
        cmd=[
            'xacro',
            xacro_file,
            '-o',
            tmp_urdf
        ],
        output='screen'
    )

    # -----------------------
    # Gazebo
    # -----------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # -----------------------
    # Step 2: Spawn robot (after URDF is ready)
    # -----------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', tmp_urdf,
            '-name', 'my_bot',
            '-z', '0.5'
        ],
        output='screen'
    )

    spawn_after_xacro = RegisterEventHandler(
        OnProcessExit(
            target_action=generate_urdf,
            on_exit=[spawn_entity],
        )
    )

    # -----------------------
    # Step 3: ROS <-> Gazebo Bridge
    # -----------------------
    # Here we emulate your CLI command exactly
    # This will bridge /lidar2 (Ignition) → /laser_scan (ROS)
    laser_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',  # Ignition topic is /scan
        '--ros-args',
        '-r', '/scan:=/laser_scan'  # Remap to ROS topic /laser_scan
    ],
    output='screen'
)

    # -----------------------
    # Launch everything
    # -----------------------
    return LaunchDescription([
        rsp,
        generate_urdf,
        gazebo,
        spawn_after_xacro,
        laser_bridge
    ])