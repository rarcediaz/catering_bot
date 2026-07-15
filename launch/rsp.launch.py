import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    sim_mode = LaunchConfiguration('sim_mode')

    pkg_path = get_package_share_directory('my_bot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = Command([
        'xacro ',
        xacro_file,
        ' use_ros2_control:=',
        use_ros2_control,
        ' sim_mode:=',
        sim_mode,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock.',
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Include the physical ros2_control hardware interface.',
        ),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Include Gazebo simulation plugins in the robot model.',
        ),
        robot_state_publisher,
    ])
