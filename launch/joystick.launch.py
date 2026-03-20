import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name='my_bot' #<--- CHANGE ME

    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[{'use_sim_time': use_sim_time}, joy_params],
    )
    
    teleop_node = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name = 'teleop_node',
        parameters=[{'use_sim_time': use_sim_time}, joy_params],
        remappings={('/cmd_vel', '/cmd_vel_joy')},
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
    ])