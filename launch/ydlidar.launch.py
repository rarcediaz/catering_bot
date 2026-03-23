from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'my_bot'
    ydlidar_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'ydlidar.yaml'
    )

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar',
        output='screen',
        parameters=[ydlidar_params]
    )

    return LaunchDescription([
        ydlidar_node
    ])