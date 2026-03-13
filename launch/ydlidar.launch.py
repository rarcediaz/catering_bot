import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame',

            'baudrate': 115200,
            'lidar_type': 1,
            'device_type': 0,

            'isSingleChannel': True,
            'intensity': False,

            'sample_rate': 5,
            'abnormal_check_count': 4,
            'fixed_resolution': True,

            'reversion': False,
            'inverted': False,
            'auto_reconnect': True,

            'angle_min': -180.0,
            'angle_max': 180.0,

            'range_min': 0.1,
            'range_max': 12.0,

            'frequency': 7.0
        }]
    )

    return LaunchDescription([
        ydlidar_node
    ])