import fcntl
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


_hardware_lock_handle = None


def acquire_hardware_lock():
    """Keep systemd and manual launches from sharing the serial devices."""
    global _hardware_lock_handle

    lock_path = os.environ.get(
        'MY_BOT_HARDWARE_LOCK_FILE',
        f'/tmp/my-bot-hardware-{os.getuid()}.lock',
    )
    try:
        lock_handle = open(lock_path, 'a+', encoding='utf-8')
        fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except (BlockingIOError, OSError) as exc:
        try:
            lock_handle.close()
        except (NameError, OSError):
            pass
        return None, lock_path, exc

    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write(f'{os.getpid()}\n')
    lock_handle.flush()
    _hardware_lock_handle = lock_handle
    return lock_handle, lock_path, None


def generate_launch_description():
    _, lock_path, lock_error = acquire_hardware_lock()
    if lock_error is not None:
        reason = (
            'Another RPi hardware stack is already running and owns the robot '
            f'devices (lock: {lock_path}). If systemd is active, stop it first '
            'with: sudo systemctl stop my-bot-robot.service'
        )
        return LaunchDescription([
            LogInfo(msg=f'ERROR: {reason} ({lock_error})'),
            EmitEvent(event=Shutdown(reason=reason)),
        ])

    package_name = 'my_bot'
    package_share = get_package_share_directory(package_name)
    cyclonedds_uri = f'file://{os.path.join(package_share, "config", "cyclonedds.xml")}'
    use_joystick = LaunchConfiguration('use_joystick')
    obstacle_stop_distance_m = LaunchConfiguration('obstacle_stop_distance_m')
    obstacle_stop_distance_max_m = LaunchConfiguration('obstacle_stop_distance_max_m')
    obstacle_stop_speed_mps = LaunchConfiguration('obstacle_stop_speed_mps')
    obstacle_slowdown_margin_m = LaunchConfiguration('obstacle_slowdown_margin_m')
    front_stop_start_x_m = LaunchConfiguration('front_stop_start_x_m')
    rear_stop_start_x_m = LaunchConfiguration('rear_stop_start_x_m')
    front_stop_width_m = LaunchConfiguration('front_stop_width_m')
    side_stop_distance_m = LaunchConfiguration('side_stop_distance_m')
    side_hard_stop_distance_m = LaunchConfiguration('side_hard_stop_distance_m')
    side_min_speed_scale = LaunchConfiguration('side_min_speed_scale')
    side_stop_start_y_m = LaunchConfiguration('side_stop_start_y_m')
    nav_timeout_sec = LaunchConfiguration('nav_timeout_sec')
    nav_stop_hold_sec = LaunchConfiguration('nav_stop_hold_sec')
    scan_timeout_sec = LaunchConfiguration('scan_timeout_sec')
    startup_quiet_sec = LaunchConfiguration('startup_quiet_sec')
    motor_device = LaunchConfiguration('motor_device')
    lidar_device = LaunchConfiguration('lidar_device')

    robot_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'launch_robot.launch.py'
            )
        ),
        launch_arguments={
            'use_joystick': use_joystick,
            'obstacle_stop_distance_m': obstacle_stop_distance_m,
            'obstacle_stop_distance_max_m': obstacle_stop_distance_max_m,
            'obstacle_stop_speed_mps': obstacle_stop_speed_mps,
            'obstacle_slowdown_margin_m': obstacle_slowdown_margin_m,
            'front_stop_start_x_m': front_stop_start_x_m,
            'rear_stop_start_x_m': rear_stop_start_x_m,
            'front_stop_width_m': front_stop_width_m,
            'side_stop_distance_m': side_stop_distance_m,
            'side_hard_stop_distance_m': side_hard_stop_distance_m,
            'side_min_speed_scale': side_min_speed_scale,
            'side_stop_start_y_m': side_stop_start_y_m,
            'nav_timeout_sec': nav_timeout_sec,
            'nav_stop_hold_sec': nav_stop_hold_sec,
            'scan_timeout_sec': scan_timeout_sec,
            'startup_quiet_sec': startup_quiet_sec,
            'motor_device': motor_device,
            'lidar_device': lidar_device,
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI',
            EnvironmentVariable('CYCLONEDDS_URI', default_value=cyclonedds_uri),
        ),
        DeclareLaunchArgument(
            'use_joystick',
            default_value='false',
            description='Launch the Pi-connected joystick and route it through safety.'
        ),
        DeclareLaunchArgument(
            'obstacle_stop_distance_m',
            default_value='0.25',
            description=(
                'Immediate Pi-local hard-stop clearance in meters; motion is '
                'progressively slowed before reaching this boundary.'
            )
        ),
        DeclareLaunchArgument(
            'obstacle_stop_distance_max_m',
            default_value='0.60',
            description='Clearance used to derive the progressive speed limit.'
        ),
        DeclareLaunchArgument(
            'obstacle_stop_speed_mps',
            default_value='0.60',
            description='Forward speed that maps to the maximum stop distance.'
        ),
        DeclareLaunchArgument(
            'obstacle_slowdown_margin_m',
            default_value='0.15',
            description=(
                'Additional distance ahead of the stop threshold where forward '
                'speed is scaled down.'
            )
        ),
        DeclareLaunchArgument(
            'front_stop_start_x_m',
            default_value='0.09',
            description='Distance from lidar to the robot front edge in meters.'
        ),
        DeclareLaunchArgument(
            'rear_stop_start_x_m',
            default_value='0.91',
            description='Distance from lidar to the robot rear edge in meters.'
        ),
        DeclareLaunchArgument(
            'front_stop_width_m',
            default_value='0.8596',
            description='Width of the forward stop corridor in meters.'
        ),
        DeclareLaunchArgument(
            'side_stop_distance_m',
            default_value='0.25',
            description=(
                'Begin progressively slowing turns toward an obstacle within '
                'this side clearance in meters.'
            )
        ),
        DeclareLaunchArgument(
            'side_hard_stop_distance_m',
            default_value='0.08',
            description=(
                'Stop a turn toward an obstacle only inside this imminent '
                'side-collision clearance in meters.'
            )
        ),
        DeclareLaunchArgument(
            'side_min_speed_scale',
            default_value='0.25',
            description=(
                'Minimum progressive turn scale immediately outside the side '
                'hard-stop boundary.'
            )
        ),
        DeclareLaunchArgument(
            'side_stop_start_y_m',
            default_value='0.34',
            description='Distance from lidar centerline to the robot side edge in meters.'
        ),
        DeclareLaunchArgument(
            'nav_timeout_sec',
            default_value='0.50',
            description=(
                'Keep the latest navigation command through a short DDS/Wi-Fi '
                'gap while fresh Pi-local lidar safety remains active.'
            )
        ),
        DeclareLaunchArgument(
            'nav_stop_hold_sec',
            default_value='0.50',
            description=(
                'High-priority zero-command hold after a moving navigation '
                'stream is lost, not after an intentional fresh zero.'
            )
        ),
        DeclareLaunchArgument(
            'scan_timeout_sec',
            default_value='0.50',
            description='Block motion when filtered lidar scans become stale.'
        ),
        DeclareLaunchArgument(
            'startup_quiet_sec',
            default_value='5.0',
            description=(
                'Seconds of quiet raw motion commands required before the '
                'startup gate opens automatically.'
            )
        ),
        DeclareLaunchArgument(
            'motor_device',
            default_value=EnvironmentVariable(
                'ROBOT_MOTOR_DEVICE',
                default_value='/dev/ttyACM0',
            ),
            description='Arduino serial device; use /dev/serial/by-id/... in production.'
        ),
        DeclareLaunchArgument(
            'lidar_device',
            default_value=EnvironmentVariable(
                'ROBOT_LIDAR_DEVICE',
                default_value='/dev/ttyUSB0',
            ),
            description='YDLidar serial device; use /dev/serial/by-id/... in production.'
        ),
        robot_base,
    ])
