"""Static regression tests for the Pi hardware/safety runtime contract."""

import re
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read(relative_path):
    return (PACKAGE_ROOT / relative_path).read_text(encoding='utf-8')


def test_systemd_selects_only_production_hardware_launch():
    unit = read('systemd/my-bot-robot.service.in')
    wrapper = read('scripts/start_robot_stack.sh')
    defaults = read('systemd/my-bot-robot.default')
    assert 'Environment=ROBOT_LAUNCH_FILE=rpi_robot.launch.py' in unit
    assert 'rpi_autonomy.launch.py' not in unit
    assert 'ROBOT_LAUNCH_FILE}" != "rpi_robot.launch.py"' in wrapper
    assert 'network-online.target' not in unit
    assert 'ROBOT_INITIAL_CLEAN_START=once' in defaults
    assert 'perform_initial_clean_start' in wrapper
    assert 'initial-clean-start-complete' in wrapper
    assert 'collect_device_owner_pids' in wrapper
    assert 'read_cleanup_pids' in wrapper
    assert 'process discovery failed; refusing to continue' in wrapper
    assert 'if ! legacy_robot_pids="$(collect_legacy_robot_pids)"' in wrapper
    assert 'if ! device_owner_pids="$(collect_device_owner_pids)"' in wrapper


def test_production_launch_closure_has_no_central_compute_nodes():
    production_files = (
        'launch/rpi_robot.launch.py',
        'launch/launch_robot.launch.py',
        'launch/safety.launch.py',
        'launch/rsp.launch.py',
        'launch/ydlidar.launch.py',
        'launch/joystick.launch.py',
    )
    banned_node_packages = (
        'nav2_',
        "package='slam_toolbox'",
        "package='rviz2'",
        "executable='amcl'",
        "executable='map_server'",
    )
    combined = '\n'.join(read(path) for path in production_files)
    for banned in banned_node_packages:
        assert banned not in combined
    assert "executable='heartbeat_node.py'" not in combined


def test_manual_and_navigation_commands_both_pass_through_safety():
    safety = read('scripts/safety_node.py')
    mux = read('config/twist_mux.yaml')
    assert "create_subscription(Twist, '/cmd_vel_joy'" in safety
    assert "create_subscription(Twist, '/cmd_vel_nav_raw'" in safety
    assert "'/cmd_vel_joy_safe'" in safety
    assert "'/cmd_vel_nav_safe'" in safety
    assert 'topic: /cmd_vel_joy_safe' in mux
    assert 'topic: /cmd_vel_nav_safe' in mux
    assert re.search(
        r'safety:\s+topic: /cmd_vel_safety\s+timeout: 0\.15\s+priority: 255',
        mux,
    )
    assert re.search(
        r'joystick:\s+topic: /cmd_vel_joy_safe\s+timeout: 0\.20\s+priority: 100',
        mux,
    )
    assert re.search(
        r'navigation:\s+topic: /cmd_vel_nav_safe\s+timeout: 0\.20\s+priority: 10',
        mux,
    )


def test_only_twist_mux_targets_diff_drive_command_topic():
    code_files = tuple((PACKAGE_ROOT / 'launch').glob('*.py')) + tuple(
        (PACKAGE_ROOT / 'scripts').glob('*')
    )
    owners = [
        path.relative_to(PACKAGE_ROOT).as_posix()
        for path in code_files
        if path.is_file()
        and '/diff_cont/cmd_vel_unstamped' in path.read_text(encoding='utf-8')
    ]
    assert owners == ['launch/launch_robot.launch.py']
    launch = read('launch/launch_robot.launch.py')
    assert "package='twist_mux'" in launch
    assert "('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')" in launch


def test_restart_and_sensor_loss_are_fail_safe():
    safety = read('scripts/safety_node.py')
    assert "declare_parameter('startup_quiet_sec', 5.0)" in safety
    assert 'self.startup_gate_open = False' in safety
    assert 'not self.startup_gate_open or not self.is_scan_healthy()' in safety
    assert "declare_parameter('scan_timeout_sec', 0.5)" in safety
    assert 'not self.is_scan_healthy()' in safety
    assert 'if joy_active:' in safety
    assert 'else:\n            self.joy_gate_pub.publish(Twist())' in safety
    assert 'self.joy_gate_pub.publish(Twist())' in safety
    assert "'/robot/power_command'" not in safety
    assert 'RESET' not in safety


def test_local_readiness_requires_filtered_scan_and_odometry():
    health = read('scripts/robot_health_node.py')
    launch = read('launch/launch_robot.launch.py')
    assert "'/scan_filtered'" in health
    assert "'/diff_cont/odom'" in health
    assert "'/joint_states'" in health
    assert "'/robot_health/ready'" in health
    assert "'/robot_health/startup_gate_open'" in health
    assert 'and lidar_healthy' in health
    assert 'and odometry_healthy' in health
    assert "executable='robot_health_node.py'" in launch


def test_controller_and_firmware_timeouts_are_short():
    controllers = read('config/my_controllers.yaml')
    firmware = read('arduino_sketch/arduino_sketch.ino')
    controller_timeout = float(
        re.search(r'cmd_vel_timeout:\s*([0-9.]+)', controllers).group(1)
    )
    firmware_timeout_ms = int(
        re.search(r'#define COMMAND_TIMEOUT_MS\s+([0-9]+)UL', firmware).group(1)
    )
    assert controller_timeout <= 0.25
    assert firmware_timeout_ms <= 200


def test_remote_navigation_stream_has_wifi_jitter_margin():
    nav2 = read('config/nav2_params.yaml')
    safety = read('scripts/safety_node.py')
    safety_launch = read('launch/safety.launch.py')

    controller_frequency = float(
        re.search(r'controller_frequency:\s*([0-9.]+)', nav2).group(1)
    )
    smoothing_frequency = float(
        re.search(r'smoothing_frequency:\s*([0-9.]+)', nav2).group(1)
    )
    velocity_timeout = float(
        re.search(r'velocity_timeout:\s*([0-9.]+)', nav2).group(1)
    )
    nav_timeout = float(
        re.search(
            r"declare_parameter\('nav_timeout_sec',\s*([0-9.]+)\)",
            safety,
        ).group(1)
    )

    assert controller_frequency >= 20.0
    assert smoothing_frequency >= 20.0
    assert velocity_timeout >= 0.5
    assert nav_timeout >= 0.5
    assert "nav_timeout_sec = LaunchConfiguration('nav_timeout_sec')" in safety_launch
    assert "'nav_timeout_sec': nav_timeout_sec" in safety_launch
    assert "default_value='0.50'" in safety_launch
    assert 'self.create_timer(0.05, self.publish_safety_hold)' in safety


def test_serial_devices_are_launch_parameters():
    rpi_launch = read('launch/rpi_robot.launch.py')
    rsp_launch = read('launch/rsp.launch.py')
    lidar_launch = read('launch/ydlidar.launch.py')
    control_xacro = read('description/ros2_control.xacro')
    assert "'motor_device'" in rpi_launch
    assert "'lidar_device'" in rpi_launch
    assert "' motor_device:='" in rsp_launch
    assert "{'port': port}" in lidar_launch
    assert '<param name="device">$(arg motor_device)</param>' in control_xacro


def test_cyclonedds_base_config_has_no_machine_specific_peers():
    config_path = PACKAGE_ROOT / 'config/cyclonedds.xml'
    config = config_path.read_text(encoding='utf-8')
    ET.parse(config_path)
    assert 'rodrigo-linux-laptop' not in config
    assert 'zrpi-desktop' not in config
    assert '<Peer Address=' not in config


def test_legacy_autonomy_launch_is_not_silent():
    legacy = read('launch/rpi_autonomy.launch.py')
    assert 'legacy all-in-one diagnostic' in legacy
    assert 'not supported as the Pi production service entrypoint' in legacy
