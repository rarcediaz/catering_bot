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
    assert 'elif self.joy_was_active:' in safety
    assert 'Continuously publishing inactive joystick zeros' in safety
    assert 'else:\n            self.joy_gate_pub.publish(Twist())' not in safety
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

    # Trajectory scoring may run more slowly than the remote command stream.
    # velocity_smoother republishes the accepted command at 20 Hz while the Pi
    # independently applies fresh lidar constraints and its navigation deadman.
    assert controller_frequency >= 10.0
    assert smoothing_frequency >= 20.0
    assert smoothing_frequency >= controller_frequency
    assert velocity_timeout >= 0.5
    assert nav_timeout >= 0.5
    assert "nav_timeout_sec = LaunchConfiguration('nav_timeout_sec')" in safety_launch
    assert "'nav_timeout_sec': nav_timeout_sec" in safety_launch
    assert "default_value='0.50'" in safety_launch
    assert 'self.create_timer(0.05, self.publish_safety_hold)' in safety


def test_navigation_speed_ceiling_matches_hardware_limit():
    nav2 = read('config/nav2_params.yaml')
    controllers = read('config/my_controllers.yaml')

    max_vel_x = float(re.search(r'max_vel_x:\s*([0-9.]+)', nav2).group(1))
    max_speed_xy = float(re.search(r'max_speed_xy:\s*([0-9.]+)', nav2).group(1))
    smoother_max = float(
        re.search(r'max_velocity:\s*\[([0-9.]+),', nav2).group(1)
    )
    hardware_max = float(
        re.search(r'linear\.x\.max_velocity:\s*([0-9.]+)', controllers).group(1)
    )

    assert max_vel_x == 0.70
    assert max_speed_xy == 0.70
    assert smoother_max == 0.70
    assert hardware_max >= smoother_max
    assert 'feedback: "OPEN_LOOP"' in nav2


def test_navigation_finishes_by_position_without_a_final_heading_spin():
    nav2 = read('config/nav2_params.yaml')

    assert 'plugin: "nav2_controller::PositionGoalChecker"' in nav2
    assert 'rotate_to_goal_heading: false' in nav2
    assert re.search(r'xy_goal_tolerance:\s*0\.35\b', nav2)


def test_navigation_turns_are_bounded_and_use_the_rotation_shim():
    nav2 = read('config/nav2_params.yaml')
    max_vel_theta = float(
        re.search(r'max_vel_theta:\s*([0-9.]+)', nav2).group(1)
    )
    smoother_theta = float(
        re.search(
            r'max_velocity:\s*\[[0-9.]+,\s*0\.0,\s*([0-9.]+)\]',
            nav2,
        ).group(1)
    )

    assert (
        'plugin: "nav2_rotation_shim_controller::RotationShimController"'
        in nav2
    )
    assert 'primary_controller: "dwb_core::DWBLocalPlanner"' in nav2
    assert re.search(r'rotate_to_heading_angular_vel:\s*0\.55\b', nav2)
    assert max_vel_theta == 0.80
    assert smoother_theta == max_vel_theta


def test_navigation_scores_the_rectangular_footprint_with_safety_clearance():
    nav2 = read('config/nav2_params.yaml')
    inflation_radii = [
        float(value)
        for value in re.findall(r'inflation_radius:\s*([0-9.]+)', nav2)
    ]

    assert '"ObstacleFootprint"' in nav2
    assert '"BaseObstacle"' not in nav2
    assert re.search(r'ObstacleFootprint\.scale:\s*0\.50\b', nav2)
    assert re.search(r'ObstacleFootprint\.sum_scores:\s*false\b', nav2)
    assert inflation_radii == [0.75, 1.10]
    assert re.search(r'vx_samples:\s*12\b', nav2)
    assert re.search(r'vtheta_samples:\s*24\b', nav2)


def test_normal_navigation_can_reverse_safely_and_replans_stably():
    nav2 = read('config/nav2_params.yaml')
    nav2_launch = read('launch/nav2.launch.py')
    cmake = read('CMakeLists.txt')
    behavior_tree = read(
        'behavior_trees/navigate_to_pose_stable_replanning.xml'
    )

    assert re.search(r'controller_frequency:\s*10\.0\b', nav2)
    assert re.search(r'min_vel_x:\s*-0\.35\b', nav2)
    assert re.search(r'smoothing_frequency:\s*20\.0\b', nav2)
    assert re.search(r'min_velocity:\s*\[-0\.35,\s*0\.0,\s*-0\.80\]', nav2)
    assert '<RateController hz="0.5">' in behavior_tree
    backup = '<BackUp backup_dist="0.50" backup_speed="0.10"/>'
    spin = '<Spin spin_dist="1.57"/>'
    assert backup in behavior_tree
    assert behavior_tree.index(backup) < behavior_tree.index(spin)
    assert re.search(r'failure_tolerance:\s*1\.5\b', nav2)
    assert "remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')]" in nav2_launch
    assert (
        'default_nav_to_pose_bt_xml: '
        '"NAVIGATE_TO_POSE_BT_XML_IS_SET_BY_LAUNCH"' in nav2
    )
    assert "'default_nav_to_pose_bt_xml': navigate_to_pose_bt_file" in nav2_launch
    assert 'DIRECTORY behavior_trees config description' in cmake


def test_amcl_cannot_randomly_relocate_during_navigation():
    nav2 = read('config/nav2_params.yaml')
    assert re.search(r'recovery_alpha_fast:\s*0\.0\b', nav2)
    assert re.search(r'recovery_alpha_slow:\s*0\.0\b', nav2)
    assert 'explicit, stationary operator action' in nav2


def test_localization_remains_recoverable_before_navigation_activation():
    central_launch = read('launch/central_compute.launch.py')
    nav2_launch = read('launch/nav2.launch.py')
    startup_gate = read('scripts/nav2_startup_gate.py')

    assert "isolate_localization = LaunchConfiguration('isolate_localization')" in (
        central_launch
    )
    assert "'isolate_localization': isolate_localization" in central_launch
    assert re.search(
        r"DeclareLaunchArgument\(\s*'isolate_localization',\s*"
        r"default_value='true'",
        central_launch,
    )
    assert re.search(
        r"DeclareLaunchArgument\(\s*'isolate_localization',\s*"
        r"default_value='true'",
        nav2_launch,
    )
    assert "'autostart': 'true'" in nav2_launch
    assert "'autostart': False" in nav2_launch
    assert "executable='nav2_startup_gate.py'" in nav2_launch
    assert 'not self._localized' in startup_gate
    assert 'Navigation lifecycle startup was not successful. Retrying.' in startup_gate


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
