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
    assert 'Requires=my-bot-network-ready.service' in unit
    assert 'Wants=my-bot-robot.service' in read(
        'systemd/my-bot-network-ready.service.in'
    )
    assert 'assert_wifi_ready' in wrapper
    assert 'ROBOT_CYCLONEDDS_INTERFACE=wlan0' in defaults
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

    max_vel_x = float(re.search(r'vx_max:\s*([0-9.]+)', nav2).group(1))
    smoother_max = float(
        re.search(r'max_velocity:\s*\[([0-9.]+),', nav2).group(1)
    )
    hardware_max = float(
        re.search(r'linear\.x\.max_velocity:\s*([0-9.]+)', controllers).group(1)
    )
    smoother_min = float(
        re.search(r'min_velocity:\s*\[(-[0-9.]+),', nav2).group(1)
    )
    hardware_min = float(
        re.search(r'linear\.x\.min_velocity:\s*(-[0-9.]+)', controllers).group(1)
    )

    assert max_vel_x == 0.50
    assert smoother_max == 0.50
    assert hardware_max >= smoother_max
    assert smoother_min == -0.50
    assert hardware_min <= smoother_min
    assert 'feedback: "OPEN_LOOP"' in nav2


def test_base_link_is_drive_axle_midpoint_and_geometry_is_consistent():
    core = read('description/robot_core.xacro')
    controllers = read('config/my_controllers.yaml')
    nav2 = read('config/nav2_params.yaml')

    modeled_separation = float(
        re.search(r'name="wheel_separation" value="([0-9.]+)"', core).group(1)
    )
    controlled_separation = float(
        re.search(r'wheel_separation:\s*([0-9.]+)', controllers).group(1)
    )

    assert modeled_separation == 0.44
    assert controlled_separation == modeled_separation
    assert 'xyz="0 ${wheel_separation / 2} -0.0538"' in core
    assert 'xyz="0 -${wheel_separation / 2} -0.0538"' in core
    assert '<origin xyz="-0.15 0 0"/>' in core
    axle_centered_footprint = (
        'footprint: "[[-0.15, -0.305], [0.917, -0.305], '
        '[0.917, 0.305], [-0.15, 0.305]]"'
    )
    assert nav2.count(axle_centered_footprint) == 2


def test_navigation_finishes_by_position_without_a_final_heading_spin():
    nav2 = read('config/nav2_params.yaml')

    assert 'plugin: "nav2_controller::PositionGoalChecker"' in nav2
    assert '"RotateToGoal"' not in nav2
    assert '"GoalAlign"' not in nav2
    assert re.search(r'xy_goal_tolerance:\s*0\.35\b', nav2)


def test_rotation_counts_as_navigation_progress():
    nav2 = read('config/nav2_params.yaml')

    assert 'plugin: "nav2_controller::PoseProgressChecker"' in nav2
    assert re.search(r'required_movement_radius:\s*0\.05\b', nav2)
    assert re.search(r'required_movement_angle:\s*0\.15\b', nav2)
    assert re.search(r'movement_time_allowance:\s*30\.0\b', nav2)


def test_mppi_prefers_front_lidar_travel_without_forbidding_reverse():
    nav2 = read('config/nav2_params.yaml')
    controller_hz = float(
        re.search(r'controller_frequency:\s*([0-9.]+)', nav2).group(1)
    )
    model_dt = float(
        re.search(r'model_dt:\s*([0-9.]+)', nav2).group(1)
    )
    time_steps = int(
        re.search(r'time_steps:\s*([0-9]+)', nav2).group(1)
    )
    max_vel_x = float(
        re.search(r'vx_max:\s*([0-9.]+)', nav2).group(1)
    )
    max_vel_theta = float(
        re.search(r'wz_max:\s*([0-9.]+)', nav2).group(1)
    )
    smoother_theta = float(
        re.search(
            r'max_velocity:\s*\[[0-9.]+,\s*0\.0,\s*([0-9.]+)\]',
            nav2,
        ).group(1)
    )

    assert 'plugin: "nav2_mppi_controller::MPPIController"' in nav2
    assert 'dwb_core::DWBLocalPlanner' not in nav2
    assert 'RotationShimController' not in nav2
    assert re.search(
        r'critics:\s*\["ConstraintCritic", "CostCritic", "GoalCritic", '
        r'"PathAlignCritic", "PathFollowCritic", "PathAngleCritic", '
        r'"PreferForwardCritic"\]',
        nav2,
    )
    assert re.search(r'^\s+PreferForwardCritic:', nav2, re.MULTILINE)
    assert re.search(
        r'PreferForwardCritic:\s*enabled:\s*true\s*cost_power:\s*1\s*'
        r'(?:#.*\s*)*cost_weight:\s*5\.0\s*'
        r'threshold_to_consider:\s*0\.5',
        nav2,
    )
    assert not re.search(r'^\s+GoalAngleCritic:', nav2, re.MULTILINE)
    assert not re.search(r'^\s+TwirlingCritic:', nav2, re.MULTILINE)
    assert re.search(r'motion_model:\s*"DiffDrive"', nav2)
    assert re.search(r'vx_min:\s*-0\.50\b', nav2)
    assert re.search(r'forward_preference:\s*false\b', nav2)
    assert re.search(r'use_path_orientations:\s*false\b', nav2)
    assert controller_hz == 10.0
    assert model_dt == 1.0 / controller_hz
    # The predicted axle travel plus the front overhang remains inside the
    # 2.5 m half-width of the rolling local costmap.
    assert time_steps * model_dt * max_vel_x + 0.917 < 2.5
    assert max_vel_theta == 0.80
    assert smoother_theta == max_vel_theta
    assert re.search(r'max_accel:\s*\[0\.35,\s*0\.0,\s*1\.50\]', nav2)
    assert re.search(r'max_decel:\s*\[-2\.00,\s*0\.0,\s*-1\.80\]', nav2)
    assert re.search(r'max_rotational_vel:\s*0\.70\b', nav2)
    assert re.search(r'min_rotational_vel:\s*0\.10\b', nav2)
    assert re.search(r'rotational_acc_lim:\s*0\.70\b', nav2)


def test_navigation_scores_the_rectangular_footprint_with_safety_clearance():
    nav2 = read('config/nav2_params.yaml')
    safety = read('scripts/safety_node.py')
    inflation_radii = [
        float(value)
        for value in re.findall(r'inflation_radius:\s*([0-9.]+)', nav2)
    ]
    inflation_scales = [
        float(value)
        for value in re.findall(r'cost_scaling_factor:\s*([0-9.]+)', nav2)
    ]
    side_start = float(
        re.search(
            r"declare_parameter\('side_stop_start_y_m',\s*([0-9.]+)\)",
            safety,
        ).group(1)
    )
    side_clearance = float(
        re.search(
            r"declare_parameter\('side_stop_distance_m',\s*([0-9.]+)\)",
            safety,
        ).group(1)
    )

    assert '"CostCritic"' in nav2
    assert re.search(r'consider_footprint:\s*true\b', nav2)
    assert re.search(r'collision_cost:\s*1000000\.0\b', nav2)
    assert re.search(r'cost_weight:\s*2\.5\b', nav2)
    assert re.search(r'inflation_layer_name:\s*"inflation_layer"', nav2)
    # Each costmap has ordinary obstacle inflation plus a second inflation
    # stage that runs after the keepout filter.
    assert inflation_radii == [0.60, 0.60, 0.60, 0.60]
    assert inflation_scales == [4.0, 4.0, 4.0, 4.0]
    assert nav2.count(
        'filters: ["keepout_filter", "keepout_inflation_layer"]'
    ) == 1
    assert nav2.count(
        'filters: ["keepout_filter", "keepout_inflation_layer", '
        '"preferred_filter"]'
    ) == 1
    assert nav2.count(
        'keepout_inflation_layer:\n'
        '        plugin: "nav2_costmap_2d::InflationLayer"'
    ) == 2
    assert abs(inflation_radii[0] - (side_start + side_clearance)) <= 0.01


def test_preferred_route_mask_is_a_separate_global_soft_cost_filter():
    nav2 = read('config/nav2_params.yaml')
    nav2_launch = read('launch/nav2.launch.py')
    central_launch = read('launch/central_compute.launch.py')
    preferred_yaml = read('maps/atrium_preferred.yaml')

    assert nav2.count('preferred_filter:') == 1
    assert 'filter_info_topic: "/preferred_costmap_filter_info"' in nav2
    assert "name='preferred_mask_server'" in nav2_launch
    assert "'topic_name': 'preferred_filter_mask'" in nav2_launch
    assert "'mask_topic': '/preferred_filter_mask'" in nav2_launch
    assert "'use_preferred': use_preferred" in central_launch
    assert "'maps', 'atrium_preferred.yaml'" in central_launch
    assert re.search(r'mode:\s*scale\b', preferred_yaml)
    assert re.search(r'resolution:\s*0\.05\b', preferred_yaml)


def test_thin_dynamic_obstacles_persist_across_lidar_sweeps():
    nav2 = read('config/nav2_params.yaml')

    # Local control only bridges a missed scan so stale returns cannot trap
    # MPPI. The global planner retains the longer memory needed to route around
    # intermittent chair-leg returns. This does not widen inflation.
    assert nav2.count('observation_persistence: 0.20') == 1
    assert nav2.count('observation_persistence: 0.75') == 1
    assert nav2.count('inflation_radius: 0.60') == 4
    assert nav2.count('cost_scaling_factor: 4.0') == 4


def test_normal_navigation_can_reverse_safely_and_replans_stably():
    nav2 = read('config/nav2_params.yaml')
    nav2_launch = read('launch/nav2.launch.py')
    cmake = read('CMakeLists.txt')
    behavior_tree = read(
        'behavior_trees/navigate_to_pose_stable_replanning.xml'
    )

    assert re.search(r'controller_frequency:\s*10\.0\b', nav2)
    assert re.search(r'vx_min:\s*-0\.50\b', nav2)
    assert re.search(r'transform_tolerance:\s*0\.70\b', nav2)
    assert re.search(r'smoothing_frequency:\s*20\.0\b', nav2)
    assert re.search(r'min_velocity:\s*\[-0\.50,\s*0\.0,\s*-0\.80\]', nav2)
    assert '<RateController hz="0.2">' in behavior_tree
    backup = '<BackUp backup_dist="0.50" backup_speed="0.10"/>'
    spin = '<Spin spin_dist="1.57"/>'
    assert '<Sequence name="BackUpThenSpin">' in behavior_tree
    assert backup in behavior_tree
    assert behavior_tree.index(backup) < behavior_tree.index(spin)
    assert re.search(r'default_server_timeout:\s*200\b', nav2)
    assert re.search(r'failure_tolerance:\s*1\.5\b', nav2)
    assert "remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')]" in nav2_launch
    assert (
        'default_nav_to_pose_bt_xml: '
        '"NAVIGATE_TO_POSE_BT_XML_IS_SET_BY_LAUNCH"' in nav2
    )
    assert "'default_nav_to_pose_bt_xml': navigate_to_pose_bt_file" in nav2_launch
    assert 'DIRECTORY behavior_trees config description' in cmake
    assert '<exec_depend>nav2_mppi_controller</exec_depend>' in read('package.xml')


def test_pi_safety_limits_converge_and_are_persistently_diagnosable():
    safety = read('scripts/safety_node.py')
    contract = read('docs/PI_CENTRAL_CONTRACT.md')
    launches = [
        read('launch/rpi_robot.launch.py'),
        read('launch/launch_robot.launch.py'),
        read('launch/safety.launch.py'),
    ]

    # The emergency boundary is fixed so odometry cannot create a stop/go
    # feedback loop. Clearance independently caps safe speed.
    get_speed = safety[safety.index('def get_forward_speed_mps'):]
    get_speed = get_speed[:get_speed.index('def get_dynamic_stop_distance')]
    assert 'self.forward_speed_mps' in get_speed
    assert 'latest_nav_cmd' not in get_speed
    get_stop = safety[safety.index('def get_dynamic_stop_distance'):]
    get_stop = get_stop[:get_stop.index('def get_clearance_speed_scale')]
    assert 'return self.obstacle_stop_distance_m, forward_speed' in get_stop
    assert 'speed_ratio' not in get_stop
    assert 'def get_clearance_speed_scale' in safety
    assert 'def get_side_turn_scale' in safety
    assert "declare_parameter('side_hard_stop_distance_m', 0.08)" in safety
    assert "declare_parameter('side_min_speed_scale', 0.25)" in safety
    assert "return 'left_turn_slowdown'" in safety
    assert "return 'right_turn_slowdown'" in safety
    assert 'closest_forward_clearance' in safety
    assert 'closest_rear_clearance' in safety

    for topic in (
        '/robot_health/left_obstacle_active',
        '/robot_health/right_obstacle_active',
        '/robot_health/rear_obstacle_active',
        '/robot_health/navigation_safety_limited',
        '/robot_health/navigation_safety_reason',
    ):
        assert topic in safety
        assert topic in contract

    assert 'self.get_logger().warning(text)' in safety
    assert 'Navigation safety constraint active:' in safety
    for launch in launches:
        assert re.search(
            r"'obstacle_stop_distance_m',\s*default_value='0\.25'",
            launch,
        )
        assert re.search(
            r"'side_hard_stop_distance_m',\s*default_value='0\.08'",
            launch,
        )


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
    assert 'from launch_ros.parameter_descriptions import ParameterValue' in rsp_launch
    assert 'robot_description = ParameterValue(' in rsp_launch
    assert 'value_type=str' in rsp_launch


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
