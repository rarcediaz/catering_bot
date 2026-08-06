"""Unit tests for the motion-constraint decisions in the Pi safety node."""

import sys
from pathlib import Path

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / 'scripts'))

from safety_node import ObstacleSafetyNode  # noqa: E402


def make_safety_node(scan_healthy=True):
    node = ObstacleSafetyNode.__new__(ObstacleSafetyNode)
    node.startup_gate_open = True
    node.front_obstacle_active = False
    node.front_obstacle_pending = False
    node.front_obstacle_detection_streak = 0
    node.front_obstacle_confirmation_scans = 3
    node.front_obstacle_pending_speed_mps = 0.10
    node.rear_obstacle_pending = False
    node.rear_obstacle_detection_streak = 0
    node.rear_obstacle_confirmation_scans = 3
    node.rear_obstacle_pending_speed_mps = 0.10
    node.rear_obstacle_active = False
    node.left_obstacle_active = False
    node.right_obstacle_active = False
    node.speed_limit_scale = 1.0
    node.rear_speed_limit_scale = 1.0
    node.left_turn_scale = 1.0
    node.right_turn_scale = 1.0
    node.closest_forward_clearance = float('inf')
    node.closest_left_clearance = float('inf')
    node.closest_right_clearance = float('inf')
    node.closest_rear_clearance = float('inf')
    node.obstacle_stop_distance_m = 0.25
    node.obstacle_stop_distance_max_m = 0.60
    node.obstacle_stop_speed_mps = 0.60
    node.obstacle_slowdown_margin_m = 0.15
    node.side_stop_distance_m = 0.25
    node.side_hard_stop_distance_m = 0.08
    node.side_min_speed_scale = 0.25
    node.turn_in_place_linear_threshold_mps = 0.05
    node.turn_in_place_angular_threshold_radps = 0.20
    node.command_epsilon = 0.005
    node.minimum_scan_ray_count = 100
    node.minimum_valid_scan_points = 10
    node.nav_constraint_reason = ''
    node.is_scan_healthy = lambda: scan_healthy
    return node


def make_command(linear=0.0, angular=0.0):
    command = Twist()
    command.linear.x = linear
    command.angular.z = angular
    return command


def make_scan(ray_count, valid_count):
    scan = LaserScan()
    scan.range_min = 0.1
    scan.range_max = 12.0
    scan.ranges = [1.0] * valid_count + [float('inf')] * (
        ray_count - valid_count
    )
    return scan


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_motion_tracking_node(command):
    node = make_safety_node()
    node.latest_nav_cmd = command
    node.joy_was_active = False
    node.command_epsilon = 0.005
    node.nav_was_active = False
    node.nav_stop_hold_until = 0.0
    node.nav_stop_hold_sec = 0.5
    node.rear_speed_limit_scale_pub = RecordingPublisher()
    node.nav_safety_limited_pub = RecordingPublisher()
    node.nav_safety_reason_pub = RecordingPublisher()
    return node


def test_closed_gate_and_stale_scan_each_fail_closed():
    node = make_safety_node()
    node.startup_gate_open = False
    stopped = node.apply_motion_constraints(make_command(0.4, 0.5))
    assert stopped.linear.x == 0.0
    assert stopped.angular.z == 0.0


def test_scan_quality_rejects_truncated_or_empty_lidar_data():
    node = make_safety_node()
    assert node.scan_is_usable(make_scan(10, 1)) is False
    assert node.scan_is_usable(make_scan(680, 0)) is False
    assert node.scan_is_usable(make_scan(680, 10)) is True

    node = make_safety_node(scan_healthy=False)
    stopped = node.apply_motion_constraints(make_command(0.4, 0.5))
    assert stopped.linear.x == 0.0
    assert stopped.angular.z == 0.0


def test_front_obstacle_blocks_forward_but_allows_reverse():
    node = make_safety_node()
    node.front_obstacle_active = True
    stopped = node.apply_motion_constraints(make_command(0.4, 0.5))
    assert stopped.linear.x == 0.0
    assert stopped.angular.z == 0.0

    reverse = node.apply_motion_constraints(make_command(-0.2, 0.5))
    assert reverse.linear.x == -0.2
    assert reverse.angular.z == 0.5

    # A front stop does not prohibit a separately collision-checked pure turn.
    assert node.apply_motion_constraints(make_command(angular=0.5)).angular.z == 0.5


def test_front_obstacle_requires_three_scans_and_crawls_while_pending():
    node = make_safety_node()

    assert node.update_front_obstacle_confirmation(True) is False
    assert node.front_obstacle_pending is True
    pending = node.apply_motion_constraints(make_command(0.5, 0.5))
    assert pending.linear.x == 0.10
    assert pending.angular.z == 0.10

    assert node.update_front_obstacle_confirmation(True) is False
    node.front_obstacle_active = node.update_front_obstacle_confirmation(True)
    assert node.front_obstacle_active is True
    confirmed = node.apply_motion_constraints(make_command(0.5, 0.5))
    assert confirmed.linear.x == 0.0
    assert confirmed.angular.z == 0.0

    node.front_obstacle_active = node.update_front_obstacle_confirmation(False)
    assert node.front_obstacle_active is False
    assert node.front_obstacle_pending is False
    assert node.front_obstacle_detection_streak == 0


def test_rear_obstacle_requires_three_scans_and_crawls_while_pending():
    node = make_safety_node()

    assert node.update_rear_obstacle_confirmation(True) is False
    assert node.rear_obstacle_pending is True
    pending = node.apply_motion_constraints(make_command(-0.5, -0.5))
    assert pending.linear.x == -0.10
    assert pending.angular.z == -0.10

    assert node.update_rear_obstacle_confirmation(True) is False
    node.rear_obstacle_active = node.update_rear_obstacle_confirmation(True)
    assert node.rear_obstacle_active is True
    confirmed = node.apply_motion_constraints(make_command(-0.5, -0.5))
    assert confirmed.linear.x == 0.0
    assert confirmed.angular.z == 0.0

    node.rear_obstacle_active = node.update_rear_obstacle_confirmation(False)
    assert node.rear_obstacle_active is False
    assert node.rear_obstacle_pending is False
    assert node.rear_obstacle_detection_streak == 0


def test_navigation_near_zero_arc_becomes_a_side_checked_in_place_turn():
    node = make_safety_node()
    node.front_obstacle_active = True
    node.rear_obstacle_active = True

    raw = make_command(0.03, -0.50)
    safe = node.apply_navigation_motion_constraints(raw)

    assert safe.linear.x == 0.0
    assert safe.angular.z == -0.50
    assert node.get_nav_constraint_reason(raw, safe) == 'turn_in_place'

    node.right_turn_scale = 0.5
    side_limited = node.apply_navigation_motion_constraints(raw)
    assert side_limited.linear.x == 0.0
    assert side_limited.angular.z == -0.25
    assert node.get_nav_constraint_reason(
        raw, side_limited
    ) == 'right_turn_slowdown'

    reverse_raw = make_command(-0.03, 0.50)
    reverse_safe = node.apply_navigation_motion_constraints(reverse_raw)
    assert reverse_safe.linear.x == 0.0
    assert reverse_safe.angular.z == 0.50


def test_turn_normalization_does_not_change_manual_or_translating_arcs():
    node = make_safety_node()
    node.front_obstacle_active = True

    manual = node.apply_motion_constraints(make_command(0.03, -0.50))
    assert manual.linear.x == 0.0
    assert manual.angular.z == 0.0

    translating_arc = node.apply_navigation_motion_constraints(
        make_command(0.06, -0.50)
    )
    assert translating_arc.linear.x == 0.0
    assert translating_arc.angular.z == 0.0

    low_angular_arc = node.apply_navigation_motion_constraints(
        make_command(0.03, -0.10)
    )
    assert low_angular_arc.linear.x == 0.0
    assert low_angular_arc.angular.z == 0.0


def test_front_slowdown_scales_only_forward_motion():
    node = make_safety_node()
    # This clearance admits 0.10 m/s with the configured linear clearance
    # model, so a 0.40 m/s raw request is scaled to one quarter.
    node.closest_forward_clearance = 0.25 + ((0.10 / 0.60) * 0.50)
    limited = node.apply_motion_constraints(make_command(0.4, 0.6))
    assert abs(limited.linear.x - 0.1) < 1e-9
    assert abs(limited.angular.z - 0.15) < 1e-9
    assert node.apply_motion_constraints(make_command(-0.4)).linear.x == -0.4


def test_hard_stop_boundary_is_fixed_and_allows_controlled_slowdown():
    node = make_safety_node()
    node.forward_speed_mps = 0.60
    stop_distance, measured_speed = node.get_dynamic_stop_distance()
    assert stop_distance == 0.25
    assert measured_speed == 0.60

    # The hard boundary does not move with odometry. The raw command may remain
    # at 0.70 m/s, but the clearance limiter admits only a small, convergent
    # command before that boundary is reached.
    node.forward_speed_mps = 0.0
    stop_distance, measured_speed = node.get_dynamic_stop_distance()
    assert stop_distance == 0.25
    assert measured_speed == 0.0
    node.closest_forward_clearance = 0.30
    limited = node.apply_motion_constraints(make_command(0.70)).linear.x
    assert 0.0 < limited < 0.20


def test_reverse_uses_the_same_clearance_speed_limiter():
    node = make_safety_node()
    node.closest_rear_clearance = 0.30
    limited = node.apply_motion_constraints(make_command(-0.70, -0.6))
    assert -0.20 < limited.linear.x < 0.0
    assert -0.20 < limited.angular.z < 0.0
    assert abs(
        limited.linear.x / -0.70 - limited.angular.z / -0.6
    ) < 1e-9


def test_side_obstacles_progressively_slow_only_turns_toward_them():
    node = make_safety_node()
    node.left_obstacle_active = True
    node.left_turn_scale = node.get_side_turn_scale(0.14)
    expected_scale = 0.25 + 0.75 * ((0.14 - 0.08) / (0.25 - 0.08))
    assert abs(node.left_turn_scale - expected_scale) < 1e-9
    assert abs(
        node.apply_motion_constraints(make_command(angular=0.5)).angular.z
        - (0.5 * expected_scale)
    ) < 1e-9
    assert node.apply_motion_constraints(make_command(angular=-0.5)).angular.z == -0.5
    assert node.get_nav_constraint_reason(
        make_command(angular=0.5),
        node.apply_motion_constraints(make_command(angular=0.5)),
    ) == 'left_turn_slowdown'

    node = make_safety_node()
    node.right_obstacle_active = True
    node.right_turn_scale = node.get_side_turn_scale(0.14)
    assert abs(
        node.apply_motion_constraints(make_command(angular=-0.5)).angular.z
        + (0.5 * expected_scale)
    ) < 1e-9
    assert node.apply_motion_constraints(make_command(angular=0.5)).angular.z == 0.5
    assert node.get_nav_constraint_reason(
        make_command(angular=-0.5),
        node.apply_motion_constraints(make_command(angular=-0.5)),
    ) == 'right_turn_slowdown'


def test_side_slowdown_preserves_a_translating_arcs_curvature():
    node = make_safety_node()
    node.left_obstacle_active = True
    node.left_turn_scale = 0.5

    safe = node.apply_motion_constraints(make_command(0.4, 0.6))

    assert safe.linear.x == 0.2
    assert safe.angular.z == 0.3
    assert safe.linear.x / 0.4 == safe.angular.z / 0.6
    assert node.get_nav_constraint_reason(
        make_command(0.4, 0.6), safe
    ) == 'left_turn_slowdown'


def test_side_turn_hard_stop_is_reserved_for_imminent_contact():
    node = make_safety_node()
    node.right_obstacle_active = True
    node.right_turn_scale = node.get_side_turn_scale(0.08)

    safe = node.apply_motion_constraints(make_command(angular=-0.5))
    assert safe.angular.z == 0.0
    assert node.get_nav_constraint_reason(
        make_command(angular=-0.5), safe
    ) == 'right_turn_stop'

    node.right_turn_scale = node.get_side_turn_scale(0.081)
    assert node.apply_motion_constraints(
        make_command(angular=-0.5)
    ).angular.z < 0.0

    node.right_turn_scale = 0.0
    stopped_arc = node.apply_motion_constraints(make_command(0.3, -0.5))
    assert stopped_arc.linear.x == 0.0
    assert stopped_arc.angular.z == 0.0
    assert node.get_nav_constraint_reason(
        make_command(0.3, -0.5), stopped_arc
    ) == 'right_turn_stop'


def test_rear_obstacle_blocks_reverse_but_allows_forward():
    node = make_safety_node()
    node.rear_obstacle_active = True
    stopped = node.apply_motion_constraints(make_command(-0.3, -0.4))
    assert stopped.linear.x == 0.0
    assert stopped.angular.z == 0.0

    forward = node.apply_motion_constraints(make_command(0.3, -0.4))
    assert forward.linear.x == 0.3
    assert forward.angular.z == -0.4

    # A rear stop does not prohibit a separately collision-checked pure turn.
    assert node.apply_motion_constraints(make_command(angular=-0.4)).angular.z == -0.4


def test_fresh_navigation_zero_does_not_create_an_extra_stop_hold():
    node = make_motion_tracking_node(make_command(linear=0.3))
    assert node.update_nav_motion_state(now=10.0, nav_active=True) is False
    assert node.nav_was_active is True

    node.latest_nav_cmd = Twist()
    assert node.update_nav_motion_state(now=10.1, nav_active=True) is False
    assert node.nav_was_active is False
    assert node.nav_stop_hold_until == 0.0


def test_lost_moving_navigation_stream_creates_a_stop_hold():
    node = make_motion_tracking_node(make_command(linear=0.3))
    assert node.update_nav_motion_state(now=20.0, nav_active=True) is False
    assert node.update_nav_motion_state(now=20.5, nav_active=False) is True

    assert node.nav_was_active is False
    assert node.nav_stop_hold_until == 21.0


def test_pi_timer_republishes_latest_lidar_constrained_navigation_command():
    node = make_motion_tracking_node(make_command(linear=0.4, angular=-0.2))
    node.latest_joy_cmd = Twist()
    node.startup_gate_open = True
    node.startup_quiet_until = 0.0
    node.scan_was_healthy = True
    node.is_nav_active = lambda: True
    node.is_joy_active = lambda: False
    node.obstacle_health_pub = RecordingPublisher()
    node.startup_gate_pub = RecordingPublisher()
    node.nav_gate_pub = RecordingPublisher()
    node.joy_gate_pub = RecordingPublisher()
    node.safety_cmd_pub = RecordingPublisher()
    node.speed_limit_scale_pub = RecordingPublisher()
    node.rear_speed_limit_scale_pub = RecordingPublisher()
    node.nav_safety_limited_pub = RecordingPublisher()
    node.nav_safety_reason_pub = RecordingPublisher()
    node.send_log = lambda *_args, **_kwargs: None

    node.publish_safety_hold()

    assert node.nav_gate_pub.messages[-1].linear.x == 0.4
    assert node.nav_gate_pub.messages[-1].angular.z == -0.2
    assert node.joy_gate_pub.messages == []


def test_expired_joystick_publishes_one_stop_then_releases_mux_ownership():
    node = make_motion_tracking_node(make_command(linear=0.4))
    node.latest_joy_cmd = make_command(linear=0.2)
    node.joy_was_active = True
    node.startup_quiet_until = 0.0
    node.scan_was_healthy = True
    node.is_nav_active = lambda: True
    node.is_joy_active = lambda: False
    node.obstacle_health_pub = RecordingPublisher()
    node.startup_gate_pub = RecordingPublisher()
    node.nav_gate_pub = RecordingPublisher()
    node.joy_gate_pub = RecordingPublisher()
    node.safety_cmd_pub = RecordingPublisher()
    node.speed_limit_scale_pub = RecordingPublisher()
    node.rear_speed_limit_scale_pub = RecordingPublisher()
    node.nav_safety_limited_pub = RecordingPublisher()
    node.nav_safety_reason_pub = RecordingPublisher()
    node.send_log = lambda *_args, **_kwargs: None

    node.publish_safety_hold()
    node.publish_safety_hold()

    assert len(node.joy_gate_pub.messages) == 1
    assert node.joy_gate_pub.messages[0].linear.x == 0.0
    assert node.joy_gate_pub.messages[0].angular.z == 0.0
    assert node.joy_was_active is False


def test_startup_gate_opens_automatically_after_a_quiet_period():
    node = make_safety_node(scan_healthy=True)
    node.startup_gate_open = False
    node.latest_joy_time = 1.0
    node.latest_nav_time = 1.0
    node.latest_joy_cmd = Twist()
    node.latest_nav_cmd = Twist()
    node.joy_was_active = False
    node.joystick_timeout_sec = 0.0
    node.nav_timeout_sec = 0.0
    node.startup_quiet_until = 0.0
    node.nav_was_active = False
    node.nav_stop_hold_until = 0.0
    node.front_obstacle_active = False
    node.rear_obstacle_active = False
    node.left_obstacle_active = False
    node.right_obstacle_active = False
    node.speed_limit_scale = 1.0
    node.scan_was_healthy = True
    node.obstacle_health_pub = RecordingPublisher()
    node.startup_gate_pub = RecordingPublisher()
    node.nav_gate_pub = RecordingPublisher()
    node.joy_gate_pub = RecordingPublisher()
    node.safety_cmd_pub = RecordingPublisher()
    node.speed_limit_scale_pub = RecordingPublisher()
    node.rear_speed_limit_scale_pub = RecordingPublisher()
    node.nav_safety_limited_pub = RecordingPublisher()
    node.nav_safety_reason_pub = RecordingPublisher()
    node.send_log = lambda *_args, **_kwargs: None

    node.publish_safety_hold()
    assert node.startup_gate_open is True
    assert node.latest_joy_time is None
    assert node.latest_nav_time is None
    assert node.nav_was_active is False
    assert node.joy_gate_pub.messages == []
