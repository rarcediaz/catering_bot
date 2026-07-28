"""Unit tests for the motion-constraint decisions in the Pi safety node."""

import sys
from pathlib import Path

from geometry_msgs.msg import Twist
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / 'scripts'))

from safety_node import ObstacleSafetyNode  # noqa: E402


def make_safety_node(scan_healthy=True):
    node = ObstacleSafetyNode.__new__(ObstacleSafetyNode)
    node.startup_gate_open = True
    node.front_obstacle_active = False
    node.rear_obstacle_active = False
    node.left_obstacle_active = False
    node.right_obstacle_active = False
    node.speed_limit_scale = 1.0
    node.is_scan_healthy = lambda: scan_healthy
    return node


def make_command(linear=0.0, angular=0.0):
    command = Twist()
    command.linear.x = linear
    command.angular.z = angular
    return command


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_closed_gate_and_stale_scan_each_fail_closed():
    node = make_safety_node()
    node.startup_gate_open = False
    stopped = node.apply_motion_constraints(make_command(0.4, 0.5))
    assert stopped.linear.x == 0.0
    assert stopped.angular.z == 0.0

    node = make_safety_node(scan_healthy=False)
    stopped = node.apply_motion_constraints(make_command(0.4, 0.5))
    assert stopped.linear.x == 0.0
    assert stopped.angular.z == 0.0


def test_front_obstacle_blocks_forward_but_allows_reverse():
    node = make_safety_node()
    node.front_obstacle_active = True
    assert node.apply_motion_constraints(make_command(0.4)).linear.x == 0.0
    assert node.apply_motion_constraints(make_command(-0.2)).linear.x == -0.2


def test_front_slowdown_scales_only_forward_motion():
    node = make_safety_node()
    node.speed_limit_scale = 0.25
    assert node.apply_motion_constraints(make_command(0.4)).linear.x == 0.1
    assert node.apply_motion_constraints(make_command(-0.4)).linear.x == -0.4


def test_side_obstacles_block_only_turns_toward_them():
    node = make_safety_node()
    node.left_obstacle_active = True
    assert node.apply_motion_constraints(make_command(angular=0.5)).angular.z == 0.0
    assert node.apply_motion_constraints(make_command(angular=-0.5)).angular.z == -0.5

    node = make_safety_node()
    node.right_obstacle_active = True
    assert node.apply_motion_constraints(make_command(angular=-0.5)).angular.z == 0.0
    assert node.apply_motion_constraints(make_command(angular=0.5)).angular.z == 0.5


def test_rear_obstacle_blocks_reverse_but_allows_forward():
    node = make_safety_node()
    node.rear_obstacle_active = True
    assert node.apply_motion_constraints(make_command(-0.3)).linear.x == 0.0
    assert node.apply_motion_constraints(make_command(0.3)).linear.x == 0.3


def test_startup_gate_opens_automatically_after_a_quiet_period():
    node = make_safety_node(scan_healthy=True)
    node.startup_gate_open = False
    node.latest_joy_time = 1.0
    node.latest_nav_time = 1.0
    node.latest_joy_cmd = Twist()
    node.latest_nav_cmd = Twist()
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
    node.send_log = lambda *_args, **_kwargs: None

    node.publish_safety_hold()
    assert node.startup_gate_open is True
    assert node.latest_joy_time is None
    assert node.latest_nav_time is None
    assert node.nav_was_active is False
