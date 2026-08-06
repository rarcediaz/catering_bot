#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


class ObstacleSafetyNode(Node):
    def __init__(self):
        super().__init__('obstacle_safety_node')

        self.declare_parameter('obstacle_stop_enabled', True)
        self.declare_parameter('obstacle_stop_distance_m', 0.25)
        self.declare_parameter('obstacle_stop_distance_max_m', 0.60)
        self.declare_parameter('obstacle_stop_speed_mps', 0.60)
        self.declare_parameter('obstacle_slowdown_margin_m', 0.15)
        # Laser-frame distances to the physical front and rear chassis edges.
        # The lidar is 0.73 m ahead of the drive axle (0.88 m in chassis).
        self.declare_parameter('front_stop_start_x_m', 0.12)
        self.declare_parameter('rear_stop_start_x_m', 0.88)
        self.declare_parameter('front_stop_width_m', 0.8596)
        self.declare_parameter('front_obstacle_confirmation_scans', 3)
        self.declare_parameter('front_obstacle_pending_speed_mps', 0.10)
        self.declare_parameter('rear_obstacle_confirmation_scans', 3)
        self.declare_parameter('rear_obstacle_pending_speed_mps', 0.10)
        self.declare_parameter('side_stop_distance_m', 0.25)
        self.declare_parameter('side_hard_stop_distance_m', 0.08)
        self.declare_parameter('side_min_speed_scale', 0.25)
        self.declare_parameter('side_stop_start_y_m', 0.34)
        self.declare_parameter('turn_in_place_linear_threshold_mps', 0.05)
        self.declare_parameter('turn_in_place_angular_threshold_radps', 0.20)
        self.declare_parameter('joystick_timeout_sec', 0.25)
        # Navigation crosses Wi-Fi/DDS. Keep enough jitter margin to bridge a
        # short network pause while this Pi-local node continues enforcing
        # fresh lidar constraints.
        self.declare_parameter('nav_timeout_sec', 0.50)
        self.declare_parameter('nav_stop_hold_sec', 0.5)
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('minimum_scan_ray_count', 100)
        self.declare_parameter('minimum_valid_scan_points', 10)
        self.declare_parameter('startup_quiet_sec', 5.0)
        self.declare_parameter('command_epsilon', 0.005)

        self.obstacle_stop_enabled = self.get_bool_parameter('obstacle_stop_enabled')
        self.obstacle_stop_distance_m = float(self.get_parameter('obstacle_stop_distance_m').value)
        self.obstacle_stop_distance_max_m = float(
            self.get_parameter('obstacle_stop_distance_max_m').value
        )
        self.obstacle_stop_speed_mps = float(self.get_parameter('obstacle_stop_speed_mps').value)
        self.obstacle_slowdown_margin_m = float(
            self.get_parameter('obstacle_slowdown_margin_m').value
        )
        self.front_stop_start_x_m = float(self.get_parameter('front_stop_start_x_m').value)
        self.rear_stop_start_x_m = float(self.get_parameter('rear_stop_start_x_m').value)
        self.front_stop_width_m = float(self.get_parameter('front_stop_width_m').value)
        self.front_stop_half_width_m = 0.5 * self.front_stop_width_m
        self.front_obstacle_confirmation_scans = max(
            1,
            int(self.get_parameter('front_obstacle_confirmation_scans').value),
        )
        self.front_obstacle_pending_speed_mps = max(
            0.0,
            float(self.get_parameter('front_obstacle_pending_speed_mps').value),
        )
        self.rear_obstacle_confirmation_scans = max(
            1,
            int(self.get_parameter('rear_obstacle_confirmation_scans').value),
        )
        self.rear_obstacle_pending_speed_mps = max(
            0.0,
            float(self.get_parameter('rear_obstacle_pending_speed_mps').value),
        )
        self.side_stop_distance_m = float(self.get_parameter('side_stop_distance_m').value)
        self.side_hard_stop_distance_m = max(
            0.0,
            min(
                float(self.get_parameter('side_hard_stop_distance_m').value),
                self.side_stop_distance_m,
            ),
        )
        self.side_min_speed_scale = max(
            0.0,
            min(float(self.get_parameter('side_min_speed_scale').value), 1.0),
        )
        self.side_stop_start_y_m = float(self.get_parameter('side_stop_start_y_m').value)
        self.turn_in_place_linear_threshold_mps = max(
            0.0,
            float(self.get_parameter('turn_in_place_linear_threshold_mps').value),
        )
        self.turn_in_place_angular_threshold_radps = max(
            0.0,
            float(self.get_parameter('turn_in_place_angular_threshold_radps').value),
        )
        self.joystick_timeout_sec = float(self.get_parameter('joystick_timeout_sec').value)
        self.nav_timeout_sec = float(self.get_parameter('nav_timeout_sec').value)
        self.nav_stop_hold_sec = float(self.get_parameter('nav_stop_hold_sec').value)
        self.scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        self.minimum_scan_ray_count = max(
            1,
            int(self.get_parameter('minimum_scan_ray_count').value),
        )
        self.minimum_valid_scan_points = max(
            1,
            int(self.get_parameter('minimum_valid_scan_points').value),
        )
        self.startup_quiet_sec = float(
            self.get_parameter('startup_quiet_sec').value
        )
        self.command_epsilon = float(self.get_parameter('command_epsilon').value)

        self.front_obstacle_active = False
        self.front_obstacle_pending = False
        self.front_obstacle_detection_streak = 0
        self.left_obstacle_active = False
        self.right_obstacle_active = False
        self.rear_obstacle_active = False
        self.rear_obstacle_pending = False
        self.rear_obstacle_detection_streak = 0
        self.closest_forward_clearance = math.inf
        self.closest_left_clearance = math.inf
        self.closest_right_clearance = math.inf
        self.closest_rear_clearance = math.inf
        self.dynamic_stop_distance_m = self.obstacle_stop_distance_m
        self.forward_speed_mps = 0.0
        self.speed_limit_scale = 1.0
        self.rear_speed_limit_scale = 1.0
        self.left_turn_scale = 1.0
        self.right_turn_scale = 1.0
        self.nav_constraint_reason = ''
        self.latest_joy_cmd = Twist()
        self.latest_nav_cmd = Twist()
        self.latest_joy_time = None
        self.latest_nav_time = None
        self.latest_scan_time = None
        self.latest_scan_quality_healthy = False
        self.joy_was_active = False
        self.nav_was_active = False
        self.nav_stop_hold_until = 0.0
        self.startup_gate_open = False
        self.startup_quiet_until = time.monotonic() + self.startup_quiet_sec
        self.scan_was_healthy = False

        self.front_range_pub = self.create_publisher(
            Float32,
            '/robot_health/closest_front_range_m',
            10
        )
        self.front_stop_distance_pub = self.create_publisher(
            Float32,
            '/robot_health/front_stop_distance_m',
            10
        )
        self.forward_speed_pub = self.create_publisher(
            Float32,
            '/robot_health/front_forward_speed_mps',
            10
        )
        self.front_obstacle_pub = self.create_publisher(
            Bool,
            '/robot_health/front_obstacle_active',
            10
        )
        self.left_obstacle_pub = self.create_publisher(
            Bool,
            '/robot_health/left_obstacle_active',
            10
        )
        self.right_obstacle_pub = self.create_publisher(
            Bool,
            '/robot_health/right_obstacle_active',
            10
        )
        self.rear_obstacle_pub = self.create_publisher(
            Bool,
            '/robot_health/rear_obstacle_active',
            10
        )
        self.left_range_pub = self.create_publisher(
            Float32,
            '/robot_health/closest_left_range_m',
            10
        )
        self.right_range_pub = self.create_publisher(
            Float32,
            '/robot_health/closest_right_range_m',
            10
        )
        self.rear_range_pub = self.create_publisher(
            Float32,
            '/robot_health/closest_rear_range_m',
            10
        )
        self.speed_limit_scale_pub = self.create_publisher(
            Float32,
            '/robot_health/front_speed_limit_scale',
            10
        )
        self.rear_speed_limit_scale_pub = self.create_publisher(
            Float32,
            '/robot_health/rear_speed_limit_scale',
            10
        )
        self.nav_safety_limited_pub = self.create_publisher(
            Bool,
            '/robot_health/navigation_safety_limited',
            10
        )
        self.nav_safety_reason_pub = self.create_publisher(
            String,
            '/robot_health/navigation_safety_reason',
            10
        )
        self.startup_gate_pub = self.create_publisher(
            Bool,
            '/robot_health/startup_gate_open',
            10
        )
        self.obstacle_health_pub = self.create_publisher(
            Bool,
            '/robot_health/obstacle_data_healthy',
            10
        )
        self.log_pub = self.create_publisher(String, '/robot_health/log', 10)
        self.nav_gate_pub = self.create_publisher(Twist, '/cmd_vel_nav_safe', 10)
        self.joy_gate_pub = self.create_publisher(Twist, '/cmd_vel_joy_safe', 10)
        self.safety_cmd_pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_joy', self.joy_cmd_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_nav_raw', self.nav_cmd_callback, 10)

        self.create_timer(0.05, self.publish_safety_hold)
        self.send_log(
            'Safety node started with motion inhibited until filtered lidar is '
            f'fresh and the raw command stream is quiet for '
            f'{self.startup_quiet_sec:.1f}s.'
        )

    def get_bool_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def send_log(self, text, is_crit=False):
        msg = String()
        msg.data = ('!!! ' if is_crit else '> ') + text
        self.log_pub.publish(msg)
        # Keep the same transition in the Pi service journal. The ROS health
        # topic is intentionally volatile, while journalctl is what operators
        # have available after a mission has already finished.
        if is_crit:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    def copy_twist(self, cmd):
        copied = Twist()
        copied.linear.x = cmd.linear.x
        copied.linear.y = cmd.linear.y
        copied.linear.z = cmd.linear.z
        copied.angular.x = cmd.angular.x
        copied.angular.y = cmd.angular.y
        copied.angular.z = cmd.angular.z
        return copied

    def is_joy_active(self):
        return (
            self.latest_joy_time is not None and
            (time.monotonic() - self.latest_joy_time) <= self.joystick_timeout_sec
        )

    def is_nav_active(self):
        return (
            self.latest_nav_time is not None and
            (time.monotonic() - self.latest_nav_time) <= self.nav_timeout_sec
        )

    def is_scan_healthy(self):
        if not self.obstacle_stop_enabled:
            return True
        return (
            self.latest_scan_time is not None and
            self.latest_scan_quality_healthy and
            (time.monotonic() - self.latest_scan_time) <= self.scan_timeout_sec
        )

    def scan_is_usable(self, msg):
        if len(msg.ranges) < self.minimum_scan_ray_count:
            return False
        valid_points = sum(
            1
            for distance in msg.ranges
            if (
                math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
            )
        )
        return valid_points >= self.minimum_valid_scan_points

    def is_twist_nonzero(self, cmd):
        return (
            abs(cmd.linear.x) > self.command_epsilon or
            abs(cmd.linear.y) > self.command_epsilon or
            abs(cmd.linear.z) > self.command_epsilon or
            abs(cmd.angular.x) > self.command_epsilon or
            abs(cmd.angular.y) > self.command_epsilon or
            abs(cmd.angular.z) > self.command_epsilon
        )

    def get_active_command(self):
        if not self.startup_gate_open or not self.is_scan_healthy():
            return None
        if self.is_joy_active():
            return self.latest_joy_cmd
        if self.is_nav_active():
            return self.latest_nav_cmd
        return None

    def update_nav_motion_state(self, now, nav_active):
        """Track a lost moving stream without holding intentional zero commands."""
        if nav_active and self.is_twist_nonzero(self.latest_nav_cmd):
            self.nav_was_active = True
        elif not nav_active and self.nav_was_active:
            self.nav_was_active = False
            self.nav_stop_hold_until = now + self.nav_stop_hold_sec
            return True
        elif nav_active:
            # A fresh zero is an intentional Nav2 stop. Forward it immediately
            # without turning it into an additional high-priority stop hold.
            self.nav_was_active = False
        return False

    def joy_cmd_callback(self, msg):
        self.latest_joy_cmd = msg
        self.latest_joy_time = time.monotonic()
        if not self.startup_gate_open:
            if self.is_twist_nonzero(msg):
                self.startup_quiet_until = (
                    time.monotonic() + self.startup_quiet_sec
                )
            self.joy_gate_pub.publish(Twist())
            self.safety_cmd_pub.publish(Twist())
            self.joy_was_active = False
            return
        if not self.is_scan_healthy():
            self.joy_gate_pub.publish(Twist())
            self.safety_cmd_pub.publish(Twist())
            self.joy_was_active = False
            return
        self.joy_gate_pub.publish(self.apply_motion_constraints(msg))
        self.joy_was_active = True

    def nav_cmd_callback(self, msg):
        self.latest_nav_cmd = msg
        self.latest_nav_time = time.monotonic()
        if not self.startup_gate_open:
            if self.is_twist_nonzero(msg):
                self.startup_quiet_until = (
                    time.monotonic() + self.startup_quiet_sec
                )
            self.nav_gate_pub.publish(Twist())
            self.safety_cmd_pub.publish(Twist())
            return
        if not self.is_scan_healthy():
            self.nav_gate_pub.publish(Twist())
            self.safety_cmd_pub.publish(Twist())
            return
        self.nav_gate_pub.publish(self.apply_navigation_motion_constraints(msg))

    def odom_callback(self, msg):
        self.forward_speed_mps = msg.twist.twist.linear.x

    def get_forward_speed_mps(self):
        # Stopping distance is a property of physical momentum, not of the raw
        # command Nav2 is still requesting. Including the raw command here can
        # latch a high-speed stop forever: the Pi outputs zero, odometry falls
        # to zero, but Nav2 keeps requesting the same speed so the stop distance
        # never shrinks enough to permit a controlled creep away from the
        # boundary.
        return max(0.0, abs(self.forward_speed_mps))

    def get_dynamic_stop_distance(self):
        """Return the fixed hard boundary and measured speed for diagnostics.

        The hard-stop boundary must not grow and shrink with odometry. Doing
        so creates a feedback loop: motion expands the boundary and triggers a
        stop, the stop contracts it, and the same command is admitted again.
        Speed is instead handled continuously by get_clearance_speed_scale(),
        while this fixed boundary remains the final fail-closed protection.
        """
        forward_speed = self.get_forward_speed_mps()
        return self.obstacle_stop_distance_m, forward_speed

    def get_clearance_speed_scale(self, clearance, command_speed):
        """Return a safe scale that converges instead of stop/start latching."""
        command_speed = abs(command_speed)
        if command_speed <= self.command_epsilon or not math.isfinite(clearance):
            return 1.0

        minimum_clearance = self.obstacle_stop_distance_m
        slowdown_clearance = (
            max(minimum_clearance, self.obstacle_stop_distance_max_m)
            + max(0.0, self.obstacle_slowdown_margin_m)
        )
        if clearance <= minimum_clearance:
            return 0.0
        if clearance >= slowdown_clearance:
            return 1.0

        clearance_span = max(slowdown_clearance - minimum_clearance, 1e-3)
        allowed_speed = self.obstacle_stop_speed_mps * (
            (clearance - minimum_clearance) / clearance_span
        )
        return max(0.0, min(1.0, allowed_speed / command_speed))

    def update_front_obstacle_confirmation(self, detected):
        """Reject isolated front returns without ignoring the first sample.

        A pending return is handled by get_front_speed_scale(), which
        immediately limits forward motion to a crawl. Consecutive returns
        confirm the hard stop. This preserves a response to a newly
        appearing obstacle while rejecting the one-scan open-space artifacts
        observed from the lidar.
        """
        if detected:
            self.front_obstacle_detection_streak += 1
        else:
            self.front_obstacle_detection_streak = 0

        confirmed = (
            detected and
            self.front_obstacle_detection_streak >=
            self.front_obstacle_confirmation_scans
        )
        self.front_obstacle_pending = detected and not confirmed
        return confirmed

    def get_front_speed_scale(self, command_speed):
        command_speed = abs(command_speed)
        if command_speed <= self.command_epsilon:
            return 1.0
        if self.front_obstacle_active:
            return 0.0
        if self.front_obstacle_pending:
            return max(
                0.0,
                min(1.0, self.front_obstacle_pending_speed_mps / command_speed),
            )
        return self.get_clearance_speed_scale(
            self.closest_forward_clearance,
            command_speed,
        )

    def update_rear_obstacle_confirmation(self, detected):
        """Apply the same short-burst rejection to the rear corridor."""
        if detected:
            self.rear_obstacle_detection_streak += 1
        else:
            self.rear_obstacle_detection_streak = 0

        confirmed = (
            detected and
            self.rear_obstacle_detection_streak >=
            self.rear_obstacle_confirmation_scans
        )
        self.rear_obstacle_pending = detected and not confirmed
        return confirmed

    def get_rear_speed_scale(self, command_speed):
        command_speed = abs(command_speed)
        if command_speed <= self.command_epsilon:
            return 1.0
        if self.rear_obstacle_active:
            return 0.0
        if self.rear_obstacle_pending:
            return max(
                0.0,
                min(1.0, self.rear_obstacle_pending_speed_mps / command_speed),
            )
        return self.get_clearance_speed_scale(
            self.closest_rear_clearance,
            command_speed,
        )

    def get_side_turn_scale(self, clearance):
        """Scale a turn toward a close side obstacle without stop/start chatter."""
        if not math.isfinite(clearance) or clearance >= self.side_stop_distance_m:
            return 1.0
        if clearance <= self.side_hard_stop_distance_m:
            return 0.0

        clearance_span = max(
            self.side_stop_distance_m - self.side_hard_stop_distance_m,
            1e-3,
        )
        progress = (
            (clearance - self.side_hard_stop_distance_m) / clearance_span
        )
        return self.side_min_speed_scale + (
            (1.0 - self.side_min_speed_scale) * progress
        )

    def has_active_motion_constraints(self):
        return (
            self.front_obstacle_active or
            self.left_obstacle_active or
            self.right_obstacle_active or
            self.rear_obstacle_active or
            self.speed_limit_scale < 1.0 or
            self.rear_speed_limit_scale < 1.0
        )

    def is_navigation_turn_in_place(self, cmd):
        """Identify MPPI's near-zero translation commands as normal turns."""
        return (
            abs(cmd.linear.x) <= self.turn_in_place_linear_threshold_mps and
            abs(cmd.angular.z) >= self.turn_in_place_angular_threshold_radps
        )

    def apply_navigation_motion_constraints(self, cmd):
        return self.apply_motion_constraints(
            cmd,
            normalize_turn_in_place=self.is_navigation_turn_in_place(cmd),
        )

    def apply_motion_constraints(self, cmd, normalize_turn_in_place=False):
        if not self.startup_gate_open or not self.is_scan_healthy():
            return Twist()
        limited = self.copy_twist(cmd)
        translation_scale = 1.0

        # MPPI samples continuous linear velocity, so an otherwise valid
        # in-place turn commonly arrives with a few cm/s of translation. Make
        # that Nav2-only command identical to a recovery spin before applying
        # the same Pi-local side-sweep protection. Manual commands retain their
        # requested curvature, and faster arcs still use front/rear limits.
        if normalize_turn_in_place:
            limited.linear.x = 0.0

        if limited.linear.x > 0.0:
            translation_scale = self.get_front_speed_scale(limited.linear.x)

        if limited.linear.x < 0.0:
            translation_scale = self.get_rear_speed_scale(limited.linear.x)

        # The central controller collision-checks a complete linear/angular
        # trajectory. Scaling only its translation turns a safe arc into a
        # different, unchecked
        # rotation about the axle, which can sweep the long trolley footprint
        # into an obstacle or keepout boundary. Preserve the selected curvature
        # whenever front/rear clearance limits a translating command. Pure
        # rotation commands remain governed by the independent side envelopes.
        limited.linear.x *= translation_scale
        if not normalize_turn_in_place and abs(cmd.linear.x) > self.command_epsilon:
            limited.angular.z *= translation_scale

        side_turn_scale = 1.0
        if cmd.angular.z > 0.0:
            side_turn_scale = self.left_turn_scale
        elif cmd.angular.z < 0.0:
            side_turn_scale = self.right_turn_scale

        # Side constraints must not straighten a collision-checked arc. That
        # was the remaining path-deformation case: suppressing only angular
        # velocity allowed the axle to continue on a different trajectory and
        # carry the trolley footprint into occupied/keepout cells. Scale the
        # complete twist for a translating arc; for a pure rotation, only the
        # angular component exists to constrain.
        if not normalize_turn_in_place and abs(cmd.linear.x) > self.command_epsilon:
            limited.linear.x *= side_turn_scale
            limited.angular.z *= side_turn_scale
        else:
            limited.angular.z *= side_turn_scale

        return limited

    def get_nav_constraint_reason(self, raw_cmd, safe_cmd):
        if not self.startup_gate_open:
            return 'startup_gate'
        if not self.is_scan_healthy():
            return 'scan_stale'
        if self.is_navigation_turn_in_place(raw_cmd):
            if (
                raw_cmd.angular.z > self.command_epsilon and
                safe_cmd.angular.z < raw_cmd.angular.z - self.command_epsilon
            ):
                if abs(safe_cmd.angular.z) <= self.command_epsilon:
                    return 'left_turn_stop'
                return 'left_turn_slowdown'
            if (
                raw_cmd.angular.z < -self.command_epsilon and
                safe_cmd.angular.z > raw_cmd.angular.z + self.command_epsilon
            ):
                if abs(safe_cmd.angular.z) <= self.command_epsilon:
                    return 'right_turn_stop'
                return 'right_turn_slowdown'
            if (
                abs(raw_cmd.linear.x) > self.command_epsilon and
                abs(safe_cmd.linear.x) <= self.command_epsilon
            ):
                return 'turn_in_place'
        if raw_cmd.linear.x > self.command_epsilon:
            if self.front_obstacle_active:
                return 'front_stop'
            if self.get_front_speed_scale(raw_cmd.linear.x) < 1.0:
                return 'front_slowdown'
        elif raw_cmd.linear.x < -self.command_epsilon:
            if self.rear_obstacle_active:
                return 'rear_stop'
            if self.get_rear_speed_scale(raw_cmd.linear.x) < 1.0:
                return 'rear_slowdown'
        if (
            raw_cmd.angular.z > self.command_epsilon and
            safe_cmd.angular.z < raw_cmd.angular.z - self.command_epsilon
        ):
            if abs(safe_cmd.angular.z) <= self.command_epsilon:
                return 'left_turn_stop'
            return 'left_turn_slowdown'
        if (
            raw_cmd.angular.z < -self.command_epsilon and
            safe_cmd.angular.z > raw_cmd.angular.z + self.command_epsilon
        ):
            if abs(safe_cmd.angular.z) <= self.command_epsilon:
                return 'right_turn_stop'
            return 'right_turn_slowdown'
        return ''

    def publish_nav_constraint_state(self, nav_active, safe_cmd):
        reason = ''
        if nav_active and self.is_twist_nonzero(self.latest_nav_cmd):
            reason = self.get_nav_constraint_reason(self.latest_nav_cmd, safe_cmd)

        self.nav_safety_limited_pub.publish(Bool(data=bool(reason)))
        self.nav_safety_reason_pub.publish(String(data=reason))
        if reason == self.nav_constraint_reason:
            return

        if reason:
            self.send_log(
                'Navigation safety constraint active: '
                f'{reason} (raw linear={self.latest_nav_cmd.linear.x:.2f}, '
                f'angular={self.latest_nav_cmd.angular.z:.2f}; '
                f'safe linear={safe_cmd.linear.x:.2f}, '
                f'angular={safe_cmd.angular.z:.2f}).',
                is_crit=True,
            )
        elif self.nav_constraint_reason:
            self.send_log('Navigation safety constraint cleared.')
        self.nav_constraint_reason = reason

    def publish_safety_hold(self):
        now = time.monotonic()
        scan_healthy = self.is_scan_healthy()
        self.obstacle_health_pub.publish(Bool(data=scan_healthy))

        if scan_healthy != self.scan_was_healthy:
            if scan_healthy:
                self.send_log('Fresh filtered lidar data available to obstacle safety.')
            elif self.latest_scan_time is not None:
                self.send_log(
                    'Filtered lidar data is stale; all motion is blocked.',
                    is_crit=True,
                )
            self.scan_was_healthy = scan_healthy

        active_nonzero_command = (
            (
                self.is_nav_active()
                and self.is_twist_nonzero(self.latest_nav_cmd)
            )
            or (
                self.is_joy_active()
                and self.is_twist_nonzero(self.latest_joy_cmd)
            )
        )
        if (
            not self.startup_gate_open
            and scan_healthy
            and not active_nonzero_command
            and now >= self.startup_quiet_until
        ):
            self.startup_gate_open = True
            self.latest_joy_time = None
            self.latest_nav_time = None
            self.joy_was_active = False
            self.nav_was_active = False
            self.send_log(
                'Startup safety gate opened automatically; fresh commands may '
                'now pass.'
            )

        self.startup_gate_pub.publish(Bool(data=self.startup_gate_open))
        if not self.startup_gate_open or not scan_healthy:
            self.nav_gate_pub.publish(Twist())
            self.joy_gate_pub.publish(Twist())
            self.safety_cmd_pub.publish(Twist())
            self.joy_was_active = False
            self.speed_limit_scale_pub.publish(Float32(data=0.0))
            self.rear_speed_limit_scale_pub.publish(Float32(data=0.0))
            self.publish_nav_constraint_state(self.is_nav_active(), Twist())
            return

        joy_active = self.is_joy_active()
        nav_active = self.is_nav_active()

        if self.update_nav_motion_state(now, nav_active):
            self.send_log(
                'Navigation command stream timed out after '
                f'{self.nav_timeout_sec:.2f}s; Pi-local stop hold engaged.',
                is_crit=True,
            )

        safe_nav_cmd = Twist()
        if nav_active:
            # Re-publish at the Pi-local 20 Hz safety rate. During a brief DDS
            # gap this keeps the downstream mux/controller fed while every
            # command is still constrained by the latest local lidar scan.
            safe_nav_cmd = self.apply_navigation_motion_constraints(
                self.latest_nav_cmd
            )
            self.nav_gate_pub.publish(safe_nav_cmd)
        else:
            self.nav_gate_pub.publish(Twist())
        self.publish_nav_constraint_state(nav_active, safe_nav_cmd)
        if joy_active:
            self.joy_gate_pub.publish(
                self.apply_motion_constraints(self.latest_joy_cmd)
            )
            self.joy_was_active = True
        elif self.joy_was_active:
            # Release manual ownership with one explicit stop, then remain
            # silent. Continuously publishing inactive joystick zeros would
            # starve lower-priority navigation at twist_mux.
            self.joy_gate_pub.publish(Twist())
            self.joy_was_active = False

        if now < self.nav_stop_hold_until:
            self.safety_cmd_pub.publish(Twist())

        active_cmd = self.get_active_command()
        if active_cmd is None:
            if self.has_active_motion_constraints():
                self.safety_cmd_pub.publish(Twist())
            self.speed_limit_scale_pub.publish(Float32(data=float(self.speed_limit_scale)))
            self.rear_speed_limit_scale_pub.publish(
                Float32(data=float(self.rear_speed_limit_scale))
            )
            return

        if self.has_active_motion_constraints():
            if joy_active:
                safe_active_cmd = self.apply_motion_constraints(active_cmd)
            else:
                safe_active_cmd = self.apply_navigation_motion_constraints(
                    active_cmd
                )
            self.safety_cmd_pub.publish(safe_active_cmd)

        self.speed_limit_scale_pub.publish(Float32(data=float(self.speed_limit_scale)))
        self.rear_speed_limit_scale_pub.publish(
            Float32(data=float(self.rear_speed_limit_scale))
        )

    def scan_callback(self, msg):
        if not self.obstacle_stop_enabled:
            return
        self.latest_scan_time = time.monotonic()
        self.latest_scan_quality_healthy = self.scan_is_usable(msg)

        closest_forward_clearance = math.inf
        closest_left_clearance = math.inf
        closest_right_clearance = math.inf
        closest_rear_clearance = math.inf
        closest_rear_point = None

        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + (index * msg.angle_increment)

            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            point_x = distance * math.cos(angle)
            point_y = distance * math.sin(angle)

            if (
                point_x >= self.front_stop_start_x_m
                and abs(point_y) <= self.front_stop_half_width_m
            ):
                forward_clearance = point_x - self.front_stop_start_x_m
                closest_forward_clearance = min(closest_forward_clearance, forward_clearance)

            if (
                point_x <= -self.rear_stop_start_x_m
                and abs(point_y) <= self.front_stop_half_width_m
            ):
                rear_clearance = (-point_x) - self.rear_stop_start_x_m
                if rear_clearance < closest_rear_clearance:
                    closest_rear_clearance = rear_clearance
                    closest_rear_point = (index, point_x, point_y, distance)

            if -self.rear_stop_start_x_m <= point_x <= self.front_stop_start_x_m:
                if point_y >= self.side_stop_start_y_m:
                    left_clearance = point_y - self.side_stop_start_y_m
                    closest_left_clearance = min(closest_left_clearance, left_clearance)
                elif point_y <= -self.side_stop_start_y_m:
                    right_clearance = (-point_y) - self.side_stop_start_y_m
                    closest_right_clearance = min(closest_right_clearance, right_clearance)

        dynamic_stop_distance, forward_speed = self.get_dynamic_stop_distance()
        raw_front_obstacle_detected = (
            closest_forward_clearance <= dynamic_stop_distance
        )
        front_obstacle_detected = self.update_front_obstacle_confirmation(
            raw_front_obstacle_detected
        )
        left_obstacle_detected = closest_left_clearance <= self.side_stop_distance_m
        right_obstacle_detected = closest_right_clearance <= self.side_stop_distance_m
        raw_rear_obstacle_detected = closest_rear_clearance <= dynamic_stop_distance
        rear_obstacle_detected = self.update_rear_obstacle_confirmation(
            raw_rear_obstacle_detected
        )

        previous_front = self.front_obstacle_active
        previous_left = self.left_obstacle_active
        previous_right = self.right_obstacle_active
        previous_rear = self.rear_obstacle_active

        self.closest_forward_clearance = closest_forward_clearance
        self.closest_left_clearance = closest_left_clearance
        self.closest_right_clearance = closest_right_clearance
        self.closest_rear_clearance = closest_rear_clearance
        self.dynamic_stop_distance_m = dynamic_stop_distance
        self.front_obstacle_active = front_obstacle_detected
        self.left_obstacle_active = left_obstacle_detected
        self.right_obstacle_active = right_obstacle_detected
        self.rear_obstacle_active = rear_obstacle_detected
        self.left_turn_scale = self.get_side_turn_scale(closest_left_clearance)
        self.right_turn_scale = self.get_side_turn_scale(closest_right_clearance)

        active_cmd = self.get_active_command()
        if active_cmd is not None and active_cmd.linear.x > self.command_epsilon:
            self.speed_limit_scale = self.get_front_speed_scale(active_cmd.linear.x)
        else:
            self.speed_limit_scale = 1.0
        if active_cmd is not None and active_cmd.linear.x < -self.command_epsilon:
            self.rear_speed_limit_scale = self.get_rear_speed_scale(
                active_cmd.linear.x
            )
        else:
            self.rear_speed_limit_scale = 1.0

        reported_range = (
            closest_forward_clearance
            if math.isfinite(closest_forward_clearance)
            else -1.0
        )
        self.front_range_pub.publish(Float32(data=float(reported_range)))
        self.front_stop_distance_pub.publish(Float32(data=float(dynamic_stop_distance)))
        self.forward_speed_pub.publish(Float32(data=float(forward_speed)))
        self.front_obstacle_pub.publish(Bool(data=front_obstacle_detected))
        self.left_obstacle_pub.publish(Bool(data=left_obstacle_detected))
        self.right_obstacle_pub.publish(Bool(data=right_obstacle_detected))
        self.rear_obstacle_pub.publish(Bool(data=rear_obstacle_detected))
        self.left_range_pub.publish(Float32(data=float(
            closest_left_clearance if math.isfinite(closest_left_clearance) else -1.0
        )))
        self.right_range_pub.publish(Float32(data=float(
            closest_right_clearance if math.isfinite(closest_right_clearance) else -1.0
        )))
        self.rear_range_pub.publish(Float32(data=float(
            closest_rear_clearance if math.isfinite(closest_rear_clearance) else -1.0
        )))

        if self.is_nav_active():
            self.nav_gate_pub.publish(
                self.apply_navigation_motion_constraints(self.latest_nav_cmd)
            )
        if self.is_joy_active():
            self.joy_gate_pub.publish(self.apply_motion_constraints(self.latest_joy_cmd))
            self.joy_was_active = True

        self.log_obstacle_transition(
            front_obstacle_detected,
            previous_front,
            f'Front obstacle stop active ({closest_forward_clearance:.2f}m <= '
            f'{dynamic_stop_distance:.2f}m at {forward_speed:.2f} m/s)',
            'Front obstacle cleared.'
        )
        self.log_obstacle_transition(
            left_obstacle_detected,
            previous_left,
            f'Left turn slowdown active ({closest_left_clearance:.2f}m <= '
            f'{self.side_stop_distance_m:.2f}m; hard stop at '
            f'{self.side_hard_stop_distance_m:.2f}m).',
            'Left side clear.'
        )
        self.log_obstacle_transition(
            right_obstacle_detected,
            previous_right,
            f'Right turn slowdown active ({closest_right_clearance:.2f}m <= '
            f'{self.side_stop_distance_m:.2f}m; hard stop at '
            f'{self.side_hard_stop_distance_m:.2f}m).',
            'Right side clear.'
        )
        rear_point_detail = ''
        if closest_rear_point is not None:
            rear_point_detail = (
                f'; beam={closest_rear_point[0]}, '
                f'laser_x={closest_rear_point[1]:.3f}, '
                f'laser_y={closest_rear_point[2]:.3f}, '
                f'range={closest_rear_point[3]:.3f}'
            )
        self.log_obstacle_transition(
            rear_obstacle_detected,
            previous_rear,
            f'Rear motion blocked ({closest_rear_clearance:.2f}m <= '
            f'{dynamic_stop_distance:.2f}m{rear_point_detail}).',
            'Rear area clear.'
        )

    def log_obstacle_transition(self, detected, was_detected, active_text, clear_text):
        if detected and not was_detected:
            self.send_log(active_text, is_crit=True)
        elif was_detected and not detected:
            self.send_log(clear_text)


def main():
    rclpy.init()
    node = ObstacleSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
