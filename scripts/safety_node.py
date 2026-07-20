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
        self.declare_parameter('obstacle_stop_distance_m', 0.20)
        self.declare_parameter('obstacle_stop_distance_max_m', 0.60)
        self.declare_parameter('obstacle_stop_speed_mps', 0.60)
        self.declare_parameter('obstacle_slowdown_margin_m', 0.15)
        self.declare_parameter('front_stop_start_x_m', 0.09)
        self.declare_parameter('rear_stop_start_x_m', 0.91)
        self.declare_parameter('front_stop_width_m', 0.8596)
        self.declare_parameter('side_stop_distance_m', 0.25)
        self.declare_parameter('side_stop_start_y_m', 0.34)
        self.declare_parameter('joystick_timeout_sec', 0.5)
        self.declare_parameter('nav_timeout_sec', 0.25)
        self.declare_parameter('nav_stop_hold_sec', 0.5)
        self.declare_parameter('command_epsilon', 0.005)

        self.obstacle_stop_enabled = self.get_bool_parameter('obstacle_stop_enabled')
        self.obstacle_stop_distance_m = float(self.get_parameter('obstacle_stop_distance_m').value)
        self.obstacle_stop_distance_max_m = float(self.get_parameter('obstacle_stop_distance_max_m').value)
        self.obstacle_stop_speed_mps = float(self.get_parameter('obstacle_stop_speed_mps').value)
        self.obstacle_slowdown_margin_m = float(self.get_parameter('obstacle_slowdown_margin_m').value)
        self.front_stop_start_x_m = float(self.get_parameter('front_stop_start_x_m').value)
        self.rear_stop_start_x_m = float(self.get_parameter('rear_stop_start_x_m').value)
        self.front_stop_width_m = float(self.get_parameter('front_stop_width_m').value)
        self.front_stop_half_width_m = 0.5 * self.front_stop_width_m
        self.side_stop_distance_m = float(self.get_parameter('side_stop_distance_m').value)
        self.side_stop_start_y_m = float(self.get_parameter('side_stop_start_y_m').value)
        self.joystick_timeout_sec = float(self.get_parameter('joystick_timeout_sec').value)
        self.nav_timeout_sec = float(self.get_parameter('nav_timeout_sec').value)
        self.nav_stop_hold_sec = float(self.get_parameter('nav_stop_hold_sec').value)
        self.command_epsilon = float(self.get_parameter('command_epsilon').value)

        self.front_obstacle_active = False
        self.left_obstacle_active = False
        self.right_obstacle_active = False
        self.rear_obstacle_active = False
        self.closest_forward_clearance = math.inf
        self.closest_left_clearance = math.inf
        self.closest_right_clearance = math.inf
        self.closest_rear_clearance = math.inf
        self.dynamic_stop_distance_m = self.obstacle_stop_distance_m
        self.forward_speed_mps = 0.0
        self.speed_limit_scale = 1.0
        self.latest_joy_cmd = Twist()
        self.latest_nav_cmd = Twist()
        self.latest_joy_time = None
        self.latest_nav_time = None
        self.nav_was_active = False
        self.nav_stop_hold_until = 0.0

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
        self.speed_limit_scale_pub = self.create_publisher(
            Float32,
            '/robot_health/front_speed_limit_scale',
            10
        )
        self.log_pub = self.create_publisher(String, '/robot_health/log', 10)
        self.nav_gate_pub = self.create_publisher(Twist, '/cmd_vel_nav_safe', 10)
        self.safety_cmd_pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_joy', self.joy_cmd_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_nav_raw', self.nav_cmd_callback, 10)

        self.create_timer(0.05, self.publish_safety_hold)
        self.send_log('Obstacle safety node active.')

    def get_bool_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def send_log(self, text, is_crit=False):
        msg = String()
        msg.data = ('!!! ' if is_crit else '> ') + text
        self.log_pub.publish(msg)

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
        if self.is_joy_active():
            return self.latest_joy_cmd
        if self.is_nav_active():
            return self.latest_nav_cmd
        return None

    def joy_cmd_callback(self, msg):
        self.latest_joy_cmd = msg
        self.latest_joy_time = time.monotonic()

    def nav_cmd_callback(self, msg):
        self.latest_nav_cmd = msg
        self.latest_nav_time = time.monotonic()
        self.nav_gate_pub.publish(self.apply_motion_constraints(msg))

    def odom_callback(self, msg):
        self.forward_speed_mps = msg.twist.twist.linear.x

    def get_forward_speed_mps(self):
        active_cmd = self.get_active_command()
        cmd_speed = abs(active_cmd.linear.x) if active_cmd is not None else 0.0
        return max(0.0, abs(self.forward_speed_mps), cmd_speed)

    def get_dynamic_stop_distance(self):
        forward_speed = self.get_forward_speed_mps()
        max_distance = max(self.obstacle_stop_distance_m, self.obstacle_stop_distance_max_m)
        if self.obstacle_stop_speed_mps <= 1e-3 or max_distance <= self.obstacle_stop_distance_m:
            return self.obstacle_stop_distance_m, forward_speed

        speed_ratio = min(forward_speed / self.obstacle_stop_speed_mps, 1.0)
        stop_distance = (
            self.obstacle_stop_distance_m +
            speed_ratio * (max_distance - self.obstacle_stop_distance_m)
        )
        return stop_distance, forward_speed

    def has_active_motion_constraints(self):
        return (
            self.front_obstacle_active or
            self.left_obstacle_active or
            self.right_obstacle_active or
            self.rear_obstacle_active or
            self.speed_limit_scale < 1.0
        )

    def apply_motion_constraints(self, cmd):
        limited = self.copy_twist(cmd)

        if self.front_obstacle_active and limited.linear.x > 0.0:
            limited.linear.x = 0.0
        elif self.speed_limit_scale < 1.0 and limited.linear.x > 0.0:
            limited.linear.x *= self.speed_limit_scale

        if self.rear_obstacle_active and limited.linear.x < 0.0:
            limited.linear.x = 0.0

        if self.left_obstacle_active and limited.angular.z > 0.0:
            limited.angular.z = 0.0

        if self.right_obstacle_active and limited.angular.z < 0.0:
            limited.angular.z = 0.0

        return limited

    def publish_safety_hold(self):
        now = time.monotonic()
        nav_active = self.is_nav_active()

        if nav_active and self.is_twist_nonzero(self.latest_nav_cmd):
            self.nav_was_active = True
        elif self.nav_was_active:
            self.nav_was_active = False
            self.nav_stop_hold_until = now + self.nav_stop_hold_sec

        if not nav_active:
            self.nav_gate_pub.publish(Twist())

        if now < self.nav_stop_hold_until:
            self.safety_cmd_pub.publish(Twist())

        active_cmd = self.get_active_command()
        if active_cmd is None:
            if self.has_active_motion_constraints():
                self.safety_cmd_pub.publish(Twist())
            self.speed_limit_scale_pub.publish(Float32(data=float(self.speed_limit_scale)))
            return

        if self.has_active_motion_constraints():
            self.safety_cmd_pub.publish(self.apply_motion_constraints(active_cmd))

        self.speed_limit_scale_pub.publish(Float32(data=float(self.speed_limit_scale)))

    def scan_callback(self, msg):
        if not self.obstacle_stop_enabled:
            return

        closest_forward_clearance = math.inf
        closest_left_clearance = math.inf
        closest_right_clearance = math.inf
        closest_rear_clearance = math.inf

        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + (index * msg.angle_increment)

            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            point_x = distance * math.cos(angle)
            point_y = distance * math.sin(angle)

            if point_x >= self.front_stop_start_x_m and abs(point_y) <= self.front_stop_half_width_m:
                forward_clearance = point_x - self.front_stop_start_x_m
                closest_forward_clearance = min(closest_forward_clearance, forward_clearance)

            if point_x <= -self.rear_stop_start_x_m and abs(point_y) <= self.front_stop_half_width_m:
                rear_clearance = (-point_x) - self.rear_stop_start_x_m
                closest_rear_clearance = min(closest_rear_clearance, rear_clearance)

            if -self.rear_stop_start_x_m <= point_x <= self.front_stop_start_x_m:
                if point_y >= self.side_stop_start_y_m:
                    left_clearance = point_y - self.side_stop_start_y_m
                    closest_left_clearance = min(closest_left_clearance, left_clearance)
                elif point_y <= -self.side_stop_start_y_m:
                    right_clearance = (-point_y) - self.side_stop_start_y_m
                    closest_right_clearance = min(closest_right_clearance, right_clearance)

        dynamic_stop_distance, forward_speed = self.get_dynamic_stop_distance()
        slowdown_distance = dynamic_stop_distance + max(0.0, self.obstacle_slowdown_margin_m)
        front_obstacle_detected = closest_forward_clearance <= dynamic_stop_distance
        left_obstacle_detected = closest_left_clearance <= self.side_stop_distance_m
        right_obstacle_detected = closest_right_clearance <= self.side_stop_distance_m
        rear_obstacle_detected = closest_rear_clearance <= dynamic_stop_distance

        if math.isfinite(closest_forward_clearance) and closest_forward_clearance < slowdown_distance:
            margin = max(self.obstacle_slowdown_margin_m, 1e-3)
            speed_limit_scale = max(
                0.0,
                min(1.0, (closest_forward_clearance - dynamic_stop_distance) / margin)
            )
        else:
            speed_limit_scale = 1.0

        previous_front = self.front_obstacle_active
        previous_left = self.left_obstacle_active
        previous_right = self.right_obstacle_active
        previous_rear = self.rear_obstacle_active

        self.closest_forward_clearance = closest_forward_clearance
        self.closest_left_clearance = closest_left_clearance
        self.closest_right_clearance = closest_right_clearance
        self.closest_rear_clearance = closest_rear_clearance
        self.dynamic_stop_distance_m = dynamic_stop_distance
        self.speed_limit_scale = 0.0 if front_obstacle_detected else speed_limit_scale
        self.front_obstacle_active = front_obstacle_detected
        self.left_obstacle_active = left_obstacle_detected
        self.right_obstacle_active = right_obstacle_detected
        self.rear_obstacle_active = rear_obstacle_detected

        reported_range = closest_forward_clearance if math.isfinite(closest_forward_clearance) else -1.0
        self.front_range_pub.publish(Float32(data=float(reported_range)))
        self.front_stop_distance_pub.publish(Float32(data=float(dynamic_stop_distance)))
        self.forward_speed_pub.publish(Float32(data=float(forward_speed)))
        self.front_obstacle_pub.publish(Bool(data=front_obstacle_detected))

        if self.is_nav_active():
            self.nav_gate_pub.publish(self.apply_motion_constraints(self.latest_nav_cmd))

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
            f'Left turn blocked ({closest_left_clearance:.2f}m <= {self.side_stop_distance_m:.2f}m).',
            'Left side clear.'
        )
        self.log_obstacle_transition(
            right_obstacle_detected,
            previous_right,
            f'Right turn blocked ({closest_right_clearance:.2f}m <= {self.side_stop_distance_m:.2f}m).',
            'Right side clear.'
        )
        self.log_obstacle_transition(
            rear_obstacle_detected,
            previous_rear,
            f'Rear motion blocked ({closest_rear_clearance:.2f}m <= {dynamic_stop_distance:.2f}m).',
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
