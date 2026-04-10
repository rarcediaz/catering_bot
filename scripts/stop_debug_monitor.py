#!/usr/bin/env python3
import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class StopDebugMonitor(Node):
    def __init__(self):
        super().__init__('stop_debug_monitor')

        default_path = os.path.expanduser('~/stop_debug_log.csv')
        self.declare_parameter('log_path', default_path)
        self.declare_parameter('log_hz', 5.0)

        self.log_path = os.path.expanduser(str(self.get_parameter('log_path').value))
        self.log_hz = float(self.get_parameter('log_hz').value)

        self.mode = 'UNKNOWN'
        self.front_obstacle_active = False
        self.closest_front_range_m = math.nan
        self.front_stop_distance_m = math.nan
        self.front_forward_speed_mps = math.nan
        self.front_speed_limit_scale = math.nan

        self.cmd_vel_nav = Twist()
        self.cmd_vel_nav_raw = Twist()
        self.cmd_vel_safety = Twist()
        self.cmd_vel_mux = Twist()
        self.odom = Odometry()

        self.last_warning_time = 0.0

        self.create_subscription(String, '/robot_state/mode', self.mode_callback, 10)
        self.create_subscription(Bool, '/robot_health/front_obstacle_active', self.front_obstacle_callback, 10)
        self.create_subscription(Float32, '/robot_health/closest_front_range_m', self.front_range_callback, 10)
        self.create_subscription(Float32, '/robot_health/front_stop_distance_m', self.front_stop_distance_callback, 10)
        self.create_subscription(Float32, '/robot_health/front_forward_speed_mps', self.front_forward_speed_callback, 10)
        self.create_subscription(Float32, '/robot_health/front_speed_limit_scale', self.front_speed_limit_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_nav_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_nav_raw', self.cmd_vel_nav_raw_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_safety', self.cmd_vel_safety_callback, 10)
        self.create_subscription(Twist, '/diff_cont/cmd_vel_unstamped', self.cmd_vel_mux_callback, 10)
        self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)

        os.makedirs(os.path.dirname(self.log_path) or '.', exist_ok=True)
        self.csv_file = open(self.log_path, 'a', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        if self.csv_file.tell() == 0:
            self.csv_writer.writerow([
                'wall_time',
                'mode',
                'front_obstacle_active',
                'closest_front_range_m',
                'front_stop_distance_m',
                'front_forward_speed_mps',
                'front_speed_limit_scale',
                'cmd_vel_nav_raw_linear_x',
                'cmd_vel_nav_raw_angular_z',
                'cmd_vel_nav_linear_x',
                'cmd_vel_nav_angular_z',
                'cmd_vel_safety_linear_x',
                'cmd_vel_safety_angular_z',
                'cmd_vel_mux_linear_x',
                'cmd_vel_mux_angular_z',
                'odom_linear_x',
                'odom_angular_z',
            ])
            self.csv_file.flush()

        self.create_timer(1.0 / max(self.log_hz, 1.0), self.log_snapshot)
        self.get_logger().info(f'Stop debug monitor logging to {self.log_path}')

    def mode_callback(self, msg: String):
        self.mode = msg.data

    def front_obstacle_callback(self, msg: Bool):
        self.front_obstacle_active = msg.data

    def front_range_callback(self, msg: Float32):
        self.closest_front_range_m = msg.data

    def front_stop_distance_callback(self, msg: Float32):
        self.front_stop_distance_m = msg.data

    def front_forward_speed_callback(self, msg: Float32):
        self.front_forward_speed_mps = msg.data

    def front_speed_limit_callback(self, msg: Float32):
        self.front_speed_limit_scale = msg.data

    def cmd_vel_nav_callback(self, msg: Twist):
        self.cmd_vel_nav = msg

    def cmd_vel_nav_raw_callback(self, msg: Twist):
        self.cmd_vel_nav_raw = msg

    def cmd_vel_safety_callback(self, msg: Twist):
        self.cmd_vel_safety = msg

    def cmd_vel_mux_callback(self, msg: Twist):
        self.cmd_vel_mux = msg

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def log_snapshot(self):
        now = time.time()
        odom_linear_x = self.odom.twist.twist.linear.x
        odom_angular_z = self.odom.twist.twist.angular.z

        self.csv_writer.writerow([
            f'{now:.3f}',
            self.mode,
            int(self.front_obstacle_active),
            f'{self.closest_front_range_m:.3f}' if math.isfinite(self.closest_front_range_m) else '',
            f'{self.front_stop_distance_m:.3f}' if math.isfinite(self.front_stop_distance_m) else '',
            f'{self.front_forward_speed_mps:.3f}' if math.isfinite(self.front_forward_speed_mps) else '',
            f'{self.front_speed_limit_scale:.3f}' if math.isfinite(self.front_speed_limit_scale) else '',
            f'{self.cmd_vel_nav_raw.linear.x:.3f}',
            f'{self.cmd_vel_nav_raw.angular.z:.3f}',
            f'{self.cmd_vel_nav.linear.x:.3f}',
            f'{self.cmd_vel_nav.angular.z:.3f}',
            f'{self.cmd_vel_safety.linear.x:.3f}',
            f'{self.cmd_vel_safety.angular.z:.3f}',
            f'{self.cmd_vel_mux.linear.x:.3f}',
            f'{self.cmd_vel_mux.angular.z:.3f}',
            f'{odom_linear_x:.3f}',
            f'{odom_angular_z:.3f}',
        ])
        self.csv_file.flush()

        mux_active_during_stop = False
        if self.front_obstacle_active:
            if self.mode == 'AUTO':
                mux_active_during_stop = (
                    abs(self.cmd_vel_mux.linear.x) > 0.01
                    or abs(self.cmd_vel_mux.angular.z) > 0.01
                )
            else:
                mux_active_during_stop = abs(self.cmd_vel_mux.linear.x) > 0.01

        if mux_active_during_stop and (now - self.last_warning_time) > 1.0:
            self.last_warning_time = now
            self.get_logger().warn(
                'Obstacle active but mux output is still nonzero: '
                f'mode={self.mode}, '
                f'range={self.closest_front_range_m:.3f}, '
                f'stop={self.front_stop_distance_m:.3f}, '
                f'forward_speed={self.front_forward_speed_mps:.3f}, '
                f'scale={self.front_speed_limit_scale:.3f}, '
                f'nav_raw_x={self.cmd_vel_nav_raw.linear.x:.3f}, '
                f'nav_x={self.cmd_vel_nav.linear.x:.3f}, '
                f'safety_x={self.cmd_vel_safety.linear.x:.3f}, '
                f'mux_x={self.cmd_vel_mux.linear.x:.3f}, '
                f'mux_w={self.cmd_vel_mux.angular.z:.3f}, '
                f'odom_x={odom_linear_x:.3f}'
            )

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = StopDebugMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()