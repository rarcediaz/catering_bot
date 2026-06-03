#!/usr/bin/env python3
import json
import math
import time
import urllib.error
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotHeartbeatNode(Node):
    def __init__(self):
        super().__init__('intellitrolley_heartbeat')

        self.declare_parameter('robot_id', 'IntelliTrolley-01')
        self.declare_parameter('server_url', 'http://127.0.0.1:8000')
        self.declare_parameter('period_sec', 1.0)
        self.declare_parameter('request_timeout_sec', 1.0)
        self.declare_parameter('odom_topic', '/diff_cont/odom')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('obstacle_stop_topic', '/robot_health/front_obstacle_active')
        self.declare_parameter('manual_cmd_topic', '/cmd_vel_joy')
        self.declare_parameter('manual_timeout_sec', 0.75)

        self.robot_id = str(self.get_parameter('robot_id').value)
        self.server_url = str(self.get_parameter('server_url').value).rstrip('/')
        self.period_sec = float(self.get_parameter('period_sec').value)
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.manual_timeout_sec = float(self.get_parameter('manual_timeout_sec').value)

        self.pose = {}
        self.battery_v = None
        self.obstacle_stop = False
        self.last_manual_cmd_at = 0.0
        self.last_success_at = 0.0

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            10,
        )
        self.create_subscription(
            BatteryState,
            str(self.get_parameter('battery_topic').value),
            self.battery_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('obstacle_stop_topic').value),
            self.obstacle_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('manual_cmd_topic').value),
            self.manual_cmd_callback,
            10,
        )

        self.create_timer(self.period_sec, self.publish_heartbeat)
        self.get_logger().info(
            f'Heartbeat active for {self.robot_id}; posting to {self.server_url}.'
        )

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.pose = {
            'x': float(position.x),
            'y': float(position.y),
            'yaw': float(yaw_from_quaternion(orientation)),
        }

    def battery_callback(self, msg):
        if msg.voltage and not math.isnan(float(msg.voltage)):
            self.battery_v = float(msg.voltage)

    def obstacle_callback(self, msg):
        self.obstacle_stop = bool(msg.data)

    def manual_cmd_callback(self, msg):
        moving = (
            abs(float(msg.linear.x)) > 0.001 or
            abs(float(msg.linear.y)) > 0.001 or
            abs(float(msg.angular.z)) > 0.001
        )
        if moving:
            self.last_manual_cmd_at = time.time()

    def publish_heartbeat(self):
        now = time.time()
        payload = {
            'connection_ok': True,
            'localization_valid': bool(self.pose),
            'obstacle_stop': self.obstacle_stop,
            'manual_override_active': (now - self.last_manual_cmd_at) <= self.manual_timeout_sec,
        }
        if self.battery_v is not None:
            payload['battery_v'] = self.battery_v
        payload.update(self.pose)

        data = json.dumps(payload).encode('utf-8')
        request = urllib.request.Request(
            f'{self.server_url}/robots/{self.robot_id}/telemetry',
            data=data,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )

        try:
            with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
                response.read()
            self.last_success_at = now
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            self.get_logger().warn(f'Heartbeat post failed: {exc}')


def main():
    rclpy.init()
    node = RobotHeartbeatNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
