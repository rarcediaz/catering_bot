#!/usr/bin/env python3
"""Publish local Pi hardware-stack readiness from usable ROS data streams."""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String


class RobotHealthNode(Node):
    def __init__(self):
        super().__init__('robot_health_node')

        self.declare_parameter('stream_timeout_sec', 1.0)
        self.declare_parameter('startup_grace_sec', 15.0)
        self.declare_parameter('minimum_scan_ray_count', 100)
        self.declare_parameter('minimum_valid_scan_points', 10)
        self.stream_timeout_sec = float(
            self.get_parameter('stream_timeout_sec').value
        )
        self.startup_grace_sec = float(
            self.get_parameter('startup_grace_sec').value
        )
        self.minimum_scan_ray_count = max(
            1,
            int(self.get_parameter('minimum_scan_ray_count').value),
        )
        self.minimum_valid_scan_points = max(
            1,
            int(self.get_parameter('minimum_valid_scan_points').value),
        )

        self.started_at = time.monotonic()
        self.last_raw_scan = None
        self.last_filtered_scan = None
        self.raw_scan_usable = False
        self.filtered_scan_usable = False
        self.last_odom = None
        self.last_joint_state = None
        self.last_safety_health = None
        self.obstacle_data_healthy = False
        self.last_startup_gate = None
        self.startup_gate_open = False
        self.last_reported = None

        self.hardware_pub = self.create_publisher(
            Bool, '/robot_health/hardware_healthy', 10
        )
        self.lidar_pub = self.create_publisher(
            Bool, '/robot_health/lidar_healthy', 10
        )
        self.odom_pub = self.create_publisher(
            Bool, '/robot_health/odometry_healthy', 10
        )
        self.controller_pub = self.create_publisher(
            Bool, '/robot_health/controller_healthy', 10
        )
        self.safety_pub = self.create_publisher(
            Bool, '/robot_health/obstacle_health', 10
        )
        self.ready_pub = self.create_publisher(
            Bool, '/robot_health/ready', 10
        )
        self.log_pub = self.create_publisher(String, '/robot_health/log', 10)

        self.create_subscription(
            LaserScan, '/scan', self.raw_scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.filtered_scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, '/diff_cont/odom', self.odom_callback, 10
        )
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.create_subscription(
            Bool,
            '/robot_health/obstacle_data_healthy',
            self.safety_health_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/robot_health/startup_gate_open',
            self.startup_gate_callback,
            10,
        )

        self.create_timer(0.25, self.publish_health)
        self.send_log(
            'Robot health monitor started; readiness requires fresh raw scan, '
            'filtered scan, odometry, joint state, obstacle-safety data, and an '
            'automatically opened startup gate.'
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

    def raw_scan_callback(self, msg):
        self.last_raw_scan = time.monotonic()
        self.raw_scan_usable = self.scan_is_usable(msg)

    def filtered_scan_callback(self, msg):
        self.last_filtered_scan = time.monotonic()
        self.filtered_scan_usable = self.scan_is_usable(msg)

    def odom_callback(self, _msg):
        self.last_odom = time.monotonic()

    def joint_state_callback(self, _msg):
        self.last_joint_state = time.monotonic()

    def safety_health_callback(self, msg):
        self.last_safety_health = time.monotonic()
        self.obstacle_data_healthy = bool(msg.data)

    def startup_gate_callback(self, msg):
        self.last_startup_gate = time.monotonic()
        self.startup_gate_open = bool(msg.data)

    def is_fresh(self, timestamp, now):
        return (
            timestamp is not None
            and (now - timestamp) <= self.stream_timeout_sec
        )

    def send_log(self, text, critical=False):
        prefix = '!!! ' if critical else '> '
        self.log_pub.publish(String(data=prefix + text))

    def publish_health(self):
        now = time.monotonic()
        raw_scan_healthy = (
            self.is_fresh(self.last_raw_scan, now)
            and self.raw_scan_usable
        )
        filtered_scan_healthy = (
            self.is_fresh(self.last_filtered_scan, now)
            and self.filtered_scan_usable
        )
        odometry_healthy = self.is_fresh(self.last_odom, now)
        joint_state_healthy = self.is_fresh(self.last_joint_state, now)
        safety_healthy = (
            self.is_fresh(self.last_safety_health, now)
            and self.obstacle_data_healthy
        )
        motion_gate_healthy = (
            self.is_fresh(self.last_startup_gate, now)
            and self.startup_gate_open
        )

        lidar_healthy = raw_scan_healthy and filtered_scan_healthy
        controller_healthy = odometry_healthy and joint_state_healthy
        hardware_healthy = joint_state_healthy
        ready = (
            hardware_healthy
            and lidar_healthy
            and odometry_healthy
            and controller_healthy
            and safety_healthy
            and motion_gate_healthy
        )

        state = (
            hardware_healthy,
            lidar_healthy,
            odometry_healthy,
            controller_healthy,
            safety_healthy,
            motion_gate_healthy,
            ready,
        )
        self.hardware_pub.publish(Bool(data=hardware_healthy))
        self.lidar_pub.publish(Bool(data=lidar_healthy))
        self.odom_pub.publish(Bool(data=odometry_healthy))
        self.controller_pub.publish(Bool(data=controller_healthy))
        self.safety_pub.publish(Bool(data=safety_healthy))
        self.ready_pub.publish(Bool(data=ready))

        if state == self.last_reported:
            return

        previous_ready = self.last_reported[-1] if self.last_reported else False
        self.last_reported = state
        if ready and not previous_ready:
            self.send_log('Pi hardware stack is ready.')
        elif previous_ready and not ready:
            self.send_log(
                'Pi hardware stack lost readiness '
                f'(hardware={hardware_healthy}, lidar={lidar_healthy}, '
                f'odometry={odometry_healthy}, controller={controller_healthy}, '
                f'obstacle_safety={safety_healthy}, '
                f'startup_gate_open={motion_gate_healthy}).',
                critical=True,
            )
        elif now - self.started_at >= self.startup_grace_sec:
            self.get_logger().debug(
                'Waiting for readiness: '
                f'hardware={hardware_healthy} lidar={lidar_healthy} '
                f'odometry={odometry_healthy} controller={controller_healthy} '
                f'obstacle_safety={safety_healthy} '
                f'startup_gate_open={motion_gate_healthy}'
            )


def main():
    rclpy.init()
    node = RobotHealthNode()
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
