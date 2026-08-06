#!/usr/bin/env python3

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


FULL_SCAN_MINIMUM_SPAN_RAD = math.radians(350.0)
SAMPLE_COUNT_TOLERANCE = 0.05
MINIMUM_SCAN_RAY_COUNT = 600


def has_duplicate_full_circle_endpoint(scan: LaserScan) -> bool:
    """Return true for an inclusive [-pi, +pi] scan with a duplicate last ray."""
    sample_count = len(scan.ranges)
    if sample_count < 2 or scan.angle_increment <= 0.0:
        return False

    angular_span = scan.angle_max - scan.angle_min
    if angular_span < FULL_SCAN_MINIMUM_SPAN_RAD:
        return False

    interval_count = angular_span / scan.angle_increment
    return abs(interval_count - (sample_count - 1)) <= SAMPLE_COUNT_TOLERANCE


def canonicalize_scan(scan: LaserScan) -> LaserScan:
    """Convert an inclusive full-circle scan to Karto's [min, max) convention."""
    canonical = copy.deepcopy(scan)
    if not has_duplicate_full_circle_endpoint(scan):
        return canonical

    original_count = len(canonical.ranges)
    canonical.ranges = canonical.ranges[:-1]
    if len(canonical.intensities) == original_count:
        canonical.intensities = canonical.intensities[:-1]

    # Keep angle_max at +pi. For a 360-degree Karto laser it is the upper
    # boundary, while the final stored reading is one increment below it.
    return canonical


class ScanCanonicalizer(Node):
    def __init__(self) -> None:
        super().__init__('scan_canonicalizer')
        self.declare_parameter('minimum_scan_ray_count', MINIMUM_SCAN_RAY_COUNT)
        self._minimum_scan_ray_count = max(
            1,
            int(self.get_parameter('minimum_scan_ray_count').value),
        )
        self._reported_correction = False
        self._dropping_partial_scan = False
        self._publisher = self.create_publisher(
            LaserScan,
            'scan_canonical',
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            LaserScan,
            'scan',
            self._scan_callback,
            qos_profile_sensor_data,
        )

    def _scan_callback(self, scan: LaserScan) -> None:
        if len(scan.ranges) < self._minimum_scan_ray_count:
            if not self._dropping_partial_scan:
                self.get_logger().warning(
                    'Dropping incomplete LaserScan with '
                    f'{len(scan.ranges)} rays; at least '
                    f'{self._minimum_scan_ray_count} are required.'
                )
            self._dropping_partial_scan = True
            return

        if self._dropping_partial_scan:
            self.get_logger().info(
                'Complete LaserScan stream recovered; publishing resumed.'
            )
            self._dropping_partial_scan = False

        canonical = canonicalize_scan(scan)
        if len(canonical.ranges) != len(scan.ranges) and not self._reported_correction:
            self.get_logger().info(
                'Canonicalizing full-circle LaserScan from '
                f'{len(scan.ranges)} to {len(canonical.ranges)} readings '
                'by removing the duplicated +pi endpoint.'
            )
            self._reported_correction = True
        self._publisher.publish(canonical)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanCanonicalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
