#!/usr/bin/env python3

"""Start the navigation lifecycle only after AMCL has produced a valid pose."""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Nav2StartupGate(Node):
    """Prevent costmaps from activating before the map-to-odom transform exists."""

    def __init__(self) -> None:
        super().__init__('nav2_startup_gate')

        self.declare_parameter('localization_topic', '/amcl_pose')
        self.declare_parameter(
            'manager_service',
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.declare_parameter('retry_period_s', 2.0)

        localization_topic = str(
            self.get_parameter('localization_topic').value
        )
        manager_service = str(self.get_parameter('manager_service').value)
        retry_period_s = float(self.get_parameter('retry_period_s').value)

        localization_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._localized = False
        self._startup_in_progress = False
        self._startup_complete = False
        self._waiting_for_manager_logged = False
        self._client = self.create_client(
            ManageLifecycleNodes,
            manager_service,
        )
        self._localization_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            localization_topic,
            self._handle_localization,
            localization_qos,
        )
        self._timer = self.create_timer(retry_period_s, self._try_startup)
        self.get_logger().info(
            f'Waiting for the first AMCL pose on {localization_topic} before '
            'starting navigation.'
        )

    def _handle_localization(self, _message: PoseWithCovarianceStamped) -> None:
        if self._localized:
            return
        self._localized = True
        self.get_logger().info(
            'AMCL pose received. Navigation lifecycle startup is now allowed.'
        )
        self._try_startup()

    def _try_startup(self) -> None:
        if (
            not self._localized
            or self._startup_in_progress
            or self._startup_complete
        ):
            return

        if not self._client.service_is_ready():
            if not self._waiting_for_manager_logged:
                self.get_logger().info(
                    'Waiting for the Nav2 navigation lifecycle manager.'
                )
                self._waiting_for_manager_logged = True
            return

        self._waiting_for_manager_logged = False
        self._startup_in_progress = True
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self._client.call_async(request)
        future.add_done_callback(self._handle_startup_result)
        self.get_logger().info('Starting the Nav2 navigation servers.')

    def _handle_startup_result(self, future) -> None:
        self._startup_in_progress = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - middleware failure path
            self.get_logger().error(
                f'Navigation lifecycle startup call failed: {exc}. Retrying.'
            )
            return

        if not response.success:
            self.get_logger().error(
                'Navigation lifecycle startup was not successful. Retrying.'
            )
            return

        self._startup_complete = True
        self.get_logger().info(
            'Navigation is active; NavigateToPose goals may now be sent.'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2StartupGate()
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
