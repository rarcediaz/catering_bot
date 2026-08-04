#!/usr/bin/env python3

"""Start navigation only after AMCL's trolley footprint is map-valid."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


FOOTPRINT_REAR_M = -0.15
FOOTPRINT_FRONT_M = 0.917
FOOTPRINT_HALF_WIDTH_M = 0.305
OCCUPIED_THRESHOLD = 65


def _yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def footprint_map_fault(
    map_message: OccupancyGrid,
    pose_message: PoseWithCovarianceStamped,
) -> str:
    """Return why the physical footprint is invalid, or an empty string."""
    info = map_message.info
    width = int(info.width)
    height = int(info.height)
    resolution = float(info.resolution)
    data = map_message.data
    if (
        width <= 0
        or height <= 0
        or not math.isfinite(resolution)
        or resolution <= 0.0
        or len(data) < width * height
    ):
        return 'navigation map metadata is invalid'

    origin = info.origin
    origin_x = float(origin.position.x)
    origin_y = float(origin.position.y)
    origin_yaw = _yaw_from_quaternion(origin.orientation)
    pose = pose_message.pose.pose
    pose_x = float(pose.position.x)
    pose_y = float(pose.position.y)
    pose_yaw = _yaw_from_quaternion(pose.orientation)
    if not all(
        math.isfinite(value)
        for value in (origin_x, origin_y, origin_yaw, pose_x, pose_y, pose_yaw)
    ):
        return 'AMCL pose or map origin is not finite'

    sample_step = max(0.01, resolution * 0.5)
    footprint_length = FOOTPRINT_FRONT_M - FOOTPRINT_REAR_M
    footprint_width = 2.0 * FOOTPRINT_HALF_WIDTH_M
    x_steps = max(1, math.ceil(footprint_length / sample_step))
    y_steps = max(1, math.ceil(footprint_width / sample_step))
    pose_cos = math.cos(pose_yaw)
    pose_sin = math.sin(pose_yaw)
    origin_cos = math.cos(origin_yaw)
    origin_sin = math.sin(origin_yaw)
    checked_cells = set()

    for x_index in range(x_steps + 1):
        local_x = (
            FOOTPRINT_REAR_M
            + footprint_length * x_index / x_steps
        )
        for y_index in range(y_steps + 1):
            local_y = (
                -FOOTPRINT_HALF_WIDTH_M
                + footprint_width * y_index / y_steps
            )
            world_x = pose_x + pose_cos * local_x - pose_sin * local_y
            world_y = pose_y + pose_sin * local_x + pose_cos * local_y
            dx = world_x - origin_x
            dy = world_y - origin_y
            map_x = origin_cos * dx + origin_sin * dy
            map_y = -origin_sin * dx + origin_cos * dy
            col = math.floor(map_x / resolution)
            row = math.floor(map_y / resolution)
            cell = (col, row)
            if cell in checked_cells:
                continue
            checked_cells.add(cell)

            if col < 0 or col >= width or row < 0 or row >= height:
                return 'robot footprint extends outside the navigation map'
            occupancy = int(data[row * width + col])
            if occupancy < 0:
                return 'robot footprint overlaps unknown map space'
            if occupancy >= OCCUPIED_THRESHOLD:
                return 'robot footprint overlaps a static wall or obstacle'

    return ''


def localization_maps_fault(
    navigation_map,
    keepout_map,
    pose_message,
    require_keepout=True,
) -> str:
    """Validate AMCL against both physical map layers."""
    if navigation_map is None:
        return 'waiting for the navigation map'
    navigation_fault = footprint_map_fault(navigation_map, pose_message)
    if navigation_fault:
        return navigation_fault
    if require_keepout:
        if keepout_map is None:
            return 'waiting for the hard keepout mask'
        keepout_fault = footprint_map_fault(keepout_map, pose_message)
        if keepout_fault:
            return keepout_fault
    return ''


class Nav2StartupGate(Node):
    """Prevent Nav2 activation for impossible AMCL poses."""

    def __init__(self) -> None:
        super().__init__('nav2_startup_gate')

        self.declare_parameter('localization_topic', '/amcl_pose')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('keepout_topic', '/keepout_filter_mask')
        self.declare_parameter('require_keepout', True)
        self.declare_parameter(
            'manager_service',
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.declare_parameter('retry_period_s', 2.0)

        localization_topic = str(
            self.get_parameter('localization_topic').value
        )
        map_topic = str(self.get_parameter('map_topic').value)
        keepout_topic = str(self.get_parameter('keepout_topic').value)
        self._require_keepout = bool(
            self.get_parameter('require_keepout').value
        )
        manager_service = str(self.get_parameter('manager_service').value)
        retry_period_s = float(self.get_parameter('retry_period_s').value)

        localization_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._localized = False
        self._navigation_map = None
        self._keepout_map = None
        self._pending_pose = None
        self._last_rejection = None
        self._startup_in_progress = False
        self._startup_complete = False
        self._startup_authorization_revoked = False
        self._startup_reset_required = False
        self._lifecycle_operation = None
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
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self._handle_map,
            map_qos,
        )
        self._keepout_subscription = self.create_subscription(
            OccupancyGrid,
            keepout_topic,
            self._handle_keepout,
            map_qos,
        )
        self._timer = self.create_timer(retry_period_s, self._try_startup)
        self.get_logger().info(
            f'Waiting for a map-valid AMCL pose on {localization_topic} before '
            'starting navigation.'
        )

    def _handle_map(self, message: OccupancyGrid) -> None:
        self._navigation_map = message
        self._validate_pending_pose()

    def _handle_keepout(self, message: OccupancyGrid) -> None:
        self._keepout_map = message
        self._validate_pending_pose()

    def _handle_localization(self, message: PoseWithCovarianceStamped) -> None:
        if self._startup_complete:
            return
        self._pending_pose = message
        self._validate_pending_pose()

    def _validate_pending_pose(self) -> None:
        if self._startup_complete or self._pending_pose is None:
            return
        fault = localization_maps_fault(
            self._navigation_map,
            self._keepout_map,
            self._pending_pose,
            self._require_keepout,
        )
        if fault:
            self._localized = False
            if (
                self._startup_in_progress
                and self._lifecycle_operation == 'startup'
            ):
                # A later valid pose must not rescue an in-flight request that
                # was unsafe at any point. Let it finish, RESET it if it
                # succeeded, then start again from a continuously valid pose.
                self._startup_authorization_revoked = True
            if fault != self._last_rejection:
                self.get_logger().warning(
                    f'AMCL pose not accepted: {fault}. Navigation remains locked.'
                )
                self._last_rejection = fault
            return

        newly_localized = not self._localized
        self._localized = True
        self._last_rejection = None
        if newly_localized:
            self.get_logger().info(
                'AMCL footprint is free in every required physical map. '
                'Navigation lifecycle startup is now allowed.'
            )
        self._try_startup()

    def _try_startup(self) -> None:
        if self._startup_in_progress or self._startup_complete:
            return

        if self._startup_reset_required:
            self._try_reset_revoked_startup()
            return

        if not self._localized:
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
        self._startup_authorization_revoked = False
        self._lifecycle_operation = 'startup'
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self._client.call_async(request)
        future.add_done_callback(self._handle_startup_result)
        self.get_logger().info('Starting the Nav2 navigation servers.')

    def _try_reset_revoked_startup(self) -> None:
        """Undo a STARTUP that completed after its pose became invalid."""
        if self._startup_in_progress or not self._startup_reset_required:
            return
        if not self._client.service_is_ready():
            return

        self._startup_in_progress = True
        self._lifecycle_operation = 'reset'
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.RESET
        future = self._client.call_async(request)
        future.add_done_callback(self._handle_revoked_startup_reset_result)
        self.get_logger().warning(
            'Resetting Nav2 because localization authorization was revoked '
            'during startup.'
        )

    def _handle_revoked_startup_reset_result(self, future) -> None:
        self._startup_in_progress = False
        self._lifecycle_operation = None
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - middleware failure path
            self.get_logger().error(
                f'Nav2 safety reset call failed: {exc}. Retrying.'
            )
            return

        if not response.success:
            self.get_logger().error(
                'Nav2 safety reset was not successful. Retrying.'
            )
            return

        self._startup_reset_required = False
        self._startup_authorization_revoked = False
        self.get_logger().info(
            'Nav2 is inactive again and waiting for a new map-valid AMCL pose.'
        )
        self._try_startup()

    def _handle_startup_result(self, future) -> None:
        self._startup_in_progress = False
        self._lifecycle_operation = None
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

        if not self._localized or self._startup_authorization_revoked:
            self._startup_reset_required = True
            self.get_logger().error(
                'Nav2 startup completed after localization authorization was '
                'revoked. A safety reset is required.'
            )
            self._try_startup()
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
