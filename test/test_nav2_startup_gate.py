"""Tests for map-valid localization lifecycle gating."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / 'scripts'))

from nav2_startup_gate import (  # noqa: E402
    ManageLifecycleNodes,
    Nav2StartupGate,
    localization_maps_fault,
)


def make_map(*, width=80, height=80, resolution=0.1, origin_yaw=0.0):
    origin = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
        orientation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(origin_yaw / 2.0),
            w=math.cos(origin_yaw / 2.0),
        ),
    )
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=origin,
        ),
        data=[0] * (width * height),
    )


def make_pose(x=2.0, y=2.0, yaw=0.0):
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            ),
        ),
    )


class RecordingLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warning(self, message):
        self.messages.append(('warning', message))

    def error(self, message):
        self.messages.append(('error', message))


def make_gate(client):
    gate = object.__new__(Nav2StartupGate)
    gate._require_keepout = True
    gate._navigation_map = None
    gate._keepout_map = None
    gate._pending_pose = None
    gate._last_rejection = None
    gate._localized = False
    gate._startup_in_progress = False
    gate._startup_complete = False
    gate._startup_authorization_revoked = False
    gate._startup_reset_required = False
    gate._lifecycle_operation = None
    gate._waiting_for_manager_logged = False
    gate._client = client
    logger = RecordingLogger()
    gate.get_logger = lambda: logger
    return gate, logger


def test_free_footprint_requires_both_physical_map_layers():
    navigation_map = make_map()
    keepout_map = make_map()
    pose = make_pose()

    assert localization_maps_fault(navigation_map, keepout_map, pose) == ''
    assert localization_maps_fault(navigation_map, None, pose) == (
        'waiting for the hard keepout mask'
    )
    assert localization_maps_fault(
        navigation_map,
        None,
        pose,
        require_keepout=False,
    ) == ''


def test_center_free_pose_is_rejected_when_long_front_clips_wall():
    navigation_map = make_map()
    keepout_map = make_map()
    navigation_map.data[20 * 80 + 29] = 100

    fault = localization_maps_fault(
        navigation_map,
        keepout_map,
        make_pose(),
    )

    assert 'static wall or obstacle' in fault


def test_hollow_navigation_object_is_rejected_by_filled_keepout_mask():
    navigation_map = make_map()
    keepout_map = make_map()
    keepout_map.data[20 * 80 + 29] = 100

    fault = localization_maps_fault(
        navigation_map,
        keepout_map,
        make_pose(),
    )

    assert 'static wall or obstacle' in fault


def test_rotated_footprint_unknown_and_off_map_are_rejected():
    navigation_map = make_map()
    keepout_map = make_map()
    navigation_map.data[29 * 80 + 20] = -1
    fault = localization_maps_fault(
        navigation_map,
        keepout_map,
        make_pose(yaw=math.pi / 2.0),
    )
    assert 'unknown map space' in fault

    off_map_fault = localization_maps_fault(
        make_map(),
        make_map(),
        make_pose(x=0.05, y=0.05),
    )
    assert 'outside the navigation map' in off_map_fault


def test_latest_invalid_pose_revokes_authorization_before_manager_is_ready():
    class UnavailableClient:
        def service_is_ready(self):
            return False

    gate, _logger = make_gate(UnavailableClient())
    navigation_map = make_map()
    keepout_map = make_map()
    gate._handle_map(navigation_map)
    gate._handle_keepout(keepout_map)
    gate._handle_localization(make_pose())
    assert gate._localized

    keepout_map.data[20 * 80 + 29] = 100
    gate._handle_keepout(keepout_map)
    assert not gate._localized
    assert 'static wall or obstacle' in gate._last_rejection


def test_startup_revocation_forces_reset_even_if_pose_recovers_in_flight():
    class DeferredFuture:
        def __init__(self):
            self._callback = None
            self._response = None

        def add_done_callback(self, callback):
            self._callback = callback

        def result(self):
            return self._response

        def complete(self, *, success):
            self._response = SimpleNamespace(success=success)
            self._callback(self)

    class RecordingClient:
        def __init__(self):
            self.commands = []
            self.futures = []

        def service_is_ready(self):
            return True

        def call_async(self, request):
            self.commands.append(request.command)
            future = DeferredFuture()
            self.futures.append(future)
            return future

    client = RecordingClient()
    gate, _logger = make_gate(client)
    navigation_map = make_map()
    keepout_map = make_map()
    gate._handle_map(navigation_map)
    gate._handle_keepout(keepout_map)
    gate._handle_localization(make_pose())
    assert client.commands == [ManageLifecycleNodes.Request.STARTUP]

    keepout_map.data[20 * 80 + 29] = 100
    gate._handle_keepout(keepout_map)
    assert not gate._localized

    # Recovering before the STARTUP response does not erase the fact that this
    # lifecycle attempt temporarily lost its safety authorization.
    keepout_map.data[20 * 80 + 29] = 0
    gate._handle_keepout(keepout_map)
    assert gate._localized
    assert client.commands == [ManageLifecycleNodes.Request.STARTUP]

    client.futures[0].complete(success=True)
    assert client.commands == [
        ManageLifecycleNodes.Request.STARTUP,
        ManageLifecycleNodes.Request.RESET,
    ]
    assert gate._startup_reset_required

    client.futures[1].complete(success=True)
    assert not gate._startup_reset_required
    assert not gate._startup_complete
    assert gate._startup_in_progress
    assert client.commands == [
        ManageLifecycleNodes.Request.STARTUP,
        ManageLifecycleNodes.Request.RESET,
        ManageLifecycleNodes.Request.STARTUP,
    ]
