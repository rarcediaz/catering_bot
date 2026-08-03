"""Regression tests for recovery-safe Pi Wi-Fi provisioning."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

from wifi_provisioning_server import (  # noqa: E402
    CommandRunner,
    ProvisioningError,
    ProvisioningManager,
    ProvisioningRequestHandler,
    ProvisioningServer,
    facility_connection_name,
    keyfile_ssid_value,
    parse_nmcli_networks,
    parse_saved_profile_rows,
    render_facility_keyfile,
    update_env_file,
    validate_psk,
    validate_ssid,
)
from preview_wifi_provisioning_ui import PreviewManager  # noqa: E402


PROFILE_UUID_A = "11111111-2222-3333-4444-555555555555"
PROFILE_UUID_B = "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"


class FakeRunner(CommandRunner):
    def __init__(self):
        self.active = "intellitrolley-ap"
        self.address = "10.42.0.1/24"
        self.robot_active = True
        self.profiles = {}
        self.delete_effective = True
        self.commands = []

    def run(self, command, *, timeout=15.0, check=True):
        self.commands.append(list(command))
        if command[:4] == [
            "nmcli",
            "-g",
            "GENERAL.CONNECTION",
            "device",
        ]:
            return subprocess.CompletedProcess(command, 0, f"{self.active}\n", "")
        if command[:4] == [
            "nmcli",
            "-g",
            "GENERAL.DBUS-PATH",
            "device",
        ]:
            return subprocess.CompletedProcess(
                command,
                0,
                "/org/freedesktop/NetworkManager/Devices/3\n",
                "",
            )
        if command[:5] == ["ip", "-j", "-4", "address", "show"]:
            address_info = []
            if self.address:
                address, prefix = self.address.split("/")
                address_info.append(
                    {
                        "family": "inet",
                        "scope": "global",
                        "local": address,
                        "prefixlen": int(prefix),
                    }
                )
            output = json.dumps(
                [
                    {
                        "addr_info": address_info
                    }
                ]
            )
            return subprocess.CompletedProcess(command, 0, output, "")
        if "SSID,SECURITY,SIGNAL" in command:
            output = (
                "Facility\\:West:WPA2:87\n"
                "Guest:--:45\n"
                "Campus:WPA2 802.1X:70\n"
            )
            return subprocess.CompletedProcess(command, 0, output, "")
        if "NAME,UUID,TYPE,AUTOCONNECT,DEVICE" in command:
            lines = [
                f"{profile['name']}:{profile_uuid}:wifi:"
                f"{'yes' if profile['confirmed'] else 'no'}:"
                f"{'wlan0' if profile.get('active') else '--'}"
                for profile_uuid, profile in self.profiles.items()
            ]
            return subprocess.CompletedProcess(command, 0, "\n".join(lines), "")
        if command[:3] == ["nmcli", "--get-values", "802-11-wireless.ssid"]:
            profile_uuid = command[-1]
            profile = self.profiles.get(profile_uuid)
            return subprocess.CompletedProcess(
                command,
                0 if profile else 10,
                f"{profile['ssid']}\n" if profile else "",
                "",
            )
        if "CheckpointCreate" in command:
            return subprocess.CompletedProcess(
                command,
                0,
                'o "/org/freedesktop/NetworkManager/Checkpoint/8"\n',
                "",
            )
        if command[:3] == ["systemctl", "is-active", "--quiet"]:
            return subprocess.CompletedProcess(
                command,
                0 if self.robot_active else 3,
                "",
                "",
            )
        if command[:2] == ["systemctl", "stop"]:
            self.robot_active = False
        elif command[:2] in (["systemctl", "start"], ["systemctl", "restart"]):
            self.robot_active = True
        if command[:3] == ["nmcli", "connection", "delete"]:
            if self.delete_effective:
                self.profiles.pop(command[-1], None)
            return subprocess.CompletedProcess(command, 0, "", "")
        if command[:4] in (
            ["nmcli", "connection", "show", "id"],
            ["nmcli", "connection", "show", "uuid"],
        ):
            exists = command[-1] in self.profiles
            return subprocess.CompletedProcess(command, 0 if exists else 10, "", "")
        if command[:3] == ["nmcli", "--wait", "90"] and "up" in command:
            connection_index = command.index("id") + 1
            self.active = command[connection_index]
            self.address = "192.168.40.18/24"
        elif command[:3] == ["nmcli", "--wait", "30"] and "up" in command:
            self.active = "intellitrolley-ap"
            self.address = "10.42.0.1/24"
        return subprocess.CompletedProcess(command, 0, "", "")


def make_manager(tmp_path, runner=None):
    defaults = tmp_path / "my-bot-robot"
    defaults.write_text(
        "ROS_DOMAIN_ID=0\n"
        "ROBOT_CYCLONEDDS_PEERS=172.20.10.10\n"
        "ROBOT_CYCLONEDDS_INTERFACE=wlan0\n"
        "ROBOT_CYCLONEDDS_ALLOW_MULTICAST=spdp\n",
        encoding="utf-8",
    )
    return ProvisioningManager(
        interface="wlan0",
        ap_connection="intellitrolley-ap",
        ap_cidr="10.42.0.1/24",
        hostname="zrpi-desktop.local",
        port=8090,
        switch_timeout_s=180,
        loss_grace_s=90,
        ready_state_path=tmp_path / "ready.json",
        network_connections_dir=tmp_path / "connections",
        state_dir=tmp_path / "state",
        robot_defaults_path=defaults,
        robot_service="my-bot-robot.service",
        runner=runner or FakeRunner(),
    )


def test_wifi_inputs_reject_keyfile_injection_and_weak_passwords():
    assert validate_ssid("Facility WiFi") == "Facility WiFi"
    assert validate_ssid("🥑 (5654)") == "🥑 (5654)"
    for unsafe in ("", "name\npsk=stolen", "bad\tssid"):
        with pytest.raises(ProvisioningError):
            validate_ssid(unsafe)
    with pytest.raises(ProvisioningError):
        validate_psk("short", "wpa-psk")
    with pytest.raises(ProvisioningError):
        validate_psk("validpass\nkeyfile=true", "wpa-psk")
    assert validate_psk("correct-horse-battery", "wpa-psk")
    assert validate_psk("", "open") == ""


def test_staged_keyfile_cannot_autoconnect_before_confirmation():
    connection_name = facility_connection_name("Facility WiFi")
    keyfile = render_facility_keyfile(
        ssid="Facility WiFi",
        psk="correct-horse-battery",
        security="wpa-psk",
        interface="wlan0",
        connection_name=connection_name,
        hidden=False,
    )
    assert f"id={connection_name}" in keyfile
    assert "mode=infrastructure" in keyfile
    assert "autoconnect=false" in keyfile
    assert "autoconnect-priority=200" in keyfile
    assert "autoconnect-retries=2" in keyfile
    assert "cloned-mac-address=permanent" in keyfile
    assert "psk=correct-horse-battery" in keyfile
    assert "security=wifi-security" in keyfile
    assert "method=auto" in keyfile
    assert f"ssid={keyfile_ssid_value('Facility WiFi')}" in keyfile
    assert "\nssid=Facility WiFi\n" not in keyfile

    ap_generator = (
        PACKAGE_ROOT / "scripts/generate_wifi_ap_config.sh"
    ).read_text(encoding="utf-8")
    assert "'autoconnect=false'" in ap_generator
    assert "'autoconnect-priority=50'" in ap_generator


def test_nmcli_scan_parser_classifies_personal_open_and_enterprise_networks():
    networks = parse_nmcli_networks(
        "Facility\\:West:WPA2:87\n"
        "Guest:--:45\n"
        "Campus:WPA2 802.1X:70\n"
    )
    by_ssid = {item["ssid"]: item for item in networks}
    assert by_ssid["Facility:West"]["security"] == "wpa-psk"
    assert by_ssid["Guest"]["security"] == "open"
    assert by_ssid["Campus"]["security"] == "enterprise"


def test_saved_profile_parser_keeps_multiple_managed_and_preexisting_wifi_profiles():
    profiles = parse_saved_profile_rows(
        f"intellitrolley-ap:99999999-8888-7777-6666-555555555555:wifi:yes:wlan0\n"
        f"intellitrolley-facility-a:{PROFILE_UUID_A}:wifi:yes:--\n"
        f"Avocado Hotspot:{PROFILE_UUID_B}:802-11-wireless:true:--\n"
        "Wired:12345678-1234-1234-1234-123456789012:ethernet:yes:eth0\n",
        "intellitrolley-ap",
    )

    assert [profile["uuid"] for profile in profiles] == [
        PROFILE_UUID_A,
        PROFILE_UUID_B,
    ]
    assert profiles[0]["managed"] is True
    assert profiles[1]["managed"] is False


def test_saved_profiles_can_be_selected_and_removed_by_uuid_without_touching_others(
    tmp_path,
):
    runner = FakeRunner()
    runner.profiles = {
        PROFILE_UUID_A: {
            "name": facility_connection_name("Facility WiFi"),
            "ssid": "Facility WiFi",
            "confirmed": True,
            "active": False,
        },
        PROFILE_UUID_B: {
            "name": "Avocado Hotspot",
            "ssid": "🥑 (5654)",
            "confirmed": True,
            "active": False,
        },
    }
    manager = make_manager(tmp_path, runner)

    selected = manager.select_saved_profile(
        remote_address="10.42.0.22",
        profile_uuid=PROFILE_UUID_B,
    )
    removed = manager.forget_profile(
        remote_address="10.42.0.22",
        profile_uuid=PROFILE_UUID_A,
    )

    assert selected["ssid"] == "🥑 (5654)"
    assert removed["forgotten"] is True
    assert PROFILE_UUID_A not in runner.profiles
    assert PROFILE_UUID_B in runner.profiles
    assert [
        "nmcli",
        "connection",
        "delete",
        "uuid",
        PROFILE_UUID_A,
    ] in runner.commands


def test_active_saved_profile_cannot_be_removed(tmp_path):
    runner = FakeRunner()
    runner.active = "Avocado Hotspot"
    runner.address = "172.20.10.9/28"
    runner.profiles = {
        PROFILE_UUID_B: {
            "name": "Avocado Hotspot",
            "ssid": "🥑 (5654)",
            "confirmed": True,
            "active": True,
        }
    }
    manager = make_manager(tmp_path, runner)
    manager.is_ap_client = lambda remote_address: True

    with pytest.raises(ProvisioningError, match="active Wi-Fi"):
        manager.forget_profile(
            remote_address="10.42.0.22",
            profile_uuid=PROFILE_UUID_B,
        )


def test_profile_removal_reports_when_networkmanager_keeps_profile(tmp_path):
    runner = FakeRunner()
    runner.delete_effective = False
    runner.profiles = {
        PROFILE_UUID_A: {
            "name": facility_connection_name("Facility WiFi"),
            "ssid": "Facility WiFi",
            "confirmed": True,
            "active": False,
        }
    }
    manager = make_manager(tmp_path, runner)

    with pytest.raises(ProvisioningError, match="still reports"):
        manager.forget_profile(
            remote_address="10.42.0.22",
            profile_uuid=PROFILE_UUID_A,
        )


def test_runtime_loss_waits_full_grace_before_starting_recovery_ap(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)
    manager.last_ready_signature = ("Facility WiFi", "192.168.40.18/24")
    runner.active = ""
    runner.address = ""
    scheduled = []
    manager._schedule_robot_service_refresh = lambda **kwargs: scheduled.append(kwargs)

    manager.check_runtime_recovery(now=100.0)
    manager.check_runtime_recovery(now=189.9)

    assert runner.active == ""
    assert not any(command[:2] == ["systemctl", "stop"] for command in runner.commands)

    manager.check_runtime_recovery(now=190.0)

    assert runner.active == "intellitrolley-ap"
    assert json.loads(manager.ready_state_path.read_text(encoding="utf-8"))["mode"] == "ap"
    assert scheduled == [{"ensure_running": True, "delay_s": 1.0}]


def test_runtime_loss_that_recovers_inside_grace_does_not_start_hotspot(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)
    runner.active = ""
    runner.address = ""
    manager.check_runtime_recovery(now=100.0)

    runner.active = "Facility WiFi"
    runner.address = "192.168.40.18/24"
    manager.check_runtime_recovery(now=150.0)
    manager.check_runtime_recovery(now=250.0)

    assert runner.active == "Facility WiFi"
    assert not any(
        command[:3] == ["nmcli", "--wait", "30"] for command in runner.commands
    )


def test_mutations_require_the_active_ap_and_an_ap_subnet_client(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)
    assert manager.is_ap_client("10.42.0.22")
    assert not manager.is_ap_client("192.168.1.20")
    runner.active = "Facility WiFi"
    assert not manager.is_ap_client("10.42.0.22")


def test_stage_writes_root_style_keyfile_without_switching(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)
    result = manager.stage(
        remote_address="10.42.0.22",
        ssid="Facility WiFi",
        password="correct-horse-battery",
        security="wpa-psk",
        hidden=False,
    )
    keyfile_path = (
        tmp_path
        / "connections"
        / f"{facility_connection_name('Facility WiFi')}.nmconnection"
    )
    assert result["staged"] is True
    assert keyfile_path.is_file()
    assert keyfile_path.stat().st_mode & 0o777 == 0o600
    assert manager.active_connection() == "intellitrolley-ap"
    assert ["nmcli", "connection", "reload"] in runner.commands


def test_switch_uses_networkmanager_dbus_checkpoint_and_never_passes_psk(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)
    manager.staged_connection = facility_connection_name("Facility WiFi")
    manager.staged_ssid = "Facility WiFi"
    manager.staged_security = "wpa-psk"
    response = manager.activate("10.42.0.22")
    manager.switch_timer.cancel()
    manager.switch_timer = None

    command = manager.checkpoint_create_command()
    assert command[:6] == [
        "busctl",
        "call",
        "org.freedesktop.NetworkManager",
        "/org/freedesktop/NetworkManager",
        "org.freedesktop.NetworkManager",
        "CheckpointCreate",
    ]
    assert command[-5:] == [
        "aouu",
        "1",
        "/org/freedesktop/NetworkManager/Devices/3",
        "210",
        "0",
    ]
    assert "correct-horse-battery" not in " ".join(command)
    assert "zrpi-desktop.local:8090" in response["confirm_url"]
    assert response["timeout_s"] == 180
    assert "deadline" not in response


def test_network_switch_stops_ros_then_restarts_after_ipv4_is_ready(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)
    manager.pending_connection = facility_connection_name("Facility WiFi")
    manager.pending_ssid = "Facility WiFi"
    manager.pending_deadline = time.monotonic() + 120
    scheduled = []
    manager._schedule_robot_service_refresh = lambda **kwargs: scheduled.append(kwargs)

    manager._start_checkpoint_switch()

    checkpoint_index = next(
        index
        for index, command in enumerate(runner.commands)
        if "CheckpointCreate" in command
    )
    stop_index = runner.commands.index(
        ["systemctl", "stop", "my-bot-robot.service"]
    )
    wifi_up_index = next(
        index
        for index, command in enumerate(runner.commands)
        if command[:5] == ["nmcli", "--wait", "90", "connection", "up"]
    )
    assert checkpoint_index < stop_index < wifi_up_index
    assert manager.pending_robot_was_active is True
    assert scheduled == [{"ensure_running": True, "delay_s": 1.0}]
    manager.rollback_timer.cancel()


def test_network_switch_restarts_robot_even_if_service_was_inactive(tmp_path):
    runner = FakeRunner()
    runner.robot_active = False
    manager = make_manager(tmp_path, runner)
    manager.pending_connection = facility_connection_name("Facility WiFi")
    manager.pending_ssid = "Facility WiFi"
    manager.pending_deadline = time.monotonic() + 120
    scheduled = []
    manager._schedule_robot_service_refresh = lambda **kwargs: scheduled.append(kwargs)

    manager._start_checkpoint_switch()

    assert ["systemctl", "stop", "my-bot-robot.service"] in runner.commands
    assert scheduled == [{"ensure_running": True, "delay_s": 1.0}]
    manager.rollback_timer.cancel()


def test_hotspot_rollback_stops_then_ensures_ros_restarts(tmp_path):
    runner = FakeRunner()
    runner.active = facility_connection_name("Facility WiFi")
    manager = make_manager(tmp_path, runner)
    manager.pending_checkpoint = "/org/freedesktop/NetworkManager/Checkpoint/8"
    manager.pending_connection = facility_connection_name("Facility WiFi")
    manager.pending_ssid = "Facility WiFi"
    manager.pending_robot_was_active = True
    scheduled = []
    manager._schedule_robot_service_refresh = lambda **kwargs: scheduled.append(kwargs)

    manager._checkpoint_expired(manager.pending_checkpoint)

    assert any("CheckpointRollback" in command for command in runner.commands)
    stop_index = runner.commands.index(
        ["systemctl", "stop", "my-bot-robot.service"]
    )
    rollback_index = next(
        index
        for index, command in enumerate(runner.commands)
        if "CheckpointRollback" in command
    )
    assert stop_index < rollback_index
    assert [
        "nmcli",
        "--wait",
        "30",
        "connection",
        "up",
        "id",
        "intellitrolley-ap",
        "ifname",
        "wlan0",
    ] in runner.commands
    assert scheduled == [{"ensure_running": True, "delay_s": 1.0}]
    assert manager.pending_robot_was_active is False


def test_robot_service_refresh_starts_an_inactive_unit(tmp_path):
    runner = FakeRunner()
    runner.robot_active = False
    manager = make_manager(tmp_path, runner)

    manager._refresh_robot_service(ensure_running=True, delay_s=0.0)

    assert ["systemctl", "start", "my-bot-robot.service"] in runner.commands
    assert runner.robot_active is True


def test_confirm_commits_checkpoint_and_updates_reciprocal_peer(tmp_path):
    runner = FakeRunner()
    runner.active = facility_connection_name("Facility WiFi")
    runner.address = "192.168.40.18/24"
    manager = make_manager(tmp_path, runner)
    token = "test-confirmation-token"
    manager.pending_token_hash = __import__("hashlib").sha256(
        token.encode("utf-8")
    ).hexdigest()
    manager.pending_connection = runner.active
    manager.pending_ssid = "Facility WiFi"
    manager.pending_deadline = time.monotonic() + 120
    manager.pending_checkpoint = (
        "/org/freedesktop/NetworkManager/Checkpoint/8"
    )
    manager.staged_connection = runner.active
    manager.staged_ssid = "Facility WiFi"
    manager.staged_security = "wpa-psk"
    manager.pending_robot_was_active = True
    scheduled = []
    manager._schedule_robot_service_refresh = lambda **kwargs: scheduled.append(kwargs)

    result = manager.confirm(
        remote_address="192.168.40.25",
        token=token,
    )

    assert any("CheckpointDestroy" in command for command in runner.commands)
    defaults = manager.robot_defaults_path.read_text(encoding="utf-8")
    assert "ROBOT_CYCLONEDDS_PEERS=192.168.40.25" in defaults
    assert result["robot_address"] == "192.168.40.18"
    assert result["robot_subnet"] == "192.168.40.0/24"
    assert result["central_address"] == "192.168.40.25"
    assert result["configuration_uri"].startswith(
        "intellitrolley://configure-network?"
    )
    assert scheduled == [{"ensure_running": True, "delay_s": 2.0}]


def test_confirmation_ignores_wall_clock_jump_after_internet_connects(
    tmp_path,
    monkeypatch,
):
    runner = FakeRunner()
    runner.active = facility_connection_name("Facility WiFi")
    runner.address = "192.168.40.18/24"
    manager = make_manager(tmp_path, runner)
    token = "ntp-safe-token"
    manager.pending_token_hash = __import__("hashlib").sha256(
        token.encode("utf-8")
    ).hexdigest()
    manager.pending_connection = runner.active
    manager.pending_ssid = "Facility WiFi"
    manager.pending_deadline = time.monotonic() + 120
    manager.pending_checkpoint = "/org/freedesktop/NetworkManager/Checkpoint/8"
    manager._schedule_robot_service_refresh = lambda **kwargs: None

    monkeypatch.setattr(time, "time", lambda: 4_102_444_800.0)
    result = manager.confirm(remote_address="192.168.40.25", token=token)

    assert result["confirmed"] is True
    assert any("CheckpointDestroy" in command for command in runner.commands)


def test_keep_hotspot_updates_peers_without_switching_wifi(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)
    scheduled = []
    manager._schedule_robot_service_refresh = lambda **kwargs: scheduled.append(kwargs)

    result = manager.configure_current_ap("10.42.0.22")

    assert manager.active_connection() == "intellitrolley-ap"
    assert not any("CheckpointCreate" in command for command in runner.commands)
    assert result["robot_address"] == "10.42.0.1"
    assert result["robot_subnet"] == "10.42.0.0/24"
    assert result["central_address"] == "10.42.0.22"
    assert "robot=10.42.0.1" in result["configuration_uri"]
    defaults = manager.robot_defaults_path.read_text(encoding="utf-8")
    assert "ROBOT_CYCLONEDDS_PEERS=10.42.0.22" in defaults
    assert scheduled == [{"ensure_running": True, "delay_s": 2.0}]


def test_env_update_preserves_unrelated_robot_settings(tmp_path):
    path = tmp_path / "robot.env"
    path.write_text(
        "ROBOT_MOTOR_DEVICE=/dev/motor\n"
        "ROBOT_CYCLONEDDS_PEERS=old\n",
        encoding="utf-8",
    )
    assert update_env_file(
        path,
        {
            "ROBOT_CYCLONEDDS_PEERS": "10.0.0.9",
            "ROBOT_CYCLONEDDS_INTERFACE": "wlan0",
        },
    )
    content = path.read_text(encoding="utf-8")
    assert "ROBOT_MOTOR_DEVICE=/dev/motor" in content
    assert "ROBOT_CYCLONEDDS_PEERS=10.0.0.9" in content
    assert "ROBOT_CYCLONEDDS_INTERFACE=wlan0" in content


def test_provisioning_and_robot_services_are_gated_on_wifi_readiness():
    unit = (
        PACKAGE_ROOT / "systemd/my-bot-wifi-provisioning.service.in"
    ).read_text(encoding="utf-8")
    installer = (
        PACKAGE_ROOT / "scripts/install_wifi_provisioning_service.sh"
    ).read_text(encoding="utf-8")
    robot_unit = (
        PACKAGE_ROOT / "systemd/my-bot-robot.service.in"
    ).read_text(encoding="utf-8")
    network_unit = (
        PACKAGE_ROOT / "systemd/my-bot-network-ready.service.in"
    ).read_text(encoding="utf-8")
    assert "Requires=my-bot-network-ready.service" in unit
    assert "After=my-bot-network-ready.service" in unit
    assert "NoNewPrivileges=true" in unit
    assert "ProtectSystem=strict" in unit
    assert "ReadWritePaths=/etc/NetworkManager/system-connections" in unit
    assert "ReadWritePaths=/run/my-bot-network" in unit
    assert "Requires=my-bot-network-ready.service" in robot_unit
    assert "After=local-fs.target my-bot-network-ready.service" in robot_unit
    assert "Before=my-bot-wifi-provisioning.service my-bot-robot.service" in network_unit
    assert "Requires=NetworkManager.service" in network_unit
    assert "Wants=my-bot-robot.service" in network_unit
    assert 'ENABLE_SERVICE=false' in installer
    assert 'START_SERVICE=false' in installer
    assert '--enable-next-boot' in installer
    assert 'if [[ "${ENABLE_SERVICE}" == true ]]' in installer
    assert "ROBOT_WIFI_LOSS_GRACE_S=90" in installer


def test_provisioning_ui_never_places_password_in_query_or_storage():
    javascript = (
        PACKAGE_ROOT / "wifi_provisioning_ui/app.js"
    ).read_text(encoding="utf-8")
    html = (
        PACKAGE_ROOT / "wifi_provisioning_ui/index.html"
    ).read_text(encoding="utf-8")
    assert 'type="password"' in html
    assert 'body: JSON.stringify({' in javascript
    assert "localStorage" not in javascript
    assert "sessionStorage" not in javascript
    assert "?password=" not in javascript
    assert 'api("/api/use-ap"' in javascript
    assert 'id="network-results"' in html
    assert 'id="saved-profiles"' in html
    assert "visibleOption.textContent" in javascript
    assert 'api("/api/select"' in javascript
    assert "profile.uuid" in javascript
    assert "startCountdown(body.timeout_s)" in javascript


def test_local_ui_preview_exercises_scan_switch_and_confirmation():
    manager = PreviewManager(port=8090)
    networks = manager.scan_networks("127.0.0.1")
    assert [network["ssid"] for network in networks] == [
        "Warehouse WiFi",
        "Office-5G",
        "Guest Network",
        "Corporate 802.1X",
    ]

    staged = manager.stage(
        remote_address="127.0.0.1",
        ssid="Warehouse WiFi",
        password="preview-password",
        security="wpa-psk",
        hidden=False,
    )
    activated = manager.activate("127.0.0.1")
    confirmed = manager.confirm(
        remote_address="127.0.0.1",
        token="preview-token",
    )

    assert staged["staged"] is True
    assert activated["timeout_s"] == 180
    assert activated["confirm_url"].endswith("?confirm=preview-token")
    assert confirmed["confirmed"] is True


def test_http_ui_is_no_store_and_rejects_mutation_outside_ap(tmp_path):
    manager = make_manager(tmp_path, FakeRunner())
    server = ProvisioningServer(
        ("127.0.0.1", 0),
        ProvisioningRequestHandler,
        manager=manager,
        ui_dir=PACKAGE_ROOT / "wifi_provisioning_ui",
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urlopen(f"{base_url}/", timeout=3) as response:
            assert response.status == 200
            assert response.headers["Cache-Control"] == "no-store, max-age=0"
            assert response.headers["X-Frame-Options"] == "DENY"
            assert b"Save without switching" in response.read()

        request = Request(
            f"{base_url}/api/stage",
            data=json.dumps(
                {
                    "ssid": "Facility WiFi",
                    "password": "correct-horse-battery",
                    "security": "wpa-psk",
                }
            ).encode("utf-8"),
            method="POST",
            headers={
                "Content-Type": "application/json",
                "X-IntelliTrolley-Provisioning": "1",
            },
        )
        with pytest.raises(HTTPError) as caught:
            urlopen(request, timeout=3)
        assert caught.value.code == 403
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
