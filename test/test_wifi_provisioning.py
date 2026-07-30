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
    render_facility_keyfile,
    update_env_file,
    validate_psk,
    validate_ssid,
)


class FakeRunner(CommandRunner):
    def __init__(self):
        self.active = "intellitrolley-ap"
        self.address = "10.42.0.1/24"
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
            address, prefix = self.address.split("/")
            output = json.dumps(
                [
                    {
                        "addr_info": [
                            {
                                "family": "inet",
                                "scope": "global",
                                "local": address,
                                "prefixlen": int(prefix),
                            }
                        ]
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
        if "CheckpointCreate" in command:
            return subprocess.CompletedProcess(
                command,
                0,
                'o "/org/freedesktop/NetworkManager/Checkpoint/8"\n',
                "",
            )
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
        "180",
        "0",
    ]
    assert "correct-horse-battery" not in " ".join(command)
    assert "zrpi-desktop.local:8090" in response["confirm_url"]


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
    manager.pending_deadline = time.time() + 120
    manager.pending_checkpoint = (
        "/org/freedesktop/NetworkManager/Checkpoint/8"
    )
    manager.staged_connection = runner.active
    manager.staged_ssid = "Facility WiFi"
    manager.staged_security = "wpa-psk"

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


def test_keep_hotspot_updates_peers_without_switching_wifi(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)

    result = manager.configure_current_ap("10.42.0.22")

    assert manager.active_connection() == "intellitrolley-ap"
    assert not any("CheckpointCreate" in command for command in runner.commands)
    assert result["robot_address"] == "10.42.0.1"
    assert result["robot_subnet"] == "10.42.0.0/24"
    assert result["central_address"] == "10.42.0.22"
    assert "robot=10.42.0.1" in result["configuration_uri"]
    defaults = manager.robot_defaults_path.read_text(encoding="utf-8")
    assert "ROBOT_CYCLONEDDS_PEERS=10.42.0.22" in defaults


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


def test_provisioning_unit_is_separate_hardened_and_disabled_by_default():
    unit = (
        PACKAGE_ROOT / "systemd/my-bot-wifi-provisioning.service.in"
    ).read_text(encoding="utf-8")
    installer = (
        PACKAGE_ROOT / "scripts/install_wifi_provisioning_service.sh"
    ).read_text(encoding="utf-8")
    robot_unit = (
        PACKAGE_ROOT / "systemd/my-bot-robot.service.in"
    ).read_text(encoding="utf-8")
    assert "After=NetworkManager.service" in unit
    assert "NoNewPrivileges=true" in unit
    assert "ProtectSystem=strict" in unit
    assert "ReadWritePaths=/etc/NetworkManager/system-connections" in unit
    assert "my-bot-wifi-provisioning" not in robot_unit
    assert 'ENABLE_SERVICE=false' in installer
    assert 'if [[ "${ENABLE_SERVICE}" == true ]]' in installer


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
