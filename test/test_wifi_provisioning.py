"""Regression tests for recovery-safe Pi Wi-Fi provisioning."""

from __future__ import annotations

import ipaddress
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
        self.checkpoint_origin = None
        self.checkpoint_live = False
        self.checkpoint_rollback_succeeds = True
        self.checkpoint_destroy_succeeds = True
        self.ap_activation_succeeds = True
        self.client_activation_succeeds = True
        self.connection_addresses = {}

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
            self.checkpoint_origin = (self.active, self.address)
            self.checkpoint_live = True
            return subprocess.CompletedProcess(
                command,
                0,
                'o "/org/freedesktop/NetworkManager/Checkpoint/8"\n',
                "",
            )
        if "CheckpointRollback" in command:
            if not self.checkpoint_rollback_succeeds:
                return subprocess.CompletedProcess(command, 1, "", "rollback failed")
            if self.checkpoint_origin is not None:
                self.active, self.address = self.checkpoint_origin
            self.checkpoint_live = False
            return subprocess.CompletedProcess(command, 0, "", "")
        if "CheckpointDestroy" in command:
            if not self.checkpoint_destroy_succeeds:
                return subprocess.CompletedProcess(command, 1, "", "destroy failed")
            self.checkpoint_live = False
            return subprocess.CompletedProcess(command, 0, "", "")
        if command[-1:] == ["Checkpoints"]:
            output = (
                'ao 1 "/org/freedesktop/NetworkManager/Checkpoint/8"\n'
                if self.checkpoint_live
                else "ao 0\n"
            )
            return subprocess.CompletedProcess(command, 0, output, "")
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
            connection_index = command.index("id") + 1
            connection = command[connection_index]
            succeeds = (
                self.ap_activation_succeeds
                if connection == "intellitrolley-ap"
                else self.client_activation_succeeds
            )
            if not succeeds:
                return subprocess.CompletedProcess(command, 10, "", "activation failed")
            self.active = connection
            self.address = self.connection_addresses.get(
                connection,
                "10.42.0.1/24"
                if connection == "intellitrolley-ap"
                else "192.168.40.18/24",
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

    with pytest.raises(ProvisioningError, match="active Wi-Fi"):
        manager.forget_profile(
            remote_address="172.20.10.8",
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
    assert scheduled == []
    assert ["systemctl", "start", "my-bot-robot.service"] in runner.commands


def test_runtime_recovery_locks_mutations_until_hotspot_and_robot_are_ready(
    tmp_path,
):
    runner = FakeRunner()
    runner.active = ""
    runner.address = ""
    manager = make_manager(tmp_path, runner)
    manager.check_runtime_recovery(now=100.0)

    stop_started = threading.Event()
    allow_stop = threading.Event()
    recovery_errors = []

    def blocking_stop():
        stop_started.set()
        assert allow_stop.wait(timeout=3)
        runner.robot_active = False
        return True

    manager._stop_robot_service_for_switch = blocking_stop

    def recover():
        try:
            manager.check_runtime_recovery(now=190.0)
        except Exception as exc:  # pragma: no cover - asserted below
            recovery_errors.append(exc)

    recovery_thread = threading.Thread(target=recover)
    recovery_thread.start()
    assert stop_started.wait(timeout=3)
    assert manager.transition_phase == "runtime_recovery"

    # Even if a client network appears while AP recovery owns the interface,
    # same-subnet browser requests must stay read-only until recovery finishes.
    runner.active = "Facility A"
    runner.address = "172.20.10.9/28"
    status = manager.status("172.20.10.8")
    assert status["can_provision"] is False
    with pytest.raises(ProvisioningError, match="current Wi-Fi operation"):
        manager.stage(
            remote_address="172.20.10.8",
            ssid="Another WiFi",
            password="correct-horse-battery",
            security="wpa-psk",
            hidden=False,
        )

    allow_stop.set()
    recovery_thread.join(timeout=3)
    assert not recovery_thread.is_alive()
    assert not recovery_errors
    assert runner.active == "intellitrolley-ap"
    assert runner.address == "10.42.0.1/24"
    assert manager.transition_phase == "idle"
    assert runner.robot_active is True


def test_failed_runtime_hotspot_recovery_retries_after_a_new_grace_period(tmp_path):
    runner = FakeRunner()
    runner.active = ""
    runner.address = ""
    runner.ap_activation_succeeds = False
    manager = make_manager(tmp_path, runner)

    manager.check_runtime_recovery(now=100.0)
    manager.check_runtime_recovery(now=190.0)

    assert manager.transition_phase == "idle"
    assert manager.loss_started_at == 190.0
    assert manager.robot_restart_required is True
    assert runner.robot_active is False
    assert ["systemctl", "start", "my-bot-robot.service"] not in runner.commands
    attempts = sum(
        command[:5] == ["nmcli", "--wait", "30", "connection", "up"]
        for command in runner.commands
    )
    manager.check_runtime_recovery(now=279.9)
    assert sum(
        command[:5] == ["nmcli", "--wait", "30", "connection", "up"]
        for command in runner.commands
    ) == attempts
    manager.check_runtime_recovery(now=280.0)
    assert sum(
        command[:5] == ["nmcli", "--wait", "30", "connection", "up"]
        for command in runner.commands
    ) == attempts + 1


def test_same_wifi_returning_after_failed_hotspot_recovery_restarts_robot(
    tmp_path,
):
    runner = FakeRunner()
    runner.active = ""
    runner.address = ""
    runner.ap_activation_succeeds = False
    manager = make_manager(tmp_path, runner)
    manager.last_ready_signature = ("Facility A", "172.20.10.9/28")

    manager.check_runtime_recovery(now=100.0)
    manager.check_runtime_recovery(now=190.0)
    assert runner.robot_active is False
    assert manager.robot_restart_required is True

    runner.active = "Facility A"
    runner.address = "172.20.10.9/28"
    manager.check_runtime_recovery(now=191.0)

    assert runner.robot_active is True
    assert manager.robot_restart_required is False
    assert manager.transition_phase == "idle"
    assert ["systemctl", "start", "my-bot-robot.service"] in runner.commands
    state = json.loads(manager.state_path.read_text(encoding="utf-8"))
    assert state["robot_restart_required"] is False


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


def test_mutations_require_a_client_on_the_active_wifi_subnet(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)
    assert manager.is_active_wifi_client("10.42.0.22")
    assert manager.is_ap_client("10.42.0.22")
    assert not manager.is_active_wifi_client("192.168.1.20")
    ap_status = manager.status("10.42.0.22")
    assert ap_status["can_provision"] is True
    assert ap_status["using_recovery_ap"] is True
    assert ap_status["can_configure_ap"] is True

    runner.active = "Facility WiFi"
    runner.address = "172.20.10.9/28"
    assert manager.is_active_wifi_client("172.20.10.8")
    assert not manager.is_active_wifi_client("10.42.0.22")
    assert not manager.is_ap_client("172.20.10.8")

    status = manager.status("172.20.10.8")
    assert status["can_provision"] is True
    assert status["using_recovery_ap"] is False
    assert status["can_configure_ap"] is False

    runner.active = ""
    runner.address = ""
    assert not manager.is_active_wifi_client("172.20.10.8")
    assert not manager.is_active_wifi_client("fe80::22")


def test_stage_from_active_facility_wifi_does_not_switch_connections(tmp_path):
    runner = FakeRunner()
    runner.active = "Avocado Hotspot"
    runner.address = "172.20.10.9/28"
    manager = make_manager(tmp_path, runner)

    result = manager.stage(
        remote_address="172.20.10.8",
        ssid="Facility WiFi",
        password="correct-horse-battery",
        security="wpa-psk",
        hidden=False,
    )

    assert result["staged"] is True
    assert runner.active == "Avocado Hotspot"
    assert runner.address == "172.20.10.9/28"
    assert "stayed on its current Wi-Fi" in result["message"]


def test_stage_cannot_overwrite_the_active_managed_profile(tmp_path):
    runner = FakeRunner()
    connection_name = facility_connection_name("Facility WiFi")
    runner.active = connection_name
    runner.address = "172.20.10.9/28"
    manager = make_manager(tmp_path, runner)
    manager.network_connections_dir.mkdir(parents=True)
    keyfile = manager.network_connections_dir / f"{connection_name}.nmconnection"
    keyfile.write_text("original-profile\n", encoding="utf-8")

    with pytest.raises(ProvisioningError, match="currently active"):
        manager.stage(
            remote_address="172.20.10.8",
            ssid="Facility WiFi",
            password="replacement-password",
            security="wpa-psk",
            hidden=False,
        )

    assert keyfile.read_text(encoding="utf-8") == "original-profile\n"
    assert ["nmcli", "connection", "reload"] not in runner.commands
    assert manager.transition_phase == "idle"


def test_profile_mutations_are_disabled_during_a_wifi_transition(tmp_path):
    runner = FakeRunner()
    runner.profiles = {
        PROFILE_UUID_A: {
            "name": facility_connection_name("Facility WiFi"),
            "ssid": "Facility WiFi",
            "confirmed": True,
            "active": False,
        }
    }
    manager = make_manager(tmp_path, runner)
    manager.staged_connection = facility_connection_name("Facility WiFi")
    manager.staged_ssid = "Facility WiFi"
    manager.transition_phase = "pending_confirmation"

    status = manager.status("10.42.0.22")
    assert status["can_provision"] is False
    assert status["can_configure_ap"] is False

    operations = [
        lambda: manager.scan_networks("10.42.0.22"),
        lambda: manager.stage(
            remote_address="10.42.0.22",
            ssid="Another WiFi",
            password="correct-horse-battery",
            security="wpa-psk",
            hidden=False,
        ),
        lambda: manager.select_saved_profile(
            remote_address="10.42.0.22",
            profile_uuid=PROFILE_UUID_A,
        ),
        lambda: manager.forget_profile(
            remote_address="10.42.0.22",
            profile_uuid=PROFILE_UUID_A,
        ),
        lambda: manager.activate("10.42.0.22"),
        lambda: manager.configure_current_ap("10.42.0.22"),
    ]
    for operation in operations:
        with pytest.raises(ProvisioningError, match="current Wi-Fi|already pending"):
            operation()


def test_loaded_pending_state_is_read_only_until_explicit_recovery(tmp_path):
    writer = make_manager(tmp_path, FakeRunner())
    writer.pending_connection = facility_connection_name("Unconfirmed B")
    writer.pending_ssid = "Unconfirmed B"
    writer.pending_previous_connection = "Facility A"
    writer.pending_checkpoint = "/org/freedesktop/NetworkManager/Checkpoint/8"
    writer.pending_robot_was_active = True
    writer._save_state("pending")

    runner = FakeRunner()
    runner.active = facility_connection_name("Unconfirmed B")
    runner.address = "192.168.50.18/24"
    runner.checkpoint_origin = ("Facility A", "172.20.10.9/28")
    runner.connection_addresses["Facility A"] = "172.20.10.9/28"
    manager = make_manager(tmp_path, runner)

    assert manager.transition_phase == "interrupted_recovery"
    assert manager.pending_checkpoint == (
        "/org/freedesktop/NetworkManager/Checkpoint/8"
    )
    assert manager.status("192.168.50.25")["can_provision"] is False
    with pytest.raises(ProvisioningError, match="current Wi-Fi operation"):
        manager.stage(
            remote_address="192.168.50.25",
            ssid="Another WiFi",
            password="correct-horse-battery",
            security="wpa-psk",
            hidden=False,
        )
    assert ["nmcli", "connection", "reload"] not in runner.commands

    manager._recover_interrupted_switch(now=100.0)

    assert runner.active == "Facility A"
    assert runner.address == "172.20.10.9/28"
    assert manager.transition_phase == "idle"
    assert not manager.pending_connection
    assert not manager.pending_previous_connection
    assert any("CheckpointRollback" in command for command in runner.commands)
    assert any("CheckpointDestroy" in command for command in runner.commands)
    assert json.loads(manager.state_path.read_text(encoding="utf-8"))["phase"] == (
        "rolled_back"
    )
    ready = json.loads(manager.ready_state_path.read_text(encoding="utf-8"))
    assert ready["mode"] == "client"
    assert ready["connection"] == "Facility A"
    assert manager.status("172.20.10.8")["can_provision"] is True
    assert not any(
        "intellitrolley-ap" in command and "up" in command
        for command in runner.commands
    )
    assert runner.robot_active is True


def test_failed_interrupted_recovery_keeps_the_portal_locked(tmp_path):
    writer = make_manager(tmp_path, FakeRunner())
    writer.pending_connection = facility_connection_name("Unconfirmed B")
    writer.pending_ssid = "Unconfirmed B"
    writer.pending_previous_connection = "Unavailable Facility A"
    writer._save_state("pending")

    runner = FakeRunner()
    runner.active = facility_connection_name("Unconfirmed B")
    runner.address = "192.168.50.18/24"
    runner.client_activation_succeeds = False
    runner.ap_activation_succeeds = False
    manager = make_manager(tmp_path, runner)

    manager._recover_interrupted_switch(now=100.0)

    assert manager.transition_phase == "interrupted_recovery"
    assert manager.status("192.168.50.25")["can_provision"] is False
    assert json.loads(manager.state_path.read_text(encoding="utf-8"))["phase"] == (
        "pending"
    )


def test_live_checkpoint_must_be_disarmed_before_interrupted_recovery_unlocks(
    tmp_path,
):
    writer = make_manager(tmp_path, FakeRunner())
    writer.pending_connection = facility_connection_name("Unconfirmed B")
    writer.pending_ssid = "Unconfirmed B"
    writer.pending_previous_connection = "Facility A"
    writer.pending_checkpoint = "/org/freedesktop/NetworkManager/Checkpoint/8"
    writer._save_state("pending")

    runner = FakeRunner()
    runner.active = facility_connection_name("Unconfirmed B")
    runner.address = "192.168.50.18/24"
    runner.checkpoint_live = True
    runner.checkpoint_rollback_succeeds = False
    runner.checkpoint_destroy_succeeds = False
    runner.connection_addresses["Facility A"] = "172.20.10.9/28"
    manager = make_manager(tmp_path, runner)

    manager._recover_interrupted_switch(now=100.0)

    assert manager.transition_phase == "interrupted_recovery"
    assert manager.pending_checkpoint == (
        "/org/freedesktop/NetworkManager/Checkpoint/8"
    )
    assert runner.active == facility_connection_name("Unconfirmed B")
    assert runner.robot_active is False
    assert manager.status("192.168.50.25")["can_provision"] is False

    runner.checkpoint_destroy_succeeds = True
    manager.interrupted_retry_at = 0.0
    manager._recover_interrupted_switch(now=200.0)

    assert manager.pending_checkpoint == ""
    assert manager.transition_phase == "idle"
    assert runner.active == "Facility A"
    assert runner.robot_active is True


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


def test_activating_the_current_wifi_does_not_start_a_checkpoint(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)
    manager.staged_connection = runner.active
    manager.staged_ssid = "IntelliTrolley"
    manager.staged_security = "saved"

    with pytest.raises(ProvisioningError, match="already active"):
        manager.activate("10.42.0.22")

    assert not any("CheckpointCreate" in command for command in runner.commands)
    assert ["systemctl", "stop", "my-bot-robot.service"] not in runner.commands


def test_network_switch_stops_ros_then_restarts_after_ipv4_is_ready(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)
    manager.pending_connection = facility_connection_name("Facility WiFi")
    manager.pending_ssid = "Facility WiFi"
    manager.pending_previous_connection = runner.active
    manager.pending_deadline = time.monotonic() + 120
    manager.transition_phase = "scheduled"
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
    manager.pending_previous_connection = runner.active
    manager.pending_deadline = time.monotonic() + 120
    manager.transition_phase = "scheduled"
    scheduled = []
    manager._schedule_robot_service_refresh = lambda **kwargs: scheduled.append(kwargs)

    manager._start_checkpoint_switch()

    assert ["systemctl", "stop", "my-bot-robot.service"] in runner.commands
    assert scheduled == [{"ensure_running": True, "delay_s": 1.0}]
    manager.rollback_timer.cancel()


def test_stop_timeout_recovery_restores_wifi_and_robot_before_unlocking(tmp_path):
    runner = FakeRunner()
    manager = make_manager(tmp_path, runner)
    manager.pending_connection = facility_connection_name("Facility WiFi")
    manager.pending_ssid = "Facility WiFi"
    manager.pending_previous_connection = runner.active
    manager.pending_deadline = time.monotonic() + 120
    manager.transition_phase = "scheduled"
    scheduled = []
    manager._schedule_robot_service_refresh = lambda **kwargs: scheduled.append(kwargs)

    def stop_then_timeout():
        runner.robot_active = False
        raise subprocess.TimeoutExpired(["systemctl", "stop"], 45)

    manager._stop_robot_service_for_switch = stop_then_timeout
    manager._start_checkpoint_switch()

    assert runner.active == "intellitrolley-ap"
    assert runner.address == "10.42.0.1/24"
    assert runner.robot_active is True
    assert manager.robot_restart_required is False
    assert manager.transition_phase == "idle"
    assert scheduled == []


def test_hotspot_rollback_stops_then_ensures_ros_restarts(tmp_path):
    runner = FakeRunner()
    runner.active = facility_connection_name("Facility WiFi")
    manager = make_manager(tmp_path, runner)
    manager.pending_checkpoint = "/org/freedesktop/NetworkManager/Checkpoint/8"
    manager.pending_connection = facility_connection_name("Facility WiFi")
    manager.pending_ssid = "Facility WiFi"
    manager.pending_previous_connection = "intellitrolley-ap"
    manager.pending_robot_was_active = True
    manager.transition_phase = "pending_confirmation"
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
    assert scheduled == []
    assert ["systemctl", "start", "my-bot-robot.service"] in runner.commands
    assert manager.pending_robot_was_active is False


def test_failed_checkpoint_recovery_keeps_robot_stopped_until_wifi_returns(
    tmp_path,
):
    runner = FakeRunner()
    runner.active = facility_connection_name("Unconfirmed B")
    runner.address = "192.168.50.18/24"
    runner.client_activation_succeeds = False
    runner.ap_activation_succeeds = False
    manager = make_manager(tmp_path, runner)
    manager.pending_checkpoint = "/org/freedesktop/NetworkManager/Checkpoint/8"
    manager.pending_connection = runner.active
    manager.pending_ssid = "Unconfirmed B"
    manager.pending_previous_connection = "Unavailable Facility A"
    manager.pending_robot_was_active = True
    manager.transition_phase = "pending_confirmation"

    manager._checkpoint_expired(manager.pending_checkpoint)

    assert runner.robot_active is False
    assert manager.robot_restart_required is True
    assert ["systemctl", "start", "my-bot-robot.service"] not in runner.commands
    assert manager.transition_phase == "idle"


def test_switch_from_facility_wifi_rolls_back_to_that_facility_wifi(tmp_path):
    runner = FakeRunner()
    runner.active = "Avocado Hotspot"
    runner.address = "172.20.10.9/28"
    manager = make_manager(tmp_path, runner)
    manager.staged_connection = facility_connection_name("Facility WiFi")
    manager.staged_ssid = "Facility WiFi"
    manager.staged_security = "wpa-psk"
    scheduled = []
    manager._schedule_robot_service_refresh = lambda **kwargs: scheduled.append(kwargs)

    manager.activate("172.20.10.8")
    manager.switch_timer.cancel()
    manager.switch_timer = None
    manager._start_checkpoint_switch()

    checkpoint = manager.pending_checkpoint
    manager.rollback_timer.cancel()
    manager._checkpoint_expired(checkpoint)

    assert runner.active == "Avocado Hotspot"
    assert runner.address == "172.20.10.9/28"
    assert not any(
        command[:5] == ["nmcli", "--wait", "30", "connection", "up"]
        and "intellitrolley-ap" in command
        for command in runner.commands
    )
    assert scheduled == [{"ensure_running": True, "delay_s": 1.0}]
    assert ["systemctl", "start", "my-bot-robot.service"] in runner.commands


def test_explicit_previous_wifi_recovery_is_used_before_the_ap(tmp_path):
    runner = FakeRunner()
    runner.active = facility_connection_name("Unconfirmed B")
    runner.address = "192.168.50.18/24"
    runner.connection_addresses["Facility A"] = "172.20.10.9/28"
    manager = make_manager(tmp_path, runner)

    restored = manager._restore_previous_connection("Facility A")

    assert restored == "Facility A"
    assert runner.active == "Facility A"
    assert runner.address == "172.20.10.9/28"
    assert not any(
        "intellitrolley-ap" in command and "up" in command
        for command in runner.commands
    )


def test_unconfirmed_target_is_never_accepted_as_the_recovery_fallback(tmp_path):
    runner = FakeRunner()
    runner.active = facility_connection_name("Unconfirmed B")
    runner.address = "192.168.50.18/24"
    runner.client_activation_succeeds = False
    runner.ap_activation_succeeds = False
    manager = make_manager(tmp_path, runner)

    restored = manager._restore_previous_connection("Unavailable Facility A")

    assert restored == ""
    assert manager.last_ready_signature is None
    assert not manager.ready_state_path.exists()


def test_recovery_ap_requires_its_exact_configured_address(tmp_path):
    runner = FakeRunner()
    runner.active = "intellitrolley-ap"
    runner.address = "10.42.1.1/24"
    runner.ap_activation_succeeds = False
    manager = make_manager(tmp_path, runner)

    restored = manager._restore_previous_connection("")

    assert restored == ""
    assert manager.last_ready_signature is None


def test_failed_ready_state_write_remains_retryable(tmp_path):
    manager = make_manager(tmp_path, FakeRunner())

    def fail_ready_state(*args, **kwargs):
        raise OSError("read-only ready-state directory")

    manager._write_ready_state = fail_ready_state
    restored = manager._record_restored_connection(
        "intellitrolley-ap",
        ipaddress.ip_interface("10.42.0.1/24"),
    )

    assert restored == "intellitrolley-ap"
    assert manager.last_ready_signature is None


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
    manager.transition_phase = "pending_confirmation"
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


def test_confirmation_rejects_a_private_client_outside_target_subnet(tmp_path):
    runner = FakeRunner()
    runner.active = facility_connection_name("Facility WiFi")
    runner.address = "192.168.40.18/24"
    manager = make_manager(tmp_path, runner)
    token = "target-subnet-token"
    manager.pending_token_hash = __import__("hashlib").sha256(
        token.encode("utf-8")
    ).hexdigest()
    manager.pending_connection = runner.active
    manager.pending_ssid = "Facility WiFi"
    manager.pending_deadline = time.monotonic() + 120
    manager.pending_checkpoint = "/org/freedesktop/NetworkManager/Checkpoint/8"
    manager.transition_phase = "pending_confirmation"

    with pytest.raises(ProvisioningError, match="active facility Wi-Fi"):
        manager.confirm(remote_address="192.168.41.25", token=token)

    assert manager.pending_checkpoint
    assert not any("CheckpointDestroy" in command for command in runner.commands)


def test_confirmation_commit_is_serialized_against_timeout_rollback(tmp_path):
    runner = FakeRunner()
    runner.active = facility_connection_name("Facility WiFi")
    runner.address = "192.168.40.18/24"
    manager = make_manager(tmp_path, runner)
    token = "serialized-confirmation-token"
    manager.pending_token_hash = __import__("hashlib").sha256(
        token.encode("utf-8")
    ).hexdigest()
    manager.pending_connection = runner.active
    manager.pending_ssid = "Facility WiFi"
    manager.pending_deadline = time.monotonic() + 120
    manager.pending_checkpoint = "/org/freedesktop/NetworkManager/Checkpoint/8"
    manager.transition_phase = "pending_confirmation"
    manager._schedule_robot_service_refresh = lambda **kwargs: None

    modify_started = threading.Event()
    allow_modify = threading.Event()
    original_run = runner.run

    def blocking_run(command, *, timeout=15.0, check=True):
        if command[:3] == ["nmcli", "connection", "modify"] and not modify_started.is_set():
            modify_started.set()
            assert allow_modify.wait(timeout=3)
        return original_run(command, timeout=timeout, check=check)

    runner.run = blocking_run
    confirmed = []
    confirm_errors = []

    def run_confirm():
        try:
            confirmed.append(
                manager.confirm(remote_address="192.168.40.25", token=token)
            )
        except Exception as exc:  # pragma: no cover - asserted below
            confirm_errors.append(exc)

    confirm_thread = threading.Thread(target=run_confirm)
    confirm_thread.start()
    assert modify_started.wait(timeout=3)
    rollback_thread = threading.Thread(
        target=manager._checkpoint_expired,
        args=(manager.pending_checkpoint,),
    )
    rollback_thread.start()
    allow_modify.set()
    confirm_thread.join(timeout=3)
    rollback_thread.join(timeout=3)

    assert not confirm_thread.is_alive()
    assert not rollback_thread.is_alive()
    assert not confirm_errors
    assert confirmed[0]["confirmed"] is True
    assert manager.transition_phase == "idle"
    assert not any("CheckpointRollback" in command for command in runner.commands)


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
    manager.transition_phase = "pending_confirmation"
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
    assert "ROBOT_WIFI_CLIENT_WAIT_S=90" in installer


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
    assert "status.transition_phase" in javascript
    assert 'elements.savedProfilesCard.classList.add("hidden")' in javascript


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


def test_http_ui_is_no_store_and_rejects_mutation_outside_active_wifi(tmp_path):
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
