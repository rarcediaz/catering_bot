"""Tests for the Wi-Fi-before-ROS boot state machine."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

from wifi_startup import Runner, WifiStartup, parse_wifi_profiles  # noqa: E402


CLIENT_UUID = "11111111-2222-3333-4444-555555555555"
AP_UUID = "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"


class FakeRunner(Runner):
    def __init__(self, *, include_client: bool, active: str = "", failures: int = 0):
        self.include_client = include_client
        self.active = active
        self.address = "192.168.20.18/24" if active else ""
        self.failures = failures
        self.commands = []

    def run(self, command, *, timeout=20.0, check=True):
        self.commands.append(list(command))
        if "NAME,UUID,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY" in command:
            lines = [f"intellitrolley-ap:{AP_UUID}:wifi:no:50"]
            if self.include_client:
                lines.append(f"Facility\\:West:{CLIENT_UUID}:wifi:yes:200")
            return subprocess.CompletedProcess(command, 0, "\n".join(lines) + "\n", "")
        if command[:4] == ["nmcli", "-g", "GENERAL.CONNECTION", "device"]:
            return subprocess.CompletedProcess(command, 0, f"{self.active or '--'}\n", "")
        if command[:5] == ["ip", "-j", "-4", "address", "show"]:
            info = []
            if self.address:
                address, prefix = self.address.split("/")
                info.append(
                    {
                        "family": "inet",
                        "scope": "global",
                        "local": address,
                        "prefixlen": int(prefix),
                    }
                )
            return subprocess.CompletedProcess(
                command, 0, json.dumps([{"addr_info": info}]), ""
            )
        if "up" in command and "uuid" in command:
            if self.failures > 0:
                self.failures -= 1
                return subprocess.CompletedProcess(command, 10, "", "not available")
            self.active = "Facility:West"
            self.address = "192.168.20.18/24"
        elif "up" in command and "id" in command:
            self.active = "intellitrolley-ap"
            self.address = "10.42.0.1/24"
        elif "down" in command:
            self.active = ""
            self.address = ""
        return subprocess.CompletedProcess(command, 0, "", "")


def make_startup(tmp_path, runner):
    return WifiStartup(
        interface="wlan0",
        ap_connection="intellitrolley-ap",
        client_wait_s=90,
        state_file=tmp_path / "ready.json",
        runner=runner,
    )


def test_saved_profiles_are_unescaped_filtered_and_priority_sorted():
    profiles = parse_wifi_profiles(
        f"Guest:{CLIENT_UUID}:wifi:yes:10\n"
        f"Facility\\:West:{AP_UUID}:802-11-wireless:true:200\n"
        f"intellitrolley-ap:99999999-8888-7777-6666-555555555555:wifi:yes:50\n"
        "Wired:12345678-1234-1234-1234-123456789012:ethernet:yes:999\n",
        "intellitrolley-ap",
    )
    assert [profile.name for profile in profiles] == ["Facility:West", "Guest"]


def test_boot_uses_saved_wifi_before_access_point(tmp_path):
    runner = FakeRunner(include_client=True)
    startup = make_startup(tmp_path, runner)

    assert startup.run().startswith("Saved Wi-Fi ready")
    state = json.loads(startup.state_file.read_text(encoding="utf-8"))
    assert state["mode"] == "client"
    assert state["connection"] == "Facility:West"
    assert state["address"] == "192.168.20.18/24"
    assert any("uuid" in command for command in runner.commands)
    assert not any("id" in command and "up" in command for command in runner.commands)


def test_boot_retries_saved_wifi_during_full_grace_period(tmp_path):
    runner = FakeRunner(include_client=True, failures=1)
    startup = make_startup(tmp_path, runner)

    assert startup.run().startswith("Saved Wi-Fi ready")
    attempts = [command for command in runner.commands if "uuid" in command and "up" in command]
    assert len(attempts) == 2
    assert not any("id" in command and "up" in command for command in runner.commands)


def test_boot_starts_access_point_when_no_saved_wifi_exists(tmp_path):
    runner = FakeRunner(include_client=False)
    startup = make_startup(tmp_path, runner)

    assert startup.run().startswith("Recovery AP ready")
    state = json.loads(startup.state_file.read_text(encoding="utf-8"))
    assert state["mode"] == "ap"
    assert state["connection"] == "intellitrolley-ap"
    assert state["address"] == "10.42.0.1/24"
    assert any("id" in command and "up" in command for command in runner.commands)


def test_already_connected_client_is_accepted_and_ap_fallback_waits_for_watchdog(tmp_path):
    runner = FakeRunner(include_client=True, active="Facility:West")
    startup = make_startup(tmp_path, runner)

    startup.run()
    assert not any("uuid" in command and "up" in command for command in runner.commands)
    assert any(
        command[:3] == ["nmcli", "connection", "modify"]
        and "connection.autoconnect" in command
        and "no" in command
        for command in runner.commands
    )
