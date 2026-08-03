#!/usr/bin/env python3
"""
Bring up a saved Wi-Fi connection or the robot recovery access point.

This program is intended to run as a systemd oneshot before the ROS service.
NetworkManager still owns wlan0 and all credentials; this only makes its boot
choice deterministic and verifies that the selected connection has IPv4.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import ipaddress
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
from typing import List, Optional


INTERFACE_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]+$")
CONNECTION_PATTERN = re.compile(r"^[A-Za-z0-9._-]+$")


def split_nmcli_terse(line: str) -> List[str]:
    values: List[str] = []
    current: List[str] = []
    escaped = False
    for character in line.rstrip("\n"):
        if escaped:
            current.append(character)
            escaped = False
        elif character == "\\":
            escaped = True
        elif character == ":":
            values.append("".join(current))
            current = []
        else:
            current.append(character)
    if escaped:
        current.append("\\")
    values.append("".join(current))
    return values


@dataclass(frozen=True)
class WifiProfile:
    name: str
    uuid: str
    priority: int


def parse_wifi_profiles(output: str, ap_connection: str) -> List[WifiProfile]:
    profiles: List[WifiProfile] = []
    for line in output.splitlines():
        fields = split_nmcli_terse(line)
        if len(fields) != 5:
            continue
        name, profile_uuid, connection_type, autoconnect, priority_text = fields
        if (
            name == ap_connection
            or connection_type not in {"wifi", "802-11-wireless"}
            or autoconnect.lower() not in {"yes", "true"}
            or not re.fullmatch(r"[A-Fa-f0-9-]{36}", profile_uuid)
        ):
            continue
        try:
            priority = int(priority_text or "0")
        except ValueError:
            priority = 0
        profiles.append(WifiProfile(name, profile_uuid, priority))
    return sorted(profiles, key=lambda profile: (-profile.priority, profile.name.lower()))


class Runner:
    def run(
        self,
        command: List[str],
        *,
        timeout: float = 20.0,
        check: bool = True,
    ) -> subprocess.CompletedProcess:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
            env={
                "LANG": "C.UTF-8",
                "LC_ALL": "C.UTF-8",
                "PATH": "/usr/sbin:/usr/bin:/sbin:/bin",
            },
        )
        if check and result.returncode != 0:
            detail = (result.stderr or result.stdout or "command failed").strip()
            raise RuntimeError(f"{' '.join(command[:3])}: {detail}")
        return result


class WifiStartup:
    def __init__(
        self,
        *,
        interface: str,
        ap_connection: str,
        client_wait_s: int,
        state_file: Path,
        runner: Optional[Runner] = None,
    ):
        if not INTERFACE_PATTERN.fullmatch(interface):
            raise ValueError(f"Invalid Wi-Fi interface: {interface!r}")
        if not CONNECTION_PATTERN.fullmatch(ap_connection):
            raise ValueError(f"Invalid AP connection name: {ap_connection!r}")
        self.interface = interface
        self.ap_connection = ap_connection
        self.client_wait_s = max(0, min(300, int(client_wait_s)))
        self.state_file = state_file
        self.runner = runner or Runner()

    def profiles(self) -> List[WifiProfile]:
        result = self.runner.run(
            [
                "nmcli",
                "--terse",
                "--escape",
                "yes",
                "--fields",
                "NAME,UUID,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY",
                "connection",
                "show",
            ],
            timeout=10,
        )
        return parse_wifi_profiles(result.stdout, self.ap_connection)

    def active_connection(self) -> str:
        result = self.runner.run(
            ["nmcli", "-g", "GENERAL.CONNECTION", "device", "show", self.interface],
            timeout=5,
            check=False,
        )
        return result.stdout.strip() if result.returncode == 0 else ""

    def ipv4_address(self) -> Optional[ipaddress.IPv4Interface]:
        result = self.runner.run(
            ["ip", "-j", "-4", "address", "show", "dev", self.interface],
            timeout=5,
            check=False,
        )
        if result.returncode != 0:
            return None
        try:
            devices = json.loads(result.stdout)
        except ValueError:
            return None
        for device in devices:
            for address in device.get("addr_info", []):
                if address.get("family") == "inet" and address.get("scope") == "global":
                    try:
                        return ipaddress.ip_interface(
                            f"{address['local']}/{int(address['prefixlen'])}"
                        )
                    except (KeyError, TypeError, ValueError):
                        continue
        return None

    def ready(self, *, allow_ap: bool) -> Optional[tuple[str, ipaddress.IPv4Interface]]:
        connection = self.active_connection()
        address = self.ipv4_address()
        if not connection or connection == "--" or address is None:
            return None
        if not allow_ap and connection == self.ap_connection:
            return None
        return connection, address

    def set_ap_autoconnect(self, enabled: bool) -> None:
        self.runner.run(
            [
                "nmcli",
                "connection",
                "modify",
                self.ap_connection,
                "connection.autoconnect",
                "yes" if enabled else "no",
                "connection.autoconnect-priority",
                "50",
            ],
            timeout=10,
        )

    def activate_profile(self, profile: WifiProfile, timeout_s: int) -> bool:
        result = self.runner.run(
            [
                "nmcli",
                "--wait",
                str(timeout_s),
                "connection",
                "up",
                "uuid",
                profile.uuid,
                "ifname",
                self.interface,
            ],
            timeout=timeout_s + 5,
            check=False,
        )
        return result.returncode == 0 and self.ready(allow_ap=False) is not None

    def write_state(self, mode: str, connection: str, address: ipaddress.IPv4Interface) -> None:
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "ready": True,
            "mode": mode,
            "interface": self.interface,
            "connection": connection,
            "address": str(address),
            "timestamp": time.time(),
        }
        fd, temporary_name = tempfile.mkstemp(
            prefix=f".{self.state_file.name}.", dir=str(self.state_file.parent), text=True
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, sort_keys=True)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.chmod(temporary_name, 0o644)
            os.replace(temporary_name, self.state_file)
        finally:
            if os.path.exists(temporary_name):
                os.unlink(temporary_name)

    def run(self) -> str:
        existing = self.ready(allow_ap=False)
        if existing is not None:
            connection, address = existing
            # Keep the AP eligible for runtime recovery if this client later
            # disappears; its lower priority prevents it replacing the client.
            self.set_ap_autoconnect(True)
            self.write_state("client", connection, address)
            return f"Saved Wi-Fi ready: {connection} on {address}"

        profiles = self.profiles()
        self.set_ap_autoconnect(False)
        if self.active_connection() == self.ap_connection:
            self.runner.run(
                ["nmcli", "connection", "down", "id", self.ap_connection],
                timeout=15,
                check=False,
            )

        deadline = time.monotonic() + self.client_wait_s
        for profile in profiles:
            remaining = int(deadline - time.monotonic())
            if remaining <= 0:
                break
            attempt_s = max(1, min(15, remaining))
            print(f"Trying saved Wi-Fi profile {profile.name!r}...", flush=True)
            if self.activate_profile(profile, attempt_s):
                connection, address = self.ready(allow_ap=False)  # type: ignore[misc]
                self.set_ap_autoconnect(True)
                self.write_state("client", connection, address)
                return f"Saved Wi-Fi ready: {connection} on {address}"

        self.set_ap_autoconnect(True)
        result = self.runner.run(
            [
                "nmcli",
                "--wait",
                "30",
                "connection",
                "up",
                "id",
                self.ap_connection,
                "ifname",
                self.interface,
            ],
            timeout=35,
            check=False,
        )
        ready = self.ready(allow_ap=True)
        if result.returncode != 0 or ready is None or ready[0] != self.ap_connection:
            detail = (result.stderr or result.stdout or "no IPv4 address").strip()
            raise RuntimeError(
                f"No saved Wi-Fi connected and recovery AP {self.ap_connection!r} "
                f"could not start: {detail}"
            )
        connection, address = ready
        self.write_state("ap", connection, address)
        return f"Recovery AP ready: {connection} on {address}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--interface", default=os.getenv("ROBOT_WIFI_INTERFACE", "wlan0")
    )
    parser.add_argument(
        "--ap-connection",
        default=os.getenv("ROBOT_WIFI_AP_CONNECTION", "intellitrolley-ap"),
    )
    parser.add_argument(
        "--client-wait",
        type=int,
        default=int(os.getenv("ROBOT_WIFI_CLIENT_WAIT_S", "30")),
    )
    parser.add_argument(
        "--state-file",
        type=Path,
        default=Path(os.getenv("ROBOT_WIFI_READY_STATE", "/run/my-bot-network/ready.json")),
    )
    args = parser.parse_args()
    startup = WifiStartup(
        interface=args.interface,
        ap_connection=args.ap_connection,
        client_wait_s=args.client_wait,
        state_file=args.state_file,
    )
    try:
        message = startup.run()
    except (OSError, RuntimeError, subprocess.TimeoutExpired, ValueError) as exc:
        # A failed client attempt must never strand the recovery profile with
        # autoconnect disabled. Ignore this best-effort cleanup only when the
        # AP profile itself is missing or NetworkManager is unavailable.
        try:
            startup.set_ap_autoconnect(True)
        except (OSError, RuntimeError, subprocess.TimeoutExpired):
            pass
        print(f"Wi-Fi startup failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
    print(message, flush=True)


if __name__ == "__main__":
    main()
