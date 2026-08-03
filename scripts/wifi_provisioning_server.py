#!/usr/bin/env python3
"""Recovery-safe Wi-Fi provisioning UI for the IntelliTrolley Raspberry Pi.

The server deliberately does not activate or modify networking on startup.
Mutating operations are accepted only from a client attached to the configured
Pi access-point subnet while that access-point profile is active. Switching to
a staged facility profile runs inside a NetworkManager checkpoint, so failure
to confirm the new connection automatically restores the access point.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import os
from pathlib import Path
import re
import secrets
import shutil
import socket
import subprocess
import tempfile
import threading
import time
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import quote, urlparse
import uuid


FACILITY_CONNECTION_PREFIX = "intellitrolley-facility-"
MAX_BODY_BYTES = 8192
SUPPORTED_SECURITY = {"wpa-psk", "open"}
CONNECTION_PATTERN = re.compile(r"^[A-Za-z0-9._-]+$")
INTERFACE_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]+$")
CHECKPOINT_ROLLBACK_GRACE_S = 30


class ProvisioningError(RuntimeError):
    """An expected, user-facing provisioning failure."""

    def __init__(self, message: str, status: int = HTTPStatus.BAD_REQUEST):
        super().__init__(message)
        self.status = int(status)


def validate_ssid(value: Any) -> str:
    ssid = str(value or "").strip()
    if not ssid or any(ord(character) < 32 or ord(character) == 127 for character in ssid):
        raise ProvisioningError(
            "SSID must contain 1-32 bytes and cannot contain control characters."
        )
    if len(ssid.encode("utf-8")) > 32:
        raise ProvisioningError("SSID must be at most 32 bytes.")
    return ssid


def validate_psk(value: Any, security: str) -> str:
    psk = str(value or "")
    if security == "open":
        if psk:
            raise ProvisioningError("Leave the password blank for an open network.")
        return ""
    if 8 <= len(psk) <= 63 and all(32 <= ord(character) <= 126 for character in psk):
        return psk
    if len(psk) == 64 and re.fullmatch(r"[A-Fa-f0-9]{64}", psk):
        return psk
    raise ProvisioningError(
        "Wi-Fi password must contain 8-63 printable ASCII characters or "
        "exactly 64 hexadecimal characters."
    )


def validate_interface(value: str) -> str:
    if not INTERFACE_PATTERN.fullmatch(value):
        raise ValueError(f"Invalid Wi-Fi interface: {value!r}")
    return value


def validate_connection_name(value: str) -> str:
    if not CONNECTION_PATTERN.fullmatch(value):
        raise ValueError(f"Invalid NetworkManager connection name: {value!r}")
    return value


def facility_connection_name(ssid: str) -> str:
    digest = hashlib.sha256(ssid.encode("utf-8")).hexdigest()[:12]
    return f"{FACILITY_CONNECTION_PREFIX}{digest}"


def facility_connection_uuid(ssid: str) -> str:
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"intellitrolley:wifi:{ssid}"))


def keyfile_ssid_value(ssid: str) -> str:
    """Encode arbitrary UTF-8 SSIDs without allowing keyfile syntax injection."""
    return ";".join(str(byte) for byte in ssid.encode("utf-8")) + ";"


def render_facility_keyfile(
    *,
    ssid: str,
    psk: str,
    security: str,
    interface: str,
    connection_name: str,
    hidden: bool,
) -> str:
    ssid = validate_ssid(ssid)
    if security not in SUPPORTED_SECURITY:
        raise ProvisioningError(
            "This version supports WPA/WPA2 Personal and open networks only."
        )
    psk = validate_psk(psk, security)
    validate_interface(interface)
    validate_connection_name(connection_name)

    lines = [
        "[connection]",
        f"id={connection_name}",
        f"uuid={facility_connection_uuid(ssid)}",
        "type=wifi",
        f"interface-name={interface}",
        "autoconnect=false",
        "autoconnect-priority=200",
        "autoconnect-retries=2",
        "",
        "[wifi]",
        "mode=infrastructure",
        f"ssid={keyfile_ssid_value(ssid)}",
        "cloned-mac-address=permanent",
    ]
    if hidden:
        lines.append("hidden=true")
    if security == "wpa-psk":
        lines.append("security=wifi-security")
        lines.extend(
            [
                "",
                "[wifi-security]",
                "key-mgmt=wpa-psk",
                f"psk={psk}",
            ]
        )
    lines.extend(
        [
            "",
            "[ipv4]",
            "method=auto",
            "",
            "[ipv6]",
            "method=auto",
            "",
        ]
    )
    return "\n".join(lines)


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


def parse_nmcli_networks(output: str) -> List[Dict[str, Any]]:
    by_ssid: Dict[str, Dict[str, Any]] = {}
    for line in output.splitlines():
        fields = split_nmcli_terse(line)
        if len(fields) < 3:
            continue
        ssid = fields[0].strip()
        if not ssid:
            continue
        security_text = fields[1].strip()
        try:
            signal = max(0, min(100, int(fields[2])))
        except ValueError:
            signal = 0
        security_upper = security_text.upper()
        if "802.1X" in security_upper or "EAP" in security_upper:
            security = "enterprise"
        elif not security_text or security_text == "--":
            security = "open"
        else:
            security = "wpa-psk"
        candidate = {
            "ssid": ssid,
            "security": security,
            "security_label": security_text or "Open",
            "signal": signal,
        }
        if ssid not in by_ssid or signal > int(by_ssid[ssid]["signal"]):
            by_ssid[ssid] = candidate
    return sorted(
        by_ssid.values(),
        key=lambda item: (-int(item["signal"]), str(item["ssid"]).lower()),
    )


def update_env_file(path: Path, replacements: Dict[str, str]) -> bool:
    if not path.exists():
        return False
    original = path.read_text(encoding="utf-8")
    remaining = dict(replacements)
    output: List[str] = []
    for line in original.splitlines():
        match = re.match(r"^([A-Z][A-Z0-9_]*)=", line)
        if match and match.group(1) in remaining:
            key = match.group(1)
            output.append(f"{key}={remaining.pop(key)}")
        else:
            output.append(line)
    if remaining:
        if output and output[-1]:
            output.append("")
        output.extend(f"{key}={value}" for key, value in remaining.items())
    rendered = "\n".join(output) + "\n"
    if rendered == original:
        return True

    stat_result = path.stat()
    temporary_fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        dir=str(path.parent),
        text=True,
    )
    try:
        with os.fdopen(temporary_fd, "w", encoding="utf-8") as handle:
            handle.write(rendered)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary_name, stat_result.st_mode & 0o777)
        try:
            os.chown(temporary_name, stat_result.st_uid, stat_result.st_gid)
        except PermissionError:
            pass
        os.replace(temporary_name, path)
    finally:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)
    return True


class CommandRunner:
    def run(
        self,
        command: List[str],
        *,
        timeout: float = 15.0,
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
            raise ProvisioningError(detail, HTTPStatus.BAD_GATEWAY)
        return result


class ProvisioningManager:
    def __init__(
        self,
        *,
        interface: str,
        ap_connection: str,
        ap_cidr: str,
        hostname: str,
        port: int,
        switch_timeout_s: int,
        network_connections_dir: Path,
        state_dir: Path,
        robot_defaults_path: Path,
        robot_service: str,
        runner: Optional[CommandRunner] = None,
    ):
        self.interface = validate_interface(interface)
        self.ap_connection = validate_connection_name(ap_connection)
        self.ap_network = ipaddress.ip_interface(ap_cidr)
        if not self.ap_network.ip.is_private:
            raise ValueError("The AP address must be private.")
        self.hostname = hostname
        self.port = int(port)
        self.switch_timeout_s = max(60, min(600, int(switch_timeout_s)))
        self.network_connections_dir = network_connections_dir
        self.state_dir = state_dir
        self.robot_defaults_path = robot_defaults_path
        self.robot_service = robot_service
        self.runner = runner or CommandRunner()
        self.lock = threading.RLock()
        self.switch_timer: Optional[threading.Timer] = None
        self.rollback_timer: Optional[threading.Timer] = None
        self.pending_checkpoint = ""
        self.pending_token_hash = ""
        self.pending_connection = ""
        self.pending_ssid = ""
        self.pending_deadline = 0.0
        self.pending_robot_was_active = False
        self.last_result = ""
        self.staged_connection = ""
        self.staged_ssid = ""
        self.staged_security = ""
        self._load_state()

    @property
    def state_path(self) -> Path:
        return self.state_dir / "state.json"

    def _load_state(self) -> None:
        try:
            payload = json.loads(self.state_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, ValueError, OSError):
            return
        self.staged_connection = str(payload.get("staged_connection") or "")
        self.staged_ssid = str(payload.get("staged_ssid") or "")
        self.staged_security = str(payload.get("staged_security") or "")
        if payload.get("phase") == "pending":
            self.pending_robot_was_active = bool(
                payload.get("pending_robot_was_active", False)
            )
            self.last_result = (
                "A previous switch was interrupted. NetworkManager will restore "
                "the previous connection through its checkpoint."
            )

    def _save_state(self, phase: str) -> None:
        self.state_dir.mkdir(parents=True, exist_ok=True)
        payload = {
            "phase": phase,
            "staged_connection": self.staged_connection,
            "staged_ssid": self.staged_ssid,
            "staged_security": self.staged_security,
            "pending_connection": self.pending_connection,
            "pending_ssid": self.pending_ssid,
            "pending_deadline": self.pending_deadline,
            "pending_robot_was_active": self.pending_robot_was_active,
            "last_result": self.last_result,
        }
        temporary_fd, temporary_name = tempfile.mkstemp(
            prefix=".state.",
            dir=str(self.state_dir),
            text=True,
        )
        try:
            with os.fdopen(temporary_fd, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.chmod(temporary_name, 0o600)
            os.replace(temporary_name, self.state_path)
        finally:
            if os.path.exists(temporary_name):
                os.unlink(temporary_name)

    def active_connection(self) -> str:
        result = self.runner.run(
            [
                "nmcli",
                "-g",
                "GENERAL.CONNECTION",
                "device",
                "show",
                self.interface,
            ],
            timeout=5,
        )
        return result.stdout.strip()

    def interface_ipv4(self) -> Optional[ipaddress.IPv4Interface]:
        result = self.runner.run(
            ["ip", "-j", "-4", "address", "show", "dev", self.interface],
            timeout=5,
        )
        try:
            payload = json.loads(result.stdout)
        except ValueError as exc:
            raise ProvisioningError(
                f"Could not read the Pi network address: {exc}",
                HTTPStatus.BAD_GATEWAY,
            ) from exc
        for device in payload:
            for address in device.get("addr_info", []):
                if address.get("family") != "inet" or address.get("scope") != "global":
                    continue
                return ipaddress.ip_interface(
                    f"{address['local']}/{int(address['prefixlen'])}"
                )
        return None

    def is_ap_client(self, remote_address: str) -> bool:
        try:
            remote = ipaddress.ip_address(remote_address)
        except ValueError:
            return False
        if remote.version != 4 or remote not in self.ap_network.network:
            return False
        try:
            return self.active_connection() == self.ap_connection
        except ProvisioningError:
            return False

    def status(self, remote_address: str) -> Dict[str, Any]:
        try:
            active_connection = self.active_connection()
        except ProvisioningError:
            active_connection = "Unavailable"
        try:
            interface_address = self.interface_ipv4()
        except ProvisioningError:
            interface_address = None
        with self.lock:
            remaining_s = max(0.0, self.pending_deadline - time.monotonic())
            pending = bool(
                self.pending_connection
                and remaining_s > 0.0
            )
            return {
                "interface": self.interface,
                "active_connection": active_connection,
                "interface_address": (
                    str(interface_address) if interface_address else None
                ),
                "ap_connection": self.ap_connection,
                "ap_gateway": str(self.ap_network),
                "can_provision": self.is_ap_client(remote_address),
                "staged": (
                    {
                        "ssid": self.staged_ssid,
                        "security": self.staged_security,
                    }
                    if self.staged_connection
                    else None
                ),
                "pending": (
                    {
                        "ssid": self.pending_ssid,
                        "expires_in_s": int(remaining_s),
                    }
                    if pending
                    else None
                ),
                "last_result": self.last_result,
                "hostname": self.hostname,
                "port": self.port,
            }

    def scan_networks(self, remote_address: str) -> List[Dict[str, Any]]:
        if not self.is_ap_client(remote_address):
            raise ProvisioningError(
                "Network scanning is available only through the Pi recovery hotspot.",
                HTTPStatus.FORBIDDEN,
            )
        result = self.runner.run(
            [
                "nmcli",
                "--terse",
                "--escape",
                "yes",
                "--fields",
                "SSID,SECURITY,SIGNAL",
                "device",
                "wifi",
                "list",
                "ifname",
                self.interface,
                "--rescan",
                "auto",
            ],
            timeout=20,
        )
        return parse_nmcli_networks(result.stdout)

    def stage(
        self,
        *,
        remote_address: str,
        ssid: Any,
        password: Any,
        security: Any,
        hidden: Any,
    ) -> Dict[str, Any]:
        if not self.is_ap_client(remote_address):
            raise ProvisioningError(
                "Wi-Fi profiles can be changed only through the Pi recovery hotspot.",
                HTTPStatus.FORBIDDEN,
            )
        clean_ssid = validate_ssid(ssid)
        clean_security = str(security or "wpa-psk")
        if clean_security not in SUPPORTED_SECURITY:
            raise ProvisioningError(
                "This version supports Personal-password and open Wi-Fi only."
            )
        clean_psk = validate_psk(password, clean_security)
        connection_name = facility_connection_name(clean_ssid)
        keyfile = render_facility_keyfile(
            ssid=clean_ssid,
            psk=clean_psk,
            security=clean_security,
            interface=self.interface,
            connection_name=connection_name,
            hidden=bool(hidden),
        )
        self.network_connections_dir.mkdir(parents=True, exist_ok=True)
        destination = (
            self.network_connections_dir / f"{connection_name}.nmconnection"
        )
        temporary_fd, temporary_name = tempfile.mkstemp(
            prefix=f".{connection_name}.",
            dir=str(self.network_connections_dir),
            text=True,
        )
        try:
            with os.fdopen(temporary_fd, "w", encoding="utf-8") as handle:
                handle.write(keyfile)
                handle.flush()
                os.fsync(handle.fileno())
            os.chmod(temporary_name, 0o600)
            os.replace(temporary_name, destination)
        finally:
            clean_psk = ""
            if os.path.exists(temporary_name):
                os.unlink(temporary_name)

        self.runner.run(["nmcli", "connection", "reload"], timeout=10)
        self.runner.run(
            ["nmcli", "connection", "show", "id", connection_name],
            timeout=10,
        )
        with self.lock:
            self.staged_connection = connection_name
            self.staged_ssid = clean_ssid
            self.staged_security = clean_security
            self.last_result = (
                f"Saved {clean_ssid}. The Pi is still on {self.ap_connection}."
            )
            self._save_state("staged")
        return {
            "staged": True,
            "ssid": clean_ssid,
            "message": self.last_result,
        }

    def activate(self, remote_address: str) -> Dict[str, Any]:
        if not self.is_ap_client(remote_address):
            raise ProvisioningError(
                "The facility switch must begin from the Pi recovery hotspot.",
                HTTPStatus.FORBIDDEN,
            )
        with self.lock:
            if self.pending_checkpoint or self.switch_timer is not None:
                raise ProvisioningError(
                    "A Wi-Fi switch is already pending.",
                    HTTPStatus.CONFLICT,
                )
            if not self.staged_connection:
                raise ProvisioningError("Save a facility Wi-Fi profile first.")
            token = secrets.token_urlsafe(24)
            self.pending_token_hash = hashlib.sha256(
                token.encode("utf-8")
            ).hexdigest()
            self.pending_connection = self.staged_connection
            self.pending_ssid = self.staged_ssid
            # A Raspberry Pi without an RTC can correct its wall clock as soon
            # as facility internet becomes available. Never use wall time for
            # a safety rollback deadline.
            self.pending_deadline = time.monotonic() + self.switch_timeout_s
            self.last_result = (
                "Switch scheduled. It will roll back automatically unless confirmed."
            )
            self._save_state("pending")
            self.switch_timer = threading.Timer(
                2.0,
                self._start_checkpoint_switch,
            )
            self.switch_timer.daemon = True
            self.switch_timer.start()
        confirm_url = (
            f"http://{self.hostname}:{self.port}/"
            f"?confirm={quote(token, safe='')}"
        )
        return {
            "accepted": True,
            "ssid": self.pending_ssid,
            "timeout_s": self.switch_timeout_s,
            "confirm_url": confirm_url,
            "message": (
                "Connect the Windows central computer to the facility Wi-Fi, "
                "then open the confirmation link before the deadline."
            ),
        }

    def _start_checkpoint_switch(self) -> None:
        with self.lock:
            self.switch_timer = None
            connection_name = self.pending_connection
            if not connection_name:
                return
        try:
            checkpoint = self._create_checkpoint()
            with self.lock:
                if self.pending_connection != connection_name:
                    self._rollback_checkpoint(checkpoint)
                    return
                self.pending_checkpoint = checkpoint
            robot_was_active = self._stop_robot_service_for_switch()
            with self.lock:
                self.pending_robot_was_active = robot_was_active
                self._save_state("pending")
            self.runner.run(
                [
                    "nmcli",
                    "--wait",
                    "45",
                    "connection",
                    "up",
                    "id",
                    connection_name,
                    "ifname",
                    self.interface,
                ],
                timeout=55,
            )
            if self.active_connection() != connection_name:
                raise ProvisioningError(
                    "Facility Wi-Fi activation returned without selecting the "
                    "staged connection.",
                    HTTPStatus.BAD_GATEWAY,
                )
            if self.interface_ipv4() is None:
                raise ProvisioningError(
                    "Facility Wi-Fi connected without receiving an IPv4 address.",
                    HTTPStatus.BAD_GATEWAY,
                )
        except (OSError, ProvisioningError, subprocess.TimeoutExpired) as exc:
            with self.lock:
                checkpoint = self.pending_checkpoint
            if checkpoint:
                self._rollback_checkpoint(checkpoint)
            with self.lock:
                restart_robot = self.pending_robot_was_active
                self.pending_checkpoint = ""
                self.last_result = f"Could not activate facility Wi-Fi: {exc}"
                self.pending_connection = ""
                self.pending_ssid = ""
                self.pending_deadline = 0.0
                self.pending_token_hash = ""
                self.pending_robot_was_active = False
                self._save_state("failed")
            self._ensure_recovery_ap()
            self._schedule_robot_service_refresh(
                ensure_running=restart_robot,
                delay_s=3.0,
            )
            return
        self._schedule_robot_service_refresh(
            ensure_running=True,
            delay_s=1.0,
        )
        with self.lock:
            self.last_result = (
                f"Connected to {self.pending_ssid}; the robot service is "
                "restarting on the facility network while confirmation remains pending."
            )
            self._save_state("pending")
            self.rollback_timer = threading.Timer(
                max(1.0, self.pending_deadline - time.monotonic()),
                self._checkpoint_expired,
                args=(checkpoint,),
            )
            self.rollback_timer.daemon = True
            self.rollback_timer.start()

    def _networkmanager_device_path(self) -> str:
        result = self.runner.run(
            [
                "nmcli",
                "-g",
                "GENERAL.DBUS-PATH",
                "device",
                "show",
                self.interface,
            ],
            timeout=5,
        )
        path = result.stdout.strip()
        if not re.fullmatch(r"/org/freedesktop/NetworkManager/Devices/[0-9]+", path):
            raise ProvisioningError(
                "NetworkManager did not return a valid Wi-Fi device path.",
                HTTPStatus.BAD_GATEWAY,
            )
        return path

    def checkpoint_create_command(self) -> List[str]:
        return [
            "busctl",
            "call",
            "org.freedesktop.NetworkManager",
            "/org/freedesktop/NetworkManager",
            "org.freedesktop.NetworkManager",
            "CheckpointCreate",
            "aouu",
            "1",
            self._networkmanager_device_path(),
            str(self.switch_timeout_s + CHECKPOINT_ROLLBACK_GRACE_S),
            "0",
        ]

    def _create_checkpoint(self) -> str:
        result = self.runner.run(
            self.checkpoint_create_command(),
            timeout=10,
        )
        match = re.fullmatch(
            r'\s*o\s+"(/org/freedesktop/NetworkManager/Checkpoint/[0-9]+)"\s*',
            result.stdout,
        )
        if not match:
            raise ProvisioningError(
                "NetworkManager did not return a checkpoint path.",
                HTTPStatus.BAD_GATEWAY,
            )
        return match.group(1)

    def checkpoint_action_command(self, action: str, checkpoint: str) -> List[str]:
        if action not in {"CheckpointDestroy", "CheckpointRollback"}:
            raise ValueError(f"Invalid checkpoint action: {action}")
        if not re.fullmatch(
            r"/org/freedesktop/NetworkManager/Checkpoint/[0-9]+",
            checkpoint,
        ):
            raise ValueError("Invalid NetworkManager checkpoint path.")
        return [
            "busctl",
            "call",
            "org.freedesktop.NetworkManager",
            "/org/freedesktop/NetworkManager",
            "org.freedesktop.NetworkManager",
            action,
            "o",
            checkpoint,
        ]

    def _rollback_checkpoint(self, checkpoint: str) -> bool:
        try:
            result = self.runner.run(
                self.checkpoint_action_command(
                    "CheckpointRollback",
                    checkpoint,
                ),
                timeout=15,
                check=False,
            )
            return result.returncode == 0
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            return False

    def _ensure_recovery_ap(self) -> None:
        try:
            if self.active_connection() == self.ap_connection:
                return
        except ProvisioningError:
            pass
        try:
            self.runner.run(
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
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            pass

    def _checkpoint_expired(self, checkpoint: str) -> None:
        with self.lock:
            self.rollback_timer = None
            if self.pending_checkpoint != checkpoint:
                return
            self.pending_checkpoint = ""
            restart_robot = True
            if self.pending_connection:
                self.last_result = (
                    "Facility Wi-Fi was not confirmed; NetworkManager restored "
                    "the previous Pi hotspot."
                )
                self.pending_connection = ""
                self.pending_ssid = ""
                self.pending_deadline = 0.0
                self.pending_token_hash = ""
                self.pending_robot_was_active = False
                self._save_state("rolled_back")
        # ROS may already be running on the unconfirmed facility network.
        # Stop it before NetworkManager restores the AP address.
        try:
            self._stop_robot_service_for_switch()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            pass
        self._rollback_checkpoint(checkpoint)
        self._ensure_recovery_ap()
        self._schedule_robot_service_refresh(
            ensure_running=restart_robot,
            delay_s=1.0,
        )

    def confirm(
        self,
        *,
        remote_address: str,
        token: Any,
    ) -> Dict[str, Any]:
        try:
            central_peer = ipaddress.ip_address(remote_address)
        except ValueError as exc:
            raise ProvisioningError("Invalid central-computer address.") from exc
        if (
            central_peer.version != 4
            or not central_peer.is_private
            or central_peer in self.ap_network.network
        ):
            raise ProvisioningError(
                "Confirm from the Windows central computer on the facility network.",
                HTTPStatus.FORBIDDEN,
            )

        supplied_hash = hashlib.sha256(
            str(token or "").encode("utf-8")
        ).hexdigest()
        with self.lock:
            if (
                not self.pending_token_hash
                or not hmac.compare_digest(
                    supplied_hash,
                    self.pending_token_hash,
                )
            ):
                raise ProvisioningError(
                    "The confirmation token is invalid or expired.",
                    HTTPStatus.FORBIDDEN,
                )
            if time.monotonic() >= self.pending_deadline:
                raise ProvisioningError(
                    "The confirmation period expired; reconnect to the Pi hotspot.",
                    HTTPStatus.CONFLICT,
                )
            if self.active_connection() != self.pending_connection:
                raise ProvisioningError(
                    "The staged facility connection is not active yet.",
                    HTTPStatus.CONFLICT,
                )
            checkpoint = self.pending_checkpoint
            if not checkpoint:
                raise ProvisioningError(
                    "The NetworkManager checkpoint is unavailable; allow it to roll back.",
                    HTTPStatus.CONFLICT,
                )
            committed_ssid = self.pending_ssid
            committed_connection = self.pending_connection

        interface_address = self.interface_ipv4()
        if interface_address is None:
            raise ProvisioningError(
                "Facility Wi-Fi is active but the Pi has no IPv4 address.",
                HTTPStatus.CONFLICT,
            )
        self.runner.run(
            [
                "nmcli",
                "connection",
                "modify",
                self.ap_connection,
                "connection.autoconnect",
                "yes",
                "connection.autoconnect-priority",
                "50",
            ],
            timeout=10,
        )
        self.runner.run(
            [
                "nmcli",
                "connection",
                "modify",
                committed_connection,
                "connection.autoconnect",
                "yes",
                "connection.autoconnect-priority",
                "200",
                "connection.autoconnect-retries",
                "2",
            ],
            timeout=10,
        )

        # Commit only after every NetworkManager validation and profile update
        # succeeds. Any failure before CheckpointDestroy leaves the checkpoint
        # armed, so NetworkManager restores the recovery hotspot automatically.
        with self.lock:
            if (
                self.pending_checkpoint != checkpoint
                or not hmac.compare_digest(
                    supplied_hash,
                    self.pending_token_hash,
                )
            ):
                raise ProvisioningError(
                    "The NetworkManager checkpoint ended before confirmation.",
                    HTTPStatus.CONFLICT,
                )
            self.runner.run(
                self.checkpoint_action_command(
                    "CheckpointDestroy",
                    checkpoint,
                ),
                timeout=15,
            )
            self.pending_checkpoint = ""
            if self.rollback_timer is not None:
                self.rollback_timer.cancel()
                self.rollback_timer = None
            self.pending_connection = ""
            self.pending_ssid = ""
            self.pending_deadline = 0.0
            self.pending_token_hash = ""
            restart_robot = True
            self.pending_robot_was_active = False

        handoff = self._apply_network_handoff(
            interface_address=interface_address,
            central_peer=central_peer,
            ensure_robot_running=restart_robot,
        )
        with self.lock:
            self.last_result = (
                f"Confirmed {committed_ssid}; the Pi recovery hotspot remains "
                "saved as an automatic fallback."
            )
            self._save_state("committed")

        return {
            "confirmed": True,
            "ssid": committed_ssid,
            **handoff,
            "message": self.last_result,
        }

    def configure_current_ap(self, remote_address: str) -> Dict[str, Any]:
        if not self.is_ap_client(remote_address):
            raise ProvisioningError(
                "Hotspot configuration is available only through the active "
                "Pi recovery hotspot.",
                HTTPStatus.FORBIDDEN,
            )
        central_peer = ipaddress.ip_address(remote_address)
        interface_address = self.interface_ipv4()
        if (
            interface_address is None
            or interface_address.ip != self.ap_network.ip
            or interface_address.network != self.ap_network.network
        ):
            raise ProvisioningError(
                "The recovery hotspot does not have its configured IPv4 address.",
                HTTPStatus.CONFLICT,
            )
        handoff = self._apply_network_handoff(
            interface_address=interface_address,
            central_peer=central_peer,
            ensure_robot_running=True,
        )
        with self.lock:
            self.last_result = (
                "The Pi hotspot is now the selected robot network; no facility "
                "Wi-Fi switch was requested."
            )
            self._save_state("ap_configured")
        return {
            "confirmed": True,
            "ssid": self.ap_connection,
            **handoff,
            "message": self.last_result,
        }

    def _apply_network_handoff(
        self,
        *,
        interface_address: ipaddress.IPv4Interface,
        central_peer: ipaddress.IPv4Address,
        ensure_robot_running: bool = False,
    ) -> Dict[str, Any]:
        ros_domain_id = self._ros_domain_id()
        try:
            defaults_updated = update_env_file(
                self.robot_defaults_path,
                {
                    "ROBOT_CYCLONEDDS_PEERS": str(central_peer),
                    "ROBOT_CYCLONEDDS_INTERFACE": self.interface,
                    "ROBOT_CYCLONEDDS_ALLOW_MULTICAST": "spdp",
                },
            )
        except OSError:
            defaults_updated = False
        robot_network = interface_address.network
        configuration_uri = (
            "intellitrolley://configure-network"
            f"?robot={quote(str(interface_address.ip), safe='')}"
            f"&subnet={quote(str(robot_network), safe='')}"
            f"&domain={ros_domain_id}"
        )
        if defaults_updated or ensure_robot_running:
            self._schedule_robot_service_refresh(
                ensure_running=ensure_robot_running,
                delay_s=2.0,
            )

        return {
            "robot_address": str(interface_address.ip),
            "robot_subnet": str(robot_network),
            "central_address": str(central_peer),
            "ros_domain_id": ros_domain_id,
            "robot_defaults_updated": defaults_updated,
            "configuration_uri": configuration_uri,
        }

    def _ros_domain_id(self) -> int:
        try:
            content = self.robot_defaults_path.read_text(encoding="utf-8")
        except OSError:
            return 0
        match = re.search(r"(?m)^ROS_DOMAIN_ID=([0-9]+)$", content)
        if not match:
            return 0
        return max(0, min(232, int(match.group(1))))

    def _stop_robot_service_for_switch(self) -> bool:
        # Stop is idempotent and also catches an activating/restarting unit.
        # Every intentional interface transition is followed by an ensured
        # start, regardless of the service state before the switch.
        self.runner.run(
            ["systemctl", "stop", self.robot_service],
            timeout=45,
        )
        return True

    def _schedule_robot_service_refresh(
        self,
        *,
        ensure_running: bool,
        delay_s: float,
    ) -> None:
        refresh_thread = threading.Thread(
            target=self._refresh_robot_service,
            kwargs={"ensure_running": ensure_running, "delay_s": delay_s},
            daemon=True,
        )
        refresh_thread.start()

    def _refresh_robot_service(self, *, ensure_running: bool, delay_s: float) -> None:
        time.sleep(delay_s)
        try:
            active = self.runner.run(
                ["systemctl", "is-active", "--quiet", self.robot_service],
                timeout=8,
                check=False,
            )
            if active.returncode == 0:
                self.runner.run(
                    ["systemctl", "restart", self.robot_service],
                    timeout=45,
                )
            elif ensure_running:
                self.runner.run(
                    ["systemctl", "start", self.robot_service],
                    timeout=45,
                )
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            return

    def forget_staged(self, remote_address: str) -> Dict[str, Any]:
        if not self.is_ap_client(remote_address):
            raise ProvisioningError(
                "Facility profiles can be removed only through the Pi recovery hotspot.",
                HTTPStatus.FORBIDDEN,
            )
        with self.lock:
            if self.pending_checkpoint or self.switch_timer is not None:
                raise ProvisioningError(
                    "Wait for the current switch to finish.",
                    HTTPStatus.CONFLICT,
                )
            connection_name = self.staged_connection
            if not connection_name:
                return {"forgotten": False, "message": "No staged profile exists."}
        if not connection_name.startswith(FACILITY_CONNECTION_PREFIX):
            raise ProvisioningError(
                "Refusing to remove a profile not owned by IntelliTrolley.",
                HTTPStatus.CONFLICT,
            )
        self.runner.run(
            ["nmcli", "connection", "delete", "id", connection_name],
            timeout=15,
            check=False,
        )
        keyfile = (
            self.network_connections_dir / f"{connection_name}.nmconnection"
        )
        try:
            keyfile.unlink()
        except FileNotFoundError:
            pass
        self.runner.run(
            ["nmcli", "connection", "reload"],
            timeout=10,
            check=False,
        )
        with self.lock:
            self.staged_connection = ""
            self.staged_ssid = ""
            self.staged_security = ""
            self.last_result = (
                f"Removed {connection_name}. The Pi remains on the recovery hotspot."
            )
            self._save_state("idle")
        return {"forgotten": True, "message": self.last_result}


class ProvisioningServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        server_address: Tuple[str, int],
        handler_class,
        *,
        manager: ProvisioningManager,
        ui_dir: Path,
    ):
        super().__init__(server_address, handler_class)
        self.manager = manager
        self.ui_dir = ui_dir


class ProvisioningRequestHandler(BaseHTTPRequestHandler):
    server_version = "IntelliTrolleyWiFi/1"

    def log_message(self, format_string: str, *args: Any) -> None:
        # Never log request bodies or query strings; confirmation tokens and
        # Wi-Fi secrets must not enter the journal.
        path = urlparse(self.path).path
        print(
            f'{self.client_address[0]} {self.command} {path} '
            f'{format_string % args}',
            flush=True,
        )

    @property
    def manager(self) -> ProvisioningManager:
        return self.server.manager

    @property
    def remote_address(self) -> str:
        return str(self.client_address[0])

    def _common_headers(self) -> None:
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'",
        )
        self.send_header("Referrer-Policy", "no-referrer")

    def _send_json(self, status: int, payload: Dict[str, Any]) -> None:
        body = json.dumps(
            payload,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self.send_response(status)
        self._common_headers()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, relative_path: str, content_type: str) -> None:
        candidate = (self.server.ui_dir / relative_path).resolve()
        ui_root = self.server.ui_dir.resolve()
        if candidate.parent != ui_root or not candidate.is_file():
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "Not found."})
            return
        body = candidate.read_bytes()
        self.send_response(HTTPStatus.OK)
        self._common_headers()
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> Dict[str, Any]:
        if self.headers.get("X-IntelliTrolley-Provisioning") != "1":
            raise ProvisioningError(
                "Missing provisioning request header.",
                HTTPStatus.FORBIDDEN,
            )
        if "application/json" not in self.headers.get("Content-Type", ""):
            raise ProvisioningError(
                "Requests must use application/json.",
                HTTPStatus.UNSUPPORTED_MEDIA_TYPE,
            )
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ProvisioningError("Invalid request length.") from exc
        if length <= 0 or length > MAX_BODY_BYTES:
            raise ProvisioningError("Invalid request size.")
        try:
            payload = json.loads(self.rfile.read(length))
        except (ValueError, UnicodeDecodeError) as exc:
            raise ProvisioningError("Invalid JSON request.") from exc
        if not isinstance(payload, dict):
            raise ProvisioningError("JSON request must be an object.")
        return payload

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        try:
            if path in ("/", "/index.html"):
                self._send_file("index.html", "text/html; charset=utf-8")
            elif path == "/app.js":
                self._send_file("app.js", "text/javascript; charset=utf-8")
            elif path == "/styles.css":
                self._send_file("styles.css", "text/css; charset=utf-8")
            elif path == "/api/status":
                self._send_json(
                    HTTPStatus.OK,
                    self.manager.status(self.remote_address),
                )
            elif path == "/api/networks":
                self._send_json(
                    HTTPStatus.OK,
                    {"networks": self.manager.scan_networks(self.remote_address)},
                )
            else:
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "Not found."})
        except ProvisioningError as exc:
            self._send_json(exc.status, {"error": str(exc)})
        except Exception:
            self._send_json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {"error": "Unexpected provisioning error. Check the Pi journal."},
            )

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            payload = self._read_json()
            if path == "/api/stage":
                result = self.manager.stage(
                    remote_address=self.remote_address,
                    ssid=payload.get("ssid"),
                    password=payload.get("password"),
                    security=payload.get("security"),
                    hidden=payload.get("hidden", False),
                )
            elif path == "/api/activate":
                if payload.get("robot_stationary") is not True:
                    raise ProvisioningError(
                        "Confirm that the robot is stationary before switching networks."
                    )
                result = self.manager.activate(self.remote_address)
            elif path == "/api/use-ap":
                if payload.get("robot_stationary") is not True:
                    raise ProvisioningError(
                        "Confirm that the robot is stationary before updating "
                        "the robot network."
                    )
                result = self.manager.configure_current_ap(self.remote_address)
            elif path == "/api/confirm":
                if payload.get("central_computer") is not True:
                    raise ProvisioningError(
                        "Confirm this page is open on the Windows central computer."
                    )
                result = self.manager.confirm(
                    remote_address=self.remote_address,
                    token=payload.get("token"),
                )
            elif path == "/api/forget":
                result = self.manager.forget_staged(self.remote_address)
            else:
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "Not found."})
                return
            self._send_json(HTTPStatus.OK, result)
        except ProvisioningError as exc:
            self._send_json(exc.status, {"error": str(exc)})
        except Exception:
            self._send_json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {"error": "Unexpected provisioning error. Check the Pi journal."},
            )


def build_manager_from_environment(args: argparse.Namespace) -> ProvisioningManager:
    return ProvisioningManager(
        interface=os.getenv("ROBOT_WIFI_INTERFACE", "wlan0"),
        ap_connection=os.getenv(
            "ROBOT_WIFI_AP_CONNECTION",
            "intellitrolley-ap",
        ),
        ap_cidr=os.getenv("ROBOT_WIFI_AP_ADDRESS_CIDR", "10.42.0.1/24"),
        hostname=os.getenv(
            "ROBOT_WIFI_HOSTNAME",
            f"{socket.gethostname()}.local",
        ),
        port=args.port,
        switch_timeout_s=int(
            os.getenv("ROBOT_WIFI_SWITCH_TIMEOUT_S", "180")
        ),
        network_connections_dir=Path(
            os.getenv(
                "ROBOT_WIFI_CONNECTIONS_DIR",
                "/etc/NetworkManager/system-connections",
            )
        ),
        state_dir=Path(
            os.getenv(
                "ROBOT_WIFI_STATE_DIR",
                "/var/lib/my-bot-wifi-provisioning",
            )
        ),
        robot_defaults_path=Path(
            os.getenv(
                "ROBOT_WIFI_ROBOT_DEFAULTS",
                "/etc/default/my-bot-robot",
            )
        ),
        robot_service=os.getenv(
            "ROBOT_WIFI_ROBOT_SERVICE",
            "my-bot-robot.service",
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Serve the IntelliTrolley recovery Wi-Fi provisioning UI."
    )
    parser.add_argument(
        "--bind",
        default=os.getenv("ROBOT_WIFI_PROVISION_BIND", "0.0.0.0"),
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.getenv("ROBOT_WIFI_PROVISION_PORT", "8090")),
    )
    parser.add_argument(
        "--ui-dir",
        type=Path,
        default=Path(
            os.getenv(
                "ROBOT_WIFI_UI_DIR",
                str(Path(__file__).resolve().parents[1] / "wifi_provisioning_ui"),
            )
        ),
    )
    args = parser.parse_args()
    if not (1 <= args.port <= 65535):
        raise SystemExit("ROBOT_WIFI_PROVISION_PORT must be between 1 and 65535.")
    if not args.ui_dir.is_dir():
        raise SystemExit(f"Provisioning UI directory is missing: {args.ui_dir}")
    for command in ("nmcli", "busctl", "ip", "systemctl"):
        if shutil.which(command) is None:
            raise SystemExit(f"Required command is missing: {command}")

    manager = build_manager_from_environment(args)
    server = ProvisioningServer(
        (args.bind, args.port),
        ProvisioningRequestHandler,
        manager=manager,
        ui_dir=args.ui_dir,
    )
    print(
        f"IntelliTrolley Wi-Fi provisioning listening on "
        f"{args.bind}:{args.port}; network changes require the active "
        f"{manager.ap_connection} recovery hotspot.",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
