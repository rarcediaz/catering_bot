#!/usr/bin/env python3
"""
Recovery-safe Wi-Fi provisioning UI for the IntelliTrolley Raspberry Pi.

Mutating operations are accepted only from a client on the IPv4 subnet of the
Pi's active Wi-Fi interface. Switching to a staged facility profile runs inside
a NetworkManager checkpoint, so failure to confirm the new connection restores
the previous Wi-Fi and falls back to the recovery access point if needed. If the
server itself is interrupted during a switch, it resumes in read-only recovery
mode and explicitly restores the previous connection before accepting changes.
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
CHECKPOINT_PATTERN = re.compile(
    r"/org/freedesktop/NetworkManager/Checkpoint/[0-9]+"
)
CHECKPOINT_ROLLBACK_GRACE_S = 30
RECOVERY_MONITOR_INTERVAL_S = 2.0
INTERRUPTED_RECOVERY_RETRY_S = 30.0


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


def parse_saved_profile_rows(
    output: str,
    ap_connection: str,
) -> List[Dict[str, Any]]:
    """Return saved client Wi-Fi profiles, including pre-existing Pi profiles."""
    profiles: List[Dict[str, Any]] = []
    for line in output.splitlines():
        fields = split_nmcli_terse(line)
        if len(fields) != 5:
            continue
        connection_name, profile_uuid, connection_type, autoconnect, device = fields
        if (
            connection_name == ap_connection
            or connection_type not in {"wifi", "802-11-wireless"}
            or not re.fullmatch(
                r"[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-"
                r"[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}",
                profile_uuid,
            )
        ):
            continue
        profiles.append(
            {
                "connection": connection_name,
                "uuid": profile_uuid.lower(),
                "confirmed": autoconnect.lower() in {"yes", "true"},
                "active": bool(device and device != "--"),
                "managed": connection_name.startswith(FACILITY_CONNECTION_PREFIX),
            }
        )
    return profiles


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
        loss_grace_s: int,
        ready_state_path: Path,
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
        self.loss_grace_s = max(30, min(600, int(loss_grace_s)))
        self.ready_state_path = ready_state_path
        self.network_connections_dir = network_connections_dir
        self.state_dir = state_dir
        self.robot_defaults_path = robot_defaults_path
        self.robot_service = robot_service
        self.runner = runner or CommandRunner()
        self.lock = threading.RLock()
        self.robot_service_lock = threading.Lock()
        self.robot_refresh_generation = 0
        self.switch_timer: Optional[threading.Timer] = None
        self.rollback_timer: Optional[threading.Timer] = None
        self.pending_checkpoint = ""
        self.pending_token_hash = ""
        self.pending_connection = ""
        self.pending_ssid = ""
        self.pending_previous_connection = ""
        self.pending_deadline = 0.0
        self.pending_robot_was_active = False
        self.robot_restart_required = False
        self.transition_phase = "idle"
        self.last_result = ""
        self.staged_connection = ""
        self.staged_ssid = ""
        self.staged_security = ""
        self.loss_started_at = 0.0
        self.pending_loss_started_at = 0.0
        self.interrupted_retry_at = 0.0
        self.last_ready_signature: Optional[Tuple[str, str]] = None
        self.last_robot_refresh_signature: Optional[Tuple[str, str]] = None
        self.recovery_stop = threading.Event()
        self.recovery_thread: Optional[threading.Thread] = None
        self._load_state()
        self._load_ready_signature()

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
        self.robot_restart_required = bool(
            payload.get("robot_restart_required", False)
        )
        if payload.get("phase") == "pending":
            self.pending_connection = str(payload.get("pending_connection") or "")
            self.pending_ssid = str(payload.get("pending_ssid") or "")
            self.pending_previous_connection = str(
                payload.get("pending_previous_connection") or ""
            )
            self.pending_robot_was_active = bool(
                payload.get("pending_robot_was_active", False)
            )
            checkpoint = str(payload.get("pending_checkpoint") or "")
            if CHECKPOINT_PATTERN.fullmatch(checkpoint):
                self.pending_checkpoint = checkpoint
            self.transition_phase = "interrupted_recovery"
            self.last_result = (
                "A previous Wi-Fi switch was interrupted. Restoring the previous "
                "connection before allowing more changes."
            )

    def _load_ready_signature(self) -> None:
        try:
            payload = json.loads(self.ready_state_path.read_text(encoding="utf-8"))
            connection = str(payload.get("connection") or "")
            address = str(payload.get("address") or "")
        except (FileNotFoundError, ValueError, OSError):
            return
        if connection and address:
            self.last_ready_signature = (connection, address)
            self.last_robot_refresh_signature = (connection, address)

    def _save_state(self, phase: str) -> None:
        self.state_dir.mkdir(parents=True, exist_ok=True)
        payload = {
            "phase": phase,
            "staged_connection": self.staged_connection,
            "staged_ssid": self.staged_ssid,
            "staged_security": self.staged_security,
            "pending_connection": self.pending_connection,
            "pending_ssid": self.pending_ssid,
            "pending_previous_connection": self.pending_previous_connection,
            "pending_checkpoint": self.pending_checkpoint,
            "pending_deadline": self.pending_deadline,
            "pending_robot_was_active": self.pending_robot_was_active,
            "robot_restart_required": self.robot_restart_required,
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

    def _current_network_signature(self) -> Optional[Tuple[str, str]]:
        try:
            connection = self.active_connection()
            address = self.interface_ipv4()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            return None
        if not connection or connection == "--" or address is None:
            return None
        return (connection, str(address))

    def is_ap_client(self, remote_address: str) -> bool:
        try:
            remote = ipaddress.ip_address(remote_address)
        except ValueError:
            return False
        if remote.version != 4 or remote not in self.ap_network.network:
            return False
        try:
            interface_address = self.interface_ipv4()
            return bool(
                self.active_connection() == self.ap_connection
                and interface_address is not None
                and interface_address.ip == self.ap_network.ip
                and interface_address.network == self.ap_network.network
            )
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            return False

    def is_active_wifi_client(self, remote_address: str) -> bool:
        """Return whether the request came from the active wlan IPv4 subnet."""
        try:
            remote = ipaddress.ip_address(remote_address)
        except ValueError:
            return False
        if remote.version != 4:
            return False
        try:
            active_connection = self.active_connection()
            interface_address = self.interface_ipv4()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            return False
        return bool(
            active_connection
            and active_connection not in {"--", "Unavailable"}
            and interface_address is not None
            and remote in interface_address.network
        )

    def _begin_idle_operation(self, phase: str) -> None:
        with self.lock:
            if self.transition_phase != "idle":
                raise ProvisioningError(
                    "Wait for the current Wi-Fi operation to finish.",
                    HTTPStatus.CONFLICT,
                )
            self.transition_phase = phase

    def _finish_operation(self, phase: str) -> None:
        with self.lock:
            if self.transition_phase == phase:
                self.transition_phase = "idle"

    def saved_profiles(self) -> List[Dict[str, Any]]:
        result = self.runner.run(
            [
                "nmcli",
                "--terse",
                "--escape",
                "yes",
                "--fields",
                "NAME,UUID,TYPE,AUTOCONNECT,DEVICE",
                "connection",
                "show",
            ],
            timeout=10,
        )
        profiles = parse_saved_profile_rows(result.stdout, self.ap_connection)
        for profile in profiles:
            ssid_result = self.runner.run(
                [
                    "nmcli",
                    "--get-values",
                    "802-11-wireless.ssid",
                    "connection",
                    "show",
                    "uuid",
                    profile["uuid"],
                ],
                timeout=10,
                check=False,
            )
            profile["ssid"] = (
                ssid_result.stdout.strip()
                if ssid_result.returncode == 0 and ssid_result.stdout.strip()
                else profile["connection"]
            )
            profile["staged"] = profile["connection"] == self.staged_connection
        return sorted(
            profiles,
            key=lambda item: (
                not bool(item["active"]),
                not bool(item["confirmed"]),
                str(item["ssid"]).lower(),
            ),
        )

    def status(self, remote_address: str) -> Dict[str, Any]:
        try:
            active_connection = self.active_connection()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            active_connection = "Unavailable"
        try:
            interface_address = self.interface_ipv4()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            interface_address = None
        try:
            saved_profiles = self.saved_profiles()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            saved_profiles = []
        try:
            remote = ipaddress.ip_address(remote_address)
        except ValueError:
            remote = None
        source_on_active_wifi = bool(
            remote is not None
            and remote.version == 4
            and active_connection not in {"", "--", "Unavailable"}
            and interface_address is not None
            and remote in interface_address.network
        )
        source_on_recovery_ap = bool(
            source_on_active_wifi
            and active_connection == self.ap_connection
            and interface_address is not None
            and interface_address.ip == self.ap_network.ip
            and interface_address.network == self.ap_network.network
            and remote in self.ap_network.network
        )
        with self.lock:
            transition_phase = self.transition_phase
            can_provision = bool(
                transition_phase == "idle"
                and source_on_active_wifi
            )
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
                "can_provision": can_provision,
                "using_recovery_ap": active_connection == self.ap_connection,
                "can_configure_ap": bool(
                    transition_phase == "idle"
                    and source_on_recovery_ap
                ),
                "transition_phase": transition_phase,
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
                "saved_profiles": saved_profiles,
                "loss_grace_s": self.loss_grace_s,
                "hostname": self.hostname,
                "port": self.port,
            }

    def scan_networks(self, remote_address: str) -> List[Dict[str, Any]]:
        if not self.is_active_wifi_client(remote_address):
            raise ProvisioningError(
                "Network scanning is available only from the Pi's active Wi-Fi network.",
                HTTPStatus.FORBIDDEN,
            )
        self._begin_idle_operation("scanning")
        try:
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
        finally:
            self._finish_operation("scanning")

    def stage(
        self,
        *,
        remote_address: str,
        ssid: Any,
        password: Any,
        security: Any,
        hidden: Any,
    ) -> Dict[str, Any]:
        if not self.is_active_wifi_client(remote_address):
            raise ProvisioningError(
                "Wi-Fi profiles can be changed only from the Pi's active Wi-Fi network.",
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
        self._begin_idle_operation("updating_profile")
        try:
            if self.active_connection() == connection_name:
                raise ProvisioningError(
                    "That managed Wi-Fi profile is currently active and cannot be overwritten.",
                    HTTPStatus.CONFLICT,
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
                    f"Saved {clean_ssid}. The Pi stayed on its current Wi-Fi."
                )
                self._save_state("staged")
            return {
                "staged": True,
                "ssid": clean_ssid,
                "message": self.last_result,
            }
        finally:
            clean_psk = ""
            self._finish_operation("updating_profile")

    def activate(self, remote_address: str) -> Dict[str, Any]:
        if not self.is_active_wifi_client(remote_address):
            raise ProvisioningError(
                "The Wi-Fi switch must begin from the Pi's active Wi-Fi network.",
                HTTPStatus.FORBIDDEN,
            )
        previous_connection = self.active_connection()
        with self.lock:
            if self.transition_phase != "idle":
                raise ProvisioningError(
                    "A Wi-Fi switch is already pending.",
                    HTTPStatus.CONFLICT,
                )
            if not self.staged_connection:
                raise ProvisioningError("Save a facility Wi-Fi profile first.")
            if self.staged_connection == previous_connection:
                raise ProvisioningError(
                    "That Wi-Fi connection is already active.",
                    HTTPStatus.CONFLICT,
                )
            token = secrets.token_urlsafe(24)
            self.pending_token_hash = hashlib.sha256(
                token.encode("utf-8")
            ).hexdigest()
            self.pending_connection = self.staged_connection
            self.pending_ssid = self.staged_ssid
            self.pending_previous_connection = previous_connection
            self.transition_phase = "scheduled"
            self.robot_refresh_generation += 1
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
                "Connect this computer to the facility Wi-Fi, "
                "then open the confirmation link before the deadline."
            ),
        }

    def _start_checkpoint_switch(self) -> None:
        with self.lock:
            self.switch_timer = None
            connection_name = self.pending_connection
            previous_connection = self.pending_previous_connection
            if not connection_name or self.transition_phase != "scheduled":
                if self.transition_phase == "scheduled":
                    self.transition_phase = "idle"
                return
            self.transition_phase = "activating"
        try:
            checkpoint = self._create_checkpoint()
            with self.lock:
                if self.pending_connection != connection_name:
                    self.pending_checkpoint = checkpoint
                    if not self._disarm_checkpoint(checkpoint):
                        self._defer_checkpoint_recovery(
                            "A cancelled Wi-Fi switch left an uncertain "
                            "NetworkManager checkpoint. Changes remain locked "
                            "while cleanup retries."
                        )
                        return
                    self.pending_checkpoint = ""
                    self.pending_connection = ""
                    self.pending_ssid = ""
                    self.pending_previous_connection = ""
                    self.pending_deadline = 0.0
                    self.pending_token_hash = ""
                    self.transition_phase = "idle"
                    self._save_state("cancelled")
                    return
                self.pending_checkpoint = checkpoint
                # An intentional network switch must always leave the robot
                # service running, even if systemctl reports a stop timeout
                # after the unit has already stopped.
                self.pending_robot_was_active = True
                self.robot_restart_required = True
                self._save_state("pending")
            self._stop_robot_service_for_switch()
            activation_deadline = time.monotonic() + self.loss_grace_s
            activation_result = self.runner.run(
                [
                    "nmcli",
                    "--wait",
                    str(self.loss_grace_s),
                    "connection",
                    "up",
                    "id",
                    connection_name,
                    "ifname",
                    self.interface,
                ],
                timeout=self.loss_grace_s + 10,
                check=False,
            )
            while True:
                if (
                    self.active_connection() == connection_name
                    and self.interface_ipv4() is not None
                ):
                    break
                remaining_s = activation_deadline - time.monotonic()
                if remaining_s <= 0.0:
                    detail = (
                        activation_result.stderr
                        or activation_result.stdout
                        or "the connection did not become ready"
                    ).strip()
                    raise ProvisioningError(
                        f"Facility Wi-Fi did not recover within "
                        f"{self.loss_grace_s} seconds: {detail}",
                        HTTPStatus.BAD_GATEWAY,
                    )
                time.sleep(min(RECOVERY_MONITOR_INTERVAL_S, remaining_s))
        except (OSError, ProvisioningError, subprocess.TimeoutExpired) as exc:
            with self.lock:
                checkpoint = self.pending_checkpoint
                self.transition_phase = "rolling_back"
                self.robot_refresh_generation += 1
                recovery_generation = self.robot_refresh_generation
            if checkpoint and not self._disarm_checkpoint(checkpoint):
                self._defer_checkpoint_recovery(
                    "The failed Wi-Fi switch left an uncertain NetworkManager "
                    "checkpoint. Changes remain locked while cleanup retries."
                )
                return
            with self.lock:
                restart_robot = self.pending_robot_was_active
                self.pending_checkpoint = ""
            restored_connection = self._restore_previous_connection(
                previous_connection
            )
            robot_refresh_succeeded = False
            if restored_connection and restart_robot:
                robot_refresh_succeeded = self._refresh_robot_service(
                    ensure_running=True,
                    delay_s=0.0,
                    expected_generation=recovery_generation,
                )
            with self.lock:
                self.pending_connection = ""
                self.pending_ssid = ""
                self.pending_previous_connection = ""
                self.pending_deadline = 0.0
                self.pending_token_hash = ""
                self.pending_robot_was_active = False
                if robot_refresh_succeeded:
                    self.robot_restart_required = False
                    self.last_robot_refresh_signature = (
                        self._current_network_signature()
                    )
                self.last_result = f"Could not activate facility Wi-Fi: {exc}"
                if restored_connection:
                    self.last_result += f" Restored {restored_connection}."
                else:
                    self.last_result += " No recovery Wi-Fi could be verified."
                if restart_robot and not robot_refresh_succeeded:
                    self.last_result += " The robot service start will retry."
                try:
                    self._save_state("failed")
                finally:
                    self.transition_phase = "idle"
            return
        self._schedule_robot_service_refresh(
            ensure_running=True,
            delay_s=1.0,
        )
        with self.lock:
            self.transition_phase = "pending_confirmation"
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
            rf'\s*o\s+"({CHECKPOINT_PATTERN.pattern})"\s*',
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
        if not CHECKPOINT_PATTERN.fullmatch(checkpoint):
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

    def _discard_checkpoint(self, checkpoint: str) -> bool:
        """Destroy a checkpoint so it cannot roll Wi-Fi back later."""
        try:
            result = self.runner.run(
                self.checkpoint_action_command(
                    "CheckpointDestroy",
                    checkpoint,
                ),
                timeout=15,
                check=False,
            )
            return result.returncode == 0
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            return False

    def _checkpoint_exists(self, checkpoint: str) -> Optional[bool]:
        """Query NetworkManager when rollback and destroy did not confirm."""
        try:
            result = self.runner.run(
                [
                    "busctl",
                    "get-property",
                    "org.freedesktop.NetworkManager",
                    "/org/freedesktop/NetworkManager",
                    "org.freedesktop.NetworkManager",
                    "Checkpoints",
                ],
                timeout=10,
                check=False,
            )
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            return None
        if result.returncode != 0:
            return None
        return checkpoint in CHECKPOINT_PATTERN.findall(result.stdout)

    def _disarm_checkpoint(self, checkpoint: str) -> bool:
        """Confirm rollback, destruction, or that the checkpoint is already gone."""
        rolled_back = self._rollback_checkpoint(checkpoint)
        destroyed = self._discard_checkpoint(checkpoint)
        if rolled_back or destroyed:
            return True
        exists = self._checkpoint_exists(checkpoint)
        return exists is False

    def _defer_checkpoint_recovery(self, message: str) -> None:
        """Keep the portal read-only until an uncertain checkpoint is disarmed."""
        with self.lock:
            self.transition_phase = "interrupted_recovery"
            self.interrupted_retry_at = (
                time.monotonic() + INTERRUPTED_RECOVERY_RETRY_S
            )
            self.last_result = message
            self._save_state("pending")

    def _ensure_recovery_ap(self) -> None:
        try:
            interface_address = self.interface_ipv4()
            if (
                self.active_connection() == self.ap_connection
                and interface_address is not None
                and interface_address.ip == self.ap_network.ip
                and interface_address.network == self.ap_network.network
            ):
                return
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
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

    def _restore_previous_connection(self, previous_connection: str) -> str:
        """Restore the checkpoint's origin, with the AP as the final fallback."""
        if previous_connection:
            try:
                active_connection = self.active_connection()
                interface_address = self.interface_ipv4()
                if (
                    active_connection == previous_connection
                    and interface_address is not None
                ):
                    return self._record_restored_connection(
                        active_connection,
                        interface_address,
                    )
                self.runner.run(
                    [
                        "nmcli",
                        "--wait",
                        "30",
                        "connection",
                        "up",
                        "id",
                        previous_connection,
                        "ifname",
                        self.interface,
                    ],
                    timeout=35,
                    check=False,
                )
                active_connection = self.active_connection()
                interface_address = self.interface_ipv4()
                if (
                    active_connection == previous_connection
                    and interface_address is not None
                ):
                    return self._record_restored_connection(
                        active_connection,
                        interface_address,
                    )
            except (OSError, ProvisioningError, subprocess.TimeoutExpired):
                pass
        self._ensure_recovery_ap()
        try:
            active_connection = self.active_connection()
            interface_address = self.interface_ipv4()
            if (
                active_connection == self.ap_connection
                and interface_address is not None
                and interface_address.ip == self.ap_network.ip
                and interface_address.network == self.ap_network.network
            ):
                return self._record_restored_connection(
                    active_connection,
                    interface_address,
                )
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            pass
        return ""

    def _record_restored_connection(
        self,
        connection: str,
        address: ipaddress.IPv4Interface,
    ) -> str:
        try:
            self._write_ready_state(
                "ap" if connection == self.ap_connection else "client",
                connection,
                address,
            )
        except OSError:
            return connection
        else:
            self.last_ready_signature = (connection, str(address))
        return connection

    def _set_ap_autoconnect(self, enabled: bool) -> None:
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

    def _write_ready_state(
        self,
        mode: str,
        connection: str,
        address: ipaddress.IPv4Interface,
    ) -> None:
        self.ready_state_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "ready": True,
            "mode": mode,
            "interface": self.interface,
            "connection": connection,
            "address": str(address),
            "timestamp": time.time(),
        }
        temporary_fd, temporary_name = tempfile.mkstemp(
            prefix=f".{self.ready_state_path.name}.",
            dir=str(self.ready_state_path.parent),
            text=True,
        )
        try:
            with os.fdopen(temporary_fd, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, sort_keys=True)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.chmod(temporary_name, 0o644)
            os.replace(temporary_name, self.ready_state_path)
        finally:
            if os.path.exists(temporary_name):
                os.unlink(temporary_name)

    def check_runtime_recovery(self, *, now: Optional[float] = None) -> None:
        """Apply the loss grace and recover to the AP after sustained failure."""
        observed_at = time.monotonic() if now is None else float(now)
        try:
            connection = self.active_connection()
            address = self.interface_ipv4()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            connection = ""
            address = None
        ap_address_ready = bool(
            connection != self.ap_connection
            or (
                address is not None
                and address.ip == self.ap_network.ip
                and address.network == self.ap_network.network
            )
        )
        ready = bool(
            connection
            and connection != "--"
            and address is not None
            and ap_address_ready
        )
        ready_signature = (
            (connection, str(address)) if ready and address is not None else None
        )

        with self.lock:
            checkpoint = self.pending_checkpoint
            pending_connection = self.pending_connection
            transition_phase = self.transition_phase
            if transition_phase not in {"idle", "pending_confirmation"}:
                return
            pending_switch = bool(
                transition_phase == "pending_confirmation"
                and pending_connection
                and checkpoint
            )

            if pending_switch:
                # The activation worker owns the initial attempt and its
                # 90-second wait. Monitor sustained loss only after that worker
                # has armed the confirmation rollback timer.
                if self.rollback_timer is None:
                    return
                if ready and connection == pending_connection:
                    self.pending_loss_started_at = 0.0
                    return
                if self.pending_loss_started_at <= 0.0:
                    self.pending_loss_started_at = observed_at
                    return
                pending_lost_for = observed_at - self.pending_loss_started_at
                should_rollback = bool(
                    checkpoint and pending_lost_for >= self.loss_grace_s
                )
            else:
                self.pending_loss_started_at = 0.0
                should_rollback = False

            ready_refresh = False
            ready_generation = self.robot_refresh_generation
            previous_signature = self.last_ready_signature
            previous_robot_signature = self.last_robot_refresh_signature
            restart_required = self.robot_restart_required
            if (
                transition_phase == "idle"
                and ready_signature is not None
                and (
                    ready_signature != previous_signature
                    or restart_required
                )
            ):
                self.transition_phase = "ready_refresh"
                self.robot_refresh_generation += 1
                ready_generation = self.robot_refresh_generation
                ready_refresh = True

        if should_rollback:
            self._checkpoint_expired(checkpoint)
            return
        if pending_switch:
            return

        if ready and address is not None:
            if not ready_refresh:
                with self.lock:
                    self.loss_started_at = 0.0
                return
            mode = "ap" if connection == self.ap_connection else "client"
            signature = (connection, str(address))
            observation_error = ""
            try:
                if mode == "client" and signature != previous_signature:
                    self._set_ap_autoconnect(False)
                if signature != previous_signature:
                    self._write_ready_state(mode, connection, address)
                    self.last_ready_signature = signature
            except (OSError, ProvisioningError, subprocess.TimeoutExpired) as exc:
                observation_error = str(exc)

            refresh_needed = bool(
                restart_required
                or (
                    previous_robot_signature is not None
                    and signature != previous_robot_signature
                )
            )
            refresh_succeeded = not refresh_needed
            if refresh_needed:
                with self.lock:
                    self.robot_restart_required = True
                refresh_succeeded = self._refresh_robot_service(
                    ensure_running=True,
                    delay_s=0.0,
                    expected_generation=ready_generation,
                )
            with self.lock:
                self.loss_started_at = 0.0
                if refresh_succeeded:
                    self.robot_restart_required = False
                    if refresh_needed:
                        self.last_robot_refresh_signature = signature
                if observation_error:
                    self.last_result = (
                        "Wi-Fi is ready, but its ready-state update will retry: "
                        f"{observation_error}"
                    )
                elif refresh_needed and not refresh_succeeded:
                    self.last_result = (
                        "Wi-Fi is ready, but the robot service restart will retry."
                    )
                try:
                    self._save_state("idle")
                finally:
                    self.transition_phase = "idle"
            return

        with self.lock:
            if self.transition_phase != "idle":
                return
            if self.loss_started_at <= 0.0:
                self.loss_started_at = observed_at
                self.last_result = (
                    f"Wi-Fi lost; waiting {self.loss_grace_s} seconds for a saved "
                    "connection before starting the recovery hotspot."
                )
                self._save_state("connection_grace")
                return
            lost_for = observed_at - self.loss_started_at
            if lost_for < self.loss_grace_s:
                return
            self.loss_started_at = 0.0
            self.transition_phase = "runtime_recovery"
            self.robot_refresh_generation += 1
            recovery_generation = self.robot_refresh_generation
            self.last_result = (
                "Saved Wi-Fi did not recover within the grace period; "
                "starting the IntelliTrolley hotspot."
            )

        # Recheck after claiming recovery ownership. A saved connection can
        # become ready at the exact grace-period boundary.
        try:
            recovered_connection = self.active_connection()
            recovered_address = self.interface_ipv4()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            recovered_connection = ""
            recovered_address = None
        if (
            recovered_connection
            and recovered_connection != "--"
            and recovered_address is not None
        ):
            mode = (
                "ap" if recovered_connection == self.ap_connection else "client"
            )
            valid_recovery = bool(
                mode == "client"
                or (
                    recovered_address.ip == self.ap_network.ip
                    and recovered_address.network == self.ap_network.network
                )
            )
            if valid_recovery:
                signature = (recovered_connection, str(recovered_address))
                previous_signature = self.last_ready_signature
                try:
                    if mode == "client" and signature != previous_signature:
                        self._set_ap_autoconnect(False)
                    if signature != previous_signature:
                        self._write_ready_state(
                            mode,
                            recovered_connection,
                            recovered_address,
                        )
                        self.last_ready_signature = signature
                except (OSError, ProvisioningError, subprocess.TimeoutExpired):
                    pass
                with self.lock:
                    self.loss_started_at = 0.0
                    self.last_result = (
                        f"{recovered_connection} recovered before the hotspot "
                        "switch began."
                    )
                    try:
                        self._save_state("idle")
                    finally:
                        self.transition_phase = "idle"
                return

        with self.lock:
            self.robot_restart_required = True
            try:
                self._save_state("runtime_recovery")
            except OSError:
                # Do not stop ROS unless the recovery obligation is durable.
                self.robot_restart_required = False
                self.loss_started_at = observed_at
                self.transition_phase = "idle"
                raise

        try:
            self._stop_robot_service_for_switch()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            pass
        try:
            self._set_ap_autoconnect(True)
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            pass
        self._ensure_recovery_ap()
        try:
            recovered_connection = self.active_connection()
            recovered_address = self.interface_ipv4()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            recovered_connection = ""
            recovered_address = None
        recovery_verified = bool(
            recovered_connection == self.ap_connection
            and recovered_address is not None
            and recovered_address.ip == self.ap_network.ip
            and recovered_address.network == self.ap_network.network
        )
        ready_state_written = False
        robot_refresh_succeeded = False
        if recovery_verified and recovered_address is not None:
            try:
                self._write_ready_state("ap", recovered_connection, recovered_address)
            except OSError:
                pass
            else:
                ready_state_written = True
                self.last_ready_signature = (
                    recovered_connection,
                    str(recovered_address),
                )
            robot_refresh_succeeded = self._refresh_robot_service(
                ensure_running=True,
                delay_s=0.0,
                expected_generation=recovery_generation,
            )

        with self.lock:
            try:
                if recovery_verified:
                    self.last_result = (
                        f"No saved Wi-Fi recovered within {self.loss_grace_s} "
                        "seconds; started the IntelliTrolley hotspot."
                    )
                    if not ready_state_written:
                        self.last_result += (
                            " The network ready-state file needs a retry."
                        )
                    if robot_refresh_succeeded:
                        self.robot_restart_required = False
                        self.last_robot_refresh_signature = (
                            recovered_connection,
                            str(recovered_address),
                        )
                    else:
                        self.last_result += " The robot service start will retry."
                    self.loss_started_at = 0.0
                    self._save_state("runtime_recovery")
                else:
                    self.loss_started_at = observed_at
                    self.last_result = (
                        "The recovery hotspot could not be verified. Wi-Fi changes "
                        "remain unavailable until a connection returns; recovery "
                        "will retry after the grace period."
                    )
                    self._save_state("recovery_failed")
            finally:
                self.transition_phase = "idle"

    def start_recovery_monitor(self) -> None:
        if self.recovery_thread is not None and self.recovery_thread.is_alive():
            return
        self.recovery_stop.clear()
        self.recovery_thread = threading.Thread(
            target=self._recovery_monitor_loop,
            name="wifi-recovery-monitor",
            daemon=True,
        )
        self.recovery_thread.start()

    def stop_recovery_monitor(self) -> None:
        self.recovery_stop.set()
        if self.recovery_thread is not None:
            self.recovery_thread.join(timeout=RECOVERY_MONITOR_INTERVAL_S + 1.0)
        self.recovery_thread = None

    def _recovery_monitor_loop(self) -> None:
        while not self.recovery_stop.wait(RECOVERY_MONITOR_INTERVAL_S):
            try:
                with self.lock:
                    interrupted = self.transition_phase == "interrupted_recovery"
                if interrupted:
                    self._recover_interrupted_switch()
                else:
                    self.check_runtime_recovery()
            except Exception as exc:
                print(f"Wi-Fi recovery monitor check failed: {exc}", flush=True)

    def _recover_interrupted_switch(self, *, now: Optional[float] = None) -> None:
        """Conservatively restore a switch whose in-memory checkpoint was lost."""
        observed_at = time.monotonic() if now is None else float(now)
        with self.lock:
            if self.transition_phase != "interrupted_recovery":
                return
            if observed_at < self.interrupted_retry_at:
                return
            previous_connection = self.pending_previous_connection
            checkpoint = self.pending_checkpoint
            self.transition_phase = "rolling_back"
            self.robot_refresh_generation += 1
            recovery_generation = self.robot_refresh_generation
            self.robot_restart_required = True
            try:
                self._save_state("pending")
            except OSError:
                self.robot_restart_required = False
                self.transition_phase = "interrupted_recovery"
                raise

        # Roll back the persisted NetworkManager checkpoint when available,
        # then explicitly verify or reactivate its saved origin.
        try:
            self._stop_robot_service_for_switch()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            pass
        if checkpoint and not self._disarm_checkpoint(checkpoint):
            self._defer_checkpoint_recovery(
                "The interrupted Wi-Fi switch could not disarm its NetworkManager "
                "checkpoint. Changes remain locked while cleanup retries."
            )
            return
        restored_connection = self._restore_previous_connection(previous_connection)

        robot_refresh_succeeded = False
        if restored_connection:
            robot_refresh_succeeded = self._refresh_robot_service(
                ensure_running=True,
                delay_s=0.0,
                expected_generation=recovery_generation,
            )

        with self.lock:
            if not restored_connection:
                self.transition_phase = "interrupted_recovery"
                self.interrupted_retry_at = (
                    time.monotonic() + INTERRUPTED_RECOVERY_RETRY_S
                )
                self.last_result = (
                    "The interrupted Wi-Fi switch could not restore a verified "
                    "connection. Changes remain locked while recovery retries."
                )
                self._save_state("pending")
                return

            self.pending_checkpoint = ""
            self.pending_connection = ""
            self.pending_ssid = ""
            self.pending_previous_connection = ""
            self.pending_deadline = 0.0
            self.pending_token_hash = ""
            self.pending_robot_was_active = False
            if robot_refresh_succeeded:
                self.robot_restart_required = False
                self.last_robot_refresh_signature = self._current_network_signature()
            self.interrupted_retry_at = 0.0
            if restored_connection == previous_connection and previous_connection:
                self.last_result = (
                    "The interrupted Wi-Fi switch was cancelled; restored "
                    f"{restored_connection}."
                )
            else:
                self.last_result = (
                    "The interrupted Wi-Fi switch could not restore its previous "
                    f"connection; started {restored_connection}."
                )
            if not robot_refresh_succeeded:
                self.last_result += " The robot service start will retry."
            try:
                self._save_state("rolled_back")
            finally:
                self.transition_phase = "idle"

    def _checkpoint_expired(self, checkpoint: str) -> None:
        with self.lock:
            if (
                self.pending_checkpoint != checkpoint
                or self.transition_phase != "pending_confirmation"
            ):
                return
            self.rollback_timer = None
            self.transition_phase = "rolling_back"
            previous_connection = self.pending_previous_connection
            self.robot_restart_required = True
            self.robot_refresh_generation += 1
            recovery_generation = self.robot_refresh_generation
        # ROS may already be running on the unconfirmed facility network.
        # Stop it before NetworkManager restores the previous Wi-Fi address.
        try:
            self._stop_robot_service_for_switch()
        except (OSError, ProvisioningError, subprocess.TimeoutExpired):
            pass
        if not self._disarm_checkpoint(checkpoint):
            self._defer_checkpoint_recovery(
                "The expired Wi-Fi switch could not disarm its NetworkManager "
                "checkpoint. Changes remain locked while cleanup retries."
            )
            return
        restored_connection = self._restore_previous_connection(previous_connection)
        robot_refresh_succeeded = False
        if restored_connection:
            robot_refresh_succeeded = self._refresh_robot_service(
                ensure_running=True,
                delay_s=0.0,
                expected_generation=recovery_generation,
            )
        with self.lock:
            self.pending_checkpoint = ""
            self.pending_connection = ""
            self.pending_ssid = ""
            self.pending_previous_connection = ""
            self.pending_deadline = 0.0
            self.pending_token_hash = ""
            self.pending_robot_was_active = False
            if robot_refresh_succeeded:
                self.robot_restart_required = False
                self.last_robot_refresh_signature = self._current_network_signature()
            if restored_connection == previous_connection and restored_connection:
                self.last_result = (
                    "Facility Wi-Fi was not confirmed; restored "
                    f"{restored_connection}."
                )
            elif restored_connection:
                self.last_result = (
                    "Facility Wi-Fi was not confirmed and the previous Wi-Fi "
                    f"was unavailable; started {restored_connection}."
                )
            else:
                self.last_result = (
                    "Facility Wi-Fi was not confirmed and no recovery Wi-Fi "
                    "could be verified."
                )
            if restored_connection and not robot_refresh_succeeded:
                self.last_result += " The robot service start will retry."
            try:
                self._save_state("rolled_back")
            finally:
                self.transition_phase = "idle"

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
                "Confirm from this computer on the facility network.",
                HTTPStatus.FORBIDDEN,
            )

        supplied_hash = hashlib.sha256(
            str(token or "").encode("utf-8")
        ).hexdigest()
        with self.lock:
            if self.transition_phase != "pending_confirmation":
                raise ProvisioningError(
                    "No facility Wi-Fi confirmation is currently pending.",
                    HTTPStatus.CONFLICT,
                )
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
                    "The confirmation period expired; reconnect to the previous Wi-Fi.",
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
            if central_peer not in interface_address.network:
                raise ProvisioningError(
                    "Confirm from this computer on the Pi's active facility Wi-Fi network.",
                    HTTPStatus.FORBIDDEN,
                )

            # Keep the timeout callback and recovery monitor serialized through
            # the complete commit. A timeout already waiting on this lock sees
            # the cleared checkpoint and exits after a successful commit.
            self.transition_phase = "confirming"
            try:
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
                self.runner.run(
                    [
                        "nmcli",
                        "connection",
                        "modify",
                        self.ap_connection,
                        "connection.autoconnect",
                        "no",
                        "connection.autoconnect-priority",
                        "50",
                    ],
                    timeout=10,
                )
                self.runner.run(
                    self.checkpoint_action_command(
                        "CheckpointDestroy",
                        checkpoint,
                    ),
                    timeout=15,
                )
            except Exception:
                self.transition_phase = "pending_confirmation"
                raise

            self.pending_checkpoint = ""
            if self.rollback_timer is not None:
                self.rollback_timer.cancel()
                self.rollback_timer = None
            self.pending_connection = ""
            self.pending_ssid = ""
            self.pending_previous_connection = ""
            self.pending_deadline = 0.0
            self.pending_token_hash = ""
            restart_robot = True
            self.pending_robot_was_active = False
            self.staged_connection = ""
            self.staged_ssid = ""
            self.staged_security = ""
            self.transition_phase = "idle"
            handoff = self._apply_network_handoff(
                interface_address=interface_address,
                central_peer=central_peer,
                ensure_robot_running=restart_robot,
            )
            self.last_result = (
                f"Confirmed {committed_ssid}; the Pi recovery hotspot remains "
                f"saved and will start after a {self.loss_grace_s}-second sustained loss."
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
        self._begin_idle_operation("configuring_ap")
        try:
            return self._configure_current_ap(remote_address)
        finally:
            self._finish_operation("configuring_ap")

    def _configure_current_ap(self, remote_address: str) -> Dict[str, Any]:
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
        with self.robot_service_lock:
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
        with self.lock:
            self.robot_refresh_generation += 1
            expected_generation = self.robot_refresh_generation
        refresh_thread = threading.Thread(
            target=self._run_scheduled_robot_refresh,
            kwargs={
                "ensure_running": ensure_running,
                "delay_s": delay_s,
                "expected_generation": expected_generation,
            },
            daemon=True,
        )
        refresh_thread.start()

    def _run_scheduled_robot_refresh(
        self,
        *,
        ensure_running: bool,
        delay_s: float,
        expected_generation: int,
    ) -> None:
        succeeded = self._refresh_robot_service(
            ensure_running=ensure_running,
            delay_s=delay_s,
            expected_generation=expected_generation,
        )
        if succeeded:
            signature = self._current_network_signature()
            with self.lock:
                if expected_generation == self.robot_refresh_generation:
                    self.robot_restart_required = False
                    if signature is not None:
                        self.last_robot_refresh_signature = signature

    def _refresh_robot_service(
        self,
        *,
        ensure_running: bool,
        delay_s: float,
        expected_generation: Optional[int] = None,
    ) -> bool:
        time.sleep(delay_s)
        with self.robot_service_lock:
            with self.lock:
                if (
                    expected_generation is not None
                    and expected_generation != self.robot_refresh_generation
                ):
                    return False
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
                    return True
                elif ensure_running:
                    self.runner.run(
                        ["systemctl", "start", self.robot_service],
                        timeout=45,
                    )
                    return True
                return True
            except (OSError, ProvisioningError, subprocess.TimeoutExpired):
                return False

    def select_saved_profile(
        self,
        *,
        remote_address: str,
        profile_uuid: Any,
    ) -> Dict[str, Any]:
        if not self.is_active_wifi_client(remote_address):
            raise ProvisioningError(
                "Saved profiles can be selected only from the Pi's active Wi-Fi network.",
                HTTPStatus.FORBIDDEN,
            )
        try:
            clean_uuid = str(uuid.UUID(str(profile_uuid or "")))
        except (ValueError, AttributeError) as exc:
            raise ProvisioningError("Invalid saved Wi-Fi profile.") from exc
        self._begin_idle_operation("updating_profile")
        try:
            return self._select_saved_profile(clean_uuid)
        finally:
            self._finish_operation("updating_profile")

    def _select_saved_profile(self, clean_uuid: str) -> Dict[str, Any]:
        profiles = {profile["uuid"]: profile for profile in self.saved_profiles()}
        profile = profiles.get(clean_uuid)
        if profile is None:
            raise ProvisioningError("That saved Wi-Fi profile no longer exists.")
        if profile["active"] or self.active_connection() == profile["connection"]:
            raise ProvisioningError(
                "That Wi-Fi connection is already active.",
                HTTPStatus.CONFLICT,
            )
        with self.lock:
            self.staged_connection = str(profile["connection"])
            self.staged_ssid = str(profile["ssid"])
            self.staged_security = "saved"
            self.last_result = (
                f"Selected {self.staged_ssid}. The Pi stayed on its current Wi-Fi."
            )
            self._save_state("staged")
            return {
                "selected": True,
                "ssid": self.staged_ssid,
                "message": self.last_result,
            }

    def forget_profile(
        self,
        *,
        remote_address: str,
        profile_uuid: Any,
    ) -> Dict[str, Any]:
        if not self.is_active_wifi_client(remote_address):
            raise ProvisioningError(
                "Facility profiles can be removed only from the Pi's active Wi-Fi network.",
                HTTPStatus.FORBIDDEN,
            )
        try:
            clean_uuid = str(uuid.UUID(str(profile_uuid or "")))
        except (ValueError, AttributeError) as exc:
            raise ProvisioningError("Invalid saved Wi-Fi profile.") from exc
        self._begin_idle_operation("updating_profile")
        try:
            return self._forget_profile(clean_uuid)
        finally:
            self._finish_operation("updating_profile")

    def _forget_profile(self, clean_uuid: str) -> Dict[str, Any]:
        profiles = {profile["uuid"]: profile for profile in self.saved_profiles()}
        profile = profiles.get(clean_uuid)
        if profile is None:
            raise ProvisioningError("That saved Wi-Fi profile no longer exists.")
        clean_name = str(profile["connection"])
        if profile["active"] or self.active_connection() == clean_name:
            raise ProvisioningError(
                "The active Wi-Fi cannot be removed. Switch to another Wi-Fi first.",
                HTTPStatus.CONFLICT,
            )
        self.runner.run(
            ["nmcli", "connection", "delete", "uuid", clean_uuid],
            timeout=15,
        )
        if profile["managed"]:
            keyfile = (
                self.network_connections_dir / f"{clean_name}.nmconnection"
            )
            try:
                keyfile.unlink()
            except FileNotFoundError:
                pass
        self.runner.run(
            ["nmcli", "connection", "reload"],
            timeout=10,
        )
        verify = self.runner.run(
            ["nmcli", "connection", "show", "uuid", clean_uuid],
            timeout=10,
            check=False,
        )
        if verify.returncode == 0:
            raise ProvisioningError(
                "NetworkManager still reports that profile; it was not removed.",
                HTTPStatus.BAD_GATEWAY,
            )
        with self.lock:
            if self.staged_connection == clean_name:
                self.staged_connection = ""
                self.staged_ssid = ""
                self.staged_security = ""
            self.last_result = (
                f"Removed {profile['ssid']}. The Pi stayed on its current Wi-Fi."
            )
            self._save_state("idle")
            return {
                "forgotten": True,
                "uuid": clean_uuid,
                "message": self.last_result,
            }


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
                        "Confirm this page is open on the computer that will operate the robot."
                    )
                result = self.manager.confirm(
                    remote_address=self.remote_address,
                    token=payload.get("token"),
                )
            elif path == "/api/select":
                result = self.manager.select_saved_profile(
                    remote_address=self.remote_address,
                    profile_uuid=payload.get("uuid"),
                )
            elif path == "/api/forget":
                if payload.get("confirm") is not True:
                    raise ProvisioningError("Confirm removal of the saved profile.")
                result = self.manager.forget_profile(
                    remote_address=self.remote_address,
                    profile_uuid=payload.get("uuid"),
                )
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
        loss_grace_s=int(
            os.getenv("ROBOT_WIFI_LOSS_GRACE_S", "90")
        ),
        ready_state_path=Path(
            os.getenv(
                "ROBOT_WIFI_READY_STATE",
                "/run/my-bot-network/ready.json",
            )
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
        f"{args.bind}:{args.port}; network changes require a client on the "
        f"active {manager.interface} IPv4 subnet.",
        flush=True,
    )
    manager.start_recovery_monitor()
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        manager.stop_recovery_monitor()
        server.server_close()


if __name__ == "__main__":
    main()
