#!/usr/bin/env python3
"""Serve a localhost-only, non-mutating preview of the Wi-Fi UI."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, List

from wifi_provisioning_server import (
    ProvisioningError,
    ProvisioningRequestHandler,
    ProvisioningServer,
    facility_connection_name,
    facility_connection_uuid,
    validate_psk,
    validate_ssid,
)


class PreviewManager:
    """In-memory manager implementing the provisioning handler contract."""

    def __init__(self, *, port: int):
        self.port = port
        self.ap_connection = "intellitrolley-ap"
        self.staged_ssid = ""
        self.staged_security = ""
        self.pending_ssid = ""
        self.profiles: Dict[str, Dict[str, Any]] = {
            "5f978c9c-06eb-5d14-b8a8-94bd31d316e5": {
                "connection": facility_connection_name("Office-5G"),
                "uuid": "5f978c9c-06eb-5d14-b8a8-94bd31d316e5",
                "ssid": "Office-5G",
                "confirmed": True,
                "active": False,
                "staged": False,
                "managed": True,
            }
        }
        self.last_result = "Preview mode: no network settings will be changed."

    def status(self, remote_address: str) -> Dict[str, Any]:
        del remote_address
        return {
            "interface": "wlan0",
            "active_connection": "intellitrolley-ap (preview)",
            "interface_address": "10.42.0.1/24",
            "ap_connection": self.ap_connection,
            "ap_gateway": "10.42.0.1/24",
            "can_provision": True,
            "using_recovery_ap": True,
            "can_configure_ap": True,
            "transition_phase": "idle",
            "staged": (
                {"ssid": self.staged_ssid, "security": self.staged_security}
                if self.staged_ssid
                else None
            ),
            "pending": (
                {"ssid": self.pending_ssid, "expires_in_s": 180}
                if self.pending_ssid
                else None
            ),
            "last_result": self.last_result,
            "saved_profiles": list(self.profiles.values()),
            "loss_grace_s": 90,
            "hostname": "127.0.0.1",
            "port": self.port,
        }

    def scan_networks(self, remote_address: str) -> List[Dict[str, Any]]:
        del remote_address
        return [
            {
                "ssid": "Warehouse WiFi",
                "security": "wpa-psk",
                "security_label": "WPA2",
                "signal": 92,
            },
            {
                "ssid": "Office-5G",
                "security": "wpa-psk",
                "security_label": "WPA2 WPA3",
                "signal": 76,
            },
            {
                "ssid": "Guest Network",
                "security": "open",
                "security_label": "Open",
                "signal": 58,
            },
            {
                "ssid": "Corporate 802.1X",
                "security": "enterprise",
                "security_label": "WPA2 802.1X",
                "signal": 43,
            },
        ]

    def stage(
        self,
        *,
        remote_address: str,
        ssid: Any,
        password: Any,
        security: Any,
        hidden: Any,
    ) -> Dict[str, Any]:
        del remote_address, hidden
        clean_ssid = validate_ssid(ssid)
        clean_security = str(security or "wpa-psk")
        if clean_security not in {"wpa-psk", "open"}:
            raise ProvisioningError("Preview supports Personal and open Wi-Fi only.")
        validate_psk(password, clean_security)
        self.staged_ssid = clean_ssid
        self.staged_security = clean_security
        profile_uuid = facility_connection_uuid(clean_ssid)
        for profile in self.profiles.values():
            profile["staged"] = False
        self.profiles[profile_uuid] = {
            "connection": facility_connection_name(clean_ssid),
            "uuid": profile_uuid,
            "ssid": clean_ssid,
            "confirmed": False,
            "active": False,
            "staged": True,
            "managed": True,
        }
        self.last_result = f"Preview saved {clean_ssid}; the hotspot remains active."
        return {"staged": True, "ssid": clean_ssid, "message": self.last_result}

    def activate(self, remote_address: str) -> Dict[str, Any]:
        del remote_address
        if not self.staged_ssid:
            raise ProvisioningError("Save a facility Wi-Fi profile first.")
        self.pending_ssid = self.staged_ssid
        return {
            "accepted": True,
            "ssid": self.pending_ssid,
            "timeout_s": 180,
            "confirm_url": f"http://127.0.0.1:{self.port}/?confirm=preview-token",
            "message": "Preview switch scheduled; no real Wi-Fi will change.",
        }

    def configure_current_ap(self, remote_address: str) -> Dict[str, Any]:
        return self._completion(remote_address, self.ap_connection)

    def confirm(self, *, remote_address: str, token: Any) -> Dict[str, Any]:
        if str(token or "") != "preview-token":
            raise ProvisioningError("Invalid preview confirmation token.")
        ssid = self.pending_ssid or self.staged_ssid or "Warehouse WiFi"
        self.pending_ssid = ""
        for profile in self.profiles.values():
            if profile["ssid"] == ssid:
                profile["confirmed"] = True
                profile["staged"] = False
        self.staged_ssid = ""
        self.staged_security = ""
        self.last_result = f"Preview confirmed {ssid}."
        return self._completion(remote_address, ssid)

    def select_saved_profile(
        self,
        *,
        remote_address: str,
        profile_uuid: Any,
    ) -> Dict[str, Any]:
        del remote_address
        profile = self.profiles.get(str(profile_uuid or ""))
        if profile is None:
            raise ProvisioningError("That preview profile does not exist.")
        for candidate in self.profiles.values():
            candidate["staged"] = False
        profile["staged"] = True
        self.staged_ssid = str(profile["ssid"])
        self.staged_security = "saved"
        self.last_result = f"Preview selected {self.staged_ssid}."
        return {"selected": True, "ssid": self.staged_ssid, "message": self.last_result}

    def forget_profile(
        self,
        *,
        remote_address: str,
        profile_uuid: Any,
    ) -> Dict[str, Any]:
        del remote_address
        clean_uuid = str(profile_uuid or "")
        profile = self.profiles.pop(clean_uuid, None)
        if profile is None:
            raise ProvisioningError("That preview profile does not exist.")
        if profile["staged"]:
            self.staged_ssid = ""
            self.staged_security = ""
        self.pending_ssid = ""
        self.last_result = f"Preview removed {profile['ssid']}."
        return {
            "forgotten": True,
            "uuid": clean_uuid,
            "message": self.last_result,
        }

    def _completion(self, remote_address: str, ssid: str) -> Dict[str, Any]:
        del remote_address
        return {
            "confirmed": True,
            "ssid": ssid,
            "robot_address": "192.168.50.42",
            "robot_subnet": "192.168.50.0/24",
            "central_address": "192.168.50.20",
            "ros_domain_id": 0,
            "robot_defaults_updated": True,
            "configuration_uri": (
                "intellitrolley://configure-network"
                "?robot=192.168.50.42&subnet=192.168.50.0%2F24&domain=0"
            ),
            "message": f"Preview completed the network flow for {ssid}.",
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        raise SystemExit("Port must be between 1 and 65535.")
    ui_dir = Path(__file__).resolve().parents[1] / "wifi_provisioning_ui"
    manager = PreviewManager(port=args.port)
    server = ProvisioningServer(
        ("127.0.0.1", args.port),
        ProvisioningRequestHandler,
        manager=manager,
        ui_dir=ui_dir,
    )
    print(
        f"Wi-Fi UI preview: http://127.0.0.1:{args.port}/ "
        "(simulation only; Ctrl-C to stop)",
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
