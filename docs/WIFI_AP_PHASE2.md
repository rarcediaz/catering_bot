# Pi-hosted Wi-Fi and recovery-safe facility provisioning

Wi-Fi remains separate from ROS 2. NetworkManager owns `wlan0`; the robot
hardware/safety service neither starts Wi-Fi nor waits for it.

The deployed Raspberry Pi has been verified with:

- Ubuntu 22.04 on Raspberry Pi;
- NetworkManager 1.36;
- `wlan0` managed by NetworkManager; and
- access-point, 2.4 GHz, and 5 GHz capability.

The design has two connection classes:

1. `intellitrolley-ap` is the predictable recovery network. Its default SSID
   is `IntelliTrolley`, its Pi gateway is `10.42.0.1/24`, and NetworkManager
   shared mode supplies DHCP and DNS.
2. `intellitrolley-facility-*` is a customer/facility profile saved through
   the Pi provisioning page. It uses DHCP. A staged profile cannot
   autoconnect; only a successfully confirmed profile is enabled with a
   higher autoconnect priority than the recovery AP.

After the recovery AP is explicitly enabled, the Pi remains on it if no
facility profile is supplied. If a confirmed facility network is unavailable
at boot, NetworkManager can fall back to the saved AP profile.

## Safety and rollback contract

Changing Wi-Fi never disables the local motor, controller, lidar, or obstacle
safety timeouts. Loss of remote velocity data stops motion locally.

A facility switch uses NetworkManager's D-Bus checkpoint API. NetworkManager
records the working AP state before activating the facility profile. The
switch must be confirmed from the Windows central computer on the new network
within the configured timeout (180 seconds by default). If activation or
confirmation fails, NetworkManager restores the previous AP state
automatically. The D-Bus path is used because Ubuntu 22.04's NetworkManager
1.36 supports checkpoints but its `nmcli` frontend does not yet expose the
newer `device checkpoint` subcommand.

The confirmation request also:

- records the Windows source address as the Pi's explicit Cyclone DDS peer;
- preserves the current ROS domain;
- keeps `wlan0` and SPDP multicast in the Pi DDS settings;
- restarts the robot service only if it was already active; and
- returns a validated `intellitrolley://` handoff link for the Windows
  installer to update WSL, scoped firewall rules, and the central DDS peer.

The Windows step requires UAC because a browser must not silently change
firewall or WSL settings.

## Supported facility networks

The first provisioning release supports:

- WPA/WPA2 Personal networks using a pre-shared password; and
- open networks for controlled testing.

It deliberately does not accept WPA3-only, 802.1X/WPA-Enterprise credentials,
captive portal acceptance, certificates, or usernames. Those require a
separately reviewed administrator profile. Facility networks that isolate
wireless clients or block multicast may also prevent ROS 2 and confirmation
traffic.

Mission Control currently assumes a trusted private robot/facility LAN. Do not
place its unauthenticated test interface on a public or untrusted network.

## Prepare everything before changing the current Wi-Fi

These steps install the recovery AP profile and provisioning service without
activating the AP or disconnecting the current network.

From the Pi workspace:

```bash
cd ~/robot_ws
```

Create an AP password file outside the repository. The password must contain
8-63 characters (12 or more recommended):

```bash
mkdir -p /tmp/intellitrolley-ap-secret
chmod 0700 /tmp/intellitrolley-ap-secret
read -rsp "Choose IntelliTrolley Wi-Fi password: " ROBOT_AP_PASSWORD
echo
printf '%s\n' "${ROBOT_AP_PASSWORD}" >/tmp/intellitrolley-ap-secret/psk
chmod 0600 /tmp/intellitrolley-ap-secret/psk
unset ROBOT_AP_PASSWORD
```

Generate, install, and reload the inactive profile. The generated profile has
`connection.autoconnect=false`, so merely installing it cannot take over the
current Wi-Fi, including after an unexpected reboot during preparation:

```bash
ROBOT_AP_PSK_FILE=/tmp/intellitrolley-ap-secret/psk \
  ./src/catering_bot/scripts/generate_wifi_ap_config.sh \
  --output-dir /tmp/intellitrolley-ap

sudo install -o root -g root -m 0600 \
  /tmp/intellitrolley-ap/intellitrolley-ap.nmconnection \
  /etc/NetworkManager/system-connections/
sudo nmcli connection reload

rm -f /tmp/intellitrolley-ap-secret/psk
rm -f /tmp/intellitrolley-ap/intellitrolley-ap.nmconnection
```

Install and explicitly enable the provisioning service:

```bash
./src/catering_bot/scripts/install_wifi_provisioning_service.sh --dry-run
./src/catering_bot/scripts/install_wifi_provisioning_service.sh --enable
```

This service may run while the current client Wi-Fi is active, but its network
mutation endpoints remain locked. They unlock only when:

- `intellitrolley-ap` is the active `wlan0` connection; and
- the request source belongs to `10.42.0.0/24`.

Confirm that preparation changed no active connection:

```bash
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
systemctl is-active my-bot-wifi-provisioning.service
nmcli -f connection.id,connection.autoconnect,connection.autoconnect-priority,802-11-wireless.ssid,802-11-wireless.mode,802-11-wireless.band,802-11-wireless.channel,ipv4.method,ipv4.addresses \
  connection show intellitrolley-ap
```

## Activate the recovery AP

Use a local Pi console or known wired recovery path. Activating the AP
disconnects the current wireless SSH session:

```bash
sudo nmcli connection modify intellitrolley-ap \
  connection.autoconnect yes \
  connection.autoconnect-priority 50
sudo nmcli connection up intellitrolley-ap
```

Join `IntelliTrolley` from Windows, verify `ping 10.42.0.1`, then open:

```text
http://10.42.0.1:8090/
```

At this point the AP may be used permanently. No facility switch occurs unless
an operator saves a profile and presses **Switch and start rollback timer**.

To keep the hotspot as the robot network, select **Keep the robot hotspot** on
the page, confirm the robot is stationary, and choose **Use this hotspot**.
That records the current Windows hotspot address as the Pi DDS peer and
provides the Windows handoff for robot `10.42.0.1`, subnet `10.42.0.0/24`, and
the existing ROS domain. The Pi stays on the AP throughout.

## Provision a facility network

1. Open the provisioning page from the Windows central computer.
2. Scan or type the facility SSID.
3. Select Personal or Open security and enter the password if required.
4. Press **Save without switching**. The Pi remains on its AP, and the staged
   profile is kept out of NetworkManager autoconnect until confirmation.
5. Ensure Windows already knows the same facility Wi-Fi.
6. Immobilize the robot and start the protected switch.
7. Connect Windows to the facility Wi-Fi.
8. Open the displayed `zrpi-desktop.local:8090` confirmation link.
9. Confirm from the Windows central computer.
10. Select **Configure IntelliTrolley on Windows**, approve UAC, and complete
    the existing reciprocal-peer safety confirmation.
11. Run IntelliTrolley Diagnostics and verify the Pi topics and UI.

If the page cannot be reached on the facility network, do not guess a new Pi
address or edit profiles blindly. Wait for the checkpoint timeout, reconnect
to `IntelliTrolley`, and correct the facility settings.

## Boot behavior

The recovery AP profile has autoconnect priority `50`. Confirmed facility
profiles use priority `200`. NetworkManager chooses the facility connection
when available and can select the recovery AP when it is not.

`my-bot-wifi-provisioning.service` starts independently after NetworkManager.
`my-bot-robot.service` remains independent of `network-online.target`; the
local hardware and safety stack can be healthy while no central computer is
connected.

To start the robot service automatically after boot:

```bash
sudo systemctl enable my-bot-robot.service
```

## Manual recovery

From a local Pi console:

```bash
sudo nmcli connection modify intellitrolley-ap connection.autoconnect yes
sudo nmcli connection up intellitrolley-ap
```

To disable a bad facility profile without deleting it:

```bash
nmcli -t -f NAME connection show | grep '^intellitrolley-facility-'
sudo nmcli connection modify <facility-profile-name> connection.autoconnect no
sudo nmcli connection up intellitrolley-ap
```

The provisioning page can forget only profiles whose names begin with
`intellitrolley-facility-`; it refuses to delete unrelated user connections.

## Logs and status

```bash
sudo systemctl status my-bot-wifi-provisioning.service --no-pager
sudo journalctl -u my-bot-wifi-provisioning.service -f -o cat
nmcli connection show --active
ip -br address show wlan0
```
