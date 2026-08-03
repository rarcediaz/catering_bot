# Wi-Fi-before-ROS boot and recovery-safe facility provisioning

NetworkManager owns `wlan0`, credentials, DHCP, and the recovery access point.
`my-bot-network-ready.service` makes the boot choice before ROS starts:

1. accept an already-connected saved client Wi-Fi;
2. try saved, autoconnect-enabled Wi-Fi profiles in priority order; or
3. start `intellitrolley-ap` when no saved client profile connects.

The gate succeeds only after `wlan0` has a global IPv4 address.
`my-bot-robot.service` and the provisioning UI both require that gate. The ROS
wrapper performs its own IPv4 check as a second guard and binds Cyclone DDS to
`wlan0`. The network gate also wants the robot unit, so reaching either a saved
client network or the recovery AP starts the robot automatically without a UI
selection.

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

An intentional UI-driven Wi-Fi switch stops the ROS service before changing
the address. The controller and Arduino command watchdogs stop motion while
the service is down. After NetworkManager selects the new connection and
assigns IPv4, ROS starts on that network while confirmation remains pending.
Rollback stops ROS before restoring the hotspot, then starts it again on the
hotspot. An ordinary transient loss of remote commands is still handled by the
local safety timeouts.

A facility switch uses NetworkManager's D-Bus checkpoint API. NetworkManager
records the working AP state before activating the facility profile. The
switch must be confirmed from the Windows central computer on the new network
within the configured timeout (180 seconds by default). If activation or
confirmation fails, the provisioner explicitly rolls back the checkpoint and
activates the AP if NetworkManager has not restored it. Confirmation and
rollback use a monotonic timer, so acquiring internet and correcting the Pi's
wall clock cannot expire a new switch. The D-Bus path is used because Ubuntu
22.04's NetworkManager
1.36 supports checkpoints but its `nmcli` frontend does not yet expose the
newer `device checkpoint` subcommand.

The confirmation request also:

- records the Windows source address as the Pi's explicit Cyclone DDS peer;
- preserves the current ROS domain;
- keeps `wlan0` and SPDP multicast in the Pi DDS settings;
- ensures the robot service is running on the selected Wi-Fi; and
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
  ./src/my_bot/scripts/generate_wifi_ap_config.sh \
  --output-dir /tmp/intellitrolley-ap

sudo install -o root -g root -m 0600 \
  /tmp/intellitrolley-ap/intellitrolley-ap.nmconnection \
  /etc/NetworkManager/system-connections/
sudo nmcli connection reload

rm -f /tmp/intellitrolley-ap-secret/psk
rm -f /tmp/intellitrolley-ap/intellitrolley-ap.nmconnection
```

Install the Wi-Fi startup gate and provisioning service without starting them:

```bash
./src/my_bot/scripts/install_wifi_provisioning_service.sh --dry-run
./src/my_bot/scripts/install_wifi_provisioning_service.sh
```

Confirm that preparation changed no active connection:

```bash
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
systemctl is-enabled my-bot-wifi-provisioning.service
systemctl is-enabled my-bot-network-ready.service
nmcli -f connection.id,connection.autoconnect,connection.autoconnect-priority,802-11-wireless.ssid,802-11-wireless.mode,802-11-wireless.band,802-11-wireless.channel,ipv4.method,ipv4.addresses \
  connection show intellitrolley-ap
```

Now enable the stack. An already-active client connection remains selected;
otherwise saved profiles are tried before the recovery AP:

```bash
./src/my_bot/scripts/install_wifi_provisioning_service.sh --enable
```

The installer enables both `my-bot-network-ready.service` and
`my-bot-wifi-provisioning.service`. Provisioning mutations remain locked unless:

- `intellitrolley-ap` is the active `wlan0` connection; and
- the request source belongs to `10.42.0.0/24`.

## Activate the recovery AP

Use a local Pi console or known wired recovery path. Activating the AP
disconnects the current wireless SSH session:

```bash
sudo systemctl stop my-bot-robot.service
sudo nmcli connection modify intellitrolley-ap \
  connection.autoconnect yes \
  connection.autoconnect-priority 50
sudo nmcli connection up intellitrolley-ap
sudo systemctl start my-bot-robot.service
```

Join `IntelliTrolley` from Windows, verify `ping 10.42.0.1`, then open:

```text
http://10.42.0.1:8090/
```

At this point the AP may be used permanently, and the robot service starts on
it automatically. No facility switch occurs unless an operator saves a profile
and presses **Switch and start rollback timer**.

The optional **Configure this computer for the robot hotspot** action records
the current Windows hotspot address as the Pi DDS peer and provides the Windows
handoff for robot `10.42.0.1`, subnet `10.42.0.0/24`, and the existing ROS
domain. It is not required to start the Pi network or robot service. The Pi
stays on the AP throughout.

## Provision a facility network

1. Open the provisioning page from the Windows central computer.
2. Scan and select the facility SSID from the visible nearby-network list, or
   type it exactly for a hidden network.
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
profiles use priority `200`. The startup gate temporarily holds the AP out of
autoconnect while it tries saved client profiles for
`ROBOT_WIFI_CLIENT_WAIT_S` (30 seconds by default). If none connects, it
re-enables and activates the AP. With no saved client profiles, the AP starts
immediately. Read the selected mode and address from:

```bash
cat /run/my-bot-network/ready.json
```

The gate does not depend on the broad `network-online.target`; it verifies the
specific interface ROS uses. The robot service cannot start if both saved
Wi-Fi and the AP fail. This avoids Cyclone DDS starting against the wrong or
missing interface.

The Wi-Fi gate requests the robot service automatically after either network
mode becomes ready. The robot installer also enables its unit directly, so it
remains independently manageable with `systemctl`.

## ROS 2 network acceptance

The generated Cyclone DDS configuration selects `wlan0`, uses UDP transport,
enables SPDP multicast discovery, and can add the central computer as an
explicit peer. The AP uses NetworkManager shared mode, which supplies the
`10.42.0.0/24` LAN without wireless client isolation. A facility WLAN must
also allow peer-to-peer client traffic; guest networks with client isolation
are unsuitable.

If UFW is enabled on the Pi, allow the trusted robot subnet to reach DDS UDP
and the provisioning page. Substitute the actual facility subnet when not on
the hotspot:

```bash
sudo ufw allow in on wlan0 from 10.42.0.0/24 to any proto udp
sudo ufw allow in on wlan0 from 10.42.0.0/24 to any port 8090 proto tcp
```

Then verify from the central computer, using the same `ROS_DOMAIN_ID`:

```bash
ping <pi-wlan0-address>
ros2 multicast receive
```

Run `ros2 multicast send` on the Pi and confirm the central receiver sees it.
Next, with `my-bot-robot.service` active, run on the central computer:

```bash
ros2 node list
ros2 topic echo /robot_health/ready --once
ros2 topic hz /scan_filtered
ros2 topic hz /diff_cont/odom
```

Successful ping alone is not acceptance: the ROS graph, health topic, lidar,
and odometry must all cross the selected Wi-Fi. If multicast is blocked but
unicast peer traffic is allowed, complete the UI handoff so each machine has
the other as an explicit Cyclone DDS peer, then repeat the graph checks.

## Manual recovery

From a local Pi console:

```bash
sudo systemctl stop my-bot-robot.service
sudo nmcli connection modify intellitrolley-ap connection.autoconnect yes
sudo nmcli connection up intellitrolley-ap
sudo systemctl start my-bot-robot.service
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
sudo systemctl status my-bot-network-ready.service --no-pager
sudo journalctl -u my-bot-wifi-provisioning.service -f -o cat
sudo journalctl -b -u my-bot-network-ready.service -o cat
nmcli connection show --active
ip -br address show wlan0
```
