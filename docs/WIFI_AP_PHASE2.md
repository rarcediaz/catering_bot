# Phase 2: Pi-hosted robot Wi-Fi

This design is disabled by default and is not part of
`install_robot_service.sh`. No current network configuration is changed.

## Manager and target assumptions

Raspberry Pi OS Bookworm and newer use NetworkManager by default. The actual
target Pi is currently unavailable, so verify it on the Pi before proceeding:

```bash
./src/my_bot/scripts/generate_wifi_ap_config.sh --detect-manager
systemctl is-active NetworkManager
nmcli general status
nmcli device status
```

If the target uses `systemd-networkd`, legacy `dhcpcd`, or another owner for
the Wi-Fi interface, stop and create a separate reviewed design. Do not let
two network managers configure the same interface.

Reference: [Raspberry Pi networking
documentation](https://www.raspberrypi.com/documentation/configuration/computers/raspberry-pi.html).

## Configurable design

Defaults are a `wlan0` AP named `IntelliTrolley` with Pi gateway
`10.42.0.1/24`, country `CA`, and NetworkManager IPv4 shared mode for local
DHCP and DNS. Override:

- `ROBOT_AP_INTERFACE`
- `ROBOT_AP_SSID`
- `ROBOT_AP_ADDRESS_CIDR`
- `ROBOT_AP_COUNTRY`
- `ROBOT_AP_CONNECTION_NAME`
- `ROBOT_AP_PSK_FILE`

No laptop hostname or password is stored in Git. Preview without writing:

```bash
./src/my_bot/scripts/generate_wifi_ap_config.sh --dry-run
```

On the Pi, place a strong password in a root-readable temporary or secrets
file outside the repository, then generate a reviewable keyfile:

```bash
ROBOT_AP_PSK_FILE=/run/secrets/robot-ap-psk \
  ./src/my_bot/scripts/generate_wifi_ap_config.sh \
  --output-dir /tmp/intellitrolley-ap
```

The generated keyfile has mode 0600. The helper does not install or enable it.

## Boot order and stack relationship

Wi-Fi is not started by ROS and is not a node in `rpi_robot.launch.py`. The
generator above only prepares a configuration file; it does not change the
Pi's networking.

Before Phase 2 is enabled, the Pi continues using whatever networking the OS
already has, and the robot service starts without waiting for Wi-Fi. After the
generated profile is installed once, its `autoconnect=true` setting tells
NetworkManager to activate `intellitrolley-ap` during normal OS boot.
NetworkManager creates the access point, assigns the Pi gateway address, and
provides local DHCP and DNS independently of ROS.

The robot systemd service is ordered only after local filesystems. In
particular, it does not depend on `network-online.target`: it can start the
hardware, safety, lidar, and controller stack while NetworkManager is still
bringing up the access point. The central computer and phone join the AP
later. ROS 2 DDS discovery begins when the central computer is reachable.

If the AP starts late or restarts, the local robot stack does not need a
restart. Loss of Wi-Fi makes the remote velocity input expire, so the Pi-side
safety and controller timeouts command zero while the service remains alive.

## Explicit future enable step

Use a local console or a tested wired recovery path because activating an AP
can disconnect the current Wi-Fi session.

```bash
sudo raspi-config nonint do_wifi_country CA
sudo install -o root -g root -m 0600 \
  /tmp/intellitrolley-ap/intellitrolley-ap.nmconnection \
  /etc/NetworkManager/system-connections/
sudo nmcli connection reload
sudo nmcli connection up intellitrolley-ap
```

Confirm the Pi owns the configured gateway, a client receives DHCP and DNS,
and `ping`, SSH, ROS 2 discovery/data, and the central-hosted Mission Control
API work over the robot LAN. The Mission Control API remains on the central
computer, not the Pi.

## ROS 2 settings on the robot LAN

Choose one non-default `ROS_DOMAIN_ID` and configure it identically on Pi and
central computer. Either:

- use SPDP multicast bound to the AP interface; or
- give the central computer a predictable address (for example a reviewed
  static client address) and configure explicit Cyclone peers.

Pi `/etc/default/my-bot-robot` example:

```text
ROS_DOMAIN_ID=42
ROBOT_CYCLONEDDS_INTERFACE=wlan0
ROBOT_CYCLONEDDS_PEERS=10.42.0.2
ROBOT_CYCLONEDDS_ALLOW_MULTICAST=spdp
```

Configure the central peer as `10.42.0.1`; do not hardcode a developer laptop
hostname in shared configuration.

## Exposure limits

- Use WPA2/WPA3 and a strong secret stored only on deployed machines.
- Permit SSH only from the robot subnet and prefer key authentication.
- Bind the central Mission Control API to the robot-LAN address, or firewall
  its port so only the robot subnet can reach it.
- Permit DDS UDP only on the AP interface/robot subnet.
- Do not add public router port forwards.
- Review NetworkManager shared-mode forwarding. If an uplink exists, add a
  firewall forward-drop policy so the robot LAN is not an internet gateway
  unless that behavior is explicitly approved.

Wi-Fi loss must not affect the local service: the Pi controller timeout,
Arduino watchdog, safety node, lidar processing, startup motion gate, and
device watchdog all remain local.

## Rollback and recovery

From a console or wired recovery link:

```bash
sudo nmcli connection down intellitrolley-ap
sudo nmcli connection delete intellitrolley-ap
sudo rm /etc/NetworkManager/system-connections/intellitrolley-ap.nmconnection
sudo nmcli connection reload
sudo nmcli radio wifi off
sudo nmcli radio wifi on
```

Then bring up the prior client connection with
`sudo nmcli connection up <previous-name>`. Before enabling Phase 2, record
that previous connection name and test console access. If NetworkManager
cannot recover, remove the AP keyfile from local console or by mounting the SD
card on another Linux computer, then reboot.
