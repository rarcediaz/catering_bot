# IntelliTrolley ROS 2

The Raspberry Pi is the robot's hardware-and-safety computer. Its only
supported production entrypoint is:

```bash
ros2 launch my_bot rpi_robot.launch.py
```

That launch owns the Arduino/`ros2_control` stack, lidar and local scan
processing, wheel odometry, robot TF, local command selection, obstacle safety,
automatic startup motion gating, and robot-health reporting. It does **not** start
SLAM, AMCL, Nav2, maps, RViz, Mission Control, or a phone backend.

The central computer owns `map -> odom`, SLAM/localization, maps, Nav2, Mission
Control, and the phone-facing API. `rpi_autonomy.launch.py` is retained only as
a legacy all-in-one diagnostic during migration and is rejected by the
production service wrapper.

## Safety command path

```text
central Nav2  /cmd_vel_nav_raw ─┐
                                ├─> obstacle_safety_node
manual input  /cmd_vel_joy ─────┘        ├─> /cmd_vel_nav_safe
                                         ├─> /cmd_vel_joy_safe
scan/startup safety ────────────────────> /cmd_vel_safety
                                                   │
                                                   v
                                              twist_mux
                                                   │
                                                   v
                                  /diff_cont/cmd_vel_unstamped
                                                   │
                                                   v
                                  diff_drive_controller -> Arduino
```

Only `twist_mux` publishes the diff-drive command topic. Safety override has
the highest mux priority, safe manual control is next, and safe navigation is
last. Missing raw commands, a failed safety node, stale filtered scans, the
controller command timeout, and the Arduino firmware watchdog all fail toward
zero velocity.

There is no Reset/On command or Pi-side operator latch. On every safety-node
start, motion is inhibited automatically until filtered lidar is fresh and no
active raw motion command has been observed for five seconds. This prevents an
already-streaming command from passing straight through a restart.

The operator Stop button is a central-computer operation: it cancels the
current Nav2 goal. Nav2 then stops or zeros `/cmd_vel_nav_raw`, and the Pi
command/controller timeouts stop the motors. This is a navigation stop, not an
emergency stop; a physical emergency-stop circuit is still required.

## Documents

- [Pi-to-central topic and TF contract](docs/PI_CENTRAL_CONTRACT.md)
- [Pi deployment and hardware acceptance](docs/PI_DEPLOYMENT.md)
- [Disabled Phase 2 Wi-Fi access-point design](docs/WIFI_AP_PHASE2.md)

## Service summary

Build first, then run the installer as the normal Pi user:

```bash
cd /path/to/robot_ws
colcon build --symlink-install --packages-up-to my_bot
./src/my_bot/scripts/install_robot_service.sh --dry-run
./src/my_bot/scripts/install_robot_service.sh --no-start
```

Edit `/etc/default/my-bot-robot` to select stable
`/dev/serial/by-id/...` motor and lidar paths, a common `ROS_DOMAIN_ID`, and
optional Cyclone DDS peers. Then:

```bash
sudo systemctl start my-bot-robot.service
sudo systemctl status my-bot-robot.service --no-pager
sudo journalctl -u my-bot-robot.service -f -o cat
```

On its first managed start, the wrapper stops old robot/autonomy processes and
serial-device owners, then records a marker under
`~/.local/state/my-bot/`. Later starts rely on the wrapper lock, launch lock,
and explicit device-owner checks instead of repeatedly killing processes.

The wrapper waits for both devices without treating Wi-Fi or DDS discovery as
a startup dependency. Shutdown publishes a best-effort zero safety command
before terminating the launch process group; controller and firmware timeouts
remain the independent backstops.

## How Wi-Fi relates to the robot stack

Wi-Fi is not a ROS node and is not started by `rpi_robot.launch.py`. The Phase
2 access point is currently disabled. Once its reviewed NetworkManager profile
is explicitly installed, NetworkManager starts the AP automatically during Pi
boot because the profile has `autoconnect=true`. The ROS hardware service
starts independently and may be running before Wi-Fi is ready.

The central computer and phone then join the Pi-hosted network. DDS discovers
the central computer when the network appears; a Wi-Fi failure removes remote
commands but does not stop the local lidar, safety, controller, or firmware
watchdogs. See [the Phase 2 boot flow](docs/WIFI_AP_PHASE2.md#boot-order-and-stack-relationship).

Never run `launch_robot.launch.py` directly in production. It is an internal
include without the top-level hardware lock.
