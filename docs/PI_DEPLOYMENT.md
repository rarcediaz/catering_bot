# Raspberry Pi deployment and hardware acceptance

These steps are intentionally deferred until the Pi is connected. Do not run
them on a developer workstation.

## 1. Transfer and build

Use the project's normal Git deployment flow, or copy the repository to the
Pi only when it is available. On the Pi:

```bash
cd /path/to/robot_ws
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt update
sudo apt install psmisc util-linux \
  ros-$ROS_DISTRO-controller-manager \
  ros-$ROS_DISTRO-diff-drive-controller \
  ros-$ROS_DISTRO-joint-state-broadcaster \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-twist-mux \
  ros-$ROS_DISTRO-laser-filters \
  ros-$ROS_DISTRO-rmw-cyclonedds-cpp
colcon build --symlink-install --packages-up-to my_bot
source install/setup.bash
```

`ydlidar_ros2_driver` and `diffdrive_arduino` must also be present in the
workspace or installed separately. The Pi service installer intentionally
does not require Nav2, AMCL, SLAM, RViz, lifecycle managers, or map servers.
Because this repository also contains central-computer launch files, avoid a
blanket Pi `rosdep install` if keeping the Pi image minimal.

This change intentionally uses low-risk runtime separation and does not rename
or split the ROS package. A later dependency-isolation migration could move
central launch/config/map files into a `my_bot_navigation` package while
keeping hardware interfaces in `my_bot`; that should be planned and tested as
a separate central-computer change, not folded into Pi deployment.

## 2. Select stable serial devices

Connect one device at a time and record the stable names:

```bash
ls -l /dev/serial/by-id/
udevadm info --query=property --name=/dev/ttyACM0
udevadm info --query=property --name=/dev/ttyUSB0
```

Prefer the discovered `/dev/serial/by-id/...` symlinks. Do not invent udev
vendor/product/serial matches without the physical hardware. If a device has
no unique serial number, create a reviewed Pi-only udev rule later using its
actual attributes and USB port path.

## 3. Render and install the service

First install the reviewed recovery AP profile as described in
[the Wi-Fi provisioning flow](WIFI_AP_PHASE2.md), then install the services:

```bash
cd /path/to/robot_ws
source install/setup.bash
./src/my_bot/scripts/install_wifi_provisioning_service.sh --dry-run
./src/my_bot/scripts/install_wifi_provisioning_service.sh --enable
./src/my_bot/scripts/install_robot_service.sh --dry-run
./src/my_bot/scripts/install_robot_service.sh --no-start
sudoedit /etc/default/my-bot-robot
```

The robot installer refuses a real installation when the network-ready unit
is absent.

Set at least:

```text
ROBOT_LIDAR_DEVICE=/dev/serial/by-id/<actual-lidar-id>
ROBOT_MOTOR_DEVICE=/dev/serial/by-id/<actual-arduino-id>
ROS_DOMAIN_ID=<shared-domain-id>
```

Optionally set `ROBOT_CYCLONEDDS_INTERFACE` and
`ROBOT_CYCLONEDDS_PEERS`. Then:

```bash
sudo systemctl start my-bot-robot.service
sudo systemctl status my-bot-robot.service --no-pager
sudo journalctl -u my-bot-robot.service -f -o cat
```

The unit selects `rpi_robot.launch.py`. It requires
`my-bot-network-ready.service`, checks that `wlan0` has IPv4, waits for both
serial devices, and rejects duplicate wrappers or pre-existing device owners.

On the first managed start, the wrapper performs a targeted clean-slate sweep
of stale `my_bot`, hardware-driver, controller, and central-autonomy processes
that could conflict with this Pi stack. It records completion in
`~/.local/state/my-bot/initial-clean-start-complete`. Later starts rely on the
normal locks and serial-owner checks instead of repeatedly killing processes.

For a deliberate recovery cleanup, temporarily set
`ROBOT_INITIAL_CLEAN_START=always` in `/etc/default/my-bot-robot`, restart the
service once, and then return it to `once`. Alternatively, with the service
stopped, delete only the marker above and start the service.

## 4. First-start checks with wheels raised

Keep the drive wheels clear of the floor during these checks.

```bash
ros2 node list
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic hz /diff_cont/odom
ros2 topic hz /joint_states
ros2 topic echo /robot_health/ready --once
ros2 topic echo /robot_health/startup_gate_open
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
```

At startup, verify `startup_gate_open: false` and that navigation/manual commands do
not move the wheels. With fresh filtered scans and no active nonzero raw
command, the gate must open automatically after the configured quiet period.
Confirm raw and filtered scan geometry, encoder direction, odometry direction,
wheel separation, and lidar frame orientation.

Command low forward, reverse, left-turn, and right-turn velocities through
`/cmd_vel_joy` or `/cmd_vel_nav_raw`; never publish the controller command
topic directly. Confirm each direction. From the central UI, start a small
navigation goal and press Stop; verify the current Nav2 goal is canceled and
the raw navigation command becomes zero or expires.

## 5. Safety and disconnect acceptance

- Place a test obstacle in the front corridor. Confirm
  `/robot_health/front_obstacle_active` and zero/limited forward motion.
- Verify reverse remains available when only the front is blocked, then test
  rear and side constraints.
- Stop filtered scans. Motion must stop within the configured 0.50 s scan
  freshness timeout and obstacle health must become false.
- Stop navigation command updates. Motion must stop through the 0.25 s safety
  timeout and 0.25 s controller timeout.
- Stop the safety node. The safe mux inputs must expire and the robot must
  stop; the remote raw topics must not reach the controller.
- Stop the controller process while moving with wheels raised. The Arduino's
  200 ms host-command watchdog must stop both motors.
- Run `sudo systemctl restart my-bot-robot.service`. Old commands must not
  replay. The startup gate must remain closed while a nonzero raw command is
  present, then open automatically only after fresh scans and a quiet period.
- Unplug each serial device separately. After persistent local device loss,
  the wrapper must stop the process group and systemd may retry without two
  serial owners.

Do not claim acceptance until these physical tests pass.

## 6. Reboot and central-disconnection test

1. Reboot the Pi with the central computer powered off.
2. Confirm `my-bot-network-ready.service` selects a saved Wi-Fi, or starts the
   recovery AP when no saved Wi-Fi is reachable, before the robot service.
3. Confirm the startup safety gate opens automatically after fresh scans and a
   quiet period, with the wheels stationary.
4. Start the central computer and confirm discovery, scan, odometry, and TF.
5. Command low speed with wheels raised, then disable central Wi-Fi or stop
   its ROS processes.
6. Confirm motion stops and the Pi service remains alive.
7. Restore connectivity. Confirm motion does not resume from an old command;
   only a fresh command may move the robot.
8. Reboot once more and inspect:

```bash
sudo systemctl status my-bot-robot.service --no-pager
sudo journalctl -b -u my-bot-robot.service -o cat
```
