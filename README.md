# My Bot

## Start the physical robot automatically at boot

The Raspberry Pi service starts only the hardware-facing robot stack. Nav2,
AMCL, mapping, and Mission Control belong on the central computer.

On the Raspberry Pi, build the workspace and install the service once:

```bash
cd /home/zrpi/robot_ws
colcon build --symlink-install
./src/catering_bot/scripts/install_robot_service.sh
```

The installer discovers its actual checkout directory, so it also works when
the repository folder is named `my_bot` instead of `catering_bot`. The ROS
package and launch command remain named `my_bot` in either case.

Stop any manually launched robot stack before running the installer. The
installer is safe to rerun after an update; it replaces the unit and restarts
the managed service.

The installer enables and immediately starts `my-bot-robot.service`. On each
boot, the service waits for `/dev/ttyUSB0` (lidar) and `/dev/ttyACM0` (motor
controller), then runs:

```bash
ros2 launch my_bot rpi_robot.launch.py use_heartbeat:=false
```

After a startup grace period, the wrapper checks that the lidar and motor
devices remain present. Three consecutive device failures cause the complete
launch to restart through systemd. The lidar launch also respawns its driver
after an isolated driver exit. ROS topic probes are disabled by default because
short ROS graph discovery delays can otherwise cause false restarts; they can
be enabled with `ROBOT_WATCHDOG_TOPIC_CHECKS=true` for diagnostics.

Useful commands on the Raspberry Pi:

```bash
sudo systemctl status my-bot-robot.service --no-pager
sudo journalctl -u my-bot-robot.service -f -o cat
sudo systemctl restart my-bot-robot.service
sudo systemctl stop my-bot-robot.service
```

For developer access from the central computer, use SSH rather than exposing a
second debug service:

```bash
ssh zrpi@zrpi-desktop.local 'sudo systemctl status my-bot-robot.service --no-pager'
ssh -t zrpi@zrpi-desktop.local 'sudo journalctl -u my-bot-robot.service -f -o cat'
```

Use SSH keys and keep the private key only on the developer's central computer.
The robot does not expose its logs through the Mission Control API.

The wrapper automatically detects ROS 2 Humble or Jazzy. Deployment settings
can be overridden with `sudo systemctl edit my-bot-robot.service`; supported
variables include `ROS_DOMAIN_ID`, `ROS_SETUP_FILE`, `ROBOT_WORKSPACE`,
`ROBOT_LIDAR_DEVICE`, `ROBOT_MOTOR_DEVICE`, and the `ROBOT_WATCHDOG_*`
settings. After an override, restart the service:

```bash
sudo systemctl daemon-reload
sudo systemctl restart my-bot-robot.service
```

For more reliable hardware naming, replace `/dev/ttyUSB0` and `/dev/ttyACM0`
with stable `/dev/serial/by-id/...` paths in both the service overrides and the
matching ROS configuration.
