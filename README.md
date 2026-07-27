# My Bot

## Run the autonomous robot on the Raspberry Pi

The Raspberry Pi service owns the robot-critical stack: hardware, lidar,
odometry, robot TF, obstacle safety, Nav2, AMCL, and the map layers. Mission
Control runs as a second Pi service, while the laptop only needs a browser.

On the Raspberry Pi, build the workspace and install the service once:

```bash
cd /home/zrpi/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
./src/catering_bot/scripts/install_robot_service.sh
```

The installer discovers its actual checkout directory, so it also works when
the repository folder is named `my_bot` instead of `catering_bot`. The ROS
package and launch command remain named `my_bot` in either case.

The installer is safe to rerun after an update; it replaces the unit and
restarts the managed service. Every service start performs a clean-start sweep:
it stops stale robot/Nav2 processes, releases the lidar and motor serial
devices, and only then launches one new stack in its own process group.

The installer enables and immediately starts `my-bot-robot.service`. On each
boot, the service first clears stale owners, waits for `/dev/ttyUSB0` (lidar)
and `/dev/ttyACM0` (motor controller), then runs:

```bash
ros2 launch my_bot rpi_autonomy.launch.py use_heartbeat:=false
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

Do not run another robot launch manually while the service is active. The
hardware launch uses a process lock to reject a second stack before it can
compete for the Arduino and lidar serial streams. For a manual full-stack
debug launch:

```bash
sudo systemctl stop my-bot-robot.service
source /home/zrpi/robot_ws/install/setup.bash
ros2 launch my_bot rpi_autonomy.launch.py
```

For hardware-only diagnostics, use `rpi_robot.launch.py`. The original
central-compute mode remains available as a fallback during migration.

Restore automatic operation afterward with:

```bash
sudo systemctl start my-bot-robot.service
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
`ROBOT_LAUNCH_FILE`, `ROBOT_LIDAR_DEVICE`, `ROBOT_MOTOR_DEVICE`, and the
`ROBOT_WATCHDOG_*` settings. Clean startup is enabled by default with
`ROBOT_CLEAN_START=true`. Set
`ROBOT_LAUNCH_FILE=rpi_robot.launch.py` to temporarily restore the old
hardware-only profile. After an override, restart the service:

```bash
sudo systemctl daemon-reload
sudo systemctl restart my-bot-robot.service
```

For more reliable hardware naming, replace `/dev/ttyUSB0` and `/dev/ttyACM0`
with stable `/dev/serial/by-id/...` paths in both the service overrides and the
matching ROS configuration.

## Stop behavior

**Stop Navigation** cancels only the current Nav2 goal. **Safety Stop**
publishes `STOP` on `/robot/power_command`; the Pi safety node then holds zero
velocity continuously at the highest twist-mux priority. This latch survives a
browser disconnect or Mission Control restart and clears only after **Robot
On** or another reset command.

Software stops supplement, but do not replace, a physical emergency-stop
circuit.
