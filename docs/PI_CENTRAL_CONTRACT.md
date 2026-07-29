# Pi-to-central ROS 2 contract

## Responsibility boundary

The Raspberry Pi owns hardware access, local sensing, wheel odometry, robot
frames below `odom`, motion selection, motion safety, and hardware health. The
central computer owns `map -> odom`, SLAM or localization, maps, Nav2, Mission
Control, and the phone API.

Both computers must use the same `ROS_DOMAIN_ID`. Cyclone DDS multicast
discovery is the default. Set `ROBOT_CYCLONEDDS_INTERFACE` and the
comma-separated `ROBOT_CYCLONEDDS_PEERS` in `/etc/default/my-bot-robot` when
the deployment requires an explicit interface or unicast peers.

## Inputs to the Pi

| Topic | Type | Owner/purpose |
|---|---|---|
| `/cmd_vel_nav_raw` | `geometry_msgs/msg/Twist` | Central Nav2 velocity output. Never remap it to the controller topic. |
| `/cmd_vel_joy` | `geometry_msgs/msg/Twist` | Raw manual/joystick velocity from either the optional Pi joystick launch or an approved remote operator. |

No remote node may publish `/cmd_vel_nav_safe`, `/cmd_vel_joy_safe`,
`/cmd_vel_safety`, or `/diff_cont/cmd_vel_unstamped`.

## Central Stop contract

The Stop button belongs to the central navigation application. It cancels the
current Nav2 goal and stops publishing nonzero navigation commands. The Pi
then produces zero velocity as soon as the raw navigation input becomes zero
or expires. A later navigation request is a new goal and needs no Pi-side
reset.

There is no `/robot/power_command` interface and there are no Reset, On, or
mode commands in the Pi runtime. Stop is a navigation stop, not a substitute
for motor-power isolation.

## Outputs from the Pi

| Topic | Type | Producer |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | YDLidar driver |
| `/scan_filtered` | `sensor_msgs/msg/LaserScan` | canonicalizer + local laser filter chain |
| `/diff_cont/odom` | `nav_msgs/msg/Odometry` | `diff_drive_controller` |
| `/joint_states` | `sensor_msgs/msg/JointState` | joint-state broadcaster |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | motor hardware interface battery telemetry |
| `/tf` | `tf2_msgs/msg/TFMessage` | diff-drive controller and robot-state publisher |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | robot-state publisher |
| `/robot_health/front_obstacle_active` | `std_msgs/msg/Bool` | obstacle safety node |
| `/robot_health/front_speed_limit_scale` | `std_msgs/msg/Float32` | obstacle safety node |
| `/robot_health/startup_gate_open` | `std_msgs/msg/Bool` | automatic startup safety gate state |
| `/robot_health/log` | `std_msgs/msg/String` | safety and health nodes |

Additional Pi health topics are:

- `/robot_health/hardware_healthy`
- `/robot_health/lidar_healthy`
- `/robot_health/odometry_healthy`
- `/robot_health/controller_healthy`
- `/robot_health/obstacle_health`
- `/robot_health/ready`

All are `std_msgs/msg/Bool`. `ready` becomes true only after fresh raw scans,
fresh locally filtered scans, odometry, joint states, and obstacle-safety
health have all been observed and the automatic startup safety gate is open.
It is reporting only; a brief DDS discovery delay does not restart the
hardware service.

Mission Control may display the age of the most recently received
`/robot_health/ready` sample as **signal age**. This is a freshness indicator,
not a network round-trip-time measurement and does not require a Pi command or
ping topic.

## Command ownership and timeouts

The safety node transforms navigation input to `/cmd_vel_nav_safe` and manual
input to `/cmd_vel_joy_safe`. It publishes `/cmd_vel_safety` for a global
zero-velocity override. `twist_mux` is the only publisher to
`/diff_cont/cmd_vel_unstamped`.

| Layer | Timeout | Failure behavior |
|---|---:|---|
| startup motion gate | 5.0 s quiet period | motion stays zero until scans are fresh and no nonzero raw command is active |
| filtered scan freshness | 0.50 s | safety node continuously commands zero |
| raw navigation update | 0.25 s | safe navigation output becomes zero |
| raw manual update | 0.25 s | safe manual output becomes zero |
| mux safety/manual/nav inputs | 0.15/0.20/0.20 s | stale channel is dropped |
| diff-drive command | 0.25 s | controller commands zero wheel speed |
| Arduino host command | 0.20 s | firmware directly stops both motors |

The safety override priority is 255, manual priority is 100, and navigation
priority is 10.

## TF ownership audit

| Transform | Publisher | Computer |
|---|---|---|
| `map -> odom` | SLAM or localization node | central only |
| `odom -> base_link` | `diff_drive_controller` (`enable_odom_tf: true`) | Pi only |
| `base_link -> base_footprint` | robot-state publisher, fixed URDF joint | Pi only |
| `base_link -> chassis` | robot-state publisher, fixed URDF joint | Pi only |
| `chassis -> laser_frame` | robot-state publisher, fixed URDF joint | Pi only |
| `base_link -> left_wheel` | robot-state publisher from `/joint_states` | Pi only |
| `base_link -> right_wheel` | robot-state publisher from `/joint_states` | Pi only |
| remaining chassis/support/caster frames | robot-state publisher from fixed URDF joints | Pi only |

There is no Pi `map -> odom` publisher. The diff-drive controller is the sole
`odom -> base_link` publisher. Robot-state publisher does not publish that
transform because `odom` is not part of the URDF tree.
