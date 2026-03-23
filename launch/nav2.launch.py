Rodrigo
rarcediaz
Invisible

Rodrigo
 — 
Yesterday at 3:23 PM

] [1774218045.265837298] [twist_mux]: Topic handler 'topics.keyboard' subscribed to topic 'cmd_vel_keyboard': timeout = 0.500000s , priority = 90.
[ydlidar_ros2_driver_node-4] [2026-03-22 15:20:45][info] Start to getting intensity flag
[ydlidar_ros2_driver_node-4] [2026-03-22 15:20:46][info] Auto set intensity 0
[ydlidar_ros2_driver_node-4] [2026-03-22 15:20:46][info] [YDLIDAR] End to getting intensity flag
[ydlidar_ros2_driver_node-4] [2026-03-22 15:20:46][info] [YDLIDAR] Create thread 0x9E8678E0
[ydlidar_ros2_driver_node-4] [2026-03-22 15:20:46][info] Successed to start scan mode, Elapsed time 1428 ms

message.txt
4 KB
process started with pid [7433]
[INFO] [spawner-8]: process started with pid [7435]
[ros2_control_node-6] [INFO] [1774218398.423085342] [controller_manager]: Subscribing to '~/robot_description' topic for robot description file.
[ros2_control_node-6] [INFO] [1774218398.428473399] [controller_manager]: update rate is 30 Hz
[ros2_control_node-6] [INFO] [1774218398.428654970] [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
[ros2_control_node-6] [WARN] [1774218398.430816600] [controller_manager]: Could not enable FIFO RT scheduling policy: with error number <1>(Operation not permitted). See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on how to enable realtime scheduling.
[ydlidar_ros2_driver_node-4] [2026-03-22 15:26:38][info] Single Fixed Size: 750
[ydlidar_ros2_driver_node-4] [2026-03-22 15:26:38][info] Sample Rate: 4.00K
[ydlidar_ros2_driver_node-4] [2026-03-22 15:26:38][info] Successed to check the lidar, Elapsed time 1724 ms
[ydlidar_ros2_driver_node-4] [2026-03-22 15:26:38][info] Now lidar is scanning...
ItsJason
 — 
Yesterday at 3:44 PM
Attachment file type: unknown
ros2_control.xacro
1.13 KB
Attachment file type: unknown
robot.urdf.xacro
506 bytes

controller_manager:
  ros__parameters:
    update_rate: 30

    diff_cont:
      type: diff_drive_controller/DiffDriveController

my_controllers.yaml
1 KB
Rodrigo
 — 
Yesterday at 4:02 PM
/dev/input/js0
zrpi@zrpi-desktop:~$ ros2 topic echo /cmd_vel_joy
Cannot echo topic '/cmd_vel_joy', as it contains more than one type: [geometry_msgs/msg/Twist, geometry_msgs/msg/TwistStamped]
zrpi@zrpi-desktop:~$ ^C
zrpi@zrpi-desktop:~$
Rodrigo
 — 
Yesterday at 4:10 PM
$ ros2 topic info /cmd_vel_joy -v
Type: ['geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped']

Publisher count: 1

Node name: teleop_node
Node namespace: /
Topic type: geometry_msgs/msg/TwistStamped
Endpoint type: PUBLISHER
GID: 01.0f.1f.88.fe.24.93.86.00.00.00.00.00.00.12.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: twist_mux
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: SUBSCRIPTION
GID: 01.0f.1f.88.02.25.84.d5.00.00.00.00.00.00.12.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

zrpi@zrpi-desktop:~$
ItsJason
 — 
Yesterday at 4:13 PM

twist_mux:
  ros__parameters:

    topics:
      keyboard:
        topic: cmd_vel_keyboard

twist_mux.yaml
1 KB
Rodrigo
 — 
Yesterday at 4:18 PM
^Crarcediaz@rodrigo-linux-laptop:~$ ros2 topic echo /cmd_vel_joy
Cannot echo topic '/cmd_vel_joy', as it contains more than one type: [geometry_msgs/msg/Twist, geometry_msgs/msg/TwistStamped]
rarcediaz@rodrigo-linux-laptop:~$
ros2 topic info /cmd_vel_joy -v
Type: ['geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped']

Publisher count: 1

Node name: teleop_node
Node namespace: /
Topic type: geometry_msgs/msg/TwistStamped
Endpoint type: PUBLISHER
GID: 01.0f.1f.88.9c.26.c1.2d.00.00.00.00.00.00.12.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: twist_mux
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: SUBSCRIPTION
GID: 01.0f.1f.88.a2.26.e3.bb.00.00.00.00.00.00.12.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

rarcediaz@rodrigo-linux-laptop:~$
ItsJason
 — 
Yesterday at 4:20 PM
grep -n "publish_stamped_twist" $(ros2 pkg prefix my_bot)/share/my_bot/config/joystick.yaml
grep -n "use_stamped" $(ros2 pkg prefix my_bot)/share/my_bot/config/twist_mux.yaml
grep -n "use_stamped_vel" $(ros2 pkg prefix my_bot)/share/my_bot/config/my_controllers.yaml
ros2 param get /teleop_node publish_stamped_twist
Rodrigo
 — 
Yesterday at 4:21 PM
s$ grep -n "publish_stamped_twist" $(ros2 pkg prefix my_bot)/share/my_bot/config/joystick.yaml
grep -n "use_stamped" $(ros2 pkg prefix my_bot)/share/my_bot/config/twist_mux.yaml
grep -n "use_stamped_vel" $(ros2 pkg prefix my_bot)/share/my_bot/config/my_controllers.yaml
11:    publish_stamped_twist: true
19:    use_stamped_vel: false
zrpi@zrpi-desktop:~/robot_ws$ ros2 param get /teleop_node publish_stamped_twist
Node not found
zrpi@zrpi-desktop:~/robot_ws$
ros2 param get /teleop_node publish_stamped_twist
Boolean value is: False
zrpi@zrpi-desktop:~$ ros2 topic info /cmd_vel_joy -v
Type: geometry_msgs/msg/Twist

Publisher count: 1

Node name: teleop_node
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: PUBLISHER
GID: 01.0f.1f.88.24.28.f3.db.00.00.00.00.00.00.12.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: twist_mux
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: SUBSCRIPTION
GID: 01.0f.1f.88.2a.28.db.39.00.00.00.00.00.00.12.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

zrpi@zrpi-desktop:~$

    0.0

velocity:

    0.0

0.0effort:

    .nan

.nan---
header:
  stamp:
    sec: 1774222043
    nanosec: 40506876
  frame_id: base_link
name:

    left_wheel_joint

right_wheel_jointposition:

    0.0

0.0velocity:

    0.0

0.0effort:

    .nan

.nan---
header:
  stamp:
    sec: 1774222044
    nanosec: 44128549
  frame_id: base_link
name:

    left_wheel_joint

right_wheel_jointposition:

    0.0

0.0velocity:

    0.0

0.0effort:

    .nan

.nan---
header:
  stamp:
    sec: 1774222045
    nanosec: 47465304
  frame_id: base_link
name:

    left_wheel_joint

right_wheel_jointposition:

    0.0

0.0velocity:

    0.0

0.0effort:

    .nan

.nan---
header:
  stamp:
    sec: 1774222046
    nanosec: 51043548
  frame_id: base_link
name:

    left_wheel_joint

right_wheel_jointposition:

    0.0

0.0velocity:

    0.0

0.0effort:

    .nan

.nan---
header:
  stamp:
    sec: 1774222047
    nanosec: 54447651
  frame_id: base_link
name:

    left_wheel_joint

right_wheel_jointposition:

    0.0

0.0velocity:

    0.0

0.0effort:

    .nan

ItsJason
 — 
Yesterday at 4:40 PM

#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdlib.h>


// ============================================================

arduino_sketch.ino
10 KB
ItsJason
 — 
Yesterday at 4:51 PM
grep -n "cmd_vel_unstamped" $(ros2 pkg prefix my_bot)/share/my_bot/launch/launch_robot.launch.py
ItsJason
 — 
Yesterday at 11:41 PM
bro u good with morning?
Rodrigo
 — 
Yesterday at 11:41 PM
Yeah , we’ll see when I wake up, cuz I just got home a bit ago need to eat dinner still
ItsJason
 — 
Yesterday at 11:43 PM
okok yeah good sleep is needed
let's assume 10 ish, up to ur time 
Rodrigo
 — 
9:49 AM
Shit I just woke up
ItsJason
 — 
10:31 AM
Dw
Come
Rodrigo
 — 
11:36 AM
Almost there
I’m transiting up so it’s taking me a little bit more time
ItsJason
 — 
11:37 AM
yeah np
Im here
ItsJason
 — 
12:18 PM

slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    resolution: 0.05
    restamp_tf: false
    min_laser_range: 0.1
    max_laser_range: 12.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    check_min_dist_and_heading_precisely: false
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
    min_pass_through: 2
    occupancy_threshold: 0.1

mapper_params_online_async.yaml
2 KB

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

launch_slam.launch.py
1 KB

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

slam.launch.py
2 KB
ItsJason
 — 
1:16 PM

#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdlib.h>


// ============================================================

arduino_sketch.ino
10 KB

amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2

nav2_params.yaml
11 KB

twist_mux:
  ros__parameters:
    topics:
      joystick:
        topic: /cmd_vel_joy
        timeout: 0.5

twist_mux.yaml
1 KB

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

nav2.launch.py
2 KB

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

slam.launch.py
1 KB

<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_bot</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>

package.xml
2 KB
1111
﻿
ItsJason
isjason.mk

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    package_name = 'my_bot'
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2_params.yaml'
    )

    nav2 = GroupAction([
        SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_nav'),
        SetRemap(src='smoothed_cmd_vel', dst='cmd_vel_nav'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'map': map_file,
                'params_file': params_file,
            }.items()
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true.'
        ),
        DeclareLaunchArgument(
            'map',
            description='Full path to the saved map YAML file.'
        ),
        nav2,
    ])
