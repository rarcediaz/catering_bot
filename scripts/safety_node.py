#!/usr/bin/env python3
import rclpy
from action_msgs.srv import CancelGoal
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, Float64MultiArray, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math

try:
    import board
    from adafruit_ads1x15.ads1115 import ADS1115
    from adafruit_ads1x15.analog_in import AnalogIn
    import adafruit_ads1x15.ads1x15 as ads1x15
    ADC_IMPORT_ERROR = None
except Exception as exc:
    board = None
    ADS1115 = None
    AnalogIn = None
    ads1x15 = None
    ADC_IMPORT_ERROR = exc

class IntegrityNode(Node):
    def __init__(self):
        super().__init__('integrity_node')
        
        # --- CONFIGURATION ---
        self.DIVIDER_RATIO = 8.2727 
        self.CRITICAL_LATENCY_MS = 2500.0
        self.CRITICAL_VOLTAGE = -1 # Updated to match your 0% threshold
        
        # --- BATTERY S-CURVE LUT ---
        self.BATTERY_LUT = [
            (24.00, 100), (23.70, 90), (23.40, 80), (23.15, 70),
            (22.90, 60),  (22.65, 50), (22.40, 40), (22.15, 30),
            (21.90, 20),  (21.75, 15), (21.60, 10), (21.45, 5),
            (21.35, 4),   (21.25, 3),  (21.15, 2),  (21.05, 1),
            (21.00, 0)
        ]
        
        # --- STATE & SAFETY ---
        self.current_mode = "STOP"
        self.safety_lock = False
        self.front_obstacle_active = False
        self.left_obstacle_active = False
        self.right_obstacle_active = False
        self.rear_obstacle_active = False
        self.latency_fail_count = 0
        self.history = []
        self.MAX_SAMPLES = 20
        self.SAFETY_PUBLISH_HZ = 20.0

        self.declare_parameter('obstacle_stop_enabled', True)
        self.declare_parameter('obstacle_stop_distance_m', 0.20)
        self.declare_parameter('obstacle_stop_distance_max_m', 0.60)
        self.declare_parameter('obstacle_stop_speed_mps', 0.60)
        self.declare_parameter('obstacle_slowdown_margin_m', 0.15)
        self.declare_parameter('front_stop_start_x_m', 0.0508)
        self.declare_parameter('rear_stop_start_x_m', 1.016)
        self.declare_parameter('front_stop_width_m', 0.8596)
        self.declare_parameter('side_stop_distance_m', 0.25)
        self.declare_parameter('side_stop_start_y_m', 0.34)
        self.declare_parameter('joystick_timeout_sec', 0.5)
        self.declare_parameter('nav_timeout_sec', 0.25)
        self.declare_parameter('safety_bypass_mode', False)

        self.obstacle_stop_enabled = self.get_bool_parameter('obstacle_stop_enabled')
        self.obstacle_stop_distance_m = float(self.get_parameter('obstacle_stop_distance_m').value)
        self.obstacle_stop_distance_max_m = float(self.get_parameter('obstacle_stop_distance_max_m').value)
        self.obstacle_stop_speed_mps = float(self.get_parameter('obstacle_stop_speed_mps').value)
        self.obstacle_slowdown_margin_m = float(self.get_parameter('obstacle_slowdown_margin_m').value)
        self.front_stop_start_x_m = float(self.get_parameter('front_stop_start_x_m').value)
        self.rear_stop_start_x_m = float(self.get_parameter('rear_stop_start_x_m').value)
        self.front_stop_width_m = float(self.get_parameter('front_stop_width_m').value)
        self.front_stop_half_width_m = 0.5 * self.front_stop_width_m
        self.side_stop_distance_m = float(self.get_parameter('side_stop_distance_m').value)
        self.side_stop_start_y_m = float(self.get_parameter('side_stop_start_y_m').value)
        self.joystick_timeout_sec = float(self.get_parameter('joystick_timeout_sec').value)
        self.nav_timeout_sec = float(self.get_parameter('nav_timeout_sec').value)
        self.safety_bypass_mode = self.get_bool_parameter('safety_bypass_mode')
        if self.safety_bypass_mode:
            self.current_mode = "AUTO"

        self.closest_forward_clearance = math.inf
        self.closest_left_clearance = math.inf
        self.closest_right_clearance = math.inf
        self.closest_rear_clearance = math.inf
        self.dynamic_stop_distance_m = self.obstacle_stop_distance_m
        self.forward_speed_mps = 0.0
        self.speed_limit_scale = 1.0
        self.latest_joy_cmd = Twist()
        self.latest_nav_cmd = Twist()
        self.latest_joy_time = None
        self.latest_nav_time = None

        # --- ADC HARDWARE SETUP ---
        try:
            if ADC_IMPORT_ERROR is not None:
                raise RuntimeError(f"ADC Python dependencies unavailable: {ADC_IMPORT_ERROR}")
            i2c = board.I2C()
            self.ads = ADS1115(i2c, address=0x48)
            self.chan = AnalogIn(self.ads, ads1x15.Pin.A0)
            self.adc_active = True
            self.get_logger().info("ADC Hardware Online.")
        except Exception as e:
            self.get_logger().error(f"ADC Offline: {e}")
            self.adc_active = False

        # --- ROS2 COMMS ---
        self.lat_pub = self.create_publisher(Float32, '/robot_health/latency_ms', 10)
        self.batt_pub = self.create_publisher(Float32, '/robot_health/battery', 10)
        self.front_range_pub = self.create_publisher(Float32, '/robot_health/closest_front_range_m', 10)
        self.front_stop_distance_pub = self.create_publisher(Float32, '/robot_health/front_stop_distance_m', 10)
        self.forward_speed_pub = self.create_publisher(Float32, '/robot_health/front_forward_speed_mps', 10)
        self.front_obstacle_pub = self.create_publisher(Bool, '/robot_health/front_obstacle_active', 10)
        self.speed_limit_scale_pub = self.create_publisher(Float32, '/robot_health/front_speed_limit_scale', 10)
        self.mode_pub = self.create_publisher(String, '/robot_state/mode', 10)
        self.log_pub = self.create_publisher(String, '/robot_health/log', 10)
        self.nav_gate_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.safety_cmd_pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)

        self.create_subscription(Float64MultiArray, '/ping_t1', self.sync_callback, 10)
        self.create_subscription(String, '/ui/set_mode', self.handle_mode_change, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_joy', self.joy_cmd_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_nav_raw', self.nav_cmd_callback, 10)

        self.navigate_to_pose_cancel_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal'
        )
        self.navigate_through_poses_cancel_client = self.create_client(
            CancelGoal,
            '/navigate_through_poses/_action/cancel_goal'
        )

        self.create_timer(1.0, self.publish_battery)
        self.create_timer(1.0, self.broadcast_status)
        self.create_timer(1.0 / self.SAFETY_PUBLISH_HZ, self.publish_safety_hold)
        if self.safety_bypass_mode:
            self.send_log("Mode bypass enabled: Nav2 commands use lidar safety without UI mode gating.")

    def broadcast_status(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def get_bool_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def handle_mode_change(self, msg):
        if self.safety_bypass_mode:
            self.current_mode = "AUTO"
            self.send_log("Mode bypass active; ignoring UI mode command.")
            return

        cmd = msg.data.upper()
        if cmd == "RESET":
            self.safety_lock = False
            self.current_mode = "STOP"
            self.clear_cached_commands()
            self.nav_gate_pub.publish(Twist())
            self.publish_zero_twist()
            self.send_log("SAFETY LOCK DEACTIVATED. Ready for Mode Change.")
            return

        if cmd == "STOP":
            self.trigger_stop("UI Emergency Stop")
        elif not self.safety_lock:
            self.current_mode = cmd
            if cmd != "AUTO":
                self.nav_gate_pub.publish(Twist())
            self.send_log(f"Mode changed to: {cmd}")
        else:
            self.send_log("LOCK ACTIVE: Click 'DEACTIVATE STOP' first", is_crit=True)

    def trigger_stop(self, reason):
        self.current_mode = "STOP"
        self.safety_lock = True
        self.clear_cached_commands()
        self.send_log(f"EMERGENCY STOP: {reason}", is_crit=True)
        self.cancel_navigation_goals()
        self.nav_gate_pub.publish(Twist())
        self.publish_zero_twist()

    def send_log(self, text, is_crit=False):
        msg = String()
        msg.data = ("!!! " if is_crit else "> ") + text
        self.log_pub.publish(msg)

    def publish_zero_twist(self):
        self.safety_cmd_pub.publish(Twist())

    def copy_twist(self, cmd):
        copied = Twist()
        copied.linear.x = cmd.linear.x
        copied.linear.y = cmd.linear.y
        copied.linear.z = cmd.linear.z
        copied.angular.x = cmd.angular.x
        copied.angular.y = cmd.angular.y
        copied.angular.z = cmd.angular.z
        return copied

    def clear_cached_commands(self):
        self.latest_joy_cmd = Twist()
        self.latest_nav_cmd = Twist()
        self.latest_joy_time = None
        self.latest_nav_time = None

    def cancel_navigation_goals(self):
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0

        for name, client in (
            ('navigate_to_pose', self.navigate_to_pose_cancel_client),
            ('navigate_through_poses', self.navigate_through_poses_cancel_client),
        ):
            if not client.wait_for_service(timeout_sec=0.1):
                continue

            future = client.call_async(request)
            future.add_done_callback(
                lambda future, action_name=name: self._handle_cancel_response(action_name, future)
            )

    def _handle_cancel_response(self, action_name, future):
        try:
            response = future.result()
            self.send_log(
                f"Canceled {action_name} goals ({len(response.goals_canceling)} active goals matched)."
            )
        except Exception as exc:
            self.send_log(f"Failed to cancel {action_name} goals: {exc}", is_crit=True)

    def publish_safety_hold(self):
        if self.current_mode == "STOP":
            self.publish_zero_twist()
            self.speed_limit_scale_pub.publish(Float32(data=0.0))
            return

        if self.safety_lock:
            self.publish_zero_twist()
            self.speed_limit_scale_pub.publish(Float32(data=0.0))
            return

        active_cmd = self.get_active_command()
        if active_cmd is None:
            if self.current_mode == "AUTO":
                self.nav_gate_pub.publish(Twist())
            if self.has_active_motion_constraints():
                self.publish_zero_twist()
                self.speed_limit_scale_pub.publish(Float32(data=float(self.speed_limit_scale)))
                return
            self.speed_limit_scale_pub.publish(Float32(data=1.0))
            return

        constrained_cmd = self.apply_motion_constraints(
            active_cmd,
            auto_front_stop=(self.current_mode == "AUTO")
        )

        if self.has_active_motion_constraints():
            self.safety_cmd_pub.publish(constrained_cmd)

        self.speed_limit_scale_pub.publish(Float32(data=float(self.speed_limit_scale)))

    def joy_cmd_callback(self, msg):
        self.latest_joy_cmd = msg
        self.latest_joy_time = time.monotonic()

    def nav_cmd_callback(self, msg):
        self.latest_nav_cmd = msg
        self.latest_nav_time = time.monotonic()

        if self.current_mode == "AUTO" and not self.safety_lock:
            constrained_cmd = self.apply_motion_constraints(msg, auto_front_stop=True)
            self.nav_gate_pub.publish(constrained_cmd)
        else:
            self.nav_gate_pub.publish(Twist())

    def get_active_command(self):
        now = time.monotonic()
        joy_active = self.latest_joy_time is not None and (now - self.latest_joy_time) <= self.joystick_timeout_sec
        nav_active = self.latest_nav_time is not None and (now - self.latest_nav_time) <= self.nav_timeout_sec

        if self.current_mode == "MANUAL":
            return self.latest_joy_cmd if joy_active else None
        if self.current_mode == "AUTO":
            return self.latest_nav_cmd if nav_active else None
        return None

    def odom_callback(self, msg):
        self.forward_speed_mps = msg.twist.twist.linear.x

    def get_forward_speed_mps(self):
        cmd = self.get_active_command()
        cmd_speed = abs(cmd.linear.x) if cmd is not None else 0.0
        return max(0.0, abs(self.forward_speed_mps), cmd_speed)

    def get_dynamic_stop_distance(self):
        forward_speed = self.get_forward_speed_mps()
        max_distance = max(self.obstacle_stop_distance_m, self.obstacle_stop_distance_max_m)
        if self.obstacle_stop_speed_mps <= 1e-3 or max_distance <= self.obstacle_stop_distance_m:
            return self.obstacle_stop_distance_m, forward_speed

        speed_ratio = min(forward_speed / self.obstacle_stop_speed_mps, 1.0)
        stop_distance = self.obstacle_stop_distance_m + speed_ratio * (max_distance - self.obstacle_stop_distance_m)
        return stop_distance, forward_speed

    def has_active_motion_constraints(self):
        return (
            self.front_obstacle_active or
            self.left_obstacle_active or
            self.right_obstacle_active or
            self.rear_obstacle_active or
            self.speed_limit_scale < 1.0
        )

    def apply_motion_constraints(self, cmd, auto_front_stop=False):
        if auto_front_stop and self.front_obstacle_active:
            return Twist()

        limited = self.copy_twist(cmd)

        if self.front_obstacle_active and limited.linear.x > 0.0:
            limited.linear.x = 0.0
        elif self.speed_limit_scale < 1.0 and limited.linear.x > 0.0:
            limited.linear.x *= self.speed_limit_scale

        if self.rear_obstacle_active and limited.linear.x < 0.0:
            limited.linear.x = 0.0

        if self.left_obstacle_active and limited.angular.z > 0.0:
            limited.angular.z = 0.0

        if self.right_obstacle_active and limited.angular.z < 0.0:
            limited.angular.z = 0.0

        return limited

    def scan_callback(self, msg):
        if not self.obstacle_stop_enabled:
            return

        closest_forward_clearance = math.inf
        closest_left_clearance = math.inf
        closest_right_clearance = math.inf
        closest_rear_clearance = math.inf

        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + (index * msg.angle_increment)

            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            point_x = distance * math.cos(angle)
            point_y = distance * math.sin(angle)

            if point_x < self.front_stop_start_x_m:
                pass
            elif abs(point_y) <= self.front_stop_half_width_m:
                forward_clearance = point_x - self.front_stop_start_x_m
                closest_forward_clearance = min(closest_forward_clearance, forward_clearance)

            if point_x <= -self.rear_stop_start_x_m and abs(point_y) <= self.front_stop_half_width_m:
                rear_clearance = (-point_x) - self.rear_stop_start_x_m
                closest_rear_clearance = min(closest_rear_clearance, rear_clearance)

            if -self.rear_stop_start_x_m <= point_x <= self.front_stop_start_x_m:
                if point_y >= self.side_stop_start_y_m:
                    left_clearance = point_y - self.side_stop_start_y_m
                    closest_left_clearance = min(closest_left_clearance, left_clearance)
                elif point_y <= -self.side_stop_start_y_m:
                    right_clearance = (-point_y) - self.side_stop_start_y_m
                    closest_right_clearance = min(closest_right_clearance, right_clearance)

        self.closest_forward_clearance = closest_forward_clearance
        self.closest_left_clearance = closest_left_clearance
        self.closest_right_clearance = closest_right_clearance
        self.closest_rear_clearance = closest_rear_clearance
        dynamic_stop_distance, forward_speed = self.get_dynamic_stop_distance()
        slowdown_distance = dynamic_stop_distance + max(0.0, self.obstacle_slowdown_margin_m)
        obstacle_detected = closest_forward_clearance <= dynamic_stop_distance
        left_obstacle_detected = closest_left_clearance <= self.side_stop_distance_m
        right_obstacle_detected = closest_right_clearance <= self.side_stop_distance_m
        rear_obstacle_detected = closest_rear_clearance <= dynamic_stop_distance
        reported_range = closest_forward_clearance if math.isfinite(closest_forward_clearance) else -1.0
        if math.isfinite(closest_forward_clearance) and closest_forward_clearance < slowdown_distance:
            margin = max(self.obstacle_slowdown_margin_m, 1e-3)
            speed_limit_scale = max(
                0.0,
                min(1.0, (closest_forward_clearance - dynamic_stop_distance) / margin)
            )
        else:
            speed_limit_scale = 1.0

        self.front_range_pub.publish(Float32(data=float(reported_range)))
        self.front_stop_distance_pub.publish(Float32(data=float(dynamic_stop_distance)))
        self.forward_speed_pub.publish(Float32(data=float(forward_speed)))
        self.front_obstacle_pub.publish(Bool(data=obstacle_detected))
        self.dynamic_stop_distance_m = dynamic_stop_distance
        self.speed_limit_scale = 0.0 if obstacle_detected else speed_limit_scale

        if self.current_mode == "AUTO" and not self.safety_lock:
            if obstacle_detected:
                self.nav_gate_pub.publish(Twist())
                self.publish_zero_twist()
            else:
                constrained_cmd = self.apply_motion_constraints(self.latest_nav_cmd, auto_front_stop=True)
                self.nav_gate_pub.publish(constrained_cmd)

        if obstacle_detected and not self.front_obstacle_active:
            self.send_log(
                f"Front obstacle stop active ({closest_forward_clearance:.2f}m <= {dynamic_stop_distance:.2f}m at {forward_speed:.2f} m/s)",
                is_crit=True
            )
        elif self.front_obstacle_active and not obstacle_detected:
            self.send_log("Front obstacle cleared.")

        if left_obstacle_detected and not self.left_obstacle_active:
            self.send_log(
                f"Left turn blocked ({closest_left_clearance:.2f}m <= {self.side_stop_distance_m:.2f}m).",
                is_crit=True
            )
        elif self.left_obstacle_active and not left_obstacle_detected:
            self.send_log("Left side clear.")

        if right_obstacle_detected and not self.right_obstacle_active:
            self.send_log(
                f"Right turn blocked ({closest_right_clearance:.2f}m <= {self.side_stop_distance_m:.2f}m).",
                is_crit=True
            )
        elif self.right_obstacle_active and not right_obstacle_detected:
            self.send_log("Right side clear.")

        if rear_obstacle_detected and not self.rear_obstacle_active:
            self.send_log(
                f"Rear motion blocked ({closest_rear_clearance:.2f}m <= {dynamic_stop_distance:.2f}m).",
                is_crit=True
            )
        elif self.rear_obstacle_active and not rear_obstacle_detected:
            self.send_log("Rear area clear.")

        self.front_obstacle_active = obstacle_detected
        self.left_obstacle_active = left_obstacle_detected
        self.right_obstacle_active = right_obstacle_detected
        self.rear_obstacle_active = rear_obstacle_detected

    def sync_callback(self, msg):
        if not msg.data:
            self.send_log("Received empty /ping_t1 sample; ignoring.", is_crit=True)
            return

        t2 = time.time()
        t1 = msg.data[0]
        raw_diff = t2 - t1
        self.history.append(raw_diff)
        if len(self.history) > self.MAX_SAMPLES: self.history.pop(0)
        
        best_offset = min(self.history)
        latency_ms = (raw_diff - best_offset) * 1000
        
        if latency_ms > self.CRITICAL_LATENCY_MS:
            self.latency_fail_count += 1
            if (
                self.latency_fail_count >= 3 and
                self.current_mode != "STOP"
            ):
                self.trigger_stop(f"Persistent Latency ({latency_ms:.1f}ms)")
        else:
            self.latency_fail_count = 0
        self.lat_pub.publish(Float32(data=float(latency_ms)))

    def publish_battery(self):
        if not self.adc_active:
            self.batt_pub.publish(Float32(data=0.0))
            return

        volts = (self.chan.voltage * self.DIVIDER_RATIO)
        
        if volts < self.CRITICAL_VOLTAGE and not self.safety_lock:
            self.trigger_stop(f"Low Voltage ({volts:.2f}V)")

        # Interpolation Logic for S-Curve
        percent = 0.0
        if volts >= self.BATTERY_LUT[0][0]:
            percent = 100.0
        elif volts <= self.BATTERY_LUT[-1][0]:
            percent = 0.0
        else:
            for i in range(len(self.BATTERY_LUT) - 1):
                v_high, p_high = self.BATTERY_LUT[i]
                v_low, p_low = self.BATTERY_LUT[i+1]
                if v_high >= volts >= v_low:
                    # Linear interpolation between two LUT points
                    percent = p_low + (p_high - p_low) * ((volts - v_low) / (v_high - v_low))
                    break

        self.batt_pub.publish(Float32(data=float(percent)))

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = IntegrityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
