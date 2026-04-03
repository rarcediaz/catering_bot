#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math
import board
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_ads1x15.ads1x15 as ads1x15

class IntegrityNode(Node):
    def __init__(self):
        super().__init__('integrity_node')
        
        # --- CONFIGURATION ---
        self.DIVIDER_RATIO = 8.2727 
        self.CRITICAL_LATENCY_MS = 250.0
        self.CRITICAL_VOLTAGE = 21.0 # Updated to match your 0% threshold
        
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
        self.latency_fail_count = 0
        self.history = []
        self.MAX_SAMPLES = 20
        self.SAFETY_PUBLISH_HZ = 10.0

        self.declare_parameter('obstacle_stop_enabled', True)
        self.declare_parameter('obstacle_stop_distance_m', 0.10)
        self.declare_parameter('front_stop_half_angle_deg', 15.0)

        self.obstacle_stop_enabled = bool(self.get_parameter('obstacle_stop_enabled').value)
        self.obstacle_stop_distance_m = float(self.get_parameter('obstacle_stop_distance_m').value)
        self.front_stop_half_angle_deg = float(self.get_parameter('front_stop_half_angle_deg').value)
        self.front_stop_half_angle_rad = math.radians(self.front_stop_half_angle_deg)

        # --- ADC HARDWARE SETUP ---
        try:
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
        self.mode_pub = self.create_publisher(String, '/robot_state/mode', 10)
        self.log_pub = self.create_publisher(String, '/robot_health/log', 10)
        self.safety_cmd_pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)

        self.create_subscription(Float64MultiArray, '/ping_t1', self.sync_callback, 10)
        self.create_subscription(String, '/ui/set_mode', self.handle_mode_change, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.create_timer(1.0, self.publish_battery)
        self.create_timer(1.0, self.broadcast_status)
        self.create_timer(1.0 / self.SAFETY_PUBLISH_HZ, self.publish_safety_hold)

    def broadcast_status(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def handle_mode_change(self, msg):
        cmd = msg.data.upper()
        if cmd == "RESET":
            self.safety_lock = False
            self.current_mode = "STOP"
            self.send_log("SAFETY LOCK DEACTIVATED. Ready for Mode Change.")
            return

        if cmd == "STOP":
            self.trigger_stop("UI Emergency Stop")
        elif not self.safety_lock:
            self.current_mode = cmd
            self.send_log(f"Mode changed to: {cmd}")
        else:
            self.send_log("LOCK ACTIVE: Click 'DEACTIVATE STOP' first", is_crit=True)

    def trigger_stop(self, reason):
        self.current_mode = "STOP"
        self.safety_lock = True
        self.send_log(f"EMERGENCY STOP: {reason}", is_crit=True)
        self.publish_zero_twist()

    def send_log(self, text, is_crit=False):
        msg = String()
        msg.data = ("!!! " if is_crit else "> ") + text
        self.log_pub.publish(msg)

    def publish_zero_twist(self):
        self.safety_cmd_pub.publish(Twist())

    def publish_safety_hold(self):
        if self.safety_lock or self.front_obstacle_active:
            self.publish_zero_twist()

    def scan_callback(self, msg):
        if not self.obstacle_stop_enabled:
            return

        closest_front_range = math.inf

        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + (index * msg.angle_increment)

            if abs(angle) > self.front_stop_half_angle_rad:
                continue
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            closest_front_range = min(closest_front_range, distance)

        obstacle_detected = closest_front_range <= self.obstacle_stop_distance_m

        if obstacle_detected and not self.front_obstacle_active:
            self.send_log(
                f"Front obstacle stop active ({closest_front_range:.2f}m <= {self.obstacle_stop_distance_m:.2f}m)",
                is_crit=True
            )
            self.publish_zero_twist()
        elif self.front_obstacle_active and not obstacle_detected:
            self.send_log("Front obstacle cleared.")

        self.front_obstacle_active = obstacle_detected

    def sync_callback(self, msg):
        t2 = time.time()
        t1 = msg.data[0]
        raw_diff = t2 - t1
        self.history.append(raw_diff)
        if len(self.history) > self.MAX_SAMPLES: self.history.pop(0)
        
        best_offset = min(self.history)
        latency_ms = (raw_diff - best_offset) * 1000
        
        if latency_ms > self.CRITICAL_LATENCY_MS:
            self.latency_fail_count += 1
            if self.latency_fail_count >= 3 and self.current_mode != "STOP":
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

def main():
    rclpy.init()
    rclpy.spin(IntegrityNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()