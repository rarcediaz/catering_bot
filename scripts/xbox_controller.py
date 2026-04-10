import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String 
import pygame
import sys
import math
import time


class XboxMover(Node):
    def __init__(self):
        super().__init__('xbox_teleop_node')

        # 1. Initialize Mode (Default to STOP for safety)
        self.current_mode = "MANUAL"

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_joy', 10)

        # 2. Subscribe to the mode topic from Mission Control
        self.mode_sub = self.create_subscription(
            String,
            '/robot_state/mode',
            self.mode_callback,
            10)

        # Initialize Pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Xbox Controller found! Plug it in and restart.")
            sys.exit()

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

        # Motion settings
        self.MAX_LIN = 0.18
        self.MAX_ANG = 0.30
        self.DEADZONE = 0.2
        self.RAMP_TIME_SEC = 0.25

        self.FLIP_LINEAR = False
        self.FLIP_ANGULAR = False

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_update_time = time.monotonic()

        self.timer = self.create_timer(0.1, self.update_and_publish)

        self.get_logger().info(f"--- XBOX CONTROL ACTIVE: {self.joy.get_name()} ---")
        self.get_logger().info("Status: Defaulting to STOP mode until MANUAL selected.")

    def mode_callback(self, msg: String):
        """Updates the internal mode whenever a new message is received."""
        new_mode = msg.data.strip().upper()
        if new_mode != self.current_mode:
            self.get_logger().info(f"Mode changed from {self.current_mode} to {new_mode}")
            self.current_mode = new_mode

    def _ramp_towards(self, current, target, max_delta):
        if target > current:
            return min(target, current + max_delta)
        return max(target, current - max_delta)

    def _apply_axis_ramp(self, current, target, max_delta):
        if target == 0.0:
            return 0.0
        if current != 0.0 and ((current > 0.0 and target < 0.0) or (current < 0.0 and target > 0.0)):
            return 0.0
        return self._ramp_towards(current, target, max_delta)

    def update_and_publish(self):
        pygame.event.pump()
        now = time.monotonic()
        dt = max(now - self.last_update_time, 1e-3)
        self.last_update_time = now

        msg = Twist()

        # Only publish joystick commands while MANUAL is the active mode.
        if self.current_mode != "MANUAL":
            self.current_linear = 0.0
            self.current_angular = 0.0
            return

        # --- Everything below only runs if mode is MANUAL ---
        x = self.joy.get_axis(0)
        y = -self.joy.get_axis(1)

        magnitude = math.sqrt(x**2 + y**2)
        target_linear = 0.0
        target_angular = 0.0

        if magnitude > self.DEADZONE:
            angle = math.degrees(math.atan2(y, x))
            if angle < 0:
                angle += 360

            if 45 <= angle < 135:
                target_linear = self.MAX_LIN
            elif 135 <= angle < 225:
                target_angular = self.MAX_ANG
            elif 225 <= angle < 315:
                target_linear = -self.MAX_LIN
            else:
                target_angular = -self.MAX_ANG

        if self.FLIP_LINEAR:
            target_linear *= -1
        if self.FLIP_ANGULAR:
            target_angular *= -1

        # Emergency stop (B button)
        if self.joy.get_button(1):
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.get_logger().warn("!!! EMERGENCY STOP !!!")
        else:
            lin_delta = (self.MAX_LIN / self.RAMP_TIME_SEC) * dt
            ang_delta = (self.MAX_ANG / self.RAMP_TIME_SEC) * dt
            self.current_linear = self._apply_axis_ramp(self.current_linear, target_linear, lin_delta)
            self.current_angular = self._apply_axis_ramp(self.current_angular, target_angular, ang_delta)

        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = XboxMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        rclpy.shutdown()


if __name__ == '__main__':
    main()