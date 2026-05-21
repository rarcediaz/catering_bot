import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import sys
import math
import time


class XboxMover(Node):
    def __init__(self):
        super().__init__('xbox_teleop_node')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_joy', 10)

        # Initialize Pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Xbox Controller found! Plug it in and restart.")
            sys.exit()

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

        # Motion settings. Default to a localization-safe profile and allow
        # a held turbo button for higher speed when the operator needs it.
        self.MAX_LIN = 0.30
        self.MAX_ANG = 0.28
        self.TURBO_MAX_LIN = 0.70
        self.TURBO_MAX_ANG = 1.00
        self.DEADZONE = 0.2
        self.RAMP_TIME_SEC = 0.20

        self.FLIP_LINEAR = False
        self.FLIP_ANGULAR = False

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_update_time = time.monotonic()

        self.timer = self.create_timer(0.02, self.update_and_publish)

        self.get_logger().info(f"--- XBOX CONTROL ACTIVE: {self.joy.get_name()} ---")

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

    def _shape_axis(self, raw_value):
        magnitude = abs(raw_value)
        if magnitude <= self.DEADZONE:
            return 0.0

        normalized = (magnitude - self.DEADZONE) / (1.0 - self.DEADZONE)
        shaped = normalized * normalized
        return math.copysign(shaped, raw_value)

    def update_and_publish(self):
        pygame.event.pump()
        now = time.monotonic()
        dt = max(now - self.last_update_time, 1e-3)
        self.last_update_time = now

        msg = Twist()

        raw_angular = -self.joy.get_axis(0)
        raw_linear = -self.joy.get_axis(1)

        turbo_enabled = bool(self.joy.get_button(5))
        max_lin = self.TURBO_MAX_LIN if turbo_enabled else self.MAX_LIN
        max_ang = self.TURBO_MAX_ANG if turbo_enabled else self.MAX_ANG

        target_linear = self._shape_axis(raw_linear) * max_lin
        target_angular = self._shape_axis(raw_angular) * max_ang

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
            lin_delta = (max_lin / self.RAMP_TIME_SEC) * dt
            ang_delta = (max_ang / self.RAMP_TIME_SEC) * dt
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
