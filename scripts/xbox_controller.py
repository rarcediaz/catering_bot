#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import sys
import math


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

        # Motion settings
        self.MAX_LIN = 0.6   # steering
        self.MAX_ANG = 1.0   # forward/back
        self.DEADZONE = 0.2  # magnitude deadzone

        self.FLIP_LINEAR = False     # flip steering
        self.FLIP_ANGULAR = False    # flip forward/back

        self.timer = self.create_timer(0.1, self.update_and_publish)

        self.get_logger().info(f"--- XBOX CONTROL ACTIVE: {self.joy.get_name()} ---")
        self.get_logger().info("Directional control enabled (angle-based)")
        self.get_logger().info("B Button = Emergency Stop")

    def update_and_publish(self):
        pygame.event.pump()

        # Get joystick axes
        x = self.joy.get_axis(0)
        y = -self.joy.get_axis(1)  # invert so up is positive

        # Compute magnitude
        magnitude = math.sqrt(x**2 + y**2)

        msg = Twist()

        # Default stop
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        # Only move if outside deadzone
        if magnitude > self.DEADZONE:
            # Convert to angle (0–360)
            angle = math.degrees(math.atan2(y, x))
            if angle < 0:
                angle += 360

            # --- YOUR SECTOR MAPPING ---
            if 30 <= angle < 150:
                # UP → forward
                msg.angular.z = self.MAX_ANG
            elif 150 <= angle < 210:
                # LEFT
                msg.linear.x = self.MAX_LIN
            elif 210 <= angle < 330:
                # DOWN → reverse
                msg.angular.z = -self.MAX_ANG
            else:
                # RIGHT
                msg.linear.x = -self.MAX_LIN

            # Debug output
            self.get_logger().info(f"Angle: {angle:.1f} | Mag: {magnitude:.2f}")

        # --- FIX MOTOR ORIENTATION ---
        if self.FLIP_LINEAR:
            msg.linear.x *= -1
        if self.FLIP_ANGULAR:
            msg.angular.z *= -1

        # Emergency stop (B button)
        if self.joy.get_button(1):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().warn("!!! EMERGENCY STOP !!!")

        # Publish command
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