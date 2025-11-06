#!/usr/bin/env python3
"""
ROS 2 Node: ArmController
Controls a 2-link robotic arm simulator via /arm_command topic
and publishes status updates on /arm_status.
"""

import math
import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_robotic_arm.ai_controller import parse_command
import csv
import time

class ArmController(Node):
    """ROS 2 + Pygame arm controller."""

    def __init__(self):
        super().__init__("arm_controller")

        # ROS interfaces
        self.sub_cmd = self.create_subscription(String, "/arm_command", self._on_command, 10)
        self.pub_status = self.create_publisher(String, "/arm_status", 10)

        # Simulator setup
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("ROS 2 Controlled AI Robotic Arm")
        self.clock = pygame.time.Clock()
        self.angle_speed      = 2
        # Arm geometry
        self.base_x, self.base_y = 400, 400
        self.l1, self.l2 = 150.0, 100.0
        self.angle1, self.angle2 = 45.0, 45.0
        self.running = True
        self.log_file = open('arm_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(["timestamp", "command", "angle1", "angle2"])


        self.get_logger().info("✅ ArmController node started — awaiting /arm_command messages.")

    # ---------------------------------------------------------------------
    # ROS callback
    # ---------------------------------------------------------------------
    def _on_command(self, msg: String) -> None:
        """Process incoming /arm_command messages."""
        cmd = msg.data.strip().lower()
        parsed = parse_command(cmd)
        self.csv_writer.writerow([time.time(), cmd, self.angle1, self.angle2])
        self.log_file.flush()

        if not parsed:
            status = f"Unknown command: {cmd}"
            self.pub_status.publish(String(data=status))
            self.get_logger().warn(status)
            return

        key, val = parsed
        if key == "angle1":
            self.angle1 += float(val)
        elif key == "angle2":
            self.angle2 += float(val)
        elif key == "action":
            self.get_logger().info(f"Performing action: {val}")

        status = f"Executed: {cmd} | A1={self.angle1:.1f}°, A2={self.angle2:.1f}°"
        self.pub_status.publish(String(data=status))
        self.get_logger().info(status)

    # ---------------------------------------------------------------------
    # Drawing
    # ---------------------------------------------------------------------
    def _draw_arm(self) -> None:
        """Draw current arm configuration in Pygame window."""
        BLACK, WHITE, BLUE = (0, 0, 0), (255, 255, 255), (50, 150, 255)
        self.screen.fill(BLACK)

        # Forward kinematics
        jx = self.base_x + self.l1 * math.cos(math.radians(self.angle1))
        jy = self.base_y - self.l1 * math.sin(math.radians(self.angle1))
        ex = jx + self.l2 * math.cos(math.radians(self.angle1 + self.angle2))
        ey = jy - self.l2 * math.sin(math.radians(self.angle1 + self.angle2))

        # Draw
        pygame.draw.line(self.screen, BLUE, (self.base_x, self.base_y), (jx, jy), 8)
        pygame.draw.line(self.screen, BLUE, (jx, jy), (ex, ey), 6)
        pygame.draw.circle(self.screen, WHITE, (int(ex), int(ey)), 8)
        pygame.display.flip()

    # ---------------------------------------------------------------------
    # Main loop
    # ---------------------------------------------------------------------
    def spin(self) -> None:
        """Main simulation + ROS spin loop."""
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            # 5️⃣ Manual Control (Optional)
            keys = pygame.key.get_pressed()
            if keys[pygame.K_a]: self.angle1 -= self.angle_speed
            if keys[pygame.K_d]: self.angle1 += self.angle_speed
            if keys[pygame.K_w]: self.angle2 -= self.angle_speed
            if keys[pygame.K_s]: self.angle2 += self.angle_speed

            # 6️⃣ Simple Auto Demo Mode (Press SPACE to toggle)
            if keys[pygame.K_SPACE]:
                self.angle1 += 1.5 * math.sin(pygame.time.get_ticks() * 0.002)
                self.angle2 += 1.5 * math.cos(pygame.time.get_ticks() * 0.002)


            # Handle ROS callbacks without blocking
            rclpy.spin_once(self, timeout_sec=0)

            # Update visualization
            self._draw_arm()

            # Limit FPS
            self.clock.tick(30)

        # Graceful exit
        pygame.quit()
        self.get_logger().info("ArmController shut down cleanly.")


# -------------------------------------------------------------------------
# Entry point
# -------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        node.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted — shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
