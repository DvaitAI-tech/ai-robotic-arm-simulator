#!/usr/bin/env python3
"""
🤖 AIAutopilot Node
Publishes random commands to /arm_command until a stop signal is received on /ai_stop.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class AIAutopilot(Node):
    def __init__(self):
        super().__init__('ai_autopilot')

        # Publisher: send random movement/action commands
        self.publisher_ = self.create_publisher(String, '/arm_command', 10)

        # Subscriber: listen for stop command
        self.stop_sub = self.create_subscription(String, '/ai_stop', self.stop_callback, 10)

        # Timer: send a command every 3 seconds
        self.timer = self.create_timer(1.0, self.publish_command)

        # Internal state
        self.active = True
        self.commands = ["move up", "move down", "move left", "move right", "pick", "drop"]

        self.get_logger().info("🤖 AI Autopilot initialized — waiting for stop signal on /ai_stop")

    # ----------------------------------------------------------------
    # Timer Callback: Publish a random command
    # ----------------------------------------------------------------
    def publish_command(self):
        if not self.active:
            return  # Ignore if stopped

        cmd = random.choice(self.commands)
        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)
        self.get_logger().info(f"🧠 AI decided: {cmd}")

    # ----------------------------------------------------------------
    # Stop Callback: Called when a stop signal is received
    # ----------------------------------------------------------------
    def stop_callback(self, msg):
        """When dashboard publishes 'stop', halt AI autopilot."""
        if msg.data.strip().lower() == "stop":
            self.active = False
            self.get_logger().warn("🛑 Stop signal received — AI Autopilot stopping...")
            # Optional: destroy timer so it stops running
            if self.timer is not None:
                self.timer.cancel()
            # Graceful shutdown
            self.destroy_node()
            rclpy.shutdown()


# -------------------------------------------------------------------
# Entry Point
# -------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = AIAutopilot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🧩 AI Autopilot interrupted manually.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
