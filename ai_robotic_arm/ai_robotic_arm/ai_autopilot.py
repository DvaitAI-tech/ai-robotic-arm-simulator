import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class AIAutopilot(Node):
    def __init__(self):
        super().__init__('ai_autopilot')
        self.publisher_ = self.create_publisher(String, '/arm_command', 10)
        self.timer = self.create_timer(3.0, self.publish_command)
        self.commands = ["move up", "move down", "move left", "move right", "pick", "drop"]
        self.get_logger().info("AI Autopilot initialized — ready to take control.")

    def publish_command(self):
        cmd = random.choice(self.commands)
        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)
        self.get_logger().info(f"AI decided: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = AIAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("AI Autopilot stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
