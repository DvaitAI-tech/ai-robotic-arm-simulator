import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json # Import the JSON library to handle structured data

class ArmCommandPublisher(Node):
    def __init__(self):
        # Renamed node for clarity (it's sending commands, not status)
        super().__init__('arm_command_publisher')
        
        # Publishing to the arm's command topic
        self.publisher = self.create_publisher(String, '/arm_command', 10)
        
        # Commands to cycle through, stored in a list
        # Using a list of dictionaries to represent the commands
        self.command_sequence = ["move up","move down", "pick", "move left","move right", "drop"]
        
        # Set a fast timer interval (e.g., 0.5 seconds) for "rapidly"
        self.command_index = 0
        self.timer = self.create_timer(0.5, self.publish_command) 
        
        self.get_logger().info("✅ Arm Command Publisher started and set to send commands every 0.5s")

    def publish_command(self):
        # 1. Get the current command dictionary
        current_command = self.command_sequence[self.command_index]
        
        # 2. Convert the dictionary into a JSON string
        # This is packed into the std_msgs.msg.String
        current_command
        
        msg = String()
        msg.data = current_command
        
        # 3. Publish the command
        self.publisher.publish(msg)
        self.get_logger().info(f"Published Command: '{msg.data}'")
        
        # 4. Move to the next command in the sequence
        self.command_index = (self.command_index + 1) % len(self.command_sequence)

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommandPublisher() # Use the new class name
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down command publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()