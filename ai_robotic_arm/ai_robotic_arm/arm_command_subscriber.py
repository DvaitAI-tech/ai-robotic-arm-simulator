# arm_command_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_robotic_arm.ai_controller import parse_command

class ArmCommandSubscriber(Node):
    def __init__(self):
        super().__init__('arm_command_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/arm_command',
            self.listener_callback,
            10)
        self.subscription
        self.get_logger().info('✅ Arm Command Subscriber Node Started')

    def listener_callback(self, msg):
        command = msg.data.strip()
        action = parse_command(command)
        self.get_logger().info(f'Received: "{command}" → {action}')
        # TODO: send the action to simulator (later through shared state or service)

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommandSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
