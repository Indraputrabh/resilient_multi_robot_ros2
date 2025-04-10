import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.create_subscription(String, 'swarm_command', self.command_callback, 10)
        self.ack_publisher = self.create_publisher(String, 'swarm_ack', 10)
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)
    
    def command_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.data}")
        # Extract sequence number
        try:
            seq = msg.data.split(";")[0].split(":")[1]
            ack_msg = String()
            ack_msg.data = f"ack:{seq}"
            self.ack_publisher.publish(ack_msg)
        except:
            self.get_logger().warn("Malformed command received")
        
        # Publish status for demonstration
        status_msg = String()
        status_msg.data = "Status: OK"
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
