#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotNode(Node):
    def __init__(self, robot_id='robot1'):
        super().__init__('robot_node')
        # Unique identifier for the robot
        self.robot_id = robot_id

        # Subscribe to the 'swarm_command' topic to receive commands
        self.create_subscription(
            String,
            'swarm_command',
            self.command_callback,
            10
        )
        # Publisher to send acknowledgments on the 'swarm_ack' topic
        self.ack_publisher = self.create_publisher(String, 'swarm_ack', 10)
        # Publisher for robot status on the 'robot_status' topic
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)
    
    def command_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.data}")
        # Attempt to extract the sequence number from the command message.
        # Expected format: "seq:<sequence>;cmd:<command>"
        try:
            seq = msg.data.split(";")[0].split(":")[1]
            # Construct the ACK message including the robot's unique ID.
            ack_msg = String()
            ack_msg.data = f"ack:{seq};id:{self.robot_id}"
            self.ack_publisher.publish(ack_msg)
            self.get_logger().info(f"Published ACK: {ack_msg.data}")
        except Exception as e:
            self.get_logger().warn(f"Malformed command received: {msg.data} (Error: {str(e)})")
        
        # Publish a status update (for demonstration purposes).
        status_msg = String()
        status_msg.data = f"Status: OK from {self.robot_id}"
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    # Optionally, we can pass a different robot ID if needed.
    node = RobotNode(robot_id='robot1')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
