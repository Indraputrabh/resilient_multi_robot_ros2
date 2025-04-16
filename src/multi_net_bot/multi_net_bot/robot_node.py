#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotNode(Node):
    def __init__(self, robot_id):
        super().__init__('robot_node')
        self.robot_id = robot_id

        # Subscription for receiving commands.
        self.create_subscription(String, 'swarm_command', self.command_callback, 10)
        # Publisher for ACKs.
        self.ack_publisher = self.create_publisher(String, 'swarm_ack', 10)
        # Publisher for status messages.
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)
        # Publisher for registration messages.
        self.registration_publisher = self.create_publisher(String, 'swarm_registration', 10)
        # Publisher for heartbeat messages.
        self.heartbeat_publisher = self.create_publisher(String, 'swarm_heartbeat', 10)

        # Send registration message on startup.
        self.send_registration_message()
        # Set up a periodic heartbeat timer.
        self.create_timer(1.0, self.send_heartbeat_message)

    def send_registration_message(self):
        msg = String()
        msg.data = f"register:{self.robot_id}"
        self.registration_publisher.publish(msg)
        self.get_logger().info(f"Sent registration: {msg.data}")

    def send_heartbeat_message(self):
        msg = String()
        msg.data = f"heartbeat:{self.robot_id}"
        self.heartbeat_publisher.publish(msg)
        self.get_logger().info(f"Sent heartbeat: {msg.data}")

    def command_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.data}")
        try:
            # Extract sequence number from the command message, expecting "seq:<seq>;cmd:<command>"
            seq = msg.data.split(";")[0].split(":")[1]
            # Construct the ACK message with the robot's unique id.
            ack_msg = String()
            ack_msg.data = f"ack:{seq};id:{self.robot_id}"
            self.ack_publisher.publish(ack_msg)
            self.get_logger().info(f"Published ACK: {ack_msg.data}")
        except Exception as e:
            self.get_logger().warn(f"Malformed command: {msg.data} (Error: {str(e)})")
        
        status_msg = String()
        status_msg.data = f"Status: OK from {self.robot_id}"
        self.status_publisher.publish(status_msg)

def main(args=None):
    import re
    rclpy.init(args=args)
    # Prompt the user for a valid robot id.
    while True:
        user_id = input("Enter robot id (in the format robot(x), where x is a number): ")
        # Validate the format: must start with "robot" and followed by one or more digits.
        if re.fullmatch(r"robot\d+", user_id):
            break
        else:
            print("Invalid format. Please enter the robot id in the format robot(x), e.g., robot1, robot2, etc.")
    node = RobotNode(robot_id=user_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Optional: send a deregistration message here if necessary.
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
