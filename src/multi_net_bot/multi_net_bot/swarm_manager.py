#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReliablePublisher:
    """
    A helper class that adds a TCP-like reliability mechanism over UDP, with support for multiple robots.
    It publishes commands with a sequence number and retransmits them until ACKs from all expected robots
    are received or maximum attempts are reached.
    """
    def __init__(self, node, publisher, ack_topic, expected_robot_ids=None, retransmission_timeout=1.0, max_attempts=3):
        self.node = node
        self.publisher = publisher
        self.retransmission_timeout = retransmission_timeout
        self.max_attempts = max_attempts

        self.current_seq = None
        self.current_message = None
        self.attempts = 0
        self.timer = None

        # Set of expected robot IDs that must send an ack.
        # If not provided, default to an empty set (i.e., no tracking)
        self.expected_robot_ids = set(expected_robot_ids) if expected_robot_ids is not None else set()
        # Set to track the IDs of robots that have already acknowledged the current message.
        self.current_acks = set()

        # Subscribe to the acknowledgment topic.
        self.ack_subscriber = node.create_subscription(
            String,
            ack_topic,
            self.ack_callback,
            10
        )

    def send_message(self, command):
        # Cancel any pending timer from a previous message.
        if self.timer is not None:
            self.timer.cancel()
        # When a new command arrives, drop any unsent/old commands.
        self.current_seq = (self.current_seq + 1) if self.current_seq is not None else 1
        self.current_message = f"seq:{self.current_seq};cmd:{command}"
        self.attempts = 0
        # Reset current ACKs for the new message.
        self.current_acks = set()
        self._transmit()

    def _transmit(self):
        if self.attempts >= self.max_attempts:
            self.node.get_logger().warn(
                f"Failed to deliver message {self.current_message} after {self.attempts} attempts."
            )
            self.current_message = None  # Drop the message.
            return

        self.attempts += 1
        self.node.get_logger().info(
            f"Transmitting message: {self.current_message} (Attempt {self.attempts}). Waiting for ACKs from: {self.expected_robot_ids - self.current_acks}"
        )
        self.publisher.publish(String(data=self.current_message))
        # Set a timer to call _on_timeout after the specified interval.
        self.timer = self.node.create_timer(self.retransmission_timeout, self._on_timeout)

    def _on_timeout(self):
        # Timer callback to retransmit the message if not all ACKs have been received.
        if self.timer is not None:
            self.timer.cancel()
        self._transmit()

    def ack_callback(self, msg):
        # Process the ACK message.
        ack = msg.data.strip()  # Remove any extra whitespace
        # We expect the ACK message to be in the form "ack:<sequence>;id:<robot_id>"
        if ack.startswith("ack:"):
            try:
                parts = ack.split(";")
                seq_part = parts[0]
                id_part = parts[1] if len(parts) > 1 else ""
                ack_seq = int(seq_part.split(":")[1])
                # Only consider ACKs for the current command sequence.
                if self.current_seq != ack_seq:
                    self.node.get_logger().info(f"Ignoring ACK for old seq {ack_seq}")
                    return

                # Parse the robot id. Expecting "id:<robot_id>"
                if id_part.startswith("id:"):
                    robot_id = id_part.split(":")[1]
                    self.current_acks.add(robot_id)
                    self.node.get_logger().info(f"Received ack from {robot_id} for message seq {ack_seq}")
                else:
                    self.node.get_logger().warn("Received ACK without robot ID")
                    return

                # Check if we have received ACKs from all expected robots.
                if self.expected_robot_ids and self.current_acks >= self.expected_robot_ids:
                    self.node.get_logger().info(f"All expected ACKs received for message seq {ack_seq}")
                    if self.timer is not None:
                        self.timer.cancel()
                    self.current_message = None  # Message successfully delivered.
                elif not self.expected_robot_ids:
                    # If no expected IDs are provided, treat one ACK as enough.
                    self.node.get_logger().info(f"Received ACK for message seq {ack_seq}")
                    if self.timer is not None:
                        self.timer.cancel()
                    self.current_message = None

            except (ValueError, IndexError):
                self.node.get_logger().warn("Received malformed ACK message")
        else:
            self.node.get_logger().info("Received non-ACK message: " + ack)

class SwarmManager(Node):
    """
    The Swarm Manager node publishes robot commands using a ReliablePublisher.
    This implementation now expects acknowledgements from multiple robots.
    """
    def __init__(self):
        super().__init__('swarm_manager')
        self.get_logger().info("Swarm Manager node started.")

        # Publisher for swarm commands.
        self.publisher = self.create_publisher(String, 'swarm_command', 10)
        # Define expected robot IDs (for example, "robot1", "robot2", "robot3").
        expected_robot_ids = {"robot1", "robot2", "robot3"}
        # Instantiate a ReliablePublisher that handles ACKs on the "swarm_ack" topic.
        self.reliable_publisher = ReliablePublisher(
            self, self.publisher, "swarm_ack", expected_robot_ids=expected_robot_ids, retransmission_timeout=1.0, max_attempts=3
        )
        # Timer to send commands periodically.
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # Subscribe to additional topics (e.g., raw robot status).
        self.status_subscriber = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10
        )

    def timer_callback(self):
        # In the real project, you might check game state information before sending commands.
        command = "Move forward"  # Example command; this could be dynamic.
        self.get_logger().info(f"Attempting to send command: {command}")
        self.reliable_publisher.send_message(command)

    def status_callback(self, msg):
        # Process status messages from robots.
        self.get_logger().info(f"Received status: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SwarmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
