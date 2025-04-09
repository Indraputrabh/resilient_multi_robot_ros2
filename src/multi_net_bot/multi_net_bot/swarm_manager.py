#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReliablePublisher:
    """
    A helper class that adds a TCP-like reliability mechanism over UDP.
    It publishes commands with a sequence number and retransmits them until an ACK
    is received or maximum attempts are reached.
    """
    def __init__(self, node, publisher, ack_topic, retransmission_timeout=1.0, max_attempts=3):
        self.node = node
        self.publisher = publisher
        self.retransmission_timeout = retransmission_timeout
        self.max_attempts = max_attempts

        self.current_seq = None
        self.current_message = None
        self.attempts = 0
        self.timer = None

        # Subscribe to the acknowledgment topic.
        self.ack_subscriber = node.create_subscription(
            String,
            ack_topic,
            self.ack_callback,
            10
        )

    def send_message(self, command):
        # If a previous message is pending, cancel its timer.
        if self.timer is not None:
            self.timer.cancel()
        # When a new command arrives, drop any unsent/old commands.
        self.current_seq = (self.current_seq + 1) if self.current_seq is not None else 1
        self.current_message = f"seq:{self.current_seq};cmd:{command}"
        self.attempts = 0
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
            f"Transmitting message: {self.current_message} (Attempt {self.attempts})"
        )
        self.publisher.publish(String(data=self.current_message))
        # Set a timer to call _on_timeout after the specified interval.
        self.timer = self.node.create_timer(self.retransmission_timeout, self._on_timeout)

    def _on_timeout(self):
        # Timer callback to retransmit the message if no ACK was received.
        if self.timer is not None:
            self.timer.cancel()
        self._transmit()

    def ack_callback(self, msg):
        # Process the ACK message.
        ack = msg.data
        # We expect the ACK message to be in the form "ack:<sequence>"
        if ack.startswith("ack:"):
            try:
                ack_seq = int(ack.split(":")[1])
                if self.current_seq == ack_seq:
                    self.node.get_logger().info(f"Received ack for message seq {ack_seq}")
                    if self.timer is not None:
                        self.timer.cancel()
                    self.current_message = None  # Message successfully delivered.
            except ValueError:
                self.node.get_logger().warn("Received malformed ack message")
        else:
            self.node.get_logger().info("Received non-ack message: " + ack)

class SwarmManager(Node):
    """
    The Swarm Manager node publishes robot commands using a ReliablePublisher to
    simulate a TCP-like reliability layer over UDP. It also listens for status messages.
    """
    def __init__(self):
        super().__init__('swarm_manager')
        self.get_logger().info("Swarm Manager node started.")

        # Publisher for swarm commands.
        self.publisher = self.create_publisher(String, 'swarm_command', 10)
        # Instantiate a ReliablePublisher that handles ACKs on the "swarm_ack" topic.
        self.reliable_publisher = ReliablePublisher(
            self, self.publisher, "swarm_ack", retransmission_timeout=1.0, max_attempts=3
        )
        # Timer to send commands periodically.
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # Optionally, if you wish to subscribe to additional topics (like raw robot status), do so here.
        self.status_subscriber = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10
        )

    def timer_callback(self):
        # In your real project, this might check for new game state information
        # and decide whether to transmit a new command.
        command = "Move forward"  # Example command; this could be dynamic.
        self.get_logger().info(f"Attempting to send command: {command}")
        self.reliable_publisher.send_message(command)

    def status_callback(self, msg):
        # Process other status messages from robots.
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

