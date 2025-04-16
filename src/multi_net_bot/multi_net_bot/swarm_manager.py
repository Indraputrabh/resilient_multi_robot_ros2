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

        # Expected robot IDs come from dynamic registration.
        self.expected_robot_ids = set(expected_robot_ids) if expected_robot_ids is not None else set()
        self.current_acks = set()

        # Subscribe to ACKs.
        self.ack_subscriber = node.create_subscription(
            String,
            ack_topic,
            self.ack_callback,
            10
        )

    def send_message(self, command):
        if self.timer is not None:
            self.timer.cancel()
        self.current_seq = (self.current_seq + 1) if self.current_seq is not None else 1
        self.current_message = f"seq:{self.current_seq};cmd:{command}"
        self.attempts = 0
        self.current_acks = set()  # Reset ACK tracking.
        self._transmit()

    def _transmit(self):
        if self.attempts >= self.max_attempts:
            self.node.get_logger().warn(
                f"Failed to deliver message {self.current_message} after {self.attempts} attempts."
            )
            self.current_message = None  # Drop the message.
            return

        self.attempts += 1
        missing = self.expected_robot_ids - self.current_acks
        self.node.get_logger().info(
            f"Transmitting message: {self.current_message} (Attempt {self.attempts}). Waiting for ACKs from: {missing}"
        )
        self.publisher.publish(String(data=self.current_message))
        self.timer = self.node.create_timer(self.retransmission_timeout, self._on_timeout)

    def _on_timeout(self):
        if self.timer is not None:
            self.timer.cancel()
        self._transmit()

    def ack_callback(self, msg):
        ack = msg.data.strip()
        if ack.startswith("ack:"):
            try:
                parts = ack.split(";")
                seq_part = parts[0]
                id_part = parts[1] if len(parts) > 1 else ""
                ack_seq = int(seq_part.split(":")[1])
                if self.current_seq != ack_seq:
                    self.node.get_logger().info(f"Ignoring ACK for old seq {ack_seq}")
                    return

                if id_part.startswith("id:"):
                    robot_id = id_part.split(":")[1]
                    self.current_acks.add(robot_id)
                    self.node.get_logger().info(f"Received ACK from {robot_id} for seq {ack_seq}")
                else:
                    self.node.get_logger().warn("Received ACK without robot id")
                    return

                if self.expected_robot_ids and self.current_acks >= self.expected_robot_ids:
                    self.node.get_logger().info(f"All expected ACKs received for seq {ack_seq}")
                    if self.timer is not None:
                        self.timer.cancel()
                    self.current_message = None
                elif not self.expected_robot_ids:
                    self.node.get_logger().info(f"Received ACK for seq {ack_seq}")
                    if self.timer is not None:
                        self.timer.cancel()
                    self.current_message = None

            except (ValueError, IndexError) as e:
                self.node.get_logger().warn(f"Received malformed ACK: {ack} (Error: {str(e)})")
        else:
            self.node.get_logger().info("Received non-ACK message: " + ack)

class SwarmManager(Node):
    """
    0.3-alpha - The Swarm Manager node now supports dynamic robot registration, heartbeat monitoring,
    and updates the list of expected robots for reliable messaging.
    """
    def __init__(self):
        super().__init__('swarm_manager')
        self.get_logger().info("Swarm Manager node started.")

        # Publisher for commands.
        self.publisher = self.create_publisher(String, 'swarm_command', 10)

        # Dynamic tracking of active robots: robot_id -> last heartbeat (in seconds)
        self.active_robots = {}
        self.heartbeat_timeout = 5.0  # seconds

        # Subscriptions for registration, heartbeat, and deregistration.
        self.create_subscription(String, 'swarm_registration', self.registration_callback, 10)
        self.create_subscription(String, 'swarm_heartbeat', self.heartbeat_callback, 10)
        self.create_subscription(String, 'swarm_deregistration', self.deregistration_callback, 10)

        # Timer to check for heartbeat timeouts.
        self.create_timer(1.0, self.check_heartbeat_timeout)

        # Instantiate ReliablePublisher with an initially empty expected robot set.
        self.reliable_publisher = ReliablePublisher(
            self, self.publisher, "swarm_ack", expected_robot_ids=set(), retransmission_timeout=1.0, max_attempts=3
        )

        # Timer to periodically send a command.
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Subscribe to robot status messages.
        self.create_subscription(String, 'robot_status', self.status_callback, 10)

    def registration_callback(self, msg):
        # Expected format: "register:<robot_id>"
        try:
            data = msg.data.strip()
            if data.startswith("register:"):
                robot_id = data.split(":")[1]
                self.active_robots[robot_id] = self.get_clock().now().nanoseconds / 1e9  # use seconds
                self.get_logger().info(f"Registered robot: {robot_id}")
                self.update_expected_robot_ids()
            else:
                self.get_logger().warn(f"Invalid registration format: {data}")
        except Exception as e:
            self.get_logger().warn(f"Error in registration callback: {str(e)}")

    def heartbeat_callback(self, msg):
        # Expected format: "heartbeat:<robot_id>"
        try:
            data = msg.data.strip()
            if data.startswith("heartbeat:"):
                robot_id = data.split(":")[1]
                self.active_robots[robot_id] = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info(f"Heartbeat received from: {robot_id}")
            else:
                self.get_logger().warn(f"Invalid heartbeat format: {data}")
        except Exception as e:
            self.get_logger().warn(f"Error in heartbeat callback: {str(e)}")

    def deregistration_callback(self, msg):
        # Expected format: "deregister:<robot_id>"
        try:
            data = msg.data.strip()
            if data.startswith("deregister:"):
                robot_id = data.split(":")[1]
                if robot_id in self.active_robots:
                    del self.active_robots[robot_id]
                    self.get_logger().info(f"Deregistered robot: {robot_id}")
                    self.update_expected_robot_ids()
            else:
                self.get_logger().warn(f"Invalid deregistration format: {data}")
        except Exception as e:
            self.get_logger().warn(f"Error in deregistration callback: {str(e)}")

    def check_heartbeat_timeout(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        to_remove = []
        for robot_id, last_seen in self.active_robots.items():
            if current_time - last_seen > self.heartbeat_timeout:
                to_remove.append(robot_id)
        for robot_id in to_remove:
            del self.active_robots[robot_id]
            self.get_logger().warn(f"Robot {robot_id} timed out due to missed heartbeat.")
        if to_remove:
            self.update_expected_robot_ids()

    def update_expected_robot_ids(self):
        # Update the expected robots in the ReliablePublisher.
        self.reliable_publisher.expected_robot_ids = set(self.active_robots.keys())
        self.get_logger().info(f"Updated expected robot IDs: {self.reliable_publisher.expected_robot_ids}")

    def timer_callback(self):
        command = "Move forward"
        self.get_logger().info(f"Attempting to send command: {command}")
        self.reliable_publisher.send_message(command)

    def status_callback(self, msg):
        self.get_logger().info(f"Status update: {msg.data}")

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
