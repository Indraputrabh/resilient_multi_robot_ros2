#!/usr/bin/env python3
# swarm_manager.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque

# -----------------------------------------------------------------------------
# ReliablePublisher: provides TCP-like reliability on top of ROS2 topics.
# - Configurable ACK threshold (absolute or percentage).
# - Selective retransmission: only resend to robots that still need it.
# - Exponential backoff with dynamic scaling based on missing ACKs.
# - History buffer to keep track of recent messages (for future replay or debugging).
# -----------------------------------------------------------------------------
class ReliablePublisher:
    def __init__(self,
                 node,
                 publisher,
                 ack_topic,
                 expected_robot_ids=None,
                 base_timeout=1.0,
                 max_attempts=5,
                 backoff_factor=2.0,
                 ack_threshold=1.0,  # fraction (0.0-1.0) or absolute count
                 history_size=10):
        # Store references and parameters
        self.node = node
        self.pub = publisher
        self.ack_sub = node.create_subscription(
            String, ack_topic, self._ack_callback, 10
        )
        # Timing and retry settings
        self.base_timeout = base_timeout
        self.max_attempts = max_attempts
        self.backoff_factor = backoff_factor
        # ACK policy: can be int (e.g. 3 robots) or float (e.g. 0.8 => 80%)
        self.ack_threshold = ack_threshold
        # Track which robots we expect ACKs from
        self.expected = set(expected_robot_ids or ())
        # State for the current message
        self.current_acks = set()
        self.seq = 0
        self.history = deque(maxlen=history_size)
        self.attempt = 0
        self._pending = None
        self._timer = None

    def send_message(self, command):
        # Cancel any running timer before starting a new send
        if self._timer:
            self._timer.cancel()
        # Increment sequence and prepare payload
        self.seq += 1
        self._pending = f"seq:{self.seq};cmd:{command}"
        # Record in history for debugging or replay
        self.history.append((self.seq, self._pending))
        # Reset ACK tracking and attempt count
        self.current_acks.clear()
        self.attempt = 0
        # Kick off the first transmission
        self._transmit()

    def _compute_timeout(self):
        # Compute dynamic timeout: base × backoff^attempt × (1 + missing/total)
        missing = len(self.expected) - len(self.current_acks)
        scale = 1.0 + (missing / len(self.expected)) if self.expected else 1.0
        return self.base_timeout * (self.backoff_factor ** self.attempt) * scale

    def _transmit(self):
        # Check if we've exhausted retry attempts
        if self.attempt >= self.max_attempts:
            self.node.get_logger().warn(
                f"Give up on seq {self.seq} after {self.attempt} attempts; "
                f"ACKs={len(self.current_acks)}/{len(self.expected)}"
            )
            # Drop pending message
            self._pending = None
            return

        # Increase attempt count
        self.attempt += 1
        # Determine which robots still need the message
        missing = self.expected - self.current_acks
        # If some robots already ACKed, send only to missing ones
        if missing and missing != self.expected:
            to_field = ",".join(sorted(missing))
            payload = f"{self._pending};to:{to_field}"
        else:
            # Broadcast to all if no selective list
            payload = self._pending

        # Log and publish
        self.node.get_logger().info(
            f"[Attempt {self.attempt}] pub → '{payload}' waiting on {missing}"
        )
        self.pub.publish(String(data=payload))
        # Schedule next retry using dynamic timeout
        timeout = self._compute_timeout()
        self._timer = self.node.create_timer(timeout, self._on_timeout)

    def _on_timeout(self):
        # Cancel previous timer and retry
        if self._timer:
            self._timer.cancel()
        self._transmit()

    def _ack_callback(self, msg):
        data = msg.data.strip()
        if not data.startswith("ack:"):
            return  # ignore non-ACKs
        try:
            # Parse semicolon-separated key:value pairs
            parts = dict(p.split(":", 1) for p in data.split(";") if ":" in p)
            ack_seq = int(parts.get("ack", -1))
            sender = parts.get("id")
        except Exception:
            self.node.get_logger().warn(f"Malformed ACK → {data}")
            return

        # Only care about ACKs for the current sequence
        if ack_seq != self.seq:
            self.node.get_logger().info(f"Ignoring ACK for seq {ack_seq}")
            return

        # Record ACK and log progress
        if sender:
            self.current_acks.add(sender)
            self.node.get_logger().info(
                f"ACK from {sender} ({len(self.current_acks)}/{len(self.expected)})"
            )

        # Determine how many ACKs we need
        if isinstance(self.ack_threshold, int):
            required = self.ack_threshold
        else:
            # percentage of expected robots, at least 1
            required = max(1, int(self.ack_threshold * len(self.expected)))

        # If we've met the threshold, finish up
        if len(self.current_acks) >= required:
            self.node.get_logger().info(
                f"Threshold met: {len(self.current_acks)}/{len(self.expected)}"
            )
            if self._timer:
                self._timer.cancel()
            self._pending = None


# -----------------------------------------------------------------------------
# SwarmManager: tracks active robots and sends commands via ReliablePublisher
# -----------------------------------------------------------------------------
class SwarmManager(Node):
    def __init__(self):
        super().__init__('swarm_manager')
        self.get_logger().info("Swarm Manager node started.")

        # Publisher for sending commands to the swarm
        self.publisher = self.create_publisher(String, 'swarm_command', 10)

        # Track active robots: {robot_id: last_heartbeat_timestamp}
        self.active_robots = {}
        self.heartbeat_timeout = 5.0  # seconds

        # Subscriptions for registration, heartbeat, deregistration, and status
        self.create_subscription(String, 'swarm_registration', self._reg_cb, 10)
        self.create_subscription(String, 'swarm_heartbeat', self._hb_cb, 10)
        self.create_subscription(String, 'swarm_deregistration', self._dreg_cb, 10)
        self.create_subscription(String, 'robot_status', self._status_cb, 10)

        # Periodic timer to purge stale robots
        self.create_timer(1.0, self._check_heartbeat)

        # Initialize ReliablePublisher with desired settings
        self.reliable_publisher = ReliablePublisher(
            node=self,
            publisher=self.publisher,
            ack_topic='swarm_ack',
            expected_robot_ids=set(),
            base_timeout=1.0,
            max_attempts=5,
            backoff_factor=2.0,
            ack_threshold=0.8,  # require 80% ACKs
            history_size=20
        )

        # Timer to send a test command periodically
        self.create_timer(2.0, self._timer_cb)

    def _reg_cb(self, msg):
        # Handle "register:<robot_id>" messages
        data = msg.data.strip()
        if data.startswith("register:"):
            rid = data.split(":", 1)[1]
            # Record registration timestamp
            self.active_robots[rid] = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"Registered {rid}")
            self._update_expected()
        else:
            self.get_logger().warn(f"Bad registration: {data}")

    def _hb_cb(self, msg):
        # Handle "heartbeat:<robot_id>" messages
        data = msg.data.strip()
        if data.startswith("heartbeat:"):
            rid = data.split(":", 1)[1]
            self.active_robots[rid] = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"Heartbeat from {rid}")
        else:
            self.get_logger().warn(f"Bad heartbeat: {data}")

    def _dreg_cb(self, msg):
        # Handle "deregister:<robot_id>" messages
        data = msg.data.strip()
        if data.startswith("deregister:"):
            rid = data.split(":", 1)[1]
            self.active_robots.pop(rid, None)
            self.get_logger().info(f"Deregistered {rid}")
            self._update_expected()

    def _status_cb(self, msg):
        # Log robot status updates for monitoring
        self.get_logger().info(f"Status: {msg.data}")

    def _check_heartbeat(self):
        # Periodically remove robots that haven't sent a heartbeat in time
        now = self.get_clock().now().nanoseconds / 1e9
        removed = []
        for rid, ts in list(self.active_robots.items()):
            if now - ts > self.heartbeat_timeout:
                removed.append(rid)
                self.active_robots.pop(rid)
                self.get_logger().warn(f"Timeout {rid}")
        if removed:
            self._update_expected()

    def _update_expected(self):
        # Refresh the expected IDs in ReliablePublisher based on active robots
        self.reliable_publisher.expected = set(self.active_robots.keys())
        self.get_logger().info(
            f"Expecting ACKs from: {self.reliable_publisher.expected}"
        )

    def _timer_cb(self):
        # Periodic test command; replace with your own logic as needed
        cmd = "Move forward"
        self.get_logger().info(f"Sending: {cmd}")
        self.reliable_publisher.send_message(cmd)
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
