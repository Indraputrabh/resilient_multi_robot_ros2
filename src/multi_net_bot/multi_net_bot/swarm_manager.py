#!/usr/bin/env python3
# swarm_manager.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque

# -----------------------------------------------------------------------------
# ReliablePublisher: provides TCP‑like reliability on top of ROS2 topics.
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
                 ack_threshold=1.0,   # fraction (0.0–1.0) or absolute count
                 history_size=10):
        # Store references and parameters
        self.node = node
        self.pub = publisher
        self.ack_sub = node.create_subscription(
            String, ack_topic, self._ack_callback, 10
        )
        # Timing & retry settings
        self.base_timeout = base_timeout
        self.max_attempts = max_attempts
        self.backoff_factor = backoff_factor
        # ACK policy: int for count or float for percentage
        self.ack_threshold = ack_threshold
        # Track expected ACK senders
        self.expected = set(expected_robot_ids or ())
        # State for current message
        self.current_acks = set()
        self.seq = 0
        self.history = deque(maxlen=history_size)
        self.attempt = 0
        self._pending = None
        self._timer = None

    def send_message(self, command):
        # Cancel any prior timer
        if self._timer:
            self._timer.cancel()
        # Prepare new message
        self.seq += 1
        self._pending = f"seq:{self.seq};cmd:{command}"
        self.history.append((self.seq, self._pending))
        self.current_acks.clear()
        self.attempt = 0
        # Start retransmission cycle
        self._transmit()

    def _compute_timeout(self):
        # Dynamic timeout = base × backoff^attempt × (1 + missing/total)
        missing = len(self.expected) - len(self.current_acks)
        scale = 1.0 + (missing / len(self.expected)) if self.expected else 1.0
        return self.base_timeout * (self.backoff_factor ** self.attempt) * scale

    def _transmit(self):
        # If out of attempts, give up
        if self.attempt >= self.max_attempts:
            self.node.get_logger().warn(
                f"Give up on seq {self.seq} after {self.attempt} attempts; "
                f"ACKs={len(self.current_acks)}/{len(self.expected)}"
            )
            self._pending = None
            return

        self.attempt += 1
        missing = self.expected - self.current_acks

        # Selective retransmit: include only missing robots
        if missing and missing != self.expected:
            to_field = ",".join(sorted(missing))
            payload = f"{self._pending};to:{to_field}"
        else:
            # Broadcast to all
            payload = self._pending

        # Publish and schedule next timeout
        self.node.get_logger().info(
            f"[Attempt {self.attempt}] pub → '{payload}' waiting on {missing}"
        )
        self.pub.publish(String(data=payload))
        timeout = self._compute_timeout()
        self._timer = self.node.create_timer(timeout, self._on_timeout)

    def _on_timeout(self):
        # Retry on timeout
        if self._timer:
            self._timer.cancel()
        self._transmit()

    def _ack_callback(self, msg):
        # Handle incoming ACKs
        data = msg.data.strip()
        if not data.startswith("ack:"):
            return
        try:
            parts = dict(p.split(":", 1) for p in data.split(";") if ":" in p)
            ack_seq = int(parts.get("ack", -1))
            sender = parts.get("id")
        except Exception:
            self.node.get_logger().warn(f"Malformed ACK → {data}")
            return

        # Ignore out‑of‑date ACKs
        if ack_seq != self.seq:
            return

        # Record ACK and check threshold
        if sender:
            self.current_acks.add(sender)
            self.node.get_logger().info(
                f"ACK from {sender} ({len(self.current_acks)}/{len(self.expected)})"
            )

        if isinstance(self.ack_threshold, int):
            required = self.ack_threshold
        else:
            required = max(1, int(self.ack_threshold * len(self.expected)))

        if len(self.current_acks) >= required:
            self.node.get_logger().info(
                f"Threshold met: {len(self.current_acks)}/{len(self.expected)}"
            )
            if self._timer:
                self._timer.cancel()
            self._pending = None


# -----------------------------------------------------------------------------
# SwarmManager: tracks active robots, reacts to role changes, and sends
# role‑specific commands via ReliablePublisher.
# -----------------------------------------------------------------------------
class SwarmManager(Node):
    def __init__(self):
        super().__init__('swarm_manager')
        self.get_logger().info("Swarm Manager node started.")

        # Track active robots and their roles
        self.active_robots = {}    # robot_id -> last heartbeat timestamp
        self.robot_roles = {}      # robot_id -> current role
        self.heartbeat_timeout = 5.0

        # Subscriptions for lifecycle
        self.create_subscription(String, 'swarm_registration',   self._reg_cb,    10)
        self.create_subscription(String, 'swarm_heartbeat',      self._hb_cb,     10)
        self.create_subscription(String, 'swarm_deregistration', self._dreg_cb,   10)
        self.create_subscription(String, 'robot_status',         self._status_cb, 10)

        # Subscription for role updates
        self.create_subscription(String, 'swarm_roles',          self._role_cb,   10)

        # Periodic timer to purge stale robots
        self.create_timer(1.0, self._check_heartbeat)

        # Publisher & reliable layer
        self.publisher = self.create_publisher(String, 'swarm_command', 10)
        self.reliable_publisher = ReliablePublisher(
            node=self,
            publisher=self.publisher,
            ack_topic='swarm_ack',
            expected_robot_ids=set(),
            base_timeout=1.0,
            max_attempts=5,
            backoff_factor=2.0,
            ack_threshold=0.8,   # require 80% ACKs
            history_size=20
        )

        # Timer to dispatch role‑specific commands
        self.create_timer(2.0, self._role_timer_cb)

    # -------------------
    # Robot lifecycle callbacks
    # -------------------
    def _reg_cb(self, msg):
        rid = msg.data.split(":", 1)[1]
        self.active_robots[rid] = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Registered {rid}")
        self._update_expected()

    def _hb_cb(self, msg):
        rid = msg.data.split(":", 1)[1]
        self.active_robots[rid] = self.get_clock().now().nanoseconds / 1e9

    def _dreg_cb(self, msg):
        rid = msg.data.split(":", 1)[1]
        self.active_robots.pop(rid, None)
        self.robot_roles.pop(rid, None)
        self.get_logger().info(f"Deregistered {rid}")
        self._update_expected()

    def _status_cb(self, msg):
        self.get_logger().info(f"Status: {msg.data}")

    def _check_heartbeat(self):
        now = self.get_clock().now().nanoseconds / 1e9
        removed = []
        for rid, ts in list(self.active_robots.items()):
            if now - ts > self.heartbeat_timeout:
                removed.append(rid)
        for rid in removed:
            del self.active_robots[rid]
            self.robot_roles.pop(rid, None)
            self.get_logger().warn(f"Timeout {rid}")
        if removed:
            self._update_expected()

    def _update_expected(self):
        self.reliable_publisher.expected = set(self.active_robots.keys())
        self.get_logger().info(
            f"Expecting ACKs from: {self.reliable_publisher.expected}"
        )

    # -------------------
    # Role updates callback
    # -------------------
    def _role_cb(self, msg):
        parts = dict(p.split(":",1) for p in msg.data.split(";") if ":" in p)
        rid  = parts.get("assign")
        role = parts.get("role")
        if rid and role:
            self.robot_roles[rid] = role
            self.get_logger().info(f"Role update: {rid} → {role}")

    # -------------------
    # Dispatch role‑specific commands
    # -------------------
    def _role_timer_cb(self):
        role_cmds = {
            'striker':    'Move forward aggressively',
            'defender':   'Move forward defensively',
            'goalkeeper': 'Hold position',
        }
        for role, cmd in role_cmds.items():
            recipients = {rid for rid, r in self.robot_roles.items() if r == role}
            if not recipients:
                continue
            self.reliable_publisher.expected = recipients
            self.get_logger().info(f"Sending '{cmd}' to {role}s: {recipients}")
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
