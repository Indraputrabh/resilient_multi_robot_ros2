#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# RobotNode: listens for commands, enforces ordering/selective delivery,
# and replies with ACKs and status updates.
class RobotNode(Node):
    def __init__(self, robot_id):
        super().__init__('robot_node')
        self.robot_id = robot_id
        # Track last seen sequence to drop duplicates or old messages
        self.last_seq = 0

        # Subscriptions and publishers
        self.create_subscription(String, 'swarm_command', self._cmd_cb, 10)
        self.ack_pub = self.create_publisher(String, 'swarm_ack', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.reg_pub = self.create_publisher(String, 'swarm_registration', 10)
        self.hb_pub = self.create_publisher(String, 'swarm_heartbeat', 10)

        # Announce ourselves and start heartbeats
        self._send_registration()
        self.create_timer(1.0, self._send_heartbeat)

    def _send_registration(self):
        msg = String(data=f"register:{self.robot_id}")
        self.reg_pub.publish(msg)
        self.get_logger().info(f"Reg→ {msg.data}")

    def _send_heartbeat(self):
        msg = String(data=f"heartbeat:{self.robot_id}")
        self.hb_pub.publish(msg)
        self.get_logger().info(f"HB→ {msg.data}")

    def _cmd_cb(self, msg):
        # Parse incoming command: seq, cmd, optional to-list
        parts = dict(p.split(":",1) for p in msg.data.split(";") if ":" in p)
        seq = int(parts.get("seq", 0))
        to_list = parts.get("to")
        cmd = parts.get("cmd")

        # Selective delivery: ignore if not addressed to me
        if to_list and self.robot_id not in to_list.split(","):
            return
        # Ordering: drop duplicates or stale commands
        if seq <= self.last_seq:
            self.get_logger().info(f"Ignored seq {seq}")
            return
        self.last_seq = seq

        # Send ACK back
        ack = String(data=f"ack:{seq};id:{self.robot_id}")
        self.ack_pub.publish(ack)
        self.get_logger().info(f"ACK→ {ack.data}")

        # Optionally publish status
        status = String(data=f"OK:{seq}:{self.robot_id}")
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    # Prompt for a valid robot id: 'robotN'
    import re
    while True:
        rid = input("Enter robot id (robotN): ")
        if re.fullmatch(r"robot\\d+", rid):
            break
    node = RobotNode(rid)
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
