#!/usr/bin/env python3
# role_manager.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RoleManager(Node):
    """
    Manages dynamic role assignment and failover for a robot swarm.
    - Prompts user for how many of each role to create.
    - Subscribes to registration, heartbeat, deregistration.
    - Assigns roles on register or on first heartbeat if registration was missed.
    - Reassigns freed roles to the next waiting robot on timeout/deregister.
    - Publishes assignments on /swarm_roles.
    """
    def __init__(self, roles_config):
        super().__init__('role_manager')

        # Expand each role into individual slots
        self.available_roles = []
        for role, count in roles_config.items():
            self.available_roles += [role] * count

        self.role_pub      = self.create_publisher(String, 'swarm_roles', 10)
        self.active_robots = {}   # {robot_id: last_heartbeat_time}
        self.roles         = {}   # {robot_id: assigned_role}
        self.heartbeat_timeout = 5.0

        # Subscriptions for registration, heartbeat, deregistration
        self.create_subscription(String, 'swarm_registration',   self._on_register,   10)
        self.create_subscription(String, 'swarm_heartbeat',      self._on_heartbeat,  10)
        self.create_subscription(String, 'swarm_deregistration', self._on_deregister, 10)

        # Timer to expire dead robots
        self.create_timer(1.0, self._check_timeouts)

    def _on_register(self, msg: String):
        rid = msg.data.split(':',1)[1]
        now = self.get_clock().now().nanoseconds / 1e9
        self.active_robots[rid] = now
        self.get_logger().info(f"Register → {rid}")
        self._assign_role(rid)

    def _on_heartbeat(self, msg: String):
        rid = msg.data.split(':',1)[1]
        now = self.get_clock().now().nanoseconds / 1e9
        self.active_robots[rid] = now
        # Only assign if they don’t already have one
        if rid not in self.roles:
            self.get_logger().info(f"Heartbeat → {rid}, checking role…")
            self._assign_role(rid)

    def _on_deregister(self, msg: String):
        rid = msg.data.split(':',1)[1]
        self.active_robots.pop(rid, None)
        if rid in self.roles:
            freed = self.roles.pop(rid)
            self.get_logger().info(f"Deregister → {rid}, freed '{freed}'")
            self._failover(freed)

    def _check_timeouts(self):
        now = self.get_clock().now().nanoseconds / 1e9
        for rid, ts in list(self.active_robots.items()):
            if now - ts > self.heartbeat_timeout:
                self.get_logger().warn(f"Timeout → {rid}")
                del self.active_robots[rid]
                if rid in self.roles:
                    freed = self.roles.pop(rid)
                    self.get_logger().info(f"Freed '{freed}' from {rid}")
                    self._failover(freed)

    def _assign_role(self, rid: str):
        # Skip if already has a role or no slots left
        if rid in self.roles or not self.available_roles:
            return
        role = self.available_roles.pop(0)
        self.roles[rid] = role
        self.get_logger().info(f"Assigning role: {rid} → {role}")
        self._publish_role(rid, role)

    def _failover(self, freed_role: str):
        # Return the freed slot to the pool
        self.available_roles.append(freed_role)
        # Immediately give it to the first active robot without a role
        for rid in self.active_robots:
            if rid not in self.roles:
                self._assign_role(rid)
                return
        self.get_logger().info(f"Slot '{freed_role}' now available")

    def _publish_role(self, rid: str, role: str):
        msg = String(data=f"assign:{rid};role:{role}")
        self.role_pub.publish(msg)
        self.get_logger().info(f"Published role: {rid} → {role}")


def main(args=None):
    # 1) Prompt for each role count
    striker_cnt   = int(input("Amount of strikers: "))
    defender_cnt  = int(input("Amount of defenders: "))
    goalkeeper_cnt= int(input("Amount of goalkeepers: "))

    # 2) Build config dict
    roles_cfg = {
        'striker':    striker_cnt,
        'defender':   defender_cnt,
        'goalkeeper': goalkeeper_cnt,
    }

    # 3) Spin up the node
    rclpy.init(args=args)
    node = RoleManager(roles_cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
