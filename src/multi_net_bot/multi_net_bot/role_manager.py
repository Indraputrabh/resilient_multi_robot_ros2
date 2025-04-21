#!/usr/bin/env python3
# role_manager.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# -----------------------------------------------------------------------------
# RoleManager: tracks registrations and heartbeats, assigns/fails over roles,
# and provides a periodic summary instead of spamming every heartbeat.
# -----------------------------------------------------------------------------
class RoleManager(Node):
    def __init__(self, roles_config):
        super().__init__('role_manager')

        # Build the pool of role slots, e.g. {'striker':1,'defender':2} →
        # ['striker','defender','defender']
        self.available_roles = []
        for role, cnt in roles_config.items():
            self.available_roles += [role] * cnt

        self.role_pub       = self.create_publisher(String, 'swarm_roles', 10)
        self.active_robots  = {}   # robot_id -> last heartbeat time
        self.roles          = {}   # robot_id -> assigned role
        self.heartbeat_timeout = 5.0

        # Subscriptions
        self.create_subscription(String, 'swarm_registration',   self._on_register,   10)
        self.create_subscription(String, 'swarm_heartbeat',      self._on_heartbeat,  10)
        self.create_subscription(String, 'swarm_deregistration', self._on_deregister, 10)

        # Timer: expire dead robots every second
        self.create_timer(1.0, self._check_timeouts)
        # Timer: print a summary every 5 seconds
        self.create_timer(5.0, self._print_summary)

    # -------------------------------------------------------------------------
    # Registration: immediate assignment
    # -------------------------------------------------------------------------
    def _on_register(self, msg: String):
        rid = msg.data.split(':',1)[1]
        now = self.get_clock().now().nanoseconds / 1e9
        self.active_robots[rid] = now
        self.get_logger().info(f"Register → {rid}")
        self._assign_role(rid)

    # -------------------------------------------------------------------------
    # Heartbeat: assign if new, bench if no slot, silent otherwise
    # -------------------------------------------------------------------------
    def _on_heartbeat(self, msg: String):
        rid = msg.data.split(':',1)[1]
        now = self.get_clock().now().nanoseconds / 1e9
        self.active_robots[rid] = now

        # Only react if this robot has no role yet
        if rid not in self.roles:
            if self.available_roles:
                self.get_logger().info(f"Heartbeat → {rid}, assigning role…")
                self._assign_role(rid)
            else:
                self.get_logger().info(f"{rid} is benched!")

    # -------------------------------------------------------------------------
    # Deregistration: free slot and failover
    # -------------------------------------------------------------------------
    def _on_deregister(self, msg: String):
        rid = msg.data.split(':',1)[1]
        self.active_robots.pop(rid, None)
        if rid in self.roles:
            freed = self.roles.pop(rid)
            self.get_logger().info(f"Deregister → {rid}, freed '{freed}'")
            self._failover(freed)

    # -------------------------------------------------------------------------
    # Timeout checker: same as deregister on missed heartbeat
    # -------------------------------------------------------------------------
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

    # -------------------------------------------------------------------------
    # Assign one slot to a robot
    # -------------------------------------------------------------------------
    def _assign_role(self, rid: str):
        if rid in self.roles or not self.available_roles:
            return
        role = self.available_roles.pop(0)
        self.roles[rid] = role
        self.get_logger().info(f"Assigning role: {rid} → {role}")
        self._publish_role(rid, role)

    # -------------------------------------------------------------------------
    # Failover: return slot and immediately reassign if anyone’s waiting
    # -------------------------------------------------------------------------
    def _failover(self, freed_role: str):
        self.available_roles.append(freed_role)
        for rid in self.active_robots:
            if rid not in self.roles:
                self._assign_role(rid)
                return
        # no one waiting, slot stays in available_roles

    # -------------------------------------------------------------------------
    # Publish a single assignment
    # -------------------------------------------------------------------------
    def _publish_role(self, rid: str, role: str):
        msg = String(data=f"assign:{rid};role:{role}")
        self.role_pub.publish(msg)
        self.get_logger().info(f"Published role: {rid} → {role}")

    # -------------------------------------------------------------------------
    # Periodic summary
    # -------------------------------------------------------------------------
    def _print_summary(self):
        if not self.active_robots:
            self.get_logger().info("No active robots.")
            return
        summary = []
        for rid in sorted(self.active_robots):
            role = self.roles.get(rid, 'bench')
            summary.append(f"{rid}:{role}")
        self.get_logger().info("Current roles → " + ", ".join(summary))

def main(args=None):
    # prompt counts
    s = int(input("Amount of strikers: "))
    d = int(input("Amount of defenders: "))
    g = int(input("Amount of goalkeepers: "))

    rclpy.init(args=args)
    node = RoleManager({'striker':s, 'defender':d, 'goalkeeper':g})
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
