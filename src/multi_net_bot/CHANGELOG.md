## [0.1] 12-04-2025
### Added
- Implemented a basic ReliablePublisher class that sends commands with a sequence number and retransmits them until an acknowledgment (ACK) is received.
- Created a simple SwarmManager node that publishes a static command (e.g., "Move forward") to the "swarm_command" topic and waits for a single ACK on the "swarm_ack" topic.
- Developed a basic RobotNode that subscribes to "swarm_command", sends an ACK (formatted as "ack:<seq>") in response, and publishes simple status messages on the "robot_status" topic.
- Added logging throughout the system for tracking message transmission, retransmission, and ACK reception.


## [0.2] - 16-04-2025
### Added
- Enhanced reliable messaging to support multiple robots.
- Updated ReliablePublisher to accept a set of expected robot IDs.
- Modified ACK callback to parse ACK messages in the format "ack:<seq>;id:<robot_id>" and track acknowledgments.
- Added retransmission logic until all expected ACKs are received.
- Revised RobotNode to include a unique robot identifier in its ACK messages.
- Enhanced logging for debugging multi-robot ACK tracking.

## [0.3.0-alpha] - 17-04-2025
### Added
- Implemented dynamic robot registration: Robot nodes now prompt the user for a unique identifier in the format "robot(x)" and automatically publish a registration message on the `/swarm_registration` topic.
- Integrated a heartbeat mechanism: Robot nodes periodically send heartbeat messages on the `/swarm_heartbeat` topic to indicate they are still active.
- Added heartbeat monitoring and deregistration in the SwarmManager: If heartbeats are not received within a defined interval, the corresponding robot is marked offline and removed from the active robot list.
- Updated ReliablePublisher to dynamically update its expected robot IDs based on incoming registration messages.
- Enhanced logging for registration, heartbeat, and deregistration events to aid in monitoring and debugging.

# ## [0.3.1] - 20-04-2025
### Added
- Configurable ACK threshold (absolute or percentage) in ReliablePublisher.
- Selective retransmission via `to:` field to only resend to missing robots.
- Exponential backoff with dynamic timeout scaling based on missing ACKs.
- History buffer for message replay or debugging.
- Message ordering and selective delivery logic in RobotNode.
- Detailed inline comments throughout to explain code sections.

## [0.3.2] - 21-04-2025

### SwarmManager
- Subscribed to `/swarm_roles` and now tracks each robot’s current role in `self.robot_roles`.
- Added `_role_timer_cb()` to dispatch **role‑specific commands** (e.g. “Move forward defensively” to defenders) by temporarily setting `ReliablePublisher.expected` to only that role’s robots.
- Consolidated all lifecycle callbacks (`_reg_cb`, `_hb_cb`, `_dreg_cb`, `_status_cb`) to keep `active_robots` up to date and refresh the ACK‑expected set automatically.
- Retained the existing reliable messaging layer but wired it into role‑aware command dispatch.

### RoleManager
- New standalone node that **prompts** at startup for counts of each role (e.g. striker, defender, goalkeeper).
- Expands those counts into a pool of available role slots and **assigns** them on robot registration or first heartbeat.
- Implements **automatic failover**: when a robot deregisters or times out, its freed role is returned to the pool and immediately reassigned to the next waiting robot.
- Publishes all role assignments on `/swarm_roles` so the rest of the system stays in sync.

### RobotNode
- Subscribes to `/swarm_roles` and stores its assigned `self.my_role`.
- No longer blindly accepts every command: filters incoming `/swarm_command` messages by `to:` list **and** by `role_req:` (if present) so only role‑relevant commands are executed.
- Continues to enforce in‑order delivery via `self.last_seq`, send ACKs (`ack:<seq>;id:<robot>`) and status updates, and heartbeat/registration for dynamic discovery.
