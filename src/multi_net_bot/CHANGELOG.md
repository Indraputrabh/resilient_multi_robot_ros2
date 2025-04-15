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
