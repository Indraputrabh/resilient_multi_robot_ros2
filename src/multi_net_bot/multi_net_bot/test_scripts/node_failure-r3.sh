#!/usr/bin/env bash
# run_node_fail_test.sh — Reproduces R3 with FIVE RobotNodes (IDs 1-5)

set -euo pipefail

WAIT_BEFORE_KILL=30      # seconds before we kill robot3
WAIT_AFTER_KILL=40       # seconds to observe recovery
LOG_DIR="$HOME/multinet_logs_r3"
mkdir -p "$LOG_DIR"

# 0. Force UDP, disable shared-memory
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_SHM=0

open_tab () {
  local title="$1" cmd="$2" log="$LOG_DIR/$3"
  gnome-terminal --tab --title="$title" -- bash -c "\
     echo '[INFO] $cmd'; \
     $cmd | tee \"$log\""
}

# 1. RoleManager (1-1-1 split)
open_tab "RoleManager" \
  "ros2 run multi_net_bot role_manager <<< \$'1\n1\n1\n'" \
  role_manager.log
sleep 2

# 2. SwarmManager (leave ACK threshold at default 0.8)
open_tab "SwarmManager" \
  "ros2 run multi_net_bot swarm_manager" \
  swarm_manager.log
sleep 2

# 3. Launch FIVE RobotNodes (IDs 1-5)
for id in 1 2 3 4 5; do
  open_tab "Robot$id" "echo $id | ros2 run multi_net_bot robot_node" \
           robot${id}.log
  sleep 1
done

# 4. Let system stabilise, then kill robot3
echo "[INFO] Swarm stabilising for $WAIT_BEFORE_KILL s …"
sleep "$WAIT_BEFORE_KILL"

echo "[INFO] Killing RobotNode 3 to simulate failure …"
pkill -f "robot_node.*robot3" || true

echo "[INFO] Observing recovery for $WAIT_AFTER_KILL s …"
sleep "$WAIT_AFTER_KILL"

echo "[DONE] Check RoleManager tab for timeout + reassignment, \
and SwarmManager tab for updated ACK list. Logs in $LOG_DIR."
