#!/usr/bin/env bash
# run_loss_test_gui.sh – GUI-based loss demo with egress-only netem
set -euo pipefail

LOSS=70          # packet-loss percentage
DURATION=60       # seconds to run swarm
ACK=1.0           # 100 % ACKs to force retries
LOG_DIR="$HOME/multinet_logs"
mkdir -p "$LOG_DIR"

# ── Force DDS to use UDP, not shared memory ────────────────────────────────
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_SHM=0

sudo -v   # single password prompt

open_tab () {
  local title="$1" cmd="$2" log="$LOG_DIR/$3"
  gnome-terminal --tab --title="$title" -- bash -c "\
     echo '[INFO]' $cmd; \
     $cmd | tee \"$log\""
}

# ── Launch nodes ───────────────────────────────────────────────────────────
open_tab "RoleManager" "printf '1\n1\n1\n' | ros2 run multi_net_bot role_manager" \
         role_manager.log
sleep 2

open_tab "SwarmManager" \
         "ros2 run multi_net_bot swarm_manager --ros-args -p ack_threshold:=$ACK" \
         swarm_manager.log
sleep 2

for id in 1 2 3; do
  open_tab "Robot$id" "echo $id | ros2 run multi_net_bot robot_node" \
           robot${id}.log
  sleep 1
done

# ── Egress-only  netem  on loopback ────────────────────────────────────────
echo "[INFO] Loading sch_netem ..."
sudo modprobe sch_netem

echo "[INFO] Clearing any previous qdisc on lo ..."
sudo tc qdisc del dev lo root 2>/dev/null || true

echo "[INFO] Adding $LOSS % egress loss on lo ..."
sudo tc qdisc add dev lo root netem loss ${LOSS}%

# ── Let swarm run ──────────────────────────────────────────────────────────
echo "[INFO] Swarm running for $DURATION s with $LOSS % loss …"
sleep $DURATION

# ── Cleanup ────────────────────────────────────────────────────────────────
echo "[INFO] Removing qdisc ..."
sudo tc qdisc del dev lo root 2>/dev/null || true

echo "[INFO] Finished.  Logs in $LOG_DIR.  Look in the SwarmManager tab for [Attempt N] lines."
