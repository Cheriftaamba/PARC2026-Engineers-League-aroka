#!/usr/bin/env bash
# LAPTOP: full PARC phase-1 run in one command. Starts the simulator (Gazebo + RViz), waits for the
# robot, then runs our solution. The simulator stays open at the end of the run to look at the result.
#
#   ./scripts/sim.sh
#
# Logs: $LOG_DIR (default ~/aroka_phase1_logs/<date>)    Ctrl-C stops everything.
source "$(dirname "$0")/env.sh"
check_no_sim
LOG_DIR=${LOG_DIR:-~/aroka_phase1_logs/$(date +%Y%m%d_%H%M%S)}
mkdir -p "$LOG_DIR"
echo ">>> logs in $LOG_DIR"

cleanup() {
  echo ">>> stopping the simulation"
  kill -INT -- "-$SIM" 2>/dev/null; sleep 3; kill -9 -- "-$SIM" 2>/dev/null
  pkill -9 -f "[g]z sim .*task_world.sdf" 2>/dev/null
  exit 0
}
trap cleanup INT TERM

# In its own session, so that Ctrl-C reaches the solution first and it can shut Nav2 down cleanly.
setsid ros2 launch parc_robot_bringup task.launch.py > "$LOG_DIR/sim.log" 2>&1 &
SIM=$!

echo ">>> simulator starting, waiting for the robot"
for i in $(seq 1 60); do
  timeout 5 gz model --list 2>/dev/null | grep -q sitoe_robot && break
  kill -0 "$SIM" 2>/dev/null || { echo "simulator stopped - see $LOG_DIR/sim.log"; exit 1; }
  sleep 2
done
timeout 5 gz model --list 2>/dev/null | grep -q sitoe_robot || { echo "robot never appeared - see $LOG_DIR/sim.log"; cleanup; }
sleep 10   # bridges and robot_state_publisher

# In the foreground for the same reason; tee -i so it outlives Ctrl-C and keeps the shutdown logs.
ros2 run parc_solution task_solution.py 2>&1 | tee -i "$LOG_DIR/solution.log"

echo ">>> run finished - report in ${TMPDIR:-/tmp}/parc_solution_report.json - Ctrl-C to close the simulator"
wait "$SIM"
cleanup
