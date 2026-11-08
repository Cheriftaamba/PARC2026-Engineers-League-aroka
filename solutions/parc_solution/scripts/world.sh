#!/usr/bin/env bash
# LAPTOP, first terminal: only the competition simulator (Gazebo + RViz). Ctrl-C stops it.
#   ./scripts/world.sh
# Then ./scripts/solution.sh in a second terminal.
source "$(dirname "$0")/env.sh"
check_no_sim
exec ros2 launch parc_robot_bringup task.launch.py
