#!/usr/bin/env bash
# LAPTOP, second terminal: our solution, with world.sh already up. It starts Nav2 by itself and
# sends the goal. Ctrl-C cancels the run and stops Nav2.
#   ./scripts/solution.sh
source "$(dirname "$0")/env.sh"
exec ros2 run parc_solution task_solution.py
