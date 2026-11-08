#!/usr/bin/env bash
# LAPTOP: build the workspace (needed once, then after every code or config change).
#   ./scripts/build.sh                                  whole workspace
#   ./scripts/build.sh --packages-select parc_solution  only our package
WS="$(cd "$(dirname "$0")/../../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$WS" && colcon build "$@"
