# sourced by the phase-1 launch scripts
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"   # ros2_ws, wherever it was cloned
source /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] || { echo "no built workspace in $WS - run scripts/build.sh first"; exit 1; }
source "$WS/install/setup.bash"
# Own Gazebo partition and a private, localhost-only ROS domain: no cross-talk with a robot or
# another team on the same Wi-Fi. Every script sources this, so they all see each other.
export GZ_PARTITION=aroka_phase1 ROS_DOMAIN_ID=78 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# A leftover Gazebo corrupts the sensor data of the next run: refuse to start on top of one.
check_no_sim() {
  if pgrep -f "[g]z sim .*task_world.sdf" >/dev/null; then
    echo "a Gazebo task_world is already running - stop it first (pkill -9 -f 'gz sim .*task_world.sdf')"
    exit 1
  fi
}
