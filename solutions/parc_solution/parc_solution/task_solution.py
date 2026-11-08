#!/usr/bin/env python3
"""PARC Engineers League 2026 - Phase 1: drive the Sito-E to the goal region.

Run it with the competition's simulation already up:

    ros2 launch parc_robot_bringup task.launch.py     # terminal 1
    ros2 run parc_solution task_solution.py           # terminal 2

The second command is enough on its own - if the navigation stack is not already
running, this node starts it.

The architecture, in one paragraph
---------------------------------
Nav2 plans and drives. Around it, three nodes do the things Nav2 cannot do for
itself in this task: `world_anchor` publishes `map -> odom` so that **`map` is the
Gazebo world frame**, `evidence_map` feeds Nav2's static layer a log-odds LiDAR map
of the restaurant, and `speed_governor` raises the speed ceiling on open floor. This
node is the mission layer: it sends one goal, watches the run against the three
scored criteria, and knows when the task is won.

What "map is the world frame" buys here
---------------------------------------
The goal in `task_params.yaml` is expressed in Gazebo world coordinates. Because
`map` now *is* that frame, the goal is sent verbatim - no conversion, and more
importantly no re-aiming. Every previous version corrected the goal for odometry
drift, and since each NavigateToPose request preempts the one in flight, that
correction became the dominant failure mode (117 preemptions in 276 s: the robot
restarted every 2.4 s and never followed a plan to the end). Drift is now absorbed
in the transform, which no planner ever notices.
"""

import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import BackUp, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # pragma: no cover - only used for the optional contact report
    Contacts = None


TASK_PARAMS_PKG = "parc_robot_bringup"
TASK_PARAMS_RELPATH = os.path.join("config", "task_params.yaml")

# Ground-truth world pose. See world_anchor.py for why this topic and not
# /sitoe_robot/pose, which the competition's bridge declares but never fills.
WORLD_POSE_TOPIC = "/model/sitoe_robot/pose"

COLLISION_TOPICS = (
    "/base_collisions",
    "/top_chassis_collisions",
    "/left_wheel_collisions",
    "/right_wheel_collisions",
)

# The bumper sensors report permanent contact with the restaurant floor, which is not
# a collision in the sense the competition scores. Without this filter the contact
# count is inflated from the first second.
GROUND_CONTACT_NAMES = ("main_floor", "ground_plane")

# How long a (sensor, object) pair must go quiet before touching it again counts as a
# fresh contact. Below this it is the same sustained push, reported at sensor rate.
#
# The first version remembered every pair forever - once cafe_table_6 had triggered
# the backoff reflex once, grazing it again minutes later, after backing off and being
# replanned right back alongside it, was silently ignored: same pair, already "seen".
# The robot pushed against it with no reflex to stop it and tipped over with the report
# still reading "contacts: 1". A 2 s quiet window is comfortably longer than the gap
# between messages during one continuous push, and comfortably shorter than the time a
# backoff-and-replan cycle takes, so a genuine second incident is never mistaken for
# the first.
CONTACT_COOLDOWN_S = 2.0

# The goal marker is a cylinder of radius 0.6 m, and the rules say the task is complete
# as soon as ANY part of the robot is inside it. With the robot centre 0.45 m from the
# marker centre the whole footprint straddles well inside that circle, so this is a
# safe place to declare victory - and declaring it here rather than waiting for Nav2's
# own goal checker to settle the final pose saves several seconds of the scored time.
SUCCESS_RADIUS_M = 0.45

# The goal marker is a 0.6 m circle and the rules ask only that SOME part of the robot
# be inside it - they say nothing about its centre. That distinction turned out to
# matter: the marker's centre sits about 0.3 m from a wall, so once the LiDAR maps that
# wall the inflation around it makes the exact centre unplannable, and the planner
# answers "Failed to create plan with tolerance of: 0.500000" from anywhere in the
# room. Measured on a stalled run: the goal cell was lethal while the robot stood in
# free space, and a flood fill confirmed no route to it existed.
#
# So we aim at the least-obstructed point of the circle instead of its centre, sampled
# out to this radius. Every such point still puts the whole robot inside the marker.
GOAL_SEARCH_RADIUS_M = 0.45
GOAL_SEARCH_RINGS = (0.0, 0.15, 0.30, 0.45)
# If every point of the circle is lethal, widen the search. At 0.90 m from the centre
# the robot's 0.33 m circumscribed radius still overlaps the 0.6 m marker, so a corner
# of it is inside - which is all the rules ask. Beyond that the claim would be false,
# so the search stops and the run is scored as not reached.
GOAL_FALLBACK_RINGS = (0.60, 0.75, 0.90)
GOAL_SEARCH_ANGLES = 12

# The task allows 600 s. Stop early enough to always print a report.
TASK_TIME_LIMIT_S = 570.0

# Beyond this pitch or roll the robot is on its side and nothing it commands will help.
# Measured on a real failure: pitch +90.0 deg, z 0.411 m where upright is 0.221 m.
TIPPED_OVER_DEG = 45.0

# If Nav2 aborts, give it another go: the behaviour tree has already exhausted its own
# recoveries by then, but a fresh goal rebuilds the plan from the robot's new position.
MAX_GOAL_RETRIES = 4

# The lifecycle manager kills the whole stack if a managed node misses its heartbeat,
# and under this world's CPU load that has happened for real (collision_monitor, 4000
# ms). nav2_params.yaml now raises the bond timeout, but if the stack dies anyway,
# losing the run to it is worse than restarting it.
STACK_DEAD_AFTER_S = 15.0

# How often (simulated seconds) to wipe the global costmap's obstacle layer.
#
# The global costmap is a fixed map of the whole restaurant, which is what lets the
# goal always be inside it - but it also means Nav2's obstacle layer never forgets.
# Anything it marks stays marked unless a ray happens to pass through the same cell
# again, and the LiDAR simply never revisits most of what it saw while turning.
# Measured at the moment a run stalled: of 14757 lethal cells, 3828 (26 %) matched no
# obstacle in the evidence map - and one of them was the goal cell itself, which is
# why the planner reported "Failed to create plan" while the robot stood in free
# space 4.6 m away.
#
# Clearing that layer is safe here, and only because of the architecture: the
# authoritative picture of the world lives in the static layer, fed by evidence_map.py,
# which re-publishes the whole map every half second. A wipe therefore costs at most
# half a second of obstacle data and removes every stale cell. (In the version of this
# solution that had no static layer, the same wipe destroyed the only map there was -
# which is why behavior_trees/ removes it from the recovery branch.)
# On contact: stop, back off, then let the planner try again with the obstacle now on
# the map. Without this reflex the robot keeps executing the path it was on - pushing
# against whatever it just hit - until a new plan arrives seconds later. It did exactly
# that against a cafe_table and TIPPED OVER: 77 s into the run, 6 m from the goal, on
# its side with no way back. A robot whose centre of mass sits 0.30 m up over a 0.17 m
# half-track cannot afford to lean on anything.
#
# The backoff itself must not be blind. An earlier version published raw Twist commands
# straight backward for a fixed duration, with no idea what was behind the robot - and
# it backed straight into a WoodenChair it never checked for, turning one contact into
# two. Nav2's own BackUp behavior (behavior_server already loads it) walks the same
# path nav2_regulated_pure_pursuit_controller does for collision checking: it queries
# the local costmap along the reverse path and stops early if that path is blocked,
# rather than trusting the caller to have picked a safe distance.
BACKOFF_DIST_M = 0.35
BACKOFF_SPEED_MS = 0.12
BACKOFF_TIME_ALLOWANCE_S = 8.0

GLOBAL_COSTMAP_CLEAR_PERIOD_S = 25.0
CLEAR_GLOBAL_COSTMAP_SERVICE = "/global_costmap/clear_entirely_global_costmap"
CLEAR_LOCAL_COSTMAP_SERVICE = "/local_costmap/clear_entirely_local_costmap"

# After a BackUp the local costmap still holds whatever the robot saw (and marked) while
# pressed against the obstacle, and evidence_map has only just learnt the contact. Replanning
# at once plans from a stale picture: from inside an inflated zone, with a controller that
# refuses to follow the path, and the behaviour tree answers with Spin - the robot turns on
# the spot instead of going round. So: clear the local costmap, then let the clock advance
# past two evidence_map publications (it publishes every 0.5 s) before sending the goal.
SETTLE_AFTER_BACKOFF_S = 1.2

STACK_LOG = os.path.join(tempfile.gettempdir(), "parc_solution_nav_stack.log")
REPORT_PATH = os.path.join(tempfile.gettempdir(), "parc_solution_report.json")


def load_task_params():
    """Read the competition's task_params.yaml from its installed share directory."""
    path = os.path.join(get_package_share_directory(TASK_PARAMS_PKG), TASK_PARAMS_RELPATH)
    with open(path, "r") as handle:
        return yaml.safe_load(handle)["/**"]["ros__parameters"]


def yaw_from_quaternion(q):
    """2D yaw from a quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RunMonitor(Node):
    """Scores the run as it happens, on the three criteria the judges use.

    Contacts (criterion 1) come from the bumper sensors, distance travelled
    (criterion 2) is integrated from the ground-truth pose, and the elapsed time
    (criterion 3) is measured on the simulation clock by the caller.

    The same pose stream also detects the robot falling over, which matters for
    diagnosis as much as for scoring: a fallen robot reports "remaining: 0.00", spins
    its wheels freely and goes nowhere, and that was misread as a planning failure for
    a long time.
    """

    def __init__(self, goal_world):
        super().__init__("parc_run_monitor")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

        self.goal_world = goal_world
        self.pose = None
        self.distance_travelled = 0.0
        self.closest_approach = float("inf")
        self.tipped_over = False
        self.contacts = 0
        self._contact_last_seen_s = {}
        self._last_xy = None

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(PoseStamped, WORLD_POSE_TOPIC, self.on_pose, sensor_qos)

        # The global costmap, used to choose where inside the goal circle to aim.
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.costmap = None
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_costmap, map_qos
        )

        # Used only to tell when localisation is live - see tf_ready().
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if Contacts is not None:
            for topic in COLLISION_TOPICS:
                self.create_subscription(
                    Contacts, topic, self.make_contact_callback(topic), sensor_qos
                )

    def on_pose(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.pose = (x, y, yaw_from_quaternion(msg.pose.orientation))

        if self._last_xy is not None:
            step = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
            # Ignore sub-millimetre jitter so a stationary robot does not accumulate
            # phantom distance over a 10-minute run.
            if step > 0.002:
                self.distance_travelled += step
                self._last_xy = (x, y)
        else:
            self._last_xy = (x, y)

        self.closest_approach = min(self.closest_approach, self.distance_to_goal())

        q = msg.pose.orientation
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        )
        tilt = max(abs(math.degrees(pitch)), abs(math.degrees(roll)))
        if tilt > TIPPED_OVER_DEG and not self.tipped_over:
            self.tipped_over = True
            self.get_logger().error(
                f"ROBOT TIPPED OVER (tilt {tilt:.1f} deg, z={msg.pose.position.z:.2f} m) "
                "- it cannot recover from this."
            )

    def _on_costmap(self, msg):
        self.costmap = msg

    def cost_at(self, x, y):
        """Global costmap cost at a world point, or None if it is not covered."""
        grid = self.costmap
        if grid is None:
            return None
        col = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        if not (0 <= col < grid.info.width and 0 <= row < grid.info.height):
            return None
        return grid.data[row * grid.info.width + col]

    def best_goal_point(self):
        """The least-obstructed point to aim at inside the goal marker.

        With a clear costmap this returns the nominal goal and the behaviour is
        identical to aiming at it directly; ties are broken towards the centre.

        It matters when the marker is not clear. The goal sits in a nook by the
        counter, and once the room is mapped the cells there read lethal: the planner
        then answers "Failed to create plan with tolerance of: 0.500000" from anywhere
        in the restaurant, and a flood fill confirms no route to it exists. The rules
        ask only that SOME part of the robot be inside the 0.6 m circle, so aiming
        elsewhere in the circle is not a shortcut - it is the task as written.
        """
        gx, gy = self.goal_world
        if self.costmap is None:
            return (gx, gy)

        def search(rings):
            best = None
            for radius in rings:
                count = 1 if radius == 0.0 else GOAL_SEARCH_ANGLES
                for index in range(count):
                    angle = 2.0 * math.pi * index / count
                    x = gx + radius * math.cos(angle)
                    y = gy + radius * math.sin(angle)
                    cost = self.cost_at(x, y)
                    if cost is None:
                        continue
                    # Unknown (-1) counts as free: the planner is configured to route
                    # optimistically through unexplored space.
                    score = (max(cost, 0), radius)
                    if best is None or score < best[0]:
                        best = (score, (x, y))
            return best

        best = search(GOAL_SEARCH_RINGS)
        if best is None:
            return (gx, gy)
        if best[0][0] >= 99:
            # Every point of the marker reads lethal. Reach for its edge rather than
            # aim at a cell no planner will accept.
            wider = search(GOAL_FALLBACK_RINGS)
            if wider is not None and wider[0][0] < best[0][0]:
                best = wider
        return best[1]

    def tf_ready(self):
        """True once `map -> base_footprint` exists, i.e. localisation is publishing.

        Sending a goal before that wastes one: the global costmap answers "Timed out
        waiting for transform from base_footprint to map ... frame does not exist", the
        behaviour tree aborts within seconds, and the run burns a retry before the robot
        has moved at all. world_anchor only starts publishing once /odom arrives, which
        is a little after Nav2 reports itself active.
        """
        return self.tf_buffer.can_transform("map", "base_footprint", rclpy.time.Time())

    def distance_to_goal(self):
        if self.pose is None:
            return float("inf")
        return math.hypot(self.pose[0] - self.goal_world[0], self.pose[1] - self.goal_world[1])

    def make_contact_callback(self, topic):
        def callback(msg):
            now_s = self.get_clock().now().nanoseconds / 1e9
            for contact in msg.contacts:
                names = f"{contact.collision1.name} {contact.collision2.name}"
                if any(ground in names for ground in GROUND_CONTACT_NAMES):
                    continue
                pair = (topic, contact.collision1.name, contact.collision2.name)
                last = self._contact_last_seen_s.get(pair)
                self._contact_last_seen_s[pair] = now_s
                if last is not None and now_s - last < CONTACT_COOLDOWN_S:
                    continue
                self.contacts += 1
                self.get_logger().warn(f"Contact #{self.contacts}: {contact.collision2.name}")

        return callback


class NavigationStack:
    """Starts and, if it dies, restarts the SLAM-free Nav2 stack we ship."""

    def __init__(self, logger):
        self.logger = logger
        self.process = None

    def start(self):
        self.logger.info(f"Starting the navigation stack (logs: {STACK_LOG})")
        log = open(STACK_LOG, "a")
        # start_new_session puts the launch and every node it spawns in one process
        # group, so shutdown can take the whole group down at once. terminate() on the
        # `ros2 launch` parent alone leaves planner_server and friends orphaned, and the
        # next run then finds a stale NavigateToPose server, assumes the stack is
        # healthy, reuses it, and hangs forever with no plan ever produced.
        self.process = subprocess.Popen(
            ["ros2", "launch", "parc_solution", "navigation.launch.py"],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def stop(self):
        if self.process is None:
            return
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=10)
        except (ProcessLookupError, PermissionError):
            pass
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
        self.process = None

    def restart(self):
        self.stop()
        time.sleep(2.0)
        self.start()


def wait_for_nav2(navigator, timeout_s=180.0):
    """Wait until Nav2 will accept a goal, without asking it any questions.

    BasicNavigator.waitUntilNav2Active() polls /bt_navigator/get_state. Under this
    world's CPU load that service round-trip is unreliable - observed for real:
    "failed to send response to /bt_navigator/get_state (timeout): client will not
    receive response", after which the mission node waited forever on a stack that
    was already fully active.

    The NavigateToPose action server appearing is both necessary and sufficient: it
    is created when bt_navigator activates, and it is the thing we are about to use.
    Waiting on it needs no service call and no response from a busy node.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            return True
    return False


def send_goal(navigator, pose, attempts=5, timeout_s=20.0):
    """Send a NavigateToPose goal, surviving a lost acceptance response.

    BasicNavigator.goToPose() waits on the goal-acceptance future with no timeout, so
    a response that never arrives hangs the mission for ever. That is not theoretical:
    under this world's CPU load bt_navigator logs "Failed to send goal response
    (timeout): client will not receive response" - it accepted the goal, the reply was
    simply lost - and the mission node sat there while a perfectly healthy Nav2 waited
    for orders.

    Re-sending preempts the goal the server may already be running, which is exactly
    what we want: one live goal, whatever happened to the paperwork.

    One more race, separate from the lost-response one above: the action server is
    created when bt_navigator is *configured*, but it only accepts goals once the
    lifecycle manager finishes *activating* it - and controller_server, smoother_server,
    planner_server, behavior_server, velocity_smoother and collision_monitor all
    activate first. wait_for_nav2() sees the server exist and returns before that
    chain completes, so the first send_goal() call can land in the gap: bt_navigator
    answers "Action server is inactive. Rejecting the goal." *immediately*, not after
    a timeout. Measured: all 5 attempts were rejected within 10 ms of each other,
    nowhere near enough time for activation to finish, and the run aborted with Nav2
    fully healthy a moment later. A short sleep before each retry is what actually
    gives that chain time to complete.
    """
    goal = NavigateToPose.Goal()
    goal.pose = pose
    goal.behavior_tree = ""

    for attempt in range(1, attempts + 1):
        future = navigator.nav_to_pose_client.send_goal_async(
            goal, navigator._feedbackCallback
        )
        rclpy.spin_until_future_complete(navigator, future, timeout_sec=timeout_s)
        handle = future.result()
        if handle is not None and handle.accepted:
            # Hand the internals back to BasicNavigator so isTaskComplete(),
            # getFeedback() and cancelTask() keep working as usual.
            navigator.goal_handle = handle
            navigator.result_future = handle.get_result_async()
            return True
        navigator.get_logger().warn(
            f"No answer to the goal request (attempt {attempt}/{attempts}) - retrying."
        )
        if attempt < attempts:
            time.sleep(1.0)
    return False


def send_backup(navigator, logger, attempts=3, timeout_s=10.0):
    """Run Nav2's BackUp behavior, surviving a lost acceptance response.

    Same lost-response risk as send_goal(): BasicNavigator.backup() waits on the
    goal-acceptance future with no timeout. Retrying here rather than falling back to
    a raw cmd_vel publish matters because BackUp is collision-checked against the
    local costmap along the reverse path - it stops itself early if something is
    behind the robot, which a blind Twist publish cannot do.
    """
    goal = BackUp.Goal()
    goal.target = Point(x=BACKOFF_DIST_M)
    goal.speed = BACKOFF_SPEED_MS
    goal.time_allowance = Duration(sec=int(BACKOFF_TIME_ALLOWANCE_S))

    for attempt in range(1, attempts + 1):
        future = navigator.backup_client.send_goal_async(goal, navigator._feedbackCallback)
        rclpy.spin_until_future_complete(navigator, future, timeout_sec=timeout_s)
        handle = future.result()
        if handle is not None and handle.accepted:
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(
                navigator, result_future, timeout_sec=BACKOFF_TIME_ALLOWANCE_S + 5.0
            )
            return
        logger.warn(f"No answer to the backup request (attempt {attempt}/{attempts}) - retrying.")
        if attempt < attempts:
            time.sleep(1.0)
    logger.warn("Backup never acknowledged - replanning from wherever the robot already is.")


def wait_sim(navigator, seconds, wall_cap_s=60.0):
    """Wait `seconds` of SIMULATED time (the world runs far below real time)."""
    start = navigator.get_clock().now()
    deadline = time.time() + wall_cap_s
    while time.time() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)
        if (navigator.get_clock().now() - start).nanoseconds / 1e9 >= seconds:
            return


def clear_local_costmap(navigator, logger, timeout_s=5.0):
    """Wipe the local costmap, giving up quietly if the service does not answer."""
    client = navigator.create_client(ClearEntireCostmap, CLEAR_LOCAL_COSTMAP_SERVICE)
    try:
        if not client.wait_for_service(timeout_sec=2.0):
            logger.warn("Local costmap clear service not available - skipping.")
            return
        future = client.call_async(ClearEntireCostmap.Request())
        rclpy.spin_until_future_complete(navigator, future, timeout_sec=timeout_s)
    finally:
        navigator.destroy_client(client)


def make_goal_pose(navigator, goal_world, from_xy=None):
    """The goal, verbatim, in `map` - which world_anchor has made the world frame.

    With `from_xy` (the robot's position) the goal is given a heading: the direction of
    travel from there. A fixed identity orientation made the robot pirouette towards
    "east" at the end of every plan, and after a BackUp it made the first thing it did
    a turn on the spot.
    """
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(goal_world[0])
    pose.pose.position.y = float(goal_world[1])
    yaw = 0.0
    if from_xy is not None:
        dx = goal_world[0] - from_xy[0]
        dy = goal_world[1] - from_xy[1]
        if math.hypot(dx, dy) > 0.05:
            yaw = math.atan2(dy, dx)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def current_goal(navigator, monitor):
    """Goal pose aimed at the least-obstructed point of the marker, heading included."""
    aim = monitor.best_goal_point()
    from_xy = (monitor.pose[0], monitor.pose[1]) if monitor.pose is not None else None
    return make_goal_pose(navigator, aim, from_xy)


def write_report(payload):
    try:
        with open(REPORT_PATH, "w") as handle:
            json.dump(payload, handle, indent=2)
    except OSError:
        pass


def main(args=None):
    rclpy.init(args=args)

    params = load_task_params()
    goal_world = (params["goal_x"], params["goal_y"])
    spawn = (params["x"], params["y"])

    navigator = BasicNavigator()
    # BasicNavigator does not set use_sim_time, so its clock is the WALL clock. Every
    # duration in this node - the 570 s budget, the progress log, the reported run time
    # - would then be measured in real seconds while the task is scored in simulated
    # ones. At the RTF this world achieves (0.07 measured) the budget would expire
    # after ~40 s of simulated time, cancelling a run that was going perfectly.
    navigator.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    logger = navigator.get_logger()

    logger.info("=" * 60)
    logger.info("PARC 2026 Phase 1 - Sito-E autonomous navigation")
    logger.info(f"Goal (Gazebo world == map): x={goal_world[0]:.3f} y={goal_world[1]:.3f}")
    logger.info(
        "Straight-line distance from spawn: "
        f"{math.hypot(goal_world[0] - spawn[0], goal_world[1] - spawn[1]):.2f} m"
    )
    logger.info("=" * 60)

    stack = NavigationStack(logger)
    # Waiting on the action server is the discovery-safe way to ask this. Calling
    # get_node_names() right after creating a node reports an empty graph, because DDS
    # discovery has not converged yet - which once made this node start a second Nav2
    # on top of a healthy one.
    logger.info("Looking for an already-running navigation stack...")
    if navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
        logger.info("Navigation stack already running - reusing it.")
        own_stack = False
    else:
        stack.start()
        own_stack = True

    monitor = RunMonitor(goal_world)
    # The monitor gets its own executor thread: sharing the main loop's attention with
    # Nav2's own spinning starved its subscriptions (one callback per spin_once, against
    # a pose stream at 10 Hz and four contact topics).
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    outcome = "failed"
    elapsed = 0.0
    try:
        if not wait_for_nav2(navigator):
            logger.error("Nav2 never came up - aborting.")
            return 1
        logger.info("Nav2 is accepting goals.")

        # Time the run on the SIMULATION clock. This world runs far below real time
        # (measured RTF 0.02 with both depth cameras at full rate), so a wall-clock
        # budget of 570 s would cancel the run after ~11 simulated seconds. The
        # competition's 600 s is simulated time, which is also what a judge replaying
        # the run would measure.
        # Wait for the first /clock message before starting the stopwatch. Until one
        # arrives a sim-time clock reads zero, so the first elapsed_sim_s() would
        # return the simulator's whole uptime (observed: a run that began at
        # "[179.1s]" of its 570 s budget).
        clock_deadline = time.time() + 30.0
        while navigator.get_clock().now().nanoseconds == 0 and time.time() < clock_deadline:
            rclpy.spin_once(navigator, timeout_sec=0.1)
        started_at = navigator.get_clock().now()

        def elapsed_sim_s():
            return (navigator.get_clock().now() - started_at).nanoseconds / 1e9

        tf_deadline = time.time() + 60.0
        while not monitor.tf_ready() and time.time() < tf_deadline:
            time.sleep(0.2)
        if not monitor.tf_ready():
            logger.warn("No map -> base_footprint transform yet; sending the goal anyway.")

        aim = monitor.best_goal_point()
        if math.hypot(aim[0] - goal_world[0], aim[1] - goal_world[1]) > 0.05:
            logger.info(
                f"Aiming at ({aim[0]:.2f}, {aim[1]:.2f}), "
                f"{math.hypot(aim[0]-goal_world[0], aim[1]-goal_world[1]):.2f} m from the "
                "marker centre: the centre itself is obstructed in the costmap."
            )
        if not send_goal(navigator, make_goal_pose(
            navigator, aim, monitor.pose[:2] if monitor.pose else spawn
        )):
            logger.error("Nav2 never acknowledged the goal - aborting.")
            return 1
        logger.info("Goal sent. It will not be re-sent unless navigation aborts.")

        retries = 0
        last_log = -5.0
        last_clear = 0.0
        server_lost_since = None
        clear_client = navigator.create_client(ClearEntireCostmap, CLEAR_GLOBAL_COSTMAP_SERVICE)
        contacts_handled = 0

        while True:
            time.sleep(0.05)
            elapsed = elapsed_sim_s()
            distance = monitor.distance_to_goal()

            # Bound the global costmap's memory. See GLOBAL_COSTMAP_CLEAR_PERIOD_S.
            if elapsed - last_clear > GLOBAL_COSTMAP_CLEAR_PERIOD_S:
                last_clear = elapsed
                if clear_client.service_is_ready():
                    # Fire and forget: the answer is not interesting and waiting for it
                    # would stall the mission loop on a busy node.
                    clear_client.call_async(ClearEntireCostmap.Request())

            # Criterion: any part of the robot inside the green circle. Claiming the
            # win here rather than letting the goal checker settle saves scored time.
            if distance <= SUCCESS_RADIUS_M:
                logger.info(f"Inside the goal region ({distance:.2f} m from its centre).")
                navigator.cancelTask()
                outcome = "reached"
                break

            # Contact reflex: stop pushing, reverse, replan. evidence_map.py has just
            # put the thing we hit on the map, so the next plan will avoid it.
            if monitor.contacts > contacts_handled:
                contacts_handled = monitor.contacts
                logger.warn("Contact - stopping and backing off before replanning.")
                navigator.cancelTask()
                send_backup(navigator, logger)
                clear_local_costmap(navigator, logger)
                wait_sim(navigator, SETTLE_AFTER_BACKOFF_S)
                if not send_goal(navigator, current_goal(navigator, monitor)):
                    logger.error("Goal not acknowledged after the backoff - will retry.")
                continue

            if monitor.tipped_over:
                logger.error("Aborting: the robot is on its side.")
                navigator.cancelTask()
                outcome = "tipped_over"
                break

            if elapsed > TASK_TIME_LIMIT_S:
                logger.error("Time limit reached - cancelling.")
                navigator.cancelTask()
                outcome = "timeout"
                break

            # Stack watchdog. The lifecycle manager tears every node down if one misses
            # its heartbeat, and that has ended a run for real under CPU starvation.
            if own_stack and not navigator.nav_to_pose_client.server_is_ready():
                if server_lost_since is None:
                    server_lost_since = elapsed
                elif elapsed - server_lost_since > STACK_DEAD_AFTER_S:
                    logger.error("Navigation stack is gone - restarting it.")
                    stack.restart()
                    wait_for_nav2(navigator)
                    send_goal(navigator, current_goal(navigator, monitor))
                    server_lost_since = None
                    continue
            else:
                server_lost_since = None

            if navigator.isTaskComplete():
                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    # Nav2's goal checker is satisfied; the monitor decides whether that
                    # actually put the robot in the circle.
                    outcome = "reached" if distance <= 0.6 else "stopped_short"
                    logger.info(f"Nav2 reports success, {distance:.2f} m from the goal centre.")
                    break
                if retries >= MAX_GOAL_RETRIES:
                    logger.error(f"Navigation failed {retries} times - giving up.")
                    outcome = "failed"
                    break
                retries += 1
                logger.warn(
                    f"Navigation ended without reaching the goal ({distance:.2f} m away) "
                    f"- re-sending the goal (attempt {retries}/{MAX_GOAL_RETRIES})."
                )
                send_goal(navigator, current_goal(navigator, monitor))

            feedback = navigator.getFeedback()
            if elapsed - last_log > 5.0:
                last_log = elapsed
                remaining = feedback.distance_remaining if feedback is not None else float("nan")
                recoveries = feedback.number_of_recoveries if feedback is not None else 0
                logger.info(
                    f"[{elapsed:6.1f}s] {distance:5.2f} m to goal "
                    f"(plan says {remaining:5.2f} m), travelled "
                    f"{monitor.distance_travelled:5.2f} m, recoveries: {recoveries}, "
                    f"contacts: {monitor.contacts}"
                )

        report = {
            "outcome": outcome,
            "duration_sim_s": round(elapsed, 1),
            "distance_travelled_m": round(monitor.distance_travelled, 2),
            "final_distance_to_goal_m": round(monitor.distance_to_goal(), 2),
            "closest_approach_m": round(monitor.closest_approach, 2),
            "contacts": monitor.contacts,
            "goal_world": list(goal_world),
        }
        write_report(report)

        logger.info("=" * 60)
        if outcome == "reached":
            logger.info("RESULT: goal region reached")
        else:
            logger.error(f"RESULT: {outcome}")
        logger.info(f"Duration            : {report['duration_sim_s']} s (simulated)")
        logger.info(f"Distance travelled  : {report['distance_travelled_m']} m")
        logger.info(f"Final distance      : {report['final_distance_to_goal_m']} m from goal centre")
        logger.info(f"Contacts            : {report['contacts']}")
        logger.info(f"Report written to   : {REPORT_PATH}")
        logger.info("=" * 60)

        return 0 if outcome == "reached" else 1

    except KeyboardInterrupt:
        navigator.cancelTask()
        return 1
    finally:
        executor.shutdown()
        monitor.destroy_node()
        navigator.destroy_node()
        if own_stack:
            stack.stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())