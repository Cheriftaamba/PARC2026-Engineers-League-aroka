#!/usr/bin/env python3
"""Publish the LiDAR evidence grid as a live map for Nav2's static layer.

This is the node that lets a custom perception layer into Nav2 without writing a
C++ costmap plugin: it speaks the one interface Nav2 already has for "here is what
the world looks like", nav_msgs/OccupancyGrid on a map topic, and the static layer
of the global costmap consumes it exactly as it would consume a map_server.

What Nav2 gains from it
-----------------------
* **Obstacle memory in the world frame.** `map` is the Gazebo world frame here (see
  world_anchor.py), so evidence accumulated 10 m ago is still valid and still in the
  right place. The obstacle layers alone see only what the sensors can currently
  reach.
* **Furniture completion.** Four chair legs become one chair. See perception.py.
* **Confidence.** Log-odds, so one bad return cannot block a corridor and a visitor
  who moves away is cleared.

The grid is *not* a SLAM map: nothing is estimated from the scans except occupancy.
Localisation comes from world_anchor.py, so there is no pose graph to lurch.
"""

import math
import os

import numpy as np
import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from parc_solution.perception import TOUCH_TTL_S, EvidenceGrid

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # pragma: no cover - the touch layer is optional
    Contacts = None

# The Sito-E's bumper sensors. They are the only thing on this robot that perceives a
# tabletop: the LiDAR passes under it and the depth camera proved unusable (60 % of its
# lethal cells were phantoms, one of them 0.10 m from the robot's own centre).
CONTACT_TOPICS = (
    "/base_collisions",
    "/top_chassis_collisions",
    "/left_wheel_collisions",
    "/right_wheel_collisions",
)
# The bumpers report the floor continuously; that is not something to map.
GROUND_CONTACT_NAMES = ("main_floor", "ground_plane")

# Where a contact is marked. Preferred: the contact point the simulator reports
# (world frame == map here), pushed TOUCH_PUSH_M further out so that the obstacle lands
# on the far side of the point of touch, where the thing actually is. A point farther
# than TOUCH_MAX_DIST_M from the robot is not credible and is ignored.
#
# Fallback when the message carries no usable point: TOUCH_FORWARD_M ahead of the
# robot's centre, only if the robot is not turning - while it rotates its heading says
# nothing about where it touched, and a mark in the wrong place sends the planner round
# an obstacle that is not there while the real one stays invisible.
TOUCH_FORWARD_M = 0.25
TOUCH_PUSH_M = 0.10
TOUCH_MAX_DIST_M = 0.70
TOUCH_MAX_YAW_RATE = 0.35  # rad/s, fallback only
# A small patch: the contact point, not a block. The inflation layer widens it anyway.
TOUCH_RADIUS_M = 0.15
# The bumpers fire every physics step while in contact - hundreds of messages a second,
# all describing the same collision. One mark per this many seconds is plenty.
TOUCH_COOLDOWN_S = 1.0

TASK_PARAMS_PKG = "parc_robot_bringup"
TASK_PARAMS_RELPATH = os.path.join("config", "task_params.yaml")

# Big enough to hold the whole restaurant plus the approach, centred between spawn
# and goal. Because this map defines the global costmap's extent, it also disposes of
# an old failure: the goal used to fall outside the costmap and Nav2 rejected it as
# out of bounds in 45 ms, too fast to look like a planning failure.
MAP_SIZE_M = 34.0
MAP_RESOLUTION_M = 0.10


def load_task_params():
    path = os.path.join(get_package_share_directory(TASK_PARAMS_PKG), TASK_PARAMS_RELPATH)
    with open(path, "r") as handle:
        return yaml.safe_load(handle)["/**"]["ros__parameters"]


class EvidenceMapPublisher(Node):

    def __init__(self):
        super().__init__("evidence_map")

        params = load_task_params()
        spawn = (float(params["x"]), float(params["y"]))
        goal = (float(params["goal_x"]), float(params["goal_y"]))
        center = (0.5 * (spawn[0] + goal[0]), 0.5 * (spawn[1] + goal[1]))

        # 2 Hz, not 1 Hz. This map is the authoritative picture of the world, and
        # task_solution.py periodically wipes the global costmap's obstacle layer to
        # bound its accumulation of stale cells; the blind window that wipe opens lasts
        # exactly until the next publication here, so it is worth halving. The cost is
        # one morphological closing over 340 x 340 cells, a few milliseconds.
        self.declare_parameter("publish_period", 0.5)
        self.declare_parameter("map_topic", "/parc/evidence_map")

        self.goal = goal
        self.grid = EvidenceGrid(center, size_m=MAP_SIZE_M, resolution=MAP_RESOLUTION_M)
        self.angles = None
        self.robot_xy = spawn
        self.integrated = 0
        self.next_report = 100
        self.dropped_no_tf = 0

        # Latched, like every map topic in ROS: the static layer subscribes with
        # transient-local durability and would otherwise wait a full period - or
        # miss the map entirely if it subscribes between two publications.
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.publisher = self.create_publisher(
            OccupancyGrid, self.get_parameter("map_topic").value, map_qos
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose = None
        self.yaw_rate = 0.0
        self._last_yaw = None
        self._last_yaw_t = None
        self.touches = 0
        self.last_touch_s = -1e9
        if Contacts is not None:
            touch_qos = QoSProfile(depth=10)
            touch_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            for topic in CONTACT_TOPICS:
                self.create_subscription(Contacts, topic, self.on_contact, touch_qos)

        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_timer(self.get_parameter("publish_period").value, self.publish_map)

        # Publish an empty map straight away. The global costmap sizes itself from
        # this message, and until it arrives the costmap is 0x0 and every plan fails.
        self.publish_map()

        self.get_logger().info(
            f"Evidence map {MAP_SIZE_M:.0f} x {MAP_SIZE_M:.0f} m at "
            f"{MAP_RESOLUTION_M:.2f} m, centred on ({center[0]:.2f}, {center[1]:.2f}) "
            "in the Gazebo world frame."
        )

    def on_scan(self, msg):
        if self.angles is None or len(self.angles) != len(msg.ranges):
            self.angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment

        # Look the sensor's own pose up, rather than assuming where it is: the Sito-E
        # chains base_footprint -> base_link (rotated 90 deg) -> lidar_link (rotated
        # back), and hard-coding the result of that is how frame bugs get written.
        # Asking TF for map -> the scan's own frame_id is immune to all of it.
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.2),
            )
        except tf2_ros.TransformException:
            self.dropped_no_tf += 1
            if self.dropped_no_tf in (50, 500):
                self.get_logger().warn(
                    f"Still no TF map -> {msg.header.frame_id} after "
                    f"{self.dropped_no_tf} scans; the map cannot be built."
                )
            return

        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = (tf.transform.translation.x, tf.transform.translation.y, yaw)

        self.grid.integrate(pose, self.angles, np.asarray(msg.ranges, dtype=np.float64))

        # Smoothed yaw rate, used to decide whether the heading can be trusted as the
        # direction of a contact (see on_contact).
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_yaw is not None and stamp_s > self._last_yaw_t:
            delta = math.atan2(math.sin(yaw - self._last_yaw), math.cos(yaw - self._last_yaw))
            rate = delta / (stamp_s - self._last_yaw_t)
            self.yaw_rate += 0.5 * (rate - self.yaw_rate)
        self._last_yaw, self._last_yaw_t = yaw, stamp_s

        self.pose = pose
        self.robot_xy = (pose[0], pose[1])
        self.integrated += 1

    def on_contact(self, msg):
        """Record what the robot has bumped into, where it touched it.

        This is the map learning by touch. Everything else in this grid comes from the
        LiDAR, which cannot see a 0.755 m tabletop from 6 cm off the floor - so the
        robot drove into `cafe_table_6::link::surface` on three runs in a row, wedged
        itself against an edge its map said was empty, and had no way to work out why.
        A contact says where the invisible thing is.
        """
        if self.pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_touch_s < TOUCH_COOLDOWN_S:
            return
        for contact in msg.contacts:
            names = f"{contact.collision1.name} {contact.collision2.name}"
            if any(ground in names for ground in GROUND_CONTACT_NAMES):
                continue
            x, y, yaw = self.pose

            target = None
            source = ""
            points = [
                (p.x, p.y)
                for p in getattr(contact, "positions", [])
                if math.isfinite(p.x) and math.isfinite(p.y)
            ]
            if points:
                cx = sum(p[0] for p in points) / len(points)
                cy = sum(p[1] for p in points) / len(points)
                dist = math.hypot(cx - x, cy - y)
                if dist <= TOUCH_MAX_DIST_M:
                    if dist > 1e-3:
                        cx += TOUCH_PUSH_M * (cx - x) / dist
                        cy += TOUCH_PUSH_M * (cy - y) / dist
                    target = (cx, cy)
                    source = f"contact point, {dist:.2f} m from the robot"
            if target is None:
                if abs(self.yaw_rate) > TOUCH_MAX_YAW_RATE:
                    # Turning: the heading is not where the contact is. Do not guess,
                    # and do not start the cooldown - the next message may be usable.
                    continue
                target = (
                    x + TOUCH_FORWARD_M * math.cos(yaw),
                    y + TOUCH_FORWARD_M * math.sin(yaw),
                )
                source = "robot heading (no usable contact point)"

            self.grid.remember_touch(
                target[0], target[1], radius=TOUCH_RADIUS_M, now=now, ttl=TOUCH_TTL_S
            )
            self.touches += 1
            self.last_touch_s = now
            self.get_logger().warn(
                f"Touched {contact.collision2.name} - marking ({target[0]:.2f}, "
                f"{target[1]:.2f}) from the {source}, for {TOUCH_TTL_S:.0f} s."
            )
            return

    def publish_map(self):
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.info.resolution = self.grid.resolution
        message.info.width = self.grid.size
        message.info.height = self.grid.size
        message.info.origin.position.x = self.grid.origin[0]
        message.info.origin.position.y = self.grid.origin[1]
        message.info.origin.orientation.w = 1.0
        message.data = self.grid.occupancy_data(
            clear_around=self.robot_xy,
            protect=self.goal,
            now=self.get_clock().now().nanoseconds * 1e-9,
        )
        self.publisher.publish(message)

        if self.integrated >= self.next_report:
            self.next_report = self.integrated + 100
            occupied = int(np.count_nonzero(self.grid.logodds > 1.0))
            explored = int(np.count_nonzero(self.grid.observed))
            self.get_logger().info(
                f"{self.integrated} scans mapped: {explored} cells explored, "
                f"{occupied} occupied."
            )


def main(args=None):
    rclpy.init(args=args)
    node = EvidenceMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()