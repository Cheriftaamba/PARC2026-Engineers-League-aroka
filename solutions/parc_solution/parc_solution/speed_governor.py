#!/usr/bin/env python3
"""Context-aware speed limit for the controller: fast in the open, slow in the furniture.

Execution time is one of the three scored criteria, and the obvious way to buy time -
raising the controller's maximum speed - is also the way to lose the other two: the
Sito-E carries a 7.5 kg upper chassis at z = 0.69 m over a half-track of 0.17 m, and
it has already been measured tipping fully over (pitch +90 deg) after clipping a
tabletop. A single speed ceiling has to be chosen for the worst moment of the run and
then paid for during all of it: the previous configuration drove the entire 10 m
traverse at 0.26 m/s because two metres of it pass between tables.

So the ceiling is raised and made conditional instead. This node watches the LiDAR's
forward clearance and publishes a nav2_msgs/SpeedLimit as a percentage - Nav2's own
interface for this, the one a costmap SpeedFilter would use, which MPPI applies
through setSpeedLimit. Open floor gets the full ceiling; approaching furniture brings
it down before the controller is anywhere near it.

The clearance is measured on the raw scan rather than the costmap deliberately: it
must react to what is in front of the robot *now*, without waiting for a costmap
update cycle that, at this world's RTF of 0.02, can be hundreds of milliseconds late.
"""

import math

import numpy as np
import rclpy
from nav2_msgs.msg import SpeedLimit
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from parc_solution.perception import SELF_RETURN_MAX_M

# Clearance thresholds, in metres, measured in the forward cone.
CLEAR_AHEAD_FULL_SPEED_M = 3.0
CLEAR_AHEAD_MIN_SPEED_M = 0.9

# Percentage of the controller's configured maximum. The floor is what the whole run
# used to be driven at (0.26 of 0.36 m/s), so the slow case is no more aggressive
# than the previous fixed setting - it is simply no longer permanent.
MIN_SPEED_PERCENT = 45.0
MAX_SPEED_PERCENT = 100.0

# Half-angle of the cone that governs the limit. Wide enough to catch a table the
# robot is about to turn towards, narrow enough that a wall it drives parallel to
# does not throttle it.
FORWARD_CONE_RAD = math.radians(50.0)

# Exponential smoothing on the published percentage. Without it the limit chatters
# with every noisy beam, and MPPI re-solves against a moving constraint.
SMOOTHING = 0.35

# Republish period (simulated seconds) and the change that is worth a message.
PUBLISH_PERIOD_S = 0.4
MIN_CHANGE_PERCENT = 2.0


class SpeedGovernor(Node):

    def __init__(self):
        super().__init__("speed_governor")

        self.declare_parameter("speed_limit_topic", "/parc/speed_limit")

        self.percent = MAX_SPEED_PERCENT
        self.published = None
        self.cone_mask = None
        self.publisher = self.create_publisher(
            SpeedLimit, self.get_parameter("speed_limit_topic").value, 10
        )
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_timer(PUBLISH_PERIOD_S, self.publish_limit)

        self.get_logger().info(
            f"Speed governor: {MIN_SPEED_PERCENT:.0f}% of the ceiling at "
            f"{CLEAR_AHEAD_MIN_SPEED_M:.1f} m clearance, 100% beyond "
            f"{CLEAR_AHEAD_FULL_SPEED_M:.1f} m."
        )

    def on_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if self.cone_mask is None or self.cone_mask.size != ranges.size:
            angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
            self.cone_mask = np.abs(np.arctan2(np.sin(angles), np.cos(angles))) <= FORWARD_CONE_RAD

        # Same self-return filter as the mapper: the arc behind the robot reads at the
        # sensor's range_min because it is looking at the chassis. Those beams are
        # outside the forward cone anyway, but the threshold also removes any stray
        # short return that would otherwise pin the limit at its floor for the whole run.
        cone = ranges[self.cone_mask]
        valid = cone[np.isfinite(cone) & (cone > SELF_RETURN_MAX_M)]
        clearance = float(np.min(valid)) if valid.size else CLEAR_AHEAD_FULL_SPEED_M

        span = CLEAR_AHEAD_FULL_SPEED_M - CLEAR_AHEAD_MIN_SPEED_M
        fraction = (clearance - CLEAR_AHEAD_MIN_SPEED_M) / span
        target = MIN_SPEED_PERCENT + (MAX_SPEED_PERCENT - MIN_SPEED_PERCENT) * min(
            1.0, max(0.0, fraction)
        )
        self.percent += SMOOTHING * (target - self.percent)

    def publish_limit(self):
        if self.published is not None and abs(self.percent - self.published) < MIN_CHANGE_PERCENT:
            return
        self.published = self.percent
        message = SpeedLimit()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_footprint"
        message.percentage = True
        message.speed_limit = float(self.percent)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedGovernor()
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
