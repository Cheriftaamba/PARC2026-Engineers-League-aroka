#!/usr/bin/env python3
"""Drop the LiDAR returns that hit the Sito-E's own chassis.

The RPLIDAR C1 sits low on the base and a wide arc behind the robot (roughly
168 deg to 180 deg, 101 of 352 valid beams) comes back at 0.05 m - the sensor's
declared `range_min`, i.e. something pressed against it. That something is the
robot's own body.

Left alone, those returns are marked as an obstacle at the robot's own centre.
The costmap then inflates them to INSCRIBED_INFLATED_OBSTACLE, the planner finds
its own start cell blocked, and every ComputePathToPose fails with "Failed to
create plan" - the robot never moves at all.

slam_toolbox exposes only `max_laser_range`, so the filtering has to happen
upstream of both SLAM and the costmaps rather than in their configuration.

Threshold: measured on the running simulation, every self-return is below 0.40 m
and the nearest genuine return is beyond 0.60 m, so 0.45 m separates them with
margin on both sides. Anything genuinely within 0.45 m of the LiDAR is already
in contact with the robot.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


DEFAULT_MIN_RANGE = 0.45


class ScanFilter(Node):

    def __init__(self):
        super().__init__("scan_filter")

        self.declare_parameter("min_range", DEFAULT_MIN_RANGE)
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_filtered")

        self.min_range = self.get_parameter("min_range").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        # Keep the default (reliable) QoS of the gz bridge: a reliable publisher
        # also serves best-effort subscribers, so both slam_toolbox and the Nav2
        # costmaps match it whichever profile they request.
        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.on_scan, 10
        )

        self.get_logger().info(
            f"Filtering {input_topic} -> {output_topic}, dropping returns below "
            f"{self.min_range:.2f} m"
        )
        self._reported = False

    def on_scan(self, msg):
        dropped = 0
        ranges = list(msg.ranges)
        for index, value in enumerate(ranges):
            if math.isfinite(value) and value < self.min_range:
                # inf reads as "no return in this direction", which is what the
                # consumers expect for a beam that hit nothing measurable.
                ranges[index] = float("inf")
                dropped += 1

        msg.ranges = ranges
        # Keep range_min consistent with what we now guarantee.
        msg.range_min = max(msg.range_min, float(self.min_range))
        self.publisher.publish(msg)

        if not self._reported:
            self._reported = True
            self.get_logger().info(
                f"First scan filtered: dropped {dropped}/{len(ranges)} returns."
            )


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
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
