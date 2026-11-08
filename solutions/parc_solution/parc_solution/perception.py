#!/usr/bin/env python3
"""World-frame occupancy evidence built from the LiDAR, for Nav2's static layer.

What this adds on top of Nav2's own costmap layers
--------------------------------------------------
Nav2's VoxelLayer already marks and clears obstacles, but it is a *costmap*: it
forgets in the way a costmap forgets - a cell is lethal or it is not, and one
spurious return makes it lethal for as long as nothing clears it.

This grid holds log-odds evidence (Elfes/Moravec) instead, which buys three things
the costmap layers cannot give us here:

1. **Confidence.** A cell needs repeated hits to become an obstacle, so a single
   bad return cannot block a corridor, and a visitor who walks away is cleared
   progressively rather than leaving a phantom.

2. **Furniture completion.** At the LiDAR's 6 cm height a WoodenChair is four 3 cm
   legs - four isolated dots with 0.4 m of apparent free space between them, which
   any planner will route straight through. The robot's 0.69 m upper chassis then
   hits the seat. A morphological closing turns that constellation back into one
   solid object (see occupancy_data).

3. **World-frame permanence.** Because `map` is the Gazebo world frame in this
   solution (see world_anchor.py), the grid is anchored to the world and nothing
   ever shifts it. There is no pose graph to lurch and no rolling window to fall
   out of.

The result is published as a nav_msgs/OccupancyGrid and consumed by the static
layer of both costmaps - the standard Nav2 interface a map_server would use. That
is deliberate: it injects a custom perception layer into Nav2 without a C++ plugin.
"""

import numpy as np
from scipy import ndimage

# Log-odds increments per observation. With these values a cell needs ~2 hits to
# cross OCCUPIED and ~4 clear sweeps to fall back below it: slow to forget a real
# obstacle, but still able to forget one that has genuinely moved.
L_HIT = 0.85
L_MISS = -0.40
L_CLAMP = 4.0
L_OCCUPIED = 1.0

# The RPLIDAR C1 sits 6 cm off the floor and a wide arc behind the robot comes back
# at the sensor's own range_min - it is seeing the Sito-E's own chassis (measured:
# 101 of 352 valid beams at 0.05 m, between 168 deg and 180 deg).
#
# Those beams are discarded rather than treated as free space: anything within
# 0.45 m of the LiDAR is inside the robot's own footprint, so the beam carries no
# information about the world in either direction.
SELF_RETURN_MAX_M = 0.45

# Specified to 12 m, but the last metre is noisy indoors. A beam that reports no
# return still means "free out to here", which is why this is used as both the
# marking limit and the ray-tracing limit.
USABLE_RANGE_M = 11.0

# Morphological closing radius for furniture completion.
#
# Measured on synthetic cases at this grid resolution: 0.25 m fills nothing (the
# dilated legs only touch, and the erosion undoes the bridge), 0.30 m and up fill the
# 0.4 m between a WoodenChair's legs. A 0.9 m doorway survives even at 0.40 m, because
# closing a slit in a wall needs a radius of half the gap - so there is a comfortable
# band between "closes a chair" and "closes a door", and 0.32 m sits in it.
FURNITURE_CLOSING_RADIUS_M = 0.32

# Overhang halo. A cafe_table is a pedestal about 0.42 m across carrying a top 0.91 m
# across, and the LiDAR at 6 cm sees only the pedestal: the planner routes the robot
# past it at a distance that looks safe and the 0.69 m upper chassis meets the
# overhanging top. That is not a theory - it is how the best run so far ended, wedged
# against `cafe_table_6::link::surface`, 5.83 m travelled and then nothing, with a
# valid plan on screen and no recovery firing because Nav2 believed everything was
# fine.
#
# Dilating every measured obstacle by the overhang half-width puts the missing 0.25 m
# back. It thickens walls by the same amount, which is why the halo is published as an
# expensive-but-passable value rather than as an obstacle: it shapes plans, it does not
# forbid anything.
OVERHANG_HALO_RADIUS_M = 0.25

# The halo is only worth having if the planner respects it. Published as an expensive
# but passable value it was simply driven through - the direct route is shorter, so
# the robot paid the cost and hit `cafe_table_6::link::surface` again. Around furniture
# it is therefore published as an obstacle.
#
# Around walls it is not, and that distinction is the whole trick: a lethal halo on
# every wall would narrow a 0.9 m doorway to 0.4 m and seal the restaurant. Furniture
# is told apart from architecture by size - a table pedestal is ~14 cells at this
# resolution and a chair leg is one or two, while a wall runs to hundreds.
FURNITURE_MAX_CELLS = 60

# ...and it must be compact as well as small. Size alone is not enough: a wall that has
# only been half seen arrives as several short fragments, each of them under the cell
# count, and giving those a lethal halo narrows every corridor at once. The planner's
# answer was a 20 m detour away from the goal - the robot drove backwards, from 8.8 m
# to 11.3 m, while the direct route sat there blocked by a halo around nothing.
#
# A cafe_table pedestal measures about 0.4 m across and a chair leg a few centimetres;
# a wall fragment is long and thin. Requiring both extents to fit inside this box keeps
# the furniture and lets the architecture through.
FURNITURE_MAX_EXTENT_M = 0.8

# OccupancyGrid values. 100 is lethal to the static layer; the completion cells are
# published as a high-but-passable cost instead, so that if the closing ever bridges
# a real gap the planner may still use it when there is no alternative.
VALUE_UNKNOWN = -1
VALUE_FREE = 0
VALUE_FURNITURE = 60
VALUE_OCCUPIED = 100

# How long a bumper-contact mark stays on the map (clock seconds). It must outlive the
# BackUp + replan cycle (about 15 s) with room to spare, but not be permanent.
TOUCH_TTL_S = 60.0


def _disk(radius_cells):
    """Boolean disk structuring element of the given radius, in cells."""
    r = int(round(radius_cells))
    span = np.arange(-r, r + 1)
    dy, dx = np.meshgrid(span, span, indexing="ij")
    return (dx * dx + dy * dy) <= r * r


class EvidenceGrid:
    """Log-odds occupancy evidence over a fixed window of the Gazebo world frame."""

    def remember_touch(self, x, y, radius=0.15, now=0.0, ttl=TOUCH_TTL_S):
        """Mark a spot the robot physically bumped into, for `ttl` seconds.

        The LiDAR sits 6 cm off the floor and a cafe_table is a 0.42 m pedestal down
        there and a 0.91 m top at 0.755 m. The robot's 0.69 m upper chassis therefore
        collides with something no sensor of ours can see, and once wedged it has no
        way to understand why: the map in front of it is empty, so every plan it makes
        drives into the same invisible edge again.

        A bumper contact is the one measurement that does see it. Recording it here
        turns a collision into knowledge - the cell becomes an obstacle like any other
        and the planner routes around it.

        The mark is small (the contact point, not a whole room-sized patch) and it
        EXPIRES. A permanent 0.7 m lethal square wrapped in 0.60 m of inflation sealed
        the robot into its own recovery: BackUp refused to move, the planner found no
        way out, and the behaviour tree fell through to Spin - the robot stopped and
        turned on the spot instead of going around. After `ttl` the mark is dropped;
        if the obstacle is really still there the robot touches it again and re-marks.
        """
        row, col = self.cells_of(np.array([x]), np.array([y]))
        span = max(1, int(round(radius / self.resolution)))
        r0, r1 = max(0, row[0] - span), min(self.size, row[0] + span + 1)
        c0, c1 = max(0, col[0] - span), min(self.size, col[0] + span + 1)
        self.touched_until[r0:r1, c0:c1] = max(now + ttl, 1e-3)

    def __init__(self, center, size_m=34.0, resolution=0.10):
        self.resolution = resolution
        self.size = int(round(size_m / resolution))
        self.origin = (
            center[0] - 0.5 * self.size * resolution,
            center[1] - 0.5 * self.size * resolution,
        )
        self.logodds = np.zeros((self.size, self.size), dtype=np.float32)
        self.observed = np.zeros((self.size, self.size), dtype=bool)
        # Obstacles discovered by bumping into them, each with the time (clock
        # seconds) at which the mark lapses. See remember_touch.
        self.touched_until = np.zeros((self.size, self.size), dtype=np.float64)
        self.scans_integrated = 0

    def cells_of(self, x, y):
        """Vectorised world -> (row, col), clipped to the grid."""
        col = np.clip(
            ((x - self.origin[0]) / self.resolution).astype(np.int32), 0, self.size - 1
        )
        row = np.clip(
            ((y - self.origin[1]) / self.resolution).astype(np.int32), 0, self.size - 1
        )
        return row, col

    def integrate(self, pose, angles, ranges):
        """Fold one LaserScan, taken at world `pose` = (x, y, yaw), into the grid.

        The sweep is processed as two vectorised scatter-adds rather than a per-beam
        loop, which is what makes a home-made grid affordable next to Nav2: ~3 ms
        for a 360-beam scan. That matters because this world runs at RTF 0.02 and
        every core spent on perception is a core Gazebo does not get.
        """
        px, py, yaw = pose

        finite = np.isfinite(ranges)
        keep = ~(finite & (ranges <= SELF_RETURN_MAX_M))
        if not np.any(keep):
            return

        beam_angles = yaw + angles[keep]
        beam_ranges = np.where(
            finite[keep], np.minimum(ranges[keep], USABLE_RANGE_M), USABLE_RANGE_M
        )
        is_hit = finite[keep] & (ranges[keep] < USABLE_RANGE_M)

        cos_a = np.cos(beam_angles)
        sin_a = np.sin(beam_angles)

        # Free space: sample every beam at grid resolution and keep the samples that
        # fall short of the return.
        steps = np.arange(1, int(USABLE_RANGE_M / self.resolution) + 1) * self.resolution
        inside = steps[None, :] < (beam_ranges[:, None] - self.resolution)
        if np.any(inside):
            beam_index, step_index = np.nonzero(inside)
            d = steps[step_index]
            row, col = self.cells_of(
                px + d * cos_a[beam_index], py + d * sin_a[beam_index]
            )
            # De-duplicate before accumulating. Several samples of one beam land in
            # the same cell near the sensor, and without this a cell 0.2 m away would
            # collect ten times the free-space evidence of a cell 5 m away.
            flat = np.unique(row.astype(np.int64) * self.size + col)
            rows, cols = np.divmod(flat, self.size)
            self.logodds[rows, cols] += L_MISS
            self.observed[rows, cols] = True

        # Occupied: the returns themselves.
        if np.any(is_hit):
            row, col = self.cells_of(
                px + beam_ranges[is_hit] * cos_a[is_hit],
                py + beam_ranges[is_hit] * sin_a[is_hit],
            )
            flat = np.unique(row.astype(np.int64) * self.size + col)
            rows, cols = np.divmod(flat, self.size)
            self.logodds[rows, cols] += L_HIT
            self.observed[rows, cols] = True

        np.clip(self.logodds, -L_CLAMP, L_CLAMP, out=self.logodds)
        self.scans_integrated += 1

    def occupancy_data(
        self,
        clear_around=None,
        clear_radius=0.45,
        unknown_value=VALUE_FREE,
        protect=None,
        protect_radius=0.55,
        now=None,
    ):
        """The grid as nav_msgs/OccupancyGrid data, row-major.

        `clear_around` forces a disc of free space at the robot's own position. The
        LiDAR self-returns are already filtered, but the robot also drives over
        cells it previously marked while looking at them from a distance, and one
        stale lethal cell under the robot makes the planner declare its own start
        cell blocked - which surfaces as "Failed to create plan", not as a
        perception problem.

        `protect` is a world point around which furniture completion is suppressed -
        in practice the goal. The completion is an *inference*, not a measurement, and
        the goal is by construction a spot the organisers placed in free space, so a
        completion that covers it is wrong by definition. It happened: the goal sits in
        a nook by the counter, the closing bridged the gap, the goal cell went lethal
        and the planner answered "Failed to create plan with tolerance of: 0.500000"
        from everywhere in the room while a flood fill showed no route existed at all.
        Measured obstacles inside the protected disc are left untouched.

        `unknown_value` defaults to FREE rather than to the -1 that means "unknown"
        in an OccupancyGrid, and that is on purpose. Unexplored space must read as
        free here: the goal sits ~9.9 m away in a part of the restaurant the robot
        has never seen, and cells marked NO_INFORMATION made NavFn treat the goal as
        buried inside an obstacle - planning failed outright and the robot never
        moved. Publishing free space instead makes the behaviour independent of how
        each costmap layer happens to interpret unknown cells; the planner routes
        optimistically through the unexplored and replans as the LiDAR reveals the
        real layout.
        """
        occupied = self.logodds > L_OCCUPIED

        radius_cells = max(1, int(round(FURNITURE_CLOSING_RADIUS_M / self.resolution)))
        closed = ndimage.binary_closing(occupied, structure=_disk(radius_cells))

        halo_cells = max(1, int(round(OVERHANG_HALO_RADIUS_M / self.resolution)))
        closed |= ndimage.binary_dilation(occupied, structure=_disk(halo_cells))

        # Furniture-sized components get their overhang as a genuine obstacle.
        labels, count = ndimage.label(occupied)
        furniture = np.zeros_like(occupied)
        if count:
            sizes = np.bincount(labels.ravel())
            max_cells = int(round(FURNITURE_MAX_EXTENT_M / self.resolution))
            keep = []
            for index, box in enumerate(ndimage.find_objects(labels), start=1):
                if box is None or sizes[index] > FURNITURE_MAX_CELLS:
                    continue
                height = box[0].stop - box[0].start
                width = box[1].stop - box[1].start
                if height <= max_cells and width <= max_cells:
                    keep.append(index)
            if keep:
                furniture = np.isin(labels, keep)
        overhang = ndimage.binary_dilation(furniture, structure=_disk(halo_cells))

        data = np.full(self.logodds.shape, np.int8(unknown_value), dtype=np.int8)
        data[self.observed] = VALUE_FREE
        completion = closed & ~occupied
        overhang = overhang & ~occupied
        if protect is not None:
            row, col = self.cells_of(
                np.array([protect[0]]), np.array([protect[1]])
            )
            span = int(round(protect_radius / self.resolution))
            r0, r1 = max(0, row[0] - span), min(self.size, row[0] + span + 1)
            c0, c1 = max(0, col[0] - span), min(self.size, col[0] + span + 1)
            completion[r0:r1, c0:c1] = False
            overhang[r0:r1, c0:c1] = False

        # The overhang is published as expensive, NOT as an obstacle.
        #
        # Making it lethal did keep the planner away from the tabletop - but it also
        # closed the restaurant: with the 0.60 m inflation on top, each table grew a
        # 0.70 m exclusion disc, table-and-chair pairs merged into solid blocks, and the
        # robot could not even leave its spawn ("Failed to create plan" seven times in
        # the first fifteen seconds, 0.38 m travelled). Expensive-but-passable lets the
        # planner route around the tabletop whenever there is room and squeeze past when
        # there is not, which is what a 0.49 m robot in a restaurant actually needs.
        #
        # What stops the chassis clipping the overhang is then the controller, not the
        # map: its lookahead is short enough (0.35 m) to follow the plan instead of
        # smoothing the corner off it.
        data[completion | overhang] = VALUE_FURNITURE
        # What the robot has touched is an obstacle, full stop - and it outranks the
        # goal protection below, because a measured collision beats any inference.
        data[self.touched_until > (0.0 if now is None else now)] = VALUE_OCCUPIED
        data[occupied] = VALUE_OCCUPIED

        if clear_around is not None:
            row, col = self.cells_of(
                np.array([clear_around[0]]), np.array([clear_around[1]])
            )
            span = int(round(clear_radius / self.resolution))
            r0, r1 = max(0, row[0] - span), min(self.size, row[0] + span + 1)
            c0, c1 = max(0, col[0] - span), min(self.size, col[0] + span + 1)
            data[r0:r1, c0:c1] = VALUE_FREE

        return data.ravel(order="C").tolist()