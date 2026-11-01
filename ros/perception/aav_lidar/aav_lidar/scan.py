"""
The roof lidar's cloud, flattened into a 2D scan around base_link. No ROS here.

lidar_scan (the node) hands this module the cloud's points; it returns, for each direction
around the car, the distance to the nearest point between two heights above the road:
what the car would hit. Below the lower height is the road; above the upper one, what the
car passes under. Nav2's local costmap and collision_monitor read the result
(/sensors/lidar/scan), a few kB instead of the cloud's 0.9 MB.

The same idea as the stock pointcloud_to_laserscan, which we tried first: it subscribes
best effort, and on a Mac (UDP, no shared memory) no 0.9 MB cloud ever arrives that way
(docs/mac.md). This one subscribes reliably.
"""

from dataclasses import dataclass
import math

import numpy


@dataclass(frozen=True)
class ScanShape:
    """Which points count, and the scan's directions. Metres and radians, in base_link."""

    min_height: float = 0.3
    max_height: float = 2.0
    angle_min: float = -math.pi
    angle_max: float = math.pi
    angle_increment: float = math.radians(0.5)
    range_min: float = 0.2
    range_max: float = 30.0
    # The car itself, in base_link: points inside this box are its own roof and frame
    # (the simulated lidar sees bits of the car's top). x from, x to, half the width.
    body: tuple = (0.0, 0.0, 0.0)

    @property
    def count(self) -> int:
        """Return how many directions the scan has."""
        return int(round((self.angle_max - self.angle_min) / self.angle_increment))


def rotation(rpy) -> numpy.ndarray:
    """Return the 3 x 3 rotation of roll, pitch, yaw (rad), as URDF and REP-103 mean them."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return numpy.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def to_base(points: numpy.ndarray, xyz, rpy) -> numpy.ndarray:
    """Return (N, 3) points from the lidar's frame in base_link's, the lidar at xyz, rpy."""
    return points @ rotation(rpy).T + numpy.asarray(xyz, dtype=float)


def flatten(points: numpy.ndarray, shape: ScanShape) -> numpy.ndarray:
    """
    Return the scan's ranges (m) from (N, 3) points in base_link: inf where nothing is.

    Each range is the nearest point in its direction between the two heights, measured
    flat from base_link. Points that are NaN or inf (rays that hit nothing) don't count,
    nor do points on the car itself (shape.body).
    """
    ranges = numpy.full(shape.count, numpy.inf)
    if len(points) == 0:
        return ranges
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    flat = numpy.hypot(x, y)
    back, front, half = shape.body
    own = (x >= back) & (x <= front) & (numpy.abs(y) <= half)
    keep = (numpy.isfinite(flat) & numpy.isfinite(z) & (z >= shape.min_height)
            & (z <= shape.max_height) & (flat >= shape.range_min) & (flat <= shape.range_max)
            & ~own)
    angle = numpy.arctan2(y[keep], x[keep])
    # The nearest direction, not the one below: a LaserScan's range i lies AT angle_min +
    # i * increment, so rounding down turned every point half a step (0.25 degrees)
    # clockwise: 13 cm at 30 m, which AMCL read as a heading error (29 Oct). All the way
    # round, the last half step is the first direction again (-pi and pi are one).
    index = numpy.rint((angle - shape.angle_min) / shape.angle_increment).astype(int)
    if shape.angle_max - shape.angle_min >= 2.0 * math.pi - 1e-6:
        index %= shape.count
    inside = (index >= 0) & (index < shape.count)
    numpy.minimum.at(ranges, index[inside], flat[keep][inside])
    return ranges
