"""
The roof lidar's cloud as coarse cubes: what stands in the car's way. No ROS here.

Ben, 30 Oct: "reducing the navigation area down to cubes. Less precise but this is a car
not a needle." The voxels node hands this module one cloud's points, already in base_link
(aav_lidar scan.to_base); it returns:

    obstacle cubes   one point for every cube (CubeShape.size, 0.25 m) holding a point
                     between min_height and max_height above the road: of its points,
                     the nearest to base_link, so the stop zone measures the real face
                     (a cube's centre can be 0.18 m from it)
    ground cubes     the centre of every flat square where the lidar saw the road: free
                     space, which the costmap clears its old marks with

The road's height comes from the cloud itself (fit_ground): a plane through the lowest
points, so a car leaning in a turn, or braking, doesn't see the road rising as an
obstacle (aav_lidar ground.py: the flat cut fails past about 1.5 degrees of roll). Points
on the car itself (aav_lidar scan.body_box: its roof, mirrors, cameras) never count.

The design: docs/designs/2026-10-30_voxels.md.
"""

from dataclasses import dataclass
import math
from typing import NamedTuple, Optional, Tuple

import numpy


@dataclass(frozen=True)
class CubeShape:
    """Which points count, and how big a cube is. Metres and radians, in base_link."""

    size: float = 0.25            # one cube's edge
    min_height: float = 0.3       # above the road: lower is the road (vehicle.obstacle_heights)
    max_height: float = 2.0       # above the road: higher is what the car passes under
    range_max: float = 30.0       # flat from base_link: further is too sparse to matter
    # The car itself: x from, x to, half the width (aav_lidar scan.body_box).
    body: tuple = (0.0, 0.0, 0.0)
    # The ground plane (fit_ground).
    ground_cell: float = 2.0      # the lowest point of each such square is a road candidate
    ground_near: float = 2.0      # m flat from base_link: candidates from here...
    ground_far: float = 20.0      # ...to here
    ground_points: int = 20       # fewer candidates that agree: keep the road flat
    # A body leans about 3 degrees at most on its springs (docs/learn-state-space.md 10);
    # a steeper fit is a wrong one (a ramp, cars all round): keep the road flat. A fit
    # 4 degrees too steep lifts the road 0.56 m at 8 m: a 1 m box there still shows.
    max_tilt: float = math.radians(4.0)
    max_offset: float = 0.2       # m: the road's height under base_link, at most
    max_spread: float = 0.05      # m: the root-mean-square of the road's points off the plane
    ground_band: float = 0.15     # m either side of the plane: the road, for clearing


class Ground(NamedTuple):
    """The road under the car: z = a x + b y + c in base_link (m), and was it fitted."""

    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    fitted: bool = False
    candidates: int = 0

    def height(self, x, y):
        """Return the road's height in base_link under (x, y)."""
        return self.a * x + self.b * y + self.c

    @property
    def roll(self) -> float:
        """
        Return the body's lean on the road, side to side (rad, REP-103: + = right side down).

        Leaning right, the road on the right looks higher: it rises as y falls.
        """
        return -math.atan(self.b)

    @property
    def pitch(self) -> float:
        """Return the body's lean on the road, front to back (rad, REP-103: + = nose down)."""
        return math.atan(self.a)


class Cubes(NamedTuple):
    """One cloud's cubes: (N, 3) centres in base_link, and the road it measured from."""

    obstacles: numpy.ndarray
    ground: numpy.ndarray
    road: Ground
    points: int                   # finite points, off the car, within range_max


def _plane(p: numpy.ndarray) -> numpy.ndarray:
    """Return (a, b, c) of the least-squares plane z = a x + b y + c through (N, 3) points."""
    design = numpy.column_stack([p[:, 0], p[:, 1], numpy.ones(len(p))])
    return numpy.linalg.lstsq(design, p[:, 2], rcond=None)[0]


def _least_per_key(points: numpy.ndarray, keys: numpy.ndarray,
                   by: numpy.ndarray) -> numpy.ndarray:
    """Return, of the (N, 3) points sharing each row of keys, the one with the least `by`."""
    if len(points) == 0:
        return numpy.zeros((0, 3))
    # Sort by key, then by `by`: the first of each key is its least.
    order = numpy.lexsort((by,) + tuple(keys[:, i] for i in reversed(range(keys.shape[1]))))
    k = keys[order]
    first = numpy.ones(len(k), dtype=bool)
    first[1:] = numpy.any(k[1:] != k[:-1], axis=1)
    return points[order[first]]


def lowest_per_cell(points: numpy.ndarray, cell: float) -> numpy.ndarray:
    """Return the lowest of the (N, 3) points in each cell x cell square, one per square."""
    keys = numpy.floor(points[:, :2] / cell).astype(numpy.int64)
    return _least_per_key(points, keys, points[:, 2])


def fit_ground(points: numpy.ndarray, shape: CubeShape) -> Ground:
    """
    Return the road under the car, fitted to (N, 3) finite points in base_link.

    The lowest point of each ground_cell square between ground_near and ground_far is a
    candidate: the road, or the bottom of a wall where the lidar saw no road. A plane
    through the lower three quarters of them, then again through those within 0.3 m of
    it, then within 0.1 m: the walls drop out. A fit with too few candidates, steeper
    than max_tilt, further than max_offset from base_link, or whose road points spread
    more than max_spread off it (a crest, a crowned road: not one plane), is not believed:
    the road is taken as flat (z = 0), as lidar_scan takes it.
    """
    flat = numpy.hypot(points[:, 0], points[:, 1])
    near = points[(flat >= shape.ground_near) & (flat <= shape.ground_far)]
    candidates = lowest_per_cell(near, shape.ground_cell)
    if len(candidates) < shape.ground_points:
        return Ground(candidates=len(candidates))
    # The first plane through the lowest candidates: the road lies under everything else.
    keep = candidates[:, 2] <= numpy.percentile(candidates[:, 2], 75)
    for tolerance in (0.3, 0.1, None):
        n = int(keep.sum())
        if n < shape.ground_points:
            return Ground(candidates=n)
        a, b, c = _plane(candidates[keep])
        if tolerance is not None:
            off = candidates[:, 2] - (a * candidates[:, 0] + b * candidates[:, 1] + c)
            keep = numpy.abs(off) <= tolerance
    tilt = math.atan(math.hypot(a, b))
    inliers = candidates[keep]
    spread = math.sqrt(float(numpy.mean(
        (inliers[:, 2] - (a * inliers[:, 0] + b * inliers[:, 1] + c)) ** 2)))
    if tilt > shape.max_tilt or abs(c) > shape.max_offset or spread > shape.max_spread:
        return Ground(candidates=n)
    return Ground(float(a), float(b), float(c), True, n)


def cube_centres(points: numpy.ndarray, size: float, flat: bool = False) -> numpy.ndarray:
    """
    Return the centre of every size-m cube holding one of the (N, 3) points, each once.

    flat: squares on the ground instead (x, y only); their z is 0, for the caller to set.
    """
    if len(points) == 0:
        return numpy.zeros((0, 3))
    axes = 2 if flat else 3
    keys = numpy.unique(numpy.floor(points[:, :axes] / size).astype(numpy.int64), axis=0)
    centres = (keys + 0.5) * size
    if flat:
        centres = numpy.column_stack([centres, numpy.zeros(len(centres))])
    return centres


def nearest_per_cube(points: numpy.ndarray, size: float) -> numpy.ndarray:
    """Return each size-m cube's point nearest to base_link (flat), of the (N, 3) points."""
    keys = numpy.floor(points / size).astype(numpy.int64)
    return _least_per_key(points, keys, numpy.hypot(points[:, 0], points[:, 1]))


def cubes(points: numpy.ndarray, shape: CubeShape, road: Optional[Ground] = None) -> Cubes:
    """
    Return one cloud's obstacle and ground cubes from its (N, 3) points in base_link.

    Points that are NaN or inf (rays that hit nothing), on the car itself (shape.body) or
    further than range_max don't count. road: the ground to measure heights from; None:
    fit it to these points (fit_ground).
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    flat = numpy.hypot(x, y)
    back, front, half = shape.body
    own = (x >= back) & (x <= front) & (numpy.abs(y) <= half)
    good = points[numpy.isfinite(flat) & numpy.isfinite(z) & (flat <= shape.range_max) & ~own]
    if road is None:
        road = fit_ground(good, shape)
    above = good[:, 2] - road.height(good[:, 0], good[:, 1])
    obstacles = nearest_per_cube(
        good[(above >= shape.min_height) & (above <= shape.max_height)], shape.size)
    ground = cube_centres(good[numpy.abs(above) <= shape.ground_band], shape.size, flat=True)
    ground[:, 2] = road.height(ground[:, 0], ground[:, 1])
    return Cubes(obstacles, ground, road, len(good))


def shape_of(size: float, heights: Tuple[float, float], body: tuple, **rest) -> CubeShape:
    """Return a CubeShape from the launch file's numbers: the size, (min, max) heights, body."""
    return CubeShape(size=float(size), min_height=float(heights[0]),
                     max_height=float(heights[1]), body=tuple(body), **rest)
