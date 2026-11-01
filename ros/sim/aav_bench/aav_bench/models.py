"""
What each pretend sensor would measure, given where the model car really is.

No ROS imports, so the tests run anywhere. The car's truth comes from spine_sim on
/vehicle/bench/truth: position and heading in the map frame (x east, y north), speed, turn rate.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import List, Optional, Sequence, Tuple

import numpy as np

G = 9.80665                      # m/s^2
WGS84_A = 6378137.0              # m, the Earth's equatorial radius
WGS84_E2 = 6.69437999014e-3      # the ellipse's eccentricity squared

# The Earth's field near Ottawa, roughly (NOAA's model for 2026): 17 uT north, 52 uT down.
FIELD_NORTH_T = 17.0e-6
FIELD_DOWN_T = 52.0e-6


@dataclass
class Truth:
    """One /vehicle/bench/truth sample."""

    t: float          # s
    x: float          # m, east of the datum
    y: float          # m, north of the datum
    yaw: float        # rad, from east, positive left
    speed: float      # m/s, negative in reverse
    yaw_rate: float   # rad/s


def offset(truth: Truth, xyz: Sequence[float]) -> Tuple[float, float]:
    """Return where a point fixed on the car is, in the map frame: e.g. the GPS antenna."""
    c, s = math.cos(truth.yaw), math.sin(truth.yaw)
    return truth.x + c * xyz[0] - s * xyz[1], truth.y + s * xyz[0] + c * xyz[1]


def geo_from_map(x: float, y: float, datum_lat: float, datum_lon: float) -> Tuple[float, float]:
    """
    Turn metres east and north of the datum into latitude and longitude (degrees).

    Uses the Earth's local radii at the datum: within a few hundred metres the error is
    millimetres, well under what navsat_transform's UTM grid differs by anyway.
    """
    phi = math.radians(datum_lat)
    w = 1.0 - WGS84_E2 * math.sin(phi) ** 2
    meridian = WGS84_A * (1.0 - WGS84_E2) / w ** 1.5   # north-south radius
    normal = WGS84_A / math.sqrt(w)                       # east-west radius
    return (datum_lat + math.degrees(y / meridian),
            datum_lon + math.degrees(x / (normal * math.cos(phi))))


Vec3 = Tuple[float, float, float]


def imu_reading(now: Truth, before: Optional[Truth]) -> Tuple[Vec3, Vec3]:
    """
    Return the IMU's (accel, gyro) in its own frame, without noise.

    accel x: speeding up; accel y: the turn's sideways pull (v times the turn rate); accel z:
    gravity, as a real accelerometer reads it at rest. gyro z: the turn rate.
    """
    ax = 0.0
    if before is not None and now.t > before.t:
        ax = (now.speed - before.speed) / (now.t - before.t)
    return (ax, now.speed * now.yaw_rate, G), (0.0, 0.0, now.yaw_rate)


def magnetic_field(yaw: float) -> Vec3:
    """Return the Earth's field in the car's frame (tesla): x forward, y left, z up."""
    return (FIELD_NORTH_T * math.sin(yaw), FIELD_NORTH_T * math.cos(yaw), -FIELD_DOWN_T)


def noisy(values: Sequence[float], stddev: float, rng: random.Random,
          bias: Sequence[float] = (0.0, 0.0, 0.0)) -> Tuple[float, ...]:
    """Add Gaussian noise (and a fixed bias) to each value."""
    return tuple(v + b + rng.gauss(0.0, stddev) for v, b in zip(values, bias))


class GpsError:
    """
    A GPS receiver's position error, east and north (m): a slow drift and a little jitter.

    A real receiver without RTK is wrong by about the same metre for a while: the error
    follows the satellites and the air, over tens of seconds. Only a few centimetres of it
    are fresh on each fix. Each axis's drift is a first-order Gauss-Markov process: it
    wanders round 0 with a steady spread of `drift_stddev`, and forgets where it was over
    `drift_time` seconds (it keeps exp(-dt / drift_time) of itself each step). On top,
    `jitter_stddev` of fresh Gaussian noise per fix. A `drift_time` of 0 makes the drift
    fresh on every fix too: white noise, as the bench had before (TODO N7).

    The drift starts with its steady spread, so the very first fix is as wrong as any.
    """

    def __init__(self, drift_stddev: float, drift_time: float, jitter_stddev: float,
                 rng: random.Random):
        if drift_stddev < 0.0 or drift_time < 0.0 or jitter_stddev < 0.0:
            raise ValueError('the GPS error needs drift_stddev, drift_time and jitter_stddev '
                             '>= 0')
        self.drift_stddev = drift_stddev
        self.drift_time = drift_time
        self.jitter_stddev = jitter_stddev
        self.rng = rng
        self.drift = [rng.gauss(0.0, drift_stddev), rng.gauss(0.0, drift_stddev)]

    @property
    def stddev(self) -> float:
        """Return the whole error's spread on each axis (m): what the receiver reports."""
        return math.hypot(self.drift_stddev, self.jitter_stddev)

    def step(self, dt: float) -> Tuple[float, float]:
        """Move the drift on by dt seconds; return this fix's (east, north) error, m."""
        keep = math.exp(-dt / self.drift_time) if self.drift_time > 0.0 else 0.0
        fresh = self.drift_stddev * math.sqrt(1.0 - keep * keep)
        self.drift = [keep * d + self.rng.gauss(0.0, fresh) for d in self.drift]
        return (self.drift[0] + self.rng.gauss(0.0, self.jitter_stddev),
                self.drift[1] + self.rng.gauss(0.0, self.jitter_stddev))


def battery_voltage(hours_driven: float, full: float, empty: float, hours: float) -> float:
    """Return a steady drain from full to empty over `hours` of driving, never below empty."""
    left = max(0.0, 1.0 - hours_driven / hours)
    return empty + (full - empty) * left


@dataclass
class Camera:
    """A pinhole camera on the car: where it sits, and its lens."""

    xyz: Sequence[float]   # m, on the car (camera_left_link in vehicle.yaml)
    width: int
    height: int
    focal: float           # px
    height_above_ground: float
    pitch: float = 0.0     # rad, + = looking down (vehicle.yaml's rpy[1])

    def pixel(self, forward: float, left: float, up: float) -> Tuple[float, float, float]:
        """
        Return (u, v, depth) for a point forward, left and up of the camera (m, level axes).

        depth is the distance along the lens's axis: the point is behind the camera if it's
        not positive.
        """
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        depth = c * forward - s * up
        rise = s * forward + c * up
        if depth <= 0.0:
            return math.nan, math.nan, depth
        return (self.width / 2.0 - self.focal * left / depth,
                self.height / 2.0 - self.focal * rise / depth, depth)


@dataclass
class SignView:
    """A stop sign in the picture: its centre (px), its size (px) and how far it is (m)."""

    u: float
    v: float
    size: float
    distance: float
    foot_v: Optional[float] = None   # px: the row where its post meets the ground


def sign_in_view(cam: Camera, truth: Truth, sign_xy: Sequence[float], sign_height: float,
                 sign_size: float) -> Optional[SignView]:
    """
    Return where a stop sign lands in the camera's picture, or None if the camera can't see it.

    The sign is a sign_size-metre plate whose centre stands sign_height metres up at sign_xy
    on the map.
    """
    cx, cy = offset(truth, cam.xyz)
    dx, dy = sign_xy[0] - cx, sign_xy[1] - cy
    c, s = math.cos(truth.yaw), math.sin(truth.yaw)
    forward, left = c * dx + s * dy, -s * dx + c * dy
    u, v, depth = cam.pixel(forward, left, sign_height - cam.height_above_ground)
    if depth < 1.0:
        return None
    size = cam.focal * sign_size / depth
    if not (0.0 <= u <= cam.width and 0.0 <= v <= cam.height) or size < 4.0:
        return None
    _, foot_v, _ = cam.pixel(forward, left, -cam.height_above_ground)
    return SignView(u, v, size, math.hypot(forward, left), foot_v)


@dataclass
class Road:
    """
    The bench world's road: one lane between two painted lines, in the map frame.

    It runs east along y, straight; with a bend_radius, it bends from bend_x on, round a
    half circle.
    """

    y: float = 0.0              # m north of the datum: the lane's centre line
    lane_width: float = 3.5     # m between the lines' centres
    line_width: float = 0.15    # m of paint
    shoulder: float = 0.5       # m of tarmac outside each line
    bend_x: float = 0.0         # m east: where the bend starts
    bend_radius: float = 0.0    # m: + bends left (north), - right, 0 straight

    def _centre(self) -> Tuple[float, float]:
        return self.bend_x, self.y + self.bend_radius

    def lateral(self, x, y):
        """Return how far (x, y) is left of the lane's centre line (m). Takes arrays too."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        across = y - self.y
        if self.bend_radius:
            r = self.bend_radius
            ox, oy = self._centre()
            arc = math.copysign(1.0, r) * (abs(r) - np.hypot(x - ox, y - oy))
            across = np.where(x > self.bend_x, arc, across)
        return across

    def pose(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        """Return the car's (offset, heading error): m left of the centre, rad left of the road."""
        direction = 0.0
        if self.bend_radius and x > self.bend_x:
            ox, oy = self._centre()
            direction = math.atan2(y - oy, x - ox) + math.copysign(math.pi / 2, self.bend_radius)
        error = (yaw - direction + math.pi) % (2.0 * math.pi) - math.pi
        return float(self.lateral(x, y)), error


def pairs(flat: Sequence[float]) -> List[Tuple[float, float]]:
    """Read a flat [x1, y1, x2, y2, ...] parameter as (x, y) pairs."""
    if len(flat) % 2:
        raise ValueError('stop_signs needs x, y pairs')
    return [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]


Box = Tuple[float, float, float, float]   # x1, y1, x2, y2 on the map, as in make_map's specs


def boxes(flat: Sequence[float]) -> List[Box]:
    """Read a flat [x1, y1, x2, y2, ...] parameter as boxes, corners in any order."""
    if len(flat) % 4:
        raise ValueError('boxes needs four numbers per box: x1, y1, x2, y2')
    return [(min(flat[i], flat[i + 2]), min(flat[i + 1], flat[i + 3]),
             max(flat[i], flat[i + 2]), max(flat[i + 1], flat[i + 3]))
            for i in range(0, len(flat), 4)]


@dataclass
class Radar:
    """A radar on the car: where it sits, and how far and wide it sees."""

    xyz: Sequence[float]   # m, on the car (radar_link in vehicle.yaml)
    yaw: float             # rad, how it's turned on the car
    max_range: float       # m
    fov: float             # rad, the whole width it sees, centred ahead
    ray_step: float        # rad between the rays this model traces


def ray_hits_box(ox: float, oy: float, dx: float, dy: float, box: Box) -> Optional[float]:
    """Return how far a ray from (ox, oy), along the unit vector (dx, dy), goes to the box."""
    near, far = 0.0, math.inf
    for o, d, lo, hi in ((ox, dx, box[0], box[2]), (oy, dy, box[1], box[3])):
        if abs(d) < 1e-12:
            if not lo <= o <= hi:
                return None
            continue
        t1, t2 = sorted(((lo - o) / d, (hi - o) / d))
        near, far = max(near, t1), min(far, t2)
        if near > far:
            return None
    return near


def radar_returns(radar: Radar, truth: Truth, obstacles: Sequence[Box], height: float,
                  rng: random.Random, keep: float = 0.8, range_noise: float = 0.05,
                  height_noise: float = 0.1) -> List[Tuple[float, float, float, float]]:
    """
    Return the points a radar sees on the boxes: (x, y, z, radial speed) in radar_link.

    A sparse, noisy radar: rays fan out across the field of view, each stops at the nearest
    box face (so a near box hides one behind it), and only `keep` of them return a point.
    Each point gets range noise, and a height anywhere on the box (0.1 m to `height`) plus
    noise, as a radar's elevation is poor. The ground returns nothing. The radial speed is
    the car's own, seen from the radar (negative: closing). x forward, y left, z up.
    """
    ox, oy = offset(truth, radar.xyz)
    heading = truth.yaw + radar.yaw
    points = []
    rays = int(radar.fov / radar.ray_step) + 1
    for i in range(rays):
        a = -radar.fov / 2.0 + i * radar.ray_step
        dx, dy = math.cos(heading + a), math.sin(heading + a)
        hits = [r for r in (ray_hits_box(ox, oy, dx, dy, b) for b in obstacles)
                if r is not None and 0.0 < r <= radar.max_range]
        if not hits or rng.random() > keep:
            continue
        r = min(hits) + rng.gauss(0.0, range_noise)
        z = rng.uniform(0.1, height) - radar.xyz[2] + rng.gauss(0.0, height_noise)
        points.append((r * math.cos(a), r * math.sin(a), z, -truth.speed * math.cos(a)))
    return points


@dataclass
class Lidar:
    """A spinning lidar on the car: where it sits, its lasers' angles, how far it sees."""

    xyz: Sequence[float]          # m, on the car (lidar_link in vehicle.yaml)
    elevations: Sequence[float]   # rad above the horizon, one per laser
    max_range: float              # m, along the beam


def lidar_ranges(lidar: Lidar, truth: Truth, obstacles: Sequence[Box], height: float,
                 bearing: float) -> List[float]:
    """
    Return each laser's distance along its beam at a bearing, 0 where it sees nothing.

    bearing: rad, clockwise from the car's heading, as a spinning lidar counts it. A laser
    stops at the nearest box face it meets between the ground and the box's `height`, or
    else at the flat ground (lasers aimed down), or sees nothing (lasers aimed up, or past
    max_range). Each box face is a wall: a beam that clears its top flies on.
    """
    ox, oy = offset(truth, lidar.xyz)
    heading = truth.yaw - bearing
    dx, dy = math.cos(heading), math.sin(heading)
    walls = sorted(r for r in (ray_hits_box(ox, oy, dx, dy, b) for b in obstacles)
                   if r is not None and r > 0.0)
    mount = lidar.xyz[2]
    out = []
    for elevation in lidar.elevations:
        slope = math.tan(elevation)
        across = None              # m along the ground to what this laser hits
        for r in walls:
            if 0.0 <= mount + r * slope <= height:
                across = r
                break
        if slope < 0.0:
            ground = mount / -slope
            across = ground if across is None else min(across, ground)
        beam = across / math.cos(elevation) if across is not None else 0.0
        out.append(beam if beam <= lidar.max_range else 0.0)
    return out
