"""
The geofence: an area the computer's drivers may not leave. No ROS here, so the tests run anywhere.

The fence is a `Circle` round the car's marked start spot or a site's `Polygon`, in the map
frame. `Geofence` takes each global pose and GPS arrival, and says whether twist_mux's
geofence lock should be on: when the car's stopping distance, forward or back, straight
or at full lock, brings a corner within `margin` (plus the pose's spread) of the edge;
when the car is outside (latched); when the pose jumped (latched); and while the pose
can't be trusted.

The stopping distance is RSS's, with ego's speed cap:

    the car may speed up at a for rho s (up to its cap), then brakes at b:
    d = v*t1 + a*t1^2/2 + v1*(rho - t1) + v1^2 / (2*b)      v1 = min(v + a*rho, max(v, cap))

rho and b come from the parts that make them (`reaction_time`, `braking`, `motion_from`),
read from ego.yaml, vehicle.yaml and the backend's config by the launch file.

Design: docs/designs/2026-10-29_geofence.md. Units: m, s, rad, m/s, m/s^2.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

Point = Tuple[float, float]

STARTING, OK, NEAR, OUTSIDE, LOST, JUMP, COASTING, STANDS = (
    'STARTING', 'OK', 'NEAR', 'OUTSIDE', 'LOST', 'JUMP', 'COASTING', 'STANDS')
LATCHED = (OUTSIDE, JUMP)

# spine-core's own command watchdog (firmware/spine, cmd_timeout_ms): it brakes this long
# after the last DRIVE_CMD, on top of spine_bridge's cmd_timeout (spine.yaml).
SPINE_WATCHDOG = 0.1    # s
# gazebo_adapter's cmd_timeout default (aav_gazebo gazebo_adapter.py): Gazebo's stand-in.
GAZEBO_TIMEOUT = 0.2    # s
BACKENDS = ('spine', 'uart', 'gazebo', 'gazebo_esp32')
SAMPLE = 0.5            # m between the points checked along each stopping path


def _finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


# --- the fence -------------------------------------------------------------------------

class Circle:
    """A round fence: `radius` m round (cx, cy)."""

    def __init__(self, cx: float, cy: float, radius: float):
        if not _finite(cx, cy, radius) or radius <= 0.0:
            raise ValueError('a circle needs a finite centre and a radius > 0')
        self.cx, self.cy, self.radius = cx, cy, radius

    def depth(self, x: float, y: float) -> float:
        """Return how far (x, y) is inside the fence (m); negative outside."""
        return self.radius - math.hypot(x - self.cx, y - self.cy)

    def outline(self, n: int = 36) -> List[Point]:
        """Return n points round the circle, for drawing."""
        return [(self.cx + self.radius * math.cos(2 * math.pi * i / n),
                 self.cy + self.radius * math.sin(2 * math.pi * i / n)) for i in range(n)]


class Polygon:
    """A fence with corners (x, y), in order round it, either way. Not self-crossing."""

    def __init__(self, corners: Sequence[Point]):
        points = [(float(x), float(y)) for x, y in corners]
        if len(points) < 3 or not all(_finite(x, y) for x, y in points):
            raise ValueError('a polygon needs at least 3 finite corners')
        area = sum(x0 * y1 - x1 * y0
                   for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1])) / 2.0
        if abs(area) < 1.0:
            raise ValueError(f'the polygon is {abs(area):.2f} m^2: is it a line?')
        self.corners = points

    @classmethod
    def from_flat(cls, values: Sequence[float]) -> 'Polygon':
        """Make one from [x0, y0, x1, y1, ...], as a ROS parameter holds it."""
        if len(values) % 2:
            raise ValueError('a polygon parameter needs pairs: x0, y0, x1, y1, ...')
        return cls(list(zip(values[0::2], values[1::2])))

    def _edges(self):
        return zip(self.corners, self.corners[1:] + self.corners[:1])

    def contains(self, x: float, y: float) -> bool:
        inside = False
        for (x0, y0), (x1, y1) in self._edges():
            if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
                inside = not inside
        return inside

    def depth(self, x: float, y: float) -> float:
        """Return how far (x, y) is inside the fence (m); negative outside."""
        nearest = min(_to_segment(x, y, a, b) for a, b in self._edges())
        return nearest if self.contains(x, y) else -nearest

    def outline(self) -> List[Point]:
        return list(self.corners)


def _to_segment(x: float, y: float, a: Point, b: Point) -> float:
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def site_fence(site: str, corners: Sequence[float], site_datum: Sequence[float],
               datum: Sequence[float], tolerance: float = 1e-7) -> Polygon:
    """
    Return a site's polygon, if it was drawn for the datum the map frame uses now.

    corners: map frame [x0, y0, ...]. site_datum: the [lat, lon] it was drawn for.
    datum: navsat.yaml's [lat, lon, ...]. A site drawn on another datum is somewhere else
    on the ground: refused, rather than a fence in the wrong place.
    """
    if len(site_datum) < 2:
        raise ValueError(f'site {site}: no datum [lat, lon] stored with it')
    if len(datum) < 2 or any(abs(a - b) > tolerance for a, b in zip(site_datum[:2], datum[:2])):
        raise ValueError(f'site {site} was drawn for the datum {list(site_datum[:2])}, but the '
                         f'map frame uses {list(datum[:2])}: redraw it, or set the datum back')
    return Polygon.from_flat(corners)


# --- how far the car goes before it stands still ----------------------------------------

@dataclass(frozen=True)
class Motion:
    """What the car may do before it stops: from ego.yaml, vehicle.yaml and the backend."""

    reaction_time: float      # s from the edge coming near to the brake biting
    braking: float            # m/s^2 counted on
    max_accel: float          # m/s^2: ego's max_acceleration, while the lock reacts
    max_speed: float          # m/s forward: ego's cap
    max_reverse_speed: float  # m/s backward
    max_curvature: float      # 1/m at full lock: tan(max_steering_angle) / wheelbase

    def __post_init__(self):
        for name in ('reaction_time', 'max_accel', 'max_speed', 'max_reverse_speed',
                     'max_curvature'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f'{name} must be a finite number >= 0')
        if not (math.isfinite(self.braking) and self.braking > 0.0):
            raise ValueError('braking must be a finite number > 0')


def reaction_time(tick: float, ego_timeout: float, backend_stale: float, brake_lag: float,
                  pose_lag: float, brake_dead_time: float = 0.0) -> float:
    """
    Return rho: every delay between the car's edge coming near and the brake biting (s).

    tick: the geofence's own period; ego_timeout: ego's cmd_timeout (twist_mux goes quiet,
    ego brakes after this); backend_stale: the backend's own command timeout, in case ego
    is the part that died; brake_lag, brake_dead_time: vehicle.yaml's actuators; pose_lag:
    how old the global pose is when it arrives (the EKF, the network).
    """
    parts = (tick, ego_timeout, backend_stale, brake_lag, pose_lag, brake_dead_time)
    if not all(math.isfinite(p) and p >= 0.0 for p in parts):
        raise ValueError('every delay must be a finite number of seconds >= 0')
    return sum(parts)


def braking(ego_max_deceleration: float, max_brake: float) -> float:
    """Return b: half of the softer of ego's deceleration and the brake's (m/s^2)."""
    return min(ego_max_deceleration, max_brake) / 2.0


def backend_stale(backend: str, spine: Optional[Mapping] = None,
                  legacy: Optional[Mapping] = None) -> float:
    """Return how long the backend drives on a command before it stops by itself (s)."""
    if backend == 'spine':
        return float(spine['cmd_timeout']) + SPINE_WATCHDOG
    if backend in ('uart', 'gazebo_esp32'):
        return float(legacy['cmd_timeout'])
    if backend == 'gazebo':
        return GAZEBO_TIMEOUT
    raise ValueError(f'backend {backend}: pick one of {", ".join(BACKENDS)}')


def coasts(backend: str, legacy: Optional[Mapping], actuators: Mapping) -> bool:
    """
    Return whether the backend can't be counted on to brake to a stop.

    legacy_uart_bridge's speed_loop.mode `open` brakes only below stop_speed: from 1 m/s
    the car coasts on drag, about 10 m. Until test 4.3 measures how it really stops
    (vehicle.yaml actuators `open_loop_decel`), the geofence keeps every program held.
    Gazebo's ESP32 stand-in is a simulator: never counted as coasting.
    """
    return (backend == 'uart' and legacy.get('speed_loop.mode', 'open') == 'open'
            and not actuators.get('open_loop_decel'))


def motion_from(ego: Mapping, vehicle_cfg: Mapping, backend: str, rate_hz: float,
                pose_lag: float, spine: Optional[Mapping] = None,
                legacy: Optional[Mapping] = None,
                max_speed: Optional[float] = None) -> Tuple[Motion, bool]:
    """
    Work out the Motion and whether the backend coasts, from the configs (dicts).

    ego: ego.yaml's parameters; vehicle_cfg: the whole vehicle.yaml; spine, legacy:
    spine_bridge's and legacy_uart_bridge's parameters; max_speed: a launch's lower cap.
    """
    v, act = vehicle_cfg['vehicle'], vehicle_cfg['actuators']
    rho = reaction_time(1.0 / rate_hz, float(ego['cmd_timeout']),
                        backend_stale(backend, spine, legacy), float(act['brake_lag']),
                        pose_lag, float(act.get('brake_dead_time', 0.0)))
    coasting = coasts(backend, legacy, act)
    brake = float(act['max_brake'])
    if backend == 'uart' and not coasting and act.get('open_loop_decel'):
        brake = min(brake, float(act['open_loop_decel']))
    top = float(ego['max_speed']) if max_speed is None else float(max_speed)
    motion = Motion(reaction_time=rho,
                    braking=braking(float(ego['max_deceleration']), brake),
                    max_accel=float(ego['max_acceleration']),
                    max_speed=top,
                    max_reverse_speed=float(v['max_reverse_speed']),
                    max_curvature=math.tan(float(v['max_steering_angle'])) / float(v['wheelbase']))
    return motion, coasting


def stopping_distance(speed: float, rho: float, accel: float, brake: float,
                      cap: float = math.inf) -> float:
    """Return how far the car goes before it stands still (m): RSS, with the speed cap."""
    v = abs(speed)
    top = max(v, cap)
    t1 = min(rho, (top - v) / accel) if accel > 0.0 else 0.0
    v1 = v + accel * t1
    return v * t1 + 0.5 * accel * t1 * t1 + v1 * (rho - t1) + v1 * v1 / (2.0 * brake)


def along_arc(x: float, y: float, yaw: float, s: float, k: float) -> Tuple[float, float, float]:
    """Return the pose after s m (negative: backward) on curvature k (1/m, + left)."""
    if abs(k) < 1e-9:
        return x + s * math.cos(yaw), y + s * math.sin(yaw), yaw
    turn = k * s
    return (x + (math.sin(yaw + turn) - math.sin(yaw)) / k,
            y - (math.cos(yaw + turn) - math.cos(yaw)) / k, yaw + turn)


# --- the lock --------------------------------------------------------------------------

@dataclass
class Settings:
    """The geofence's limits (config/geofence.yaml)."""

    margin: float = 1.0          # m kept between the stopping point and the edge
    hysteresis: float = 0.5      # m further in before NEAR lets go
    max_std: float = 3.0         # m: a pose spread (1 sigma) over this can't be trusted
    max_age: float = 1.0         # s without a global pose: lost
    gps_max_age: float = 2.0     # s without a GPS position into the EKF: lost
    settle_time: float = 3.0     # s of good poses before the fence is used
    jump_margin: float = 2.0     # m a pose may move beyond speed x time before it's a jump

    def __post_init__(self):
        for name in ('margin', 'hysteresis', 'settle_time', 'jump_margin'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f'{name} must be a finite number >= 0')
        for name in ('max_std', 'max_age', 'gps_max_age'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be a finite number > 0')


@dataclass
class Pose:
    """The global EKF's pose: map frame, base_link's position and heading, forward speed."""

    x: float
    y: float
    yaw: float
    speed: float        # m/s along base_link's x, negative reversing
    std: float          # m: the position's spread, 1 sigma

    def good(self, max_std: float) -> bool:
        return _finite(self.x, self.y, self.yaw, self.speed, self.std) and self.std <= max_std


class Geofence:
    """
    Decide the geofence lock from each pose and each GPS arrival.

    footprint: the car's corners in base_link (aav_description vehicle.footprint_points).
    gps: whether a GPS position must keep reaching the EKF (always, but on stands).
    coasting: the backend can't brake to a stop (`coasts`): every program stays held.
    stands: the wheels are off the ground: no fence, the lock stays open (said loudly).
    Times: any steady clock, in s.
    """

    def __init__(self, footprint: Sequence[Point], fence, motion: Motion,
                 settings: Settings = Settings(), gps: bool = True, coasting: bool = False,
                 stands: bool = False):
        self.footprint = [(float(x), float(y)) for x, y in footprint]
        if not self.footprint:
            raise ValueError('the footprint needs corners')
        self.fence, self.motion, self.settings = fence, motion, settings
        self.gps, self.coasting, self.stands = gps, coasting, stands
        self.pose: Optional[Pose] = None
        self.pose_time: Optional[float] = None
        self.gps_time: Optional[float] = None
        self.good_since: Optional[float] = None
        self.latched: Optional[Tuple[str, str]] = None    # (OUTSIDE or JUMP, why)
        self.near_std: Optional[float] = None             # sigma when NEAR began
        self.state, self.why = STARTING, 'no global pose yet'

    def update(self, now: float, pose: Pose) -> None:
        """Take a global pose that arrived at `now`."""
        s = self.settings
        last, last_time = self.pose, self.pose_time
        self.pose, self.pose_time = pose, now
        if not pose.good(s.max_std):
            self.good_since = None
            return
        settled = self.good_since is not None and now - self.good_since >= s.settle_time
        if self.good_since is None:
            self.good_since = now
        if settled and last is not None and last.good(s.max_std) and self.latched is None:
            step = math.hypot(pose.x - last.x, pose.y - last.y)
            allowed = max(abs(last.speed), abs(pose.speed)) * (now - last_time) + s.jump_margin
            if step > allowed:
                self.latched = (JUMP, f'the global pose jumped {step:.1f} m in '
                                      f'{now - last_time:.1f} s (at most {allowed:.1f} m)')

    def gps_arrived(self, now: float) -> None:
        """Take the arrival of a GPS position into the EKF (/localization/gps/odom)."""
        self.gps_time = now

    def stopping(self, speed: float) -> Tuple[float, float]:
        """Return the stopping distances (m) forward and backward, from |speed|."""
        m = self.motion
        return (stopping_distance(speed, m.reaction_time, m.max_accel, m.braking, m.max_speed),
                stopping_distance(speed, m.reaction_time, m.max_accel, m.braking,
                                  m.max_reverse_speed))

    def points(self, pose: Pose) -> List[Point]:
        """
        Return the car's corners now, and along every path it could stop on.

        Forward and backward, straight and at full lock either way, each its stopping
        distance long, checked every SAMPLE m.
        """
        poses = [(pose.x, pose.y, pose.yaw)]
        k = self.motion.max_curvature
        for d, sign in zip(self.stopping(pose.speed), (1.0, -1.0)):
            n = max(1, math.ceil(d / SAMPLE))
            for curvature in (-k, 0.0, k):
                poses += [along_arc(pose.x, pose.y, pose.yaw, sign * d * i / n, curvature)
                          for i in range(1, n + 1)]
        out = []
        for x, y, yaw in poses:
            c, s = math.cos(yaw), math.sin(yaw)
            out += [(x + fx * c - fy * s, y + fx * s + fy * c) for fx, fy in self.footprint]
        return out

    def check(self, now: float) -> Tuple[bool, str, str]:
        """Return (locked, state, why) at `now`."""
        s = self.settings
        if self.stands:
            return self._say(False, STANDS, 'stands mode: NO FENCE. The wheels must be off '
                                            'the ground')
        if self.coasting:
            return self._say(True, COASTING, 'the backend coasts (legacy speed_loop open): '
                                             'no program may drive until test 4.3 measures '
                                             'its stop (vehicle.yaml open_loop_decel)')
        pose = self.pose
        if pose is None:
            return self._say(True, STARTING, 'no global pose yet')
        if now - self.pose_time > s.max_age:
            return self._say(True, LOST, f'no global pose for {now - self.pose_time:.1f} s')
        if not pose.good(s.max_std):
            return self._say(True, LOST, f'the global pose is unsure: {pose.std:.1f} m '
                                         f'(over {s.max_std:.1f} m) or not a number')
        if self.gps and (self.gps_time is None or now - self.gps_time > s.gps_max_age):
            age = 'none yet' if self.gps_time is None else f'none for {now - self.gps_time:.1f} s'
            return self._say(True, LOST, f'no GPS position into the EKF ({age}): it drifts')
        if now - self.good_since < s.settle_time:
            return self._say(True, STARTING, 'waiting for the global pose to settle')
        here = self.fence.depth(pose.x, pose.y)
        if here < 0.0 and (self.latched is None or self.latched[0] != OUTSIDE):
            self.latched = (OUTSIDE, f'the car went {-here:.1f} m outside the fence')
        if self.latched is not None:
            state, why = self.latched
            where = (f'{-here:.1f} m outside: a person drives it back in, then clears it'
                     if here < 0.0 else f'{here:.1f} m inside: a person clears it')
            return self._say(True, state, f'{why}; now {where}')
        std = pose.std if self.near_std is None else max(pose.std, self.near_std)
        nearest = min(self.fence.depth(x, y) for x, y in self.points(pose))
        need = s.margin + std + (s.hysteresis if self.near_std is not None else 0.0)
        if nearest < need:
            if self.near_std is None:
                self.near_std = pose.std
            forward, back = self.stopping(pose.speed)
            return self._say(True, NEAR, f'a stopping path ({forward:.1f} m ahead, {back:.1f} m '
                                         f'back) comes {nearest:.1f} m from the edge (needs '
                                         f'{need:.1f} m) at {pose.speed:.1f} m/s')
        self.near_std = None
        return self._say(False, OK, f'{nearest:.1f} m from the edge at {pose.speed:.1f} m/s')

    def clear(self, now: float) -> Tuple[bool, str]:
        """Forget OUTSIDE or JUMP: a person says the car is back in. Refused while not."""
        if self.latched is None:
            return True, 'nothing to clear'
        pose = self.pose
        if (pose is None or now - self.pose_time > self.settings.max_age
                or not pose.good(self.settings.max_std)):
            return False, 'no good global pose: the fence cannot say where the car is'
        here = self.fence.depth(pose.x, pose.y)
        if here < 0.0:
            return False, f'the car is still {-here:.1f} m outside the fence'
        state = self.latched[0]
        self.latched = None
        return True, f'cleared {state}: the car is {here:.1f} m inside'

    def _say(self, locked: bool, state: str, why: str) -> Tuple[bool, str, str]:
        self.state, self.why = state, why
        return locked, state, why


def describe(motion: Motion) -> Dict[str, float]:
    """Return the stopping distance at the cap, and its parts, for the log."""
    return {'forward_at_cap': stopping_distance(motion.max_speed, motion.reaction_time,
                                                motion.max_accel, motion.braking,
                                                motion.max_speed),
            'reaction_time': motion.reaction_time, 'braking': motion.braking}
