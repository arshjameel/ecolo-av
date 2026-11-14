"""
A small tracker: one constant-velocity Kalman filter per thing. No ROS.

AB3DMOT's finding (Weng et al., 2020): a plain Kalman filter per object and a simple
association is a strong baseline. Each thing's state is where it is and how fast it moves
over the ground, in odom:

    state  [x, y, vx, vy]           predict   x += vx dt, y += vy dt  (speed kept)
    lidar  measures x, y            radar     measures the speed along its ray

Association is greedy nearest first inside a gate: the closest (thing, group) pair is
matched, then the next closest, until none is closer than `gate`. A group nobody claims
starts a new thing; a thing nobody saw coasts on its speed.

Memory: a thing seen `confirm` times is kept, coasting on its speed (taken as parked when
slower than `still_speed`), for `keep` s after it was last seen, so
what went into the blind zone in front of the bumper (the roof lidar can't see a 0.5 m
box within about 2.5 m) is still in the list. Inside `blind_range` m of the car it is kept
for `keep_blind` s: there the lidar can't see it, so not seeing it says nothing.
A thing seen fewer times is dropped after `keep_tentative` s (noise).

Names come later: `kind` is Thing.UNKNOWN until camera-to-lidar calibration exists
(Q22). The hook is `Tracker.name`: give it a function from a thing to a name.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable, Iterable, Optional

import numpy as np

from .groups import Group

UNKNOWN = 'unknown'


@dataclass(frozen=True)
class TrackerShape:
    """The tracker's settings."""

    gate: float = 2.0             # m: a group farther than this from a thing is another thing
    position_noise: float = 0.15  # m: one sigma, a group's middle as the lidar sees it
    accel_noise: float = 1.5      # m/s²: one sigma, how hard a thing may change its speed
    radar_noise: float = 0.3      # m/s: one sigma, the radar's speed along its ray
    radar_gate: float = 2.0       # m: a radar cluster this close to a thing gives it a speed
    confirm: int = 3              # times seen before a thing is kept through gaps
    keep: float = 3.0             # s: a confirmed thing stays this long unseen
    keep_blind: float = 30.0      # s: ... and this long while it is inside blind_range
    blind_range: float = 4.0      # m from the car (base_link): where the lidar can't see low
    keep_tentative: float = 0.3   # s: a thing seen fewer than `confirm` times
    start_speed_sigma: float = 3.0  # m/s: how little a new thing's speed is known
    still_speed: float = 0.5      # m/s: an unseen thing slower than this is taken as parked


@dataclass
class Track:
    """One thing, followed."""

    id: int                           # noqa: A003 - as the Thing message names it
    x: np.ndarray                     # [x, y, vx, vy]
    p: np.ndarray                     # its 4 x 4 covariance
    length: float
    width: float
    height: float
    last_seen: float                  # s
    seen: int = 1
    radar: bool = False
    coasting: bool = False
    kind: str = UNKNOWN

    @property
    def position_sigma(self) -> float:
        return math.sqrt(max(0.0, (self.p[0, 0] + self.p[1, 1]) / 2.0))

    def confidence(self, now: float, shape: TrackerShape) -> float:
        """0..1: how often it was seen (up to 2 x confirm), less as it goes unseen."""
        seen = min(1.0, self.seen / (2.0 * shape.confirm))
        unseen = max(0.0, now - self.last_seen)
        return seen * max(0.0, 1.0 - unseen / shape.keep_blind)


def predict(x: np.ndarray, p: np.ndarray, dt: float, accel_noise: float):
    """Return the state and covariance moved on by dt (s) at constant speed."""
    f = np.eye(4)
    f[0, 2] = f[1, 3] = dt
    # White acceleration noise (the discrete constant-velocity model).
    q1 = np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]]) * accel_noise ** 2
    q = np.zeros((4, 4))
    q[np.ix_([0, 2], [0, 2])] = q1
    q[np.ix_([1, 3], [1, 3])] = q1
    return f @ x, f @ p @ f.T + q


def update(x: np.ndarray, p: np.ndarray, z: np.ndarray, h: np.ndarray, r: np.ndarray):
    """Return the state and covariance after a linear measurement z = h x (noise r)."""
    s = h @ p @ h.T + r
    k = p @ h.T @ np.linalg.inv(s)
    x = x + k @ (z - h @ x)
    # Joseph's form: stays symmetric and positive.
    i_kh = np.eye(4) - k @ h
    return x, i_kh @ p @ i_kh.T + k @ r @ k.T


@dataclass(frozen=True)
class RadarCluster:
    """One radar cluster, in odom: where, and its speed over the ground along the ray."""

    x: float
    y: float
    ray_x: float          # unit vector from the radar to the cluster, in odom
    ray_y: float
    speed: float          # m/s over the ground along the ray (+ away from the radar)


def ground_radial_speed(measured: float, ray: tuple[float, float],
                        car_velocity: tuple[float, float]) -> float:
    """
    Return a radar speed over the ground along its ray, from the speed it measured.

    The radar measures the speed along the ray relative to itself (negative while
    closing): ray · (v_thing - v_car). Adding ray · v_car leaves ray · v_thing. The car's
    turning is left out, as the radar stand-in leaves it out.
    """
    return measured + ray[0] * car_velocity[0] + ray[1] * car_velocity[1]


@dataclass
class Tracker:
    """Every thing near the car, in odom."""

    shape: TrackerShape = TrackerShape()
    tracks: list[Track] = field(default_factory=list)
    time: Optional[float] = None
    next_id: int = 1
    # The naming hook (Q22): a function from a Track to a name, once the camera's boxes
    # can be matched to the lidar's cubes. None: every thing stays UNKNOWN.
    name: Optional[Callable[[Track], str]] = None

    def step(self, now: float, groups: Iterable[Group], car: tuple[float, float]) -> None:
        """
        Take one cloud's groups (in odom) seen at `now` (s), the car at `car` (x, y, odom).

        A cloud older than the last is dropped: a Kalman filter can't go back in time.
        """
        groups = list(groups)
        if self.time is not None and now < self.time:
            return
        self._advance(now)
        sh = self.shape
        r = np.eye(2) * sh.position_noise ** 2
        h = np.zeros((2, 4))
        h[0, 0] = h[1, 1] = 1.0
        pairs = sorted(
            (math.hypot(t.x[0] - g.x, t.x[1] - g.y), ti, gi)
            for ti, t in enumerate(self.tracks) for gi, g in enumerate(groups))
        used_t, used_g = set(), set()
        for distance, ti, gi in pairs:
            if distance > sh.gate:
                break
            if ti in used_t or gi in used_g:
                continue
            used_t.add(ti)
            used_g.add(gi)
            t, g = self.tracks[ti], groups[gi]
            t.x, t.p = update(t.x, t.p, np.array([g.x, g.y]), h, r)
            # The outline: a little of each new look, so a half-seen side doesn't jump it.
            t.length += 0.3 * (g.length - t.length)
            t.width += 0.3 * (g.width - t.width)
            t.height = max(g.height, 0.7 * t.height + 0.3 * g.height)
            t.last_seen, t.seen, t.coasting = now, t.seen + 1, False
        for ti, t in enumerate(self.tracks):
            if ti not in used_t:
                t.coasting = True
                # The lidar's view of a parked thing shifts as the car moves, which
                # reads as a little speed; coasting on it would walk the thing away.
                if math.hypot(t.x[2], t.x[3]) < sh.still_speed:
                    t.x = np.array([t.x[0], t.x[1], 0.0, 0.0])
        for gi, g in enumerate(groups):
            if gi not in used_g:
                self.tracks.append(self._start(g, now))
        self.expire(now, car)
        if self.name is not None:
            for t in self.tracks:
                t.kind = self.name(t)

    def radar(self, now: float, clusters: Iterable[RadarCluster]) -> None:
        """Give each nearest confirmed thing within radar_gate its radar speed along the ray."""
        if self.time is None:
            return
        sh = self.shape
        r = np.array([[sh.radar_noise ** 2]])
        for c in clusters:
            best, best_d = None, sh.radar_gate
            for t in self.tracks:
                if t.seen < sh.confirm:
                    continue
                at = t.x[:2] + t.x[2:] * max(0.0, now - self.time)
                d = math.hypot(at[0] - c.x, at[1] - c.y)
                if d <= best_d:
                    best, best_d = t, d
            if best is not None:
                h = np.array([[0.0, 0.0, c.ray_x, c.ray_y]])
                best.x, best.p = update(best.x, best.p, np.array([c.speed]), h, r)
                best.radar = True

    def expire(self, now: float, car: tuple[float, float]) -> None:
        """Drop the things unseen for too long (the blind zone remembers longer)."""
        sh = self.shape

        def kept(t: Track) -> bool:
            unseen = now - t.last_seen
            if t.seen < sh.confirm:
                return unseen <= sh.keep_tentative
            at = t.x[:2] + t.x[2:] * max(0.0, now - (self.time or now))
            if math.hypot(at[0] - car[0], at[1] - car[1]) <= sh.blind_range:
                return unseen <= sh.keep_blind
            return unseen <= sh.keep

        self.tracks = [t for t in self.tracks if kept(t)]

    def at(self, now: float) -> list[Track]:
        """
        Return the confirmed things as they will be at `now`, without changing the tracker.

        The places move on their speed from the last cloud: the list a screen or planner
        reads between clouds.
        """
        dt = 0.0 if self.time is None else max(0.0, now - self.time)
        out = []
        for t in self.tracks:
            if t.seen < self.shape.confirm:
                continue
            x, p = predict(t.x, t.p, dt, self.shape.accel_noise) if dt else (t.x, t.p)
            out.append(Track(id=t.id, x=x, p=p, length=t.length, width=t.width,
                             height=t.height, last_seen=t.last_seen, seen=t.seen,
                             radar=t.radar, coasting=t.coasting or dt > 0.5, kind=t.kind))
        return out

    def _advance(self, now: float) -> None:
        if self.time is not None:
            dt = now - self.time
            if dt > 0:
                for t in self.tracks:
                    t.x, t.p = predict(t.x, t.p, dt, self.shape.accel_noise)
        self.time = now

    def _start(self, g: Group, now: float) -> Track:
        sh = self.shape
        p = np.diag([sh.position_noise ** 2] * 2 + [sh.start_speed_sigma ** 2] * 2)
        track = Track(id=self.next_id, x=np.array([g.x, g.y, 0.0, 0.0]), p=p,
                      length=g.length, width=g.width, height=g.height, last_seen=now)
        self.next_id += 1
        return track
