r"""
The behaviour states: why the car should slow, and the speed cap that slows it.

No ROS here (behaviour_node.py is the node). docs/stopping.md, part 2; the design:
docs/designs/2026-11-05_behaviour-and-speed.md.

```text
 DRIVE ─► APPROACH ─► STOPPED ─► WAIT ─► GO ─► DRIVE
  (no cap)  (cap falls)  (lock on,  (lock on)  (cap rises)
                         still)        │
                                       └─► ASK (a person is told)
```

The cap slows the car. **It can't stop it**: a lock or Nav2's goal check makes the final
stop. So a cap is 0.0 (Nav2: no limit) or at least the floor, never in between
(stop_law.nav2_cap), and this module never clears a lock, a latch or the e-stop.

Each stop is a `Stop`: a reason, the metres `left` before its final stop, and the lock
that makes that stop (none for the goal: Nav2's goal check does). The cap for a stop is
the stopping law's v_cap(left) (aav_description stop_law.py). The lowest one binds.

| Reason | `left` is | The final stop |
|---|---|---|
| `sign` | camera range to the sign − the watcher's `stop_distance_m` | the `stop_sign` lock |
| `obstacle` | to a thing in the lane (moving, with things), less bumper and gap | `stop_zone` |
| `fence` | along the plan to where the geofence would close NEAR | the `geofence` lock |
| `goal` | along the plan to its end | Nav2's goal check |
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import Dict, List, Optional, Sequence, Tuple

from aav_description.stop_law import FLOOR, nav2_cap, StopCurve

DRIVE, APPROACH, STOPPED, WAIT, GO, ASK = 'DRIVE', 'APPROACH', 'STOPPED', 'WAIT', 'GO', 'ASK'
STATES = (DRIVE, APPROACH, STOPPED, WAIT, GO, ASK)

# The lock that makes each reason's final stop (twist_mux's names, aav_command_mux).
LOCK_OF = {'sign': 'stop_sign', 'obstacle': 'stop_zone', 'fence': 'geofence', 'goal': None}
LOCKS = ('estop', 'stop_sign', 'stop_zone', 'geofence', 'sensors')


@dataclass(frozen=True)
class Settings:
    """The node's parameters. m/s, m/s², s."""

    base: float = 1.0          # m/s: Nav2's own top speed (the cap is clamped to it)
    floor: float = FLOOR       # m/s: the lowest cap sent (0.0 would mean no limit)
    accel: float = 0.5         # m/s²: how fast the cap rises in GO
    fall: float = 2.0          # m/s²: how fast the cap may fall (the car can't slow faster)
    still: float = 0.05        # m/s: slower counts as standing still
    ask_after: float = 30.0    # s a lock other than the sign's may hold before ASK
    odom_stale: float = 0.5    # s without odometry: the cap goes to the floor
    sign_lost: float = 5.0     # s without a sign point in APPROACH for the sign: DRIVE
    passed: float = 1.0        # m past a sign's stop with no lock: DRIVE, and a warning
    # s an obstacle must stay gone before APPROACH lets go of it (TODO 189): a thing at the
    # lane's edge drops in and out of it tick by tick, and the car sped up and slowed again
    # four times for one person crossing. Slowing stays at once; only the release waits.
    release_hold: float = 1.0


@dataclass(frozen=True)
class Stop:
    """One reason to stop: why, and the metres left before its final stop."""

    reason: str
    left: float


@dataclass
class World:
    """What the node saw this tick."""

    now: float
    speed: Optional[float] = None          # m/s; None: no odometry yet
    odom_age: float = math.inf             # s since the last odometry
    locks: Dict[str, bool] = field(default_factory=dict)
    near: bool = False                     # the geofence says NEAR
    stops: Sequence[Stop] = ()             # every stop in view this tick
    sign_age: float = math.inf             # s since the last sign point
    goal_active: bool = False
    goal_aborted: bool = False             # this tick's news: Nav2 aborted the goal


@dataclass(frozen=True)
class Decision:
    """What the node publishes this tick."""

    state: str
    reason: str
    cap: float                 # m/s for Nav2: 0.0 = no limit
    left: float                # m to the binding stop (inf: none)
    warnings: Tuple[str, ...] = ()


class Behaviour:
    """The state machine. `step(world)` once a tick; it returns the Decision."""

    def __init__(self, curve: StopCurve, settings: Settings = Settings()):
        if not (0.0 < settings.floor <= settings.base):
            raise ValueError('the floor must be above 0 and at most the base speed')
        self.curve = curve
        self.s = settings
        self.state = DRIVE          # a start or a restart: DRIVE, sending 0.0
        self.reason = ''
        self.since = -math.inf      # when the current state began
        self.cap = 0.0              # the last cap sent (0.0: no limit)
        self.last = None            # the last tick's time
        self.sign_lock = False      # the stop_sign lock, last tick
        self.served = -math.inf     # when the stop_sign lock last went off
        self.gone_since = None      # APPROACH(obstacle): when the obstacle stopped binding

    # --- the pieces ------------------------------------------------------------------

    def cap_for(self, left: float) -> float:
        """Return Nav2's cap for a stop `left` m ahead: 0.0, or floor..base."""
        return nav2_cap(left, self.curve, self.s.base, self.s.floor)

    def binding(self, stops: Sequence[Stop]) -> Optional[Stop]:
        """Return the stop whose cap binds (the lowest), or None if none does."""
        best, best_cap = None, math.inf
        for stop in stops:
            cap = self.cap_for(stop.left)
            if cap > 0.0 and cap < best_cap:
                best, best_cap = stop, cap
        return best

    def _go(self, state: str, reason: str, now: float) -> None:
        if state != self.state or reason != self.reason:
            self.since = now
        self.state, self.reason = state, reason

    def _limit(self, want: float, dt: float) -> float:
        """Return the cap to send: `want`, but falling no faster than `fall` allows."""
        if want == 0.0:
            return 0.0
        if self.cap == 0.0:
            # From no limit: start at the base speed and fall from there.
            previous = self.s.base
        else:
            previous = self.cap
        return max(want, previous - self.s.fall * dt, self.s.floor) if want < previous else want

    # --- one tick ------------------------------------------------------------------

    def step(self, w: World) -> Decision:
        s = self.s
        dt = 0.0 if self.last is None else max(0.0, w.now - self.last)
        self.last = w.now
        warnings: List[str] = []
        locks = {name: bool(on) for name, on in w.locks.items() if on}
        w = self._unserved(w, 'stop_sign' in locks)
        still = w.speed is not None and abs(w.speed) < s.still
        stop = self.binding(w.stops)
        left = stop.left if stop is not None else math.inf

        # Any state: the e-stop and an aborted goal.
        if locks.get('estop'):
            self._go(WAIT, 'e-stop', w.now)
        elif w.goal_aborted:
            self._go(ASK, 'the goal was aborted', w.now)
        elif self.state == DRIVE:
            if locks and still:
                self._go(WAIT, ', '.join(sorted(locks)), w.now)
            elif stop is not None:
                self._go(APPROACH, stop.reason, w.now)
        elif self.state == APPROACH:
            self._approach(w, locks, still, stop, warnings)
        elif self.state == STOPPED:
            self._go(WAIT, self.reason, w.now)
        elif self.state == WAIT:
            if not locks:
                self._go(GO, self.reason, w.now)
            elif w.near and still:
                self._go(ASK, 'the geofence is NEAR: only a person can clear it', w.now)
            elif (set(locks) != {'stop_sign'} and w.now - self.since > s.ask_after):
                self._go(ASK, f'held over {s.ask_after:g} s: {", ".join(sorted(locks))}',
                         w.now)
        elif self.state == GO:
            if locks:
                self._go(WAIT, ', '.join(sorted(locks)), w.now)
            elif stop is not None and self.cap_for(stop.left) <= max(self.cap, s.floor):
                self._go(APPROACH, stop.reason, w.now)
        elif self.state == ASK:
            if not locks:
                self._go(GO, self.reason, w.now)

        # The cap for the state.
        if self.state == DRIVE:
            want = self.cap_for(left) if stop is not None else 0.0
        elif self.state == APPROACH and self.gone_since is not None:
            # Holding the release (TODO 189): keep the cap it had; a new stop still binds.
            want = max(self.cap, s.floor)
            if stop is not None:
                want = min(want, max(self.cap_for(left), s.floor))
        elif self.state == APPROACH:
            mine = [x for x in w.stops if x.reason == self.reason]
            ahead = min((x.left for x in mine), default=left)
            want = max(self.cap_for(min(ahead, left)), s.floor)
            # The sign's cap never rises in APPROACH: a new picture can't speed the car up.
            if self.reason == 'sign' and self.cap > 0.0:
                want = min(want, self.cap)
            left = min(ahead, left)
        elif self.state in (STOPPED, WAIT, ASK):
            want = s.floor
        else:   # GO: rise at accel from the floor, under any new stop's cap
            # From where the cap was (the floor after a wait), rising from the next tick.
            rising = max(self.cap, s.floor) + (0.0 if self.since == w.now else s.accel * dt)
            bound = self.cap_for(left) if stop is not None else 0.0
            want = min(rising, bound) if bound > 0.0 else rising
            if want >= s.base:
                self._go(DRIVE, '', w.now)
                want = 0.0
        # Held by a lock, the car stands: the floor at once. Moving, the cap falls gently.
        cap = want if self.state in (STOPPED, WAIT, ASK) else self._limit(want, dt)

        # Any state: stale odometry sends the floor, and says so.
        if w.speed is None or w.odom_age > s.odom_stale:
            warnings.append(f'odometry stale ({w.odom_age:.1f} s): the floor')
            cap = s.floor
        self.cap = cap
        return Decision(self.state, self.reason, cap, left, tuple(warnings))

    def _unserved(self, w: World, sign_lock: bool) -> World:
        """
        Return the world without a sign already stopped for.

        Once the sign's lock goes off, its last point (from before the stop) is behind the
        car's stop: it would bring APPROACH(sign) back at the floor until it went stale. Only
        a point newer than the lock's end is a new sign (the watcher sends none while it
        ignores signs, after a stop).
        """
        if self.sign_lock and not sign_lock:
            self.served = w.now
        self.sign_lock = sign_lock
        if math.isfinite(self.served) and w.now - w.sign_age <= self.served:
            return replace(w, stops=[x for x in w.stops if x.reason != 'sign'])
        return w

    def _approach(self, w: World, locks: Dict[str, bool], still: bool, stop: Optional[Stop],
                  warnings: List[str]) -> None:
        s = self.s
        mine = [x for x in w.stops if x.reason == self.reason]
        if self.reason in ('obstacle', 'fence', 'goal') and self.binding(mine) is None:
            # No longer binds: a new goal's long path, or the obstacle moved further off.
            mine = []
        if mine or self.reason != 'obstacle':
            self.gone_since = None
        lock = LOCK_OF.get(self.reason)
        if lock is not None and locks.get(lock) and still:
            self._go(STOPPED, self.reason, w.now)
        elif locks and still:
            self._go(WAIT, ', '.join(sorted(locks)), w.now)
        elif self.reason == 'sign' and mine and min(x.left for x in mine) < -s.passed:
            warnings.append('sign passed, no lock')
            self._go(DRIVE, '', w.now)
        elif self.reason == 'sign' and not mine and w.sign_age > s.sign_lost:
            warnings.append('sign lost')
            self._go(DRIVE, '', w.now)
        elif self.reason == 'goal' and not w.goal_active:
            self._go(DRIVE, '', w.now)
        elif self.reason in ('obstacle', 'fence', 'goal') and not mine:
            # What we slowed for has gone (the obstacle moved, a new plan): speed up again;
            # an obstacle only once it has stayed gone release_hold s (TODO 189).
            if self.reason == 'obstacle' and stop is None:
                if self.gone_since is None:
                    self.gone_since = w.now
                if w.now - self.gone_since < s.release_hold:
                    return
            self.gone_since = None
            self._go(GO, self.reason, w.now) if stop is None else self._go(
                APPROACH, stop.reason, w.now)


def active_plan(point: Sequence[Tuple[float, float]], nav: Sequence[Tuple[float, float]],
                nav_fresh: bool) -> Tuple[Sequence[Tuple[float, float]], str]:
    """
    Return (the plan the car is driving, whose: 'point', 'nav' or '').

    A drive to a point outranks Nav2 in twist_mux (25 against 20), so its line A -> B wins
    while it runs (a non-empty `point`); else Nav2's /plan if fresh; else nothing.
    """
    if len(point) >= 2:
        return point, 'point'
    if nav and nav_fresh:
        return nav, 'nav'
    return [], ''


def path_left(points: Sequence[Tuple[float, float]], x: float, y: float) -> Tuple[float, float]:
    """
    Return (metres along the path from the car's foot to its end, the car's distance off it).

    points: the path's (x, y), in order; (x, y): the car, in the same frame.
    """
    if not points:
        return math.inf, math.inf
    if len(points) == 1:
        d = math.hypot(points[0][0] - x, points[0][1] - y)
        return d, d
    lengths = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])]
    total = sum(lengths)
    best, best_s, done = math.inf, 0.0, 0.0
    for (a, b), length in zip(zip(points, points[1:]), lengths):
        if length > 0.0:
            t = ((x - a[0]) * (b[0] - a[0]) + (y - a[1]) * (b[1] - a[1])) / (length * length)
            t = min(1.0, max(0.0, t))
        else:
            t = 0.0
        px, py = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        d = math.hypot(px - x, py - y)
        if d < best:
            best, best_s = d, done + t * length
        done += length
    return total - best_s, best


def along_until(points: Sequence[Tuple[float, float]], x: float, y: float, bad) -> float:
    """
    Return the metres along the path from the car's foot to the first point where `bad(px, py)`.

    inf if no point on the rest of the path is bad. Points every few cm are enough: Nav2's
    plans are 5 cm apart.
    """
    if not points:
        return math.inf
    left, _ = path_left(points, x, y)
    total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
    start = total - left
    done = 0.0
    for i, p in enumerate(points):
        if i:
            done += math.hypot(p[0] - points[i - 1][0], p[1] - points[i - 1][1])
        if done >= start and bad(p[0], p[1]):
            return done - start
    return math.inf


def lane_ahead(ranges: Sequence[float], angle_min: float, angle_increment: float,
               front: float, half_width: float, reach: float = 30.0) -> float:
    """
    Return the nearest x (m, base_link) of a scan point in the lane straight ahead, or inf.

    ranges: a LaserScan in base_link (the lidar_scan node's); the lane is |y| <= half_width,
    from the front bumper (`front` m ahead of base_link) out to `reach`.
    """
    nearest = math.inf
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        a = angle_min + i * angle_increment
        px, py = r * math.cos(a), r * math.sin(a)
        if front < px <= reach and abs(py) <= half_width:
            nearest = min(nearest, px)
    return nearest


@dataclass(frozen=True)
class Seen:
    """One thing from /world/things, in odom: centre, speed over the ground, size (m, m/s)."""

    x: float
    y: float
    vx: float
    vy: float
    length: float
    width: float


MOVING = 0.3   # m/s over the ground: slower is parked (the planner goes round it)


def moving_ahead(things: Sequence[Seen], car: Tuple[float, float, float], front: float,
                 half_width: float, reach: float = 30.0, moving: float = MOVING) -> float:
    """
    Return the nearest x (m, base_link) of a moving thing in the lane straight ahead, or inf.

    car: (x, y, yaw) in odom, as the things are. The lane is |y| <= half_width, widened by
    each thing's half width, from the front bumper out to `reach`. A thing slower than
    `moving` is parked: it is left to the planner, which plans round it (the whole car,
    "the things"; TODO 162). What moves (a person, a car) is waited for. The stop zone's
    lock still stops the car for anything, parked or not.
    """
    cx, cy, yaw = car
    c, s = math.cos(yaw), math.sin(yaw)
    nearest = math.inf
    for t in things:
        if math.hypot(t.vx, t.vy) < moving:
            continue
        dx, dy = t.x - cx, t.y - cy
        px, py = c * dx + s * dy, -s * dx + c * dy
        near = px - t.length / 2.0
        if front < px + t.length / 2.0 and near <= reach and abs(py) <= half_width + t.width / 2.0:
            nearest = min(nearest, max(near, front))
    return nearest


def diagnostic(decision: Decision) -> Dict[str, str]:
    """Return the state topic's key-value pairs (all strings)."""
    return {'state': decision.state, 'reason': decision.reason or '-',
            'cap_mps': f'{decision.cap:.3f}',
            'left_m': 'inf' if not math.isfinite(decision.left) else f'{decision.left:.2f}',
            'warnings': '; '.join(decision.warnings) or '-'}
