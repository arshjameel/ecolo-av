r"""
The road ahead: one strip of road that says where the car may be, how fast, and where it stops.

No ROS here (road_ahead_ros.py turns it into the message). The design:
docs/designs/2026-11-14_the-whole-car.md, "The road ahead". The behaviour planner is the
only writer (`build`); every driver and screen reads it (`limit_at`, `room_at`).

```text
   car ●──────── the line: a point every `spacing` m ──────────► ║ END
   speed limit:  base … base … cap(end − s) … 0                  ║ (may not pass)
                 └──────── the stopping law, from the end ───────┘
```

| Part | From |
|---|---|
| The line | the plan, from the car's foot on it, for up to `length` m |
| Stop points | the behaviour planner's stops: `left` m from the car is `along` on the line |
| The end | the nearest stop, or the line's last point: the car may run no further |
| The speed limit | the stopping law's cap for the room left to the end, never above `base` |

**The end is the safety idea** (a railway's movement authority). The strip always ends in
a stop. If the writer dies, the car runs out of strip and stops by itself: no timeout to
tune, and silence never means "carry on". A reader that has no strip at all allows 0.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import List, Optional, Sequence, Tuple

from aav_description.stop_law import StopCurve

WAIT, FINAL = 0, 1      # as aav_interfaces/StopPoint
# Each reason's kind: stop then go on, or stop and stay.
KIND_OF = {'sign': WAIT, 'obstacle': WAIT, 'fence': FINAL, 'goal': FINAL, 'moving': WAIT,
           'stop_line': WAIT}

LENGTH = 40.0           # m: how much road the writer grants at a time
SPACING = 0.5           # m between the line's points


@dataclass(frozen=True)
class StopOnLine:
    """One stop: metres along the line, why, and WAIT or FINAL."""

    along: float
    reason: str
    kind: int


@dataclass(frozen=True)
class Strip:
    """The road ahead. Lists are per point of the line; metres, radians, m/s."""

    spacing: float
    x: Tuple[float, ...] = ()
    y: Tuple[float, ...] = ()
    heading: Tuple[float, ...] = ()
    speed_limit: Tuple[float, ...] = ()
    stops: Tuple[StopOnLine, ...] = ()
    end: float = 0.0
    state: str = ''
    reason: str = ''
    # NaN: not known yet (the lane map brings them, W5).
    left_hard: Tuple[float, ...] = field(default=())
    right_hard: Tuple[float, ...] = field(default=())
    left_soft: Tuple[float, ...] = field(default=())
    right_soft: Tuple[float, ...] = field(default=())

    def __len__(self) -> int:
        return len(self.x)


def _foot(points: Sequence[Tuple[float, float]], x: float, y: float) -> Tuple[int, float]:
    """Return (segment index, fraction along it) of the point on the path nearest (x, y)."""
    best, best_i, best_t = math.inf, 0, 0.0
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = dx * dx + dy * dy
        t = 0.0 if n == 0.0 else min(1.0, max(0.0, ((x - a[0]) * dx + (y - a[1]) * dy) / n))
        d = math.hypot(a[0] + t * dx - x, a[1] + t * dy - y)
        if d < best:
            best, best_i, best_t = d, i, t
    return best_i, best_t


def resample(points: Sequence[Tuple[float, float]], x: float, y: float,
             spacing: float = SPACING, length: float = LENGTH
             ) -> List[Tuple[float, float, float]]:
    """
    Return the path ahead of (x, y) as (x, y, heading), one point every `spacing` m.

    The first point is the car's foot on the path; the last is the path's end or `length`
    m along, whichever is nearer. Fewer than two path points: nothing (no line to follow).
    """
    if spacing <= 0.0:
        raise ValueError(f'spacing must be above 0 (got {spacing})')
    if len(points) < 2:
        return []
    i, t = _foot(points, x, y)
    a, b = points[i], points[i + 1]
    # The rest of the path, from the foot.
    rest = [(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))] + list(points[i + 1:])
    out: List[Tuple[float, float, float]] = []
    want = 0.0          # m along: the next point to place
    done = 0.0          # m along at the start of this segment
    heading = math.atan2(b[1] - a[1], b[0] - a[0])
    for p, q in zip(rest, rest[1:]):
        seg = math.hypot(q[0] - p[0], q[1] - p[1])
        if seg == 0.0:
            continue
        heading = math.atan2(q[1] - p[1], q[0] - p[0])
        while want <= done + seg + 1e-9 and want <= length + 1e-9:
            f = (want - done) / seg
            out.append((p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]), heading))
            want += spacing
        done += seg
        if want > length + 1e-9:
            break
    # Keep the path's real end when it falls between two points.
    total = min(done, length)
    if out and total - (len(out) - 1) * spacing > 1e-6 and done <= length + 1e-9:
        end = rest[-1]
        out.append((end[0], end[1], heading))
    return out


def _along(line: Sequence[Tuple[float, float, float]], spacing: float, i: int) -> float:
    """Return the metres along the line to point i (the last may be nearer than spacing)."""
    if i < len(line) - 1:
        return i * spacing
    if i == 0:
        return 0.0
    p, q = line[i - 1], line[i]
    return (i - 1) * spacing + math.hypot(q[0] - p[0], q[1] - p[1])


def build(plan: Sequence[Tuple[float, float]], car: Optional[Tuple[float, float]],
          stops: Sequence, curve: StopCurve, base: float, *, state: str = '',
          reason: str = '', spacing: float = SPACING, length: float = LENGTH) -> Strip:
    """
    Return the road ahead for the car at `car` on `plan`.

    stops: the behaviour planner's `Stop`s (a reason and `left`, metres from the car to its
    final stop); a stop behind the car (left < 0) sits at 0: stop now. No plan or no pose:
    an empty strip whose end is 0, so every reader allows 0.
    """
    if base <= 0.0:
        raise ValueError(f'base must be above 0 (got {base})')
    line = resample(plan, *car, spacing=spacing, length=length) if car is not None else []
    if not line:
        return Strip(spacing=spacing, state=state, reason=reason)
    last = _along(line, spacing, len(line) - 1)
    on_line = sorted((StopOnLine(max(0.0, s.left), s.reason, KIND_OF.get(s.reason, WAIT))
                      for s in stops if s.left <= last),
                     key=lambda s: s.along)
    end = min([last] + [s.along for s in on_line])
    limits = []
    for i in range(len(line)):
        room = end - _along(line, spacing, i)
        limits.append(0.0 if room <= 0.0 else min(base, curve.cap(room)))
    nan = (math.nan,) * len(line)
    return Strip(spacing=spacing, x=tuple(p[0] for p in line), y=tuple(p[1] for p in line),
                 heading=tuple(p[2] for p in line), speed_limit=tuple(limits),
                 stops=tuple(on_line), end=end, state=state, reason=reason,
                 left_hard=nan, right_hard=nan, left_soft=nan, right_soft=nan)


def capped(strip: Strip, cap: float) -> Strip:
    """
    Return the strip with no speed limit above `cap` m/s; 0.0 is no cap.

    The behaviour planner's cap (its states: the floor while held, the gentle rise after a
    wait, the sign's cap that never rises) goes into the road ahead this way: ego already
    caps by it, so the cap reaches the car whichever follower drives (TODO 233).
    """
    if cap <= 0.0:
        return strip
    return replace(strip, speed_limit=tuple(min(v, cap) for v in strip.speed_limit))


def along_of(strip: Strip, x: float, y: float) -> Tuple[float, float]:
    """Return (metres along the strip's line of the point nearest (x, y), distance off it)."""
    if len(strip) == 0:
        return 0.0, math.inf
    if len(strip) == 1:
        return 0.0, math.hypot(strip.x[0] - x, strip.y[0] - y)
    points = list(zip(strip.x, strip.y))
    i, t = _foot(points, x, y)
    a, b = points[i], points[i + 1]
    seg = math.hypot(b[0] - a[0], b[1] - a[1])
    fx, fy = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
    return i * strip.spacing + t * seg, math.hypot(fx - x, fy - y)


def room_at(strip: Strip, x: float, y: float) -> float:
    """Return the metres from the car at (x, y) to the strip's end: 0 or less, no further."""
    if len(strip) == 0:
        return 0.0
    s, _ = along_of(strip, x, y)
    return strip.end - s


def limit_at(strip: Optional[Strip], x: float, y: float, curve: Optional[StopCurve] = None,
             base: Optional[float] = None) -> float:
    """
    Return the speed the strip allows the car at (x, y): m/s, 0 with no strip or past its end.

    With a `curve` (the reader's own stopping law), the limit is worked out again from the
    room left, so it falls smoothly as the car closes on an old strip's end: a reader of a
    strip whose writer has died still stops in time. Without one, it is the writer's
    limit at the nearest point.
    """
    if strip is None or len(strip) == 0:
        return 0.0
    s, _ = along_of(strip, x, y)
    room = strip.end - s
    if room <= 0.0:
        return 0.0
    i = min(len(strip) - 1, int(s / strip.spacing))
    written = strip.speed_limit[i]
    if curve is None:
        return written
    mine = curve.cap(room)
    if base is not None:
        mine = min(mine, base)
    return min(written, mine) if written > 0.0 else mine
