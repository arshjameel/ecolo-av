"""
Forward and back: a path cut at its cusps, and a reversing stretch driven by a forward tracker.

No ROS here. TODO 174: every tracker reverses where the plan says.

The planner's poses face the way the car faces, so on a reversing stretch they point
back along the path: a segment against its pose's yaw is driven backwards.

```text
   forward  ─────────►  cusp  ◄───────── backwards  cusp  ─────────► forward
   stretch 0                  stretch 1                   stretch 2
```

A tracker only ever drives forwards. A backwards stretch is given to it **mirrored**: the
car turned round (yaw + pi), the poses turned round, its speed and steering negated.
Driving backwards at v with steering d, base_link (the rear axle) moves as a car facing
the other way moving forwards at |v| with steering -d: the same bicycle, so the answer is
the real car's with both signs turned back.

| Real car, backwards | The tracker sees |
|---|---|
| yaw | yaw + pi |
| v < 0 | -v > 0 |
| steering d | -d |
| poses (x, y, yaw) | (x, y, yaw + pi) |

`Gears` keeps which stretch the car is on. The next is taken at the cusp: within
`reach` m of the stretch's end and nearly still; or standing `wait` s within `near` m of
it (a lock held it short, as mppi_lag's `cusp_wait`).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

from aav_track.path import Path, wrap

Pose = Tuple[float, float, float]


def directions(poses: Sequence[Pose]) -> List[int]:
    """Return each segment's gear: +1 forward, -1 backwards (against its first pose's yaw)."""
    out = []
    for (x0, y0, yaw0), (x1, y1, _) in zip(poses, poses[1:]):
        dx, dy = x1 - x0, y1 - y0
        if math.hypot(dx, dy) < 1e-6:
            out.append(out[-1] if out else 1)
            continue
        out.append(1 if math.cos(math.atan2(dy, dx) - yaw0) >= 0.0 else -1)
    return out


@dataclass(frozen=True)
class Stretch:
    """One gear's run of poses (as planned), and its way."""

    poses: Tuple[Pose, ...]
    backwards: bool

    def path(self) -> Path:
        """Return the path the tracker follows: as planned, or mirrored if backwards."""
        if not self.backwards:
            return Path(self.poses)
        return Path([(x, y, wrap(yaw + math.pi)) for x, y, yaw in self.poses])


def stretches(poses: Sequence[Pose]) -> List[Stretch]:
    """Return the path cut at each change of gear; the cusp's pose ends one and starts the next."""
    poses = list(poses)
    if len(poses) < 2:
        return [Stretch(tuple(poses), False)]
    gears = directions(poses)
    out, start = [], 0
    for i in range(1, len(gears)):
        if gears[i] != gears[i - 1]:
            out.append(Stretch(tuple(poses[start:i + 1]), gears[i - 1] < 0))
            start = i
    out.append(Stretch(tuple(poses[start:]), gears[-1] < 0))
    return out


def _length(poses: Sequence[Pose]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:]))


def mirror_state(x: float, y: float, yaw: float, v: float, yaw_rate: float, steer: float,
                 backwards: bool) -> Tuple[float, float, float, float, float, float]:
    """Return the car as the tracker sees it: turned round if `backwards` (yaw_rate stays)."""
    if not backwards:
        return x, y, yaw, v, yaw_rate, steer
    return x, y, wrap(yaw + math.pi), -v, yaw_rate, -steer


@dataclass
class Gears:
    """Which stretch the car is on, and when to take the next."""

    parts: List[Stretch]
    reach: float = 0.3      # m from a stretch's end: the cusp is reached
    still: float = 0.15     # m/s: slow enough to change gear
    near: float = 1.5       # m: standing this close for `wait` s will do
    wait: float = 2.0       # s
    index: int = 0
    _standing: Optional[float] = None

    @classmethod
    def of(cls, poses: Sequence[Pose], **settings) -> 'Gears':
        return cls(stretches(poses), **settings)

    @property
    def current(self) -> Stretch:
        return self.parts[self.index]

    @property
    def last(self) -> bool:
        return self.index == len(self.parts) - 1

    @property
    def backwards(self) -> bool:
        return self.current.backwards

    @property
    def any_backwards(self) -> bool:
        return any(p.backwards for p in self.parts)

    def left_after(self) -> float:
        """Return the metres of the stretches after this one."""
        return sum(_length(p.poses) for p in self.parts[self.index + 1:])

    def update(self, left: float, v: float, now: float) -> bool:
        """
        Take the next stretch if this one is done: `left` m to its end, the car at v m/s.

        Return True when the stretch changed (the tracker needs the new path).
        """
        if self.last:
            return False
        if abs(v) <= 0.05 and left <= self.near:
            if self._standing is None:
                self._standing = now
        else:
            self._standing = None
        at_cusp = left <= self.reach and abs(v) <= self.still
        waited = self._standing is not None and now - self._standing >= self.wait
        if not (at_cusp or waited):
            return False
        self.index += 1
        self._standing = None
        return True
