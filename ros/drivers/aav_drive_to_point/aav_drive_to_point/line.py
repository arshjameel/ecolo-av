"""
Where the car is against the line to B, and how to steer onto it (pure pursuit).

No ROS here. Map frame (x east, y north), metres and radians; the pose is base_link's,
on the ground under the rear axle, which is where pure pursuit's maths puts the car.

```text
            turn: pure pursuit to B            line: pure pursuit on A -> B
   car >---.                             A ------------------x-------------> B
            `-.  (the tightest circle            carrot: lookahead m ahead on the line
               `.  ego allows)                   cross track: the car's distance from it
                 `---> lined up: A = here
```

1. **Turn**: while B is more than `align` off the car's nose, aim straight at B, at most
   at full lock. A car can't turn on the spot, so B must lie outside both of its tightest
   circles (`reachable`), or it would circle B for ever.
2. **Line**: once lined up, A is where the car was, and it follows the line A -> B. The
   carrot runs on past B, so the car keeps its heading while it stops.

The **fence** (`Fence`) is a limit of its own, independent of Nav2 and of the steering: a
car that leaves it stops, whatever the reason (a wrong steering sign, a bad position).

```text
        start                                       max_off_line
          o . . . . . . . . . . . . . . . . . . . . . . . . . .
            A ===================================== B ===|  max_overrun past B
          o . . . . . . . . . . . . . . . . . . . . . . . . . .
   and never further from the start than max(|AB|, 2 R) + max_overrun
```
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float   # rad, anticlockwise from east


def wrap(angle: float) -> float:
    """Return the angle in [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def to_car(pose: Pose, point: Point) -> Point:
    """Return a map point in the car's frame: x ahead, y to the left."""
    dx, dy = point[0] - pose.x, point[1] - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return c * dx + s * dy, -s * dx + c * dy


def bearing_error(pose: Pose, target: Point) -> float:
    """Return how far B is off the car's nose (rad, + to the left)."""
    ahead, left = to_car(pose, target)
    return math.atan2(left, ahead)


def min_radius(wheelbase: float, max_steering: float) -> float:
    """Return the tightest turning radius (m) of base_link, at full lock."""
    if wheelbase <= 0 or not 0 < max_steering < math.pi / 2:
        raise ValueError('wheelbase must be above 0 and max_steering in (0, pi/2)')
    return wheelbase / math.tan(max_steering)


def reachable(pose: Pose, target: Point, radius: float, margin: float = 0.5) -> bool:
    """
    Return True if a forward-only car can drive to `target` without reversing.

    It can't reach a point inside either of its tightest circles (left and right, through
    the car): it would circle it. `margin` m more, as pure pursuit isn't perfect.
    """
    for side in (1.0, -1.0):
        cx = pose.x - side * radius * math.sin(pose.yaw)
        cy = pose.y + side * radius * math.cos(pose.yaw)
        if math.hypot(target[0] - cx, target[1] - cy) < radius + margin:
            return False
    return True


@dataclass(frozen=True)
class Line:
    """The straight line from A to B."""

    a: Point
    b: Point

    @property
    def length(self) -> float:
        return math.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1])

    def unit(self) -> Point:
        n = self.length
        if n <= 0:
            raise ValueError('A and B are the same point')
        return (self.b[0] - self.a[0]) / n, (self.b[1] - self.a[1]) / n

    def progress(self, p: Point) -> Tuple[float, float]:
        """Return (metres left to B along the line, metres left (+) of the line)."""
        ux, uy = self.unit()
        dx, dy = p[0] - self.b[0], p[1] - self.b[1]
        return -(dx * ux + dy * uy), -dx * uy + dy * ux

    def carrot(self, p: Point, lookahead: float) -> Point:
        """
        Return the point on the line `lookahead` m from p, ahead of it (past B is fine).

        Off the line by more than the lookahead, the nearest point on the line instead.
        """
        ux, uy = self.unit()
        left, cross = self.progress(p)
        along = self.length - left              # metres from A to p's foot on the line
        if abs(cross) < lookahead:
            along += math.sqrt(lookahead * lookahead - cross * cross)
        return self.a[0] + along * ux, self.a[1] + along * uy


@dataclass(frozen=True)
class Fence:
    """
    Where a drive to B may go. Distances in m.

    start: where the drive started. turn_radius: the car's tightest turn (min_radius).
    While it turns towards B there is no line yet: only the circle round the start, which
    a turn round (up to 2 R from the start) stays inside. Once lined up, also a corridor
    of max_off_line either side of the line it follows, ending max_overrun past B.
    """

    start: Point
    target: Point
    turn_radius: float
    max_off_line: float = 3.0
    max_overrun: float = 2.0

    def __post_init__(self):
        for name in ('turn_radius', 'max_off_line', 'max_overrun'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0):
                raise ValueError(f'{name} must be above 0 (got {value})')

    @property
    def reach(self) -> float:
        """Return how far from the start the car may ever be."""
        ab = math.hypot(self.target[0] - self.start[0], self.target[1] - self.start[1])
        return max(ab, 2.0 * self.turn_radius) + self.max_overrun

    def breach(self, p: Point, line: Optional[Line] = None) -> Optional[str]:
        """Return why the car at p is outside the fence, or None. line: the one it follows."""
        if line is not None:
            left, cross = line.progress(p)
            if abs(cross) > self.max_off_line:
                side = 'left' if cross > 0 else 'right'
                return (f'outside the fence: {abs(cross):.1f} m {side} of the line to B, '
                        f'over max_off_line {self.max_off_line:g} m')
            if -left > self.max_overrun:
                return (f'outside the fence: {-left:.1f} m past B, over max_overrun '
                        f'{self.max_overrun:g} m')
        far = math.hypot(p[0] - self.start[0], p[1] - self.start[1])
        if far > self.reach:
            return (f'outside the fence: {far:.1f} m from the start, over {self.reach:.1f} m '
                    '(the distance to B, or a turn round, plus max_overrun)')
        return None


def pursuit_curvature(pose: Pose, point: Point, max_curvature: float) -> float:
    """
    Return the curvature (1/m, + left) of the arc from the car through `point`.

    Pure pursuit: k = 2 y / (x^2 + y^2) in the car's frame, clamped to full lock. A point
    behind the car gets full lock towards its side (left if dead behind).
    """
    ahead, left = to_car(pose, point)
    if ahead <= 0:
        return max_curvature if left >= 0 else -max_curvature
    d2 = ahead * ahead + left * left
    k = 2.0 * left / d2 if d2 > 0 else 0.0
    return max(-max_curvature, min(max_curvature, k))


def turn_speed_limit(curvature: float, lateral_accel: float) -> float:
    """Return the fastest speed for this curvature with at most `lateral_accel` sideways."""
    k = abs(curvature)
    return math.sqrt(lateral_accel / k) if k > 1e-6 else math.inf
