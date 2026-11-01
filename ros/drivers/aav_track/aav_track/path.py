"""
A planned path as the trackers see it: where the car is along it, and how it bends.

No ROS here. Odom frame (the path is moved there once, when it arrives: `moved`), metres
and radians. The pose is base_link's, on the ground under the rear axle.

```text
   path:  p0 ---- p1 ---- p2 ---- p3 ---- ... ---- end
                     ^ the car's foot on the path: s (m from p0), cross (+ = car left of it)
                     |
                    car
```

`project` searches forward from the last foot only (`window` m), as Regulated Pure
Pursuit's max_robot_pose_search_dist does, so a path that loops back past itself can't
make the car jump ahead. Past the end, the path carries on straight (`point_at`), so a
carrot or a front axle beyond it still has a line to follow.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


def wrap(angle: float) -> float:
    """Return the angle in [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def moved(points: Sequence[Tuple[float, float, float]],
          transform: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
    """Return (x, y, yaw) points moved by a 2D transform (tx, ty, tyaw): map into odom."""
    tx, ty, tyaw = transform
    c, s = math.cos(tyaw), math.sin(tyaw)
    return [(tx + c * x - s * y, ty + s * x + c * y, wrap(yaw + tyaw)) for x, y, yaw in points]


@dataclass(frozen=True)
class Foot:
    """Where a point is against the path."""

    s: float            # m along the path from its start, to the point's foot on it
    cross: float        # m, + = the point is left of the path
    heading: float      # rad, the path's direction at the foot
    curvature: float    # 1/m, + = bending left, smoothed over `Path.bend`
    index: int          # the segment the foot is on


class Path:
    """A polyline with its arc length; the last pose's yaw is the goal's heading."""

    def __init__(self, poses: Sequence[Tuple[float, float, float]], bend: float = 0.5):
        """
        Take the planner's (x, y, yaw) poses; repeated points are dropped.

        bend: m either side over which curvature is measured (Smac's points are 5-10 cm
        apart: a finite difference over one segment is noise).
        """
        if not poses:
            raise ValueError('an empty path')
        pts: List[Point] = []
        for x, y, _ in poses:
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError('a path point is not a number')
            if not pts or math.hypot(x - pts[-1][0], y - pts[-1][1]) > 1e-6:
                pts.append((x, y))
        self.goal_yaw = poses[-1][2]
        if len(pts) == 1:
            # One point: a short line along the goal's heading, so there is a direction.
            x, y = pts[0]
            pts.append((x + 0.01 * math.cos(self.goal_yaw), y + 0.01 * math.sin(self.goal_yaw)))
        self.points = pts
        self.bend = bend
        self.s = [0.0]
        self.headings = []
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            self.s.append(self.s[-1] + math.hypot(x1 - x0, y1 - y0))
            self.headings.append(math.atan2(y1 - y0, x1 - x0))

    @property
    def length(self) -> float:
        return self.s[-1]

    @property
    def end(self) -> Point:
        return self.points[-1]

    def segment(self, s: float) -> int:
        """Return the segment index holding arc length s (the first or last past the ends)."""
        return min(max(bisect_right(self.s, s) - 1, 0), len(self.headings) - 1)

    def point_at(self, s: float) -> Point:
        """Return the point s m along; before the start or past the end, straight on."""
        i = self.segment(s)
        x0, y0 = self.points[i]
        h = self.headings[i]
        d = s - self.s[i]
        return x0 + d * math.cos(h), y0 + d * math.sin(h)

    def heading_at(self, s: float) -> float:
        return self.headings[self.segment(s)]

    def curvature_at(self, s: float) -> float:
        """Return the path's curvature at s (1/m, + left): the turn over +-bend m."""
        a = max(0.0, s - self.bend)
        b = min(self.length, s + self.bend)
        if b - a < 1e-6:
            return 0.0
        return wrap(self.heading_at(b - 1e-9) - self.heading_at(a)) / (b - a)

    def max_curvature(self, s0: float, s1: float, step: float = 0.25) -> float:
        """Return the largest |curvature| between s0 and s1 (m along)."""
        s1 = min(s1, self.length)
        worst, s = 0.0, max(0.0, s0)
        while s <= s1:
            worst = max(worst, abs(self.curvature_at(s)))
            s += step
        return worst

    def project(self, p: Point, hint: int = 0, window: float = 10.0) -> Foot:
        """
        Return where p is against the path, searching segments from `hint` on, `window` m.

        The nearest point on those segments; past the last one, the line straight on
        (so `s` may exceed `length`, and `left` go negative).
        """
        hint = min(max(hint, 0), len(self.headings) - 1)
        best: Optional[Tuple[float, int, float]] = None   # (distance^2, index, t m along)
        i = hint
        while i < len(self.headings) and self.s[i] - self.s[hint] <= window:
            x0, y0 = self.points[i]
            h = self.headings[i]
            seg = self.s[i + 1] - self.s[i]
            ux, uy = math.cos(h), math.sin(h)
            t = (p[0] - x0) * ux + (p[1] - y0) * uy
            # Before the first point and past the last, the line carries on.
            lo = -math.inf if i == 0 else 0.0
            hi = math.inf if i == len(self.headings) - 1 else seg
            t = min(max(lo, t), hi)
            d2 = (p[0] - x0 - t * ux) ** 2 + (p[1] - y0 - t * uy) ** 2
            if best is None or d2 < best[0] - 1e-12:
                best = (d2, i, t)
            i += 1
        _, i, t = best
        x0, y0 = self.points[i]
        h = self.headings[i]
        cross = -(p[0] - x0) * math.sin(h) + (p[1] - y0) * math.cos(h)
        s = self.s[i] + t
        return Foot(s=s, cross=cross, heading=h, curvature=self.curvature_at(s), index=i)

    def left(self, s: float) -> float:
        """Return the metres from s to the path's end (negative past it)."""
        return self.length - s
