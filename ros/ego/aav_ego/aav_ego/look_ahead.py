"""
The squiggly line's maths: the plan beside the trace, and the room left at the stop point.

No ROS imports, so the tests run anywhere. The node look_ahead (look_ahead_node.py) turns
these into /ego/plan_ahead and /ego/ahead for every screen (docs/hud.md, "The squiggly
line"). Every point here is (x, y) in base_link: the car at (0, 0), facing +x.

    the plan (in base_link) ──▶ crop_ahead ──▶ the faint line: what the planner WANTS
    the trace + the plan    ──▶ gap        ──▶ how far the bold line (what the car WILL do)
                                              strays from it: green, amber, red
    the stop point + plan   ──▶ to_goal    ──▶ m from the stop point to the plan's end
    the stop point + grid   ──▶ first_hit  ──▶ m from the stop point to the first obstacle
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

from aav_description.road_ahead import along_of, limit_at

Point = Tuple[float, float]

OK, WARN, BAD = 0, 1, 2   # diagnostic_msgs/DiagnosticStatus levels: green, amber, red


def length(points: Sequence[Point]) -> float:
    """Return how long the polyline is (m)."""
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def project(points: Sequence[Point], p: Point) -> Tuple[float, float]:
    """
    Return how far along the polyline p's nearest point is, and how far p is from it.

    Both in metres, as (along, off). A single point: (0, its distance).
    """
    if not points:
        raise ValueError('an empty polyline')
    if len(points) == 1:
        return 0.0, math.dist(points[0], p)
    best = (math.inf, 0.0)   # (off, along)
    walked = 0.0
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg = math.hypot(dx, dy)
        u = 0.0 if seg == 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy)
                                                / (seg * seg)))
        off = math.dist((a[0] + u * dx, a[1] + u * dy), p)
        if off < best[0]:
            best = (off, walked + u * seg)
        walked += seg
    return best[1], best[0]


def crop_ahead(points: Sequence[Point], ahead: float) -> List[Point]:
    """
    Return the plan from the point nearest the car (the origin) to `ahead` m further on.

    The cut ends are on the polyline itself, so the line starts beside the car.
    """
    if len(points) < 2:
        return list(points)
    start, _ = project(points, (0.0, 0.0))
    return cut(points, start, start + ahead)


def cut(points: Sequence[Point], start: float, end: float) -> List[Point]:
    """Return the part of a polyline between `start` and `end` m along it."""
    out: List[Point] = []
    walked = 0.0
    for a, b in zip(points, points[1:]):
        seg = math.dist(a, b)
        lo, hi = walked, walked + seg
        if hi >= start and lo <= end and seg > 0.0:
            if not out:
                out.append(along_segment(a, b, (max(start, lo) - lo) / seg))
            out.append(along_segment(a, b, (min(end, hi) - lo) / seg))
        walked = hi
        if walked > end:
            break
    return out


def along_segment(a: Point, b: Point, u: float) -> Point:
    return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]))


def gap(trace: Sequence[Point], plan: Sequence[Point]) -> Optional[float]:
    """
    How far the trace strays from the plan: the largest distance of a trace point from it.

    Only trace points beside the plan count: one past the plan's end (the plan ends at the
    goal, the trace stops there or later) is measured to the end. None without both lines.
    """
    if not trace or len(plan) < 2:
        return None
    return max(project(plan, p)[1] for p in trace)


@dataclass(frozen=True)
class Limits:
    warn: float = 0.3   # m: amber past this (a third of the car's width)
    bad: float = 0.6    # m: red past this


def grade(value: Optional[float], limits: Limits = Limits()) -> int:
    """OK, WARN or BAD for a gap; OK with no plan (a person drives: nothing to stray from)."""
    if value is None:
        return OK
    if value > limits.bad:
        return BAD
    return WARN if value > limits.warn else OK


def to_goal(plan: Sequence[Point], stop: Point) -> Optional[float]:
    """
    Return the metres from the stop point to the plan's end (its goal), along the plan.

    + the car stops short of the goal, - it stops past it. None without a plan.
    """
    if len(plan) < 2:
        return None
    along, _ = project(plan, stop)
    total = length(plan)
    if along >= total - 1e-9:
        # Past the end: how far beyond it, along the last segment's direction.
        (ax, ay), (bx, by) = plan[-2], plan[-1]
        seg = math.hypot(bx - ax, by - ay) or 1.0
        return -max(0.0, ((stop[0] - bx) * (bx - ax) + (stop[1] - by) * (by - ay)) / seg)
    return total - along


@dataclass(frozen=True)
class Grid:
    """An occupancy grid (nav_msgs/OccupancyGrid's numbers), already in base_link."""

    width: int
    height: int
    resolution: float           # m per cell
    origin: Tuple[float, float, float]   # x, y, yaw of cell (0, 0)'s corner
    data: Sequence[int]         # row-major, -1 unknown, 0..100

    def cost(self, p: Point) -> int:
        """Return the cell's value under p, -1 outside the grid."""
        ox, oy, yaw = self.origin
        dx, dy = p[0] - ox, p[1] - oy
        c, s = math.cos(yaw), math.sin(yaw)
        gx, gy = (c * dx + s * dy) / self.resolution, (-s * dx + c * dy) / self.resolution
        i, j = math.floor(gx), math.floor(gy)
        if 0 <= i < self.width and 0 <= j < self.height:
            return self.data[j * self.width + i]
        return -1


def extend(path: Sequence[Tuple[float, float, float]], total: float
           ) -> List[Tuple[float, float, float]]:
    """Return a path of (x, y, yaw) poses, carried on straight to `total` m long."""
    out = list(path) or [(0.0, 0.0, 0.0)]
    have = length([(x, y) for x, y, _ in out])
    if have < total:
        x, y, yaw = out[-1]
        more = total - have
        out.append((x + more * math.cos(yaw), y + more * math.sin(yaw), yaw))
    return out


def first_hit(grid: Grid, path: Sequence[Tuple[float, float, float]], front: float,
              half_width: float, lethal: int = 100, step: float = 0.1) -> Optional[float]:
    """
    Return the metres base_link travels along `path` before the bumper meets a lethal cell.

    path is base_link's poses (x, y, yaw) ahead; front is the bumper's distance ahead of
    base_link, half_width half the body's. The bumper is checked across its width, corner
    to corner, at most `step` m apart, every `step` m along. None: nothing in the way.
    """
    across = max(1, math.ceil(2.0 * half_width / step))
    sides = [-half_width + 2.0 * half_width * k / across for k in range(across + 1)]
    walked = 0.0
    for (ax, ay, ayaw), (bx, by, byaw) in zip(path, path[1:]):
        seg = math.hypot(bx - ax, by - ay)
        n = max(1, math.ceil(seg / step))
        for k in range(n + 1):
            u = k / n
            x, y = ax + u * (bx - ax), ay + u * (by - ay)
            yaw = ayaw + u * math.remainder(byaw - ayaw, math.tau)
            c, s = math.cos(yaw), math.sin(yaw)
            fx, fy = x + front * c, y + front * s
            for side in sides:
                if grid.cost((fx - side * s, fy + side * c)) >= lethal:
                    return walked + u * seg
        walked += seg
    return None


def describe(gap_m: Optional[float], level: int, stop_m: Optional[float],
             goal_m: Optional[float], room_m: Optional[float]) -> str:
    """One line for people, such as 'ON PLAN 0.12 m - stops in 1.4 m, 0.8 m short of goal'."""
    head = {OK: 'ON PLAN', WARN: 'OFF PLAN', BAD: 'FAR OFF PLAN'}[level]
    head = 'NO PLAN' if gap_m is None else f'{head} {gap_m:.2f} m'
    parts = ['stop point unknown' if stop_m is None else f'stops in {stop_m:.1f} m']
    if goal_m is not None:
        parts.append(f'{abs(goal_m):.1f} m {"short of" if goal_m >= 0 else "past"} the goal')
    if room_m is not None:
        parts.append(f'{room_m:.1f} m before an obstacle' if room_m >= 0
                     else f'HITS an obstacle ({-room_m:.1f} m short)')
    return f'{head} - {", ".join(parts)}'


@dataclass(frozen=True)
class Road:
    """What the road ahead (/world/road_ahead) says where the car is."""

    limit: float            # m/s: the speed it allows here (0: none)
    room: float             # m left to its end (0 or less: there)
    why: str                # why it ends there: the nearest stop's reason ('' for none)


def road(strip, x: float, y: float) -> Optional[Road]:
    """
    Return the road ahead at the car (x, y, the strip's frame); None with no strip.

    The writer's own limit (no curve): the screens show what the road says, ego trims by it.
    """
    if strip is None or len(strip) == 0:
        return None
    s, _ = along_of(strip, x, y)
    ahead = [stop for stop in strip.stops if stop.along >= s - strip.spacing]
    why = min(ahead, key=lambda stop: stop.along).reason if ahead else ''
    return Road(limit_at(strip, x, y), strip.end - s, why)
