"""
The lane's edges for the trackers (TODO 169): each path point kept inside the lane.

No ROS here. The behaviour planner's road ahead (/world/road_ahead) carries the lane's
line and, with a lane map, its soft edges: how far left and right of the line the car
may be. The trackers follow Nav2's path; `keep_in_lane` moves each of its points
sideways, only as far as it must, so the car's whole width stays between the edges,
less a margin. Where the edges aren't known (NaN, no lane map), or a point is far from
the line (a turn off it), the point stays as planned. `aav_mppi_lag`'s LaneCritic
does the same job inside MPPI.

```text
   left edge  ─────────────────────────────
                 ● path point, moved in  ↓
   line       ── ── ── ── ── ── ── ── ──
   right edge ─────────────────────────────
```
"""

from __future__ import annotations

import math
from typing import List, NamedTuple, Sequence, Tuple


class LinePoint(NamedTuple):
    """One road-ahead point: where, the line's heading, and its soft edges (m, + = room)."""

    x: float
    y: float
    heading: float
    left: float
    right: float


def nearest(line: Sequence[LinePoint], x: float, y: float) -> Tuple[int, float]:
    """Find the line point closest to x, y: return its index and distance (m)."""
    best, far = 0, math.inf
    for i, p in enumerate(line):
        d = math.hypot(x - p.x, y - p.y)
        if d < far:
            best, far = i, d
    return best, far


def keep_in_lane(path: Sequence[Tuple[float, float, float]], line: Sequence[LinePoint],
                 half_width: float, margin: float = 0.1,
                 reach: float = 5.0) -> List[Tuple[float, float, float]]:
    """
    Return the (x, y, yaw) path with each point moved sideways into the lane.

    half_width: half the car's width (m); margin: room kept inside each edge (m); reach: a
    point further than this from every line point is off the road ahead and stays put.
    A lane narrower than the car puts the point on the line itself.
    """
    if not line:
        return list(path)
    out = []
    for x, y, yaw in path:
        i, far = nearest(line, x, y)
        p = line[i]
        if far > reach or not (math.isfinite(p.left) and math.isfinite(p.right)):
            out.append((x, y, yaw))
            continue
        nx, ny = -math.sin(p.heading), math.cos(p.heading)      # to the line's left
        side = (x - p.x) * nx + (y - p.y) * ny
        low = -(p.right - half_width - margin)
        high = p.left - half_width - margin
        inside = 0.0 if low > high else min(max(side, low), high)
        shift = inside - side
        out.append((x + shift * nx, y + shift * ny, yaw))
    return out
