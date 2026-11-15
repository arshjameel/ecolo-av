r"""
The lane map: which lane the car drives in, so the road ahead's line is the lane's centre.

No ROS here. The design: docs/designs/2026-11-14_the-whole-car.md, "The road ahead" (W3,
W5). The behaviour planner reads it with `lane_map:=`; without one it keeps /plan's line.

One road, seen from above, the car driving left to right along the route:

```text
   ══════════════════════════════  left kerb   ─┐
   ── ── ── ── ── ── ── ── ── ──   centre line  │ left_hard
                                                │
   ● ──────── the car's lane centre ───────►    ┼─ left_soft = right_soft = lane / 2
   ─────────────────────────────   edge line  ──┤ right_soft
   ══════════════════════════════  right kerb ──┘ right_hard (edge line + edge_inset)
```

| The road | Its lanes | The car's lane |
|---|---|---|
| one lane (`lanes: 1`, the road world) | one, both ways | the whole lane: the road's centre |
| two-way, `n` lanes | `n / 2` each way | the rightmost, as the car faces |
| one-way, `n` lanes | all `n` one way | the rightmost |

Which way the car faces comes from the route, not from the way the road was drawn: the
same road gives the other lane when the route runs the other way.

The file, `lanes.yaml` (the campus's, made by `pixi run campus`, is this format):

```yaml
roads:
- name: road
  width: 4.5        # m, kerb to kerb (the tarmac)
  edge_inset: 0.5   # m from the kerb to the edge line's centre (optional, 0.5)
  lanes: 1          # every lane, both ways (optional: width / 3.5, rounded)
  oneway: false
  points: [[-20.0, 0.0], [100.0, 0.0]]   # the centre line, map frame, m
```

Lanelet2 (W5) was tried first, 14 Nov: RoboStack's 1.2.3 installs, but on a Mac its Python
modules are `.dylib` files Python can't import, and loading them by hand hangs (they link a
second libpython). So this simple file, until it does.

Units: m, rad. The map frame, x east, y north.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Dict, List, Optional, Sequence, Tuple

from aav_description.road_ahead import resample, Strip

LANE_WIDTH = 3.5        # m: one lane, when the file doesn't say how many
EDGE_INSET = 0.5        # m: kerb to edge line (tools/campus_world.py's EDGE_INSET)
MAX_TURN = math.radians(45.0)   # the route and the road agree within this, or it's not that road
CELL = 20.0             # m: the index's squares
SMOOTH = 3.0            # m each side: the lane line eases from one road to the next over this
MARGIN = 0.5            # m beyond the kerb a route point may be and still be on the road


@dataclass(frozen=True)
class Road:
    """One road: its centre line (map frame, m), its width kerb to kerb, how its lanes run."""

    name: str
    points: Tuple[Tuple[float, float], ...]
    width: float
    lanes: int = 0              # every lane, both ways; 0: width / LANE_WIDTH, rounded
    oneway: bool = False
    edge_inset: float = EDGE_INSET

    def lane_count(self) -> int:
        return self.lanes if self.lanes > 0 else max(1, round(self.width / LANE_WIDTH))


@dataclass(frozen=True)
class Lane:
    """The car's lane at one place: where its centre is, and its edges (m, all positive)."""

    x: float
    y: float
    heading: float              # rad: the way the car drives it
    left_soft: float
    right_soft: float
    left_hard: float
    right_hard: float
    road: str


def lane_offset(road: Road) -> Tuple[float, float]:
    """
    Return (the car's lane centre, its width): metres right of the road's centre (+ = right).

    Seen the way the car drives. One lane: 0 and the whole painted width.
    """
    painted = road.width - 2.0 * road.edge_inset
    n = road.lane_count()
    if n <= 1:
        return 0.0, painted
    if road.oneway:
        width = painted / n
    else:
        width = (painted / 2.0) / max(1, n // 2)
    return painted / 2.0 - width / 2.0, width


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class LaneMap:
    """The roads, with a grid index so a lookup reads only the segments near a point."""

    def __init__(self, roads: Sequence[Road]):
        self.roads = [r for r in roads if len(r.points) >= 2 and r.width > 0.0]
        self._cells: Dict[Tuple[int, int], List[Tuple[int, int]]] = {}
        self._reach = max([r.width / 2.0 + MARGIN for r in self.roads] or [0.0])
        for ri, road in enumerate(self.roads):
            for si, (a, b) in enumerate(zip(road.points, road.points[1:])):
                for key in self._cells_near(a, b, road.width / 2.0 + MARGIN):
                    self._cells.setdefault(key, []).append((ri, si))

    @staticmethod
    def _cells_near(a, b, pad):
        x0, x1 = sorted((a[0], b[0]))
        y0, y1 = sorted((a[1], b[1]))
        for i in range(math.floor((x0 - pad) / CELL), math.floor((x1 + pad) / CELL) + 1):
            for j in range(math.floor((y0 - pad) / CELL), math.floor((y1 + pad) / CELL) + 1):
                yield i, j

    def lane_at(self, x: float, y: float, heading: float) -> Optional[Lane]:
        """
        Return the car's lane at (x, y), driving at `heading` (rad), or None off every road.

        The road is the nearest whose tarmac (plus MARGIN) holds the point and that runs
        within MAX_TURN of `heading`, either way along it.
        """
        best = None
        for ri, si in self._cells.get((math.floor(x / CELL), math.floor(y / CELL)), ()):
            road = self.roads[ri]
            a, b = road.points[si], road.points[si + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            n2 = dx * dx + dy * dy
            if n2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / n2))
            fx, fy = a[0] + t * dx, a[1] + t * dy
            d = math.hypot(x - fx, y - fy)
            if d > road.width / 2.0 + MARGIN:
                continue
            direction = math.atan2(dy, dx)
            turn = _wrap(heading - direction)
            if abs(turn) <= MAX_TURN:
                along = True
            elif abs(turn) >= math.pi - MAX_TURN:
                along = False
            else:
                continue
            if best is None or d < best[0]:
                best = (d, road, fx, fy, direction if along else _wrap(direction + math.pi))
        if best is None:
            return None
        _, road, fx, fy, way = best
        right, width = lane_offset(road)
        # Right of the way the car drives: (sin, -cos) of its heading.
        cx, cy = fx + right * math.sin(way), fy - right * math.cos(way)
        half = road.width / 2.0
        return Lane(x=cx, y=cy, heading=way, left_soft=width / 2.0, right_soft=width / 2.0,
                    left_hard=half + right, right_hard=half - right, road=road.name)


def load(path: str) -> LaneMap:
    """Return the lane map in a lanes.yaml (format above)."""
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return from_dict(data)


def from_dict(data: dict) -> LaneMap:
    """Return the lane map from lanes.yaml's contents."""
    return LaneMap([Road(name=str(r.get('name') or r.get('kind', '')),
                         points=tuple((float(p[0]), float(p[1])) for p in r['points']),
                         width=float(r['width']), lanes=int(r.get('lanes', 0)),
                         oneway=bool(r.get('oneway', False)),
                         edge_inset=float(r.get('edge_inset', EDGE_INSET)))
                    for r in data.get('roads', []) or []])


def lane_line(lanes: LaneMap, route: Sequence[Tuple[float, float]],
              car: Tuple[float, float], spacing: float = 0.5, length: float = 40.0,
              smooth: float = SMOOTH) -> List[Tuple[float, float]]:
    """
    Return the car's lane centre along the route ahead of the car: points, map frame.

    Each route point ahead (from the car's foot, every `spacing` m, `smooth` m past
    `length`) moves onto its lane's centre, however the route weaves inside the lane. Off
    every road (a lot, a gap at a junction) it stays where the route is. Where the road
    changes, the points are averaged over `smooth` m each side, so the line eases from one
    lane to the next instead of stepping.
    """
    ahead = resample(route, *car, spacing=spacing, length=length + smooth)
    if not ahead:
        return []
    # Each route point's place: its lane's centre (and that road), or itself off every road.
    targets = []
    for x, y, h in ahead:
        lane = lanes.lane_at(x, y, h)
        targets.append((x, y, None) if lane is None else (lane.x, lane.y, lane.road))
    k = max(0, int(round(smooth / spacing)))
    out = []
    for i, (x, y, road) in enumerate(targets):
        window = targets[max(0, i - k):i + k + 1]
        # On one road all along the window: the lane's centre itself, however the route
        # weaves inside the lane. Where roads change or one ends: the window's mean.
        if road is not None and all(w[2] == road for w in window):
            out.append((x, y))
        else:
            out.append((sum(w[0] for w in window) / len(window),
                        sum(w[1] for w in window) / len(window)))
    return out


def with_edges(strip: Strip, lanes: LaneMap) -> Strip:
    """Return the strip with its edges from the lane map: NaN where a point is on no road."""
    if len(strip) == 0:
        return strip
    edges = {'left_hard': [], 'right_hard': [], 'left_soft': [], 'right_soft': []}
    for x, y, h in zip(strip.x, strip.y, strip.heading):
        lane = lanes.lane_at(x, y, h)
        for name in edges:
            if lane is None:
                edges[name].append(math.nan)
                continue
            # The point may sit off the lane's centre (a smoothed bend, a junction):
            # its edges are measured from where it is.
            nx, ny = -math.sin(lane.heading), math.cos(lane.heading)
            left = (x - lane.x) * nx + (y - lane.y) * ny      # + = left of the centre
            value = getattr(lane, name) + (-left if name.startswith('left') else left)
            edges[name].append(max(0.0, value))
    return replace(strip, **{k: tuple(v) for k, v in edges.items()})
