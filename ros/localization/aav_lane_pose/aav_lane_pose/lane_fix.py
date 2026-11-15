r"""
The lane finder as a pose sensor: where the car is across its lane, as a map pose.

No ROS here (TODO 155). The node, `lane_pose.py`, feeds it.

The camera sees the painted lines; the lane map knows where they are. Together they say
where the car is **across** the road, to a few centimetres, but nothing about where it is
**along** it. So the pose this makes is sure across the lane and unsure along it:

```text
         along the lane: sigma_along (50 m: no information)
        <───────────────────────────────────────────────>
   ════════════════════════════════════════════════════════  edge line
                              ▲ d (+ = left of the centre)
   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─●─ the car ─ ─ ─ ─ ─ ─ ─ ─ ─   the lane's centre (map)
                              ▼ sigma_across (0.15 m, both lines)
   ════════════════════════════════════════════════════════  edge line
```

| Step | What |
|---|---|
| `seen_from` | the lines (base_link) → `d`, the car from the lane's centre; `psi`, its angle |
| `LanePose.update` | the gates below; then the map lane's centre moved `d` sideways |

Along the lane, the pose is where the filter already has the car.

The gates (each refused picture is counted by its reason):

| Reason | Refused when |
|---|---|
| `no line` | no line detected |
| `confidence` | the surest line under `min_confidence` |
| `turned` | the car more than `max_heading` from the lane (changing lanes, a junction) |
| `off road` | the filter's pose is on no road of the map |
| `junction` | another road, or the road's end, from `behind` m behind to `ahead` m ahead |
| `width` | both lines seen, their gap off the map lane's width by over `width_tolerance` |
| `jump` | `d` moved more than `max_jump` from the last one taken, under `jump_window` s ago |
| `innovation` | `d` over `max_innovation` from where the filter has the car (the other lane?) |

Signs as REP-103 and lane_lines.py: x forward, y left, `d` + when the car is left of the
centre, `psi` + when the car points left of the lane. Units: m, rad, s. The map frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Optional, Tuple

from aav_description.lane_map import Lane, LaneMap

LOOSE = 1.0e3           # m^2, rad^2: what the pose says nothing about (z, roll, pitch)


@dataclass(frozen=True)
class Settings:
    """The gates and the covariance. The node makes each one a parameter of the same name."""

    min_confidence: float = 0.5     # 0-1: the surest line must reach it
    max_heading: float = 0.3        # rad: the car against the lane
    width_tolerance: float = 0.6    # m: both lines' gap against the map lane's width
    max_jump: float = 0.4           # m: d against the last one taken ...
    jump_window: float = 1.0        # s: ... under this long ago
    max_innovation: float = 1.5     # m: d against where the filter puts the car
    behind: float = 4.0             # m: no junction or road end this far behind
    ahead: float = 12.0             # m: ... and this far ahead (the camera sees 4-16 m)
    step: float = 4.0               # m: the walk along the lane, for those checks
    sigma_both: float = 0.15        # m: across the lane, both lines seen
    sigma_one: float = 0.30         # m: across the lane, one line seen
    sigma_along: float = 50.0       # m: along the lane (no information)
    sigma_yaw: float = 0.05         # rad: the lane's heading less psi (both lines; x2 with one)


@dataclass(frozen=True)
class Line:
    """One painted line in base_link: y = a x^2 + b x + c (m)."""

    a: float
    b: float
    c: float


@dataclass(frozen=True)
class Seen:
    """What the camera says: the car against the lane's centre, at base_link."""

    offset: float       # m, d: + = the car is left of the lane's centre
    heading: float      # rad, psi: + = the car points left of the lane
    width: float        # m: between the lines (the finder's guess with one line)
    lines: int          # 1 or 2
    confidence: float   # 0-1: the surest line's


@dataclass(frozen=True)
class Fix:
    """A pose for the EKF: map frame, sure across the lane only."""

    x: float
    y: float
    yaw: float
    covariance: Tuple[float, ...]   # 36 numbers, row-major: x, y, z, roll, pitch, yaw
    offset: float                   # m: d, as measured
    road: str


def seen_from(left: Optional[Line], right: Optional[Line], width: float,
              left_confidence: float, right_confidence: float) -> Optional[Seen]:
    """
    Return the car against the lane's centre from the lines detected (None: no line).

    The centre line at x = 0 (base_link, under the rear axle), as lane_lines.centre_y does:
    the middle of both lines, or half the width beside one. A straight centre line
    y = c + b x seen from a car `d` left of it and turned `psi` has b = -tan(psi) and
    c = -d / cos(psi).
    """
    if left and right:
        c, b = (left.c + right.c) / 2.0, (left.b + right.b) / 2.0
        psi = -math.atan(b)
        return Seen(-c * math.cos(psi), psi, (left.c - right.c) * math.cos(psi), 2,
                    max(left_confidence, right_confidence))
    line, side, confidence = ((left, -1.0, left_confidence) if left
                              else (right, 1.0, right_confidence) if right else (None, 0, 0))
    if line is None:
        return None
    c = line.c + side * width / 2.0 * math.sqrt(1.0 + line.b ** 2)
    psi = -math.atan(line.b)
    return Seen(-c * math.cos(psi), psi, width, 1, confidence)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def across_covariance(heading: float, across: float, along: float, yaw: float
                      ) -> Tuple[float, ...]:
    """Return the 6 x 6 covariance (row-major): x, y from the lane's axes; yaw; LOOSE rest."""
    c, s = math.cos(heading), math.sin(heading)
    va, vc = along ** 2, across ** 2
    # R diag(along, across) R^T, R the lane's heading.
    xx = va * c * c + vc * s * s
    yy = va * s * s + vc * c * c
    xy = (va - vc) * c * s
    cov = [0.0] * 36
    cov[0], cov[1], cov[6], cov[7] = xx, xy, xy, yy
    cov[14] = cov[21] = cov[28] = LOOSE
    cov[35] = yaw ** 2
    return tuple(cov)


class LanePose:
    """The gates, and the last offset taken (for the jump gate). One per lane finder."""

    def __init__(self, lanes: LaneMap, settings: Settings = Settings()):
        self.lanes, self.s = lanes, settings
        self.last: Optional[Tuple[float, float]] = None     # (t, d) of the last fix
        self.counts: Dict[str, int] = {}

    def update(self, t: float, pose: Tuple[float, float, float], seen: Optional[Seen]
               ) -> Tuple[Optional[Fix], str]:
        """
        Return (the fix, 'ok') or (None, the reason), for one picture at time t (s).

        pose: where the filter has base_link at t, map frame (x, y, yaw).
        """
        fix, reason = self._update(t, pose, seen)
        self.counts[reason] = self.counts.get(reason, 0) + 1
        return fix, reason

    def _update(self, t, pose, seen):
        s = self.s
        if seen is None:
            return None, 'no line'
        if seen.confidence < s.min_confidence:
            return None, 'confidence'
        if abs(seen.heading) > s.max_heading:
            return None, 'turned'
        x, y, yaw = pose
        lane = self.lanes.lane_at(x, y, yaw)
        if lane is None:
            return None, 'off road'
        if not self._clear(lane):
            return None, 'junction'
        if seen.lines == 2 and abs(seen.width - (lane.left_soft + lane.right_soft)) \
                > s.width_tolerance:
            return None, 'width'
        if (self.last is not None and 0.0 <= t - self.last[0] < s.jump_window
                and abs(seen.offset - self.last[1]) > s.max_jump):
            return None, 'jump'
        nx, ny = -math.sin(lane.heading), math.cos(lane.heading)    # left of the lane
        estimate = (x - lane.x) * nx + (y - lane.y) * ny
        if abs(seen.offset - estimate) > s.max_innovation:
            return None, 'innovation'
        self.last = (t, seen.offset)
        across = s.sigma_both if seen.lines == 2 else s.sigma_one
        sigma_yaw = s.sigma_yaw * (1.0 if seen.lines == 2 else 2.0)
        return Fix(x=lane.x + seen.offset * nx, y=lane.y + seen.offset * ny,
                   yaw=_wrap(lane.heading + seen.heading),
                   covariance=across_covariance(lane.heading, across, s.sigma_along, sigma_yaw),
                   offset=seen.offset, road=lane.road), 'ok'

    def _clear(self, lane: Lane) -> bool:
        """Return True if one road, and no other's tarmac, runs from `behind` to `ahead` m."""
        s = self.s
        for sign, reach in ((-1.0, s.behind), (1.0, s.ahead)):
            here = lane
            gone = 0.0
            while True:
                if [r.name for r in self.lanes.roads_at(here.x, here.y)] != [lane.road]:
                    return False
                if gone >= reach:
                    break
                step = min(s.step, reach - gone)
                gone += step
                nxt = self.lanes.lane_at(here.x + sign * step * math.cos(here.heading),
                                         here.y + sign * step * math.sin(here.heading),
                                         here.heading)
                # Past the road's end the lane's point stays at the end, so it moves too
                # little (on a bend the chord still moves nearly a whole step).
                if (nxt is None or nxt.road != lane.road
                        or math.hypot(nxt.x - here.x, nxt.y - here.y) < 0.8 * step):
                    return False
                here = nxt
        return True

    def summary(self) -> str:
        """Return the counts by reason, as one line, and start counting again."""
        line = ', '.join(f'{k} {v}' for k, v in sorted(self.counts.items())) or 'nothing'
        self.counts = {}
        return line


def lines_from(fields: dict) -> Tuple[Optional[Line], Optional[Line]]:
    """Return the left and right lines from LaneGeometry's fields (None: not detected)."""
    left = (Line(fields['left_poly_a'], fields['left_poly_b'], fields['left_poly_c'])
            if fields['left_lane_detected'] else None)
    right = (Line(fields['right_poly_a'], fields['right_poly_b'], fields['right_poly_c'])
             if fields['right_lane_detected'] else None)
    return left, right
