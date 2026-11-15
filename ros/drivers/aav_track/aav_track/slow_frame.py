"""
Follow in odom, correct slowly (TODO 186): the held map -> odom, moved at a bounded rate.

No ROS here. The same rule as aav_mppi_lag's `map_correction.rs`. The follower drives on
the smooth local odometry; the global EKF's map -> odom jumps with each GPS fix (the
simulator: about 0.09 m sideways, 5 times a second). Moving each new path by the newest
map -> odom moves it under the car: a fake cross-track error the tracker steers at.

```text
  newest map -> odom --(jumps)--> held map -> odom: at most `speed` m/s and `turn_rate`
                                  rad/s, measured at the car; a gap over `snap_distance`
                                  m or `snap_angle` rad is taken at once (relocalised)
```

A transform here is odom's pose in map: (x, y, yaw). Speed or turn rate 0: off, the old
way (each new path by the newest map -> odom, nothing between).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from aav_track.path import wrap

Pose = Tuple[float, float, float]


def into_map(p: Pose, odom_in_map: Pose) -> Pose:
    """Return a pose in odom as a pose in map."""
    ox, oy, oyaw = odom_in_map
    c, s = math.cos(oyaw), math.sin(oyaw)
    x, y, yaw = p
    return (ox + c * x - s * y, oy + s * x + c * y, wrap(yaw + oyaw))


def into_odom(p: Pose, odom_in_map: Pose) -> Pose:
    """Return a pose in map as a pose in odom."""
    ox, oy, oyaw = odom_in_map
    c, s = math.cos(oyaw), math.sin(oyaw)
    dx, dy = p[0] - ox, p[1] - oy
    return (c * dx + s * dy, -s * dx + c * dy, wrap(p[2] - oyaw))


@dataclass(frozen=True)
class SlewLimits:
    """How fast the held map -> odom may move, and when it jumps instead."""

    speed: float = 0.1                          # m/s at the car; 0: off
    turn_rate: float = math.radians(1.0)        # rad/s; 0: off
    snap_distance: float = 2.0                  # m: a relocalisation, taken at once
    snap_angle: float = math.radians(10.0)      # rad

    @property
    def on(self) -> bool:
        return self.speed > 0.0 and self.turn_rate > 0.0


class SlowFrame:
    """The held map -> odom: odom's pose in map, moved toward the newest within limits."""

    def __init__(self, limits: SlewLimits = SlewLimits()):
        self.limits = limits
        self.held: Optional[Pose] = None

    def for_new_path(self, newest: Pose, driving: bool) -> Pose:
        """
        Return the map -> odom a new path is placed by.

        Off, not driving, or nothing held yet: the newest, which is then held. Otherwise
        the held one, so a new path lands where the last one was.
        """
        if self.held is not None and driving and self.limits.on:
            return self.held
        self.held = newest
        return newest

    def update(self, newest: Pose, car: Pose, dt: float) -> Optional[str]:
        """
        Step `dt` s toward `newest`, the car at `car` (odom).

        Return None (off, or nothing moved), 'slewed', or 'snapped'.
        """
        if not self.limits.on:
            return None
        if self.held is None:
            self.held = newest
            return None
        at_held = into_map(car, self.held)
        at_newest = into_map(car, newest)
        dx, dy = at_newest[0] - at_held[0], at_newest[1] - at_held[1]
        distance = math.hypot(dx, dy)
        angle = wrap(newest[2] - self.held[2])
        if not all(math.isfinite(v) for v in (distance, angle, dt)):
            return None
        old = self.held
        if distance > self.limits.snap_distance or abs(angle) > self.limits.snap_angle:
            self.held = newest
            return 'snapped'
        dt = max(dt, 0.0)
        step = self.limits.speed * dt
        f = step / distance if distance > step else 1.0
        turn = self.limits.turn_rate * dt
        car_map = (at_held[0] + f * dx, at_held[1] + f * dy,
                   at_held[2] + max(-turn, min(turn, angle)))
        yaw = wrap(car_map[2] - car[2])
        c, s = math.cos(yaw), math.sin(yaw)
        self.held = (car_map[0] - (c * car[0] - s * car[1]),
                     car_map[1] - (s * car[0] + c * car[1]), yaw)
        return 'slewed' if self.held != old else None

    def gap(self, newest: Pose, car: Pose) -> float:
        """Return m between where the held and the newest map -> odom put the car."""
        if self.held is None:
            return 0.0
        a, b = into_map(car, self.held), into_map(car, newest)
        return math.hypot(a[0] - b[0], a[1] - b[1])
