"""
When a path is done, and when the car has stopped making progress: Nav2's own two checks.

No ROS here. The same rules as controller_server's plugins in nav2.yaml, so every
follower, Nav2's or ours, has the same "done" and the same "stuck":

| Check | Nav2's plugin | Rule |
|---|---|---|
| `GoalCheck` | `SimpleGoalChecker` | within xy_goal_tolerance of the end, then the heading |
| `ProgressCheck` | `PoseProgressChecker` | moved or turned enough in time, else stuck |

GoalCheck: within xy_goal_tolerance of the path's end; then (staying "there" once it was,
as stateful does) the heading within yaw_goal_tolerance. ProgressCheck: moved
required_movement_radius m or turned required_movement_angle rad within
movement_time_allowance s.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Optional, Tuple

from aav_track.path import wrap


@dataclass
class GoalCheck:
    xy: float           # m
    yaw: float          # rad
    stateful: bool = True
    _xy_done: bool = field(default=False, repr=False)

    def reset(self):
        self._xy_done = False

    def done(self, x: float, y: float, yaw: float, goal: Tuple[float, float, float]) -> bool:
        gx, gy, gyaw = goal
        if not self._xy_done:
            if math.hypot(gx - x, gy - y) > self.xy:
                return False
            self._xy_done = self.stateful
        return abs(wrap(gyaw - yaw)) <= self.yaw


@dataclass
class ProgressCheck:
    radius: float       # m
    angle: float        # rad
    time: float         # s
    _base: Optional[Tuple[float, float, float, float]] = field(default=None, repr=False)

    def reset(self):
        self._base = None

    def stuck(self, x: float, y: float, yaw: float, now: float) -> bool:
        """Return True once the car has neither moved nor turned enough for `time` s."""
        if self._base is None:
            self._base = (x, y, yaw, now)
            return False
        bx, by, byaw, t0 = self._base
        if math.hypot(x - bx, y - by) >= self.radius or abs(wrap(yaw - byaw)) >= self.angle:
            self._base = (x, y, yaw, now)
            return False
        return now - t0 > self.time
