"""
Pure pursuit on a planned path: the trial's control, with the same speed as the others.

No ROS here. aav_drive_to_point's line.pursuit_curvature (k = 2 y / d^2 to a point in the
car's frame) aimed at a carrot `lookahead` m along the path from the car's foot on it;
the lookahead grows with speed as Regulated Pure Pursuit's does (nav2.yaml's
lookahead_time, min_lookahead_dist, max_lookahead_dist). Past the path's end the carrot
runs on straight, so the car keeps its heading while it stops.
"""

from __future__ import annotations

import math

from aav_drive_to_point.line import Pose, pursuit_curvature
from aav_track.trackers import clamp, Command, Limits, State, Tracker


class PurePursuit(Tracker):
    name = 'pure_pursuit'

    def __init__(self, limits: Limits, lookahead_min: float = 1.5,
                 lookahead_max: float = 4.0, lookahead_time: float = 1.5):
        super().__init__(limits)
        if not 0 < lookahead_min <= lookahead_max:
            raise ValueError('the lookahead needs 0 < min <= max')
        self.lookahead_min, self.lookahead_max = lookahead_min, lookahead_max
        self.lookahead_time = lookahead_time

    def lookahead(self, v: float) -> float:
        return clamp(abs(v) * self.lookahead_time, self.lookahead_min, self.lookahead_max)

    def step(self, state: State, dt: float) -> Command:
        foot = self.foot(state.x, state.y)
        ld = self.lookahead(state.v)
        # ld from the car, along the path (line.Line.carrot's rule); off the path by more
        # than ld, the foot itself: back to the path first.
        on = math.sqrt(ld * ld - foot.cross ** 2) if abs(foot.cross) < ld else 0.0
        carrot = self.path.point_at(foot.s + on)
        lim = self.limits
        k = pursuit_curvature(Pose(state.x, state.y, state.yaw), carrot,
                              math.tan(lim.max_steer) / lim.wheelbase)
        steer = self.limited(math.atan(lim.wheelbase * k), state.steer, dt)
        return Command(steer, debug={'lookahead_m': ld})
