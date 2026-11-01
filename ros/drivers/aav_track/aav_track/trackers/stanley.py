"""
Stanley steering (Thrun et al., 2006): the heading error, and the front axle's offset.

No ROS here.

    delta = theta_e + atan(k e_f / (v_soft + v))

theta_e is the path's heading at the front axle's foot minus the car's; e_f the front
axle's distance right of the path (+ steers left). The front axle is base_link plus the
wheelbase ahead. v_soft keeps the gain finite near a standstill (where the car doesn't
steer anyway: ego holds the wheels below its min_speed_for_steering).
"""

from __future__ import annotations

import math

from aav_track.path import wrap
from aav_track.trackers import Command, Limits, State, Tracker


class Stanley(Tracker):
    name = 'stanley'

    def __init__(self, limits: Limits, gain: float = 1.0, soft_speed: float = 1.0,
                 yaw_rate_gain: float = 0.0):
        super().__init__(limits)
        if gain <= 0 or soft_speed <= 0:
            raise ValueError('gain and soft_speed must be above 0')
        self.gain, self.soft_speed, self.yaw_rate_gain = gain, soft_speed, yaw_rate_gain

    def step(self, state: State, dt: float) -> Command:
        L = self.limits.wheelbase
        fx = state.x + L * math.cos(state.yaw)
        fy = state.y + L * math.sin(state.yaw)
        foot = self.foot(fx, fy)
        heading = wrap(foot.heading - state.yaw)
        # foot.cross is + when the axle is left of the path: steer right, so minus.
        offset = math.atan2(-self.gain * foot.cross, self.soft_speed + max(state.v, 0.0))
        steer = heading + offset - self.yaw_rate_gain * (
            state.yaw_rate - max(state.v, 0.0) * foot.curvature)
        steer = self.limited(steer, state.steer, dt)
        return Command(steer, debug={'front_cross_m': foot.cross, 'heading_rad': heading})
