"""
A pretend car for the drive tools: no ROS, no time passing. For tests and the model test.

It "drives" each goal at once and exactly: a straight drive moves the car forward by the
distance, an arc turns it by length x curvature. It keeps a list of every goal it was sent,
so a test can see what the tools asked for.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

from aav_language import safety
from aav_language.drive_maths import DriveGoal
from aav_language.drive_tools import BackendError, CarState


class FakeBackend:
    """A car on a stand-in network (LOCALHOST), parked at the origin, GPS at Carleton."""

    def __init__(self, ego: Optional[Dict[str, float]] = None, nav: bool = True,
                 where: Optional[safety.Where] = None):
        self.ego = ego if ego is not None else {
            'max_speed': 1.0, 'max_reverse_speed': 0.5, 'max_acceleration': 1.0,
            'max_deceleration': 2.0, 'wheelbase': 1.57, 'max_steering_angle': 0.5}
        self.nav = nav
        self.place = where or safety.Where('LOCALHOST', '', False, ('ego', 'twist_mux'))
        self.pose = [0.0, 0.0, 0.0]
        self.gps = (45.3855, -75.6965)   # about 80 m from the model test's GPS goal
        self.estop_on = False
        self.fence = None            # /safety/geofence/fence's corners, map frame
        self.mux_silent = False      # True: as if twist_mux's /diagnostics never arrived
        self.sent: List[dict] = []
        self.results: Dict[str, dict] = {}
        self.pending: List[str] = []
        self.slept = 0.0
        self.estops = 0

    def state(self) -> CarState:
        return CarState(speed=0.0, odom_pose=tuple(self.pose), odom_age=0.0,
                        map_pose=tuple(self.pose), gps=self.gps, gps_status=0,
                        estop=self.estop_on, vehicle_estop=self.estop_on, armed=True,
                        mux_locks=None if self.mux_silent else {'estop': self.estop_on,
                                                                'stop_sign': False},
                        mux={'driver': None, 'blocked_by': 'estop' if self.estop_on else None},
                        fence=self.fence)

    def where(self) -> safety.Where:
        return self.place

    def ego_limits(self):
        return self.ego

    def _drive(self, length: float, curvature: float, sign: float):
        x, y, a = self.pose
        if curvature == 0:
            self.pose = [x + sign * length * math.cos(a), y + sign * length * math.sin(a), a]
            return
        turn = sign * length * curvature
        r = 1.0 / curvature
        self.pose = [x + r * (math.sin(a + turn) - math.sin(a)),
                     y - r * (math.cos(a + turn) - math.cos(a)), a + turn]

    def send_scripted(self, goal: DriveGoal) -> str:
        if abs(goal.target_speed) > 2.0:
            raise BackendError('scripted_driver rejected the goal (above its max_speed)')
        goal_id = f'scripted-{len(self.sent) + 1}'
        self.sent.append({'kind': 'scripted', **goal.as_dict(), 'distance': goal.distance})
        self._drive(goal.distance, goal.curvature, math.copysign(1.0, goal.target_speed))
        self.results[goal_id] = {'status': 'succeeded', 'message': 'done',
                                 'distance_travelled': goal.distance,
                                 'duration': goal.expected_duration}
        return goal_id

    def send_gps(self, lat: float, lon: float) -> str:
        goal_id = f'gps-{len(self.sent) + 1}'
        self.sent.append({'kind': 'gps', 'lat': lat, 'lon': lon})
        self.gps = (lat, lon)
        self.results[goal_id] = {'status': 'succeeded', 'message': 'reached the waypoint'}
        return goal_id

    def nav_ready(self) -> bool:
        return self.nav

    def result(self, goal_id: str, timeout: Optional[float]) -> Optional[dict]:
        return self.results.get(goal_id)

    def running(self) -> List[str]:
        return list(self.pending)

    def cancel_all(self) -> List[str]:
        cancelled, self.pending = self.pending, []
        return cancelled

    def estop(self) -> None:
        self.estop_on = True
        self.estops += 1

    def sleep(self, seconds: float) -> None:
        self.slept += seconds
