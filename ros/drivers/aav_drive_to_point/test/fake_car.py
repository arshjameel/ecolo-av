"""
A pretend car for the tests: ego's model, then Gazebo's actuator model, then a flat plane.

The same chain as `pixi run sim` (ego -> sim_adapter's actuator model -> the car), with
no ROS or Gazebo, so a drive to B can be checked end to end in a second.
"""

from dataclasses import dataclass, field
import math
from typing import List, Tuple

from aav_drive_to_point.approach import STOPPING
from aav_drive_to_point.line import Pose
from aav_ego.ackermann import EgoModel, Limits
from aav_sim_models.actuator_model import ActuatorConfig, Actuators


@dataclass
class Run:
    """What happened: (t, x, y, yaw, speed, commanded, phase) every tick."""

    rows: List[Tuple[float, float, float, float, float, float, int]] = field(
        default_factory=list)

    @property
    def end(self):
        return self.rows[-1]

    def decelerations(self, window=0.5):
        """Return (t, the car's deceleration over the next `window` s, m/s^2, + = slowing)."""
        out, j = [], 0
        for i, row in enumerate(self.rows):
            j = max(j, i)
            while j < len(self.rows) and self.rows[j][0] - row[0] < window:
                j += 1
            if j < len(self.rows):
                out.append((row[0], (row[4] - self.rows[j][4]) / (self.rows[j][0] - row[0])))
        return out


def drive_to(make_drive, start=Pose(0.0, 0.0, 0.0), limits=None, cfg=ActuatorConfig(),
             dt=0.02, max_time=300.0, settle=3.0):
    """Drive the pretend car with the Drive that make_drive(start) builds; return the Run."""
    limits = limits or Limits(max_speed=5.0)
    ego, car = EgoModel(limits), Actuators(cfg)
    drive = make_drive(start)
    x, y, yaw, speed, t = start.x, start.y, start.yaw, 0.0, 0.0
    run, stopped_at = Run(), None
    while t < max_time:
        cmd = drive.tick(Pose(x, y, yaw), speed, dt)
        ego_v, ego_d, _ = ego.step((cmd.speed, cmd.speed * cmd.curvature), dt)
        s = car.step(ego_v, ego_d, dt)
        speed = s.speed
        yaw += speed * math.tan(s.steering) / limits.wheelbase * dt
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt
        t += dt
        run.rows.append((t, x, y, yaw, speed, cmd.speed, cmd.phase))
        if cmd.phase == STOPPING and stopped_at is None:
            stopped_at = t
        if stopped_at is not None and t - stopped_at > settle and abs(speed) < 1e-3:
            return run
    raise AssertionError('the drive never finished')
