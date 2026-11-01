"""
A pretend car for the trackers' tests: ego's model, then Gazebo's actuator model, a flat plane.

The same chain as aav_drive_to_point's fake_car.py (ego -> gazebo_adapter's actuator model
-> the car), driven the way the node drives it: a tracker and the speed policy at 20 Hz,
ego and the car at 100 Hz in between. No ROS, no Gazebo.
"""

from collections import deque
from dataclasses import dataclass, field
import math
import time
from typing import List, Tuple

from aav_ego.ackermann import EgoModel, Limits as EgoLimits
from aav_ego.vehicle_model import VehicleModel
from aav_gazebo.actuator_model import ActuatorConfig, Actuators
from aav_track.checks import GoalCheck
from aav_track.path import Path
from aav_track.speed import SpeedPolicy
from aav_track.trackers import Limits, make, MODEL_TRACKERS, State

LIMITS = Limits(wheelbase=1.57, max_steer=0.5, max_steer_rate=0.6)
MODEL = VehicleModel()


def tracker(name, **params):
    return make(name, LIMITS, MODEL if name in MODEL_TRACKERS else None, **params)


@dataclass
class Run:
    """Every tick: (t, x, y, yaw, speed, steer asked, cross track m, heading error rad)."""

    rows: List[Tuple[float, ...]] = field(default_factory=list)
    reached: bool = False
    step_ms: List[float] = field(default_factory=list)
    asked: List[float] = field(default_factory=list)      # m/s, each tick

    def cross(self, after=0.0, beyond=-1e9):
        """Return |cross track| from `after` s on, where x is past `beyond` m."""
        return [abs(r[6]) for r in self.rows if r[0] >= after and r[1] >= beyond]

    def steers(self):
        return [r[5] for r in self.rows]


def follow(trk, poses, start, cruise=1.0, period=0.05, max_time=120.0, physics=0.01,
           delay=0.0):
    """
    Drive the pretend car along `poses` (x, y, yaw) with `trk`, from `start` (x, y, yaw).

    delay: s by which the tracker's view of the car is late (the EKF, DDS, a busy computer).
    """
    path = Path(poses)
    trk.reset(path)
    if getattr(trk, 'plans_speed', False):
        trk.cruise = cruise           # mpc_full: the same cruise as the policy's
    policy = SpeedPolicy(cruise=cruise, accel=1.0, max_brake=3.0)
    goal = GoalCheck(xy=1.0, yaw=0.75)
    ego = EgoModel(EgoLimits(max_speed=5.0, wheelbase=LIMITS.wheelbase,
                             max_steering_angle=LIMITS.max_steer, max_steering_rate=0.6))
    car = Actuators(ActuatorConfig())
    x, y, yaw = start
    speed = asked = steer = 0.0
    t, run, hint = 0.0, Run(), 0
    seen = deque(maxlen=round(delay / period) + 1)      # seen[0]: delay s ago
    while t < max_time:
        foot = path.project((x, y), hint)
        hint = foot.index
        if goal.done(x, y, yaw, (*path.end, path.goal_yaw)):
            run.reached = True
            return run
        t0 = time.perf_counter()
        seen.append((x, y, yaw, speed))
        cmd = trk.step(State(*seen[0], steer=steer), period)
        run.step_ms.append(1000.0 * (time.perf_counter() - t0))
        steer = cmd.steer
        if cmd.speed is None:
            asked = policy.step(path, foot.s, asked, speed, period, cmd.speed_cap)
        else:
            asked = cmd.speed          # mpc_full: its own plan
        run.asked.append(asked)
        w = asked * math.tan(steer) / LIMITS.wheelbase
        run.rows.append((t, x, y, yaw, speed, steer, foot.cross,
                         math.remainder(yaw - foot.heading, 2 * math.pi)))
        for _ in range(round(period / physics)):
            ego_v, ego_d, _ = ego.step((asked, w), physics)
            s = car.step(ego_v, ego_d, physics)
            speed = s.speed
            yaw += speed * math.tan(s.steering) / LIMITS.wheelbase * physics
            x += speed * math.cos(yaw) * physics
            y += speed * math.sin(yaw) * physics
            t += physics
    return run


def straight(length=40.0, step=0.1, y=0.0):
    n = int(length / step)
    return [(i * step, y, 0.0) for i in range(n + 1)]


def arc(radius=8.0, angle=math.pi / 2, lead=5.0, tail=10.0, step=0.1):
    """Straight on for `lead` m, a left turn of `radius` through `angle`, straight `tail` m."""
    pts = [(i * step, 0.0, 0.0) for i in range(int(lead / step))]
    cx, cy = lead, radius
    for i in range(int(radius * angle / step) + 1):
        a = i * step / radius
        pts.append((cx + radius * math.sin(a), cy - radius * math.cos(a), a))
    ex, ey, h = pts[-1]
    for i in range(1, int(tail / step) + 1):
        pts.append((ex + i * step * math.cos(h), ey + i * step * math.sin(h), h))
    return pts


def uturn(radius=3.6, swing=math.pi / 3, lead=6.0, tail=12.0, step=0.05):
    """
    Return a forward-only U-turn: Smac's light bulb.

    Right `swing`, left pi + 2 swing, right `swing`, every bend `radius`; it comes back on
    its own line (y = 0), facing the other way, and spans y = -radius..radius: a 14 m
    lane's middle is y = 0.
    """
    pts = [(0.0, 0.0, 0.0)]
    x = y = h = 0.0
    for length, k in ((lead, 0.0), (radius * swing, -1.0 / radius),
                      (radius * (math.pi + 2 * swing), 1.0 / radius),
                      (radius * swing, -1.0 / radius), (tail, 0.0)):
        n = max(1, int(length / step))
        ds = length / n
        for _ in range(n):
            if k == 0.0:
                x, y = x + ds * math.cos(h), y + ds * math.sin(h)
            else:
                h2 = h + k * ds
                x += (math.sin(h2) - math.sin(h)) / k
                y -= (math.cos(h2) - math.cos(h)) / k
                h = h2
            pts.append((x, y, h))
    return pts


# The car's outline about base_link (vehicle.yaml body: 2 x 1 m, 0.2 m behind the axle).
BODY = ((1.8, 0.5), (1.8, -0.5), (-0.2, -0.5), (-0.2, 0.5))


def widest(run):
    """Return the largest |y| any corner of the car's body reached, m."""
    return max(abs(y + bx * math.sin(yaw) + by * math.cos(yaw))
               for _, _, y, yaw, *_ in run.rows for bx, by in BODY)
