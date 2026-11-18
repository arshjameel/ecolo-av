"""
A pretend car for the tests: ego's model, then Gazebo's actuator model, then a flat plane.

The same chain as `pixi run sim`, without ROS or Gazebo, so the tests can check that the
analysis gets vehicle.yaml's numbers back out of what the car did.
"""

import math

from aav_actuator_tests import procedures as pr
from aav_actuator_tests.reports import COLUMNS
from aav_ego.ackermann import EgoModel, Limits
from aav_sim_models.actuator_model import ActuatorConfig, Actuators

NAN = float('nan')
DT = 0.02   # s: sim_adapter's 50 Hz


def drive(steps, cfg=ActuatorConfig(), limits=None, dt=DT, max_time=900.0):
    """Run a test's steps on the pretend car; return the log's rows and the runner."""
    limits = limits or Limits(max_speed=5.0)
    ego = EgoModel(limits)
    car = Actuators(cfg)
    runner = pr.Runner(list(steps))
    t = distance = heading = x = y = 0.0
    speed = 0.0
    rows = []
    while t < max_time:
        cmd = runner.tick(pr.Obs(t, speed, distance, heading))
        if cmd is None:
            return rows, runner
        w = cmd.speed * math.tan(cmd.steering) / limits.wheelbase
        ego_v, ego_d, _ = ego.step((cmd.speed, w), dt)
        s = car.step(ego_v, ego_d, dt)
        yaw_rate = s.speed * math.tan(s.steering) / limits.wheelbase
        row = dict.fromkeys(COLUMNS, NAN)
        row.update(t=t, phase=cmd.phase, cmd_speed=cmd.speed, cmd_steer=cmd.steering,
                   speed=s.speed, steer_cmd=s.steer_command, steer=s.steering,
                   throttle_cmd=s.throttle_command, throttle=s.throttle,
                   brake_cmd=s.brake_command, brake=s.brake, ax=s.accel, ay=0.0, gz=yaw_rate,
                   heading=heading, distance=distance, x=x, y=y, yaw=heading)
        rows.append(row)
        speed = s.speed
        distance += abs(speed) * dt
        heading += yaw_rate * dt
        x += speed * math.cos(heading) * dt
        y += speed * math.sin(heading) * dt
        t += dt
    raise AssertionError('the test never finished')
