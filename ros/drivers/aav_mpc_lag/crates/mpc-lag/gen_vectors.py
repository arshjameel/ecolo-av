#!/usr/bin/env python3
"""
Regenerate mpc-lag's shared test vectors, from the Python it ports.

  - model: aav_track/lag_model.py step(), one state at a time, on random states, inputs
    and bends, for three cars and both PI speed loops (default, smooth). The Rust
    (src/model.rs) must give the same states within 1e-9: the same f64 sums.
  - tracker: aav_track/trackers/mpc_lag.py on its pretend car (test/sim_car.py), every
    tick's observation and command, on a bend and the U-turn at 2 m/s. The Rust
    (src/tracker.rs) is fed the same observations and must send the same commands
    within 2e-3 (its estimate runs spine-core's own f32 speed loop, the Python's port of
    it runs in f64; and the QP stops at its tolerance, 1e-4).

Writes ros/drivers/aav_mpc_lag/crates/mpc-lag/vectors.json, read by tests/vectors.rs.

Run it after any change to lag_model.py, mpc_lag.py or qp.py:
  pixi run python3 ros/drivers/aav_mpc_lag/crates/mpc-lag/gen_vectors.py
"""

from dataclasses import fields, replace
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROS = HERE.parents[3]
for package in ('drivers/aav_track', 'drivers/aav_track/test', 'drivers/aav_drive_to_point',
                'bringup/aav_description', 'ego/aav_ego'):
    sys.path.insert(0, str(ROS / package))

from aav_track import lag_model as lm  # noqa: E402
import numpy as np  # noqa: E402
import sim_car  # noqa: E402

OUT = HERE / 'vectors.json'

CARS = {
    'default': lm.CarParams(),
    'laggy': replace(lm.lags_scaled(lm.CarParams(), 2.0), steer_offset=0.01,
                     throttle_deadband=0.05),
    'plan': lm.without_deadbands(lm.CarParams(max_steering=0.6, wheelbase=1.57)),
}


def car_dict(car):
    return {f.name: getattr(car, f.name) for f in fields(car)}


def model_cases(rng):
    cases = []
    for car_name, car in CARS.items():
        for loop_name in ('default', 'smooth'):
            loop = lm.speed_loop(loop_name)
            for _ in range(40):
                x = np.array([
                    rng.uniform(-1.0, 1.0),            # e_y
                    rng.uniform(-0.6, 0.6),            # e_theta
                    rng.uniform(0.0, 20.0),            # s
                    rng.choice([0.0, 0.01, rng.uniform(0.0, 3.0)]),   # v (stopped too)
                    rng.uniform(0.0, 1.0),             # throttle
                    rng.choice([0.0, rng.uniform(0.0, 1.0)]),        # brake
                    rng.uniform(-0.5, 0.5),            # setpoint
                    rng.uniform(-0.5, 0.5),            # motor
                    rng.uniform(-0.5, 0.5),            # wheels
                    rng.uniform(0.0, 0.3),             # integral
                ])
                u = np.array([rng.uniform(-0.7, 0.7),
                              rng.choice([0.0, -0.2, rng.uniform(0.0, 3.0)])])
                if rng.uniform() < 0.2:
                    u[0] = x[lm.SETPOINT] + 0.003      # inside the steering's deadband
                kappa = float(rng.choice([0.0, rng.uniform(-0.35, 0.35)]))
                dt = float(rng.choice([0.05, 0.02, 0.1]))
                substeps = int(rng.choice([1, 2, 4]))
                out = lm.step(car, loop, x, u, kappa, dt, substeps)
                cases.append({'car': car_name, 'loop': loop_name, 'x': x.tolist(),
                              'u': u.tolist(), 'kappa': kappa, 'dt': dt,
                              'substeps': substeps, 'out': [float(v) for v in out]})
    return cases


class Recorder:
    """The tracker, its every observation and command kept."""

    def __init__(self, inner):
        self.inner = inner
        self.ticks = []

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def __setattr__(self, name, value):
        if name in ('inner', 'ticks'):
            object.__setattr__(self, name, value)
        else:
            setattr(self.inner, name, value)

    def step(self, state, dt):
        cmd = self.inner.step(state, dt)
        self.ticks.append({'x': state.x, 'y': state.y, 'yaw': state.yaw, 'v': state.v,
                           'steer': state.steer, 'dt': dt, 'cmd_steer': cmd.steer,
                           'cmd_speed': cmd.speed})
        return cmd


def tracker_case(name, poses, cruise, ticks):
    rec = Recorder(sim_car.tracker('mpc_lag'))
    sim_car.follow(rec, poses, (0.0, 0.0, 0.0), cruise=cruise)
    lim = sim_car.LIMITS
    return {'name': name, 'poses': [list(p) for p in poses], 'cruise': cruise,
            'limits': {'wheelbase': lim.wheelbase, 'max_steer': lim.max_steer,
                       'max_steer_rate': lim.max_steer_rate},
            'car': car_dict(sim_car.CAR), 'ticks': rec.ticks[:ticks]}


def main():
    rng = np.random.default_rng(199)
    data = {
        'cars': {k: car_dict(v) for k, v in CARS.items()},
        'model': model_cases(rng),
        'tracker': [tracker_case('bend_r8_at_2', sim_car.arc(radius=8.0), 2.0, 200),
                    tracker_case('u_turn_at_2', sim_car.uturn(), 2.0, 200)],
    }
    OUT.write_text(json.dumps(data, indent=None, separators=(',', ':')) + '\n')
    print(f'wrote {OUT}: {len(data["model"])} model cases, '
          f'{sum(len(t["ticks"]) for t in data["tracker"])} tracker ticks')


if __name__ == '__main__':
    main()
