"""
A pretend hallway session: known numbers in, the hall-* passes driven, a session folder out.

The same chain as Ravensim (the test's steps, ego's model, the spine's speed loop, the car
model) without ROS, with what the real car's sensors add: an IMU with noise, a bias and the
floor's slope; wheels with noise and slip. hallway.analyse must give the numbers back
within their error bars (test_hallway.py). Run it alone for the recovery table:

    python3 ros/drivers/aav_actuator_tests/test/fake_hallway.py [folder]
"""

from dataclasses import replace
import math
import os
import random
import sys

if __name__ == '__main__':
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import conftest  # noqa: F401  (the source tree's packages)

from aav_actuator_tests import procedures as pr
from aav_actuator_tests import reports
from aav_actuator_tests import safety
from aav_description.car_model import ActuatorConfig, Actuators, speed_loop_profile
from aav_ego.ackermann import EgoModel, Limits
import yaml

NAN = float('nan')
DT = 0.02
G = 9.80665

# The numbers the pretend car runs: not vehicle.yaml's (its guesses), on purpose.
TRUE = {'max_accel': 1.2, 'throttle_lag': 0.30, 'throttle_dead_time': 0.10,
        'max_brake': 2.5, 'brake_lag': 0.25, 'brake_dead_time': 0.06, 'drag': 0.20,
        'steer_lag': 0.20, 'steer_dead_time': 0.08, 'steer_rate': 0.8}
# vehicle.yaml's guesses, as the drivers read them (the "now" column).
GUESS = {'actuators.max_accel': 1.5, 'actuators.throttle_lag': 0.1, 'actuators.max_brake': 3.0,
         'actuators.brake_lag': 0.2, 'actuators.drag': 0.1, 'actuators.steer_rate': 0.8,
         'actuators.steer_lag': 0.1, 'actuators.steer_offset': 0.0,
         'actuators.steer_deadband': 0.005, 'actuators.throttle_deadband': 0.0,
         'vehicle.wheelbase': 1.561, 'vehicle.max_steering_angle': 0.5, 'vehicle.mass': 181.0}


def drive_pass(test, cfg, hall, direction, seed, imu_noise=0.2, imu_bias=0.05, slope=0.0,
               wheel_noise=0.05, wheel_slip=0.02, steer_sensor=True):
    """Drive one hall-* test on the pretend car; return its log rows and its meta."""
    rng = random.Random(seed)
    lim = Limits(max_speed=1.0, wheelbase=cfg.wheelbase, max_steering_angle=cfg.max_steering,
                 max_steering_rate=3.0)
    ego = EgoModel(lim)
    car = Actuators(cfg, speed_loop_profile('default'))
    s = pr.Settings(cap=hall.top, max_steering=cfg.max_steering, wheelbase=cfg.wheelbase,
                    room=hall.room, stop_decel=safety.HALL_DECEL,
                    reaction=safety.HALL_REACTION)
    plan = pr.plan(test, s)
    runner = pr.Runner(list(plan.steps), room=s.room, stop_decel=s.stop_decel,
                       reaction=s.reaction)
    pitch = slope * (1.0 if direction == 'there' else -1.0)
    t = distance = heading = 0.0
    speed_read = 0.0
    rows = []
    while t < 300.0:
        cmd = runner.tick(pr.Obs(t, speed_read, distance, heading))
        if cmd is None:
            break
        # Capped as actuator_test caps it (a coast follows a noisy speed: it may ask more).
        cmd = replace(cmd, speed=min(cmd.speed, s.cap))
        w = cmd.speed * math.tan(cmd.steering) / lim.wheelbase
        ego_v, ego_d, _ = ego.step((cmd.speed, w), DT)
        before = car.speed
        st = car.step(ego_v, ego_d, DT)
        if abs(car.speed) > 0.05:          # the floor's slope pulls a rolling car
            car.speed = car.speed - G * math.sin(pitch) * DT
        accel = (car.speed - before) / DT
        yaw_rate = car.speed * math.tan(st.steering) / cfg.wheelbase
        speed_read = car.speed * (1 + wheel_slip) + rng.gauss(0.0, wheel_noise)
        row = dict.fromkeys(reports.COLUMNS, NAN)
        stamp = t + DT
        row.update(t=t, phase=cmd.phase, cmd_speed=cmd.speed, cmd_steer=cmd.steering,
                   speed=speed_read, steer_cmd=st.steer_command,
                   steer=st.steering if steer_sensor else NAN,
                   throttle_cmd=st.throttle_command, throttle=st.throttle,
                   brake_cmd=st.brake_command, brake=st.brake,
                   ax=accel + G * math.sin(pitch) + imu_bias + rng.gauss(0.0, imu_noise),
                   ay=0.0, gz=yaw_rate + rng.gauss(0.0, 0.02), heading=heading,
                   distance=distance, ego_speed=ego_v, ego_steer=ego_d,
                   t_speed=stamp, t_steer=stamp, t_throttle=stamp, t_brake=stamp,
                   t_wheels=stamp, t_imu=stamp, t_ego=stamp)
        rows.append(row)
        distance += abs(car.speed) * DT
        heading += yaw_rate * DT
        t += DT
    meta = {'test': test, 'source': 'SIM', 'wheelbase': cfg.wheelbase,
            'max_steering': cfg.max_steering, 'vehicle_yaml': dict(GUESS),
            'shaping': {'backend': 'ravensim', 'command_lead': DT},
            'hallway': {'length': hall.length, 'margin': hall.margin, 'room': hall.room,
                        'top': hall.top, 'direction': direction},
            'mass': cfg.mass,
            'steps': [{'phase': d.step.phase, 'cut': d.cut} for d in runner.done],
            'max_distance': max(r['distance'] for r in rows)}
    return rows, meta


def write_session(folder, truth=TRUE, length=15.0, repeats=3, both_ways=True, seed=1,
                  kinds=('coast', 'speed-up', 'brake', 'steer'), **noise):
    """Drive a whole session into `folder`; return the folder."""
    cfg = replace(ActuatorConfig(), **truth)
    hall, why = safety.hallway_limits(length)
    assert hall is not None, why
    os.makedirs(folder, exist_ok=True)
    n = 0
    for r in range(repeats):
        for kind in kinds:
            direction = 'back' if both_ways and n % 2 else 'there'
            rows, meta = drive_pass(f'hall-{kind}', cfg, hall, direction, seed + n, **noise)
            sub = os.path.join(folder, f'{n + 1:02d}-hall-{kind}-{direction}', 'run')
            os.makedirs(sub, exist_ok=True)
            with open(os.path.join(sub, 'log.csv'), 'w') as f:
                f.write(','.join(reports.COLUMNS) + '\n')
                for row in rows:
                    f.write(','.join(row['phase'] if k == 'phase' else reports.number(row[k])
                                     for k in reports.COLUMNS) + '\n')
            with open(os.path.join(sub, 'meta.yaml'), 'w') as f:
                yaml.safe_dump(meta, f)
            n += 1
    from aav_actuator_tests.hallway import with_truth
    info = {'started': 'pretend', 'where': 'fake_hallway.py (no ROS)', 'length': length,
            'margin': hall.margin, 'top': hall.top, 'mass': cfg.mass,
            'truth': with_truth(GUESS, truth)}
    with open(os.path.join(folder, 'session.yaml'), 'w') as f:
        yaml.safe_dump(info, f)
    return folder


if __name__ == '__main__':
    from aav_actuator_tests import hallway
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/fake-hallway'
    write_session(out)
    s = hallway.analyse(out)
    hallway.write(s)
    print(hallway.text_table(s))
    print(f'Saved in {out}')
