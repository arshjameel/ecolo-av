#!/usr/bin/env python3
"""
Write point-core's agreement vectors from aav_drive_to_point's Python modules.

Every function the Rust twin ports, on chosen and random inputs: approach.py's speed plan,
line.py's pursuit and corridor, drive.py's ticks, rules.py's refusals and stops, and
run.py's whole drives (a plain kinematic car driven by the commands: to B, an e-stop, a
cancel, a car that won't move, a speeding one, a GPS jump, a stopping-law drive). The Rust
(crates/point-core, tests/vectors.rs) must give the same numbers (to 1e-12) and words.
aav_drive_to_point's test_rust_vectors.py checks the Python still gives this file's answers.

Writes ros/drivers/aav_drive_to_point_rs/crates/point-core/tests/vectors.json:

  pixi run python ros/drivers/aav_drive_to_point_rs/gen_vectors.py
"""

import json
import math
from pathlib import Path
import random
import sys

HERE = Path(__file__).resolve().parent
ROS = HERE.parents[1]
sys.path[:0] = [str(HERE.parent / 'aav_drive_to_point'), str(HERE.parent / 'aav_actuator_tests'),
                str(ROS / 'bringup' / 'aav_description')]

from aav_actuator_tests.safety import Kind, source_kind, taken_over  # noqa: E402
from aav_description.car_model import ActuatorConfig  # noqa: E402
from aav_drive_to_point import rules, run  # noqa: E402
from aav_drive_to_point.approach import Approach, planned_deceleration  # noqa: E402
from aav_drive_to_point.drive import Drive, Steering  # noqa: E402
from aav_drive_to_point.line import (bearing_error, Fence, Line, min_radius, Pose,  # noqa: E402
                                     pursuit_curvature, reachable, turn_speed_limit)

OUT = HERE / 'crates' / 'point-core' / 'tests' / 'vectors.json'
WHEELBASE, STEER = 1.57, 0.5


def enc(v):
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    if isinstance(v, (list, tuple)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


def text(why):
    return None if why is None else str(why)


def approaches(rng):
    """approach.py: step, braking_speed, reach and ideal_time, with and without the curve."""
    cases = []
    curve = run.stop_law.from_car_model(ActuatorConfig(), 2.0, 2.0)
    for i in range(400):
        plan = {'cruise': rng.choice((1.0, 3.0, rng.uniform(0.2, 4.0))),
                'accel': rng.uniform(0.2, 1.5), 'decel': rng.uniform(0.5, 2.5),
                'lead': rng.choice((0.0, 0.8, rng.uniform(0, 1.5))),
                'stop_band': 0.05, 'creep': rng.choice((0.0, 0.1))}
        a = Approach(**plan, curve=curve if i % 3 == 0 else None)
        step = {'previous': rng.uniform(0, 4),
                'dt': rng.choice((0.02, -0.1, rng.uniform(0, 0.6))),
                'left': rng.choice((rng.uniform(-2, 60), rng.uniform(0, 1), 0.05, 0.0)),
                'measured': rng.choice((rng.uniform(-0.5, 4), 0.0)),
                'limit': rng.choice((math.inf, rng.uniform(0.3, 3)))}
        speed, phase = a.step(**step)
        cases.append({'plan': plan, 'curve': i % 3 == 0, 'step': step,
                      'speed': speed, 'phase': phase,
                      'braking': a.braking_speed(step['left'], step['measured']),
                      'reach': a.reach(step['left']),
                      'ideal': a.ideal_time(abs(step['left']) + 1.0)})
    errors = []
    for args in ((0.0, 0.5), (3.0, 1.5), (3.0, 0.0), (float('nan'), 0.5), (3.0, 1.0)):
        try:
            errors.append({'args': args, 'value': planned_deceleration(*args)})
        except ValueError as e:
            errors.append({'args': args, 'error': str(e)})
    return {'curve': [curve.a, curve.j, curve.rho], 'cases': cases, 'planned': errors}


def lines(rng):
    """line.py: pursuit, bearing, reachable, progress, carrot, the fence's words."""
    cases = []
    radius = min_radius(WHEELBASE, STEER)
    for _ in range(400):
        pose = Pose(rng.uniform(-20, 20), rng.uniform(-20, 20), rng.uniform(-4, 4))
        point = (rng.uniform(-30, 30), rng.uniform(-30, 30))
        a = (rng.uniform(-20, 20), rng.uniform(-20, 20))
        line = Line(a, (a[0] + rng.uniform(1, 40), a[1] + rng.uniform(-40, 40)))
        p = (rng.uniform(-30, 40), rng.uniform(-30, 40))
        look = rng.uniform(0.5, 6)
        fence = Fence(line.a, line.b, radius, rng.uniform(0.5, 4), rng.uniform(0.5, 3))
        cases.append({
            'pose': [pose.x, pose.y, pose.yaw], 'point': point,
            'curvature': pursuit_curvature(pose, point, 0.35),
            'bearing': bearing_error(pose, point),
            'reachable': reachable(pose, point, radius),
            'line': [line.a, line.b], 'p': p, 'look': look,
            'progress': line.progress(p), 'carrot': line.carrot(p, look),
            'fence': [fence.max_off_line, fence.max_overrun],
            'breach_line': text(fence.breach(p, line)), 'breach': text(fence.breach(p)),
            'turn_limit': turn_speed_limit(rng.choice((0.0, 1e-7, rng.uniform(-0.4, 0.4))), 1.5),
        })
    return {'radius': radius, 'cases': cases}


def kinematic(pose, speed, cmd, dt, lag):
    """Move the plain car one step: speed lags the command; yaw by the curvature."""
    speed += (cmd[0] - speed) * min(1.0, dt / lag)
    yaw = pose.yaw + speed * cmd[1] * dt
    return Pose(pose.x + speed * math.cos(yaw) * dt, pose.y + speed * math.sin(yaw) * dt,
                yaw), speed


def drives(rng):
    """drive.py: whole drives, tick by tick; the Rust replays the same poses and speeds."""
    out = []
    radius = min_radius(WHEELBASE, STEER)
    for i in range(5):
        start = Pose(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-3, 3))
        angle = start.yaw + rng.choice((0.0, 0.3, 1.2, 2.6, -2.0))
        dist = rng.uniform(6, 15)
        target = (start.x + dist * math.cos(angle), start.y + dist * math.sin(angle))
        plan = {'cruise': rng.choice((1.0, 2.5)), 'accel': 0.5, 'decel': 1.5, 'lead': 0.8,
                'stop_band': 0.05, 'creep': 0.1}
        drive = Drive(target, Approach(**plan), Steering(max_curvature=1.0 / radius), start)
        pose, speed, ticks = start, 0.0, []
        for _ in range(3000):
            dt = 0.02 + rng.uniform(-0.002, 0.002)
            cmd = drive.tick(pose, speed, dt)
            # [pose, measured, dt, (speed, curvature, phase, left, cross)]
            ticks.append([[pose.x, pose.y, pose.yaw], speed, dt,
                          [cmd.speed, cmd.curvature, cmd.phase, cmd.left, cmd.cross]])
            pose, speed = kinematic(pose, speed, (cmd.speed, cmd.curvature), dt, 0.3)
            if cmd.phase == 4 and abs(speed) < 1e-3:
                break
        out.append({'start': [start.x, start.y, start.yaw], 'target': target, 'plan': plan,
                    'max_curvature': 1.0 / radius, 'ticks': ticks})
    return out


def now_of(rng):
    def age():
        return rng.choice((None, rng.uniform(0, 0.4), rng.uniform(0, 3), float('nan')))
    return rules.Now(pose_age=age(), fix_age=age(), estop=rng.random() < 0.15,
                     locked=rng.random() < 0.15, kind=rng.choice(list(Kind)),
                     taken_over=rng.random() < 0.15, lock_age=age())


def now_dict(now):
    """[pose_age, fix_age, estop, locked, kind, taken_over, lock_age]."""
    return [now.pose_age, now.fix_age, now.estop, now.locked, now.kind.value, now.taken_over,
            now.lock_age]


def limits_dict(lim):
    return {'max_distance': lim.max_distance, 'pose_timeout': lim.pose_timeout,
            'gps_timeout': lim.gps_timeout, 'require_gps': lim.require_gps,
            'watchdog_timeout': lim.watchdog_timeout}


def rule_cases(rng):
    """rules.py: every refusal and stop, and the small helpers."""
    radius = min_radius(WHEELBASE, STEER)
    cases = []
    for _ in range(500):
        now = now_of(rng)
        lim = rules.Limits(max_distance=rng.choice((200.0, 30.0)),
                           require_gps=rng.random() < 0.7,
                           watchdog_timeout=rng.choice((1.0, 0.0)))
        confirmed = rng.random() < 0.5
        pose = Pose(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-3, 3))
        target = rng.choice(((rng.uniform(-60, 60), rng.uniform(-60, 60)),
                             (pose.x + 0.2, pose.y), (pose.x, pose.y + 3.0),
                             (float('nan'), 1.0)))
        nums = [rng.choice((0.0, 1.0, -0.5, float('nan'), float('inf'), 1e-05, 2.5))
                for _ in range(3)]
        depth = rng.choice((None, rng.uniform(-5, 5)))
        cases.append({'now': now_dict(now), 'limits': limits_dict(lim), 'confirmed': confirmed,
                      'start': text(rules.refuse_start(now, lim, confirmed)),
                      'stop': text(rules.stop_reason(now, lim)),
                      'pose': [pose.x, pose.y, pose.yaw], 'target': target,
                      'target_why': text(rules.refuse_target(pose, target, lim, radius)),
                      'numbers': nums, 'numbers_why': text(rules.check_numbers(*nums)),
                      'depth': depth, 'outside': text(rules.refuse_outside(depth))})
    small = {
        'tolerance': [[s, rules.tolerance(s)] for s in (None, -1, 0, 1, 2)],
        'confirmed': [[t, rules.confirmed(t)] for t in (None, 'drive', ' drive\n', 'Drive', '')],
        'not_moving': [[c, m, rules.not_moving(c, m)] for c, m in
                       ((0.3, 0.0), (0.1, 0.0), (0.3, 0.1), (0.2, -0.04))],
        'source_kind': [[vals, source_kind({'a': vals}).value] for vals in
                        ([], [1], [1, 0], [1, 3], [2, 1], [7], [3, 2])],
        'taken_over': [[ours, theirs, taken_over(ours, theirs)] for ours, theirs in
                       (([(1.0, 0.1)], (1.0, 0.1)), ([(1.0, 0.1)], (1.0005, 0.1)),
                        ([(1.0, 0.1)], (0.5, 0.1)), ([], (0.0, 0.0)))],
    }
    return {'radius': radius, 'cases': cases, 'small': small}


def plans(rng):
    """run.plan: the goal's numbers, ego's caps, the settings; with and without the car."""
    out = []
    keys = ('max_speed', 'max_acceleration', 'max_deceleration', 'max_jerk')
    for i in range(200):
        caps = {k: rng.uniform(0.3, 4) for k in keys if rng.random() < 0.6}
        s = run.Settings(deceleration=rng.choice((0.0, 1.2)),
                         cruise_speed=rng.choice((1.0, 2.0)), creep=rng.choice((0.1, 2.5)))
        goal = [rng.choice((0.0, 0.5, 3.0, -1.0, float('nan'))) for _ in range(3)]
        car = ActuatorConfig() if i % 4 == 0 else None
        approach, why = run.plan(*goal, caps, s, car)
        out.append({'goal': goal, 'caps': caps, 'settings': vars(s), 'car': car is not None,
                    'why': why if why is None else str(why),
                    'approach': None if approach is None else {
                        'cruise': approach.cruise, 'accel': approach.accel,
                        'decel': approach.decel, 'lead': approach.lead,
                        'stop_band': approach.stop_band, 'creep': approach.creep,
                        'curve': None if approach.curve is None else [
                            approach.curve.a, approach.curve.j, approach.curve.rho]}})
    return out


SCENARIOS = ('to_b', 'estop', 'cancel', 'stuck', 'speeding', 'jump', 'hybrid', 'taken',
             'stale_pose', 'slow_finish', 'turn_round')


def runs(rng):
    """run.Run: whole drives with their inputs each tick, and how each ended."""
    out = []
    radius = min_radius(WHEELBASE, STEER)
    for name in SCENARIOS:
        start = Pose(1.0, -2.0, 0.4)
        angle = start.yaw + (2.6 if name == 'turn_round' else 0.15)
        dist = 12.0
        target = (start.x + dist * math.cos(angle), start.y + dist * math.sin(angle))
        s = run.Settings(still_time=0.5 if name == 'slow_finish' else 1.0)
        lim = rules.Limits(require_gps=name != 'stale_pose')
        caps = {'max_speed': 1.5, 'max_deceleration': 2.0, 'max_jerk': 2.0}
        approach, _ = run.plan(0.0, 0.0, 0.0, caps, s,
                               ActuatorConfig() if name == 'hybrid' else None)
        t = 100.0
        r = run.Run(target, approach, s, lim, radius, start, t)
        pose, speed, ticks = start, 0.0, []
        for i in range(6000):
            t += 0.02
            now = rules.Now(pose_age=0.01, fix_age=0.05, estop=name == 'estop' and i > 150,
                            locked=False, kind=Kind.SIM, lock_age=0.1,
                            taken_over=name == 'taken' and i > 100)
            if name == 'stale_pose' and i > 120:
                now = rules.Now(pose_age=0.8, fix_age=None, estop=False, locked=False,
                                kind=Kind.SIM, lock_age=0.1)
            cancel = name == 'cancel' and i > 200
            measured = speed * (3.0 if name == 'speeding' and i > 100 else 1.0)
            if name == 'stuck':
                measured = 0.0
            seen = pose
            if name == 'jump' and i > 150:
                seen = Pose(pose.x - 6.0 * math.sin(pose.yaw), pose.y + 6.0 * math.cos(pose.yaw),
                            pose.yaw)
            step = r.tick(t, now, seen, measured, cancel)
            fb = step.feedback
            # [t, now, pose, measured, cancel, send, stopped, finished, feedback]
            ticks.append([t, now_dict(now), [seen.x, seen.y, seen.yaw], measured, cancel,
                          step.send, text(step.stopped), step.finished,
                          None if fb is None else [fb.speed, fb.curvature, fb.phase, fb.left,
                                                   fb.cross]])
            if step.send is not None and name != 'stuck':
                pose, speed = kinematic(pose, speed, step.send, 0.02, 0.3)
            if step.finished:
                break
        out.append({'name': name, 'start': [start.x, start.y, start.yaw], 'target': target,
                    'settings': vars(s), 'limits': limits_dict(lim), 'caps': caps,
                    'hybrid': name == 'hybrid', 'radius': radius, 't0': 100.0,
                    'max_path': r.max_path, 'max_time': r.max_time, 'ticks': ticks,
                    'end_pose': [pose.x, pose.y, pose.yaw], 'end_t': t,
                    'fix_status': None, 'outcome': None})
        fix = rng.choice((None, 0, 2))
        end = r.outcome(pose, fix, True, t)
        out[-1].update(fix_status=fix, outcome={
            'status': end.status, 'success': end.success, 'message': end.message,
            'left': end.left, 'tolerance': end.tolerance, 'travelled': end.travelled,
            'duration': end.duration})
    return out


def build():
    rng = random.Random(18)
    return {'note': 'Written by gen_vectors.py from aav_drive_to_point; do not edit.',
            'approach': approaches(rng), 'line': lines(rng), 'drives': drives(rng),
            'rules': rule_cases(rng), 'plans': plans(rng), 'runs': runs(rng)}


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(enc(build()), separators=(',', ':')) + '\n')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
