#!/usr/bin/env python3
"""
Write ego-core's agreement vectors from the Python modules (aav_ego, aav_description).

The Rust (crates/ego-core, tests/vectors.rs) must give the same answers:

  - ego's step, the lead, the pedal, the body, the stopping law, the road ahead, the stop
    point and the squiggly line: within 1e-9 (the same f64 sums; libm's hypot and Python's
    own differ in the last bit, and Python 3.12's sum() is compensated);
  - the trace and the twin: within 1e-3, as the Rust runs spine-core's own speed loop (f32),
    the one the Python SpeedLoop is a port of (as car-model's vectors.json).

Writes ros/ego/aav_ego_rs/crates/ego-core/tests/vectors.json. Run it after any change to
those Python modules or to ego-core:

  python3 ros/ego/aav_ego_rs/gen_vectors.py
"""

from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import random
import sys

HERE = Path(__file__).resolve().parent
ROS = HERE.parent.parent
sys.path[:0] = [str(ROS / 'ego' / 'aav_ego'), str(ROS / 'bringup' / 'aav_description')]

from aav_description import car_model as cm  # noqa: E402
from aav_description import road_ahead as ra  # noqa: E402
from aav_description import stop_law as sl  # noqa: E402
from aav_ego import dynamics, look_ahead as la, road_limit, stop_ahead, trace_ahead  # noqa: E402
from aav_ego.ackermann import EgoModel, Limits  # noqa: E402
from aav_ego.dynamics import Body  # noqa: E402
from aav_ego.lag_lead import LagLead, LeadLimits, LeadTimes  # noqa: E402
from aav_ego.pedal import may_shift, Pedal, slow_rate  # noqa: E402

OUT = HERE / 'crates' / 'ego-core' / 'tests' / 'vectors.json'
LAGGY = cm.ActuatorConfig(throttle_dead_time=0.06, brake_dead_time=0.04,
                          steer_dead_time=0.1, steer_offset=0.01, throttle_deadband=0.05)
CFGS = {'default': cm.ActuatorConfig(), 'laggy': LAGGY,
        'loaded': cm.ActuatorConfig(payload=120.0)}


def state_dict(s):
    d = asdict(s)
    for k in ('throttle_delay', 'brake_delay', 'steer_delay'):
        d[k] = list(d[k])
    return d


def commands(rng, n, every, stale=0.15, reverse=True, top=1.5):
    """Return n (cmd or None, dt) steps: a new random request every `every` steps."""
    out, cmd = [], None
    for i in range(n):
        if i % every == 0:
            r = rng.random()
            if r < stale:
                cmd = None
            else:
                lo = -0.8 if reverse else 0.0
                cmd = [rng.uniform(lo, top), rng.uniform(-0.8, 0.8)]
        dt = rng.choice([0.02, 0.02, 0.019, 0.021, 0.0])
        out.append([cmd, dt])
    return out


def ego_runs():
    rng = random.Random(7)
    times = LeadTimes.from_config(LAGGY)
    runs = [
        {'name': 'plain', 'limits': asdict(Limits()), 'body': None, 'pedal': None,
         'lead': None, 'steps': commands(rng, 300, 30)},
        {'name': 'body and jerk', 'limits': asdict(Limits(max_jerk=2.0, max_speed=2.5)),
         'body': asdict(Body()), 'pedal': None, 'lead': None,
         'steps': commands(rng, 400, 40, top=2.5)},
        {'name': 'loaded', 'limits': asdict(Limits(max_jerk=2.0)),
         'body': asdict(Body(payload=250.0, max_lateral_accel=3.0)), 'pedal': None,
         'lead': None, 'steps': commands(rng, 300, 25)},
        {'name': 'pedal', 'limits': asdict(Limits(max_jerk=2.0)), 'body': asdict(Body()),
         'pedal': asdict(Pedal(let_off=1.0, full=1.0)), 'lead': None,
         'steps': commands(rng, 400, 20)},
        {'name': 'pedal soft', 'limits': asdict(Limits()), 'body': None,
         'pedal': asdict(Pedal(let_off=0.4, full=0.8, shift_hold=0.3)), 'lead': None,
         'steps': commands(rng, 300, 15)},
        {'name': 'pedal wheels', 'limits': asdict(Limits()), 'body': None,
         'pedal': asdict(Pedal(shift_hold=0.2, stop_speed=0.1)), 'lead': None,
         'steps': commands(rng, 400, 25),
         'wheels': [rng.choice([None, None, 0.0, 0.05, -0.08, 0.3]) for _ in range(400)]},
        {'name': 'lead', 'limits': asdict(Limits(max_jerk=2.0)), 'body': asdict(Body()),
         'pedal': None, 'lead': {'times': asdict(times), 'limits': asdict(LeadLimits())},
         'steps': commands(rng, 400, 35)},
        {'name': 'lead unfiltered', 'limits': asdict(Limits()), 'body': None, 'pedal': None,
         'lead': {'times': asdict(times), 'limits': asdict(LeadLimits(filter_time=0.0))},
         'steps': commands(rng, 200, 20)},
    ]
    for run in runs:
        model = EgoModel(Limits(**run['limits']),
                         Body(**run['body']) if run['body'] else None)
        if run['pedal']:
            model.pedal = Pedal(**run['pedal'])
        lead = (LagLead(LeadTimes(**run['lead']['times']), LeadLimits(**run['lead']['limits']))
                if run['lead'] else None)
        outs = []
        wheels = run.get('wheels') or [None] * len(run['steps'])
        for (cmd, dt), wheel in zip(run['steps'], wheels):
            c = tuple(cmd) if cmd is not None else None
            on = lead is not None and c is not None
            if lead is not None and not on:
                lead.reset()
            model.pedal = Pedal(**run['pedal']) if run['pedal'] and c is not None else None
            model.wheel_speed = wheel
            outs.append(list(model.step(c, dt, lead.step if on else None)))
        run['outs'] = outs
    return runs


def may_shifts():
    rng = random.Random(5)
    out = []
    for _ in range(100):
        p = Pedal(shift_hold=rng.choice([0.0, 0.5, 1.0]), stop_speed=rng.choice([0.05, 0.2]))
        held = rng.choice([0.0, 0.5, 1.0, rng.uniform(0.0, 2.0)])
        wheel = rng.choice([None, 0.0, rng.uniform(-0.3, 0.3)])
        out.append({'pedal': asdict(p), 'held': held, 'wheel': wheel,
                    'may': may_shift(p, held, wheel)})
    return out


def slow_rates():
    rng = random.Random(3)
    out = []
    for _ in range(200):
        p = Pedal(let_off=rng.choice([1.0, 0.5, 3.0]), full=rng.choice([1.0, 0.5, 0.0]))
        cur = rng.choice([0.0, rng.uniform(-2, 2)])
        tgt = rng.choice([0.0, rng.uniform(-2, 2)])
        dec = rng.choice([2.0, 0.8])
        out.append({'pedal': asdict(p), 'current': cur, 'target': tgt, 'max_deceleration': dec,
                    'rate': slow_rate(p, cur, tgt, dec)})
    return out


def bodies():
    out = []
    for body in (Body(), Body(payload=300.0), Body(tyre_grip=0.3, cg_height=0.9),
                 Body(max_lateral_accel=10.0, track=1.2)):
        ks = [0.0, 1e-10, 0.1, 0.5, -0.8, 2.0]
        out.append({
            'body': asdict(body), 'speed_up': dynamics.speed_up_limit(body),
            'braking': dynamics.braking_limit(body), 'lateral': dynamics.lateral_limit(body),
            'curve': [[k, dynamics.curve_speed(body, k)] for k in ks if k >= 1e-9 or k < 0],
            'stopping': [[v, d, dynamics.stopping_distance(body, v, d)]
                         for v in (0.0, 1.0, -2.5) for d in (1.0, 2.0)],
            'energy': [[v, dynamics.energy(body, v)] for v in (0.0, 1.0, 3.0)],
            'describe': dynamics.describe(body, 1.561, 0.5, 1.0, 1.0, 2.0, 2.0)})
    return out


def stop_curves():
    out = []
    loops = {'default': cm.SpeedLoopConfig(), 'smooth': cm.speed_loop_profile('smooth'),
             'lqi': cm.speed_loop_profile('lqi'), 'none': None}
    for cfg_name, cfg in CFGS.items():
        for loop_name, loop in loops.items():
            for dec, jerk in ((2.0, 2.0), (1.0, 1e6), (5.0, 0.5)):
                c = sl.from_car_model(cfg, dec, jerk, sl.INPUT_DELAY, loop)
                out.append({
                    'cfg': cfg_name, 'speed_loop': loop_name, 'max_deceleration': dec,
                    'max_jerk': jerk,
                    'a': c.a, 'j': c.j, 'rho': c.rho, 'tau': c.tau,
                    'distance': [[v, c.distance(v)] for v in (0.0, 0.5, -1.0, 2.5)],
                    'cap': [[d, c.cap(d)] for d in (-1.0, 0.0, 0.3, 2.0, 15.0)],
                    'describe': c.describe()})
    for brake in (3.0, 1.2):
        c = sl.from_numbers(brake, 2.0, 2.0)
        out.append({'numbers': brake, 'a': c.a, 'j': c.j, 'rho': c.rho})
    return out


def strip_dict(strip):
    return {'spacing': strip.spacing, 'x': list(strip.x), 'y': list(strip.y),
            'speed_limit': list(strip.speed_limit), 'end': strip.end,
            'stops': [[s.along, s.reason] for s in strip.stops]}


class Stop:
    def __init__(self, reason, left):
        self.reason, self.left = reason, left


def roads():
    curve = sl.from_car_model(cm.ActuatorConfig(), 2.0, 2.0)
    rng = random.Random(11)
    plans = [
        ([(0.0, 0.0), (30.0, 0.0)], (0.2, 0.3), [Stop('sign', 12.0)]),
        ([(0.0, 0.0), (5.0, 0.0), (5.0, 8.0), (-3.0, 12.0)], (1.0, -0.4),
         [Stop('obstacle', 6.5), Stop('goal', 14.0)]),
        ([(2.0, 1.0), (2.0, 1.0), (10.0, 4.0)], (2.0, 1.0), []),
        ([(0.0, 0.0), (60.0, 10.0)], (0.0, 0.0), [Stop('fence', -2.0)]),
    ]
    out = []
    for plan, car, stops in plans:
        strip = ra.build(plan, car, stops, curve, 1.0)
        queries = []
        for _ in range(25):
            x = car[0] + rng.uniform(-2.0, 20.0)
            y = car[1] + rng.uniform(-3.0, 3.0)
            age = rng.choice([0.0, 0.5, 1.5, math.inf])
            r = la.road(strip, x, y)
            queries.append({
                'x': x, 'y': y, 'along_of': list(ra.along_of(strip, x, y)),
                'room': ra.room_at(strip, x, y), 'limit': ra.limit_at(strip, x, y),
                'limit_curve': ra.limit_at(strip, x, y, curve),
                'limit_base': ra.limit_at(strip, x, y, curve, 0.6),
                'age': age if math.isfinite(age) else 1e300,
                'allowed': road_limit.allowed(strip, (x, y), age, 1.0, curve, 1.0),
                'road': None if r is None else [r.limit, r.room, r.why]})
        out.append({'strip': strip_dict(strip), 'queries': queries})
    caps = [{'cmd': list(c), 'limit': lim, 'out': list(road_limit.cap(c, lim))}
            for c in ((1.0, 0.4), (-1.0, 0.4), (0.0, 0.3), (2.0, -0.6), (0.5, 0.1))
            for lim in (0.0, 0.3, 0.5, 1.5, math.inf)
            if math.isfinite(lim)]
    return {'curve': {'a': curve.a, 'j': curve.j, 'rho': curve.rho}, 'strips': out, 'caps': caps}


def car_states(rng, cfg, n=6):
    """Return a few CarStates of cfg, driven by pedals, each with its delay lines full."""
    states, s = [], cm.CarState()
    for i in range(n * 40):
        throttle = rng.uniform(0.0, 1.0) if i % 80 < 50 else 0.0
        brake = 0.0 if throttle else rng.uniform(0.0, 0.5)
        s = cm.step(cfg, s, cm.Command(steer=rng.uniform(-0.6, 0.6), throttle=throttle,
                                       brake=brake, reverse=(i // 120) % 2 == 1), 0.02)
        if i % 40 == 39:
            states.append(s)
    return states


def stop_points():
    rng = random.Random(5)
    out = []
    turning = cm.CarState(v=2.0, steer=0.3, steer_setpoint=0.3)
    for name, cfg in CFGS.items():
        for s in car_states(rng, cfg) + [cm.CarState(), turning]:
            for dt in (0.02, 0.05):
                p = stop_ahead.stop_point(cfg, s, dt)
                out.append({'cfg': name, 'state': state_dict(s), 'dt': dt,
                            'out': [p.x, p.y, p.yaw, p.distance, p.time, p.found]})
    no_brake = replace(cm.ActuatorConfig(), max_brake=0.0, drag=0.0)
    p = stop_ahead.stop_point(no_brake, cm.CarState(v=1.0), 0.1, max_time=0.5)
    out.append({'cfg': 'default', 'state': state_dict(cm.CarState(v=1.0)), 'dt': 0.1,
                'max_time': 0.5, 'cfg_override': {'max_brake': 0.0, 'drag': 0.0},
                'out': [p.x, p.y, p.yaw, p.distance, p.time, p.found]})
    return out


def traces():
    rng = random.Random(9)
    out = []
    times = LeadTimes.from_config(LAGGY)
    for name, cfg in CFGS.items():
        for profile in ('default', 'smooth', 'lqi'):
            for case in range(3):
                twin = trace_ahead.Twin(cfg, cm.speed_loop_profile(profile))
                lim = Limits(max_jerk=2.0, max_speed=2.0)
                model = EgoModel(lim, Body() if case != 1 else None)
                lead = LagLead(times) if case == 2 else None
                ops = []
                cmd = (rng.uniform(0.0, 2.0), rng.uniform(-0.5, 0.5))
                for i in range(60):
                    if i % 20 == 0:
                        stale = rng.random() < 0.2
                        cmd = None if stale else (rng.uniform(-0.5, 2.0), rng.uniform(-0.5, 0.5))
                    if i % 5 == 0:
                        v = twin.state.v + rng.uniform(-0.05, 0.05)
                        twin.measured(v)
                        ops.append(['measured', v])
                    on = lead is not None and cmd is not None
                    if lead is not None and not on:
                        lead.reset()
                    sp, st, _ = model.step(cmd, 0.02, lead.step if on else None)
                    twin.sent(sp, st, 0.02)
                    ops.append(['sent', sp, st, 0.02])
                held = (rng.uniform(-0.5, 2.0), rng.uniform(-0.5, 0.5))
                on = lead is not None
                cmds = trace_ahead.ego_commands(model, held, lead if on else None, 100, 0.02)
                poses = trace_ahead.trace(twin, model, held, lead if on else None, 2.0, 0.02, 5)
                out.append({
                    'cfg': name, 'speed_loop': profile, 'ops': ops,
                    'model': {'limits': asdict(lim),
                              'body': asdict(model.body) if model.body else None,
                              'speed': model.speed, 'steering': model.steering,
                              'accel': model.accel},
                    'lead': None if lead is None else {
                        'times': asdict(lead.times), 'limits': asdict(lead.limits),
                        'filtered': None if lead.speed is None else [lead.speed, lead.angle]},
                    'cmd': list(held), 'twin': state_dict(twin.state),
                    'commands': [[c.speed, c.steer] for c in cmds],
                    'poses': [[p.t, p.x, p.y, p.yaw, p.v, p.steer] for p in poses]})
    return out


def polyline(rng, n, start=(-3.0, 0.0)):
    pts, (x, y), yaw = [], start, rng.uniform(-0.3, 0.3)
    for _ in range(n):
        pts.append((x, y))
        yaw += rng.uniform(-0.4, 0.4)
        d = rng.choice([0.0, 0.5, 1.0, 2.0])
        x, y = x + d * math.cos(yaw), y + d * math.sin(yaw)
    return pts


def looks():
    rng = random.Random(13)
    cases = []
    for i in range(40):
        plan = polyline(rng, rng.choice([0, 1, 2, 5, 30]))
        trace = [(0.1 * k, 0.01 * k * k * rng.uniform(-1, 1)) for k in range(21)]
        stop = (rng.uniform(-1.0, 12.0), rng.uniform(-1.0, 1.0))
        faint = la.crop_ahead(plan, 10.0)
        g = la.gap(trace, faint)
        cases.append({
            'plan': plan, 'trace': trace, 'stop': list(stop), 'length': la.length(plan),
            'project': None if not plan else list(la.project(plan, stop)),
            'faint': faint, 'cut': la.cut(plan, 1.0, 6.5), 'gap': g,
            'grade': la.grade(g, la.Limits()), 'to_goal': la.to_goal(plan, stop),
            'nearest': (min(range(len(plan)), key=lambda i: math.hypot(*plan[i]))
                        if plan else None)})
    grids = []
    for i in range(12):
        w, h = rng.choice([(60, 40), (50, 50), (30, 80)])
        data = [rng.choice([0, 0, 0, 0, 50, -1, 100, 99]) if rng.random() < 0.05 else 0
                for _ in range(w * h)]
        origin = (rng.uniform(-6, -1), rng.uniform(-5, -1), rng.uniform(-0.5, 0.5))
        grid = la.Grid(w, h, rng.choice([0.05, 0.1, 0.2]), origin, data)
        path = []
        x = y = yaw = 0.0
        for _ in range(rng.choice([1, 2, 21])):
            path.append((x, y, yaw))
            jump = 3.0 if rng.random() < 0.1 else 0.0
            yaw = math.remainder(yaw + rng.uniform(-0.3, 0.3) + jump, math.tau)
            x, y = x + 0.1 * math.cos(yaw), y + 0.1 * math.sin(yaw)
        ext = la.extend(path, 15.0)
        probes = [(rng.uniform(-8, 12), rng.uniform(-8, 12)) for _ in range(30)]
        grids.append({
            'width': w, 'height': h, 'resolution': grid.resolution, 'origin': list(origin),
            'data': data, 'path': path, 'extend': ext,
            'hit': la.first_hit(grid, ext, 1.9, 0.6, 100),
            'hit_soft': la.first_hit(grid, ext, 1.9, 0.6, 50, 0.25),
            'costs': [[p[0], p[1], grid.cost(p)] for p in probes]})
    describes = []
    for gap_m in (None, 0.0, 0.12, 0.449, 2.0):
        for stop_m in (None, 0.0, 1.45):
            for goal_m in (None, 0.8, -1.25):
                for room_m in (None, 3.0, -0.05):
                    level = la.grade(gap_m)
                    text = la.describe(gap_m, level, stop_m, goal_m, room_m)
                    describes.append({'gap': gap_m, 'level': level, 'stop': stop_m,
                                      'goal': goal_m, 'room': room_m, 'text': text})
    return {'cases': cases, 'grids': grids, 'describes': describes}


def main():
    vectors = {'ego_runs': ego_runs(), 'slow_rates': slow_rates(),
               'may_shifts': may_shifts(), 'bodies': bodies(),
               'stop_curves': stop_curves(), 'roads': roads(), 'stop_points': stop_points(),
               'traces': traces(), 'look_ahead': looks(),
               'cfgs': {k: asdict(v) for k, v in CFGS.items()}}
    OUT.write_text(json.dumps(vectors, separators=(',', ':'), allow_nan=False) + '\n')
    print(f'wrote {OUT.relative_to(ROS.parent)} ({OUT.stat().st_size // 1024} kB)')


if __name__ == '__main__':
    main()
