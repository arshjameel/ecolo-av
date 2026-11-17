#!/usr/bin/env python3
"""
Write behaviour-core's agreement vectors from the Python modules.

The modules: aav_behaviour, aav_description, aav_safety's geofence.

The Rust (crates/behaviour-core, tests/vectors.rs) must give the same answers, within 1e-9
(the same f64 sums; Python's own hypot and its compensated sum() are ported too):

  - the state machine: long random runs of worlds, each tick's state, reason, cap, left
    and warnings, and the state topic's text;
  - path_left, along_until (the fence's depth), lane_ahead, moving_ahead;
  - the road ahead (resample, build), the lane map (lane_at, lane_line, with_edges);
  - the fence's depth.

Infinity and NaN are written as the strings 'inf', '-inf' and 'nan'.

Writes ros/drivers/aav_behaviour_rs/crates/behaviour-core/tests/vectors.json. Run it with
Python 3.12 or newer (sum() is compensated from 3.12) after any change to those modules:

  pixi run python ros/drivers/aav_behaviour_rs/gen_vectors.py
"""

import json
import math
from pathlib import Path
import random
import sys

HERE = Path(__file__).resolve().parent
ROS = HERE.parent.parent
sys.path[:0] = [str(ROS / 'drivers' / 'aav_behaviour'), str(ROS / 'bringup' / 'aav_description'),
                str(ROS / 'safety' / 'aav_safety')]

from aav_behaviour import behaviour as bh  # noqa: E402
from aav_description import lane_map as lm  # noqa: E402
from aav_description import road_ahead as ra  # noqa: E402
from aav_description.stop_law import from_car_model, StopCurve  # noqa: E402
from aav_safety.geofence import Polygon  # noqa: E402

OUT = HERE / 'crates' / 'behaviour-core' / 'tests' / 'vectors.json'
REASONS = ('sign', 'obstacle', 'fence', 'goal')
# The road ahead's stops also carry the hold for something moving (a stop at 0 m).
ROAD_REASONS = REASONS + ('moving',)
LOCKS = ('estop', 'stop_sign', 'stop_zone', 'geofence', 'sensors')


def enc(v):
    """Return v with every non-finite float as a string, for strict JSON."""
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    if isinstance(v, (list, tuple)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


CURVES = {'car': from_car_model(), 'quick': StopCurve(a=3.0, j=5.0, rho=0.2)}
SETTINGS = {
    'default': bh.Settings(),
    'fast': bh.Settings(base=3.0, fall=1.0, accel=0.8),
    'impatient': bh.Settings(ask_after=2.0, release_hold=0.0, sign_lost=1.5),
    'low floor': bh.Settings(base=2.0, floor=0.1, passed=0.3),
}


def worlds(rng, n):
    """Return n worlds: stops that close in and go, locks that come and go, a car slowing."""
    now, speed = 0.0, 1.0
    lefts = dict.fromkeys(REASONS)
    locks = set()
    sign_time = -math.inf
    moving = False
    out = []
    for _ in range(n):
        now += rng.choice([0.05, 0.05, 0.05, 0.049, 0.0, 0.2, 1.3])
        for r in REASONS:
            if lefts[r] is None:
                if rng.random() < 0.03:
                    lefts[r] = rng.uniform(-1.0, 12.0)
            else:
                lefts[r] -= speed * 0.05 + rng.uniform(-0.05, 0.05)
                if rng.random() < 0.02 or lefts[r] < -3.0:
                    lefts[r] = None
                elif rng.random() < 0.02:
                    lefts[r] = rng.uniform(0.0, 20.0)
        for lock in LOCKS:
            p_on = 0.002 if lock == 'estop' else 0.008
            if lock in locks and rng.random() < 0.15:
                locks.discard(lock)
            elif lock not in locks and rng.random() < p_on:
                locks.add(lock)
        if locks:
            speed = max(0.0, speed - 0.3)
        else:
            speed = min(2.0, speed + rng.uniform(-0.1, 0.2))
        if rng.random() < 0.1:
            speed = rng.choice([0.0, 0.03, 0.06, speed])
        stops = [bh.Stop(r, lefts[r]) for r in rng.sample(REASONS, 4) if lefts[r] is not None]
        if lefts['sign'] is not None and rng.random() < 0.7:
            sign_time = now - rng.choice([0.0, 0.05, 0.3])
        odom = rng.random()
        # Something moving near comes and goes (a person walks by), and flickers.
        if moving and rng.random() < 0.05 or not moving and rng.random() < 0.01:
            moving = not moving
        out.append(bh.World(
            now=now, speed=None if odom < 0.02 else speed,
            odom_age=rng.choice([0.05, 0.05, 0.05, 0.6, math.inf]) if odom < 0.1 else 0.05,
            locks=dict.fromkeys(locks, True) | ({'estop': False} if rng.random() < 0.1 else {}),
            near=rng.random() < 0.01, stops=stops, sign_age=now - sign_time,
            goal_active=rng.random() < 0.8, goal_aborted=rng.random() < 0.002,
            moving=moving != (rng.random() < 0.02)))
    return out


def world_dict(w):
    return {'now': w.now, 'speed': w.speed, 'odom_age': w.odom_age,
            'locks': sorted(k for k, on in w.locks.items() if on), 'near': w.near,
            'stops': [[s.reason, s.left] for s in w.stops], 'sign_age': w.sign_age,
            'goal_active': w.goal_active, 'goal_aborted': w.goal_aborted, 'moving': w.moving}


def machine_runs():
    runs = []
    for (sname, settings), seed in zip(SETTINGS.items(), range(4)):
        for cname, curve in CURVES.items():
            ws = worlds(random.Random(seed * 10 + len(cname)), 600)
            b = bh.Behaviour(curve, settings)
            ticks = []
            for w in ws:
                d = b.step(w)
                ticks.append({'world': world_dict(w), 'state': d.state, 'reason': d.reason,
                              'holding': b.holding,
                              'cap': d.cap, 'left': d.left, 'warnings': list(d.warnings),
                              'diagnostic': bh.diagnostic(d)})
            runs.append({'settings': sname, 'curve': cname, 'ticks': ticks})
    return runs


def caps():
    rng = random.Random(3)
    out = []
    for cname, curve in CURVES.items():
        for _ in range(200):
            left = rng.choice([rng.uniform(-2, 30), 0.0, math.inf, rng.uniform(0, 2)])
            base, floor = rng.choice([(1.0, 0.3), (3.0, 0.3), (2.0, 0.1)])
            out.append({'curve': cname, 'left': left, 'base': base, 'floor': floor,
                        'cap': bh.nav2_cap(left, curve, base, floor)})
    return out


def wiggly(rng, n, step=0.5):
    x, y, h = rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi)
    pts = [(x, y)]
    for _ in range(n - 1):
        h += rng.uniform(-0.3, 0.3)
        s = rng.choice([step, step, 0.0, step * 3.7, 0.05])
        x, y = x + s * math.cos(h), y + s * math.sin(h)
        pts.append((x, y))
    return pts


def paths():
    rng = random.Random(11)
    fence = Polygon([(-20.0, -20.0), (25.0, -18.0), (30.0, 22.0), (-15.0, 25.0), (-25.0, 5.0)])
    out = []
    for _ in range(150):
        pts = wiggly(rng, rng.choice([0, 1, 2, 5, 40, 120]))
        x, y = rng.uniform(-8, 8), rng.uniform(-8, 8)
        depth = rng.choice([0.5, 2.5, 8.0])
        out.append({'points': pts, 'x': x, 'y': y, 'left': bh.path_left(pts, x, y),
                    'depth': depth,
                    'until': bh.along_until(pts, x, y,
                                            lambda px, py: fence.depth(px, py) < depth)})
    return {'fence': fence.corners, 'cases': out,
            'depths': [[x, y, fence.depth(x, y)] for x, y in
                       ((rng.uniform(-35, 35), rng.uniform(-35, 35)) for _ in range(300))]}


def scans():
    rng = random.Random(5)
    out = []
    for _ in range(60):
        n = rng.choice([90, 360, 720])
        # float32 angles and ranges, as a LaserScan carries them.
        import struct
        f32 = lambda v: struct.unpack('f', struct.pack('f', v))[0]  # noqa: E731
        amin, inc = f32(-math.pi), f32(2 * math.pi / n)
        ranges = [f32(rng.choice([math.inf, rng.uniform(0.1, 35.0), 0.0, math.nan]))
                  if rng.random() < 0.6 else math.inf for _ in range(n)]
        front, half = rng.choice([(2.0, 0.6), (1.8, 0.9)])
        out.append({'ranges': ranges, 'angle_min': amin, 'angle_increment': inc,
                    'front': front, 'half_width': half,
                    'ahead': bh.lane_ahead(ranges, amin, inc, front, half)})
    return out


def movers():
    rng = random.Random(9)
    out = []
    for _ in range(300):
        car = (rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi))
        things = [bh.Seen(car[0] + rng.uniform(-10, 30), car[1] + rng.uniform(-10, 10),
                          rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5),
                          rng.uniform(0.3, 4.0), rng.uniform(0.3, 2.0))
                  for _ in range(rng.choice([0, 1, 3, 8]))]
        speed = rng.choice([0.0, 0.9, 2.0])
        crossing = rng.choice([bh.Crossing(), None, bh.Crossing(slack=1.0, horizon=3.0)])
        rows = [[t.x, t.y, t.vx, t.vy, t.length, t.width] for t in things]
        out.append({'car': car, 'things': rows,
                    'speed': speed,
                    'crossing': None if crossing is None else [crossing.across, crossing.slack,
                                                               crossing.horizon],
                    'ahead': bh.moving_ahead(things, car, 1.8, 0.6, speed=speed,
                                             crossing=crossing)})
    return out


def nears():
    """Return moving_near's cases: things round the car, a road ahead in base_link or none."""
    rng = random.Random(13)
    out = []
    for _ in range(300):
        car = (rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi))
        things = [bh.Seen(car[0] + rng.uniform(-15, 30), car[1] + rng.uniform(-15, 15),
                          rng.choice([0.0, rng.uniform(-2.0, 2.0), 0.2]),
                          rng.choice([0.0, rng.uniform(-2.0, 2.0), 0.25]),
                          rng.uniform(0.3, 4.5), rng.uniform(0.3, 2.0))
                  for _ in range(rng.choice([0, 1, 3, 8]))]
        road = rng.choice([[], wiggly(rng, 80, 0.5), [(0.0, 0.0)]])
        tight = bh.Nearby(radius=5.0, within=1.5, step=0.1, moving=0.5)
        nearby = rng.choice([bh.Nearby(), tight, bh.Nearby(radius=0.0, within=0.0)])
        front, half = rng.choice([(2.0, 0.6), (1.8, 0.9)])
        out.append({'car': car, 'things': [[t.x, t.y, t.vx, t.vy, t.length, t.width]
                                           for t in things],
                    'road': road, 'front': front, 'half_width': half,
                    'nearby': [nearby.radius, nearby.within, nearby.step, nearby.moving],
                    'near': bh.moving_near(things, car, front, road, half, nearby)})
    return out


def settles():
    """Return MovingFor's runs: things whose speed comes and goes, seen or coasting."""
    rng = random.Random(17)
    out = []
    for _ in range(40):
        settle = rng.choice([1.0, 0.5, 0.0])
        sure = bh.MovingFor(settle=settle)
        now, ticks = 0.0, []
        speeds = {i: rng.choice([0.0, 0.8]) for i in range(1, 6)}
        for _ in range(60):
            now += rng.choice([0.1, 0.1, 0.05, 0.3])
            things = []
            for i in range(1, 6):
                if rng.random() < 0.1:
                    speeds[i] = rng.choice([0.0, 0.2, 0.31, 0.8, 1.5])
                if rng.random() < 0.85:
                    a = rng.uniform(-math.pi, math.pi)
                    things.append((i, rng.random() < 0.25,
                                   bh.Seen(rng.uniform(-5, 20), rng.uniform(-8, 8),
                                           speeds[i] * math.cos(a), speeds[i] * math.sin(a),
                                           0.5, 0.5)))
            got = sure.update(now, things)
            ticks.append({'now': now,
                          'things': [[i, c, [t.x, t.y, t.vx, t.vy, t.length, t.width]]
                                     for i, c, t in things],
                          'sure': [[t.x, t.y] for t in got]})
        out.append({'settle': settle, 'ticks': ticks})
    return out


LANES = """
roads:
- name: main
  width: 8.0
  lanes: 2
  points: [[-30.0, 0.0], [20.0, 0.0], [45.0, 10.0], [70.0, 10.0]]
- name: side
  width: 4.5
  edge_inset: 0.5
  lanes: 1
  points: [[20.0, -40.0], [20.0, 0.0], [21.0, 30.0]]
- kind: service
  width: 12.0
  oneway: true
  points: [[45.0, 10.0], [45.0, 60.0]]
- name: tiny
  width: 0.0
  points: [[0.0, 0.0], [1.0, 1.0]]
- name: wide
  width: 9.0
  points: [[-30.0, 30.0], [10.0, 30.0], [10.0, 30.0], [40.0, 31.0]]
"""


def roads():
    import yaml
    rng = random.Random(21)
    lanes = lm.from_dict(yaml.safe_load(LANES))
    curve = CURVES['car']
    builds, lines, edges, at = [], [], [], []
    routes = [[(-28.0, 0.5), (19.0, -0.5), (20.5, 25.0)],
              [(-28.0, -1.0), (21.0, 0.0), (44.0, 10.5), (46.0, 50.0)],
              [(65.0, 9.0), (40.0, 9.5), (20.0, 1.0), (19.5, -35.0)],
              [(-25.0, 31.0), (39.0, 29.0)]]
    for _ in range(12):
        routes.append(wiggly(rng, 80, 1.0))
    for route in routes:
        for _ in range(4):
            i = rng.randrange(len(route))
            car = (route[i][0] + rng.uniform(-1, 1), route[i][1] + rng.uniform(-1, 1))
            spacing = rng.choice([0.5, 0.5, 0.3, 1.0])
            length = rng.choice([40.0, 10.0, 2.0])
            stops = [bh.Stop(r, rng.uniform(-1.0, 45.0))
                     for r in rng.sample(ROAD_REASONS, rng.randrange(6))]
            base = rng.choice([1.0, 2.5])
            passing = rng.random() < 0.3
            strip = ra.build(route, car, stops, curve, base, state='APPROACH', reason='sign',
                             spacing=spacing, length=length)
            builds.append({'route': route, 'car': car, 'stops': [[s.reason, s.left]
                                                                 for s in stops],
                           'base': base, 'spacing': spacing, 'length': length,
                           'x': strip.x, 'y': strip.y, 'heading': strip.heading,
                           'speed_limit': strip.speed_limit, 'end': strip.end,
                           'on_line': [[s.along, s.reason, s.kind] for s in strip.stops]})
            line = lm.lane_line(lanes, route, car, spacing=spacing, length=length,
                                passing=passing)
            lines.append({'route': route, 'car': car, 'spacing': spacing, 'length': length,
                          'passing': passing, 'line': line})
            strip = ra.build(line, car, stops, curve, base, spacing=spacing, length=length)
            e = lm.with_edges(strip, lanes, passing=passing)
            edges.append({'x': e.x, 'y': e.y, 'heading': e.heading, 'passing': passing,
                          'left_hard': e.left_hard, 'right_hard': e.right_hard,
                          'left_soft': e.left_soft, 'right_soft': e.right_soft})
    for _ in range(600):
        x, y, h = rng.uniform(-35, 75), rng.uniform(-45, 65), rng.uniform(-4, 4)
        lane = lanes.lane_at(x, y, h)
        at.append({'x': x, 'y': y, 'heading': h,
                   'lane': None if lane is None else [lane.x, lane.y, lane.heading,
                                                      lane.left_soft, lane.right_soft,
                                                      lane.left_hard, lane.right_hard,
                                                      lane.road]})
    return {'yaml': LANES, 'builds': builds, 'lines': lines, 'edges': edges, 'lane_at': at}


def main():
    vectors = {'curves': {k: [c.a, c.j, c.rho] for k, c in CURVES.items()},
               'settings': {k: [s.base, s.floor, s.accel, s.fall, s.still, s.ask_after,
                                s.odom_stale, s.sign_lost, s.passed, s.release_hold]
                            for k, s in SETTINGS.items()},
               'runs': machine_runs(), 'caps': caps(), 'paths': paths(), 'scans': scans(),
               'movers': movers(), 'nears': nears(), 'settles': settles(), 'roads': roads()}
    OUT.write_text(json.dumps(enc(vectors), separators=(',', ':'), allow_nan=False) + '\n')
    print(f'wrote {OUT.relative_to(ROS.parent)} ({OUT.stat().st_size // 1024} kB)')


if __name__ == '__main__':
    main()
