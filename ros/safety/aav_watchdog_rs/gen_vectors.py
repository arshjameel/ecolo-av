#!/usr/bin/env python3
"""
Write watchdog-core's agreement vectors from the Python modules.

The modules: aav_safety's watchdog.py and wheel_check.py, driven the way
sensor_watchdog.py drives them (Harness, below: the node without ROS). The Rust
(crates/watchdog-core, tests/vectors.rs) must give the same answers, every tick: each
input's state and words, whether it stops the drivers, the lock, the state line, what
is logged, and /diagnostics' statuses once a second.

Each run is 40 s of a simulated car with every kind of input, and faults on purpose: a
frozen wheel speed, NaN, silence, a GPS with no fix and a GPS jump, a frozen camera, a
lidar that goes silent and one that freezes (the same cloud again while moving), a
HUD that drops out, steering that sticks, a throttle that runs away, a node that dies,
the watchdog itself stalled, and a person clearing the latches.

Writes ros/safety/aav_watchdog_rs/crates/watchdog-core/tests/vectors.json. Run it with
Python 3.12 or newer after any change to those modules:

  pixi run python ros/safety/aav_watchdog_rs/gen_vectors.py
"""

from dataclasses import asdict
import json
import math
from pathlib import Path
import random
import sys
import zlib

HERE = Path(__file__).resolve().parent
ROS = HERE.parent.parent
sys.path[:0] = [str(ROS / 'safety' / 'aav_safety')]

from aav_safety.watchdog import NEVER, OK, Rule, Watchdog  # noqa: E402
from aav_safety.wheel_check import CrossCheck  # noqa: E402

OUT = HERE / 'crates' / 'watchdog-core' / 'tests' / 'vectors.json'
IMU_TOPIC = '/sensors/imu/data'
RATE = 10.0
LAT0, LON0 = 45.3860, -75.6960
M_PER_DEG = math.radians(1.0) * 6371000.0


def enc(v):
    """Return v with every non-finite float as a string, for strict JSON."""
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    if isinstance(v, (list, tuple)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


class Harness:
    """sensor_watchdog.py's on_message, tick and publish_diagnostics, without ROS."""

    def __init__(self, rules):
        self.dog = Watchdog(rules, stall=max(0.5, 5.0 / RATE))
        self.speed = None
        self.speed_at = None
        self.said = {}

    def odom(self, now, name, v):
        watch = self.dog.watches[name]
        if watch.rule.topic == '/vehicle/odom':
            self.speed, self.speed_at = v[0], now
        watch.arrived(now, v, digest=v)

    def imu(self, now, name, v):
        self.dog.watches[name].arrived(now, v, digest=v)
        if self.dog.watches[name].rule.topic == IMU_TOPIC:
            self.dog.imu(now, v[3], v[2])

    def gps(self, now, name, stamp, lat, lon, has_fix):
        self.dog.watches[name].arrived(now, fix=(lat, lon), has_fix=has_fix, stamp=stamp)
        if has_fix:
            self.dog.gps(stamp, (lat, lon))

    def image(self, now, name, digest):
        self.dog.watches[name].arrived(now, digest=digest, speed=self.speed)

    def detections(self, now, name, frame, values):
        if self.dog.watches[name].rule.counts(frame):
            self.dog.watches[name].arrived(now, values)

    def actuator(self, now, name, command, measured):
        self.dog.watches[name].arrived(now, (command, measured))

    def speed_cmd(self, now, name, command):
        measured = float('nan') if self.speed is None else self.speed
        age = math.inf if self.speed_at is None else now - self.speed_at
        self.dog.watches[name].arrived(now, (command, measured, age))

    def heartbeat(self, now, name, count):
        self.dog.watches[name].arrived(now, digest=count)

    def cloud(self, now, name, digest):
        self.dog.watches[name].arrived(now, digest=digest, speed=self.speed)

    def status(self, now, name, status_name, level, words):
        if self.dog.watches[name].rule.hears(status_name):
            self.dog.watches[name].arrived(now, (level,), words=words)

    def tick(self, now, steady, running):
        for rule in (w.rule for w in self.dog.watches.values()):
            if rule.kind == 'node' and rule.topic in running:
                self.dog.watches[rule.name].arrived(now)
        self.dog.update(now, steady)
        states = self.dog.states(now)
        locked = any(stops for *_, stops in states)
        summary = self.dog.summary(now)
        said = []
        for name, state, detail, stops in states:
            before = self.said.get(name)
            if before == state:
                continue
            if state == OK:
                if before not in (None, NEVER):
                    said.append(['back', f'{name}: OK again'])
            elif before is not None or state != NEVER:
                if stops:
                    said.append(['stops', f'{name} {state}: {detail}. Nav2 and the AI '
                                 'slot stop (twist_mux lock)'])
                else:
                    said.append(['warns', f'{name} {state}: {detail}'])
            self.said[name] = state
        return states, locked, summary, said

    def diagnostics(self, now, states):
        out = []
        for name, state, detail, stops in states:
            watch = self.dog.watches[name]
            rule = watch.rule
            level = 0 if state == OK else 2 if stops else 1
            rate, gap, worst_gap = watch.traffic(now)
            values = [['rate_hz', f'{rate:.1f}'], ['longest_gap_s', f'{gap:.2f}'],
                      ['worst_gap_s', f'{worst_gap:.2f}'], ['max_age', f'{rule.max_age:g}'],
                      ['stops', str(rule.stop).lower()]]
            if watch.cross is not None:
                gaps = watch.cross.gaps
                for i, key in enumerate(('speed_gap', 'turn_gap', 'gps_gap')):
                    values += [[key, '' if gaps is None else f'{gaps[i]:.3f}'],
                               [f'worst_{key}', f'{watch.cross.worst[i]:.3f}']]
            out.append([f'sensor_watchdog: {name}', level,
                        f'{state}: {detail}, {rate:.1f} Hz, longest gap {gap:.2f} s',
                        rule.topic, values])
        return out


def rules(variant):
    cross = CrossCheck() if variant != 1 else CrossCheck(window=1.0, persist=0.5,
                                                         gps_window=6.0)
    return [
        Rule('odom', '/vehicle/odom', 'odom', 0.5, cross_check=cross),
        Rule('imu', IMU_TOPIC, 'imu', 0.5, frozen_after=1.0),
        Rule('gps', '/sensors/gps/fix', 'gps', 2.0, max_jump_speed=15.0, jump_margin=5.0),
        Rule('local', '/localization/odom/local', 'odom', 0.5),
        Rule('camera', '/sensors/camera/front_left/image_raw', 'image', 1.0,
             frozen_after=1.0),
        Rule('front_left_stream', '/perception/detections', 'detections', 1.0,
             frame_id='camera_left_link'),
        Rule('hud', '/hud/heartbeat', 'heartbeat', 1.0, frozen_after=1.0, latch=True,
             latch_grace=5.0 if variant != 2 else 2.0),
        Rule('steering', '/vehicle/steering', 'actuator', 1.0, follow_tolerance=0.15,
             follow_persist=1.0),
        Rule('speed', '/ego/actuator_cmd', 'speed', 1.0, over_margin=0.2, over_warn=1.0,
             over_persist=2.0, over_slowing=0.3),
        Rule('lidar', '/sensors/lidar/points', 'cloud', 1.0, frozen_after=1.0),
        Rule('radar', '/sensors/radar/points', 'cloud', 1.0, stop=False),
        Rule('node_planner', '/planner', 'node', 3.0, stop=False),
        Rule('status_costmap', '/drivers/nav/costmap/status', 'status', 3.0, stop=False,
             status_name='costmap local', min_level=1),
        Rule('status_planner', '/drivers/nav/planner/status', 'status', 3.0,
             stop=variant == 1, quiet_ok=True),
    ]


def rule_dict(r):
    d = asdict(r)
    d['cross_check'] = None if r.cross_check is None else asdict(r.cross_check)
    return d


def r4(x):
    return round(x, 4)


def run(seed, variant):
    """Return one run: the rules, the events and each tick's answers."""
    rng = random.Random(seed)
    rs = rules(variant)
    h = Harness(rs)
    events, ticks = [], []

    # Faults: (start, end) windows, each picked at random in the run.
    def window(lo, hi, length):
        start = rng.uniform(lo, hi)
        return (start, start + length)
    faults = {
        # Frozen while speeding up (DISAGREE), or at a steady speed (nothing to see).
        'wheels_frozen': window(1.5, 3, 3.0) if variant == 0 else window(8, 12, 4.0),
        # Long enough for the wheels' last speed to go stale while ego asks 0 (16 Nov).
        'odom_silent': window(20, 24, 3.0),
        'odom_nan': window(28, 30, 0.3), 'imu_frozen': window(14, 18, 1.8),
        'imu_silent': window(30, 34, 0.8), 'gps_nofix': window(5, 8, 3.0),
        'gps_jump': window(18, 22, 0.5), 'cam_frozen': window(10, 14, 2.5),
        'hud_drop1': window(1.5, 2.5, 1.4), 'hud_drop2': window(15, 25, 1.5),
        'hud_frozen': window(30, 33, 1.6), 'det_nan': window(12, 16, 0.4),
        'det_silent': window(24, 27, 1.5), 'stuck': window(6, 10, 2.5),
        'overspeed': window(26, 30, 3.0), 'radar_silent': window(3, 8, 2.0),
        'node_gone': window(10, 20, 4.0), 'stall': window(16, 20, 0.0),
        'costmap_waits': window(5, 15, 2.0), 'costmap_hangs': window(22, 30, 4.0),
        'planning': window(3, 12, 8.0), 'no_path': window(14, 20, 1.5),
        'lidar_silent': window(3, 8, 1.5), 'lidar_frozen': window(10, 14, 2.0),
        'clear': (rng.uniform(34, 37), 0.0),
    }

    def on(name, t):
        a, b = faults[name]
        return a <= t < b

    v, h_yaw, x, y = 0.0, 0.0, 0.0, 0.0
    held, imu_last = None, None
    count, cam_digest, lidar_digest, steer = 0, 0, 0, 0.0
    steady, stalled, cleared = 0.0, False, False
    dt = 0.02
    for k in range(int(40.0 / dt)):
        t = r4(k * dt)
        # The car: speeds up, cruises, slows, turns gently.
        accel = 0.5 if t < 4 else (0.0 if t < 25 else -0.3 if v > 0.2 else 0.0)
        cmd = r4(min(2.0, 0.5 * t) if t < 25 else max(0.0, 2.0 - 0.4 * (t - 25)))
        if on('overspeed', t):
            accel = 0.6                 # a throttle that runs away
        if on('odom_silent', t):
            cmd = 0.0                   # the lock stopped ego; the wheels' last speed stays
        v = max(0.0, v + accel * dt)
        yaw_rate = 0.1 * math.sin(0.3 * t)
        h_yaw += yaw_rate * dt
        x += v * math.cos(h_yaw) * dt
        y += v * math.sin(h_yaw) * dt
        # Wheels, 50 Hz.
        if not on('odom_silent', t):
            if on('wheels_frozen', t):
                held = held or (r4(v), r4(yaw_rate))
                wv, ww = held
            else:
                held = None
                wv, ww = r4(v), r4(yaw_rate)
            vals = [wv, 0.0, ww, r4(x), r4(y)]
            if on('odom_nan', t):
                vals[1] = float('nan')
            events.append(['odom', t, 'odom', vals])
        # IMU, 50 Hz.
        if not on('imu_silent', t):
            if on('imu_frozen', t):
                vals = imu_last
            else:
                vals = [r4(rng.gauss(0, 0.002)), r4(rng.gauss(0, 0.002)),
                        r4(yaw_rate + rng.gauss(0, 0.002)),
                        r4(accel + rng.gauss(0, 0.02)), r4(rng.gauss(0, 0.02)), 9.81]
                imu_last = vals
            events.append(['imu', t, 'imu', vals])
        # The local EKF, 25 Hz.
        if k % 2 == 0:
            events.append(['odom', t, 'local', [r4(v), 0.0, r4(yaw_rate), r4(x), r4(y)]])
        # GPS, 5 Hz; now and then two at once, or an old one late.
        if k % 10 == 0:
            north = y + rng.gauss(0, 0.3) + (200.0 if on('gps_jump', t) else 0.0)
            east = x + rng.gauss(0, 0.3)
            lat = LAT0 + north / M_PER_DEG
            lon = LON0 + east / (M_PER_DEG * math.cos(math.radians(LAT0)))
            stamp = r4(t - rng.choice([0.0, 0.0, 0.05, 0.3]))
            has = not on('gps_nofix', t)
            if not has and rng.random() < 0.5:
                lat = float('nan')
            events.append(['gps', t, 'gps', stamp, lat, lon, has])
        # Camera, 10 Hz: a new picture each time, the same one while frozen.
        if k % 5 == 0:
            if not on('cam_frozen', t):
                cam_digest = zlib.crc32(rng.randbytes(64))
            events.append(['image', t, 'camera', cam_digest])
        # Detections, both cameras on one topic.
        if k % 3 == 0 and not on('det_silent', t):
            frame = rng.choice(['camera_left_link', 'camera_right_link'])
            boxes = [r4(rng.uniform(0, 640)) for _ in range(rng.choice([0, 5, 10]))]
            if on('det_nan', t) and boxes:
                boxes[0] = float('inf')
            events.append(['detections', t, 'front_left_stream', frame, boxes])
        # The HUD's heartbeat, 10 Hz.
        if k % 5 == 0 and t > 0.5 and not on('hud_drop1', t) and not on('hud_drop2', t):
            if not on('hud_frozen', t):
                count += 1
            events.append(['heartbeat', t, 'hud', count])
        # Steering, 50 Hz: follows, but sticks for a while.
        steer_cmd = r4(0.3 * math.sin(0.5 * t))
        steer = steer if on('stuck', t) else steer_cmd
        measured = float('nan') if rng.random() < 0.01 else r4(steer)
        events.append(['actuator', t, 'steering', steer_cmd, measured])
        # Ego's command, 50 Hz.
        events.append(['speed', t, 'speed', cmd])
        # The lidar, 10 Hz: a new cloud each time, the same one while frozen.
        if k % 5 == 2 and not on('lidar_silent', t):
            if not on('lidar_frozen', t):
                lidar_digest = zlib.crc32(rng.randbytes(64))
            events.append(['cloud', t, 'lidar', lidar_digest])
        # Radar, 10 Hz.
        if k % 5 == 0 and not on('radar_silent', t):
            events.append(['cloud', t, 'radar', zlib.crc32(rng.randbytes(16))])
        # Our nodes' status (TODO 171): both costmaps on one topic, 2 Hz; the local one
        # waits for odometry a while, then hangs. The planner speaks only while it plans
        # (1 Hz), and one plan in a while finds no path.
        if k % 25 == 0 and not on('costmap_hangs', t):
            waits = on('costmap_waits', t)
            events.append(['status', t, 'status_costmap', 'costmap local', 1 if waits else 0,
                           'local costmap: waiting for odometry' if waits
                           else 'local costmap: 0.40 ms an update'])
            events.append(['status', t, 'status_costmap', 'costmap global', 1,
                           'global costmap: waiting for the map'])
        if k % 50 == 7 and (on('planning', t) or on('no_path', t)):
            failed = on('no_path', t)
            events.append(['status', t, 'status_planner', 'planner', 2 if failed else 0,
                           'no path: the goal is blocked' if failed else 'a path, 12 poses'])
        # A tick, 10 Hz.
        if k % 5 == 4:
            steady = r4(steady + 0.1)
            if not stalled and faults['stall'][0] <= t:
                steady = r4(steady + 2.0)       # the watchdog itself stopped 2 s
                stalled = True
            if not cleared and faults['clear'][0] <= t:
                events.append(['clear', t])
                cleared = True
            running = [] if on('node_gone', t) else ['/planner']
            events.append(['tick', t, steady, running])
    # Play the events into the harness.
    for e in events:
        what, t = e[0], e[1]
        if what == 'odom':
            h.odom(t, e[2], tuple(e[3]))
        elif what == 'imu':
            h.imu(t, e[2], tuple(e[3]))
        elif what == 'gps':
            h.gps(t, e[2], e[3], e[4], e[5], e[6])
        elif what == 'image':
            h.image(t, e[2], e[3])
        elif what == 'detections':
            h.detections(t, e[2], e[3], e[4])
        elif what == 'heartbeat':
            h.heartbeat(t, e[2], e[3])
        elif what == 'actuator':
            h.actuator(t, e[2], e[3], e[4])
        elif what == 'speed':
            h.speed_cmd(t, e[2], e[3])
        elif what == 'cloud':
            h.cloud(t, e[2], e[3])
        elif what == 'status':
            h.status(t, e[2], e[3], e[4], e[5])
        elif what == 'clear':
            h.dog.clear()
        elif what == 'tick':
            states, locked, summary, said = h.tick(t, e[2], set(e[3]))
            tick = {'states': [list(s) for s in states], 'locked': locked,
                    'summary': summary, 'said': said}
            if len(ticks) % 10 == 9:
                tick['diagnostics'] = h.diagnostics(t, states)
            ticks.append(tick)
    return {'rules': [rule_dict(r) for r in rs], 'events': events, 'ticks': ticks,
            'faults': faults}


def picture_digests():
    rng = random.Random(4)
    out = []
    for n in (0, 1, 996, 997, 998, 5000, 20000):
        data = rng.randbytes(n)
        out.append({'hex': data.hex(), 'digest': zlib.crc32(data[::997])})
    return out


def main():
    vectors = {'runs': [run(seed, variant) for seed, variant in
                        ((1, 0), (2, 1), (3, 2), (4, 0))],
               'pictures': picture_digests()}
    OUT.write_text(json.dumps(enc(vectors), separators=(',', ':'), allow_nan=False) + '\n')
    print(f'wrote {OUT.relative_to(ROS.parent)} ({OUT.stat().st_size // 1024} kB)')


if __name__ == '__main__':
    main()
