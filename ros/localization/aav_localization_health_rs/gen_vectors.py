#!/usr/bin/env python3
"""
Write health-core's agreement vectors from the Python module, and tf2's own answers.

The module: aav_localization_health's health.py, fed the way localization_health.py
feeds it. The Rust (crates/health-core, tests/vectors.rs) must give the same answers,
exactly: every check's level, words and numbers, the summary line, and each number as
/diagnostics writes it (format(v, '.4g')).

Each run is 60 s of a made-up car: parked, driving, parked again, with faults on purpose:
a gyro bias, an IMU upside down, tilted or in g, a GPS losing its fix or its covariance,
late and future stamps, wrong frames, a dead or unmeasured front wheel, a runaway or
jumping EKF, inputs gone quiet, and the TF check's answers.

And the TF tree's ages: random trees of the car's frames (some links missing, moving or
static, one stamp or a stream of them) fed to tf2_ros's own Buffer, asked what the node
asks (base_link -> imu_link and gps_link there? map -> base_link's stamp at Time()),
against crates/health-core/src/tree.rs. Needs ROS (pixi); without it, that part is
left out.

Writes ros/localization/aav_localization_health_rs/crates/health-core/tests/vectors.json.
Run it with Python 3.12 or newer (sum() is compensated from 3.12) after any change:

  pixi run python ros/localization/aav_localization_health_rs/gen_vectors.py
"""

from dataclasses import replace
import json
import math
from pathlib import Path
import random
import sys

HERE = Path(__file__).resolve().parent
ROS = HERE.parent.parent
sys.path[:0] = [str(ROS / 'localization' / 'aav_localization_health')]

from aav_localization_health.health import (  # noqa: E402
    front_wheel_speeds, G, Health, Settings, summary)

OUT = HERE / 'crates' / 'health-core' / 'tests' / 'vectors.json'
WHEELBASE, TRACK = 1.561, 0.75
MOUNTS = ['level', 'upside', 'units', 'tilt', 'level', 'tilt']


def enc(v):
    """Return v with every non-finite float as a string, for strict JSON."""
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    if isinstance(v, (list, tuple)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


def r4(x):
    return round(x, 4)


def run(seed, settings):
    rng = random.Random(seed)
    h = Health(WHEELBASE, TRACK, settings)
    events, reports = [], []

    def window(lo, hi, length):
        start = rng.uniform(lo, hi)
        return (start, start + length)
    faults = {
        'imu_silent': window(20, 30, 3.0), 'odom_silent': window(42, 50, 2.6),
        'imu_late': window(14, 18, 1.0), 'imu_future': window(30, 34, 0.6),
        'odom_frame': window(5, 8, 1.0), 'gps_frame': window(20, 25, 1.5),
        'local_child': window(35, 38, 0.7),
        'gps_nofix': window(15, 20, 2.0), 'gps_nocov': window(25, 28, 2.0),
        'gps_poor': window(30, 32, 1.0), 'gps_sure': window(33, 35, 1.0),
        'gps_silent': window(36, 39, 3.0) if seed % 2 else (99.0, 99.0),
        'wheel_dead': window(18, 26, 2.5), 'wheel_nan': window(28, 32, 1.0),
        'local_fast': window(16, 20, 0.04), 'local_jump': window(22, 26, 0.04),
        'global_jump': window(24, 28, 0.04), 'local_spin': window(32, 36, 0.04),
        'global_kick': window(26, 30, 0.1),
    }
    # The parked IMU: biased or not, mounted right or not (one per run).
    bias = math.radians(rng.choice([0.0, 0.05, 0.4, -0.8]))
    mount = MOUNTS[(seed - 1) % len(MOUNTS)]
    gravity = {'level': (0.0, 0.0, G), 'upside': (0.0, 0.0, -G), 'units': (0.0, 0.0, 1.0),
               'tilt': (G * math.sin(0.3), 0.0, G * math.cos(0.3))}[mount]
    wheels_here = seed % 3 != 0         # no wheel speeds on today's car
    gps_here = seed % 4 != 1            # no GPS on the bench

    def on(name, t):
        a, b = faults[name]
        return a <= t < b

    v, yaw, x, y, gx, gy = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    dt = 0.02
    for k in range(int(60.0 / dt)):
        t = r4(k * dt)
        driving = 12.0 <= t < 40.0
        target = 2.0 if driving else 0.0
        v = min(target, v + 0.8 * dt) if v < target else max(target, v - 1.2 * dt)
        v = r4(v)
        wz = r4(0.3 * math.sin(0.4 * t) if driving else 0.0)
        yaw += wz * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        # IMU, 25 Hz.
        if k % 2 == 0 and not on('imu_silent', t):
            stamp = t - 1.0 if on('imu_late', t) else t + 0.3 if on('imu_future', t) else t
            accel = [r4(gravity[i] + rng.gauss(0, 0.05)) for i in range(3)]
            gyro = r4(wz + bias + rng.gauss(0, 0.002))
            events.append(['imu', t, r4(stamp), 'imu_link', gyro, accel])
        # Wheel odometry, 25 Hz.
        if k % 2 == 1 and not on('odom_silent', t):
            stamp = r4(t - rng.choice([0.0, 0.0, 0.01, 0.04]))
            child = 'base_footprint' if on('odom_frame', t) else 'base_link'
            events.append(['odom', t, stamp, 'odom', child, v])
        # Front wheels, 25 Hz.
        if wheels_here and k % 2 == 0:
            left, right = front_wheel_speeds(v, wz, WHEELBASE, TRACK)
            left, right = r4(left + rng.gauss(0, 0.01)), r4(right + rng.gauss(0, 0.01))
            if on('wheel_dead', t):
                right = 0.0
            if on('wheel_nan', t):
                left = float('nan')
            events.append(['wheels', t, t, 'base_link', left, right])
        # GPS, 5 Hz.
        if gps_here and k % 10 == 0 and not on('gps_silent', t):
            sigma = rng.uniform(0.3, 2.0)
            status, ctype = 0, 2
            if on('gps_nofix', t):
                status = -1
            if on('gps_nocov', t):
                ctype = 0
            if on('gps_poor', t):
                sigma = 8.0
            if on('gps_sure', t):
                sigma = 0.002
            cov = [r4(sigma ** 2), 0.0, 0.0, 0.0, r4(sigma ** 2), 0.0, 0.0, 0.0,
                   r4(4 * sigma ** 2)]
            frame = 'gps' if on('gps_frame', t) else 'gps_link'
            events.append(['gps', t, r4(t - 0.1), frame, status, ctype, cov])
        # ekf_local, 25 Hz; ekf_global, 10 Hz.
        if k % 2 == 0:
            lv, lx, lwz = v, r4(x), wz
            if on('local_fast', t):
                lv = 9.0
            if on('local_jump', t):
                lx = r4(x + 30.0)
            if on('local_spin', t):
                lwz = 3.0
            child = 'base' if on('local_child', t) else 'base_link'
            events.append(['ekf', 'local', t, t, 'odom', child, lx, r4(y), lv, lwz])
        if k % 5 == 0:
            gx, gy = r4(x + rng.gauss(0, 0.05)), r4(y + rng.gauss(0, 0.05))
            if on('global_jump', t):
                gx += 12.0
            # A kick: 1 m/s more in 0.1 s (10 m/s^2), then back.
            gv = r4(v + 1.0) if on('global_kick', t) else v
            events.append(['ekf', 'global', t, t, 'map', 'base_link', gx, gy, gv, wz])
        # The node's tick: the TF check, then the report (1 Hz on the car; 10 Hz here).
        if k % 5 == 4:
            pick = rng.random()
            if pick < 0.8:
                tf = [[], r4(rng.uniform(0.0, 0.3))]
            elif pick < 0.85:
                tf = [['gps_link'], 0.1]
            elif pick < 0.9:
                tf = [['imu_link', 'gps_link'], None]
            elif pick < 0.95:
                tf = [[], None]
            else:
                tf = [[], 3.5]
            events.append(['tf', t] + tf)
            events.append(['report', t])
    for e in events:
        what = e[0]
        if what == 'imu':
            h.imu(e[1], e[2], e[3], e[4], tuple(e[5]))
        elif what == 'odom':
            h.odom(e[1], e[2], e[3], e[4], e[5])
        elif what == 'wheels':
            h.wheels(e[1], e[2], e[3], e[4], e[5])
        elif what == 'gps':
            h.gps_fix(e[1], e[2], e[3], e[4], e[5], e[6])
        elif what == 'ekf':
            h.ekf(e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8], e[9])
        elif what == 'tf':
            h.tf(e[2], e[3])
        elif what == 'report':
            statuses = h.report(e[1])
            reports.append({
                'statuses': [[s.name, s.level, s.message, [[k, v] for k, v in s.values.items()],
                              [f'{v:.4g}' for v in s.values.values()]] for s in statuses],
                'summary': summary(statuses)})
    return {'settings': vars(settings), 'events': events, 'reports': reports,
            'faults': faults, 'mount': mount, 'bias': bias}


def tf_trees():
    """Random trees of the car's frames through tf2's own Buffer: the node's questions."""
    try:
        from builtin_interfaces.msg import Time as Stamp
        from geometry_msgs.msg import TransformStamped
        from rclpy.duration import Duration
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformException
    except ImportError:
        print('no ROS here: the TF trees are left out', file=sys.stderr)
        return []
    rng = random.Random(7)
    parents = {'odom': 'map', 'base_link': 'odom', 'imu_link': 'base_link',
               'gps_link': 'base_link'}
    out = []
    for _ in range(300):
        buf = Buffer(cache_time=Duration(seconds=10.0))
        sets = []
        for child, parent in parents.items():
            if rng.random() < 0.2:
                continue                                    # this link is missing
            fixed = child in ('imu_link', 'gps_link') and rng.random() < 0.85
            if fixed:
                stamps = [0.0]
            else:
                # One stamp, or a stream ending at a random time (a filter's output).
                end = round(rng.uniform(100.0, 105.0), 3)
                n = rng.choice([1, 1, 3, 20])
                stamps = [round(end - i * rng.choice([0.02, 0.1]), 3) for i in range(n)][::-1]
            for stamp in stamps:
                msg = TransformStamped()
                msg.header.frame_id, msg.child_frame_id = parent, child
                sec = int(stamp)
                msg.header.stamp = Stamp(sec=sec, nanosec=int(round((stamp - sec) * 1e9)))
                msg.transform.rotation.w = 1.0
                if fixed:
                    buf.set_transform_static(msg, 'gen_vectors')
                else:
                    buf.set_transform(msg, 'gen_vectors')
                sets.append([parent, child, msg.header.stamp.sec, msg.header.stamp.nanosec,
                             fixed])
        asks = []
        for target, source in (('base_link', 'imu_link'), ('base_link', 'gps_link'),
                               ('map', 'base_link'), ('map', 'imu_link')):
            try:
                got = buf.lookup_transform(target, source, Time())
                s = got.header.stamp
                asks.append([target, source, s.sec + s.nanosec * 1e-9])
            except TransformException:
                asks.append([target, source, None])
        out.append({'sets': sets, 'asks': asks})
    return out


def main():
    quick = replace(Settings(), still_min=3.0, stale_after=1.5, hold=2.5, persist=0.5)
    vectors = {'runs': [run(seed, s) for seed, s in
                        ((1, Settings()), (2, Settings()), (3, quick), (4, Settings()),
                         (5, Settings()), (6, quick))],
               'trees': tf_trees()}
    OUT.write_text(json.dumps(enc(vectors), separators=(',', ':'), allow_nan=False) + '\n')
    print(f'wrote {OUT.relative_to(ROS.parent)} ({OUT.stat().st_size // 1024} kB)')


if __name__ == '__main__':
    main()
