#!/usr/bin/env python3
"""
Write lane-follow-core's agreement vectors from aav_lane_geometry's lane_follow.py.

Runs of lane messages and 0.1 s ticks, as control_interface.py drives LaneFollower: a lane
kept with a jittering offset, drifts both ways, a lane change and its commit, lines lost
and found, one line only, tight bends, messages that stop (the watchdog), a stop and the
resume; on the wall clock and a node clock that runs at another rate (a simulator slower
than real time). Several settings. The Rust (crates/lane-follow-core, tests/vectors.rs,
with the PID) must give the same commands, status lines, states and transitions, every
event. Message fields are rounded to f32 first, as LaneGeometry carries them.

Writes ros/drivers/aav_lane_follower_rs/crates/lane-follow-core/tests/vectors.json:

  pixi run python ros/drivers/aav_lane_follower_rs/gen_vectors.py
"""

from dataclasses import asdict
import json
import math
from pathlib import Path
import random
import struct
import sys
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parents[1] / 'perception' / 'aav_lane_geometry' / 'scripts')]

from lane_follow import LaneFollower, Settings  # noqa: E402

OUT = HERE / 'crates' / 'lane-follow-core' / 'tests' / 'vectors.json'


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


def message(offset, radius=9999.0, left=True, right=True):
    """Return a LaneGeometry's fields the follower reads, rounded to f32."""
    return SimpleNamespace(vehicle_offset_m=f32(offset), mean_curvature_m=f32(radius),
                           left_lane_detected=left, right_lane_detected=right,
                           both_lanes_detected=left and right)


# Each phase: (name, seconds). The message each phase sends at time i.
PHASES = [('keep', 3.0), ('drift_left', 2.0), ('keep', 1.0), ('change', 2.5),
          ('keep', 2.0), ('one_line', 1.5), ('bend', 2.0), ('lost', 3.0), ('keep', 1.0),
          ('quiet', 0.45), ('keep', 1.0), ('quiet', 2.0), ('keep', 2.0), ('drift_right', 2.0),
          ('tight', 1.5), ('lost', 0.4), ('keep', 1.0), ('tie', 0.5), ('quiet', 0.25),
          ('keep', 1.0)]


def lane_for(phase, i, rng):
    """Return the message for this phase, or None (no message: the messages stopped)."""
    jitter = rng.uniform(-0.02, 0.02)
    if phase == 'keep':
        return message(0.05 * math.sin(i / 7.0) + jitter)
    if phase == 'drift_left':
        return message(0.03 * i + jitter)
    if phase == 'drift_right':
        return message(-0.025 * i + jitter)
    if phase == 'change':
        # Jumps to the next lane's centre and comes back down as the car crosses.
        return message(1.6 - 0.12 * i if i < 14 else 0.05 + jitter)
    if phase == 'one_line':
        return message(0.1 + jitter, 60.0, left=i % 3 != 0, right=i % 3 == 0)
    if phase == 'bend':
        return message(0.08 + jitter, 35.0 - i)
    if phase == 'tight':
        return message(-0.1 + jitter, 6.0 + 0.5 * i, right=i % 4 != 0)
    if phase == 'lost':
        return message(0.0, 9999.0, left=False, right=False)
    if phase == 'tie':
        return message([0.0625, -0.0625, 0.0, -0.0, 0.1875][i % 5], [12.25, 0.0][i % 2])
    return None                                  # quiet


def run(settings, seed, rate):
    """Drive one follower; `rate` is the node clock's speed against the wall's."""
    rng = random.Random(seed)
    wall = 1_700_000_000.0 + rng.uniform(0.0, 10.0)
    now = rng.uniform(0.0, 100.0)
    follower = LaneFollower(settings, wall, now)
    start = {'wall': wall, 'now': now}
    events = []
    next_tick = now + 0.1
    for phase, seconds in PHASES:
        for i in range(int(seconds * 10)):
            step = 0.1 + rng.uniform(-0.01, 0.01)
            wall = round(wall + step, 6)
            now = round(now + step * rate, 6)
            msg = lane_for(phase, i, rng)
            if msg is not None:
                reply = follower.on_geometry(msg, wall, now)
                events.append({'op': 'geo', 'wall': wall, 'now': now, 'g': vars(msg),
                               'reply': asdict(reply)})
            while now >= next_tick:
                reply = follower.tick(wall, now)
                events.append({'op': 'tick', 'wall': wall, 'now': now,
                               'reply': asdict(reply)})
                next_tick += 0.1
    return {'settings': asdict(settings), 'start': start, 'events': events}


def build():
    runs = [
        (Settings(), 1, 1.0),
        (Settings(), 2, 0.37),
        (Settings(base_speed_mps=1.5, max_angular_z=0.3, pid_kp=1.2, pid_ki=0.05,
                  pid_kd=0.1, invert_offset_sign=False), 3, 1.0),
        (Settings(stop_wait_s=0.15, decel_rate=1.0, accel_rate=0.6, min_curvature_m=40.0,
                  lane_change_offset_threshold=0.3, lane_change_timeout_s=1.0), 4, 0.8),
        (Settings(decel_rate=0.0, min_speed_mps=0.6, lane_change_rate_threshold=0.5), 5, 1.6),
    ]
    return {'note': 'Written by gen_vectors.py from aav_lane_geometry lane_follow.py; '
                    'do not edit.',
            'runs': [run(s, seed, rate) for s, seed, rate in runs]}


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(build(), separators=(',', ':')) + '\n')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
