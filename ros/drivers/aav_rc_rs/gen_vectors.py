#!/usr/bin/env python3
"""
Write rc-core's agreement vectors from aav_rc's rc_drive.py.

A run of /vehicle/rc messages and ticks, as rc_driver.py drives RcDrive: every mode switch,
the sticks moving (NaN and past the ends too), the gear on brake, silences on and off
RC via ROS. The Rust (crates/rc-core, tests/vectors.rs) must give the same command, or
none, every tick.

Writes ros/drivers/aav_rc_rs/crates/rc-core/tests/vectors.json:

  pixi run python ros/drivers/aav_rc_rs/gen_vectors.py
"""

import json
import math
from pathlib import Path
import random
import sys

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / 'aav_rc')]

from aav_rc.rc_drive import Limits, RcDrive, Sticks  # noqa: E402

OUT = HERE / 'crates' / 'rc-core' / 'tests' / 'vectors.json'


def enc(v):
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    if isinstance(v, (list, tuple)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


def run(limits, seed):
    rng = random.Random(seed)
    drive = RcDrive(limits)
    events = []
    switch = 2
    for i in range(1500):
        t = round(i * 0.02 + rng.uniform(0, 0.003), 6)
        if i % 150 == 0:
            switch = rng.choice((0, 1, 2, 2, 3, 7))
        quiet = (i // 100) % 5 == 3
        if not quiet and i % 2 == 0:
            throttle = rng.choice((rng.uniform(-1.3, 1.3), 0.0, 1.0, -1.0, float('nan')))
            steering = rng.choice((rng.uniform(-1.3, 1.3), 0.0, float('nan')))
            brake = rng.random() < 0.1
            drive.heard(Sticks(switch, throttle, steering, brake), t)
            events.append({'op': 'rc', 't': t, 'switch': switch, 'throttle': throttle,
                           'steering': steering, 'brake': brake})
        cmd = drive.command(t)
        events.append({'op': 'tick', 't': t, 'cmd': None if cmd is None else list(cmd),
                       'driving': drive.driving})
    return {'limits': vars(limits), 'events': events}


def main():
    data = {'note': 'Written by gen_vectors.py from aav_rc; do not edit.',
            'runs': [run(Limits(), 1),
                     run(Limits(max_speed=2.0, max_reverse_speed=0.7, max_steering_angle=0.45,
                                wheelbase=1.6, stale=0.3), 2)]}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(enc(data), separators=(',', ':')) + '\n')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
