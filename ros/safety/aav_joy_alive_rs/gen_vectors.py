#!/usr/bin/env python3
"""
Write joy-alive-core's agreement vectors from aav_safety's joy_alive.py.

Runs of Joy messages and ticks, as joy_alive_node.py drives JoyAlive: a pad released,
driven by a jittering hand, frozen while driving, silent, back with the trigger held,
short messages, NaN axes; several settings. The Rust (crates/joy-alive-core,
tests/vectors.rs) must give the same answer, the same trip and the same line for people,
every event. Axes are rounded to f32 first, as sensor_msgs/Joy carries them.

Writes ros/safety/aav_joy_alive_rs/crates/joy-alive-core/tests/vectors.json:

  pixi run python ros/safety/aav_joy_alive_rs/gen_vectors.py
"""

import json
import math
from pathlib import Path
import random
import struct
import sys

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / 'aav_safety')]

from aav_safety.joy_alive import JoyAlive  # noqa: E402

OUT = HERE / 'crates' / 'joy-alive-core' / 'tests' / 'vectors.json'


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


def enc(v):
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    if isinstance(v, (list, tuple)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


def pad(rng, phase, i, settings):
    """Return (axes, buttons) for this phase, or None for silence."""
    n_buttons = 3 if phase != 'short' else 1
    held = [0] * n_buttons
    if settings['enable_button'] < n_buttons:
        held[settings['enable_button']] = 1
    released = [0] * n_buttons
    if phase == 'silent':
        return None
    if phase == 'released':
        return [0.0, f32(rng.uniform(-1, 1)), 0.0], released
    if phase == 'hand':
        return [f32(rng.uniform(-0.02, 0.02)), f32(0.5 + 0.004 * ((i * 7) % 5 - 2)), 0.0], held
    if phase == 'frozen':
        return [0.0, 1.0, 0.0], held
    if phase == 'still':
        return [0.3, 0.0, 0.0], held
    if phase == 'nan':
        return [0.0, float('nan'), 0.0], held
    return [0.0, 0.5], held                       # short


def run(settings, seed):
    rng = random.Random(seed)
    check = JoyAlive(**settings)
    events = []
    t = rng.uniform(0.0, 1.0)
    phases = ['hand', 'released', 'hand', 'frozen', 'hand', 'released', 'hand', 'silent', 'hand',
              'released', 'still', 'nan', 'short', 'released', 'frozen', 'silent']
    for k in range(30):
        phase = phases[k % len(phases)] if k < len(phases) else rng.choice(phases)
        seconds = rng.choice([0.3, 1.0, 2.5, 4.0])
        for i in range(int(seconds / 0.05)):
            t = round(t + 0.05 + rng.uniform(-0.004, 0.004), 6)
            msg = pad(rng, phase, i, settings)
            if msg is not None:
                axes, buttons = msg
                ok = check.on_joy(t, axes, buttons)
                events.append({'op': 'joy', 't': t, 'axes': axes, 'buttons': buttons, 'ok': ok,
                               'tripped': check.tripped, 'sizes': list(check.sizes)})
            if i % 2 == 0:
                tripped_now = check.tick(t)
                events.append({'op': 'tick', 't': t, 'tripped_now': tripped_now,
                               'tripped': check.tripped, 'state': check.state(t)})
    return {'settings': {k: (list(v) if isinstance(v, tuple) else v)
                         for k, v in settings.items()}, 'events': events}


def build():
    """Return the vectors (NaN as 'nan'): aav_safety's test_joy_alive_vectors.py reruns it."""
    settings = [
        {'enable_button': 0, 'motion_axes': (1,), 'motion_threshold': 0.05,
         'silent_after': 0.5, 'frozen_after': 2.0},
        {'enable_button': 2, 'motion_axes': (0, 1), 'motion_threshold': 0.1,
         'silent_after': 0.35, 'frozen_after': 1.25},
        {'enable_button': 0, 'motion_axes': (1, 7), 'motion_threshold': 0.0,
         'silent_after': 1.0, 'frozen_after': 0.0},
        {'enable_button': 5, 'motion_axes': (1,), 'motion_threshold': 0.05,
         'silent_after': 0.123456789, 'frozen_after': 1234567.0},
    ]
    return enc({'note': 'Written by gen_vectors.py from aav_safety joy_alive.py; do not edit.',
                'runs': [run(s, i + 1) for i, s in enumerate(settings)]})


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(build(), separators=(',', ':')) + '\n')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
