#!/usr/bin/env python3
"""
Write legacy-bridge's agreement vectors from aav_legacy's Python modules.

The modules: legacy_protocol.py (the mapping, the uplink parser), uart_relay.py (what is
written each tick), speed_control.py (the pi loop), uplink_reader.py and link_report.py,
driven the way legacy_uart_bridge.py drives them. The Rust (crates/legacy-bridge,
tests/vectors.rs) must give the same answers: every frame byte for byte, every pedal
exactly, every word of the calibration line.

Writes ros/can_bridge/aav_legacy_rs/crates/legacy-bridge/tests/vectors.json. Run it after
any change to those modules:

  pixi run python ros/can_bridge/aav_legacy_rs/gen_vectors.py
"""

import json
import math
from pathlib import Path
import random
import sys

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / 'aav_legacy')]

from aav_legacy import legacy_protocol as lp  # noqa: E402
from aav_legacy import uplink_reader as ur  # noqa: E402
from aav_legacy.link_report import frame_fields, LinkReport  # noqa: E402
from aav_legacy.speed_control import SpeedControl, SpeedControlConfig  # noqa: E402
from aav_legacy.uart_relay import programs_locked, Relay  # noqa: E402

OUT = HERE / 'crates' / 'legacy-bridge' / 'tests' / 'vectors.json'
MAPPING = lp.LegacyMapping(max_steering_angle=0.45, steering_sign=-1.0,
                           full_throttle_speed=3.0, stop_speed=0.05, hold_brake=0.3,
                           stop_brake=1.0)


def enc(v):
    """Return v with every non-finite float as a string, for strict JSON."""
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    if isinstance(v, (list, tuple)):
        return [enc(x) for x in v]
    if isinstance(v, dict):
        return {k: enc(x) for k, x in v.items()}
    return v


def mapping_cases():
    """to_legacy and from_pedals on a grid, edges and NaN included."""
    speeds = [0.0, 0.01, -0.04, 0.05, 0.06, -0.06, 0.09, 0.5, -0.7, 2.9, 3.0, 4.5, -9.0,
              float('nan')]
    angles = [0.0, 0.1, -0.1, 0.45, -0.45, 0.6, -0.6, float('nan')]
    cases = []
    for v in speeds:
        for a in angles:
            cases.append({'op': 'to_legacy', 'speed': v, 'steering': a,
                          'out': list(lp.to_legacy(v, a, MAPPING))})
    for a in angles[:6]:
        for throttle, brake in ((0.0, 0.0), (0.02, 0.0), (0.03, 0.0), (0.5, 0.0), (1.0, 0.0),
                                (0.0, 0.1), (0.0, 1.0), (0.0, 1.5), (0.4, 0.2)):
            cases.append({'op': 'from_pedals', 'steering': a, 'throttle': throttle,
                          'brake': brake,
                          'out': list(lp.from_pedals(a, throttle, brake, MAPPING))})
    return cases


def speed_runs():
    """Run the pi loop alone: ego's speed profile, a lagging car, a stop, silences."""
    runs = []
    rng = random.Random(7)
    for smooth in (0.3, 0.0):
        cfg = SpeedControlConfig(smooth_stop_speed=smooth)
        loop = SpeedControl(cfg)
        car, steps = 0.0, []
        for i in range(600):
            t = i * 0.02
            target = 1.5 if t < 4.0 else max(0.0, 1.5 - (t - 4.0) * 1.0)
            if 9.0 < t < 9.5:
                target = 0.8
            dt = 0.02 + rng.uniform(-0.004, 0.004)
            measured = None if 2.0 < t < 2.4 else car + rng.uniform(-0.01, 0.01)
            p = loop.update(target, measured, dt)
            car = max(0.0, car + (p.throttle * cfg.max_accel - p.brake * cfg.max_brake
                                  - cfg.drag * car) * dt)
            steps.append({'target': target, 'measured': measured, 'dt': dt,
                          'out': [p.throttle, p.brake, p.accel, p.feedforward, p.integral]})
        runs.append({'config': vars(cfg), 'steps': steps})
    return runs


def relay_runs():
    """Relay scenarios: commands, the counter, e-stops, odometry, the panel's calibration."""
    runs = []
    for version in (1, 2, 3):
        for mode in ('open', 'pi'):
            for where in ('none', 'stands'):
                if where == 'stands' and version != 3:
                    continue
                runs.append(relay_run(version, mode, where, random.Random(
                    version * 100 + len(mode) * 10 + len(where))))
    return runs


def relay_run(version, mode, where, rng):
    loop = SpeedControl(SpeedControlConfig()) if mode == 'pi' else None
    relay = Relay(MAPPING, 0.3, version, loop, 0.3, where)
    relay.uplink_timeout = 0.2
    events = []
    counter = 0
    speed = 0.0
    for i in range(500):
        t = round(i * 0.02 + rng.uniform(0.0, 0.004), 6)
        phase = (i // 50) % 6
        if phase in (0, 1, 3) and rng.random() > 0.05:
            speed = min(1.2, speed + 0.03) if phase != 3 else max(0.0, speed - 0.05)
            steering = rng.uniform(-0.5, 0.5)
            relay.command(speed, steering, t)
            events.append({'op': 'cmd', 't': t, 'speed': speed, 'steering': steering})
            counter = (counter + 1) & 0xFF
            relay.heartbeat(counter)
            events.append({'op': 'hb', 'counter': counter})
        if i % 7 == 0:
            odom = speed * rng.uniform(0.8, 1.0) if phase != 4 else 0.0
            relay.wheel_speed(odom, t)
            events.append({'op': 'odom', 't': t, 'speed': odom})
        if i in (120, 260):
            on = i == 120
            changed = relay.set_estop(on)
            events.append({'op': 'estop', 'on': on, 'changed': changed})
        if phase in (4, 5) and i % 5 == 0:
            m = rng.choice((0, 1, 2, 2))
            target = rng.choice((-20, 300, 2000, 5000))
            relay.calibrate(m, target, t)
            events.append({'op': 'cal', 't': t, 'mode': m, 'target': target})
        if i % 2 == 0 and not (380 < i < 400):
            relay.uplink_seen(t)
            events.append({'op': 'uplink', 't': t})
        frame = relay.frame(t)
        refusal = relay.cal_refusal(t)
        events.append({'op': 'frame', 't': t, 'hex': frame.hex(),
                       'fresh': relay.fresh(t), 'cal_wanted': relay.cal_wanted(t),
                       'refusal': refusal})
        if i % 97 == 0:
            events.append({'op': 'stop_frame', 'hex': relay.stop_frame().hex()})
    return {'version': version, 'mode': mode, 'calibration': where,
            'locked': programs_locked(version, False),
            'locked_allowed': programs_locked(version, True), 'events': events}


def uplink_frames(rng, n):
    """Good uplink frames, versions 2 and 3, with every field moving."""
    out = []
    for i in range(n):
        version = 3 if i % 3 else 2
        u = lp.Uplink(sequence=(i * 1 + (5 if i == 20 else 0)) & 0xFF,
                      echo=None if i % 11 == 0 else (i * 3) & 0xFF,
                      mode=i % 3, switch=i % 4, armed=bool(i % 2), estop=i % 5 == 0,
                      faults=(i * 37) & 0xFFFF,
                      steering=None if i % 7 == 0 else rng.uniform(-0.5, 0.5),
                      brake_position=None if i % 6 == 0 else rng.uniform(0, 1),
                      throttle_out=rng.uniform(0, 1), brake_out=rng.uniform(0, 1),
                      rc_throttle=rng.uniform(-1, 1), rc_steering=rng.uniform(-1, 1),
                      rc_brake=i % 4 == 1,
                      steering_counts=None if i % 4 == 0 else rng.randrange(4096),
                      brake_counts=None if i % 5 == 1 else rng.randrange(4096),
                      calibration=i % 10)
        out.append(lp.pack_uplink(u, version))
    return out


def uplink_runs():
    """Byte streams: good frames, junk, bad CRCs, unknown versions, split reads."""
    rng = random.Random(11)
    frames = uplink_frames(rng, 60)
    stream = bytearray()
    for i, f in enumerate(frames):
        if i % 9 == 4:
            stream += bytes([0x00, 0x13, lp.SYNC, 0x07, 0x01])      # junk, a bad version
        if i % 13 == 6:
            bad = bytearray(f)
            bad[10] ^= 0x40
            stream += bad                                            # a bad CRC
        stream += f
    reader = ur.UplinkReader(0.2)
    reads, at, t = [], 0, 0.0
    while at < len(stream):
        n = rng.choice((1, 3, 17, 35, 40, 64, 90))
        chunk = bytes(stream[at:at + n])
        at += n
        t = round(t + 0.007, 6)
        found = reader.feed(chunk, t)
        reads.append({'t': t, 'hex': chunk.hex(), 'found': [uplink_out(u) for u in found],
                      'health': ur.health_fields(reader, t),
                      'health_later': ur.health_fields(reader, t + 0.25),
                      'line': ur.calibration_line(False, None, reader.latest),
                      'line_refused': ur.calibration_line(True, 'the e-stop is on',
                                                          reader.latest)})
    return {'reads': reads, 'skipped': reader.parser.skipped}


def uplink_out(u):
    command = 0.25 if u.sequence % 2 else None
    return {'sequence': u.sequence, 'status': ur.status_fields(u),
            'steering': list(ur.steering_state(u, command)), 'command': command,
            'brake': list(ur.brake_state(u)), 'throttle': list(ur.throttle_state(u)),
            'rc': ur.rc_fields(u), 'raw_steering': ur.raw_fields(u.steering_counts,
                                                                 u.calibration),
            'raw_brake': ur.raw_fields(u.brake_counts, 0)}


def link_runs():
    """frame_fields on frames of each version (and bad ones); LinkReport's samples."""
    frames = []
    for values in ((0.1, 0.2, -0.3), (-1.0, 0.0, -1.0), (0.5, 0.031, 0.0)):
        frames.append(lp.pack_frame(*values))
        frames.append(lp.pack_frame_v2(*values, counter=200, armed=True, estop=False))
        frames.append(lp.pack_frame_v3(*values, counter=9, armed=False, estop=True,
                                       cal_mode=lp.CAL_MODE_JOG, jog_target=1234))
        frames.append(lp.pack_frame_v3(*values, counter=9, armed=False, estop=True,
                                       cal_mode=lp.CAL_MODE_LIMP))
    cases = []
    for f in frames:
        cases.append({'hex': f.hex(), 'fields': frame_fields(f)})
    for f in (frames[1][:-1] + bytes([frames[1][-1] ^ 1]), frames[0][:5], b''):
        try:
            frame_fields(f)
            raise AssertionError('should be turned down')
        except ValueError:
            cases.append({'hex': f.hex(), 'fields': None})
    samples = []
    for rate in (10.0, 0.0, 7.0):
        rep = LinkReport(rate)
        rng = random.Random(int(rate))
        t, dues = 0.0, []
        for i in range(300):
            t = round(t + 0.02 + rng.uniform(-0.003, 0.003) + (0.5 if i == 150 else 0.0), 6)
            sent = 17 if i % 41 else 5
            ok = rep.wrote(bytes(17), sent)
            dues.append([t, sent, ok, rep.due(t)])
        samples.append({'rate': rate, 'steps': dues, 'written': rep.written,
                        'write_errors': rep.write_errors})
    return {'frames': cases, 'reports': samples}


def main():
    data = {'note': 'Written by gen_vectors.py from aav_legacy; do not edit.',
            'mapping': vars(MAPPING), 'mapping_cases': mapping_cases(),
            'speed_runs': speed_runs(), 'relay_runs': relay_runs(),
            'uplink': uplink_runs(), 'link': link_runs()}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(enc(data), indent=None, separators=(',', ':')) + '\n')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
