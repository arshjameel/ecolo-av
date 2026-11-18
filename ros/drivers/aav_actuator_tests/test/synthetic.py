"""
Pretend measurements with what the real car will have.

Play, coarse wheel speed, noise, a dead time. For test_robustness.py: the fits must give
the true numbers back, or refuse.

Plain Python, seeded, so every run gets the same numbers.
"""

import math
import random

from aav_actuator_tests import analysis as an

DT = 0.02          # s: the log's 50 Hz
WHEEL = 0.30       # m: the wheel's diameter
HOLES = 8          # holes in the encoder disc: a pulse every 0.118 m


def times(seconds, dt=DT):
    return [i * dt for i in range(int(round(seconds / dt)) + 1)]


def steps(t, changes):
    """Return a command that holds each value from its time on: changes = [(t, value)]."""
    out = []
    for ti in t:
        value = changes[0][1]
        for tc, v in changes:
            if ti >= tc:
                value = v
        out.append(value)
    return out


def noisy(values, sigma, seed=1):
    rng = random.Random(seed)
    return [v + rng.gauss(0.0, sigma) for v in values]


def backlash(values, width, start=0.0):
    return an.backlash(values, width, start)


def drive(t, command, gain, tau, dead, drag, sign=1.0, v0=0.0, deadband=0.0, fine=0.001):
    """
    Return (t_fine, speed, accel): command -> dead time -> lag -> push -> speed, finely.

    The actuator model's car (aav_sim_models actuator_model), stepped at 1 ms; the brake and
    the drag never reverse it.
    """
    tf = times(t[-1], fine)
    u_cmd = an.delayed(tf, t, command, dead)
    u = an.lag_response(tf, u_cmd, tau)
    v, speeds, accels = v0, [], []
    for ui in u:
        push = ui if ui > deadband else 0.0
        a = sign * gain * push - drag * v
        nv = v + a * fine
        if sign < 0 and nv < 0.0:
            nv, a = 0.0, -v / fine
        speeds.append(v)
        accels.append(a)
        v = nv
    return tf, speeds, accels


def hall(tf, speed, wheel=WHEEL, holes=HOLES, timeout=0.5, rate=0.01):
    """
    Return (t, speed) as a Hall-pulse wheel node sends it, every `rate` s.

    Each pulse gives spacing / time since the last pulse; between pulses that value is
    sent again; after `timeout` with no pulse, 0.
    """
    spacing = an.pulse_spacing(wheel, holes)
    s = 0.0
    pulses = []
    next_mark = spacing
    for i in range(1, len(tf)):
        step = 0.5 * (speed[i] + speed[i - 1]) * (tf[i] - tf[i - 1])
        while s + step >= next_mark and step > 0:
            # Timed to the microsecond, as the wheel node's timer does: within the step.
            pulses.append(tf[i - 1] + (tf[i] - tf[i - 1]) * (next_mark - s) / step)
            next_mark += spacing
        s += step
    out_t, out_v = [], []
    value, k = 0.0, 0
    t = 0.0
    while t <= tf[-1]:
        while k < len(pulses) and pulses[k] <= t:
            if k > 0:
                value = spacing / (pulses[k] - pulses[k - 1])
            k += 1
        last = pulses[k - 1] if k > 0 else -math.inf
        out_t.append(t)
        out_v.append(value if t - last <= timeout else 0.0)
        t = round(t + rate, 9)
    return out_t, out_v


def resample(tf, values, t):
    """Return values at the times t: the latest at or before each (sample-and-hold)."""
    return an.delayed(t, tf, values, 0.0)


def steer(t, command, rate, tau, dead, play=0.0, offset=0.0, fine=0.001):
    """
    Return the measured angle at the times t: dead time, motor at `rate`, play, lag, offset.

    Stepped at 1 ms, then read at t (sample-and-hold), as a sensor read by the log.
    """
    tf = times(t[-1], fine)
    motor = an.rate_lag_response(tf, an.delayed(tf, t, command, dead), rate, 0.0, 0.0)
    wheels = an.lag_response(tf, backlash(motor, play), tau)
    return resample(tf, [w + offset for w in wheels], t)
