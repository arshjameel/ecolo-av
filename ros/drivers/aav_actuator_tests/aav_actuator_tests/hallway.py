"""
Phase B's analysis: a hallway session's runs to transfer functions with error bars. No ROS.

TODO 248, docs/testing-plan.md phase B, docs/actuator-tests.md "The hallway".

    pixi run car-test --hallway                 drives the passes (car_test_node.py)
    pixi run car-test --analyse <session>       this module, again, from the files

A session folder holds one folder per pass (`NN-hall-<kind>-<direction>/`, an
actuator_test run inside) and `session.yaml`. analyse() reads them all:

| Step | What | Where |
|---|---|---|
| 1 | each pass's 10 s standing still: the IMU's offset and noise | fusion.still_imu |
| 2 | the offsets facing each way: the floor's slope; level enough? | fusion.slope_from |
| 3 | each pass: the IMU blended with the wheels: speed, acceleration | fusion.fuse |
| 4a | the coasts: the drag | fit_coast |
| 4b | the speed-ups: max_accel, the throttle's lag and dead time | fit_speed_up |
| 4c | the brakes: max_brake, the brake's lag and dead time | fit_brake |
| 4d | the steering on the ground: its lag and dead time | fit_steer |
| 5 | each number over every pass: mean ± sd, trusted or not | errorbars.estimate |
| 6 | each pass through the car model, fitted and as vehicle.yaml says: | replay, compare |
|   | speed within 10 %, stop within 0.2 m | |
| 7 | the forces: m a (vehicle.yaml's mass) | fusion.forces |
| 8 | report.md, fits.csv, compare.csv, overlays/ (CSV; PNG), vehicle-yaml.txt | write |

In Ravensim the true numbers are known (session.yaml `truth`: vehicle.yaml with the
sim's `actuators:=` overrides): the report adds the recovery table, true against fitted.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field, replace
import datetime
import math
import os
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from aav_actuator_tests import analysis as an
from aav_actuator_tests import errorbars as eb
from aav_actuator_tests import fusion
from aav_actuator_tests import procedures as pr
from aav_actuator_tests import reports
from aav_actuator_tests.errorbars import Value

NAN = float('nan')
DT = 0.02                 # s: the replay's step (the backends' 50 Hz)
SPEED_TOLERANCE = 0.10    # of the pass's top speed: the model's rms speed error (the plan's)
STOP_TOLERANCE = 0.2      # m: the model's stopping distance error (the plan's)
MOVING = 0.2              # m/s: the brake fit ends here (it can't push a stopped car)
FREE = 0.01               # a pedal level under this is off
KINDS = ('coast', 'speed-up', 'brake', 'steer')   # the fitting order: drag first
EDGE = 0.04               # s: a dead time this near 0 counts as none
# The least error bar on a number read through the wheel speed (gains, the drag, a stop):
# the wheel's size is known to a few % until the rollout test (docs/measure-the-car.md M3),
# and every metre and m/s scales with it.
SCALE = 0.03
SCALED = ('drag', 'max_accel', 'max_brake', 'stop_distance', 'stop_decel')

# Each number: its vehicle.yaml key, unit, digits, and the pass kind it comes from.
KEYS = {
    'drag': ('actuators.drag', '1/s', 3, 'coast'),
    'rolling': ('', 'm/s^2', 3, 'coast'),
    'max_accel': ('actuators.max_accel', 'm/s^2', 3, 'speed-up'),
    'throttle_lag': ('actuators.throttle_lag', 's', 3, 'speed-up'),
    'throttle_dead_time': ('actuators.throttle_dead_time', 's', 3, 'speed-up'),
    'throttle_delay': ('', 's', 3, 'speed-up'),
    'max_brake': ('actuators.max_brake', 'm/s^2', 3, 'brake'),
    'brake_lag': ('actuators.brake_lag', 's', 3, 'brake'),
    'brake_dead_time': ('actuators.brake_dead_time', 's', 3, 'brake'),
    'brake_delay': ('', 's', 3, 'brake'),
    'steer_lag': ('actuators.steer_lag', 's', 3, 'steer'),
    'steer_dead_time': ('actuators.steer_dead_time', 's', 3, 'steer'),
    'steer_delay': ('', 's', 3, 'steer'),
    'stop_distance': ('', 'm', 3, 'brake'),
    'stop_decel': ('', 'm/s^2', 3, 'brake'),
}
# <x>_delay = <x>_dead_time + <x>_lag: when the response is 63 % there. Through a ramped
# command (ego's ramp, the speed loop) a dead time and a lag trade off against each other;
# their sum is what the data pins down best.
DELAYS = {'throttle_delay': 'throttle', 'brake_delay': 'brake', 'steer_delay': 'steer'}


# --- one pass ---------------------------------------------------------------------------


@dataclass
class Pass:
    """One run down the hallway: its rows, what it was, and what was worked out."""

    name: str                 # its folder, relative to the session
    kind: str                 # coast, speed-up, brake, steer
    direction: str            # there, back
    rows: List[Dict]
    meta: Dict
    ctx: reports.Context
    still: Optional[fusion.Still] = None
    fused: Optional[fusion.Fused] = None
    values: Dict[str, Value] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)

    def phase_span(self, prefix: str) -> Optional[Tuple[float, float]]:
        """Return (start, end) of the first phase starting with prefix (row times)."""
        for name, rs in reports.phases(self.rows):
            if name.startswith(prefix):
                return float(rs[0]['t']), float(rs[-1]['t'])
        return None

    def spans(self, prefix: str) -> List[Tuple[float, float]]:
        return [(float(rs[0]['t']), float(rs[-1]['t']))
                for name, rs in reports.phases(self.rows) if name.startswith(prefix)]


def load_pass(folder: str, name: str) -> Optional[Pass]:
    """Read one actuator_test run folder (log.csv, meta.yaml); None if it isn't a hall run."""
    import yaml
    meta_path = os.path.join(folder, 'meta.yaml')
    log_path = os.path.join(folder, 'log.csv')
    if not (os.path.exists(meta_path) and os.path.exists(log_path)):
        return None
    with open(meta_path) as f:
        meta = yaml.safe_load(f) or {}
    test = str(meta.get('test', ''))
    if not test.startswith('hall-'):
        return None
    with open(log_path, newline='') as f:
        rows = reports.parse_rows(list(csv.DictReader(f)))
    hall = meta.get('hallway') or {}
    return Pass(name, test[len('hall-'):], str(hall.get('direction', 'there')), rows, meta,
                reports.context_from_meta(meta))


def signal(p: Pass, name: str) -> Tuple[List[float], List[float]]:
    return reports.signal(p.rows, name)


def stand_still(p: Pass) -> Optional[fusion.Still]:
    """Return the pass's own stand-still window: the IMU's offset and noise."""
    span = p.phase_span(pr.HALL_STILL)
    t, ax = signal(p, 'ax')
    st, sv = signal(p, 'speed')
    if span is None or not t:
        return None
    # The first second: the car may still settle on its springs.
    return fusion.still_imu(t, ax, st, sv, span[0] + 1.0, span[1])


def single(rows, ctx: reports.Context, kind: str) -> Pass:
    """One pass alone (actuator_test's own summary): its stand-still, fusion and fit."""
    p = Pass('this run', kind, 'there', list(rows), {}, ctx)
    p.still = stand_still(p)
    p.fused = fuse_pass(p, fusion.slope_from([]))
    drag = ctx.current.get('actuators.drag', 0.0)
    if kind == 'coast':
        p.values = fit_coast(p)
    elif kind == 'speed-up':
        p.values = fit_speed_up(p, drag, ctx.current.get('actuators.throttle_deadband', 0.0))
    elif kind == 'brake':
        p.values = fit_brake(p, drag, {k: ctx.current.get(f'actuators.{k}', NAN)
                                       for k in ('max_accel', 'throttle_lag')})
    else:
        p.values = fit_steer(p, ctx.current.get('actuators.steer_rate', 0.8))
    problems = p.values.pop('_problems', None)
    if problems and problems.problem:
        p.notes.append(problems.problem)
    return p


def fuse_pass(p: Pass, slope: fusion.Slope) -> fusion.Fused:
    t, ax = signal(p, 'ax')
    st, sv = signal(p, 'speed')
    s = p.still
    offset = s.offset if s and s.trusted else (s.offset if s else 0.0)
    if s is None or not s.trusted:
        p.notes.append('no trusted stand-still window: the IMU offset is '
                       + (f'{offset:+.3f} m/s^2 from {s.duration:.1f} s' if s else 'taken as 0'))
    return fusion.fuse(t, ax, st, sv, offset=offset,
                       imu_noise=s.noise if s and fusion.finite(s.noise) else 0.2,
                       wheel_noise=max(s.wheel_noise, 0.03) if s else 0.05,
                       gravity=slope.gravity(p.direction))


def window(f: fusion.Fused, start: float, end: float):
    """Return the fused samples between start and end: (t, v, a)."""
    idx = [i for i, ti in enumerate(f.t) if start <= ti <= end]
    return [f.t[i] for i in idx], [f.speed[i] for i in idx], [f.accel[i] for i in idx]


def decimate(t, *series, period=DT):
    """Keep one sample every `period` s (the fits' cost goes with the samples)."""
    keep, last = [], -math.inf
    for i, ti in enumerate(t):
        if ti - last >= period - 1e-9:
            keep.append(i)
            last = ti
    return ([t[i] for i in keep],) + tuple([s[i] for i in keep] for s in series)


def pedal(p: Pass, name: str) -> Tuple[List[float], List[float]]:
    """Return a pedal's demand (throttle or brake), each from when it began to act."""
    if reports.has(p.rows, f'{name}_cmd'):
        return reports.command_signal(p.rows, f'{name}_cmd', p.ctx)
    if name == 'throttle':
        found = reports.open_loop_throttle(p.rows, p.ctx)
        if found is not None:
            return found
    return [], []


def pedal_on(p: Pass, name: str) -> Tuple[List[float], List[float]]:
    """Return the pedal's level where measured, else its demand: when is it off."""
    if reports.has(p.rows, name):
        return signal(p, name)
    return pedal(p, name)


def off_at(ts, vs, t: float) -> bool:
    v = fusion.held_at(ts, vs, t)
    return not (fusion.finite(v) and v > FREE)


def lag_values(prefix: str, fit: an.LagFit, signal_name: str) -> Dict[str, Value]:
    """Return a LagFit's lag, dead time and delay as Values, with what can't be resolved."""
    problems = fit.problems(prefix, signal_name)
    lag_why = fit.lag_problem(prefix, signal_name) or ''
    dead_why = '' if fit.dead_time < EDGE or fit.dead_resolved else (
        f'{prefix} dead time not resolved')
    delay = fit.tau + fit.dead_time
    delay_why = ('at the edge of the search' if fit.at_edge else
                 an.unresolved(f'{prefix} delay', delay, fit.resolution, signal_name) or '')
    return {f'{prefix}_delay': Value(delay, fit.resolution, delay_why),
            f'{prefix}_lag': Value(fit.tau, fit.resolution, lag_why),
            f'{prefix}_dead_time': Value(fit.dead_time, fit.resolution,
                                         dead_why if not lag_why else lag_why),
            '_problems': Value(NAN, 0.0, '; '.join(problems))}


# --- the fits ----------------------------------------------------------------------------


def fit_coast(p: Pass) -> Dict[str, Value]:
    """Fit the drag: the coast's speed (fused) falling with the pedals off."""
    span = p.phase_span('coast from ')
    if span is None or p.fused is None:
        return {}
    t, v, a = window(p.fused, span[0] + 0.5, span[1])
    tt, tv = pedal_on(p, 'throttle')
    bt, bv = pedal_on(p, 'brake')
    use = [i for i in range(len(t)) if v[i] > 0.1 and fusion.finite(a[i])
           and off_at(tt, tv, t[i]) and off_at(bt, bv, t[i])]
    if len(use) < 20:
        return {'drag': Value(NAN, 0.0, 'too few coasting samples (a pedal on throughout?)')}
    # The longest unbroken stretch, fitted in its integral form: v(t) - v(t0) =
    # -drag x (the integral of v) - rolling x (t - t0). The speed (IMU + wheels) is far
    # quieter than its slope, so this beats a line through the acceleration.
    runs, cur = [], [use[0]]
    for i in use[1:]:
        if i == cur[-1] + 1:
            cur.append(i)
        else:
            runs.append(cur)
            cur = [i]
    runs.append(cur)
    idx = max(runs, key=len)
    if len(idx) < 20:
        return {'drag': Value(NAN, 0.0, 'too few unbroken coasting samples')}
    rows, ys, area = [], [], 0.0
    for k in range(1, len(idx)):
        i, j = idx[k - 1], idx[k]
        area += 0.5 * (v[i] + v[j]) * (t[j] - t[i])
        rows.append((-area, -(t[j] - t[idx[0]])))
        ys.append(v[j] - v[idx[0]])
    # The car model's drag alone (it has no rolling resistance): one number, well pinned
    # even by a short coast. The two-term fit beside it says whether rolling resistance
    # matters (its own row; a short coast can't split the two).
    one = an.least_squares([(r[0],) for r in rows], ys)
    two = an.least_squares(rows, ys)
    if one is None:
        return {}
    spread = max(v[i] for i in idx) - min(v[i] for i in idx)
    why = '' if spread > 0.2 else f'the coast covered only {spread:.2f} m/s'
    out = {'drag': Value(one[0], 0.0, why)}
    if two is not None:
        out['rolling'] = Value(two[1], 0.0, why)
    return out


def fit_speed_up(p: Pass, drag: float, deadband: float) -> Dict[str, Value]:
    """Throttle demand -> dead time -> lag -> push -> speed (fused): max_accel and its lag."""
    span = p.phase_span('speed up to ')
    if span is None or p.fused is None:
        return {}
    hold = p.phase_span(pr.HALL_HOLD + ' ')
    if hold is not None:
        span = (span[0], hold[1])     # the plateau too: it pins the gain against the drag
    ct, cs = pedal(p, 'throttle')
    if not ct:
        p.notes.append('no throttle demand (/vehicle/throttle command) and no open-loop '
                       'throttle: no throttle fit')
        return {}
    start = span[0] - 0.5
    keep = [(a, b) for a, b in zip(ct, cs) if start <= a <= span[1]]
    t, v, _ = window(p.fused, start, span[1])
    t, v = decimate(t, v)
    fit = an.fit_speed_response([k[0] for k in keep], [k[1] for k in keep], t, v,
                                max(drag, 0.0), deadband, average=False)
    if fit is None:
        return {'max_accel': Value(NAN, 0.0, 'too little data for the speed fit')}
    out = lag_values('throttle', fit, 'the speed (IMU + wheels)')
    out['max_accel'] = Value(fit.gain, 0.0, 'at the edge of the search' if fit.at_edge else '')
    return out


def throttle_push(p: Pass, est: Mapping[str, float]) -> Tuple[List[float], List[float]]:
    """Return the throttle's push (m/s^2) over time: its level x max_accel, or modelled."""
    gain = est.get('max_accel', NAN)
    if not fusion.finite(gain):
        return [], []
    if reports.has(p.rows, 'throttle'):
        t, lv = signal(p, 'throttle')
        return t, [gain * x for x in lv]
    ct, cs = pedal(p, 'throttle')
    if not ct:
        return [], []
    level = an.fopdt_response(ct, cs, est.get('throttle_lag', 0.1),
                              est.get('throttle_dead_time', 0.0))
    return ct, [gain * x for x in level]


def fit_brake(p: Pass, drag: float, est: Mapping[str, float]) -> Dict[str, Value]:
    """Brake demand -> dead time -> lag -> deceleration (fused, drag and throttle taken off)."""
    span = p.phase_span('brake from ')
    if span is None or p.fused is None:
        return {}
    out: Dict[str, Value] = {}
    t, v, a = window(p.fused, span[0] - 0.5, span[1])
    stop = an.stopping(t, v, span[0])
    if stop is not None:
        out['stop_distance'] = Value(stop.distance)
        out['stop_decel'] = Value(stop.mean_decel)
    bt, bs = pedal(p, 'brake')
    if not bt:
        p.notes.append('no brake demand (/vehicle/brake command): only the stop as a whole')
        return out
    pt, push = throttle_push(p, est)
    end = next((ti for ti, vi in zip(t, v) if ti > span[0] and vi < MOVING), span[1])
    sel = [i for i, ti in enumerate(t) if ti <= end]
    ts = [t[i] for i in sel]
    decel = [-a[i] - max(drag, 0.0) * v[i]
             + (fusion.held_at(pt, push, t[i]) if pt else 0.0) for i in sel]
    decel = [d if fusion.finite(d) else 0.0 for d in decel]
    ts, decel = decimate(ts, decel)
    keep = [(x, y) for x, y in zip(bt, bs) if span[0] - 1.0 <= x <= end]
    if len(ts) < 10 or len(keep) < 2:
        return out
    fit = an.fit_fopdt(ts, [k[1] for k in keep], decel, cmd_t=[k[0] for k in keep],
                       free_gain=True)
    if fit is None:
        return out
    out.update(lag_values('brake', fit, 'the deceleration (IMU + wheels)'))
    out['max_brake'] = Value(fit.gain, 0.0, 'at the edge of the search' if fit.at_edge else '')
    return out


def fit_steer(p: Pass, rate: float) -> Dict[str, Value]:
    """
    Fit the steering on the ground: its angle against ego's (dead time, motor, lag).

    The measured angle where /vehicle/steering has one; else the yaw rate's (the IMU's gz
    and the fused speed: atan(gz L / v)), the real wheels' angle on the ground.
    """
    spans = p.spans('steer ')
    if not spans or p.fused is None:
        return {}
    start, end = spans[0][0] - 0.5, spans[-1][1]
    if reports.has(p.rows, 'steer'):
        mt, ms = signal(p, 'steer')
        how = 'the measured angle'
    else:
        gt, gz = signal(p, 'gz')
        mt, ms = [], []
        for ti, g in zip(gt, gz):
            v = fusion.held_at(p.fused.t, p.fused.speed, ti)
            if fusion.finite(v, g) and v > MOVING:
                mt.append(ti)
                ms.append(math.atan(g * p.ctx.wheelbase / v))
        how = 'the yaw rate (IMU) and speed'
    for name in ('steer_cmd', 'ego_steer', 'cmd_steer'):
        if reports.has(p.rows, name):
            ct, cs = (reports.command_signal(p.rows, name, p.ctx) if name == 'steer_cmd'
                      else signal(p, name) if name in reports.STAMPS
                      else (reports.col(p.rows, 't'), reports.col(p.rows, name)))
            break
    else:
        return {}
    sel = [(a, b) for a, b in zip(mt, ms) if start <= a <= end]
    keep = [(a, b) for a, b in zip(ct, cs) if start - 0.5 <= a <= end]
    if len(sel) < 20 or len(keep) < 2:
        return {}
    ts, vs = decimate([s[0] for s in sel], [s[1] for s in sel])
    fit = an.fit_steer(ts, [k[1] for k in keep], vs, rate, cmd_t=[k[0] for k in keep])
    if fit is None:
        return {}
    return lag_values('steer', fit, how)


# --- the model, replayed -------------------------------------------------------------------


def car_config(current: Mapping[str, float], overrides: Mapping[str, float] = None):
    """Return the car model's config from vehicle.yaml's numbers, and overrides."""
    from aav_description.car_model import ActuatorConfig
    names = set(ActuatorConfig.__dataclass_fields__)
    values = {k.split('.', 1)[1]: float(v) for k, v in current.items()
              if k.startswith('actuators.') and k.split('.', 1)[1] in names}
    v = {'wheelbase': current.get('vehicle.wheelbase'),
         'max_steering': current.get('vehicle.max_steering_angle'),
         'mass': current.get('vehicle.mass')}
    values.update({k: float(x) for k, x in v.items() if x is not None})
    values.update({k: float(x) for k, x in (overrides or {}).items() if k in names})
    return ActuatorConfig(**values)


@dataclass
class Trace:
    t: List[float]
    speed: List[float]
    how: str


def replay(p: Pass, cfg) -> Optional[Trace]:
    """
    Feed the pass's commands through the car model: its speed every DT s.

    The pedal demands (throttle, brake) and ego's steering as logged, from where they
    began to act. Without pedal demands, ego's set speed through the model's speed loop
    (the spine's 'default'): then the loop is part of what's compared.
    """
    from aav_description import car_model as cm
    t_rows = reports.col(p.rows, 't')
    if not t_rows:
        return None
    tt, tv = pedal(p, 'throttle')
    bt, bv = pedal(p, 'brake')
    st, sv = ((reports.command_signal(p.rows, 'steer_cmd', p.ctx))
              if reports.has(p.rows, 'steer_cmd') else ([], []))
    pedals = bool(tt) and bool(bt)
    if not pedals:
        name = 'ego_speed' if reports.has(p.rows, 'ego_speed') else 'cmd_speed'
        et, ev = (signal(p, name) if name in reports.STAMPS
                  else (t_rows, reports.col(p.rows, name)))
        loop = cm.SpeedLoop(cm.speed_loop_profile('default'))
    state = cm.CarState()
    t0, t1 = t_rows[0], t_rows[-1]
    out = Trace([], [], 'pedal demands' if pedals else f'{name} through the default loop')
    n = int((t1 - t0) / DT)
    for k in range(n + 1):
        now = t0 + k * DT

        def at(ts, vs, default=0.0):
            x = fusion.held_at(ts, vs, now) if ts else NAN
            return x if fusion.finite(x) else default

        steer = at(st, sv)
        if pedals:
            cmd = cm.Command(steer=steer, throttle=max(0.0, at(tt, tv)),
                             brake=max(0.0, at(bt, bv)))
            state = cm.step(cfg, state, cmd, DT)
        else:
            state = cm.step(cfg, state, cm.Command(steer=steer, speed=at(et, ev)), DT, loop)
        out.t.append(now)
        out.speed.append(state.v)
    return out


@dataclass(frozen=True)
class Score:
    rms: float           # m/s
    top: float           # m/s: the pass's top measured speed
    stop_model: float    # m: the last stop's distance, the model's (NaN: no stop)
    stop_meas: float     # m: the same, measured

    @property
    def share(self) -> float:
        return self.rms / self.top if self.top > 0 else NAN

    @property
    def stop_error(self) -> float:
        return self.stop_model - self.stop_meas

    @property
    def passed(self) -> bool:
        ok = fusion.finite(self.share) and self.share <= SPEED_TOLERANCE
        if fusion.finite(self.stop_error):
            ok = ok and abs(self.stop_error) <= STOP_TOLERANCE
        return ok


def last_stop(p: Pass) -> Optional[float]:
    """Return the time of the pass's last stop request (a brake or stop phase)."""
    found = None
    for name, rs in reports.phases(p.rows):
        if name.startswith('brake from ') or name == 'stop':
            found = float(rs[0]['t'])
    return found


def stop_distance(t: Sequence[float], v: Sequence[float], start: float) -> float:
    s = an.stopping(list(t), list(v), start)
    return s.distance if s else NAN


def compare(p: Pass, trace: Trace) -> Optional[Score]:
    """Score the model's speed against the measured (fused) one: rms, and the last stop."""
    if trace is None or p.fused is None or not p.fused.t:
        return None
    meas = [fusion.held_at(p.fused.t, p.fused.speed, ti) for ti in trace.t]
    pairs = [(m, s) for m, s in zip(meas, trace.speed)
             if fusion.finite(m, s) and max(abs(m), abs(s)) > fusion.STOPPED]
    if not pairs:
        return None
    rms = math.sqrt(sum((m - s) ** 2 for m, s in pairs) / len(pairs))
    top = max(abs(m) for m, _ in pairs)
    start = last_stop(p)
    if start is None:
        return Score(rms, top, NAN, NAN)
    return Score(rms, top, stop_distance(trace.t, trace.speed, start),
                 stop_distance(p.fused.t, p.fused.speed, start))


# --- the session --------------------------------------------------------------------------


@dataclass
class Session:
    folder: str
    passes: List[Pass]
    info: Dict
    slope: fusion.Slope = None
    estimates: Dict[str, eb.Estimate] = field(default_factory=dict)
    scores: List[Tuple[Pass, Optional[Score], Optional[Score], Trace, Trace]] = field(
        default_factory=list)
    forces: Optional[fusion.Forces] = None
    notes: List[str] = field(default_factory=list)

    @property
    def current(self) -> Mapping[str, float]:
        for p in self.passes:
            return p.ctx.current
        return {}

    def fitted(self) -> Dict[str, float]:
        """Return the car model's numbers that would be pasted (chosen()), by name."""
        return {name: c.value for name, c in chosen(self).items() if c.trusted}

    def truth(self) -> Dict[str, float]:
        """Return the true numbers, in Ravensim: vehicle.yaml with the sim's overrides."""
        t = self.info.get('truth') or {}
        return {k: float(v) for k, v in t.items()}


def find_passes(folder: str) -> List[Pass]:
    """Every pass in a session folder, in order: NN-hall-<kind>-<dir>/<run>/."""
    out = []
    for sub in sorted(os.listdir(folder)):
        path = os.path.join(folder, sub)
        if not os.path.isdir(path):
            continue
        runs = sorted(d for d in os.listdir(path) if os.path.isdir(os.path.join(path, d)))
        for run in runs[-1:]:
            p = load_pass(os.path.join(path, run), os.path.join(sub, run))
            if p is not None:
                out.append(p)
    return out


def analyse(folder: str, info: Optional[Mapping] = None,
            passes: Optional[List[Pass]] = None) -> Session:
    """Work out everything from a session folder (or the given passes)."""
    if info is None:
        info = {}
        path = os.path.join(folder, 'session.yaml')
        if os.path.exists(path):
            import yaml
            with open(path) as f:
                info = yaml.safe_load(f) or {}
    s = Session(folder, passes if passes is not None else find_passes(folder), dict(info))
    if not s.passes:
        s.notes.append('no hallway pass with a log in this folder')
        return s
    for p in s.passes:
        p.still = stand_still(p)
    s.slope = fusion.slope_from([(p.direction, p.still.offset) for p in s.passes
                                 if p.still and p.still.trusted])
    for p in s.passes:
        p.fused = fuse_pass(p, s.slope)
    current = s.current
    est: Dict[str, float] = {}

    def settle(keys):
        for k in keys:
            e = eb.estimate([p.values[k] for p in s.passes if k in p.values],
                            floor=SCALE if k in SCALED else 0.0)
            if e is not None:
                s.estimates[k] = e
                est[k] = e.mean

    for kind in KINDS:
        mine = [p for p in s.passes if p.kind == kind]
        drag = est.get('drag', current.get('actuators.drag', 0.0))
        for p in mine:
            if kind == 'coast':
                p.values = fit_coast(p)
            elif kind == 'speed-up':
                p.values = fit_speed_up(p, drag, current.get('actuators.throttle_deadband', 0.0))
            elif kind == 'brake':
                p.values = fit_brake(p, drag, {**{k: current.get(f'actuators.{k}', NAN)
                                                  for k in ('max_accel', 'throttle_lag')},
                                               **est})
            else:
                p.values = fit_steer(p, current.get('actuators.steer_rate', 0.8))
            problems = p.values.pop('_problems', None)
            if problems and problems.problem:
                p.notes.append(problems.problem)
        settle([k for k, spec in KEYS.items() if spec[3] == kind])
    fitted_cfg = car_config(current, s.fitted())
    current_cfg = car_config(current)
    for p in s.passes:
        a, b = replay(p, fitted_cfg), replay(p, current_cfg)
        s.scores.append((p, compare(p, a), compare(p, b), a, b))
    up = [x for p in s.passes for span in p.spans('speed up to ')
          for x in window(p.fused, *span)[2]]
    down = [x for p in s.passes for span in p.spans('brake from ')
            for x in window(p.fused, *span)[2]]
    mass = s.info.get('mass') or current.get('vehicle.mass', NAN)
    s.forces = fusion.forces(float(mass), up, down, est.get('drag', NAN))
    return s


# --- the files ----------------------------------------------------------------------------


def fmt(x, digits=3) -> str:
    return f'{x:.{digits}f}' if fusion.finite(x) else '-'


@dataclass(frozen=True)
class Choice:
    """A vehicle.yaml number to paste, or not, and why."""

    value: float
    trusted: bool
    note: str


def chosen(s: Session) -> Dict[str, Choice]:
    """
    Return the vehicle.yaml numbers this session gives, by name: trusted, or not and why.

    A dead time the data can't resolve is left out (the model takes 0), and its lag is
    then the whole delay (dead time + lag: what a lag alone fits; the old convention,
    analysis.LagFit.best_tau).
    """
    out: Dict[str, Choice] = {}
    for k, e in s.estimates.items():
        key, unit, digits, _ = KEYS[k]
        if not key.startswith('actuators.') or e is None:
            continue
        name = key.split('.', 1)[1]
        bar = f'± {e.bar:.{digits}f} {unit} (n {e.n})'
        out[name] = Choice(max(0.0, e.mean), e.trusted,
                           f'MEASURED {bar}, hallway TODO 248' if e.trusted
                           else f'NOT TRUSTED: {e.why}')
    for k, prefix in DELAYS.items():
        dead, lag, delay = (out.get(f'{prefix}_dead_time'), out.get(f'{prefix}_lag'),
                            s.estimates.get(k))
        if dead is None or lag is None or dead.trusted or delay is None:
            continue
        if delay.trusted:
            out[f'{prefix}_lag'] = Choice(max(0.0, delay.mean), True,
                                          f'MEASURED ± {delay.bar:.3f} s (n {delay.n}): '
                                          'the whole delay, dead time + lag (the dead time '
                                          "can't be told apart), hallway TODO 248")
        out[f'{prefix}_dead_time'] = replace(dead, note=dead.note + ' (left out: the model '
                                                                    'takes 0)')
    return out


def yaml_lines(s: Session) -> str:
    """vehicle.yaml lines to paste: trusted numbers only; the rest commented, with why."""
    when = s.info.get('started', datetime.date.today().isoformat())
    out = ['# Paste into ros/bringup/aav_description/config/vehicle.yaml, actuators:',
           f'# (car-test --hallway, {when}; check them first: this tool never writes it)',
           'actuators:']
    for name, c in chosen(s).items():
        line = f'  {name}: {c.value:.3f}'
        if c.trusted:
            out.append(line.ljust(34) + f'# {c.note}')
        else:
            out.append(f'# {line.strip()}'.ljust(34) + f'# {c.note}')
    return '\n'.join(out)


def estimate_rows(s: Session):
    for k, e in s.estimates.items():
        key, unit, digits, kind = KEYS[k]
        now = s.current.get(key, NAN) if key else NAN
        yield k, key, unit, digits, kind, e, now


def recovery_rows(s: Session):
    """(name, true, estimate, within): the known numbers against the fitted ones."""
    truth = dict(s.truth())
    for k, prefix in DELAYS.items():
        if f'{prefix}_lag' in truth:
            truth[k] = truth[f'{prefix}_lag'] + truth.get(f'{prefix}_dead_time', 0.0)
    for k, e in s.estimates.items():
        key = KEYS[k][0]
        name = key.split('.', 1)[1] if key.startswith('actuators.') else k
        if name in truth and e is not None:
            yield name, truth[name], e, e.covers(truth[name])


def markdown(s: Session) -> str:
    info = s.info
    out = ['# car-test --hallway: the transfer functions', '']
    if info:
        out += [f"- **When:** {info.get('started', '-')}",
                f"- **Where:** {info.get('where', '-')}",
                f"- **Hallway:** {info.get('length', '-')} m, margin {info.get('margin', '-')}"
                f" m, top speed {info.get('top', '-')} m/s",
                f'- **Passes:** {len(s.passes)}', '']
    # Verdict
    good = sum(1 for _, a, _, _, _ in s.scores if a and a.passed)
    trusted = sum(1 for e in s.estimates.values() if e and e.trusted)
    out += ['## Result', '',
            f'- {trusted} of {len(s.estimates)} numbers trusted (error bars below)',
            f'- the fitted model matches {good} of {len(s.scores)} passes '
            f'(speed within {SPEED_TOLERANCE:.0%}, stop within {STOP_TOLERANCE} m)', '']
    out += ['## Standing still: the IMU and the floor', '',
            '| Pass | Facing | IMU offset (m/s^2) | noise (m/s^2) | still for (s) |',
            '|---|---|---|---|---|']
    for p in s.passes:
        st = p.still
        out.append(f'| {p.name.split("/")[0]} | {p.direction} | '
                   f'{fmt(st.offset) if st else "-"} | {fmt(st.noise) if st else "-"} | '
                   f'{fmt(st.duration, 1) if st else "-"} |')
    out += ['', f'**Slope:** {s.slope.describe() if s.slope else "-"}', '',
            '## The numbers, with error bars', '',
            '| Number | Mean ± sd | n | 90 % of the mean | Resolution | vehicle.yaml now | '
            'Trusted |', '|---|---|---|---|---|---|---|']
    for k, key, unit, digits, kind, e, now in estimate_rows(s):
        out.append(f'| {k} ({kind}) | {e.text(unit, digits)} | {e.n} | '
                   f'{fmt(e.lo, digits)} .. {fmt(e.hi, digits)} | {fmt(e.resolution)} | '
                   f'{fmt(now, digits)} | {"yes" if e.trusted else "NO: " + e.why} |')
    rec = list(recovery_rows(s))
    if rec:
        out += ['', '## Recovery: the known numbers (Ravensim) against the fits', '',
                '| Number | True | Fitted ± sd | Within 2 error bars |', '|---|---|---|---|']
        for name, true, e, ok in rec:
            out.append(f'| {name} | {true:g} | {e.text()} | {"yes" if ok else "NO"} |')
    f = s.forces
    if f:
        out += ['', '## The forces (F = m a)', '',
                f'Mass {fmt(f.mass, 1)} kg (vehicle.yaml).', '',
                '| Force | N | from |', '|---|---|---|',
                f'| hardest push (speeding up) | {fmt(f.drive_peak, 0)} | '
                f'{fmt(f.max_accel)} m/s^2 |',
                f'| hardest stop (braking) | {fmt(f.brake_peak, 0)} | {fmt(f.max_decel)} m/s^2 |',
                f'| drag at 1 m/s | {fmt(f.drag_at_1, 1)} | drag {fmt(f.drag_at_1 / f.mass)} /s |'
                if fusion.finite(f.drag_at_1) else '| drag at 1 m/s | - | |']
    out += ['', '## The model against the car', '',
            f'Each pass replayed through the car model. Passes when the speed is within '
            f'{SPEED_TOLERANCE:.0%} (rms, of its top speed) and the stop within '
            f'{STOP_TOLERANCE} m. Plots: `overlays/`.', '',
            '| Pass | Fitted: speed | stop | ok | vehicle.yaml now: speed | stop | ok |',
            '|---|---|---|---|---|---|---|']
    for p, a, b, _, _ in s.scores:
        def cells(x):
            if x is None:
                return '- | - | -'
            return (f'{x.share:.1%} | {fmt(x.stop_error, 2)} m | '
                    f'{"PASS" if x.passed else "FAIL"}')
        out.append(f'| {p.name.split("/")[0]} | {cells(a)} | {cells(b)} |')
    out += ['', '## Lines for vehicle.yaml', '', 'Trusted numbers only; the rest commented, '
            'with why. Check them; this tool never writes the file.', '', '```yaml',
            yaml_lines(s), '```']
    ran = info.get('passes') or []
    if ran:
        out += ['', '## The passes', '', '| Pass | Test | Facing | Result | What happened |',
                '|---|---|---|---|---|']
        for e in ran:
            out.append(f"| {e.get('pass', '')} | {e.get('test', '')} | {e.get('direction', '')}"
                       f" | **{e.get('status', '')}** | {e.get('detail', '')} |")
    notes = (list(info.get('notes') or []) + s.notes
             + [f'{p.name.split("/")[0]}: {n}' for p in s.passes for n in p.notes])
    if notes:
        out += ['', '## Notes', ''] + [f'- {n}' for n in notes]
    return '\n'.join(out).rstrip() + '\n'


def text_table(s: Session) -> str:
    """Return the numbers for the terminal."""
    lines = ['car-test --hallway', '==================']
    for k, key, unit, digits, kind, e, now in estimate_rows(s):
        lines.append(f'  {k:<19} {e.text(unit, digits):<28} now {fmt(now, digits):<7} '
                     f'{"trusted" if e.trusted else "NOT TRUSTED"}')
    for name, true, e, ok in recovery_rows(s):
        lines.append(f'  recovery {name:<19} true {true:<7g} fitted {e.text():<26} '
                     f'{"ok" if ok else "MISSED"}')
    good = sum(1 for _, a, _, _, _ in s.scores if a and a.passed)
    lines.append(f'  model vs car: the fitted model matches {good} of {len(s.scores)} passes')
    return '\n'.join(lines)


def write(s: Session) -> List[str]:
    """Write report.md, fits.csv, compare.csv, overlays/ and vehicle-yaml.txt; their paths."""
    folder = s.folder
    os.makedirs(os.path.join(folder, 'overlays'), exist_ok=True)
    made = []

    def path(name):
        made.append(os.path.join(folder, name))
        return made[-1]

    with open(path('report.md'), 'w') as f:
        f.write(markdown(s))
    with open(path('vehicle-yaml.txt'), 'w') as f:
        f.write(yaml_lines(s) + '\n')
    with open(path('fits.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['number', 'key', 'unit', 'mean', 'sd', 'n', 'lo90', 'hi90', 'resolution',
                    'now', 'trusted', 'why'] + [p.name.split('/')[0] for p in s.passes])
        for k, key, unit, digits, kind, e, now in estimate_rows(s):
            w.writerow([k, key, unit, reports.number(e.mean), reports.number(e.sd), e.n,
                        reports.number(e.lo), reports.number(e.hi),
                        reports.number(e.resolution), reports.number(now), e.trusted, e.why]
                       + [reports.number(p.values[k].value) if k in p.values else ''
                          for p in s.passes])
    with open(path('compare.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['pass', 'model', 'rms', 'top', 'share', 'stop_model', 'stop_measured',
                    'stop_error', 'passed'])
        for p, a, b, _, _ in s.scores:
            for label, x in (('fitted', a), ('vehicle.yaml', b)):
                if x is not None:
                    w.writerow([p.name.split('/')[0], label, reports.number(x.rms),
                                reports.number(x.top), reports.number(x.share),
                                reports.number(x.stop_model), reports.number(x.stop_meas),
                                reports.number(x.stop_error), x.passed])
    for p, _, _, a, b in s.scores:
        if a is None:
            continue
        with open(path(os.path.join('overlays', p.name.split('/')[0] + '.csv')), 'w',
                  newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'measured', 'wheels', 'model_fitted', 'model_vehicle_yaml'])
            wt, wv = signal(p, 'speed')
            for k, ti in enumerate(a.t):
                w.writerow([f'{ti:.3f}',
                            reports.number(fusion.held_at(p.fused.t, p.fused.speed, ti)),
                            reports.number(fusion.held_at(wt, wv, ti)),
                            reports.number(a.speed[k]),
                            reports.number(b.speed[k] if b and k < len(b.speed) else NAN)])
    png = plot(s)
    if png:
        made.append(png)
    return made


def plot(s: Session) -> Optional[str]:
    """Draw overlays/overlays.png: each pass's speed and both models (None: no matplotlib)."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    shown = [(p, a, b) for p, _, _, a, b in s.scores if a is not None]
    if not shown:
        return None
    cols = 2
    rows = (len(shown) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(11, 2.6 * rows), squeeze=False)
    for ax, (p, a, b) in zip([x for r in axes for x in r], shown):
        ax.plot(p.fused.t, p.fused.speed, color='#222222', lw=1.4, label='measured (IMU + wheels)')
        ax.plot(a.t, a.speed, color='#1f77b4', lw=1.2, label='model, fitted')
        if b is not None:
            ax.plot(b.t, b.speed, color='#d62728', lw=1.0, ls='--',
                    label='model, vehicle.yaml now')
        ax.set_title(p.name.split('/')[0], fontsize=9)
        ax.set_xlabel('s', fontsize=8)
        ax.set_ylabel('m/s', fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.3)
    for ax in [x for r in axes for x in r][len(shown):]:
        ax.axis('off')
    axes[0][0].legend(fontsize=7, loc='upper right')
    fig.tight_layout()
    out = os.path.join(s.folder, 'overlays', 'overlays.png')
    fig.savefig(out, dpi=90)
    plt.close(fig)
    return out


def with_truth(cfg_current: Mapping[str, float], overrides: Mapping[str, float]) -> Dict:
    """Return the true actuator numbers in Ravensim: vehicle.yaml's, with the overrides."""
    out = {k.split('.', 1)[1]: float(v) for k, v in cfg_current.items()
           if k.startswith('actuators.')}
    for k in ('throttle_dead_time', 'brake_dead_time', 'steer_dead_time'):
        out.setdefault(k, 0.0)
    out.update({k: float(v) for k, v in overrides.items()})
    return out


def parse_overrides(text: str) -> Dict[str, float]:
    """'throttle_lag=0.3,drag=0.2' (or spaces) -> {'throttle_lag': 0.3, 'drag': 0.2}."""
    out = {}
    for part in text.replace(',', ' ').split():
        if '=' not in part:
            raise ValueError(f'not name=value: {part!r}')
        k, v = part.split('=', 1)
        out[k.strip()] = float(v)
    return out


def main(argv=None) -> int:
    """`car-test --analyse <session>` (or python -m aav_actuator_tests.hallway <session>)."""
    import sys
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1 or not os.path.isdir(argv[0]):
        print('usage: hallway.py <session folder>')
        return 2
    s = analyse(argv[0])
    if not s.passes:
        print(f'{argv[0]}: no hallway pass with a log here')
        return 1
    write(s)
    print(text_table(s))
    print(f'Saved in {argv[0]} (report.md, vehicle-yaml.txt, overlays/)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
