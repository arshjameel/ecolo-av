"""
From one run's log to a report: a table, and the vehicle.yaml lines to paste.

No ROS here. A run's log is a list of rows (dicts, the CSV's columns below), so a report
can be made again later from the CSV alone: `actuator_test --analyse <run folder>`.
Nothing here writes vehicle.yaml: it only prints the lines, for a person to check.

- Every fit names its plant boundary (PATHS): the signal it goes from and the one it goes
  to, in the table and in each vehicle.yaml line, with its rms.
- The fits read each message at its arrival time (signal(), the t_* columns), not the
  50 Hz rows' sample-and-hold copies.
- shaping() records what shaped the commands on the way (backend, speed loop, ego's
  limits) for meta.yaml; Context carries what the fits need of it.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from aav_actuator_tests import analysis as an
from aav_actuator_tests.analysis import finite, mean, NAN

UNKNOWN = 'unknown'

# The log's columns. Units: s, m/s, rad, rad/s, m/s^2, m; throttle and brake 0..1.
# One row per tick (50 Hz), each value the latest received (sample-and-hold). The t_*
# columns (since 26 Oct) give when that value's message arrived, on the same clock as t,
# so the fits use true times; an older log without them reads as NaN, and the fits fall
# back to the row's t.
COLUMNS = (
    't', 'phase',
    'cmd_speed', 'cmd_steer',             # what this test asked for (before ego)
    'speed',                              # /vehicle/odom: measured (wheel) speed
    'steer_cmd', 'steer',                 # /vehicle/steering: ego's angle, the measured one
    'throttle_cmd', 'throttle',           # /vehicle/throttle
    'brake_cmd', 'brake',                 # /vehicle/brake
    'wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr',   # /vehicle/wheels
    'ax', 'ay', 'gz',                     # IMU: forward and left acceleration, yaw rate
    'heading', 'distance',                # the yaw rate and the speed, added up
    'x', 'y', 'yaw',                      # /localization/odom/local
    'lat', 'lon',                         # /sensors/gps/fix
    'manual_steer',                       # an angle a person typed (stands, no sensor)
    'ego_speed', 'ego_steer',             # /ego/actuator_cmd: what ego told the backend
    't_speed', 't_steer', 't_throttle', 't_brake', 't_wheels', 't_imu', 't_pose', 't_fix',
    't_ego',                              # when each of those messages arrived (s, as t)
)

# Which arrival time goes with which value.
STAMPS = {
    'speed': 't_speed', 'steer_cmd': 't_steer', 'steer': 't_steer',
    'throttle_cmd': 't_throttle', 'throttle': 't_throttle',
    'brake_cmd': 't_brake', 'brake': 't_brake',
    'wheel_fl': 't_wheels', 'wheel_fr': 't_wheels', 'wheel_rl': 't_wheels',
    'wheel_rr': 't_wheels', 'ax': 't_imu', 'ay': 't_imu', 'gz': 't_imu',
    'x': 't_pose', 'y': 't_pose', 'yaw': 't_pose', 'lat': 't_fix', 'lon': 't_fix',
    'ego_speed': 't_ego', 'ego_steer': 't_ego',
}

# Each fit's plant boundary: the signal it goes from, and the one it goes to. A fit is
# only true for the path it measured: ego's ramp and the spine's speed loop sit before
# the throttle demand, so "cmd_vel -> speed" would measure them too.
PATHS = {
    'throttle_lag': 'throttle demand (/vehicle/throttle command) -> throttle level '
                    '(/vehicle/throttle measured)',
    'throttle_plant': 'throttle level (/vehicle/throttle measured) -> wheel speed '
                      '(/vehicle/odom)',
    'throttle_open': "ego's speed / full_throttle_speed (/ego/actuator_cmd: the ESP32's "
                     'open-loop throttle) -> wheel speed (/vehicle/odom)',
    'throttle_map': 'throttle level -> acceleration (d/dt of /vehicle/odom), less drag',
    'throttle_deadband': 'throttle level where the wheel speed first moved',
    'drag': 'coasting speed -> deceleration (d/dt of /vehicle/odom)',
    'brake_map': 'brake level (/vehicle/brake measured) -> deceleration (d/dt of '
                 '/vehicle/odom), less drag',
    'brake_lag': 'brake demand (/vehicle/brake command) -> brake level (/vehicle/brake '
                 'measured)',
    'brake_imu': 'brake demand (/vehicle/brake command) -> deceleration (IMU ax), less drag',
    'stop_imu': "stop asked for (this test's speed -> 0, before ego) -> deceleration "
                '(IMU ax)',
    'steer_rate': "ego's angle (/vehicle/steering command) -> measured angle",
    'steer_lag': "ego's angle (/vehicle/steering command) -> dead time -> motor -> play "
                 '-> lag -> measured angle',
    'steer_sweep': "this test's angle (before ego) -> settled measured angle",
    'yaw': "this test's angle -> the real wheel angle (yaw rate / speed)",
}


@dataclass(frozen=True)
class Context:
    """What the analysis needs to know about the car, and about what shaped the commands."""

    wheelbase: float = 1.57
    max_steering: float = 0.5
    gps_x: float = 0.0                         # m: the antenna ahead of base_link
    current: Mapping[str, float] = field(default_factory=dict)  # vehicle.yaml's values now
    backend: str = UNKNOWN                     # shaping()'s backend: legacy, spine, gazebo...
    full_throttle_speed: Optional[float] = None   # m/s: the ESP32's open loop (legacy only)
    # s: how long before its message's stamp a *_cmd value began to act. Gazebo's adapter
    # sends the demand it just applied for one tick (1/rate_hz) with the result of it.
    command_lead: float = 0.0


@dataclass(frozen=True)
class Row:
    what: str
    value: str
    unit: str = ''
    now: str = ''      # vehicle.yaml's value today, to compare
    note: str = ''
    path: str = ''     # the plant boundary: from -> to (PATHS)


@dataclass
class Report:
    title: str
    rows: List[Row] = field(default_factory=list)
    yaml: Dict[str, Tuple[float, str]] = field(default_factory=dict)  # 'actuators.x' -> ...
    notes: List[str] = field(default_factory=list)

    def add(self, what, value, unit='', key=None, ctx=None, note='', digits=3, path=''):
        now = ''
        if key and ctx is not None and key in ctx.current:
            now = f'{ctx.current[key]:g}'
        self.rows.append(Row(what, fmt(value, digits), unit, now, note, PATHS.get(path, path)))

    def suggest(self, key: str, value: float, comment: str, path: str = '',
                rms: Optional[float] = None, unit: str = ''):
        """Offer a vehicle.yaml line: the comment says which test, which path, what rms."""
        if not finite(value):
            return
        parts = [comment]
        if path:
            parts.append(PATHS.get(path, path))
        if rms is not None and finite(rms):
            parts.append(f'rms {rms:.3g}{" " + unit if unit else ""}')
        self.yaml[key] = (value, '; '.join(parts))


def fmt(value, digits=3) -> str:
    if value is None:
        return '-'
    if isinstance(value, str):
        return value
    if not finite(value):
        return '-'
    return f'{value:.{digits}f}'


def col(rows: Sequence[Mapping], name: str) -> List[float]:
    out = []
    for r in rows:
        v = r.get(name, NAN)
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            out.append(NAN)
    return out


def phases(rows: Sequence[Mapping]) -> List[Tuple[str, List[Mapping]]]:
    """Return the rows cut into runs of the same phase, in order."""
    out: List[Tuple[str, List[Mapping]]] = []
    for r in rows:
        if not out or out[-1][0] != r['phase']:
            out.append((r['phase'], []))
        out[-1][1].append(r)
    return out


def tail(rows: Sequence[Mapping], seconds: float) -> List[Mapping]:
    """Return the rows in the last `seconds` of a phase."""
    if not rows:
        return []
    end = float(rows[-1]['t'])
    return [r for r in rows if float(r['t']) >= end - seconds]


def settled_angle(rows: Sequence[Mapping], seconds: float = 1.0) -> float:
    """Return the angle at the end of a phase: a person's reading, else the sensor's."""
    typed = [v for v in col(rows, 'manual_steer') if finite(v)]
    if typed:
        return typed[-1]
    return mean(col(tail(rows, seconds), 'steer'))


def accel_from_speed(rows: Sequence[Mapping]) -> List[float]:
    return an.derivative(col(rows, 't'), col(rows, 'speed'), 0.3)


def has(rows: Sequence[Mapping], name: str) -> bool:
    return any(finite(v) for v in col(rows, name))


def signal(rows: Sequence[Mapping], name: str) -> Tuple[List[float], List[float]]:
    """
    Return one column as (times, values), one per message, at its true arrival time.

    The rows repeat a value until the next message comes (sample-and-hold); its arrival
    stamp (STAMPS) tells them apart. Without stamps (an older log), the rows' own t.
    """
    t, v = col(rows, 't'), col(rows, name)
    stamps = col(rows, STAMPS[name]) if name in STAMPS else [NAN] * len(rows)
    ts: List[float] = []
    vs: List[float] = []
    last = None
    for ti, vi, si in zip(t, v, stamps):
        if not finite(vi):
            continue
        if finite(si):
            if si == last:
                continue
            last = si
            ti = si
        if ts and ti <= ts[-1]:
            continue
        ts.append(ti)
        vs.append(vi)
    return ts, vs


def command_signal(rows: Sequence[Mapping], name: str,
                   ctx: 'Context') -> Tuple[List[float], List[float]]:
    """Return a *_cmd column as signal() does, each value from when it began to act."""
    ts, vs = signal(rows, name)
    return [x - ctx.command_lead for x in ts], vs


def at_times(rows: Sequence[Mapping], name: str, times: Sequence[float]) -> List[float]:
    """Return a column's values at the given times, read between its messages linearly."""
    ts, vs = signal(rows, name)
    return [an.interpolate(ts, vs, x) for x in times]


def add_lag(rep: 'Report', fit: an.LagFit, what: str, prefix: str, test: str, path: str,
            source: str, ctx: 'Context', unit: str = ''):
    """
    Add a dead-time-and-lag fit: both numbers, the old fit beside them, and the rms.

    Suggests actuators.<prefix>_lag and <prefix>_dead_time only where the data resolves
    them (LagFit.problems); otherwise says why.
    """
    lag_key, dead_key = f'actuators.{prefix}_lag', f'actuators.{prefix}_dead_time'
    rep.add(f'{what} lag (fit, with dead time)', fit.tau, 's', lag_key, ctx,
            note=f'rms {fit.rms:.3g} {unit}'.rstrip(), path=path)
    rep.add(f'{what} dead time (fit)', fit.dead_time, 's', dead_key, ctx,
            note=f'resolution {fit.resolution:.3f} s', path=path)
    rep.add(f'{what} lag alone (the old fit, no dead time)', fit.lag_only, 's',
            note=f'rms {fit.rms_lag_only:.3g} {unit}'.rstrip(), path=path)
    problems = fit.problems(what, source)
    rep.notes += problems
    if fit.lag_problem(what, source):
        return
    rep.suggest(lag_key, fit.best_tau, f'test {test}', path, fit.best_rms, unit)
    if fit.dead_resolved:
        rep.suggest(dead_key, fit.best_dead, f'test {test}', path, fit.best_rms, unit)


# --- stage 3: the steering ---------------------------------------------------------------

def steer_sweep(rows, ctx: Context) -> Report:
    rep = Report('3.10 steering sweep')
    points = []
    for name, rs in phases(rows):
        if name.startswith('sweep '):
            points.append((float(rs[-1]['cmd_steer']), settled_angle(rs)))
    if not any(finite(m) for _, m in points):
        rep.notes.append('no measured angle: /vehicle/steering is silent or NaN. On the car '
                         'today (the 2025-26 ESP32), measure with a protractor: the test asks '
                         'at each step')
        return rep
    for c, m in points:
        rep.add(f'command {c:+.3f} rad', m, 'rad', note=f'{math.degrees(m):+.1f} deg'
                if finite(m) else '')
    line = an.fit_line([c for c, _ in points], [m for _, m in points])
    if line:
        rep.add('gain: measured / command', line.slope, '', note=f'r2 {line.r2:.4f}')
        rep.add('offset at command 0', line.intercept, 'rad', 'actuators.steer_offset', ctx,
                path='steer_sweep')
        rep.suggest('actuators.steer_offset', line.intercept, "test 3.10: the sweep's line",
                    'steer_sweep')
    # The same command, reached from both sides: the gap is the slack (deadband, backlash).
    seen: Dict[float, List[float]] = {}
    for c, m in points:
        seen.setdefault(round(c, 4), []).append(m)
    gaps = [max(v) - min(v) for v in seen.values() if len(v) > 1 and all(map(finite, v))]
    if gaps:
        rep.add('hysteresis (largest gap, same command)', max(gaps), 'rad')
    lefts = [m for c, m in points if c >= ctx.max_steering - 1e-6]
    rights = [m for c, m in points if c <= -ctx.max_steering + 1e-6]
    if lefts and rights:
        lock = min(abs(mean(lefts)), abs(mean(rights)))
        rep.add('full lock, left', mean(lefts), 'rad')
        rep.add('full lock, right', mean(rights), 'rad')
        rep.add('full lock (smaller side)', lock, 'rad', 'vehicle.max_steering_angle', ctx,
                path='steer_sweep')
        rep.suggest('vehicle.max_steering_angle', lock, 'test 3.10: the smaller full lock',
                    'steer_sweep')
    return rep


def steer_rate(rows, ctx: Context) -> Report:
    rep = Report('3.11 steering speed')
    parts = phases(rows)
    swings = []
    for i, (name, rs) in enumerate(parts):
        if not name.startswith('step to '):
            continue
        prev = parts[i - 1][1] if i > 0 else []
        both = list(prev) + list(rs)
        t = col(both, 't')
        cmd = col(both, 'steer_cmd')
        if not any(finite(c) for c in cmd):
            cmd = col(both, 'cmd_steer')
        sw = an.swing(t, cmd, col(both, 'steer'), float(rs[0]['t']))
        if sw is None:
            rep.notes.append(f'{name}: no clean swing in the measured angle')
            continue
        swings.append(sw)
        rep.add(f'{name}: {sw.start:+.2f} -> {sw.target:+.2f} rad', sw.time_to_95, 's',
                note=f'rate {sw.rate:.3f} rad/s')
    if not swings:
        rep.notes.append('no measured angle: time centre to full lock on a video instead')
        return rep
    full = [s.time_to_95 for s in swings if abs(s.start) < 0.05]
    if full:
        rep.add('centre to full lock (95%)', mean(full), 's')
    rate = mean([s.rate for s in swings])
    rep.add('steering speed', rate, 'rad/s', 'actuators.steer_rate', ctx, path='steer_rate')
    rep.suggest('actuators.steer_rate', rate, 'test 3.11: mean of the swings, 40-80%',
                'steer_rate')
    moving = [r for name, rs in parts if name.startswith(('step to ', 'centre')) for r in rs]
    name = 'steer_cmd' if has(moving, 'steer_cmd') else 'cmd_steer'
    cmd_t, cmd = command_signal(moving, name, ctx) if name == 'steer_cmd' else (
        col(moving, 't'), col(moving, name))
    t, measured = signal(moving, 'steer')
    play = an.steer_play(t, an.delayed(t, cmd_t, cmd, 0.0), measured)
    if play is not None:
        rep.add('play (settled from above less from below)', play.width, 'rad',
                note=f'{play.holds} holds; offset {play.offset:+.4f} rad', path='steer_rate')
    width = max(0.0, play.width) if play else 0.0
    fit = an.fit_steer(t, cmd, measured, rate, cmd_t, play=width,
                       offset=play.offset if play else 0.0)
    if fit is not None:
        add_lag(rep, fit, 'steering', 'steer', '3.11', 'steer_lag',
                'the measured steering angle', ctx, 'rad')
    cmd_rate = command_rate(rows)
    if finite(cmd_rate) and cmd_rate < rate * 1.15:
        rep.notes.append(f"ego's command ramped at {cmd_rate:.2f} rad/s, about the measured "
                         "speed: the measurement may be ego's limit, not the motor's")
    return rep


def command_rate(rows) -> float:
    """Return the fastest ego's steering command moved (rad/s)."""
    t, c = col(rows, 't'), col(rows, 'steer_cmd')
    rates = [abs(b - a) / (t2 - t1) for t1, t2, a, b in zip(t, t[1:], c, c[1:])
             if finite(a, b) and t2 - t1 > 1e-3]
    rates = [r for r in rates if r > 1e-3]
    if not rates:
        return NAN
    rates.sort()
    return rates[int(0.9 * (len(rates) - 1))]   # the 90th percentile: ignore one-tick jitter


def steer_centre(rows, ctx: Context) -> Report:
    rep = Report('3.12 centre and deadband')
    parts = phases(rows)
    centre = [rs for name, rs in parts if name == 'centre']
    if not centre:
        rep.notes.append('the centre phase never ran')
        return rep
    offset = settled_angle(centre[0], 2.0)
    if not finite(offset):
        rep.notes.append('no measured angle: measure with a protractor (the test asks)')
        return rep
    rep.add('angle when told straight', offset, 'rad', 'actuators.steer_offset', ctx,
            note=f'{math.degrees(offset):+.2f} deg; passes under 1 deg'
            if abs(math.degrees(offset)) < 1 else f'{math.degrees(offset):+.2f} deg: write '
                                                  'it down', path='steer_sweep')
    rep.suggest('actuators.steer_offset', offset, 'test 3.12', 'steer_sweep')
    estimates = []
    for side in ('left', 'right'):
        base = offset
        prev = 0.0
        found = None
        for name, rs in parts:
            if name == f'centre after {"left" if side == "right" else "none"}':
                base = settled_angle(rs, 1.0)
            if not name.startswith(f'deadband {side} '):
                continue
            c = float(rs[-1]['cmd_steer'])
            m = settled_angle(rs, 0.5)
            if finite(m) and abs(m - base) > 0.5 * abs(c):
                found = (abs(prev), abs(c))
                break
            prev = c
        if found:
            est = 0.5 * (found[0] + found[1])
            estimates.append(est)
            rep.add(f'deadband, {side}', est, 'rad',
                    note=f'moved at {found[1]:.3f}, not at {found[0]:.3f}')
        else:
            rep.notes.append(f'deadband, {side}: the wheels never moved by half a step up to '
                             '0.02 rad')
    if estimates:
        rep.add('deadband', mean(estimates), 'rad', 'actuators.steer_deadband', ctx,
                path='steer_sweep')
        rep.suggest('actuators.steer_deadband', mean(estimates), 'test 3.12: tiny steps',
                    'steer_sweep')
    return rep


# --- stage 4: on the ground ---------------------------------------------------------------

def push_fit(rows, ctx: Context) -> Optional[an.Throttle]:
    """
    Return the throttle's map from speed-ups; None without a throttle level.

    The deadband is the median throttle at which each run first moved. The drag is
    vehicle.yaml's: in a speed-up the drag and the throttle rise together, so one fit
    can't tell them apart. Run coast (4.2) first and paste its drag.
    """
    starts = []
    for _, rs in phases(rows):
        s = an.start_throttle(col(rs, 'throttle'), col(rs, 'speed'))
        if s is not None:
            starts.append(s)
    deadband = sorted(starts)[len(starts) // 2] if starts else 0.0
    a = accel_from_speed(rows)
    th, br, v = col(rows, 'throttle'), col(rows, 'brake'), col(rows, 'speed')
    use = [i for i in range(len(rows))
           if finite(th[i], v[i], a[i]) and not (finite(br[i]) and br[i] > 0.01)]
    return an.fit_throttle([th[i] for i in use], [v[i] for i in use], [a[i] for i in use],
                           drag=ctx.current.get('actuators.drag', 0.0), deadband=deadband)


def speed_up_runs(rows):
    return [(name, rs) for name, rs in phases(rows) if name.startswith('speed up to ')]


def accel_figures(rs) -> Dict[str, float]:
    """Peak and mean acceleration of one speed-up, from the wheel speed and from the IMU."""
    t, v = col(rs, 't'), col(rs, 'speed')
    t0 = t[0]
    target = float(rs[0]['cmd_speed'])
    t90 = an.first_crossing(t, v, 0.9 * target, t0)
    a_wheel = an.moving_average(t, accel_from_speed(rs), 0.5)
    a_imu = an.moving_average(t, col(rs, 'ax'), 0.5)
    end = t90 if t90 is not None else t[-1]
    within = [i for i, ti in enumerate(t) if ti <= end]
    v0 = v[0] if finite(v[0]) else 0.0
    return {
        'target': target,
        't90': (t90 - t0) if t90 is not None else NAN,
        'wheel_peak': max((a_wheel[i] for i in within if finite(a_wheel[i])), default=NAN),
        'wheel_mean': (0.9 * target - v0) / (t90 - t0) if t90 and t90 > t0 else NAN,
        'imu_peak': max((a_imu[i] for i in within if finite(a_imu[i])), default=NAN),
        'imu_mean': mean([col(rs, 'ax')[i] for i in within]),
    }


# m/s: below this the ESP32 brakes instead (legacy_protocol LegacyMapping.stop_speed).
LEGACY_STOP_SPEED = 0.05


def open_loop_throttle(rows, ctx: Context) -> Optional[Tuple[List[float], List[float]]]:
    """
    Return the throttle the 2025-26 ESP32 drove, worked out: ego's speed / full_throttle_speed.

    Only on the legacy backend, with its full_throttle_speed known (meta.yaml's shaping)
    and ego's command logged (the ego_speed column). None otherwise.
    """
    fts = ctx.full_throttle_speed
    if ctx.backend != 'legacy' or not fts or not has(rows, 'ego_speed'):
        return None
    ts, vs = signal(rows, 'ego_speed')
    return ts, [min(1.0, v / fts) if v >= LEGACY_STOP_SPEED else 0.0 for v in vs]


def unbraked(rs):
    """
    Return the first stretch of a run with the brake off (the speed fit has no brake in it).

    A speed-up starts with the hold brake letting go: that part is skipped.
    """
    out = []
    for r in rs:
        b = r.get('brake', NAN)
        if isinstance(b, float) and finite(b) and b > 0.01:
            if out:
                break
            continue
        out.append(r)
    return out


def add_speed_fit(rep: Report, rows, ctx: Context, test: str, deadband: float):
    """
    Fit the throttle -> wheel speed plant (analysis.fit_speed_response); return it or None.

    From the throttle level when the backend reports it; from the ESP32's open-loop
    throttle otherwise. Each fresh wheel speed is compared with the model's mean since the
    last one, so Hall pulses don't fool it; a delay the pulses can't resolve is refused.
    """
    runs = [unbraked(rs) for _, rs in speed_up_runs(rows)]
    if not runs:
        return None
    win = max(runs, key=len)
    if has(win, 'throttle'):
        path, (ct, cs) = 'throttle_plant', signal(win, 'throttle')
    else:
        found = open_loop_throttle(win, ctx)
        if found is None:
            return None
        path, (ct, cs) = 'throttle_open', found
    st, sv = signal(win, 'speed')
    drag = ctx.current.get('actuators.drag', 0.0)
    fit = an.fit_speed_response(ct, cs, st, sv, drag, deadband)
    if fit is None:
        return None
    rep.add('full-throttle acceleration (speed fit)', fit.gain, 'm/s^2',
            'actuators.max_accel', ctx,
            note=f"{an.to_g(fit.gain):.3f} g; rms {fit.best_rms:.3g} m/s; vehicle.yaml's drag "
                 f'{drag:g} /s; {fit.samples} fresh speeds', path=path)
    if path == 'throttle_open':
        add_lag(rep, fit, 'throttle (open loop, through the wheel speed)', 'throttle', test,
                path, 'the wheel speed', ctx, 'm/s')
    else:
        # After the measured level: the drivetrain's own delay, not throttle_lag.
        rep.add('level to wheels: dead time', fit.best_dead, 's',
                note=f'resolution {fit.resolution:.3f} s', path=path)
        rep.add('level to wheels: lag', fit.best_tau, 's', note=f'rms {fit.best_rms:.3g} m/s',
                path=path)
        problems = fit.problems('drivetrain', 'the wheel speed')
        rep.notes += [f'(level to wheels) {p}' for p in problems]
    return fit, path


def add_throttle_fit(rep: Report, rows, ctx: Context, test: str):
    fit = push_fit(rows, ctx)
    open_loop = open_loop_throttle(rows, ctx)
    if fit is None and open_loop is None:
        rep.notes.append('no measured throttle level (/vehicle/throttle silent or NaN): '
                         'max_accel can only be read from the g figures above, at the '
                         'throttle the backend chose. On the 2025-26 ESP32 the speed command '
                         'is the throttle: speed / full_throttle_speed (legacy_protocol.py); '
                         "with the legacy backend's full_throttle_speed and ego's command "
                         'logged (meta.yaml shaping, ego_speed) the speed fit uses that')
        return
    if fit is not None:
        deadband = fit.deadband
        rep.add('full-throttle acceleration (acceleration fit)', fit.gain, 'm/s^2',
                'actuators.max_accel', ctx,
                note=f"{an.to_g(fit.gain):.3f} g; vehicle.yaml's drag {fit.drag:g} /s; "
                     f'{fit.n} samples', path='throttle_map')
        rep.add('throttle deadband (where it first moved)', deadband, '',
                'actuators.throttle_deadband', ctx, path='throttle_deadband')
        rep.suggest('actuators.throttle_deadband', deadband, f'test {test}',
                    'throttle_deadband')
    else:
        deadband = ctx.current.get('actuators.throttle_deadband', 0.0)
    speed = add_speed_fit(rep, rows, ctx, test, deadband)
    if speed is not None and not speed[0].at_edge:
        sfit, path = speed
        # The speed fit reads the wheel speed itself, not its slope: it takes coarse
        # (Hall-pulse) speeds that the acceleration fit can't. Its gain stands even when
        # the delay is too short to resolve.
        rep.suggest('actuators.max_accel', sfit.gain, f'test {test}: the speed fit', path,
                    sfit.best_rms, 'm/s')
    elif fit is not None:
        rep.suggest('actuators.max_accel', fit.gain, f'test {test}', 'throttle_map')
    ct, cs = command_signal(rows, 'throttle_cmd', ctx)
    t, level = signal(rows, 'throttle')
    lag = an.fit_fopdt(t, cs, level, cmd_t=ct) if ct and t else None
    if lag is not None:
        add_lag(rep, lag, 'throttle', 'throttle', test, 'throttle_lag', 'the throttle level',
                ctx)


def speed_up(rows, ctx: Context) -> Report:
    rep = Report('4.1 speed up')
    runs = speed_up_runs(rows)
    if not runs:
        rep.notes.append('the speed-up never ran')
        return rep
    name, rs = runs[-1]
    t, v = col(rs, 't'), col(rs, 'speed')
    target = float(rs[0]['cmd_speed'])
    for s, sec in an.time_to_speeds(t, v, [x for x in (0.5, 1.0, 2.0, 3.0) if x <= target + 1e-6],
                                    t[0]).items():
        rep.add(f'time to {s:g} m/s', sec, 's', note='' if sec is not None else 'not reached')
    f = accel_figures(rs)
    rep.add('acceleration, wheel speed: mean to 90%', an.to_g(f['wheel_mean']), 'g',
            note=f"{f['wheel_mean']:.3f} m/s^2" if finite(f['wheel_mean']) else '')
    rep.add('acceleration, wheel speed: peak (0.5 s)', an.to_g(f['wheel_peak']), 'g',
            note=f"{f['wheel_peak']:.3f} m/s^2" if finite(f['wheel_peak']) else '')
    rep.add('acceleration, IMU: mean to 90%', an.to_g(f['imu_mean']), 'g')
    rep.add('acceleration, IMU: peak (0.5 s)', an.to_g(f['imu_peak']), 'g')
    add_throttle_fit(rep, [r for _, rs in runs for r in rs], ctx, '4.1')
    return rep


def coast(rows, ctx: Context) -> Report:
    rep = Report('4.2 coast')
    runs = [rs for name, rs in phases(rows) if name.startswith('coast from ')]
    if not runs:
        rep.notes.append('the coast never ran')
        return rep
    rs = runs[-1]
    a = an.moving_average(col(rs, 't'), accel_from_speed(rs), 0.5)
    th, br, v = col(rs, 'throttle'), col(rs, 'brake'), col(rs, 'speed')
    free = [i for i in range(len(rs)) if not (finite(th[i]) and th[i] > 0.01)
            and not (finite(br[i]) and br[i] > 0.01) and finite(a[i], v[i])]
    # Skip the first half second: the throttle is still letting go.
    t = col(rs, 't')
    free = [i for i in free if t[i] - t[0] > 0.5]
    line = an.fit_line([v[i] for i in free], [a[i] for i in free])
    rep.add('coasting from', v[0], 'm/s')
    rep.add('time coasted', t[-1] - t[0], 's')
    if line is None or len(free) < 20:
        rep.notes.append('too few coasting samples (throttle or brake on throughout?)')
        return rep
    rep.add('deceleration at the start', an.to_g(-line.at(v[0])), 'g')
    rep.add('drag: m/s lost per s per m/s', -line.slope, '1/s', 'actuators.drag', ctx,
            note=f'r2 {line.r2:.3f}', path='drag')
    rep.add('rolling resistance (constant part)', -line.intercept, 'm/s^2',
            note='the model has none: it should be near 0')
    rep.suggest('actuators.drag', -line.slope, 'test 4.2', 'drag')
    return rep


def brake(rows, ctx: Context) -> Report:
    rep = Report('4.3 brake')
    pts = []
    brake_rows = []
    for name, rs in phases(rows):
        if not name.startswith('brake from '):
            continue
        brake_rows += rs
        t = col(rs, 't')
        stop = an.stopping(t, col(rs, 'speed'), t[0])
        a_imu = an.moving_average(t, col(rs, 'ax'), 0.5)
        peak = -min((x for x in a_imu if finite(x)), default=NAN)
        if stop is None:
            rep.notes.append(f'{name}: never stopped')
            continue
        rep.add(f'from {stop.start_speed:.2f} m/s: distance', stop.distance, 'm',
                note=f'{stop.time:.2f} s; mean {an.to_g(stop.mean_decel):.3f} g, '
                     f'IMU peak {an.to_g(peak):.3f} g')
        pts.append(stop)
    if not pts:
        return rep
    a = accel_from_speed(brake_rows)
    br, th, v = col(brake_rows, 'brake'), col(brake_rows, 'throttle'), col(brake_rows, 'speed')
    use = [i for i in range(len(brake_rows)) if finite(br[i], a[i], v[i]) and br[i] > 0.02
           and v[i] > 0.1 and not (finite(th[i]) and th[i] > 0.01)]
    drag = ctx.current.get('actuators.drag', 0.0)
    if len(use) >= 20:
        line = an.fit_line([br[i] for i in use], [-a[i] - drag * v[i] for i in use])
        if line and line.slope > 0:
            rep.add('full-brake deceleration (fit)', line.slope, 'm/s^2',
                    'actuators.max_brake', ctx,
                    note=f'{an.to_g(line.slope):.3f} g; r2 {line.r2:.3f}; '
                         f'brake level up to {max(br[i] for i in use):.2f}', path='brake_map')
            rep.suggest('actuators.max_brake', line.slope, 'test 4.3', 'brake_map')
        # The whole run: the demand and the level go together whatever the car does.
        ct, cs = command_signal(rows, 'brake_cmd', ctx)
        t, level = signal(rows, 'brake')
        fit = an.fit_fopdt(t, cs, level, cmd_t=ct) if ct else None
        if fit is not None:
            add_lag(rep, fit, 'brake', 'brake', '4.3', 'brake_lag', 'the brake level', ctx)
    elif has(brake_rows, 'ax'):
        brake_from_imu(rep, rows, ctx, drag)
    else:
        rep.notes.append('no measured brake level and no IMU: max_brake only from the '
                         'stopping figures above')
    return rep


def braking_imu(rows, drag: float, current: Mapping[str, float], before: float = 0.0,
                moving: float = 0.2) -> Tuple[List[float], List[float]]:
    """
    Return (times, deceleration) from the IMU during each stop, drag taken off.

    From `before` s ahead of each 'brake from' phase (0: from the stop request) until the
    speed falls under `moving` (the brake can't push a stopped car, so the model ends
    there). Deceleration positive. Where a throttle level is reported, its push
    (vehicle.yaml's max_accel above throttle_deadband) is added back: just after the stop
    request the throttle is still letting go.
    """
    times: List[float] = []
    decel: List[float] = []
    ts, ax = signal(rows, 'ax')
    speed = at_times(rows, 'speed', ts)
    push = [0.0] * len(ts)
    if has(rows, 'throttle'):
        gain = current.get('actuators.max_accel', 0.0)
        deadband = current.get('actuators.throttle_deadband', 0.0)
        push = [gain * x if finite(x) and x > deadband else 0.0
                for x in at_times(rows, 'throttle', ts)]
    for name, rs in phases(rows):
        if not name.startswith('brake from '):
            continue
        start = float(rs[0]['t'])
        for k, (ti, a, v) in enumerate(zip(ts, ax, speed)):
            if ti < start - before or not finite(v) or (times and ti <= times[-1]):
                continue
            if v <= moving:
                if ti > start:
                    break
                continue
            times.append(ti)
            decel.append(-a - drag * v + push[k])
    return times, decel


def brake_from_imu(rep: Report, rows, ctx: Context, drag: float):
    """
    No brake level: the deceleration from the IMU alone.

    With the brake demand known (/vehicle/brake command), fit demand -> deceleration: its
    gain is max_brake, with the brake's lag and dead time. Without it, only the stop as a
    whole: from the moment this test asked to stop to the deceleration (no vehicle.yaml
    line: it includes ego's ramp and the backend's own stopping rule).
    """
    t, decel = braking_imu(rows, drag, ctx.current)
    if len(t) < 20:
        rep.notes.append('no measured brake level, and too few IMU samples while braking')
        return
    if not has(rows, 'throttle'):
        rep.notes.append('no throttle level either: just after the stop request the throttle '
                         'still lets go, unseen, and pushes against the brake (a few % on '
                         'max_brake in the pretend car)')
    ct, cs = command_signal(rows, 'brake_cmd', ctx)
    if ct and max(cs) - min(cs) > 1e-3:
        fit = an.fit_fopdt(t, cs, decel, cmd_t=ct, free_gain=True)
        if fit is None:
            return
        rep.add('full-brake deceleration (IMU fit)', fit.gain, 'm/s^2', 'actuators.max_brake',
                ctx, note=f'{an.to_g(fit.gain):.3f} g; offset {fit.offset:+.3f} m/s^2; '
                          f'rms {fit.best_rms:.3g} m/s^2', path='brake_imu')
        rep.suggest('actuators.max_brake', fit.gain, 'test 4.3: no brake level', 'brake_imu',
                    fit.best_rms, 'm/s^2')
        add_lag(rep, fit, 'brake', 'brake', '4.3', 'brake_imu', 'the IMU', ctx, 'm/s^2')
        return
    # Only when the stop was asked for: 1 inside each brake phase, 0 outside.
    rt = col(rows, 't')
    asked = [1.0 if str(r['phase']).startswith('brake from ') else 0.0 for r in rows]
    fit = an.fit_fopdt(t, asked, decel, cmd_t=rt, free_gain=True, max_dead=1.0, max_tau=2.0)
    if fit is None:
        return
    rep.add('stop: deceleration it settled to', fit.gain, 'm/s^2',
            note=f'rms {fit.best_rms:.3g} m/s^2', path='stop_imu')
    rep.add('stop: dead time', fit.best_dead, 's', note=f'resolution {fit.resolution:.3f} s',
            path='stop_imu')
    rep.add('stop: lag', fit.best_tau, 's', path='stop_imu')
    rep.notes += fit.problems('stop', 'the IMU')
    rep.notes.append('no brake level or demand: the stop is timed as a whole (ego, the '
                     "backend's stopping rule, the brake). No max_brake or brake_lag: the "
                     'backend must publish the brake demand it sends (/vehicle/brake '
                     'command)')


def throttle_map(rows, ctx: Context) -> Report:
    rep = Report('4.4 throttle map')
    runs = speed_up_runs(rows)
    for name, rs in runs:
        f = accel_figures(rs)
        rep.add(f"to {f['target']:g} m/s: peak acceleration", an.to_g(f['wheel_peak']), 'g',
                note=f"IMU {an.to_g(f['imu_peak']):.3f} g; 90% in {fmt(f['t90'], 2)} s")
    every = [r for _, rs in runs for r in rs]
    if has(every, 'throttle'):
        a = accel_from_speed(every)
        th, v = col(every, 'throttle'), col(every, 'speed')
        drag = ctx.current.get('actuators.drag', 0.0)
        bins: Dict[int, List[float]] = {}
        for i in range(len(every)):
            if finite(th[i], a[i], v[i]) and th[i] > 0.0:
                bins.setdefault(min(9, int(th[i] * 10)), []).append(a[i] + drag * v[i])
        for b in sorted(bins):
            if len(bins[b]) >= 5:
                rep.add(f'throttle {b * 10}-{b * 10 + 10}%', mean(bins[b]), 'm/s^2',
                        note=f'{an.to_g(mean(bins[b])):.3f} g, drag added back; '
                             f'{len(bins[b])} samples')
    add_throttle_fit(rep, every, ctx, '4.4')
    return rep


def circles(rows, ctx: Context) -> Report:
    rep = Report('4.5 circles')
    pts = []
    for name, rs in phases(rows):
        if not name.startswith('circle '):
            continue
        c = float(rs[-1]['cmd_steer'])
        v, gz = mean(col(rs, 'speed')), mean(col(rs, 'gz'))
        from_yaw = an.wheel_angle(gz, v, ctx.wheelbase)
        circle = an.fit_circle(col(rs, 'x'), col(rs, 'y'))
        from_circle = an.angle_for_radius(circle.radius, ctx.wheelbase) if circle else NAN
        gps = gps_radius(rs, ctx)
        measured = mean(col(rs, 'steer'))
        diameter = 2 * circle.radius if circle else NAN
        rep.add(f'command {c:+.3f} rad: from yaw rate', from_yaw, 'rad',
                note=f'circle {fmt(from_circle)} rad (diameter {fmt(diameter, 2)} m'
                     + (f', GPS {gps * 2:.2f} m' if finite(gps) else '')
                     + f'); sensor {fmt(measured)}; {v:.2f} m/s')
        pts.append((c, from_yaw, from_circle))
    if not pts:
        rep.notes.append('no circle finished')
        return rep
    line = an.fit_line([p[0] for p in pts], [p[1] for p in pts])
    if line:
        rep.add('gain: real angle / command', line.slope, '', note=f'r2 {line.r2:.4f}')
        rep.add('real angle at command 0', line.intercept, 'rad', 'actuators.steer_offset', ctx,
                path='yaw')
    full = [abs(p[1]) for p in pts if abs(p[0]) >= ctx.max_steering - 1e-6 and finite(p[1])]
    if full:
        lock = min(full)
        rep.add('full lock, real (smaller side)', lock, 'rad', 'vehicle.max_steering_angle', ctx,
                note=f'tightest circle: {2 * ctx.wheelbase / math.tan(lock):.2f} m across '
                     '(rear axle)', path='yaw')
        rep.suggest('vehicle.max_steering_angle', lock, 'test 4.5: the full-lock circles',
                    'yaw')
    return rep


def gps_radius(rs, ctx: Context) -> float:
    """Return the circle's radius from GPS, moved from the antenna to the rear axle (m)."""
    lat, lon = col(rs, 'lat'), col(rs, 'lon')
    pairs = [(a, b) for a, b in zip(lat, lon) if finite(a, b)]
    if len(pairs) < 10:
        return NAN
    lat0, lon0 = pairs[0]
    xy = [an.gps_to_xy(a, b, lat0, lon0) for a, b in pairs]
    circle = an.fit_circle([p[0] for p in xy], [p[1] for p in xy])
    if circle is None or circle.radius <= abs(ctx.gps_x):
        return NAN
    return math.sqrt(circle.radius ** 2 - ctx.gps_x ** 2)


def straight(rows, ctx: Context) -> Report:
    rep = Report('4.7 straight line')
    runs = [rs for name, rs in phases(rows) if name == 'straight']
    if not runs:
        rep.notes.append('the straight never ran')
        return rep
    rs = runs[-1]
    d = an.drift(col(rs, 'x'), col(rs, 'y'), col(rs, 'yaw'))
    if d is None:
        rep.notes.append('no pose from /localization/odom/local')
        return rep
    rep.add('driven along the start heading', d.along, 'm')
    rep.add('sideways drift (+ left)', d.across, 'm',
            note='passes: under 0.5 m over 10 m' if abs(d.across) < 0.5 * d.along / 10.0
            else 'FAILS: 0.5 m over 10 m')
    rep.add('heading change', math.degrees(d.heading), 'deg')
    angle = an.wheel_angle(mean(col(rs, 'gz')), mean(col(rs, 'speed')), ctx.wheelbase)
    rep.add('real wheel angle (yaw rate / speed)', angle, 'rad', 'actuators.steer_offset', ctx,
            path='yaw')
    rep.suggest('actuators.steer_offset', angle, 'test 4.7: on a straight', 'yaw')
    return rep


ANALYSES = {
    'steer-sweep': steer_sweep, 'steer-rate': steer_rate, 'steer-centre': steer_centre,
    'speed-up': speed_up, 'coast': coast, 'brake': brake, 'throttle-map': throttle_map,
    'circles': circles, 'straight': straight,
}


def analyse(test: str, rows: Sequence[Mapping], ctx: Context) -> Report:
    return ANALYSES[test](rows, ctx)


def table(rep: Report) -> str:
    """Return the report as a plain-text table; each fit's path (from -> to) below it."""
    paths: List[str] = []
    for r in rep.rows:
        if r.path and r.path not in paths:
            paths.append(r.path)
    head = ('What', 'Measured', 'Unit', 'vehicle.yaml now', 'Path', 'Note')
    body = [(r.what, r.value, r.unit, r.now,
             f'[{paths.index(r.path) + 1}]' if r.path else '', r.note) for r in rep.rows]
    widths = [max(len(str(x)) for x in column) for column in zip(head, *body)] if body else []
    lines = [rep.title, '=' * len(rep.title)]
    if body:
        lines.append('  '.join(h.ljust(w) for h, w in zip(head, widths)).rstrip())
        lines.append('  '.join('-' * w for w in widths))
        lines += ['  '.join(str(x).ljust(w) for x, w in zip(row, widths)).rstrip()
                  for row in body]
    lines += [f'PATH [{i}]: {p}' for i, p in enumerate(paths, 1)]
    lines += [f'NOTE: {n}' for n in rep.notes]
    return '\n'.join(lines)


def yaml_lines(rep: Report) -> str:
    """Return the vehicle.yaml lines to paste, by section. This tool never writes them."""
    if not rep.yaml:
        return ''
    out = ['# Paste into ros/bringup/aav_description/config/vehicle.yaml (check them first):']
    for section in ('vehicle', 'actuators'):
        keys = sorted(k for k in rep.yaml if k.startswith(section + '.'))
        if not keys:
            continue
        out.append(f'{section}:')
        for k in keys:
            value, comment = rep.yaml[k]
            out.append(f'  {k.split(".", 1)[1]}: {value:.4g}'.ljust(34) + f'# measured: {comment}')
    return '\n'.join(out)


def current_values(cfg: Mapping) -> Dict[str, float]:
    """vehicle.yaml's numbers as 'section.key' -> value, to compare against."""
    out = {}
    for section in ('vehicle', 'actuators'):
        for k, v in (cfg.get(section) or {}).items():
            if isinstance(v, (int, float)):
                out[f'{section}.{k}'] = float(v)
    return out


def context(cfg: Mapping) -> Context:
    v = cfg.get('vehicle', {})
    gps = ((cfg.get('sensors') or {}).get('gps_link') or {}).get('xyz', [0.0])
    return Context(wheelbase=float(v.get('wheelbase', 1.57)),
                   max_steering=float(v.get('max_steering_angle', 0.5)),
                   gps_x=float(gps[0]), current=current_values(cfg))


# --- What shaped the commands (meta.yaml's shaping) --------------------------------------

# The nodes whose parameters shape what reaches the actuators, and those parameters.
SHAPING_PARAMS = {
    'legacy_uart_bridge': ('full_throttle_speed', 'stop_speed', 'hold_brake', 'frame_version',
                           'uplink', 'rate_hz', 'speed_loop.mode'),
    'gazebo_adapter': ('speed_loop', 'spine', 'actuator_model', 'rate_hz'),
    'spine_sim': ('speed_loop',),
    'spine_bridge': ('role', 'drive_rate_hz', 'cmd_timeout'),
}
# ego's limits: its ramp sits between this test's command and the backend.
EGO_LIMITS = ('max_speed', 'max_acceleration', 'max_deceleration', 'max_steering_rate',
              'max_steering_angle')


def shaping(announcers: Sequence[str], params: Mapping[str, Mapping[str, object]],
            ego: Mapping[str, object], ego_changed: Optional[Mapping[str, object]] = None
            ) -> Dict[str, object]:
    """
    Return what shaped the commands on their way to the actuators, for meta.yaml.

    announcers: the /system/source announcers heard. params: SHAPING_PARAMS' values read
    from each node that answered. ego: ego's limits ({} if it didn't answer). What can't
    be known from ROS says so (UNKNOWN, or why).
    """
    names = set(announcers)
    legacy = params.get('legacy_uart_bridge') or {}
    gazebo = params.get('gazebo_adapter') or {}
    sim = params.get('spine_sim') or {}
    lead = 0.0
    fts: object = 'n/a'
    if 'legacy_uart_bridge' in names or legacy:
        backend, firmware = 'legacy', 'the 2025-26 ESP32, over UART (legacy_uart_bridge)'
        loop = 'none: open loop, throttle = speed / full_throttle_speed'
        fts = legacy.get('full_throttle_speed', UNKNOWN)
    elif 'gazebo_adapter' in names or gazebo:
        lead = 1.0 / float(gazebo.get('rate_hz') or 50.0)
        if gazebo.get('spine'):
            backend, firmware = 'gazebo-spine', "spine_sim (spine-core's Rust) in Gazebo"
            loop = sim.get('speed_loop', gazebo.get('speed_loop', UNKNOWN))
        elif gazebo.get('actuator_model') is False:
            backend, firmware = 'gazebo-ideal', 'Gazebo, no actuator model'
            loop = 'none: Gazebo follows the speed within its limits'
        else:
            backend, firmware = 'gazebo', "Gazebo's actuator model (actuator_model.py)"
            loop = gazebo.get('speed_loop', UNKNOWN)
    elif 'spine_sim' in names or sim:
        backend, firmware = 'bench', "spine_sim (spine-core's Rust), bench mode"
        loop = sim.get('speed_loop', UNKNOWN)
    elif 'spine_bridge' in names:
        backend, firmware = 'spine', 'the spine, over CAN (spine_bridge)'
        loop = "unknown: built into the spine's firmware (SpineConfig); write it down"
    else:
        backend, firmware, loop = UNKNOWN, UNKNOWN, UNKNOWN
    return {
        'backend': backend,
        'firmware': firmware,
        'speed_loop': loop,
        'full_throttle_speed': fts,
        'command_lead': lead,
        'ego': {k: ego.get(k, UNKNOWN) for k in EGO_LIMITS} if ego
        else 'unknown: ego did not answer',
        'ego_changed': dict(ego_changed or {}),
        'announcers': sorted(names),
        'params': {k: dict(v) for k, v in params.items()},
    }


def with_shaping(ctx: Context, shape: Optional[Mapping]) -> Context:
    """Return the context with shaping()'s backend, full_throttle_speed and command lead."""
    if not shape:
        return ctx
    fts = shape.get('full_throttle_speed')
    return replace(ctx, backend=str(shape.get('backend', UNKNOWN)),
                   full_throttle_speed=float(fts) if isinstance(fts, (int, float)) else None,
                   command_lead=float(shape.get('command_lead') or 0.0))


def context_from_meta(meta: Mapping) -> Context:
    """Return the context a run's meta.yaml recorded (older runs: no shaping)."""
    ctx = Context(wheelbase=meta['wheelbase'], max_steering=meta['max_steering'],
                  gps_x=meta.get('gps_x', 0.0), current=meta.get('vehicle_yaml', {}))
    return with_shaping(ctx, meta.get('shaping'))


def parse_rows(records: Sequence[Mapping[str, str]]) -> List[Dict]:
    """CSV records (all strings) back to rows: numbers as floats, the phase as text."""
    out = []
    for rec in records:
        row: Dict = {}
        for k in COLUMNS:
            v = rec.get(k, '')
            if k == 'phase':
                row[k] = v
            else:
                try:
                    row[k] = float(v) if v != '' else NAN
                except ValueError:
                    row[k] = NAN
        out.append(row)
    return out


def number(value: Optional[float]) -> str:
    """Return a CSV cell: blank for unknown."""
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return ''
    return f'{value:.6g}'
