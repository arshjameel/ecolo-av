"""
The preflight on stands: its sensor checks, its live IMU watch, and the PASS / FAIL lines.

No ROS here. The steps are procedures.preflight (an actuator test, `actuator-test
preflight --stands`): speed up, hold, slow down, brake; steer left, right, centre. While
they run every sensor is counted; afterwards one line per check, as car-day-check prints
(docs/designs/2026-10-29_preflight.md).

| Part | Where |
|---|---|
| The sensors and their rates | SENSORS, Sensor.min_rate, Samples |
| One sensor's rate and values | sensor_line() |
| Speed up, hold, slow down, brake, steer | motion_lines() |
| Stands: the wheels spin, the IMU and GPS say still | stands_lines() |
| Off the stands while it runs (live) | ImuWatch |
| The printout | table(), passed() |

On stands the wheels and the rest disagree on purpose (docs/learn-speed.md section 9).
In Gazebo and on the bench the car is on the ground, so the same maths checks the
opposite: there the IMU and the GPS must agree with the wheels.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import enum
import math
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from aav_actuator_tests import procedures as pr
from aav_actuator_tests.analysis import STOPPED

G = 9.80665              # m/s^2
GRAVITY_TOLERANCE = 0.5  # m/s^2: |a| at rest within this of G
STILL_GYRO = 0.1         # rad/s: the most a still IMU turns
RATE_MARGIN = 0.8        # pass at this x a sensor's documented rate
REACH = 0.8              # of the cap: "it sped up"
HOLD_TOLERANCE = 0.25    # of the cap: the hold's mean speed
BRAKE_TIME = 5.0         # s: still within this after the brake step starts
IMU_WINDOW = 3.0         # s at most: the IMU's speed change, from the command to 95% speed
IMU_STANDS = 0.4         # on stands: the IMU's speed change under this x the wheels'
IMU_GROUND = 0.6         # on the ground: at least this x the wheels'
DOPPLER_STILL = 0.1      # m/s: on stands the GPS's own speed stays under this
GPS_STANDS = 2.5         # m: without Doppler, on stands the fixes move less (a WARN only)
GPS_GROUND = 0.5         # on the ground: the fixes move at least this x the wheels' distance
MIN_SPIN = 1.5           # m the wheels must turn for the GPS check to mean anything
WATCH_WINDOW = 2.0       # s: the live IMU watch adds up this much acceleration
WATCH_SPEED = 0.15       # m/s: more than this from the IMU on stands: off the stands
WATCH_YAW = 0.1          # rad/s: more than this while steering on stands: off the stands
EARTH_RADIUS = 6371000.0


class Status(enum.Enum):
    PASS = 'PASS'
    FAIL = 'FAIL'
    SKIP = 'SKIP'
    WARN = 'WARN'


@dataclass(frozen=True)
class Line:
    status: Status
    check: str
    detail: str


STILL, SETTLE = pr.PF_STILL, pr.PF_SETTLE
SPEED_UP, HOLD, SLOW, BRAKE = pr.PF_SPEED_UP, pr.PF_HOLD, pr.PF_SLOW, pr.PF_BRAKE
LEFT, RIGHT, CENTRE = pr.PF_LEFT, pr.PF_RIGHT, pr.PF_CENTRE
DRIVING = (SPEED_UP, HOLD, SLOW, BRAKE)


# --- the sensors ---------------------------------------------------------------------


@dataclass(frozen=True)
class Sensor:
    name: str          # for --skip
    topic: str
    kind: str          # how its values are checked: value_problems()
    car_rate: float    # Hz on the car, documented (source)
    standin_rate: float  # Hz in Gazebo and on the bench
    source: str        # where the car's rate is written down

    def min_rate(self, standin: bool) -> float:
        return RATE_MARGIN * (self.standin_rate if standin else self.car_rate)


SENSORS: Tuple[Sensor, ...] = (
    Sensor('odom', '/vehicle/odom', 'odom', 100.0, 50.0,
           'aav.dbc 0x200/0x201 at 100 Hz (learn-sensors.md)'),
    Sensor('wheels', '/vehicle/wheels', 'wheels', 100.0, 50.0,
           'aav.dbc 0x200/0x201 at 100 Hz (learn-sensors.md)'),
    Sensor('steering', '/vehicle/steering', 'steering', 50.0, 50.0,
           'the uplink, 50 Hz (legacy_uart_bridge; needs H9)'),
    Sensor('imu', '/sensors/imu/data_raw', 'imu_raw', 200.0, 50.0,
           'Yahboom, 200 Hz set on the device (learn-sensors.md)'),
    Sensor('imu_filtered', '/sensors/imu/data', 'imu', 200.0, 50.0,
           'the raw IMU through Madgwick, one for one'),
    Sensor('gps', '/sensors/gps/fix', 'gps', 1.0, 5.0,
           'ZED-F9R, 1 Hz from the factory (learn-sensors.md)'),
    Sensor('lidar', '/sensors/lidar/points', 'lidar', 10.0, 10.0,
           'VLP-16, 10 Hz (interfaces.yaml)'),
    Sensor('radar', '/sensors/radar/points', 'radar', 10.0, 10.0,
           'TI mmWave, 10 Hz (aav_radar_ti cfg/base.cfg)'),
    Sensor('camera', '/sensors/camera/front_left/image_raw', 'image', 15.0, 10.0,
           'front camera, 15 Hz (sensor_watchdog.yaml, bandwidth.md)'),
    Sensor('battery', '/vehicle/battery', 'battery', 1.0, 1.0,
           'Victron VE.Direct, about 1 Hz (learn-sensors.md: 1 Hz bench)'),
)
NAMES = tuple(s.name for s in SENSORS)


@dataclass
class Samples:
    """One sensor's messages: arrival time (s, the node's clock) and its plain values."""

    times: List[float] = field(default_factory=list)
    values: List[Mapping[str, float]] = field(default_factory=list)

    def add(self, t: float, values: Mapping[str, float]) -> None:
        self.times.append(t)
        self.values.append(dict(values))


def finite(*xs) -> bool:
    return all(isinstance(x, (int, float)) and math.isfinite(x) for x in xs)


def inside(t: float, windows: Sequence[Tuple[float, float]]) -> bool:
    return any(a <= t <= b for a, b in windows)


def value_problems(sensor: Sensor, samples: Samples, still: Sequence[Tuple[float, float]],
                   lock: float) -> Tuple[List[str], List[str]]:
    """Return (problems, notes) about one sensor's values. Problems fail the line."""
    k, vals = sensor.kind, samples.values
    bad: List[str] = []
    notes: List[str] = []
    if k == 'odom':
        if not all(finite(v['speed']) for v in vals):
            bad.append('a speed that is not a number')
    elif k == 'wheels':
        if not any(finite(v[w]) for v in vals for w in ('fl', 'fr', 'rl', 'rr')):
            bad.append('no wheel measured (all NaN)')
    elif k == 'steering':
        measured = [v['measured'] for v in vals if finite(v['measured'])]
        if not measured:
            notes.append('no measured angle (NaN): the person checks the wheels')
        elif max(abs(m) for m in measured) > 1.2 * lock:
            bad.append(f'angle {max(abs(m) for m in measured):.2f} rad beyond full lock '
                       f'{lock:.2f}')
    elif k == 'imu_raw':
        bad, notes = imu_problems(samples, still)
    elif k == 'imu':
        if not all(finite(v['gz']) for v in vals):
            bad.append('a turn rate that is not a number')
    elif k == 'gps':
        fixed = [v for v in vals if v['status'] >= 0 and finite(v['lat'], v['lon'])]
        if not fixed:
            bad.append('no fix')
        elif len(fixed) < len(vals):
            notes.append(f'{len(vals) - len(fixed)} of {len(vals)} without a fix')
    elif k in ('lidar', 'radar'):
        points = [v['points'] for v in vals]
        if k == 'lidar' and not any(p > 0 for p in points):
            bad.append('no points in any cloud')
        notes.append(f'up to {int(max(points))} points')
    elif k == 'image':
        if not all(v['width'] > 0 and v['height'] > 0 for v in vals):
            bad.append('an empty picture')
        else:
            notes.append(f"{int(vals[-1]['width'])}x{int(vals[-1]['height'])}")
    elif k == 'battery':
        volts = [v['voltage'] for v in vals if finite(v['voltage']) and v['voltage'] > 0]
        if not volts:
            bad.append('no voltage')
        else:
            notes.append(f'{volts[-1]:.1f} V')
    return bad, notes


def imu_problems(samples: Samples, still: Sequence[Tuple[float, float]]
                 ) -> Tuple[List[str], List[str]]:
    keys = ('ax', 'ay', 'az', 'gx', 'gy', 'gz')
    vals = samples.values
    if not all(finite(*(v[x] for x in keys)) for v in vals):
        return ['a value that is not a number'], []
    first = tuple(vals[0][x] for x in keys)
    if len(vals) > 3 and all(tuple(v[x] for x in keys) == first for v in vals):
        return ['frozen: every message the same'], []
    rest = [v for t, v in zip(samples.times, vals) if inside(t, still)]
    if not rest:
        return [], ['no still moment to check gravity']
    g = sum(math.sqrt(v['ax'] ** 2 + v['ay'] ** 2 + v['az'] ** 2) for v in rest) / len(rest)
    turn = sum(math.sqrt(v['gx'] ** 2 + v['gy'] ** 2 + v['gz'] ** 2) for v in rest) / len(rest)
    bad = []
    if abs(g - G) > GRAVITY_TOLERANCE:
        bad.append(f'|a| {g:.2f} m/s^2 at rest, not {G:.2f} +- {GRAVITY_TOLERANCE}')
    if turn > STILL_GYRO:
        bad.append(f'turning {turn:.2f} rad/s at rest')
    return bad, [f'|a| {g:.2f} m/s^2 at rest']


def sensor_line(sensor: Sensor, samples: Optional[Samples], start: float, end: float,
                still: Sequence[Tuple[float, float]], lock: float, standin: bool,
                skip: Iterable[str] = ()) -> Line:
    """One sensor's line: its rate over the run (start to end, s), its longest gap, values."""
    label = f'{sensor.name} ({sensor.topic})'
    if sensor.name in set(skip):
        return Line(Status.SKIP, label, 'skipped (--skip)')
    kept = Samples()
    for t, v in zip(samples.times if samples else [], samples.values if samples else []):
        if start <= t <= end:
            kept.add(t, v)
    if not kept.times:
        return Line(Status.FAIL, label, 'nothing in the run: is it started? Meant to be '
                                        f'off: --skip {sensor.name}')
    rate = len(kept.times) / max(end - start, 1e-6)
    edges = [start] + sorted(kept.times) + [end]
    gap = max(b - a for a, b in zip(edges, edges[1:]))
    need = sensor.min_rate(standin)
    max_gap = max(1.0, 5.0 / need)
    bad, notes = value_problems(sensor, kept, still, lock)
    if rate < need:
        bad.insert(0, f'{rate:.1f} Hz, below {need:g} Hz (0.8 x '
                      f'{sensor.standin_rate if standin else sensor.car_rate:g}'
                      f'{"" if standin else ": " + sensor.source})')
    if gap > max_gap:
        bad.insert(0, f'silent for {gap:.1f} s (most {max_gap:.1f} s)')
    detail = '; '.join([f'{rate:.1f} Hz'] + bad + notes)
    return Line(Status.FAIL if bad else Status.PASS, label, detail)


def sensor_lines(samples: Mapping[str, Samples], start: float, end: float,
                 still: Sequence[Tuple[float, float]], lock: float, standin: bool,
                 skip: Iterable[str] = ()) -> List[Line]:
    skip = set(skip)
    return [sensor_line(s, samples.get(s.name), start, end, still, lock, standin, skip)
            for s in SENSORS]


# --- the drive -----------------------------------------------------------------------


def phase_rows(rows: Sequence[Mapping], phase: str) -> List[Mapping]:
    return [r for r in rows if r['phase'] == phase]


def windows(rows: Sequence[Mapping], phases: Sequence[str]) -> List[Tuple[float, float]]:
    """Return the (start, end) times of each stretch of rows in these phases."""
    out: List[Tuple[float, float]] = []
    for r in rows:
        if r['phase'] not in phases:
            continue
        if out and r['t'] - out[-1][1] < 0.2:
            out[-1] = (out[-1][0], r['t'])
        else:
            out.append((r['t'], r['t']))
    return out


def still_windows(rows: Sequence[Mapping]) -> List[Tuple[float, float]]:
    """When the car stood still: the waits and the stop, while the speed was about 0."""
    out = []
    for a, b in windows(rows, (STILL, SETTLE, 'stop')):
        calm = [r['t'] for r in rows if a <= r['t'] <= b and finite(r['speed'])
                and abs(r['speed']) < STOPPED]
        if calm:
            out.append((min(calm), max(calm)))
    return out


def mean(xs: Sequence[float]) -> float:
    xs = [x for x in xs if finite(x)]
    return sum(xs) / len(xs) if xs else float('nan')


def tail(rows: Sequence[Mapping], seconds: float) -> List[Mapping]:
    if not rows:
        return []
    end = rows[-1]['t']
    return [r for r in rows if r['t'] >= end - seconds]


def motion_lines(rows: Sequence[Mapping], cap: float, lock: float, centre: float = 0.0,
                 centre_tolerance: float = 0.03, open_loop: bool = False,
                 steering_seen: Optional[bool] = None) -> List[Line]:
    """
    Check the car did what it was told: speed up, hold, slow down, brake, steer.

    centre, centre_tolerance: where straight is and how near (rad; vehicle.yaml's
    steer_offset and steer_deadband). open_loop: the 2025-26 ESP32 sets the throttle from
    the speed asked, with no loop, so a missed hold speed is a throttle-gain finding (WARN),
    not a fault. steering_seen: with no steering sensor, what the person said.
    """
    out: List[Line] = []
    up, hold, slow, brake = (phase_rows(rows, p) for p in DRIVING)
    top = max((abs(r['speed']) for r in up + hold if finite(r['speed'])), default=0.0)
    if not up:
        out.append(Line(Status.FAIL, 'speeds up', 'the run ended before it'))
    else:
        out.append(Line(Status.PASS if top >= REACH * cap else Status.FAIL, 'speeds up',
                        f'reached {top:.2f} m/s of {cap:.2f} (needs {REACH * cap:.2f})'))
    held = mean([abs(r['speed']) for r in tail(hold, pr.PF_HOLD_TIME - 1.0)])
    if not hold:
        out.append(Line(Status.FAIL, 'holds the speed', 'the run ended before it'))
    else:
        ok = finite(held) and abs(held - cap) <= HOLD_TOLERANCE * cap
        detail = f'{held:.2f} m/s for {cap:.2f} (within {HOLD_TOLERANCE:.0%})'
        if not ok and open_loop and finite(held) and held > 0:
            out.append(Line(Status.WARN, 'holds the speed',
                            detail + f'. Open loop: the throttle gives {held / cap:.2f} x '
                            'the speed asked: a full_throttle_speed finding (TODO 52b)'))
        else:
            out.append(Line(Status.PASS if ok else Status.FAIL, 'holds the speed', detail))
    slowed = mean([abs(r['speed']) for r in tail(slow, pr.PF_SLOW_TIME - 1.0)])
    if not slow:
        out.append(Line(Status.FAIL, 'slows down', 'the run ended before it'))
    else:
        ok = finite(slowed, held) and slowed < held - 0.2 * cap
        out.append(Line(Status.PASS if ok else Status.FAIL, 'slows down',
                        f'{slowed:.2f} m/s for {0.5 * cap:.2f}, from {held:.2f}'))
    if not brake:
        out.append(Line(Status.FAIL, 'brakes', 'the run ended before it'))
    else:
        t0 = brake[0]['t']
        stopped = [r['t'] for r in brake if finite(r['speed']) and abs(r['speed']) < STOPPED]
        took = (stopped[0] - t0) if stopped else float('inf')
        out.append(Line(Status.PASS if took <= BRAKE_TIME else Status.FAIL, 'brakes',
                        f'still after {took:.1f} s (most {BRAKE_TIME:g} s)'
                        if stopped else f'never under {STOPPED} m/s'))
    out.append(steer_line(rows, lock, centre, centre_tolerance, steering_seen))
    return out


def steer_line(rows: Sequence[Mapping], lock: float, centre: float = 0.0,
               tolerance: float = 0.03, seen: Optional[bool] = None) -> Line:
    name = 'steers left, right, centre'
    if not any(phase_rows(rows, p) for p in (LEFT, RIGHT, CENTRE)):
        return Line(Status.FAIL, name, 'the run ended before it')
    settled = {p: mean([r['steer'] for r in tail(phase_rows(rows, p), 1.0)])
               for p in (LEFT, RIGHT, CENTRE)}
    if all(finite(a) for a in settled.values()):
        left, right, mid = settled[LEFT], settled[RIGHT], settled[CENTRE]
        ok = left > 0.3 * lock and right < -0.3 * lock and abs(mid - centre) <= tolerance
        return Line(Status.PASS if ok else Status.FAIL, name,
                    f'measured {left:+.2f} / {right:+.2f} / {mid:+.3f} rad (full lock '
                    f'{lock:.2f}; straight {centre:+.3f} +- {tolerance:.3f}; + left)')
    if seen is None:
        return Line(Status.SKIP, name, 'no measured angle, and nobody asked: watch the wheels')
    return Line(Status.PASS if seen else Status.FAIL, name,
                'no measured angle: the person ' + ('saw it' if seen else 'did not see it'))


# --- stands: disagreeing on purpose ---------------------------------------------------


def still_bias(rows: Sequence[Mapping]) -> Tuple[float, float]:
    """Return the IMU's forward and left reading at rest, before the drive."""
    calm = [r for r in rows if r['phase'] in (STILL, SETTLE) and finite(r['speed'])
            and abs(r['speed']) < STOPPED]
    return mean([r['ax'] for r in calm]), mean([r['ay'] for r in calm])


def speed_up_window(rows: Sequence[Mapping]) -> List[Mapping]:
    """Return the rows from the first command until the wheels first reach 95% (at most 3 s)."""
    up = phase_rows(rows, SPEED_UP)
    if not up:
        return []
    target = 0.95 * abs(up[0]['cmd_speed'])
    out = []
    for r in up:
        if r['t'] - up[0]['t'] > IMU_WINDOW:
            break
        out.append(r)
        if target > 0 and finite(r['speed']) and abs(r['speed']) >= target:
            break
    return out


def imu_speed_change(window: Sequence[Mapping], bias: Tuple[float, float]) -> float:
    """
    Return the IMU's speed change over the rows: its acceleration less the bias, added up.

    The larger of x and y, so an IMU mounted turned still counts.
    """
    sums = [0.0, 0.0]
    for a, b in zip(window, window[1:]):
        dt = b['t'] - a['t']
        for i, (k, bk) in enumerate((('ax', bias[0]), ('ay', bias[1]))):
            if finite(a[k]) and dt < 0.5:
                sums[i] += (a[k] - bk) * dt
    return max(abs(sums[0]), abs(sums[1]))


def metres(lat0: float, lon0: float, lat1: float, lon1: float) -> float:
    """Return the distance between two nearby fixes (m), flat-earth."""
    north = math.radians(lat1 - lat0) * EARTH_RADIUS
    east = math.radians(lon1 - lon0) * EARTH_RADIUS * math.cos(math.radians(lat0))
    return math.hypot(north, east)


def stands_lines(rows: Sequence[Mapping], ground: bool) -> List[Line]:
    """
    Compare the wheels with the IMU and the GPS.

    On stands (ground False) they must disagree: the wheels spin, nothing else moves.
    On the ground (Gazebo, the bench) they must agree, so the maths is tested there.
    """
    where = ' (sim: on the ground, so the opposite check)' if ground else ''
    imu_name = ('ground: IMU agrees' if ground else 'stands: IMU still') + where
    window = speed_up_window(rows)
    if not phase_rows(rows, STILL) or not window:
        return [Line(Status.FAIL, imu_name, 'the run ended too soon'),
                gps_line(rows, ground, where)]
    bias = still_bias(rows)
    wheels_dv = max(abs(window[-1]['speed']) - abs(window[0]['speed']), 0.0) \
        if finite(window[-1]['speed'], window[0]['speed']) else 0.0
    if not finite(*bias):
        imu = Line(Status.SKIP, imu_name, 'no IMU')
    else:
        imu_dv = imu_speed_change(window, bias)
        ratio = imu_dv / wheels_dv if wheels_dv > 0 else float('inf')
        ok = wheels_dv > 0 and ((ratio >= IMU_GROUND) if ground else (ratio < IMU_STANDS))
        want = f'needs at least {IMU_GROUND:g} x' if ground else f'needs under {IMU_STANDS:g} x'
        imu = Line(Status.PASS if ok else Status.FAIL, imu_name,
                   f'over {window[-1]["t"] - window[0]["t"]:.1f} s: IMU speed change '
                   f'{imu_dv:.2f} m/s, wheels {wheels_dv:.2f} m/s ({want} the wheels)')
    return [imu, gps_line(rows, ground, where)]


def gps_line(rows: Sequence[Mapping], ground: bool, where: str) -> Line:
    """Judge the GPS: by its Doppler speed if there is one, else the fixes (a WARN on stands)."""
    name = ('ground: GPS agrees' if ground else 'stands: GPS still') + where
    driving = [r for r in rows if r['phase'] in DRIVING]
    doppler = [r.get('gps_speed') for r in driving if finite(r.get('gps_speed'))]
    hold = [r for r in phase_rows(rows, HOLD) if finite(r.get('gps_speed'), r['speed'])]
    if doppler and hold:
        if ground:
            gps, wheels = mean([r['gps_speed'] for r in hold]), mean([r['speed'] for r in hold])
            ok = wheels > 0 and gps >= IMU_GROUND * abs(wheels)
            return Line(Status.PASS if ok else Status.FAIL, name,
                        f'Doppler {gps:.2f} m/s in the hold, wheels {abs(wheels):.2f} '
                        f'(needs at least {IMU_GROUND:g} x)')
        top = max(abs(d) for d in doppler)
        return Line(Status.PASS if top < DOPPLER_STILL else Status.FAIL, name,
                    f'Doppler at most {top:.2f} m/s while the wheels spun (needs under '
                    f'{DOPPLER_STILL:g})')
    before = [r for r in phase_rows(rows, SETTLE) if finite(r['lat'], r['lon'])]
    after = [r for r in tail(phase_rows(rows, BRAKE), 1.0) if finite(r['lat'], r['lon'])]
    if not before or not after:
        return Line(Status.SKIP, name, 'no GPS fix before and after')
    moved = metres(mean([r['lat'] for r in before]), mean([r['lon'] for r in before]),
                   mean([r['lat'] for r in after]), mean([r['lon'] for r in after]))
    spun = sum(abs(a['speed']) * (b['t'] - a['t']) for a, b in zip(driving, driving[1:])
               if finite(a['speed']) and b['t'] - a['t'] < 0.5)
    if spun < MIN_SPIN:
        return Line(Status.FAIL if ground else Status.WARN, name,
                    f'the wheels turned only {spun:.1f} m (needs {MIN_SPIN:g} m to tell)')
    if ground:
        ok = moved >= GPS_GROUND * spun
        return Line(Status.PASS if ok else Status.FAIL, name,
                    f'no Doppler: the fixes moved {moved:.1f} m, wheels {spun:.1f} m '
                    f'(needs at least {GPS_GROUND:g} x)')
    # Without Doppler a still GPS can't tell stands from 2 m of rolling: never a PASS.
    return Line(Status.WARN, name,
                f'no Doppler: the fixes moved {moved:.1f} m while the wheels turned '
                f'{spun:.1f} m; within the noise of {GPS_STANDS:g} m: '
                + ('consistent with stands' if moved < GPS_STANDS else 'MOVED: look'))


class ImuWatch:
    """
    Watch the IMU live, on stands: does it say the car moves (off the stands, or never on).

    Before the drive: rest() with the readings at rest gives the bias. While driving:
    add() each reading; it returns why to stop, or None. The acceleration is added up over
    the last WATCH_WINDOW s only, so the bias's small error can't build up.
    """

    def __init__(self) -> None:
        self.bias: Optional[Tuple[float, float]] = None
        self.rest_samples: List[Tuple[float, float]] = []
        self.recent: List[Tuple[float, float, float]] = []   # (t, ax - bias, ay - bias)

    def rest(self, ax: float, ay: float) -> None:
        if finite(ax, ay):
            self.rest_samples.append((ax, ay))

    def start(self) -> bool:
        """Fix the bias from the rest readings; False if there were none."""
        if not self.rest_samples:
            return False
        n = len(self.rest_samples)
        self.bias = (sum(a for a, _ in self.rest_samples) / n,
                     sum(b for _, b in self.rest_samples) / n)
        return True

    def add(self, t: float, ax: float, ay: float, gz: float, steering: bool) -> Optional[str]:
        if self.bias is None or not finite(t, ax, ay):
            return None
        if steering and finite(gz) and abs(gz) > WATCH_YAW:
            return (f'the IMU turns {gz:+.2f} rad/s while steering: the car is turning, '
                    'so it is not on stands')
        self.recent.append((t, ax - self.bias[0], ay - self.bias[1]))
        self.recent = [s for s in self.recent if t - s[0] <= WATCH_WINDOW]
        vx = vy = 0.0
        for a, b in zip(self.recent, self.recent[1:]):
            dt = b[0] - a[0]
            if 0 < dt < 0.5:
                vx += a[1] * dt
                vy += a[2] * dt
        dv = max(abs(vx), abs(vy))
        if dv > WATCH_SPEED:
            return (f'the IMU says the car changed speed by {dv:.2f} m/s in '
                    f'{WATCH_WINDOW:g} s: off the stands?')
        return None


# --- the printout --------------------------------------------------------------------


def passed(lines: Sequence[Line]) -> bool:
    return not any(line.status is Status.FAIL for line in lines)


def table(lines: Sequence[Line]) -> str:
    width = max((len(line.check) for line in lines), default=0)
    body = [f'  {line.status.value:<5} {line.check:<{width}}  {line.detail}' for line in lines]
    fails = sum(line.status is Status.FAIL for line in lines)
    verdict = ('PREFLIGHT PASSED: no check failed (read the WARN and SKIP lines)' if not fails
               else f'PREFLIGHT FAILED: {fails} check{"s" if fails != 1 else ""} failed. '
               'Not on the ground yet')
    return '\n'.join(['Preflight', '========='] + body + ['', verdict])


def counts(lines: Sequence[Line]) -> Dict[str, int]:
    return {s.value: sum(line.status is s for line in lines) for s in Status}
