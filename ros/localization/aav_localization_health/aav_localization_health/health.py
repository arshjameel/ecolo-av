"""
Localization health: is what the EKFs eat, and what they say, believable? No ROS here.

localization_health (the node) tells this module what arrived and when; report() says,
for each check, OK, WARN or STALE and why. It only ever warns: nothing here stops the
car or changes what the EKFs see. The sensor watchdog (aav_safety) already stops Nav2 for
a dead, frozen or jumping sensor and for wheels that disagree with the IMU and the GPS;
these are the checks it doesn't do.

  gyro_bias     parked (the wheels under still_speed), the gyro's mean turn rate over
                the last still_window s. A bias the EKFs fuse as a real turn: GPS off,
                0.6 deg/s drifted 40 m every 100 m in Gazebo. WARN past gyro_bias_warn,
                once it stands out of the noise (3 sigma of the mean)
  imu_mounting  parked, gravity in the IMU's frame: z about +9.8 when imu_link sits as
                vehicle.yaml says. Upside down, tilted past tilt_warn, or a size far from
                9.8 (wrong units) means vehicle.yaml's rpy or the driver is wrong
  gps_quality   the newest fix: has a fix, says how sure it is (covariance type), and
                its sigma is neither huge (gps_sigma_max) nor too sure (gps_sigma_min).
                The EKF weighs each fix by that covariance
  timestamps    each input's header.stamp against this node's clock: a computer whose
                clock is off makes the EKFs drop or misplace its data (several computers
                on the car)
  frames        each input's frame_id is the one the TF tree and the EKFs expect
  tf            base_link -> imu_link and gps_link exist (navsat_transform needs the
                antenna's), and map -> base_link is fresh
  wheels        the two front wheels against each other: their speeds must differ as the
                car's speed and the gyro's turn rate say (a dead or slipping wheel sensor)
  motion        each EKF's output: no speed, turn rate or acceleration the car can't do
                (a filter running away); and ekf_local, which never jumps by design, no
                step either. ekf_global may jump: GPS corrections are its job

A check with an input it needs that has gone quiet (stale_after) says STALE. One whose
optional input never came (no GPS on the bench, no wheel speeds on today's car) says OK,
"not on this profile". Times are seconds on the node's clock; stamps are seconds too.
"""

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Tuple

OK = 'OK'
WARN = 'WARN'
STALE = 'STALE'
G = 9.80665                 # m/s^2

# What frame each input must be in (vehicle.yaml's sensor names, REP-105).
FRAMES = {'imu': ('imu_link', None), 'gps': ('gps_link', None),
          'odom': ('odom', 'base_link'), 'wheels': ('base_link', None),
          'local': ('odom', 'base_link'), 'global': ('map', 'base_link')}
# Inputs each check can't do without; a REQUIRED one that never came is STALE too.
REQUIRED = ('imu', 'odom', 'local')
NEEDS = {'gyro_bias': ('imu', 'odom'), 'imu_mounting': ('imu', 'odom'),
         'gps_quality': ('gps',), 'wheels': ('wheels', 'imu', 'odom'),
         'motion': ('local', 'global')}
CHECKS = ('gyro_bias', 'imu_mounting', 'gps_quality', 'timestamps', 'frames', 'tf',
          'wheels', 'motion')


@dataclass(frozen=True)
class Settings:
    """The thresholds: config/localization_health.yaml."""

    stale_after: float = 2.0            # s
    still_speed: float = 0.02           # m/s: slower is parked
    still_min: float = 5.0              # s parked before the gyro and gravity are judged
    still_window: float = 10.0          # s: the newest parked samples used
    gyro_bias_warn: float = math.radians(0.2)   # rad/s
    tilt_warn: float = math.radians(10.0)       # rad
    gravity_tolerance: float = 1.0      # m/s^2
    gps_sigma_max: float = 5.0          # m
    gps_sigma_min: float = 0.005        # m
    skew_behind: float = 0.5            # s: a stamp this much older than now
    skew_ahead: float = 0.1             # s: a stamp from the future
    wheel_min_speed: float = 0.3        # m/s
    wheel_tolerance: float = 0.15       # m/s, plus wheel_share of the speed
    wheel_share: float = 0.1
    persist: float = 1.0                # s a wheel gap must last
    max_speed: float = 5.0              # m/s: 1 m/s today, Nav2's cap 2.4
    max_yaw_rate: float = 2.0           # rad/s
    max_accel: float = 6.0              # m/s^2: more than the tyres' grip allows
    step_margin: float = 2.0            # m an EKF may move beyond max_speed in a message
    hold: float = 5.0                   # s a motion warning stays up


@dataclass
class Status:
    """One check's verdict."""

    name: str
    level: str
    message: str
    values: Dict[str, float] = field(default_factory=dict)


def front_wheel_speeds(v: float, yaw_rate: float, wheelbase: float,
                       track: float) -> Tuple[float, float]:
    """Return the left and right front wheels' ground speeds (m/s) for the car's motion."""
    left = math.hypot(v - yaw_rate * track / 2.0, yaw_rate * wheelbase)
    right = math.hypot(v + yaw_rate * track / 2.0, yaw_rate * wheelbase)
    return left, right


def mean_and_sigma(values: List[float]) -> Tuple[float, float]:
    """Return the mean and the standard deviation of the mean (0 for one value)."""
    n = len(values)
    mean = sum(values) / n
    if n < 2:
        return mean, 0.0
    var = sum((x - mean) ** 2 for x in values) / (n - 1)
    return mean, math.sqrt(var / n)


class Health:
    """Every check's state, fed message by message."""

    def __init__(self, wheelbase: float, track: float, settings: Settings = Settings()):
        self.wheelbase, self.track, self.s = wheelbase, track, settings
        self.seen: Dict[str, float] = {}        # input -> when it last arrived
        self.skew: Dict[str, float] = {}        # input -> now - stamp, newest
        self.bad_frames: Dict[str, str] = {}
        self.speed: Optional[float] = None
        self.gyro_z: Optional[float] = None
        self.parked: deque = deque()            # (t, wz, ax, ay, az) while parked
        self.gps: Optional[Status] = None
        self.wheel_gap = 0.0
        self.wheel_bad_since: Optional[float] = None
        self.last_pose: Dict[str, Tuple[float, float, float, float]] = {}
        self.motion_bad: Dict[str, Tuple[float, str]] = {}
        self.tf_state: Optional[Tuple[List[str], Optional[float]]] = None

    # --- what arrived --------------------------------------------------------------

    def _arrived(self, name: str, now: float, stamp: float, frame: str,
                 child: Optional[str] = None):
        self.seen[name] = now
        self.skew[name] = now - stamp
        want, want_child = FRAMES[name]
        if frame != want or (want_child is not None and child != want_child):
            got = frame if want_child is None else f'{frame} -> {child}'
            ask = want if want_child is None else f'{want} -> {want_child}'
            self.bad_frames[name] = f'{name} in {got or "(none)"}, not {ask}'
        else:
            self.bad_frames.pop(name, None)

    def imu(self, now: float, stamp: float, frame: str, wz: float,
            accel: Tuple[float, float, float]):
        """Take an IMU message: its turn rate (rad/s) and acceleration (m/s^2)."""
        self._arrived('imu', now, stamp, frame)
        self.gyro_z = wz
        if self.speed is not None and abs(self.speed) < self.s.still_speed:
            self.parked.append((now, wz) + tuple(accel))
            while self.parked and self.parked[0][0] < now - self.s.still_window:
                self.parked.popleft()
        else:
            self.parked.clear()

    def odom(self, now: float, stamp: float, frame: str, child: str, v: float):
        """Take the wheels' odometry: the car's speed (m/s)."""
        self._arrived('odom', now, stamp, frame, child)
        self.speed = v
        if abs(v) >= self.s.still_speed:
            self.parked.clear()

    def gps_fix(self, now: float, stamp: float, frame: str, status: int,
                covariance_type: int, covariance: List[float]):
        """Take a NavSatFix: its status, covariance type (0 unknown) and covariance."""
        self._arrived('gps', now, stamp, frame)
        sigma = math.sqrt(max(covariance[0], covariance[4], 0.0))
        values = {'sigma_m': sigma, 'status': float(status)}
        if status < 0:
            self.gps = Status('gps_quality', WARN, 'no fix', values)
        elif covariance_type == 0:
            self.gps = Status('gps_quality', WARN,
                              'covariance unknown: the EKF guesses how far to trust it',
                              values)
        elif sigma > self.s.gps_sigma_max:
            self.gps = Status('gps_quality', WARN, f'poor fix: sigma {sigma:.1f} m', values)
        elif sigma < self.s.gps_sigma_min:
            self.gps = Status('gps_quality', WARN,
                              f'claims sigma {sigma * 100:.2f} cm: too sure', values)
        else:
            self.gps = Status('gps_quality', OK, f'sigma {sigma:.2f} m', values)

    def wheels(self, now: float, stamp: float, frame: str, left: float, right: float):
        """Take the front wheels' speeds (m/s); NaN is a wheel not measured."""
        self._arrived('wheels', now, stamp, frame)
        if (math.isnan(left) or math.isnan(right) or self.speed is None
                or self.gyro_z is None or abs(self.speed) < self.s.wheel_min_speed):
            self.wheel_bad_since = None
            return
        want_l, want_r = front_wheel_speeds(abs(self.speed), self.gyro_z, self.wheelbase,
                                            self.track)
        self.wheel_gap = abs((abs(right) - abs(left)) - (want_r - want_l))
        if self.wheel_gap > self.s.wheel_tolerance + self.s.wheel_share * abs(self.speed):
            if self.wheel_bad_since is None:
                self.wheel_bad_since = now
        else:
            self.wheel_bad_since = None

    def ekf(self, name: str, now: float, stamp: float, frame: str, child: str,
            x: float, y: float, v: float, yaw_rate: float):
        """Take an EKF's output (name local or global): its position, speed, turn rate."""
        self._arrived(name, now, stamp, frame, child)
        why = None
        if abs(v) > self.s.max_speed:
            why = f'{v:.1f} m/s'
        elif abs(yaw_rate) > self.s.max_yaw_rate:
            why = f'turning {yaw_rate:.1f} rad/s'
        last = self.last_pose.get(name)
        if why is None and last is not None and stamp - last[0] > 1e-3:
            dt = stamp - last[0]
            accel = (v - last[3]) / dt
            step = math.hypot(x - last[1], y - last[2])
            if abs(accel) > self.s.max_accel and dt >= 0.05:
                why = f'{accel:.1f} m/s^2'
            elif name == 'local' and step > self.s.max_speed * dt + self.s.step_margin:
                # Only ekf_local promises never to jump; ekf_global's jumps are GPS
                # corrections (metres, after a dropout), and its gate bounds them.
                why = f'jumped {step:.1f} m'
        self.last_pose[name] = (stamp, x, y, v)
        if why is not None:
            self.motion_bad[name] = (now + self.s.hold, why)

    def tf(self, missing: List[str], tree_age: Optional[float]):
        """Take the TF check: static frames missing, and map -> base_link's age (None: no)."""
        self.tf_state = (missing, tree_age)

    # --- the verdicts --------------------------------------------------------------

    def _stale(self, now: float, check: str) -> Optional[str]:
        for name in NEEDS.get(check, ()):
            if name in self.seen:
                if now - self.seen[name] > self.s.stale_after:
                    return f'nothing on {name} for {now - self.seen[name]:.1f} s'
            elif name in REQUIRED:
                return f'no {name} yet'
        return None

    def _not_here(self, check: str) -> bool:
        return any(n not in self.seen and n not in REQUIRED for n in NEEDS.get(check, ()))

    def _gyro_and_gravity(self, now: float) -> Tuple[Status, Status]:
        enough = (self.parked and now - self.parked[0][0] >= self.s.still_min
                  and len(self.parked) >= 20)
        if not enough:
            wait = f'judged after {self.s.still_min:g} s parked'
            return Status('gyro_bias', OK, wait), Status('imu_mounting', OK, wait)
        _, wz, ax, ay, az = zip(*self.parked)
        bias, sigma = mean_and_sigma(list(wz))
        values = {'bias_dps': math.degrees(bias), 'sigma_dps': math.degrees(sigma),
                  'drift_deg_per_min': math.degrees(bias) * 60.0}
        if abs(bias) > self.s.gyro_bias_warn and abs(bias) > 3.0 * sigma:
            gyro = Status('gyro_bias', WARN,
                          f'{math.degrees(bias):+.2f} deg/s parked: the heading drifts '
                          f'{abs(math.degrees(bias)) * 60:.0f} deg a minute without GPS',
                          values)
        else:
            gyro = Status('gyro_bias', OK, f'{math.degrees(bias):+.3f} deg/s', values)
        gx, gy, gz = (sum(c) / len(c) for c in (ax, ay, az))
        size = math.sqrt(gx * gx + gy * gy + gz * gz)
        tilt = math.acos(max(-1.0, min(1.0, gz / size))) if size > 0 else math.pi
        gvalues = {'gravity_x': gx, 'gravity_y': gy, 'gravity_z': gz,
                   'tilt_deg': math.degrees(tilt)}
        if gz < 0:
            mount = Status('imu_mounting', WARN,
                           f'upside down: gravity z {gz:+.1f} m/s^2 (imu_link rpy?)', gvalues)
        elif abs(size - G) > self.s.gravity_tolerance:
            mount = Status('imu_mounting', WARN,
                           f'gravity reads {size:.1f} m/s^2, not {G:.1f}', gvalues)
        elif tilt > self.s.tilt_warn:
            mount = Status('imu_mounting', WARN,
                           f'tilted {math.degrees(tilt):.0f} deg (imu_link rpy?)', gvalues)
        else:
            mount = Status('imu_mounting', OK, f'level within {math.degrees(tilt):.1f} deg',
                           gvalues)
        return gyro, mount

    def report(self, now: float) -> List[Status]:
        """Return every check's status, in CHECKS order."""
        out = {}
        out['gyro_bias'], out['imu_mounting'] = self._gyro_and_gravity(now)
        out['gps_quality'] = self.gps or Status('gps_quality', OK, 'not on this profile')
        fresh = {n: s for n, s in self.skew.items()
                 if now - self.seen[n] <= self.s.stale_after}
        late = {n: s for n, s in fresh.items()
                if s > self.s.skew_behind or s < -self.s.skew_ahead}
        out['timestamps'] = Status(
            'timestamps', WARN if late else OK,
            ('; '.join(f'{n} stamped {s:+.2f} s off' for n, s in sorted(late.items()))
             if late else f'{len(fresh)} inputs on time'),
            {f'{n}_s': s for n, s in fresh.items()})
        out['frames'] = Status('frames', WARN if self.bad_frames else OK,
                               '; '.join(sorted(self.bad_frames.values())) or 'as expected')
        if self.tf_state is None:
            out['tf'] = Status('tf', OK, 'not looked yet')
        else:
            missing, age = self.tf_state
            if missing:
                out['tf'] = Status('tf', WARN, 'no base_link -> ' + ', '.join(missing))
            elif age is None or age > self.s.stale_after:
                out['tf'] = Status('tf', WARN, 'map -> base_link missing or old')
            else:
                out['tf'] = Status('tf', OK, f'map -> base_link {age:.2f} s old',
                                   {'tree_age_s': age})
        if self.wheel_bad_since is not None and now - self.wheel_bad_since >= self.s.persist:
            out['wheels'] = Status('wheels', WARN,
                                   f'front wheels {self.wheel_gap:.2f} m/s apart from what '
                                   'the turn says', {'gap': self.wheel_gap})
        else:
            out['wheels'] = Status('wheels', OK, 'front wheels agree',
                                   {'gap': self.wheel_gap})
        bad = {n: why for n, (until, why) in self.motion_bad.items() if now < until}
        out['motion'] = Status('motion', WARN if bad else OK,
                               '; '.join(f'ekf_{n}: {w}' for n, w in sorted(bad.items()))
                               or 'believable')
        for name in CHECKS:
            stale = self._stale(now, name)
            if stale:
                out[name] = Status(name, STALE, stale)
            elif self._not_here(name) and out[name].level == OK and name != 'motion':
                out[name] = Status(name, OK, 'not on this profile')
        return [out[name] for name in CHECKS]


def summary(statuses: List[Status]) -> str:
    """Return OK, or WARN and every check that isn't OK."""
    bad = [s for s in statuses if s.level != OK]
    if not bad:
        return OK
    return WARN + ' ' + '; '.join(f'{s.name} {s.level}: {s.message}' for s in bad)
