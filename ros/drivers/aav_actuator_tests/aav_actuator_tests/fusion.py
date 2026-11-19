"""
The IMU and the wheels together: acceleration and speed you can trust. No ROS here.

TODO 248 (docs/testing-plan.md, phase B, "Using the IMU as well as the wheels").

| Source | Gives | Good | Weak |
|---|---|---|---|
| Wheel speed | speed | no drift | its change (the acceleration) is noisy, Hall pulses coarse |
| IMU | acceleration, 100 times a second | quick and smooth | an offset; the floor's slope |

The steps:

  1. still_imu: stand still first (10 s). The IMU's mean is its offset (its bias plus the
     floor's slope, g sin(pitch)); its spread is its noise.
  2. slope_from: one offset facing each way along the hallway. The bias is the same both
     ways, the slope's share flips: bias = (there + back) / 2, g sin(pitch) =
     (there - back) / 2. One way only: the slope can't be told from the bias.
  3. fuse: a small Kalman filter, state [speed, IMU bias drift]. Each IMU sample moves the
     speed on (a dt); each wheel speed pulls it back (no drift). Out: the speed and the
     acceleration, each with its standard deviation.
  4. forces: F = m a, with the car's mass from vehicle.yaml.

    IMU ax - offset ──► predict v ──► v, a (± sd)
    wheel speed ───────► correct ──┘

Units: m/s, m/s^2, s, rad, kg, N. Positive acceleration = speeding up forwards.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

G = 9.80665              # m/s^2
STOPPED = 0.05           # m/s: slower is standing still (analysis.STOPPED)
MIN_STILL = 3.0          # s of standing still to trust an offset
LEVEL_LIMIT = 0.01       # rad (1 %, 0.1 m/s^2): steeper is not level enough
WHEEL_NOISE_FLOOR = 0.02  # m/s: the least noise a wheel speed is taken to have
IMU_NOISE_FLOOR = 0.02   # m/s^2
BIAS_WALK = 0.01         # m/s^2 per sqrt(s): how fast the IMU's bias may drift
NAN = float('nan')


def finite(*values: float) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def mean_sd(values: Sequence[float]) -> Tuple[float, float]:
    """Return the mean and the sample standard deviation (NaN for none; sd 0 for one)."""
    xs = [v for v in values if finite(v)]
    if not xs:
        return NAN, NAN
    m = sum(xs) / len(xs)
    if len(xs) < 2:
        return m, 0.0
    return m, math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


@dataclass(frozen=True)
class Still:
    """The IMU standing still: its offset (bias + slope) and noise; the wheels' noise."""

    offset: float        # m/s^2: the mean forward reading
    noise: float         # m/s^2: its standard deviation
    samples: int
    duration: float      # s
    wheel_noise: float   # m/s: the wheel speed's spread meanwhile (0 for a clean zero)

    @property
    def trusted(self) -> bool:
        return self.duration >= MIN_STILL and self.samples >= 20


def still_imu(t: Sequence[float], ax: Sequence[float], speed_t: Sequence[float],
              speed: Sequence[float], start: float = -math.inf,
              end: float = math.inf) -> Optional[Still]:
    """
    Return the IMU's offset and noise while the car stood still between start and end.

    Still: the wheel speed (read between its samples) under STOPPED. None if no IMU
    sample was still.
    """
    picked = []
    for ti, a in zip(t, ax):
        if not (start <= ti <= end) or not finite(ti, a):
            continue
        v = held_at(speed_t, speed, ti)
        if finite(v) and abs(v) < STOPPED:
            picked.append((ti, a))
    if not picked:
        return None
    offset, noise = mean_sd([a for _, a in picked])
    wheels = [v for ti, v in zip(speed_t, speed) if start <= ti <= end and finite(v)
              and abs(v) < STOPPED]
    return Still(offset, noise, len(picked), picked[-1][0] - picked[0][0],
                 mean_sd(wheels)[1] if len(wheels) > 1 else 0.0)


def held_at(ts: Sequence[float], values: Sequence[float], t: float) -> float:
    """Return the latest value at or before t (sample-and-hold); NaN before the first."""
    lo, hi = 0, len(ts)
    while lo < hi:
        mid = (lo + hi) // 2
        if ts[mid] <= t:
            lo = mid + 1
        else:
            hi = mid
    return values[lo - 1] if lo > 0 else NAN


@dataclass(frozen=True)
class Slope:
    """The floor along the hallway, from the stand-still offsets facing each way."""

    angle: float     # rad, + uphill facing 'there'; NaN when only one way was measured
    bias: float      # m/s^2: the IMU's own offset (NaN with one way)
    there: float     # m/s^2: the mean offset facing 'there' (NaN if none)
    back: float      # m/s^2: the same facing 'back'
    level: Optional[bool]   # within LEVEL_LIMIT; None: can't tell (one way only)

    @property
    def grade(self) -> float:
        """Return the slope in % (+ uphill facing 'there')."""
        return 100.0 * math.tan(self.angle) if finite(self.angle) else NAN

    def gravity(self, direction: str) -> float:
        """
        Return g sin(pitch) for a run facing `direction` ('there' or 'back'): m/s^2.

        Uphill is positive. The car's true acceleration on the slope is the flat car's
        less this, so a_flat = a_measured + gravity(direction). 0 when unknown.
        """
        if not finite(self.angle):
            return 0.0
        return G * math.sin(self.angle) * (1.0 if direction == 'there' else -1.0)

    def describe(self) -> str:
        if not finite(self.angle):
            return ("one way only: the slope cannot be told from the IMU's offset. Run "
                    'facing both ways (the car turned round), or check with a spirit level')
        how = 'level enough' if self.level else 'NOT level enough'
        return (f'{self.grade:+.2f} % ({math.degrees(self.angle):+.2f} deg, uphill facing '
                f"'there' when +): {how} (limit {100 * math.tan(LEVEL_LIMIT):.1f} %); "
                f'IMU bias {self.bias:+.3f} m/s^2')


def slope_from(offsets: Sequence[Tuple[str, float]]) -> Slope:
    """
    Return the slope from the stand-still offsets: (direction, offset m/s^2) pairs.

    The accelerometer standing still reads g sin(pitch) + its bias; turning the car round
    flips the pitch but not the bias.
    """
    there = [o for d, o in offsets if d == 'there' and finite(o)]
    back = [o for d, o in offsets if d == 'back' and finite(o)]
    mt = mean_sd(there)[0] if there else NAN
    mb = mean_sd(back)[0] if back else NAN
    if not (there and back):
        return Slope(NAN, NAN, mt, mb, None)
    s = max(-1.0, min(1.0, (mt - mb) / (2.0 * G)))
    angle = math.asin(s)
    return Slope(angle, 0.5 * (mt + mb), mt, mb, abs(angle) <= LEVEL_LIMIT)


@dataclass(frozen=True)
class Fused:
    """The speed and acceleration, blended, each with its standard deviation."""

    t: List[float]
    speed: List[float]
    accel: List[float]
    speed_sd: List[float]
    accel_sd: List[float]
    how: str             # 'imu+wheels', or 'wheels' (no IMU: the speed's slope)

    def at(self, t: float) -> Tuple[float, float]:
        """Return (speed, accel) at time t, held from the sample before."""
        return held_at(self.t, self.speed, t), held_at(self.t, self.accel, t)


def fuse(imu_t: Sequence[float], ax: Sequence[float], wheel_t: Sequence[float],
         wheel_v: Sequence[float], offset: float = 0.0, imu_noise: float = 0.2,
         wheel_noise: float = 0.05, bias_walk: float = BIAS_WALK,
         gravity: float = 0.0) -> Fused:
    """
    Blend the IMU's forward acceleration with the wheel speed: a two-state Kalman filter.

    State [v, b]: the speed, and the IMU bias left after `offset` (it drifts, slowly).
    Predict with each IMU sample: v += (ax - offset - b) dt. Correct with each wheel speed
    (R = wheel_noise^2). gravity (Slope.gravity) is added to the acceleration given out:
    the flat car's. With no IMU at all, the wheels alone (their slope over 0.3 s).
    """
    imu = [(a, b) for a, b in zip(imu_t, ax) if finite(a, b)]
    wheels = [(a, b) for a, b in zip(wheel_t, wheel_v) if finite(a, b)]
    if not wheels:
        return Fused([], [], [], [], [], 'none')
    if len(imu) < 10:
        return wheels_only(wheels, gravity)
    q_a = max(imu_noise, IMU_NOISE_FLOOR) ** 2
    r = max(wheel_noise, WHEEL_NOISE_FLOOR) ** 2
    v, b = wheels[0][1], 0.0
    p00, p01, p11 = r, 0.0, (0.1 * max(imu_noise, IMU_NOISE_FLOOR)) ** 2
    out = Fused([], [], [], [], [], 'imu+wheels')
    j = 0
    last_t = imu[0][0]
    for ti, a_raw in imu:
        # The IMU sample is the acceleration over the time since the last one: predict
        # to now with it, then correct with the wheel speeds that have come meanwhile.
        dt = max(0.0, ti - last_t)
        last_t = ti
        a = a_raw - offset - b
        v += a * dt
        # P = F P F' + Q with F = [[1, -dt], [0, 1]].
        p00 = p00 - 2 * dt * p01 + dt * dt * p11 + q_a * dt * dt
        p01 = p01 - dt * p11
        p11 = p11 + bias_walk ** 2 * dt
        while j < len(wheels) and wheels[j][0] <= ti:
            s = p00 + r
            k0, k1 = p00 / s, p01 / s
            innov = wheels[j][1] - v
            v += k0 * innov
            b += k1 * innov
            p00, p01, p11 = (1 - k0) * p00, (1 - k0) * p01, p11 - k1 * p01
            j += 1
        out.t.append(ti)
        out.speed.append(v)
        out.accel.append(a + gravity)
        out.speed_sd.append(math.sqrt(max(p00, 0.0)))
        out.accel_sd.append(math.sqrt(q_a + max(p11, 0.0)))
    return out


def wheels_only(wheels: Sequence[Tuple[float, float]], gravity: float = 0.0,
                window: float = 0.3) -> Fused:
    """No IMU: the wheel speed, and its slope over `window` s (noisier, later)."""
    t = [w[0] for w in wheels]
    v = [w[1] for w in wheels]
    accel = []
    for i, ti in enumerate(t):
        idx = [k for k in range(max(0, i - 50), min(len(t), i + 51))
               if abs(t[k] - ti) <= window / 2]
        xs, ys = [t[k] for k in idx], [v[k] for k in idx]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        accel.append((sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx > 0
                      else NAN) + gravity)
    resid = [abs(b - a) for a, b in zip(v, v[1:])]
    sd_v = mean_sd(resid)[1] if len(resid) > 1 else WHEEL_NOISE_FLOOR
    sd_a = sd_v * math.sqrt(12) / max(window, 1e-3)
    return Fused(t, v, accel, [sd_v] * len(t), [sd_a] * len(t), 'wheels')


# --- forces ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Forces:
    """The forces on the car, from its mass and what it measured: N."""

    mass: float           # kg
    drive_peak: float     # N: the most the motor pushed (m a, speeding up)
    brake_peak: float     # N: the most the brake held back (m a, braking; positive)
    drag_at_1: float      # N: the drag at 1 m/s (m x drag x 1 m/s)
    max_accel: float      # m/s^2 behind drive_peak
    max_decel: float      # m/s^2 behind brake_peak


def forces(mass: float, accel_up: Sequence[float], accel_down: Sequence[float],
           drag: float, smooth: int = 5) -> Forces:
    """
    Return F = m a for the hardest push (speeding up) and the hardest stop (braking).

    accel_up and accel_down: fused accelerations while speeding up and while braking; a
    running mean over `smooth` samples first, so one noisy sample doesn't make the peak.
    """
    def peak(values, sign):
        xs = [v for v in values if finite(v)]
        if len(xs) < smooth:
            return NAN
        means = [sum(xs[i:i + smooth]) / smooth for i in range(len(xs) - smooth + 1)]
        return max(sign * m for m in means)

    up, down = peak(accel_up, 1.0), peak(accel_down, -1.0)
    return Forces(mass, mass * up if finite(up) else NAN,
                  mass * down if finite(down) else NAN,
                  mass * drag if finite(drag) else NAN, up, down)
