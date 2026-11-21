r"""
How the car stops: one curve, S(v), for every stop (docs/designs/2026-11-05_how-the-car-stops.md).

No ROS here. The numbers come from the shared car model (car_model.ActuatorConfig: the
brake's lag, dead time and full braking) and ego's limits, so every user asks the same
question and gets the same answer.

```text
   S(v)  =  v·τ  +  v²/(2a)           τ = ρ + a/(2j)
            └┬─┘    └──┬──┘
       delay + easing  braking

   the cap:     v_cap(d) = a·( √(τ² + 2d/a) − τ )     the speed that stops in d metres
   the bumper:  h = room − margin − S(v)              h above 0: room to stop
```

| Symbol | Means | From |
|---|---|---|
| `a` | planned braking (m/s²) | half the softer of ego's max_deceleration and max_brake |
| `j` | jerk (m/s³) | ego's `max_jerk` |
| `ρ` | the normal path's delay (s) | INPUT_DELAY, the brake's dead time and lag, the loop |

Who uses it (each test in test_stop_law.py checks the answer is the same or more cautious):

| User | How |
|---|---|
| `aav_track` speed.py (the trial trackers) | its stop at the path's end: `cap(left)` |
| `aav_drive_to_point` approach.py | `stop_law:=hybrid` (default `approach`, until it wins) |
| ego's log (dynamics.describe) | the stopping distance it prints |
| `aav_behaviour_rs` | the speed cap it sends Nav2 |
| `tools/sim_check.py --stop-point` | `stop_point`: where the model says the car stops |

Not used by the locks: `aav_safety` geofence.py keeps its own law, on purpose (Simplex).

`stop_point` is a different question: not "how far must I plan", but "where will this car
really stop if told to slow down at `ramp`?". It runs the car model itself (the speed loop,
dead times and lags), so it is a prediction to check against a run, not a plan.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, replace
import math
from typing import Optional, Sequence, Tuple

from aav_description.car_model import (ActuatorConfig, CarState, Command, SpeedLoop,
                                       SpeedLoopConfig, step)

# s: the input's age when a node acts on it, and one tick of each node on the way (the
# design's ρ 0.35 = this plus the brake's 0.2 s lag). A guess until test 4.3.
INPUT_DELAY = 0.15
# m/s: the lowest cap the behaviour planner sends Nav2 (0 there means no limit).
FLOOR = 0.3


@dataclass(frozen=True)
class StopCurve:
    """One stopping law. Speeds m/s, distances m, a m/s², j m/s³, ρ s."""

    a: float
    j: float
    rho: float

    def __post_init__(self):
        if not (math.isfinite(self.a) and self.a > 0.0):
            raise ValueError(f'a must be a finite number above 0 (got {self.a})')
        if not (math.isfinite(self.j) and self.j > 0.0):
            raise ValueError(f'j must be a finite number above 0 (got {self.j})')
        if not (math.isfinite(self.rho) and self.rho >= 0.0):
            raise ValueError(f'rho must be a finite number, not negative (got {self.rho})')

    @property
    def tau(self) -> float:
        """Return τ = ρ + a/(2j): the delay plus the easing in of the brake (s)."""
        return self.rho + self.a / (2.0 * self.j)

    def distance(self, speed: float) -> float:
        """Return S(v): how far the car needs to stop from `speed` (m)."""
        v = abs(speed)
        if not math.isfinite(v):
            return math.inf
        return v * self.tau + v * v / (2.0 * self.a)

    def cap(self, left: float) -> float:
        """Return v_cap, the fastest speed that still stops in `left` m; 0 if none."""
        if not math.isfinite(left):
            return math.inf if left > 0 else 0.0
        if left <= 0.0:
            return 0.0
        t = self.tau
        return self.a * (math.sqrt(t * t + 2.0 * left / self.a) - t)

    def demand(self, speed: float) -> float:
        """Return the slowing (m/s²) the curve asks for at `speed`: always below `a`."""
        v = abs(speed)
        return v / (self.tau + v / self.a)

    def h(self, room: float, speed: float, margin: float = 0.0) -> float:
        """Return the bumper h = room − margin − S(v): above 0, there is room to stop (m)."""
        return room - margin - self.distance(speed)

    def describe(self) -> str:
        return (f'the stopping law: a {self.a:.2f} m/s^2, j {self.j:.2f} m/s^3, '
                f'rho {self.rho:.2f} s (tau {self.tau:.2f} s)')


def planned_braking(ego_max_deceleration: float, max_brake: float) -> float:
    """Return a: half the softer of ego's deceleration and the brake's (m/s²)."""
    return min(ego_max_deceleration, max_brake) / 2.0


def loop_lag(loop: Optional[SpeedLoopConfig], a: float, max_brake: float) -> float:
    """
    Return how long the speed loop lags a set speed falling at `a` (s); 0 with no loop.

    The spine's PI loop coasts until the car is `coast_band` too fast, then brakes
    `brake_gain` per m/s beyond it: to brake at `a` it needs the car
    coast_band + a / (max_brake x brake_gain) too fast, which is that many seconds behind
    a set speed falling at `a`. The LQI follows the speed error in about 1 / k_speed s.
    """
    if loop is None:
        return 0.0
    if loop.lqi is not None:
        return 1.0 / loop.lqi.k_speed
    return (loop.coast_band + a / (max_brake * loop.brake_gain)) / a


def from_car_model(cfg: ActuatorConfig = ActuatorConfig(), ego_max_deceleration: float = 2.0,
                   max_jerk: float = 2.0, input_delay: float = INPUT_DELAY,
                   loop: Optional[SpeedLoopConfig] = SpeedLoopConfig()) -> StopCurve:
    """
    Return the curve for this car: the car model's brake and speed loop, ego's limits.

    loop: the speed loop between the set speed and the pedals (the spine's 'default'
    profile, as in Gazebo's adapter and on the car); None for a car that brakes exactly as
    asked (the design's first numbers, ρ 0.35).
    """
    a = planned_braking(ego_max_deceleration, cfg.max_brake)
    return StopCurve(a=a, j=max_jerk,
                     rho=(input_delay + cfg.brake_dead_time + cfg.brake_lag
                          + loop_lag(loop, a, cfg.max_brake)))


def from_numbers(max_brake: float, ego_max_deceleration: float = 2.0, max_jerk: float = 2.0,
                 brake_lag: float = ActuatorConfig.brake_lag, brake_dead_time: float = 0.0,
                 input_delay: float = INPUT_DELAY,
                 loop: Optional[SpeedLoopConfig] = SpeedLoopConfig()) -> StopCurve:
    """Return the curve from loose numbers (a node's parameters), as from_car_model."""
    return from_car_model(ActuatorConfig(max_brake=max_brake, brake_lag=brake_lag,
                                         brake_dead_time=brake_dead_time),
                          ego_max_deceleration, max_jerk, input_delay, loop)


def nav2_cap(left: float, curve: StopCurve, base: float, floor: float = FLOOR) -> float:
    """
    Return the speed limit to send Nav2 for a stop `left` m ahead: m/s, never in (0, floor).

    Nav2 reads 0.0 as no limit, so a cap is either 0.0 (it doesn't bind: no limit) or at
    least the floor, and never above `base` (a higher limit raises MPPI's envelope).
    """
    v = curve.cap(left)
    if not math.isfinite(v) or v >= base:
        return 0.0
    return min(base, max(floor, v))


def stop_point(cfg: ActuatorConfig, speed: float, ramp: float,
               loop: SpeedLoopConfig = SpeedLoopConfig(), dt: float = 0.01,
               settle: float = 8.0, timeout: float = 30.0) -> Tuple[float, float]:
    """
    Return (metres, seconds) the car model goes from the moment the set speed starts to fall.

    The car first holds `speed` for `settle` s (its speed loop settles, as on a long hold);
    then the set speed falls at `ramp` m/s² to 0 (a scripted drive's slowing, or ego's own
    max_deceleration), through the spine's speed loop and the actuators' lags and dead
    times, until the car stands still (below 0.01 m/s).
    """
    if speed <= 0.0 or ramp <= 0.0 or dt <= 0.0:
        raise ValueError('speed, ramp and dt must be above 0')
    return _stop(cfg, speed, lambda t, target: max(0.0, target - ramp * dt), loop, dt, settle,
                 timeout)


def stop_following(cfg: ActuatorConfig, speed: float,
                   commands: Sequence[Tuple[float, float]],
                   loop: SpeedLoopConfig = SpeedLoopConfig(), dt: float = 0.01,
                   settle: float = 8.0, timeout: float = 30.0) -> Tuple[float, float]:
    """
    Return (metres, seconds) the car model goes following `commands` until it stands still.

    stop_point's question for a recorded slowing: `commands` are (time s, set speed m/s),
    oldest first, from the moment the set speed starts to fall (the first one's time is
    0 s); each holds until the next, the last for ever. The car first holds `speed` for
    `settle` s, as in stop_point.
    """
    if speed <= 0.0 or dt <= 0.0:
        raise ValueError('speed and dt must be above 0')
    if not commands:
        raise ValueError('stop_following needs at least one command')
    start = commands[0][0]
    times = [t - start for t, _ in commands]

    def target(t, _):
        return commands[max(0, bisect.bisect_right(times, t) - 1)][1]

    return _stop(cfg, speed, target, loop, dt, settle, timeout)


def _stop(cfg, speed, target_at, loop, dt, settle, timeout):
    """Hold `speed`, then follow target_at(t, last target) until stopped: (metres, seconds)."""
    sl = SpeedLoop(loop)
    state = replace(CarState(), v=speed,
                    throttle_level=min(1.0, cfg.drag * speed / (cfg.max_accel * cfg.share)))
    for _ in range(int(settle / dt)):
        state = step(cfg, state, Command(speed=speed), dt, sl)
    x0, t, target = state.x, 0.0, speed
    while t < timeout:
        target = target_at(t, target)
        state = step(cfg, state, Command(speed=target), dt, sl)
        t += dt
        if target <= 0.0 and abs(state.v) < 0.01:
            break
    return state.x - x0, t
