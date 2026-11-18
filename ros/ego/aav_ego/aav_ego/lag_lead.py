"""
ego's lead: ask a little ahead, so the slow actuators answer about when the driver wanted.

No ROS imports, so the tests run anywhere. The node: ego_node.py; the page:
docs/vehicle-model.md, "ego's lead".

The car's actuators answer late: the throttle, brake and steering each wait their dead time,
then follow with a first-order lag (aav_description car_model.py). For a lag T, the lead

    u = f + T df/dt,   f = the driver's request through a filter of time constant tf

is (1 + T s) / (1 + tf s): the actuator's 1 / (1 + T s) after it leaves 1 / (1 + tf s).
The car then answers as if its lag were filter_time instead of T. A dead time can't be
undone (it would need the future), so it counts as lag: T = dead_time + lag
(e^(-s Td) is about 1 / (1 + Td s)). Each T comes from the car model's numbers:

    | Lead        | T                                   |
    |-------------|-------------------------------------|
    | speeding up | throttle_dead_time + throttle_lag   |
    | slowing     | brake_dead_time + brake_lag         |
    | steering    | steer_dead_time + steer_lag         |

The steering motor's rate limit (steer_rate) is not a lag: no lead undoes it.

Safety (ego stays a safety lock):

  - The lead runs before ego's limits: max_speed, the reverse limit, the steering limit and
    rate, the slews, the jerk and the body's caps all still apply to what it gives.
  - Each lead is at most max_speed_lead (m/s) and max_steer_lead (rad).
  - It never turns a forward request into a reverse one, or back.
  - It never softens a slow-down: while slowing, the speed is never above the request.
  - A stop (no fresh command) gets no lead at all.

When (lead_wanted): lag_lead off - never; auto - in a simulator only (use_sim_time),
because the car's numbers are guesses until tests 3.10-3.12 and 4.1-4.5; on - on the car
too, once those are measured. And never while a lag-aware driver drives (it already
leads for the lag itself: two leads would ask twice): LagAwareWatch tells.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Dict, Hashable, Optional, Tuple

MODES = ('auto', 'on', 'off')

# Read only at start-up (ego refuses to change them while running); lag_lead is live.
START_ONLY = frozenset({'lag_lead_filter', 'lag_lead_max_speed', 'lag_lead_max_steer',
                        'lag_aware_drivers', 'trace_horizon', 'trace_rate_hz',
                        'trace_speed_loop', 'vehicle_yaml', 'road_ahead',
                        'road_ahead_drivers', 'road_pose_timeout', 'pedal_drivers',
                        'pedal_let_off', 'pedal_full', 'pedal_shift_hold',
                        'pedal_stop_speed'})


def check_mode(mode) -> Optional[str]:
    """Return why this lag_lead value is refused, or None if it is fine."""
    if mode not in MODES:
        return f"lag_lead must be one of {', '.join(MODES)}, not {mode!r}"
    return None


def lead_wanted(mode: str, sim: bool, lag_aware_driving: bool) -> bool:
    """Tell whether the lead runs now: off never, auto in a simulator, on always; never twice."""
    if lag_aware_driving:
        return False
    return mode == 'on' or (mode == 'auto' and sim)


@dataclass(frozen=True)
class LeadTimes:
    """How late each actuator answers (s): its dead time plus its lag."""

    speed_up: float
    slow_down: float
    steer: float

    @classmethod
    def from_config(cls, cfg) -> 'LeadTimes':
        """From a car_model.ActuatorConfig."""
        return cls(speed_up=cfg.throttle_dead_time + cfg.throttle_lag,
                   slow_down=cfg.brake_dead_time + cfg.brake_lag,
                   steer=cfg.steer_dead_time + cfg.steer_lag)


@dataclass(frozen=True)
class LeadLimits:
    filter_time: float = 0.1      # s: the request's filter (the car then answers this fast)
    max_speed_lead: float = 0.3   # m/s: the most the lead adds to or takes from a speed
    max_steer_lead: float = 0.1   # rad: the most it adds to or takes from an angle


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class LagLead:
    """The lead on a (speed, steering angle) request, one tick at a time."""

    def __init__(self, times: LeadTimes, limits: LeadLimits = LeadLimits()):
        self.times = times
        self.limits = limits
        self.speed = None    # the filtered request; None: start from the next one
        self.angle = None

    def reset(self) -> None:
        """Forget the filters: the next request starts them again, with no kick."""
        self.speed = self.angle = None

    def _filter(self, old: float, new: float, dt: float) -> float:
        tf = self.limits.filter_time
        return new if tf <= 0.0 else old + (new - old) * (1.0 - math.exp(-dt / tf))

    def step(self, speed: float, angle: float, dt: float,
             active: bool = True) -> Tuple[float, float]:
        """Return the led (speed, angle) for the driver's request; unchanged when not active."""
        if not active or dt <= 0.0:
            self.reset()
            return speed, angle
        if self.speed is None:            # the first request: the filters start on it
            self.speed, self.angle = speed, angle
            return speed, angle
        lim = self.limits
        speeding_up = abs(speed) > abs(self.speed)
        f_speed = self._filter(self.speed, speed, dt)
        f_angle = self._filter(self.angle, angle, dt)
        rate_speed = (f_speed - self.speed) / dt
        rate_angle = (f_angle - self.angle) / dt
        self.speed, self.angle = f_speed, f_angle

        t = self.times.speed_up if speeding_up else self.times.slow_down
        led = f_speed + clamp(t * rate_speed, -lim.max_speed_lead, lim.max_speed_lead)
        # Never the other way: forward stays forward (or stopped), reverse stays reverse.
        if speed > 0.0 or (speed == 0.0 and f_speed >= 0.0):
            led = max(0.0, led)
        else:
            led = min(0.0, led)
        # Never soften a slow-down.
        if not speeding_up:
            led = min(led, speed) if speed >= 0.0 else max(led, speed)
        led_angle = f_angle + clamp(self.times.steer * rate_angle,
                                    -lim.max_steer_lead, lim.max_steer_lead)
        return led, led_angle


def command_key(stamp_sec: int, stamp_nanosec: int, vx: float, wz: float) -> Hashable:
    """Return what identifies one TwistStamped (twist_mux passes it on unchanged)."""
    return (stamp_sec, stamp_nanosec, vx, wz)


class LagAwareWatch:
    """
    Tell whether the command ego obeys now came from a lag-aware driver.

    Each driver that can lead for the lag itself says so on /drivers/<driver>/lag_aware
    (std_msgs/Bool, latched). twist_mux passes the winning driver's TwistStamped on to
    /control/cmd_vel unchanged, so ego keeps each driver's last few commands and looks for
    the one it obeys among them.
    """

    def __init__(self, drivers, keep: int = 50):
        self.aware: Dict[str, bool] = dict.fromkeys(drivers, False)
        self.recent: Dict[str, Deque[Hashable]] = {d: deque(maxlen=keep) for d in drivers}

    def set_aware(self, driver: str, aware: bool) -> None:
        self.aware[driver] = bool(aware)

    def sent(self, driver: str, key: Hashable) -> None:
        self.recent[driver].append(key)

    def whose(self, key: Optional[Hashable]) -> Optional[str]:
        """Return the watched driver whose command this is, lag-aware or not, or None."""
        if key is None:
            return None
        for driver, recent in self.recent.items():
            if key in recent:
                return driver
        return None

    def driving(self, key: Optional[Hashable]) -> Optional[str]:
        """Return the lag-aware driver whose command this is, or None."""
        if key is None:
            return None
        for driver, aware in self.aware.items():
            if aware and key in self.recent[driver]:
                return driver
        return None
