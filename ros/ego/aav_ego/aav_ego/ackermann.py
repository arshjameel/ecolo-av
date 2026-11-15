"""
The math behind ego: a Twist (v, w) becomes an Ackermann (speed, steering).

No ROS imports, so the tests run anywhere.

Model: the bicycle model about the rear axle (base_link). A car moving at speed
v with front-wheel angle d turns at w = v * tan(d) / L, where L is the
wheelbase. So d = atan(L * w / v). Nav2's car-aware planners and controllers
(SmacPlannerHybrid, Regulated Pure Pursuit, MPPI with its Ackermann model)
only ask for turns the car can make, so this conversion is exact for them.

A car can't turn on the spot: with v near 0 the formula has no answer, so ego
keeps the wheels where they are until the car moves.

With a Body (dynamics.py: the car's mass, load, track, centre of mass and tyres), ego also
never asks the loaded car to speed up harder than its motor can, and slows it for a turn
so the sideways acceleration stays within the body's limit.

Speeding up is also jerk-limited (max_jerk): the push builds up and eases off, so people
aboard aren't thrown back. Slowing down never is: a stop, from the collision monitor or a
silent driver, starts at once.

While a person drives, the pedal (pedal.py) grades how hard a slow-down is: letting the
stick off slows gently, pulling it back brakes by how far. A stop (no fresh command) and a
turn's cap still brake at max_deceleration.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Optional, Tuple

from aav_ego import dynamics
from aav_ego.dynamics import Body
from aav_ego.pedal import Pedal, slow_rate


@dataclass
class Limits:
    wheelbase: float = 1.57              # m, rear axle to front axle
    max_steering_angle: float = 0.5      # rad, either side
    max_speed: float = 1.0               # m/s forward (set per profile)
    max_reverse_speed: float = 0.5       # m/s backwards (vehicle.yaml sets it)
    max_acceleration: float = 1.0        # m/s^2 while speeding up
    max_deceleration: float = 2.0        # m/s^2 while slowing down
    max_steering_rate: float = 0.6       # rad/s
    min_speed_for_steering: float = 0.05  # m/s; below this the steering holds
    max_jerk: float = 0.0                # m/s^3 while speeding up; 0: no limit


# What a running ego lets anyone change (ros2 param set /ego ...), and its range.
# max_steering_rate: the actuator tests lift it for test 3.11, on stands, so they time the
# steering motor, not ego's own ramp; they put it back afterwards. payload: when people get
# in or out (dynamics.LIVE_BODY). Everything else is read at start-up.
LIVE_LIMITS = {'max_steering_rate': (0.05, 5.0),   # rad/s
               **dynamics.LIVE_BODY}


def live_change(name: str, value) -> Optional[str]:
    """Why ego refuses to change this parameter while running, or None if it may."""
    if name in LIVE_LIMITS:
        lo, hi = LIVE_LIMITS[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return f'{name} must be a number'
        if not lo <= value <= hi:
            return f'{name} must be between {lo} and {hi}'
        return None
    if name in vars(Limits()) or name in vars(Body()):
        return f'{name} is read only at start-up: restart ego to change it'
    return None


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def steering_angle(v: float, w: float, lim: Limits, hold: float) -> float:
    """Return the front-wheel angle for (v, w), or `hold` if the car is too slow to tell."""
    if abs(v) < lim.min_speed_for_steering:
        return hold
    d = math.atan(lim.wheelbase * w / v)
    return clamp(d, -lim.max_steering_angle, lim.max_steering_angle)


def turn_rate(v: float, d: float, wheelbase: float) -> float:
    """Return the yaw rate of the bicycle model, the inverse of steering_angle."""
    return v * math.tan(d) / wheelbase


def slew_speed(current: float, target: float, lim: Limits, dt: float,
               speed_up: Optional[float] = None,
               slow_down: Optional[float] = None) -> Tuple[float, float]:
    """
    Move the speed toward target within the limits; return (speed, rate used).

    speed_up (m/s^2), if given, replaces lim.max_acceleration: the body's lower limit.
    slow_down (m/s^2), if given, replaces lim.max_deceleration: the pedal's (pedal.py).
    """
    if current != 0.0 and target != 0.0 and (current > 0.0) != (target > 0.0):
        target = 0.0  # change direction through a stop
    speeding_up = abs(target) > abs(current)
    up = lim.max_acceleration if speed_up is None else speed_up
    down = lim.max_deceleration if slow_down is None else min(slow_down, lim.max_deceleration)
    rate = up if speeding_up else down
    step = rate * dt
    if target > current:
        return min(target, current + step), rate
    return max(target, current - step), rate


def ease_speed(current: float, target: float, accel: float, rate: float, jerk: float,
               dt: float) -> Tuple[float, float]:
    """
    Speed up toward target, the acceleration changing at most `jerk` m/s^3; return (speed, accel).

    accel is the signed acceleration of the last tick. It never passes `rate`, and eases off
    near the target: from acceleration a, easing off one jerk step a tick still gains
    a^2 / (2 jerk) + a dt / 2 m/s, so a is kept where that fits in what is left.
    """
    gap = target - current
    if gap == 0.0:
        return current, 0.0
    sign = 1.0 if gap > 0.0 else -1.0
    held = max(0.0, accel * sign)          # the push already in this direction
    ease_off = jerk * (math.sqrt((dt / 2.0) ** 2 + 2.0 * abs(gap) / jerk) - dt / 2.0)
    a = min(held + jerk * dt, rate, max(ease_off, held - jerk * dt))
    step = min(a * dt, abs(gap))
    return current + sign * step, sign * a


def curvature(steering: float, wheelbase: float) -> float:
    """Return how sharply the path bends (1/m) for a front-wheel angle: tan of it over L."""
    return math.tan(steering) / wheelbase


class EgoModel:
    """The car as ego sees it: the speed and steering it last commanded."""

    def __init__(self, limits: Limits, body: Optional[Body] = None):
        self.lim = limits
        self.body = body
        self.speed = 0.0
        self.steering = 0.0
        self.accel = 0.0         # m/s^2, signed: the last tick's change of speed
        # The pedal (pedal.py) while a person drives, else None: ego_node sets it each tick.
        self.pedal: Optional[Pedal] = None

    def step(self, cmd: Optional[Tuple[float, float]], dt: float,
             lead: Optional[Callable[[float, float, float], Tuple[float, float]]] = None
             ) -> Tuple[float, float, float]:
        """
        Advance one tick.

        cmd is (v, w) from /control/cmd_vel, or None when it is missing or stale,
        which means stop. Returns (speed, steering angle, acceleration limit).

        lead, if given, is lag_lead.LagLead.step (speed, angle, dt) -> (speed, angle): it
        changes the request before every limit below, never a stop (cmd None).
        """
        lim = self.lim
        if cmd is None:
            target_v, target_d = 0.0, self.steering
        else:
            v, w = cmd
            # Keep the requested curvature w / v even when the speed is capped.
            target_d = steering_angle(v, w, lim, self.steering)
            if lead is not None:
                v, target_d = lead(v, target_d, dt)
                target_d = clamp(target_d, -lim.max_steering_angle, lim.max_steering_angle)
            target_v = clamp(v, -lim.max_reverse_speed, lim.max_speed)
        asked = target_v
        speed_up = None
        if self.body is not None:
            speed_up = min(lim.max_acceleration, dynamics.speed_up_limit(self.body))
            # The sharper of the turn now and the one the wheels are heading for: slow down
            # before the wheels get there, and speed up only once they've come back.
            k = max(abs(curvature(target_d, lim.wheelbase)),
                    abs(curvature(self.steering, lim.wheelbase)))
            cap = dynamics.curve_speed(self.body, k)
            target_v = clamp(target_v, -cap, cap)
        # The pedal grades the slowing a person asks for; never a stop (cmd None) or a turn's
        # cap (target_v below what was asked): those brake at max_deceleration.
        slow_down = None
        if self.pedal is not None and cmd is not None and target_v == asked:
            slow_down = slow_rate(self.pedal, self.speed, cmd[0], lim.max_deceleration)
        before = self.speed
        self.speed, accel = slew_speed(self.speed, target_v, lim, dt, speed_up, slow_down)
        if lim.max_jerk > 0.0 and abs(self.speed) > abs(before):
            # Speeding up, so toward target_v (a change of direction stops first).
            self.speed, signed = ease_speed(before, target_v, self.accel, accel,
                                            lim.max_jerk, dt)
            accel = abs(signed)
        self.accel = (self.speed - before) / dt if dt > 0.0 else 0.0
        step = lim.max_steering_rate * dt
        self.steering = clamp(target_d, self.steering - step, self.steering + step)
        return self.speed, self.steering, accel
