"""
The point driver's rules: when a drive may start, when it must stop, and how close is close.

No ROS here. point_driver.py asks these on every goal and every tick, and stops the car
(zero commands) whenever one gives a reason. Every rule errs towards "this is the real car":
/system/source's kinds come from the actuator tests' rules (aav_actuator_tests safety.py),
so the two agree on what REAL means.

| Rule | Function |
|---|---|
| A goal's numbers are sane | check_numbers |
| May start: no e-stop or lock, a fresh position and fix, confirmed on REAL | refuse_start |
| B is near enough, and a forward-only car can reach it | refuse_target |
| Stop now: e-stop, lock or silent watchdog, stale position or fix, another driver | stop_reason |
| The car doesn't move though asked to (something holds it) | not_moving |
| How close counts as at B: 1 m with RTK, 2 m without | tolerance |
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

from aav_actuator_tests.safety import Kind
from aav_drive_to_point.line import Point, Pose, reachable

# sensor_msgs/NavSatStatus.
STATUS_NO_FIX, STATUS_FIX, STATUS_SBAS_FIX, STATUS_GBAS_FIX = -1, 0, 1, 2

RTK_TOLERANCE = 1.0     # m: docs/tests.md 5.2, with an RTK fix
GPS_TOLERANCE = 2.0     # m: without
CONFIRM_WORD = 'drive'   # what a person types to start a drive on the real car


@dataclass(frozen=True)
class Limits:
    """The driver's own limits (config/drive_to_point.yaml)."""

    max_distance: float = 200.0   # m: refuse a B further than this
    pose_timeout: float = 0.5     # s without the global EKF: stop
    gps_timeout: float = 2.0      # s without a GPS fix: stop (the watchdog's own max_age)
    require_gps: bool = True      # false only where no GPS runs at all (x/y goals)
    # s without the watchdog's lock (a heartbeat): stop. twist_mux's lock timeout, so the
    # driver stops when twist_mux holds it. 0 where no watchdog runs (sim watchdog:=false).
    watchdog_timeout: float = 1.0


@dataclass(frozen=True)
class Now:
    """What the driver knows, this moment."""

    pose_age: Optional[float]     # s since the last global EKF pose; None = never
    fix_age: Optional[float]      # s since the last GPS fix with a fix; None = never
    estop: bool                   # /safety/estop says true
    locked: bool                  # the sensor watchdog's lock says true
    kind: Kind                    # what /system/source says we drive
    taken_over: bool = False      # twist_mux passes someone else's commands
    lock_age: Optional[float] = 0.0   # s since the watchdog's lock last arrived; None = never


def check_numbers(cruise: float, accel: float, decel: float) -> Optional[str]:
    """Why a goal's speed, acceleration or deceleration is refused (0 = default), or None."""
    for name, value in (('cruise_speed', cruise), ('acceleration', accel),
                        ('deceleration', decel)):
        if not math.isfinite(value) or value < 0:
            return f'{name} must be 0 (the default) or above (got {value})'
    return None


def refuse_start(now: Now, limits: Limits, confirmed: bool) -> Optional[str]:
    """Why a drive may not start, or None."""
    if now.kind is Kind.REPLAY:
        return '/system/source says REPLAY: a recording plays; there is nothing to drive'
    if now.kind in (Kind.REAL, Kind.UNKNOWN) and not confirmed:
        who = ('nobody announced /system/source: treated as the real car'
               if now.kind is Kind.UNKNOWN else '/system/source says REAL')
        return (f'{who}. A person must confirm: pixi run drive-to-point asks you to type '
                f'"{CONFIRM_WORD}"')
    return stop_reason(now, limits)


def stop_reason(now: Now, limits: Limits) -> Optional[str]:
    """Why the car must stop now, or None."""
    if now.estop:
        return 'the e-stop is on (/safety/estop)'
    if now.locked:
        return "the sensor watchdog's lock is on: a sensor is stale, NaN, frozen or jumped"
    if limits.watchdog_timeout > 0 and (now.lock_age is None
                                        or now.lock_age > limits.watchdog_timeout):
        heard = 'never heard' if now.lock_age is None else f'silent for {now.lock_age:.1f} s'
        return (f"the sensor watchdog's lock is on: the watchdog is {heard}, so twist_mux "
                'holds every driver below 30')
    if now.pose_age is None:
        return 'no position: nothing on /localization/odom/global'
    if now.pose_age > limits.pose_timeout:
        return f'the position is stale: {now.pose_age:.1f} s old'
    if limits.require_gps:
        if now.fix_age is None:
            return 'no GPS fix on /sensors/gps/fix'
        if now.fix_age > limits.gps_timeout:
            return f'the GPS fix is stale: {now.fix_age:.1f} s old'
    if now.taken_over:
        return 'someone else is driving (twist_mux passes another driver)'
    return None


def refuse_target(pose: Pose, target: Point, limits: Limits, radius: float) -> Optional[str]:
    """Why B is refused (too far, here already, inside the turning circle), or None."""
    if not all(math.isfinite(v) for v in target):
        return 'B is not a number'
    distance = math.hypot(target[0] - pose.x, target[1] - pose.y)
    if distance > limits.max_distance:
        return (f'B is {distance:.0f} m away, over the limit of {limits.max_distance:g} m '
                '(drive_to_point.yaml max_distance)')
    if distance < 0.5:
        return f'the car is at B already ({distance:.2f} m)'
    if not reachable(pose, target, radius):
        return (f"B is inside the car's tightest turn ({radius:.1f} m radius): a car that "
                "can't reverse would circle it. Drive further first, or use Nav2")
    return None


def refuse_outside(depth: Optional[float]) -> Optional[str]:
    """
    Why B is refused for the geofence, or None.

    depth: how far B is inside /safety/geofence/fence (m, negative outside; aav_safety
    geofence.py), or None with no fence heard (the geofence's lock still holds the drive).
    """
    if depth is not None and depth < 0.0:
        return (f'B is {-depth:.1f} m outside the geofence (/safety/geofence/fence): the '
                'geofence would stop the car before it, on the way')
    return None


def not_moving(commanded: float, measured: float, still: float = 0.05,
               asked: float = 0.2) -> bool:
    """Return True if the car stands still though asked for at least `asked` m/s."""
    return commanded >= asked and abs(measured) < still


def tolerance(fix_status: Optional[int]) -> float:
    """Return how close to B counts as there: 1 m with an RTK fix (GBAS), 2 m without."""
    return RTK_TOLERANCE if fix_status is not None and fix_status >= STATUS_GBAS_FIX \
        else GPS_TOLERANCE


def confirmed(typed: Optional[str]) -> bool:
    """Return True if the person typed the confirmation word exactly (spaces aside)."""
    return typed is not None and typed.strip() == CONFIRM_WORD
