"""
The car with its lags, for the lag-aware MPC (mpc_lag): one small interface, no ROS.

The shared car model (aav_description car_model.py, with its Rust twin) is the truth: its
numbers (ActuatorConfig), its speed loops (SpeedLoopConfig), and mpc_lag's estimate
(car_model.step). This is the same maths 1-5 again, in the path's frame and on numpy
arrays (one column a state), because mpc_lag linearises it by finite differences 13 times
a stage, 30 stages a tick: car_model.step, one state at a time, would take most of the
tick. test_lag_model.py checks the two give the same states, step for step:

| Part | Model (one step dt) |
|---|---|
| speed loop | the spine's PI (spine-core speed_loop.rs; aav_gazebo SpeedLoop) |
| throttle, brake | first-order lags, exact: x += (command - x)(1 - exp(-dt / lag)) |
| speed | a = max_accel throttle (past its deadband) - max_brake brake - drag v |
| | the brake and drag never reverse v |
| steering motor | to its setpoint at most steer_rate dt a step; past steer_deadband only |
| wheels | lag the motor by steer_lag, + steer_offset, within full lock |
| the car | the kinematic bicycle about base_link, in the path's frame (Frenet); the new |
| | speed and steering, the heading at mid-step |
| dead times | floor(t / period + 0.5) ticks of the commands already sent (in mpc_lag) |

The numbers are vehicle.yaml's `actuators:` section, full lock and the wheelbase
(`CarParams.from_vehicle`). `step()` works on numpy arrays of states (one column each), so
mpc_lag linearises it by finite differences in one call a stage.

State (rows of x):
    E_Y, E_TH, S       cross track (+ left), heading error, distance along the path
    V                  speed, m/s (forward only)
    THROTTLE, BRAKE    the actuators' real levels, 0..1
    SETPOINT, MOTOR    the steering motor's target and angle, rad
    WHEELS             the wheels' angle after the linkage, before the offset, rad
    INTEGRAL           the speed loop's integral, throttle
Input (rows of u): STEER (the steering target, rad) and SPEED (the set speed, m/s).
"""

from __future__ import annotations

from dataclasses import replace
import math

from aav_description import car_model
import numpy as np

E_Y, E_TH, S, V, THROTTLE, BRAKE, SETPOINT, MOTOR, WHEELS, INTEGRAL = range(10)
NX = 10
STEER, SPEED = 0, 1
NU = 2


# The numbers: the shared car model's (aav_description car_model.ActuatorConfig, from
# vehicle.yaml), not a copy. CarParams is its name here.
CarParams = car_model.ActuatorConfig


def lags_scaled(car: CarParams, k: float) -> CarParams:
    """Return the numbers with every lag and dead time k times longer."""
    return replace(car, throttle_lag=k * car.throttle_lag, brake_lag=k * car.brake_lag,
                   steer_lag=k * car.steer_lag, throttle_dead_time=k * car.throttle_dead_time,
                   brake_dead_time=k * car.brake_dead_time,
                   steer_dead_time=k * car.steer_dead_time)


def without_deadbands(car: CarParams) -> CarParams:
    """Return the numbers with no deadbands: for a plan, whose gradients they'd flatten."""
    return replace(car, throttle_deadband=0.0, steer_deadband=0.0)


def without_dead_times(car: CarParams) -> CarParams:
    """Return the numbers with no dead times: mpc_lag keeps those itself, a tick at a time."""
    return replace(car, throttle_dead_time=0.0, brake_dead_time=0.0, steer_dead_time=0.0)


def speed_loop(name: str) -> car_model.SpeedLoopConfig:
    """Return the spine's speed loop profile by name: its PIs (default, smooth, firm) only."""
    loop = car_model.speed_loop_profile(name)
    if loop.lqi is not None:
        raise ValueError(f"the speed loop {name!r} (the LQI) isn't modelled here yet: "
                         'pick default, smooth or firm')
    return loop


def clip(x, lo, hi):
    """np.clip, without its overhead on small arrays (a third of the plan's time)."""
    return np.minimum(np.maximum(x, lo), hi)


def lag_factor(dt: float, tau: float) -> float:
    """Return the share of the way a first-order lag moves in dt: 1 - exp(-dt / tau)."""
    return 1.0 if tau <= 0.0 else 1.0 - math.exp(-dt / tau)


def dead_ticks(dead_time: float, period: float) -> int:
    """Whole ticks of a dead time: floor(t / period + 0.5), as the shared car model rounds."""
    return int(math.floor(dead_time / period + 0.5))


def pedals(loop: car_model.SpeedLoopConfig, target, measured, integral, dt):
    """
    Return (throttle, brake, integral): the spine's PI, forward only, on arrays.

    Anti-windup: the integral within 0..its ceiling ('bounds'); with `clamping`, also
    held while the throttle is pinned and the error would push it further ('clamp').
    """
    stopping = target <= 0.0
    t = np.maximum(target, 0.0)
    error = t - measured
    prop = loop.kp * (loop.setpoint_weight * t - measured)
    ceiling = np.maximum(loop.integral_limit + (1.0 - loop.setpoint_weight) * loop.kp * t, 0.0)
    grown = clip(integral + loop.ki * error * dt, 0.0, ceiling)
    if loop.anti_windup == 'clamp':
        demand = prop + grown
        pinned = ((demand > 1.0) & (error > 0.0)) | ((demand < 0.0) & (error < 0.0))
        integral = np.where(pinned, integral, grown)
    else:
        integral = grown
    overspeed = -error - loop.coast_band
    braking = overspeed > 0.0
    throttle = np.where(braking, 0.0, clip(prop + integral, 0.0, 1.0))
    brake = np.where(braking, clip(loop.brake_gain * overspeed, 0.0, 1.0), 0.0)
    integral = np.where(braking, 0.0, integral)
    # A stop: brake until stopped, then hold (the spine's rule: never coast).
    stopped = np.abs(measured) < loop.stopped_speed
    stop_brake = np.where(stopped, loop.hold_brake, clip(
        np.maximum(loop.hold_brake, loop.brake_gain * (measured - loop.coast_band)), 0.0, 1.0))
    return (np.where(stopping, 0.0, throttle), np.where(stopping, stop_brake, brake),
            np.where(stopping, 0.0, integral))


def step(car: CarParams, loop: car_model.SpeedLoopConfig, x: np.ndarray, u: np.ndarray,
         kappa: float,
         dt: float, substeps: int = 1) -> np.ndarray:
    """
    Return the state(s) dt later: x (NX,) or (NX, m), u (NU,) or (NU, m).

    kappa: the path's curvature here (1/m, + left), held over the step.
    """
    x = np.array(x, dtype=float)
    h = dt / substeps
    e_y, e_th, s, v = x[E_Y], x[E_TH], x[S], x[V]
    thr, brk, setpoint, motor, wheels, integral = (x[THROTTLE], x[BRAKE], x[SETPOINT],
                                                   x[MOTOR], x[WHEELS], x[INTEGRAL])
    steer_cmd, speed_cmd = u[STEER], u[SPEED]
    kt, kb, ks = (lag_factor(h, car.throttle_lag), lag_factor(h, car.brake_lag),
                  lag_factor(h, car.steer_lag))
    target = clip(steer_cmd, -car.max_steering, car.max_steering)
    for _ in range(substeps):
        # The speed loop, then throttle and brake through their lags.
        thr_cmd, brk_cmd, integral = pedals(loop, speed_cmd, v, integral, h)
        thr = thr + kt * (thr_cmd - thr)
        brk = brk + kb * (brk_cmd - brk)
        push = np.where(thr > car.throttle_deadband, thr, 0.0) * car.max_accel
        moved = v + push * h
        v_new = np.maximum(0.0, moved - (brk * car.max_brake + car.drag * np.abs(moved)) * h)
        # Steering: the deadband, the motor's rate, the linkage's lag, the offset.
        setpoint = np.where(np.abs(target - setpoint) > car.steer_deadband, target, setpoint)
        reach = car.steer_rate * h
        motor = motor + clip(setpoint - motor, -reach, reach)
        wheels = wheels + ks * (motor - wheels)
        delta = clip(wheels + car.steer_offset, -car.max_steering, car.max_steering)
        # The bicycle, in the path's frame: the new speed and steering, the heading at
        # mid-step (as the shared car model).
        turn = (v_new * np.tan(delta) / car.wheelbase
                - kappa * v_new * np.cos(e_th) / (1.0 - kappa * e_y)) * h
        mid = e_th + 0.5 * turn
        along = v_new * np.cos(mid) / (1.0 - kappa * e_y)
        e_y = e_y + v_new * np.sin(mid) * h
        e_th = e_th + turn
        s = s + along * h
        v = v_new
    return np.stack([e_y, e_th, s, v, thr, brk, setpoint, motor, wheels, integral])
