"""
One drive to B, from the plan to the result: every tick's stops and limits, and the end.

No ROS here. point_driver.py makes a `Run` once a goal is accepted and asks `Run.tick`
on every timer tick; it publishes what the tick says and ends the goal with `outcome`.
The Rust twin, aav_drive_to_point_rs's point-core, gives the same answers
(gen_vectors.py writes the shared vectors; test_rust_vectors.py checks them here).

| Step | Function |
|---|---|
| ego's caps and the goal's numbers into a speed plan | `plan` |
| one tick: stop, carry on or finish | `Run.tick` |
| the result's words | `Run.outcome` |

A tick, in order (the first that fires ends it):

| Check | Then |
|---|---|
| stopping or cancelled already | a zero command; finished after `ZEROS` of them |
| a cancel is asked for | cancelled, nothing sent this tick |
| `rules.stop_reason` | stop |
| further than `max_path`, longer than `max_time`, faster than the cruise allows | stop |
| outside the corridor (`line.Fence`), unless standing still at B | stop |
| asked to move but standing still for `stuck_time` | stop |
| otherwise | the drive's command; at B and still for `still_time`: finished |
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from aav_description import stop_law
from aav_drive_to_point import rules
from aav_drive_to_point.approach import Approach, planned_deceleration, STOPPING
from aav_drive_to_point.drive import Command, Drive, Steering
from aav_drive_to_point.line import Fence, Point, Pose

ZEROS = 5             # zero commands at the end, so twist_mux and ego see the stop
FEEDBACK_EVERY = 5    # ticks between feedback messages


@dataclass(frozen=True)
class Settings:
    """The point driver's numbers for a drive (config/drive_to_point.yaml, vehicle.yaml)."""

    cruise_speed: float = 1.0
    acceleration: float = 0.5
    max_brake: float = 3.0
    brake_fraction: float = 0.5
    deceleration: float = 0.0
    brake_lead: float = 0.8
    stop_band: float = 0.05
    creep: float = 0.1
    lookahead_min: float = 2.0
    lookahead_time: float = 1.0
    lateral_accel: float = 1.5
    still_speed: float = 0.05
    stuck_time: float = 5.0
    still_time: float = 1.0
    max_off_line: float = 3.0
    max_overrun: float = 2.0


def plan(cruise: float, accel: float, decel: float, caps: dict, s: Settings,
         car=None) -> Tuple[Optional[Approach], Optional[str]]:
    """
    Return (the Approach, None) for a goal's numbers (0 = default) and ego's caps.

    Or (None, why not). car: the car model's ActuatorConfig for the stopping law
    (stop_law:=hybrid), or None for approach's own law.
    """
    why = rules.check_numbers(cruise, accel, decel)
    if why:
        return None, why
    cruise = cruise or s.cruise_speed
    accel = accel or s.acceleration
    decel = decel or s.deceleration or planned_deceleration(s.max_brake, s.brake_fraction)
    # Ego's caps: it would cut a faster command anyway, and the plan must know.
    # Without ego, the real car's 1 m/s (ego.yaml).
    cruise = min(cruise, caps.get('max_speed', 1.0))
    accel = min(accel, caps.get('max_acceleration', accel))
    decel = min(decel, caps.get('max_deceleration', decel))
    curve = None if car is None else stop_law.from_car_model(
        car, caps.get('max_deceleration', 2.0), caps.get('max_jerk', 2.0))
    return Approach(cruise=cruise, accel=accel, decel=decel, lead=s.brake_lead,
                    stop_band=s.stop_band, creep=min(s.creep, cruise), curve=curve), None


@dataclass(frozen=True)
class Tick:
    """What one tick says to do."""

    send: Optional[Tuple[float, float]] = None   # (speed m/s, curvature 1/m) to publish
    stopped: Optional[str] = None     # the drive stops now, and why (log it)
    feedback: Optional[Command] = None   # send feedback with this command (and the speed)
    finished: bool = False            # the drive is over: end the goal


@dataclass(frozen=True)
class Outcome:
    """How a drive ended: the goal's result."""

    status: str                 # 'succeeded', 'aborted' or 'cancelled'
    success: bool
    message: str
    left: float                 # m from B, straight
    tolerance: float            # m: how close counts as at B
    travelled: float            # m, from the speed
    duration: float             # s


class Run:
    """One drive: the Drive, its corridor and limits, and what happened so far."""

    def __init__(self, target: Point, approach: Approach, s: Settings, limits: rules.Limits,
                 radius: float, start: Pose, t0: float):
        self.target = target
        self.approach = approach
        self.s = s
        self.limits = limits
        self.drive = Drive(target, approach,
                           Steering(max_curvature=1.0 / radius, lookahead_min=s.lookahead_min,
                                    lookahead_time=s.lookahead_time,
                                    lateral_accel=s.lateral_accel), start)
        # The drive's own corridor (line.Fence, TODO 187), apart from the site's geofence
        # and from Nav2: a wrong steering sign or a bad pose can't take the car far.
        self.corridor = Fence((start.x, start.y), target, turn_radius=radius,
                              max_off_line=s.max_off_line, max_overrun=s.max_overrun)
        self.distance = math.hypot(target[0] - start.x, target[1] - start.y)
        # Limits for the whole drive, from the plan: a turn round costs up to a circle.
        self.max_path = 2.0 * self.distance + 2.0 * math.pi * radius + 10.0
        self.max_time = 2.0 * approach.ideal_time(self.distance + math.pi * radius) + 30.0
        self.t0 = self.last = t0
        self.ticks = 0
        self.zeros = 0
        self.still: Optional[float] = None
        self.stuck: Optional[float] = None
        self.path = 0.0
        self.stop: Optional[str] = None
        self.cancel = False
        self.cmd: Optional[Command] = None
        self.done = False

    def halt(self, reason: str) -> Tick:
        """Stop the drive (once; never after a cancel)."""
        if self.stop is None and not self.cancel:
            self.stop = reason
            return Tick(stopped=reason)
        return Tick()

    def finish(self) -> bool:
        """Return True the first time the drive is over."""
        if self.done:
            return False
        self.done = True
        return True

    def tick(self, t: float, now: rules.Now, pose: Pose, measured: float,
             cancel_asked: bool) -> Tick:
        """
        One tick at time t (s, the node's clock): what to send.

        now: what the driver knows (rules.Now). pose, measured: the global EKF's pose and
        speed. cancel_asked: the goal's cancel was asked for.
        """
        s = self.s
        if self.stop is not None or self.cancel:
            self.zeros += 1
            return Tick(send=(0.0, 0.0), finished=self.zeros >= ZEROS and self.finish())
        if cancel_asked:
            self.cancel = True
            return Tick()
        dt, self.last = min(max(t - self.last, 0.0), 0.5), t
        why = rules.stop_reason(now, self.limits)
        if why:
            return self.halt(why)
        # The distance driven, from the speed: the global EKF's position jitters with
        # the GPS, and summing its steps counted double (sim_check, 15 Oct).
        self.path += abs(measured) * dt
        if self.path > self.max_path:
            return self.halt(f'drove {self.path:.0f} m, over the limit of {self.max_path:.0f} m')
        if t - self.t0 > self.max_time:
            return self.halt(f'ran {t - self.t0:.0f} s, over the limit of {self.max_time:.0f} s')
        if abs(measured) > self.approach.cruise * 1.2 + 0.3:
            return self.halt(f'speed {measured:.2f} m/s, over the cruise speed '
                             f'{self.approach.cruise:g} m/s')
        # The corridor, every tick until the car stands still at B: then it asks for
        # nothing more, and a GPS jump there shouldn't turn a good stop into an abort.
        standing_at_b = (self.cmd is not None and self.cmd.phase == STOPPING
                         and abs(measured) < s.still_speed)
        why = None if standing_at_b else self.corridor.breach((pose.x, pose.y),
                                                              self.drive.line)
        if why:
            return self.halt(why)
        cmd = self.drive.tick(pose, measured, dt)
        self.cmd = cmd
        # Asked to move, standing still: something holds the car (a lock this driver
        # can't see, a brake, a wall). Stop asking.
        if rules.not_moving(cmd.speed, measured, s.still_speed):
            self.stuck = self.stuck if self.stuck is not None else t
            if t - self.stuck > s.stuck_time:
                return self.halt(f'the car stood still for {s.stuck_time:g} s though asked '
                                 'to move: something holds it')
        else:
            self.stuck = None
        finished = False
        if cmd.phase == STOPPING:
            if abs(measured) < s.still_speed:
                self.still = self.still if self.still is not None else t
                finished = t - self.still >= s.still_time and self.finish()
            else:
                self.still = None
        feedback = cmd if self.ticks % FEEDBACK_EVERY == 0 else None
        self.ticks += 1
        return Tick(send=(cmd.speed, cmd.curvature), feedback=feedback, finished=finished)

    def outcome(self, pose: Pose, fix_status: Optional[int], require_gps: bool,
                t: float) -> Outcome:
        """Return the result, with the car at `pose` at time t."""
        left = math.hypot(self.target[0] - pose.x, self.target[1] - pose.y)
        tolerance = rules.tolerance(fix_status if require_gps else rules.STATUS_GBAS_FIX)
        if self.cancel:
            status, success = 'cancelled', False
            message = f'cancelled; stopped {left:.2f} m from B'
        elif self.stop is not None:
            status, success = 'aborted', False
            message = f'stopped {left:.2f} m from B: {self.stop}'
        else:
            success = left <= tolerance
            status = 'succeeded' if success else 'aborted'
            message = (f'stopped {left:.2f} m from B (tolerance {tolerance:g} m)'
                       + ('' if success else ': too far'))
        return Outcome(status, success, message, left, tolerance, self.path, t - self.t0)
