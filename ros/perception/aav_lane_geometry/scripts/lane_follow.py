"""
The 2025-26 lane follower's logic, with no ROS imports (aav, 19 Nov 2026).

Moved here from control_interface.py, unchanged, so a test can drive it with made-up times
and aav_lane_follower_rs (its Rust twin) can be checked against its answers
(that package's gen_vectors.py). control_interface.py is now the ROS shell around it.

Two clocks, as the node had them:

  wall  time.time(): the PID's dt, the offset's rate, how long a state has lasted;
  now   the node's clock (the simulator's under use_sim_time): when the last lane message
        came, the watchdog's ticks, when the last command went out.

    follower = LaneFollower(Settings(), wall, now)
    reply = follower.on_geometry(msg, wall, now)   # msg: anything with LaneGeometry's fields
    reply = follower.tick(wall, now)               # every 0.1 s
    reply.commands      # [(linear_x m/s, angular_z rad/s)] to publish, in order
    reply.status        # the line for /drivers/ai/lane_status, or None
    reply.state         # for /drivers/ai/lane_state (on_geometry only), or None
    reply.transitions   # [(old, new)] state changes, for the log
    reply.quiet_for     # s without a lane message, when the watchdog trips (tick only)
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from lane_speed import stopping_speed

LANE_KEEP = 'LANE_KEEP'
LANE_CHANGING = 'LANE_CHANGING'
LANE_COMMITTED = 'LANE_COMMITTED'
NO_LANE_STOPPING = 'NO_LANE_STOPPING'
RESUMING = 'RESUMING'


@dataclass
class Settings:
    """control_interface's parameters, with its defaults."""

    base_speed_mps: float = 2.0
    max_angular_z: float = 0.5
    min_speed_mps: float = 0.3
    min_curvature_m: float = 20.0
    lane_change_rate_threshold: float = 0.05
    lane_change_offset_threshold: float = 0.40
    lane_change_speed: float = 1.0
    lane_change_timeout_s: float = 8.0
    stop_wait_s: float = 0.3
    decel_rate: float = 0.5
    accel_rate: float = 0.3
    pid_kp: float = 0.8
    pid_ki: float = 0.01
    pid_kd: float = 0.3
    invert_offset_sign: bool = True


class PID:
    """The 2025-26 PID; its dt from the wall clock."""

    def __init__(self, kp, ki, kd, wall):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_time = wall

    def compute(self, error: float, wall: float) -> float:
        dt = max(wall - self._prev_time, 1e-4)
        self._integral += error * dt
        derivative = (error - self._prev_err) / dt
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._prev_err = error
        self._prev_time = wall
        return out

    def reset(self, wall):
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_time = wall


@dataclass
class Reply:
    """What the node publishes and logs after one message or tick."""

    commands: List[Tuple[float, float]] = field(default_factory=list)
    status: Optional[str] = None
    state: Optional[str] = None
    transitions: List[Tuple[str, str]] = field(default_factory=list)
    quiet_for: Optional[float] = None


def clip(value, low, high):
    """Return the value kept between low and high, as numpy's clip does; NaN stays NaN."""
    return min(max(value, low), high) if value == value else value


class LaneFollower:
    """The PID on the lane offset and its five states (control_interface, 2025-26)."""

    def __init__(self, settings: Settings, wall: float, now: float):
        self.s = settings
        self.pid = PID(settings.pid_kp, settings.pid_ki, settings.pid_kd, wall)
        self.state = LANE_KEEP
        self.current_speed = 0.0
        self.last_steer = 0.0
        self.last_offset = 0.0
        self.offset_rate = 0.0
        self.state_entry_t = wall
        self.last_msg_t = now
        self._tick_t = now
        self._sent_t = now
        self.lc_peak_offset = 0.0
        self._last_t = wall
        self._reply = Reply()

    # The message and the timer.

    def on_geometry(self, msg, wall: float, now: float) -> Reply:
        self._reply = Reply()
        self.last_msg_t = now
        dt = max(wall - self._last_t, 1e-4)
        self._last_t = wall
        self.offset_rate = (msg.vehicle_offset_m - self.last_offset) / dt
        self.last_offset = msg.vehicle_offset_m
        lanes_ok = msg.left_lane_detected or msg.right_lane_detected

        if self.state == LANE_KEEP:
            self.handle_lane_keep(msg, lanes_ok, wall, now)
        elif self.state == LANE_CHANGING:
            self.handle_lane_changing(msg, lanes_ok, wall, now)
        elif self.state == LANE_COMMITTED:
            self.handle_lane_committed(msg, lanes_ok, wall, now)
        elif self.state == NO_LANE_STOPPING:
            self.handle_no_lane_stopping(msg, lanes_ok, wall, now)
        elif self.state == RESUMING:
            self.handle_resuming(msg, lanes_ok, wall, now)
        self._reply.state = self.state
        return self._reply

    def tick(self, wall: float, now: float) -> Reply:
        """
        Run the watchdog: every 0.1 s, on the node's clock.

        No message for stop_wait_s slows the car at decel_rate, as lost lines do. The
        slow-down runs here, so it goes on when the messages stop
        (lane_speed.stopping_speed).
        """
        self._reply = Reply()
        dt = now - self._tick_t
        self._tick_t = now
        elapsed = now - self.last_msg_t
        if elapsed > self.s.stop_wait_s and self.state != NO_LANE_STOPPING:
            self._reply.quiet_for = elapsed
            self.transition(NO_LANE_STOPPING, wall)
        if self.state == NO_LANE_STOPPING and self.current_speed > 0.0:
            self.current_speed = stopping_speed(self.current_speed, self.s.decel_rate, dt,
                                                now - self._sent_t)
            steer = self.last_steer if self.current_speed > 0.0 else 0.0
            self.send(self.current_speed, steer, now)
        return self._reply

    # The states.

    def handle_lane_keep(self, msg, lanes_ok, wall, now):
        if not lanes_ok:
            self.transition(NO_LANE_STOPPING, wall)
            return
        offset_large = abs(msg.vehicle_offset_m) > self.s.lane_change_offset_threshold
        offset_fast = abs(self.offset_rate) > self.s.lane_change_rate_threshold
        if offset_large and offset_fast:
            self.lc_peak_offset = abs(msg.vehicle_offset_m)
            self.transition(LANE_CHANGING, wall)
            return
        self.send(*self.build_cmd_lane_keep(msg, wall), now)
        self.publish_status(msg, 'LANE_KEEP')

    def handle_lane_changing(self, msg, lanes_ok, wall, now):
        elapsed = wall - self.state_entry_t
        if elapsed > self.s.lane_change_timeout_s or not lanes_ok:
            self.transition(NO_LANE_STOPPING, wall)
            return
        curr_abs = abs(msg.vehicle_offset_m)
        self.lc_peak_offset = max(self.lc_peak_offset, curr_abs)
        if (curr_abs < self.lc_peak_offset * 0.5
                and curr_abs < self.s.lane_change_offset_threshold * 0.5):
            self.transition(LANE_COMMITTED, wall)
            return
        self.send(self.s.lane_change_speed, self.last_steer, now)
        self.current_speed = self.s.lane_change_speed
        self.publish_status(msg, f'LANE_CHANGING peak={self.lc_peak_offset:.2f}m')

    def handle_lane_committed(self, msg, lanes_ok, wall, now):
        self.pid.reset(wall)
        self.transition(LANE_KEEP, wall)
        self.handle_lane_keep(msg, lanes_ok, wall, now)

    def handle_no_lane_stopping(self, msg, lanes_ok, wall, now):
        if lanes_ok:
            self.transition(RESUMING, wall)
            return
        elapsed = wall - self.state_entry_t
        if self.current_speed > 0.0:
            # tick() slows the car, at decel_rate, whether messages come or not.
            self.publish_status(msg, f'NO_LANE decelerating t={elapsed:.1f}s')
        else:
            self.send(0.0, 0.0, now)
            self.publish_status(msg, f'STOPPED waiting t={elapsed:.1f}s')

    def handle_resuming(self, msg, lanes_ok, wall, now):
        if not lanes_ok:
            self.transition(NO_LANE_STOPPING, wall)
            return
        self.current_speed = min(self.s.base_speed_mps,
                                 self.current_speed + self.s.accel_rate * 0.1)
        # As in 2025-26: build_cmd_lane_keep sets current_speed to the lane-keeping speed,
        # so the command is that speed, not the ramp's (aav_lane_follower_rs keeps this).
        _, steer = self.build_cmd_lane_keep(msg, wall)
        self.send(self.current_speed, steer, now)
        self.publish_status(msg, f'RESUMING {self.current_speed:.2f}/{self.s.base_speed_mps:.2f}')
        if self.current_speed >= self.s.base_speed_mps * 0.95:
            self.transition(LANE_KEEP, wall)

    # Helpers.

    def send(self, linear_x, angular_z, now):
        self._reply.commands.append((float(linear_x), float(angular_z)))
        self._sent_t = now

    def build_cmd_lane_keep(self, msg, wall):
        """Return (linear_x, angular_z): the lane-keeping speed and the PID's turn."""
        speed = self.s.base_speed_mps
        if not msg.both_lanes_detected:
            speed *= 0.7
        if 0.0 < msg.mean_curvature_m < self.s.min_curvature_m:
            speed *= max(0.4, msg.mean_curvature_m / self.s.min_curvature_m)
        linear_x = float(max(self.s.min_speed_mps, speed))
        self.current_speed = linear_x
        error = -msg.vehicle_offset_m if self.s.invert_offset_sign else msg.vehicle_offset_m
        steer = self.pid.compute(error, wall)
        angular_z = float(clip(steer, -self.s.max_angular_z, self.s.max_angular_z))
        self.last_steer = angular_z
        return linear_x, angular_z

    def transition(self, new_state, wall):
        if self.state != new_state:
            self._reply.transitions.append((self.state, new_state))
        self.state = new_state
        self.state_entry_t = wall
        if new_state in (LANE_KEEP, LANE_COMMITTED):
            self.pid.reset(wall)

    def publish_status(self, msg, extra):
        self._reply.status = (
            f'[{self.state}] offset={msg.vehicle_offset_m:+.3f}m '
            f'curv={msg.mean_curvature_m:.1f}m speed={self.current_speed:.2f}m/s {extra}')
