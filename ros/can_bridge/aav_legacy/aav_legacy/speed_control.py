"""
The legacy bridge's speed loop: feed-forward plus PI, one signed acceleration. No ROS here.

Today's ESP32 (the C++ or our Rust in its place) drives open loop: the frame's speed
number goes straight to the throttle, and its accel number, below -0.05, to the brake.
This closes the loop on the Orin, before the frame (docs/designs/2026-10-27_control-plan.md,
step 1):

    ego's speed ──> wanted acceleration (how fast ego's speed is changing) ── feed-forward
                └─> speed error ──> PI ───────────────────────────────────── feedback
                                     │
                    one signed acceleration ──> the car model, inverted ──> throttle OR brake

  - Feed-forward: ego already shapes the speed (its acceleration and jerk limits), so the
    slope of its commands is the acceleration it wants. Sent at once, before any error.
  - Feedback: PI on the wheel speed (/vehicle/odom) cleans up what the model got wrong.
    Its reference is ego's speed delayed by the actuators' lag (response_lag), so it
    corrects only what the model didn't foresee:
    slope, wind, wear. The integral is limited, runs only near the target, and is frozen
    while a pedal is pinned.
  - The model (vehicle.yaml's actuators:, guesses until tests 4.1-4.4): full throttle
    gives max_accel, full brake max_brake, and coasting loses drag x speed. A motor
    controller that holds a speed per throttle fits the same model: drag = max_accel /
    the speed at full throttle.
  - Never throttle and brake together; a small dead band between them, where it coasts.
  - The smooth stop (Autoware's design, below). The car runs behind ego, so it still rolls
    when ego reaches 0: it keeps slowing as hard as it was (no step in the brake), then,
    once smooth_stop_rate x speed is gentler, the slowing fades with the speed, so the
    car stops without a jolt. At least weak_stop_decel, so it never creeps; at least
    strong_stop_decel if it still rolls after weak_stop_time. Stopped, the brake rises
    gently to stopped_brake (firm) and holds until ego asks to move again.
  - With smooth_stop_speed 0, today's stop: brake at least `hold_brake` until stopped,
    then hold it. Never coast to a stop: the loop alone would ask for a feather of brake.
  - Without a wheel speed: feed-forward only.

Credit: the smooth stop follows the design of Autoware's pid_longitudinal_controller
(the Autoware Foundation, Apache-2.0: its smooth stop, weak then strong deceleration near
the stop, and a firm brake once stopped), read in its documentation. Our own code, not a
copy; the numbers are ours (docs/controllers.md, "Where the ideas came from").

Units: m/s, m/s^2, s. Throttle and brake are 0..1. Forward only: a target below zero
brakes to a stop (reversing is off on the car, and the legacy path keeps it that way).
"""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class SpeedControlConfig:
    """The loop's gains, and the car model it inverts."""

    # Gains: GUESSES, tried against a model car with 0.1-0.2 s of dead time (calm, within
    # 2 % holding 1 m/s). Tune on the car (docs/measure-the-car.md).
    kp: float = 2.0              # m/s^2 per m/s of speed error
    ki: float = 0.5              # m/s^2 per (m/s x s)
    integral_limit: float = 0.5  # m/s^2: the most the integral may add or take
    integral_band: float = 0.3   # m/s: integrate only this close, so a big step can't wind it up
    accel_filter: float = 0.1    # s: smooths the slope of ego's speed (it steps at 20-50 Hz)
    # s: how far behind ego the car is expected to run, as the actuators lag (the brake's
    # 0.2 s in vehicle.yaml, and the firmware's throttle ramp). The PI corrects the speed
    # against ego's delayed by this (a reference model), so it doesn't brake harder to win
    # back a lag the feed-forward already allows for. 0: none. 0.3 s held a 3 m/s stop to
    # its planned 1.5 m/s^2 through the C++ firmware's model; 0 gave 1.85.
    response_lag: float = 0.3
    dead_band: float = 0.05      # m/s^2 either side of zero: coast, neither pedal
    stopped_speed: float = 0.05  # m/s: slower counts as stopped
    hold_brake: float = 0.2      # brake (0..1) held while stopped with a zero target
    # The smooth stop (Autoware's design, above): tried on the bridge -> C++ firmware ->
    # actuator model chain (test_legacy_chain.py): it halves the deceleration at the moment
    # the car stops (the jolt), for a few cm more rolling. GUESSES: tune on the car.
    smooth_stop_speed: float = 0.3    # m/s: ego and the car this slow: slowing fades (0: off)
    smooth_stop_rate: float = 1.5     # 1/s: slowing at most this x the speed near the stop
    weak_stop_decel: float = 0.2      # m/s^2: slowing at least this much, so it never creeps
    weak_stop_time: float = 2.0       # s: ego at 0 and still rolling this long: ...
    strong_stop_decel: float = 1.0    # m/s^2: ... then at least this much (a slope)
    stopped_brake: float = 0.5        # brake (0..1) held once stopped, until ego asks again
    stopped_brake_rate: float = 1.0   # brake (0..1) per s: how fast it rises to stopped_brake
    # The car model: vehicle.yaml actuators:.
    max_accel: float = 1.5       # m/s^2 at full throttle
    max_brake: float = 3.0       # m/s^2 at full brake
    drag: float = 0.1            # 1/s: m/s lost per second per m/s, coasting


@dataclass(frozen=True)
class Pedals:
    """One tick's output: throttle and brake, 0..1, never both above 0."""

    throttle: float
    brake: float
    accel: float           # m/s^2: the signed acceleration asked of the car
    feedforward: float     # m/s^2: of which, from ego's slope
    integral: float        # m/s^2: of which, the integral


def clamp(x: float, low: float, high: float) -> float:
    return min(high, max(low, x))


class SpeedControl:
    """Feed-forward plus PI on wheel speed, out as throttle or brake."""

    def __init__(self, cfg: SpeedControlConfig = SpeedControlConfig()):
        self.cfg = cfg
        self.integral = 0.0
        self.feedforward = 0.0
        self.last_target: Optional[float] = None
        self.expected: Optional[float] = None   # ego's speed, delayed by response_lag
        self.standstill = False   # stopped on ego's 0: hold stopped_brake until it asks again
        self.rolling = 0.0        # s since ego's 0, the car still moving
        self.stop_decel: Optional[float] = None   # m/s^2 the smooth stop started from
        self.last_accel = 0.0     # m/s^2 asked last tick
        self.last_brake = 0.0     # brake (0..1) sent last tick

    def reset(self) -> None:
        """Forget the past: on a stale command, an e-stop, or a new drive."""
        self.integral = 0.0
        self.feedforward = 0.0
        self.last_target = None
        self.expected = None
        self.standstill = False
        self.rolling = 0.0
        self.stop_decel = None
        self.last_accel = self.last_brake = 0.0

    def update(self, target: float, measured: Optional[float], dt: float) -> Pedals:
        """
        Return this tick's pedals.

        target is ego's speed now (m/s), measured the wheel speed (None if there is none
        fresh), dt the time since the last update (s).
        """
        c = self.cfg
        if dt <= 0.0:
            raise ValueError(f'dt must be positive, not {dt}')
        target = max(0.0, target)
        slope = 0.0 if self.last_target is None else (target - self.last_target) / dt
        self.last_target = target
        self.feedforward += (slope - self.feedforward) * clamp(dt / max(c.accel_filter, dt),
                                                               0.0, 1.0)

        # The speed the car should have now, given the actuators' lag: the PI's reference.
        if self.expected is None:
            self.expected = target
        else:
            self.expected += (target - self.expected) * clamp(
                dt / max(c.response_lag, dt), 0.0, 1.0)

        if measured is None:
            # Open loop: assume the car is where ego says, and trust the model alone.
            self.integral = 0.0
            speed = target
        else:
            speed = max(0.0, measured)

        stopping = target < c.stopped_speed
        smooth = c.smooth_stop_speed > 0.0 and measured is not None
        if not stopping:
            self.standstill = False
            self.rolling = 0.0
            self.stop_decel = None
        if stopping and smooth:
            self.integral = 0.0
            self.feedforward = 0.0
            return self.remember(self.smooth_stop(speed, dt))
        if stopping:
            self.integral = 0.0
            self.feedforward = 0.0
            if speed < c.stopped_speed:
                return self.remember(Pedals(0.0, c.hold_brake, 0.0, 0.0, 0.0))

        error = self.expected - speed if measured is not None else 0.0
        integral = self.integral
        if abs(error) < c.integral_band and not stopping:
            integral = clamp(integral + c.ki * error * dt, -c.integral_limit, c.integral_limit)
        accel = self.feedforward + c.kp * error + integral
        if smooth and target < c.smooth_stop_speed and speed < c.smooth_stop_speed:
            # Nearly stopped, ego nearly at 0: slowing fades with the speed already.
            accel = max(accel, -max(c.weak_stop_decel, c.smooth_stop_rate * speed))
        throttle, brake = self.pedals(accel, speed)
        if stopping:
            throttle, brake = 0.0, max(brake, c.hold_brake)
        # Anti-windup: keep the new integral unless a pedal is pinned and it would grow more.
        pinned = (throttle >= 1.0 and error > 0.0) or (brake >= 1.0 and error < 0.0)
        if not pinned:
            self.integral = integral
        return self.remember(Pedals(throttle, brake, accel, self.feedforward, self.integral))

    def remember(self, p: Pedals) -> Pedals:
        """Keep this tick's asked acceleration and brake: the smooth stop starts from them."""
        self.last_accel, self.last_brake = p.accel, p.brake
        return p

    def smooth_stop(self, speed: float, dt: float) -> Pedals:
        """
        Return the pedals once ego says 0 and the car still rolls: the smooth stop.

        The car runs behind ego (the actuators' lag), so it is still moving when ego
        reaches 0. Keep slowing as hard as it was (no step in the brake), until slowing
        smooth_stop_rate x speed is gentler: from there the deceleration fades with the
        speed, and the car stops without a jolt. Never under weak_stop_decel, so it can't
        creep; and at least strong_stop_decel once it has rolled weak_stop_time (a slope, a
        weak brake). Stopped, the brake rises to stopped_brake at stopped_brake_rate, and
        holds there until ego asks to move again.
        """
        c = self.cfg
        if self.standstill or speed < c.stopped_speed:
            self.standstill = True
            # Up to stopped_brake gently (a step would jolt the car as it stops); never less.
            brake = max(self.last_brake,
                        min(c.stopped_brake, self.last_brake + c.stopped_brake_rate * dt))
            return Pedals(0.0, brake, 0.0, 0.0, 0.0)
        if self.stop_decel is None:
            self.stop_decel = max(c.weak_stop_decel, -self.last_accel)
        self.rolling += dt
        weakest = c.weak_stop_decel
        if self.rolling > c.weak_stop_time:
            weakest = max(weakest, c.strong_stop_decel)
        decel = clamp(c.smooth_stop_rate * speed, weakest, max(self.stop_decel, weakest))
        _, brake = self.pedals(-decel, speed)
        return Pedals(0.0, brake, -decel, 0.0, 0.0)

    def pedals(self, accel: float, speed: float):
        """Invert the model: the throttle or brake that gives `accel` at `speed`."""
        c = self.cfg
        # Coasting already slows the car by drag x speed: the pedals make up the rest.
        push = accel + c.drag * speed
        if push > c.dead_band:
            return clamp(push / c.max_accel, 0.0, 1.0), 0.0
        if push < -c.dead_band:
            return 0.0, clamp(-push / c.max_brake, 0.0, 1.0)
        return 0.0, 0.0
