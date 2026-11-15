"""
The car model: how the car answers a command.

Shared by ego (its lead and its trace), the simulators (Gazebo's adapter) and the drivers'
tests. No ROS here.

Moved here from aav_gazebo's actuator_model.py on 7 Nov (release 0.3), so ego and the
simulators use one model; `aav_gazebo.actuator_model` still re-exports every name. Its Rust
twin is the car-model crate beside this package (car-model/, the same maths, checked by
the shared vectors car-model/vectors.json: gen_vectors.py writes them from this file).

Two ways in:

  - step(cfg, state, command, dt) -> CarState, and predict(cfg, state, commands, dt,
    horizon) -> [CarState]: the pure model, with dead times and the kinematic bicycle on
    top (x, y, yaw). ego's trace is predict's list.
  - Actuators(cfg, loop): the same step held in an object, as gazebo_adapter steps it at
    50 Hz (Gazebo's time) and sends Gazebo the speed and steering it gives
    (docs/full-simulation.md, option A):

    ego's speed ──> SpeedLoop ──> throttle %, brake % ──> Actuators ──> speed  ──> Gazebo
    ego's angle ─────────────────────────────────────> Actuators ──> angle  ──> Gazebo

  - SpeedLoop: a stand-in for the spine's speed loop (spine-core speed_loop.rs), with its
    gains, until the spine itself runs in the loop (step 4). Its profiles ('default',
    'smooth': TODO 27) are spine-core's; firmware/spine/speed_loop_vectors.txt checks both.
  - Actuators: throttle % -> acceleration, brake % -> deceleration, coasting drag; the
    steering motor's speed limit, the linkage's lag, the centre offset and a deadband.
    Its numbers are vehicle.yaml's `actuators:` section (guesses until tests 3.10-3.12
    and 4.1-4.5 in docs/tests.md).
  - Ideal: the old behaviour, for comparison (actuator_model:=false): the speed follows
    the command within Gazebo's old limits (+1.5 / -3.0 m/s^2), the steering at once.

With the spine in the loop (sim.launch.py spine:=true, step 4), the spine's own speed
loop runs in spine_sim (spine-core, Rust), and its throttle, brake, gear and steering
target go straight to Actuators.drive, skipping SpeedLoop:

    spine_sim ──> throttle %, brake %, reverse, steering target ──> Actuators.drive ──> Gazebo

The maths of one step of dt (docs/vehicle-model.md, "One car model"):

  1. Dead time: each input waits n = floor(dead_time / dt + 0.5) steps in a delay line
     (throttle with its gear, brake, steering target). 0 (the default): no wait.
  2. Throttle and brake: each level follows its input with a first-order lag,
     level' = (u - level) / lag, stepped exactly: level += (u - level)(1 - e^(-dt/lag)).
  3. Speed: v += max_accel * share * throttle (0 below throttle_deadband, - in reverse) * dt;
     then (max_brake * share * brake + drag * |v|) * dt is taken off |v|, never past 0 (the
     brake and drag stop the car, they never reverse it). share = mass / (mass + payload):
     the same motor and brakes push a heavier car less (a = F / m; ego's dynamics.py
     load_share). max_accel and max_brake are the empty car's; payload 0 gives share 1.
  4. Steering: the motor's setpoint takes the target only when it differs by more than
     steer_deadband; the motor moves toward it at most steer_rate * dt; the linkage lags
     the motor (steer_lag); the wheels' angle is that plus steer_offset, within
     +-max_steering.
  5. Bicycle (base_link on the rear axle): yaw += v tan(steer) / wheelbase * dt, then
     x += v cos(yaw_mid) dt, y += v sin(yaw_mid) dt, with yaw_mid halfway between the
     step's old and new yaw; v and steer are the step's new ones.

Units: m/s, m/s^2, rad, s. Throttle and brake are 0..1. Positive steering turns left.
"""

import copy
from dataclasses import dataclass, fields, replace
import math
from typing import Optional, Tuple


# vehicle.yaml actuators: keys this model leaves out on purpose.
# tools/tests/test_vehicle_model_tool.py checks every key is a field or in here.
# open_loop_decel and pi_loop_decel are for the geofence only (aav_safety geofence.py coasts).
IGNORED = frozenset({'open_loop_decel', 'pi_loop_decel'})


@dataclass(frozen=True)
class ActuatorConfig:
    """vehicle.yaml's `actuators:` section, with the steering limit and wheelbase (`vehicle:`)."""

    max_accel: float = 1.5          # m/s^2 at full throttle, on the flat
    throttle_deadband: float = 0.0  # throttle (0..1) below which the car doesn't push
    throttle_lag: float = 0.1       # s: the throttle's first-order response
    max_brake: float = 3.0          # m/s^2 at full brake
    brake_lag: float = 0.2          # s: the brake actuator's first-order response
    drag: float = 0.1               # 1/s: coasting, m/s lost per second per m/s
    steer_rate: float = 0.8         # rad/s: the fastest the motor turns the wheels
    steer_lag: float = 0.1          # s: the linkage's first-order lag after the motor
    steer_offset: float = 0.0       # rad: where the wheels point when told straight
    steer_deadband: float = 0.005   # rad: a change smaller than this doesn't move the motor
    max_steering: float = 0.5       # rad: full lock (vehicle.yaml max_steering_angle)
    throttle_dead_time: float = 0.0  # s before the throttle starts to answer
    brake_dead_time: float = 0.0    # s before the brake starts to answer
    steer_dead_time: float = 0.0    # s before the steering motor starts to answer
    wheelbase: float = 1.561        # m (vehicle.yaml vehicle: wheelbase)
    mass: float = 181.0             # kg, the empty car (vehicle.yaml vehicle: mass)
    payload: float = 0.0            # kg aboard: people, cargo. Not in vehicle.yaml: a run's own

    @property
    def share(self):
        """Return mass / (mass + payload): how much of the empty car's push and braking is left."""
        return self.mass / (self.mass + self.payload)

    @classmethod
    def from_vehicle(cls, cfg):
        """Build it from the whole of vehicle.yaml (as aav_description.vehicle.load gives)."""
        known = {f.name for f in fields(cls)}
        values = {k: float(v) for k, v in (cfg.get('actuators') or {}).items() if k in known}
        values['max_steering'] = float(cfg['vehicle']['max_steering_angle'])
        values['wheelbase'] = float(cfg['vehicle']['wheelbase'])
        if 'mass' in cfg['vehicle']:
            values['mass'] = float(cfg['vehicle']['mass'])
        return cls(**values)

    @classmethod
    def from_vehicle_yaml(cls, path):
        """Build it from a vehicle.yaml file (Rust: ActuatorConfig::from_vehicle_yaml)."""
        import yaml
        with open(path) as f:
            return cls.from_vehicle(yaml.safe_load(f))


@dataclass(frozen=True)
class LqiConfig:
    """The spine's LQI (speed_loop.rs LqiConfig, TODO 3): one signed acceleration."""

    k_speed: float          # m/s^2 per m/s of speed error
    k_accel: float          # per m/s^2 of the pedals' lagged acceleration beyond the drag's
    k_integral: float       # m/s^2 per (m/s x s)
    setpoint_weight: float  # b: the share of the set speed the speed term sees
    integral_limit: float   # m/s^2 the integral may add or take, beyond (1 - b) k_speed r
    dead_band: float        # m/s^2 either side of zero: coast
    max_accel: float        # m/s^2 at full throttle (the model the LQI inverts)
    max_brake: float        # m/s^2 at full brake
    drag: float             # 1/s
    throttle_lag: float     # s
    brake_lag: float        # s

    def pedals(self, accel):
        """Return (throttle, brake) for a signed acceleration: never both."""
        if accel > self.dead_band:
            return clamp(accel / self.max_accel, 0.0, 1.0), 0.0
        if accel < -self.dead_band:
            return 0.0, clamp(-accel / self.max_brake, 0.0, 1.0)
        return 0.0, 0.0

    def lagged(self, pedals, throttle, brake, dt):
        """Return the pedals' acceleration a tick later: dt / (lag + dt) of the way."""
        sent = throttle * self.max_accel - brake * self.max_brake
        lag = self.throttle_lag if sent > 0.0 else self.brake_lag
        return pedals + (sent - pedals) * dt / (lag + dt)


# spine-core's LqiConfig::MODEL: the gains `python3 tools/vehicle_model.py --lqi` gives for
# vehicle.yaml's guesses (test_actuator_model.py checks both copies and the tool agree).
LQI_MODEL = LqiConfig(k_speed=1.832, k_accel=0.3169, k_integral=0.7015, setpoint_weight=0.5,
                      integral_limit=1.5, dead_band=0.05, max_accel=1.5, max_brake=3.0,
                      drag=0.1, throttle_lag=0.1, brake_lag=0.2)


@dataclass(frozen=True)
class SpeedLoopConfig:
    """The spine's speed loop gains (firmware/spine/crates/spine-core/src/speed_loop.rs)."""

    kp: float = 0.3             # throttle per m/s of error
    ki: float = 0.1              # throttle per (m/s x s)
    integral_limit: float = 0.3  # the most throttle the integral may add (beyond its share)
    setpoint_weight: float = 1.0  # b: the share of the target the P term sees (1: plain PI)
    anti_windup: str = 'bounds'  # 'bounds' (0..limit only) or 'clamp' (and not while pinned)
    brake_gain: float = 0.5      # brake per m/s of overspeed past coast_band
    coast_band: float = 0.2      # m/s too fast that coasting handles
    stopped_speed: float = 0.05  # m/s: slower counts as stopped
    hold_brake: float = 0.3      # brake held while stopped with a zero target
    lqi: object = None           # an LqiConfig: the LQI (SpeedLaw::Lqi); None: the PI


# The named parameter sets: spine-core's SpeedLoopProfile (the same names and numbers;
# firmware/spine/speed_loop_vectors.txt checks both). 'smooth' is TODO 27's design.
SPEED_LOOP_PROFILES = {
    'default': SpeedLoopConfig(),
    'smooth': SpeedLoopConfig(kp=1.0, ki=0.25, integral_limit=1.0, setpoint_weight=0.75,
                              anti_windup='clamp'),
    'lqi': SpeedLoopConfig(lqi=LQI_MODEL),
    # TODO 163: 'smooth' with a firmer brake past a narrower coast band. On this model a set
    # speed ramped to 0 at 1 m/s^2 is overrun by 0.68 m with smooth's brake, 0.21 m here.
    'firm': SpeedLoopConfig(kp=1.0, ki=0.25, integral_limit=1.0, setpoint_weight=0.75,
                            anti_windup='clamp', brake_gain=2.0, coast_band=0.05),
}


def speed_loop_profile(name):
    """Return a profile's SpeedLoopConfig by name: 'default', 'smooth', 'lqi' or 'firm'."""
    try:
        return SPEED_LOOP_PROFILES[name]
    except KeyError:
        raise ValueError(f'no speed loop profile {name!r}: pick one of '
                         f'{", ".join(SPEED_LOOP_PROFILES)}') from None


@dataclass(frozen=True)
class Demand:
    """What a speed loop asks for (ours, or the spine's). Throttle and brake: never both."""

    throttle: float
    brake: float
    reverse: bool


@dataclass(frozen=True)
class State:
    """The actuators after one step: what they were told, and what they did."""

    speed: float          # m/s, + forward
    accel: float          # m/s^2, this step
    steering: float       # rad: the wheels' real angle (offset included)
    steer_command: float  # rad: what was asked
    throttle: float       # 0..1, the real level
    throttle_command: float
    brake: float          # 0..1, the real level
    brake_command: float


def lag(value, target, tau, dt):
    """First-order lag: move value toward target with time constant tau (0: at once)."""
    if tau <= 0.0:
        return target
    return value + (target - value) * (1.0 - math.exp(-dt / tau))


def clamp(x, low, high):
    return min(high, max(low, x))


class SpeedLoop:
    """The spine's speed loop, ported: target and measured speed in, throttle and brake out."""

    def __init__(self, cfg=SpeedLoopConfig()):
        if cfg.anti_windup not in ('bounds', 'clamp'):
            raise ValueError(f"anti_windup must be 'bounds' or 'clamp', not {cfg.anti_windup!r}")
        self.cfg = cfg
        self.integral = 0.0
        self.reverse = False
        self.lqi_integral = 0.0   # the LQI's integral of the speed error (m/s x s)
        self.pedals = 0.0         # the LQI's model of the pedals' lagged acceleration

    def reset(self):
        self.integral = 0.0
        self.lqi_integral = 0.0
        self.pedals = 0.0

    def update(self, target, measured, dt):
        c = self.cfg
        stopped = abs(measured) < c.stopped_speed
        want_reverse = target < 0.0
        # The gear only changes while stopped; until then, brake.
        if target != 0.0 and want_reverse != self.reverse:
            self.reset()
            if stopped:
                self.reverse = want_reverse
            else:
                return Demand(0.0, clamp(max(c.hold_brake, c.brake_gain * abs(measured)),
                                         0.0, 1.0), self.reverse)
        sign = -1.0 if self.reverse else 1.0
        t, m = target * sign, measured * sign
        if c.lqi is not None:
            return Demand(*self._lqi(c.lqi, t, m, stopped, dt), self.reverse)
        # A stop: brake until stopped, then hold; never coast (the spine's own rule).
        if t <= 0.0:
            self.integral = 0.0
            brake = c.hold_brake if stopped else clamp(
                max(c.hold_brake, c.brake_gain * (m - c.coast_band)), 0.0, 1.0)
            return Demand(0.0, brake, self.reverse)
        error = t - m
        # The P term sees b x target; the integral holds the rest on top of its own limit.
        proportional = c.kp * (c.setpoint_weight * t - m)
        ceiling = c.integral_limit + (1.0 - c.setpoint_weight) * c.kp * t
        integral = clamp(self.integral + c.ki * error * dt, 0.0, max(0.0, ceiling))
        demand = proportional + integral
        pinned = c.anti_windup == 'clamp' and ((demand > 1.0 and error > 0.0)
                                               or (demand < 0.0 and error < 0.0))
        if not pinned:
            self.integral = integral
        overspeed = -error - c.coast_band
        if overspeed > 0.0:
            self.integral = 0.0
            return Demand(0.0, clamp(c.brake_gain * overspeed, 0.0, 1.0), self.reverse)
        return Demand(clamp(proportional + self.integral, 0.0, 1.0), 0.0, self.reverse)

    def _lqi(self, q, t, m, stopped, dt):
        """Return the LQI's (throttle, brake): spine-core SpeedLoop::lqi, line for line."""
        c = self.cfg
        stopping = t <= 0.0
        if stopping and stopped:
            throttle, brake = 0.0, c.hold_brake
        else:
            r = max(0.0, t)
            error = r - m
            b = q.setpoint_weight
            ceiling = q.integral_limit + (1.0 - b) * q.k_speed * r
            integral = 0.0 if stopping else self.lqi_integral + error * dt
            share = clamp(q.k_integral * integral, -q.integral_limit, ceiling)
            accel = (q.drag * r + q.k_speed * (b * r - m) - q.k_accel * (self.pedals - q.drag * r)
                     + share)
            if stopping:
                accel = min(accel, -c.hold_brake * q.max_brake)
            throttle, brake = q.pedals(accel)
            pinned = (throttle >= 1.0 and error > 0.0) or (brake >= 1.0 and error < 0.0)
            if not pinned and q.k_integral > 0.0:
                self.lqi_integral = share / q.k_integral
        if stopping:
            self.lqi_integral = 0.0
        self.pedals = q.lagged(self.pedals, throttle, brake, dt)
        return throttle, brake


@dataclass(frozen=True)
class Command:
    """
    One step's command: a steering target, and pedals or a set speed.

    With speed set (m/s, - backwards), a speed loop turns it into the pedals (step's and
    predict's `loop`); otherwise throttle and brake (0..1) and the gear go in as they are.
    """

    steer: float = 0.0               # rad, + left: the steering target
    throttle: float = 0.0            # 0..1
    brake: float = 0.0               # 0..1
    reverse: bool = False
    speed: Optional[float] = None    # m/s: a set speed for the speed loop, instead of pedals


@dataclass(frozen=True)
class CarState:
    """
    The whole car at one instant: where it is, how fast, and what its actuators are doing.

    x, y, yaw are in whatever frame the first state was (base_link: all 0, ego's trace).
    The delay lines hold the inputs still waiting out their dead time, oldest first; the
    throttle's is signed (- in reverse).
    """

    x: float = 0.0                 # m
    y: float = 0.0                 # m
    yaw: float = 0.0               # rad, + left
    v: float = 0.0                 # m/s, + forward
    steer: float = 0.0             # rad: the wheels' real angle (offset included)
    throttle_level: float = 0.0    # 0..1, after the lag
    brake_level: float = 0.0       # 0..1, after the lag
    steer_motor: float = 0.0       # rad: the steering motor (before the linkage and offset)
    steer_setpoint: float = 0.0    # rad: where the motor drives to (moves past the deadband)
    steer_linkage: float = 0.0     # rad: after the linkage's lag, before the offset
    throttle_delay: Tuple[float, ...] = ()
    brake_delay: Tuple[float, ...] = ()
    steer_delay: Tuple[float, ...] = ()


def delay_steps(dead_time, dt):
    """How many steps of dt a dead time waits: floor(dead_time / dt + 0.5), as in Rust."""
    return max(0, math.floor(dead_time / dt + 0.5))


def delay(line, value, steps, rest):
    """
    Push value into a delay line; return (what comes out now, the new line).

    The output is the value pushed `steps` steps ago; `rest` until the line is that long.
    A line longer than `steps` (dt grew) drops its oldest values.
    """
    if steps <= 0:
        return value, ()
    queue = line + (value,)
    if len(queue) > steps:
        return queue[-steps - 1], queue[-steps:]
    return rest, queue


def step(cfg, state, command, dt, loop=None):
    """
    Advance the car one step of dt (s); return the new CarState (the module's maths 1-5).

    A command with a set speed needs `loop`, a SpeedLoop, which this call advances.
    """
    if dt <= 0.0:
        raise ValueError(f'dt must be positive, not {dt}')
    c = cfg
    if command.speed is not None:
        if loop is None:
            raise ValueError('a set speed needs a speed loop (loop=SpeedLoop(...))')
        d = loop.update(command.speed, state.v, dt)
        throttle, brake, reverse = d.throttle, d.brake, d.reverse
    else:
        throttle, brake, reverse = command.throttle, command.brake, command.reverse

    # 1. Dead times.
    signed, throttle_line = delay(state.throttle_delay, -throttle if reverse else throttle,
                                  delay_steps(c.throttle_dead_time, dt), 0.0)
    brake, brake_line = delay(state.brake_delay, brake, delay_steps(c.brake_dead_time, dt), 0.0)
    target, steer_line = delay(state.steer_delay,
                               clamp(command.steer, -c.max_steering, c.max_steering),
                               delay_steps(c.steer_dead_time, dt), state.steer_setpoint)

    # 2, 3. Throttle and brake through their lags; the speed. The brake and drag oppose the
    # motion, and can only stop the car, never reverse it.
    throttle_level = lag(state.throttle_level, abs(signed), c.throttle_lag, dt)
    brake_level = lag(state.brake_level, brake, c.brake_lag, dt)
    pushing = throttle_level if throttle_level > c.throttle_deadband else 0.0
    share = c.share
    push = pushing * c.max_accel * share * (-1.0 if signed < 0.0 else 1.0)
    v = state.v + push * dt
    resist = (brake_level * c.max_brake * share + c.drag * abs(v)) * dt
    v = max(0.0, v - resist) if v > 0.0 else min(0.0, v + resist)

    # 4. Steering: deadband, then the motor's rate limit, then the linkage's lag.
    setpoint = target if abs(target - state.steer_setpoint) > c.steer_deadband \
        else state.steer_setpoint
    most = c.steer_rate * dt
    motor = state.steer_motor + clamp(setpoint - state.steer_motor, -most, most)
    linkage = lag(state.steer_linkage, motor, c.steer_lag, dt)
    steer = clamp(linkage + c.steer_offset, -c.max_steering, c.max_steering)

    # 5. The kinematic bicycle about the rear axle.
    yaw = state.yaw + v * math.tan(steer) / c.wheelbase * dt
    mid = 0.5 * (state.yaw + yaw)
    return CarState(x=state.x + v * math.cos(mid) * dt, y=state.y + v * math.sin(mid) * dt,
                    yaw=yaw, v=v, steer=steer, throttle_level=throttle_level,
                    brake_level=brake_level, steer_motor=motor, steer_setpoint=setpoint,
                    steer_linkage=linkage, throttle_delay=throttle_line,
                    brake_delay=brake_line, steer_delay=steer_line)


def predict(cfg, state, commands, dt, horizon, loop=None):
    """
    Return the trace: the states every dt for horizon seconds ahead, the first one dt ahead.

    commands is one Command, held, or a sequence of them, one a step, the last held.
    loop (for set speeds) is copied: the caller's own is never advanced.
    """
    if isinstance(commands, Command):
        commands = (commands,)
    if not commands:
        raise ValueError('predict needs at least one command')
    loop = copy.deepcopy(loop)
    states = []
    for i in range(max(0, math.floor(horizon / dt + 0.5))):
        state = step(cfg, state, commands[min(i, len(commands) - 1)], dt, loop)
        states.append(state)
    return states


class Actuators:
    """
    The modelled car, as an object: the speed loop, then step()'s actuators.

    brake: an adaptive_brake.AdaptiveBrake between the speed loop and the brake (None:
    the loop's brake as it is). It reads the model's own speed, as the car's would read
    the wheels. drive() (a firmware's outputs) never goes through it.
    """

    def __init__(self, cfg=ActuatorConfig(), loop=SpeedLoopConfig(), brake=None):
        self.cfg = cfg
        self.loop = SpeedLoop(loop)
        self.brake_loop = brake
        self.state = CarState()

    # The actuators' insides, by their old names (the tests read and set them).
    speed = property(lambda self: self.state.v,
                     lambda self, v: setattr(self, 'state', replace(self.state, v=v)))
    throttle = property(lambda self: self.state.throttle_level)
    brake = property(lambda self: self.state.brake_level)
    motor = property(lambda self: self.state.steer_motor)
    setpoint = property(lambda self: self.state.steer_setpoint)
    wheels = property(lambda self: self.state.steer_linkage)

    def step(self, speed_command, steer_command, dt, fault=None):
        """
        Step ego's speed and angle: the speed loop (ours, in Python), then the actuators.

        fault: an actuator fault on purpose (aav_gazebo faults.ActuatorFaults.apply), a
        function (Demand, steer) -> (Demand, steer) between the loop and the actuators. The
        State's commands stay what was asked, its levels and angle what the car did.
        """
        if dt <= 0.0:
            raise ValueError(f'dt must be positive, not {dt}')
        d = self.loop.update(speed_command, self.speed, dt)
        if self.brake_loop is not None:
            sent = self.brake_loop.update(d.throttle, d.brake, self.speed, dt)
            d = Demand(d.throttle, sent.brake, d.reverse)
        if fault is None:
            return self.drive(d, steer_command, dt)
        out = self.drive(*fault(d, steer_command), dt)
        return replace(out, steer_command=steer_command, throttle_command=d.throttle,
                       brake_command=d.brake)

    def drive(self, d, steer_command, dt):
        """Drive the actuators alone: a throttle, brake and gear Demand, and a steering target."""
        before = self.state.v
        self.state = step(self.cfg, self.state,
                          Command(steer=steer_command, throttle=d.throttle, brake=d.brake,
                                  reverse=d.reverse), dt)
        s = self.state
        return State(speed=s.v, accel=(s.v - before) / dt, steering=s.steer,
                     steer_command=steer_command, throttle=s.throttle_level,
                     throttle_command=d.throttle, brake=s.brake_level, brake_command=d.brake)


class Ideal:
    """The old behaviour: Gazebo's own limits on the speed, and an instant steer."""

    MAX_ACCEL = 1.5   # m/s^2: aav.gazebo.xacro's max_acceleration, until 15 Oct
    MIN_ACCEL = -3.0  # m/s^2: its min_acceleration

    def __init__(self, cfg=ActuatorConfig()):
        self.cfg = cfg
        self.speed = 0.0

    def step(self, speed_command, steer_command, dt):
        if dt <= 0.0:
            raise ValueError(f'dt must be positive, not {dt}')
        before = self.speed
        self.speed += clamp(speed_command - before, self.MIN_ACCEL * dt, self.MAX_ACCEL * dt)
        steering = clamp(steer_command, -self.cfg.max_steering, self.cfg.max_steering)
        nan = float('nan')
        return State(speed=self.speed, accel=(self.speed - before) / dt, steering=steering,
                     steer_command=steer_command, throttle=nan, throttle_command=nan,
                     brake=nan, brake_command=nan)
