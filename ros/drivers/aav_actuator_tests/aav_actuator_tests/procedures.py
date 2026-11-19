"""
The actuator tests themselves: what each one commands, step by step.

No ROS here. A test is a list of Steps; a Runner walks through them, one tick at a time,
and says what to command (speed and steering angle). The node turns that into a
TwistStamped on /drivers/tests/cmd_vel and logs what the car did.

  steer-sweep   3.10  left, centre, right in steps; command vs measured angle
  steer-rate    3.11  centre to full lock and back, each way: the steering's speed
  steer-centre  3.12  straight: the wheels' angle; then tiny steps: the deadband
  speed-up      4.1   from still to a speed: time to 1, 2, 3 m/s, acceleration in g
  coast         4.2   from a speed, neither throttle nor brake: the drag
  brake         4.3   from each speed, stop: distance and deceleration in g
  throttle-map  4.4   speed-up to several speeds: throttle level vs acceleration
  circles       4.5   a full circle per steering angle, both ways: the real wheel angle
  straight      4.7   10 m with the steering straight: the sideways drift
  preflight     3.P   on stands: speed up, hold, slow, brake, steer; every sensor checked
                      at once (preflight.py judges it)
  wheel-steps   3.14  on stands: throttle on free wheels, a few speed steps; the wheel
                      speed must follow them (car_test.py's step 4)
  hall-*        B     phase B in a hallway (TODO 248): speed-up, coast, brake, steer, each
                      after 10 s standing still (the IMU's offset), each fenced: it brakes
                      in time to stop inside the hallway's room (stop_room)

Steering can only change while the car moves (ego holds it below 0.05 m/s), so the
stage-3 tests creep along: on stands the wheels spin slowly, in Gazebo the car crawls.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import enum
import math
from typing import Callable, Dict, List, Optional, Tuple

from aav_actuator_tests.analysis import STOPPED


class Until(enum.Enum):
    """When a step ends."""

    TIME = 'time'          # after `value` s
    REACHED = 'reached'    # the speed is at least `value` m/s, for `hold` s
    STOPPED = 'stopped'    # the car stands still, for `hold` s
    DISTANCE = 'distance'  # `value` m driven in this step
    TURNED = 'turned'      # the car turned `value` rad in this step
    BELOW = 'below'        # the speed fell under `value` m/s


COAST = None  # a Step's speed: follow the measured speed down (no throttle, no brake)
COAST_SMOOTH = 0.3   # s: COAST follows the speed's mean over this


@dataclass(frozen=True)
class Step:
    phase: str                       # its name, in the log
    speed: Optional[float]           # m/s to command, or COAST
    steering: float = 0.0            # rad, + left
    until: Until = Until.TIME
    value: float = 0.0
    hold: float = 0.0                # s the condition must hold
    timeout: float = 30.0            # s: give up on the condition and go on
    measure: bool = False            # a point to read the wheel angle (by hand on stands)
    must: bool = False               # timing out stops the test, rather than going on
    fenced: bool = False             # the hallway: ends where the car must start to stop
    # m/s below the measured speed (COAST), instead of the runner's own
    coast_offset: Optional[float] = None


@dataclass(frozen=True)
class Obs:
    """What the car is doing, as the runner needs it."""

    t: float          # s since the test started
    speed: float      # m/s, measured
    distance: float   # m driven since the test started
    heading: float    # rad turned since the test started (+ left), unwrapped


@dataclass(frozen=True)
class Command:
    speed: float
    steering: float
    phase: str


@dataclass
class Done:
    """A finished step, for the analysis and the log."""

    step: Step
    start: float
    end: float
    timed_out: bool
    cut: bool = False   # a fenced step ended at the brake point, not at its own condition


class Runner:
    """Walk a test's steps, one tick at a time."""

    def __init__(self, steps: List[Step], coast_offset: float = 0.1,
                 room: Optional[float] = None, stop_decel: float = 0.5,
                 reaction: float = 0.3):
        if not steps:
            raise ValueError('a test needs at least one step')
        self.steps = steps
        self.coast_offset = coast_offset  # m/s below the measured speed, inside the
        self.index = 0                    # speed loop's coast band: no throttle, no brake
        self.room = room                  # m from the start to the hallway's stop line
        self.stop_decel = stop_decel      # m/s^2 the brake point assumes
        self.reaction = reaction          # s before the brake bites, at the brake point
        self.recent: List[Tuple[float, float]] = []   # (t, speed) over COAST_SMOOTH s
        self.start: Optional[Obs] = None
        self.held_since: Optional[float] = None
        self.done: List[Done] = []

    @property
    def step(self) -> Optional[Step]:
        return self.steps[self.index] if self.index < len(self.steps) else None

    @property
    def finished(self) -> bool:
        return self.index >= len(self.steps)

    def condition(self, s: Step, obs: Obs) -> bool:
        start = self.start
        elapsed = obs.t - start.t
        if s.until is Until.TIME:
            return elapsed >= s.value
        if s.until is Until.DISTANCE:
            return obs.distance - start.distance >= s.value
        if s.until is Until.TURNED:
            return abs(obs.heading - start.heading) >= s.value
        if s.until is Until.BELOW:
            return abs(obs.speed) < s.value
        now = (abs(obs.speed) >= s.value if s.until is Until.REACHED
               else abs(obs.speed) < STOPPED)
        if not now:
            self.held_since = None
            return False
        if self.held_since is None:
            self.held_since = obs.t
        return obs.t - self.held_since >= s.hold

    def tick(self, obs: Obs) -> Optional[Command]:
        """Return the command for this tick, or None once every step is done."""
        while not self.finished:
            s = self.steps[self.index]
            if self.start is None:
                self.start, self.held_since = obs, None
            cut = s.fenced and self.at_brake_point(obs)
            met = cut or self.condition(s, obs)
            timed_out = not met and obs.t - self.start.t >= s.timeout
            if not (met or timed_out):
                break
            self.done.append(Done(s, self.start.t, obs.t, timed_out, cut))
            self.index += 1
            self.start = None
        if self.finished:
            return None
        s = self.steps[self.index]
        offset = self.coast_offset if s.coast_offset is None else s.coast_offset
        speed = max(0.0, self.smoothed(obs) - offset) if s.speed is COAST else s.speed
        return Command(speed, s.steering, s.phase)

    def smoothed(self, obs: Obs) -> float:
        """
        Return the speed's mean over the last COAST_SMOOTH s.

        A noisy wheel speed would make a coast's target jump up and down, and the speed
        loop push and brake.
        """
        self.recent = [r for r in self.recent if obs.t - r[0] < COAST_SMOOTH]
        self.recent.append((obs.t, abs(obs.speed)))
        return sum(v for _, v in self.recent) / len(self.recent)

    def at_brake_point(self, obs: Obs) -> bool:
        """Whether the car must start to stop now to stand still inside the room."""
        if self.room is None:
            return False
        return obs.distance + stop_room(obs.speed, self.stop_decel, self.reaction) >= self.room


def stop_room(speed: float, decel: float, reaction: float) -> float:
    """Return the metres a car at `speed` needs to stop: a reaction, then `decel`."""
    v = abs(speed)
    return v * reaction + v * v / (2.0 * decel)


@dataclass(frozen=True)
class Settings:
    """What a run may do: from the command line, the safety decision and ego."""

    cap: float                       # m/s: never command faster
    max_steering: float = 0.5        # rad: full lock (vehicle.yaml)
    wheelbase: float = 1.57          # m
    creep: float = 0.2               # m/s for the steering tests
    speeds: Tuple[float, ...] = ()   # m/s for speed-up, brake, coast (empty: the defaults)
    angles: Tuple[float, ...] = ()   # rad for circles (empty: the defaults)
    sweep_steps: int = 4             # steps each side of centre in steer-sweep
    dwell: float = 2.5               # s at each steering angle
    circle_speed: float = 1.0        # m/s (test 4.5)
    straight_speed: float = 0.5      # m/s (test 4.7)
    straight_distance: float = 10.0  # m (test 4.7)
    room: Optional[float] = None     # m: the hallway's length less its margin (hall-*)
    stop_decel: float = 0.5          # m/s^2 the hallway's brake point assumes
    reaction: float = 0.3            # s, the same
    hall_steer: float = 0.15         # rad: hall-steer's steps, each way
    hall_creep: float = 0.5          # m/s: hall-steer's speed (at most the cap)

    def speed_list(self, default: Tuple[float, ...]) -> Tuple[float, ...]:
        """Return the speeds to test: those asked for, else the defaults, within the cap."""
        wanted = self.speeds or default
        out = tuple(sorted({round(min(v, self.cap), 3) for v in wanted if v > 0}))
        return out or (self.cap,)


def stop(phase: str = 'stop') -> Step:
    return Step(phase, 0.0, until=Until.STOPPED, hold=0.5, timeout=10.0)


def still() -> Step:
    """Wait for a standstill: the car may still roll from the step before."""
    return Step('wait until still', 0.0, until=Until.STOPPED, hold=1.0, timeout=10.0)


def speed_up_to(v: float, hold: float, phase: str) -> Step:
    # 95% of the target: a speed loop settles just under it.
    return Step(phase, v, until=Until.REACHED, value=0.95 * v, hold=hold,
                timeout=10.0 + 4.0 * v)


def steer_sweep(s: Settings) -> List[Step]:
    n = max(1, s.sweep_steps)
    fractions = ([i / n for i in range(0, n + 1)] + [i / n for i in range(n - 1, -n - 1, -1)]
                 + [i / n for i in range(-n + 1, 1)])
    steps = [still(), Step('roll', s.creep, 0.0, until=Until.TIME, value=1.5)]
    for k, f in enumerate(fractions):
        a = round(f * s.max_steering, 4)
        steps.append(Step(f'sweep {k:02d} {a:+.3f}', s.creep, a, until=Until.TIME,
                          value=s.dwell, measure=True))
    return steps + [stop()]


def steer_rate(s: Settings) -> List[Step]:
    m = s.max_steering
    steps = [still(), Step('centre', s.creep, 0.0, until=Until.TIME, value=3.0)]
    for name, a in (('left lock', m), ('centre from left', 0.0), ('right lock', -m),
                    ('centre from right', 0.0)):
        steps.append(Step(f'step to {name}', s.creep, a, until=Until.TIME, value=3.0))
    return steps + [stop()]


def steer_centre(s: Settings) -> List[Step]:
    steps = [still(), Step('roll', s.creep, 0.0, until=Until.TIME, value=1.5),
             Step('centre', s.creep, 0.0, until=Until.TIME, value=4.0, measure=True)]
    for sign, side in ((1.0, 'left'), (-1.0, 'right')):
        for k in range(1, 11):
            a = round(sign * 0.002 * k, 4)
            steps.append(Step(f'deadband {side} {a:+.3f}', s.creep, a, until=Until.TIME,
                              value=1.5))
        steps.append(Step(f'centre after {side}', s.creep, 0.0, until=Until.TIME, value=3.0))
    return steps + [stop()]


def speed_up(s: Settings) -> List[Step]:
    v = max(s.speed_list((s.cap,)))
    return [still(), speed_up_to(v, 1.0, f'speed up to {v:g}'), stop()]


def coast(s: Settings) -> List[Step]:
    v = max(s.speed_list((2.0,)))
    return [still(), speed_up_to(v, 2.0, f'speed up to {v:g}'),
            Step(f'coast from {v:g}', COAST, until=Until.BELOW, value=max(0.3, 0.25 * v),
                 timeout=20.0),
            stop()]


def brake(s: Settings) -> List[Step]:
    steps = []
    for v in s.speed_list((1.0, 2.0, 3.0)):
        steps += [still(), speed_up_to(v, 1.5, f'speed up to {v:g}'),
                  Step(f'brake from {v:g}', 0.0, until=Until.STOPPED, hold=0.5,
                       timeout=15.0)]
    return steps


def throttle_map(s: Settings) -> List[Step]:
    steps = []
    for v in s.speed_list(tuple(s.cap * f for f in (0.25, 0.5, 0.75, 1.0))):
        steps += [still(), speed_up_to(v, 1.0, f'speed up to {v:g}'), stop(f'stop from {v:g}')]
    return steps


def circle_angles(s: Settings) -> Tuple[float, ...]:
    wanted = s.angles or tuple(f * s.max_steering for f in (0.4, 0.7, 1.0))
    out = []
    for a in wanted:
        a = min(abs(a), s.max_steering)
        out += [round(a, 4), round(-a, 4)]
    return tuple(out)


def circles(s: Settings) -> List[Step]:
    v = min(s.circle_speed, s.cap)
    steps = [still(), speed_up_to(v, 0.5, f'speed up to {v:g}')]
    for a in circle_angles(s):
        steps += [Step(f'settle {a:+.3f}', v, a, until=Until.TIME, value=3.0),
                  Step(f'circle {a:+.3f}', v, a, until=Until.TURNED, value=2 * math.pi,
                       timeout=circle_length(a, s.wheelbase) / v * 1.5 + 10.0)]
    return steps + [stop()]


def straight(s: Settings) -> List[Step]:
    v = min(s.straight_speed, s.cap)
    return [still(), speed_up_to(v, 1.0, f'speed up to {v:g}'),
            Step('straight', v, 0.0, until=Until.DISTANCE, value=s.straight_distance,
                 timeout=s.straight_distance / v * 2.0 + 5.0),
            stop()]


# The preflight's phases (preflight.py reads them back from the log).
PF_STILL, PF_SETTLE = 'wait until still', 'settle'
PF_SPEED_UP, PF_HOLD, PF_SLOW, PF_BRAKE = 'speed up', 'hold', 'slow down', 'brake'
PF_LEFT, PF_RIGHT, PF_CENTRE = 'steer left', 'steer right', 'centre'
PF_SETTLE_TIME = 3.0  # s still before the drive: the IMU's bias, the GPS's start
PF_HOLD_TIME = 5.0    # s at the cap
PF_SLOW_TIME = 3.0    # s at half the cap
PF_STEER_TIME = 3.0   # s at each steering angle


def preflight(s: Settings) -> List[Step]:
    """
    Return the preflight's steps: speed up, hold, slow, brake; steer left, right, centre.

    About 40 s at the stands' 0.3 m/s. Speeding up and braking must finish: a time-out
    stops the test (the wheels may be spinning at a speed nobody measures).
    """
    v, creep, lock = s.cap, min(s.creep, s.cap), s.max_steering
    return [
        still(),
        Step(PF_SETTLE, 0.0, until=Until.TIME, value=PF_SETTLE_TIME),
        Step(PF_SPEED_UP, v, until=Until.REACHED, value=0.95 * v, hold=1.0, timeout=10.0,
             must=True),
        Step(PF_HOLD, v, until=Until.TIME, value=PF_HOLD_TIME),
        Step(PF_SLOW, 0.5 * v, until=Until.TIME, value=PF_SLOW_TIME),
        Step(PF_BRAKE, 0.0, until=Until.STOPPED, hold=1.0, timeout=10.0, must=True),
        Step(PF_LEFT, creep, lock, until=Until.TIME, value=PF_STEER_TIME),
        Step(PF_RIGHT, creep, -lock, until=Until.TIME, value=PF_STEER_TIME),
        Step(PF_CENTRE, creep, 0.0, until=Until.TIME, value=PF_STEER_TIME),
        Step('stop', 0.0, until=Until.STOPPED, hold=1.0, timeout=10.0, must=True),
    ]


WHEEL_STEP_FRACTIONS = (1 / 3, 2 / 3, 1.0, 0.5)   # of the cap: up, up, up, down
WHEEL_STEP_TIME = 4.0                             # s at each speed
WHEEL_STEP_PHASE = 'wheel step'                   # each step's phase: 'wheel step 0.10'


def wheel_steps(s: Settings) -> List[Step]:
    """
    Return throttle on free wheels: a few speed steps, each held WHEEL_STEP_TIME s.

    On stands the speeds are the stands' cap and below (0.1, 0.2, 0.3, 0.15 m/s at 0.3);
    reports.wheel_steps judges whether the wheel speed followed them.
    """
    steps = [still()]
    for f in WHEEL_STEP_FRACTIONS:
        v = round(f * s.cap, 3)
        steps.append(Step(f'{WHEEL_STEP_PHASE} {v:.2f}', v, 0.0, until=Until.TIME,
                          value=WHEEL_STEP_TIME))
    return steps + [stop()]


# --- phase B, in a hallway (TODO 248; docs/actuator-tests.md, "The hallway") -----------

HALL_STILL = 'imu still'    # the stand-still window's phase: the IMU's offset
HALL_STILL_TIME = 10.0      # s
HALL_RELEASE = 'release'    # a tick past the coast band: the speed loop drops its integral
HALL_STEER_TIME = 1.5       # s of each steering step (twice that for the middle one)
HALL_LET_GO = 'let go'      # a creep too slow to move the car: the hold brake lets go
HALL_LET_GO_SPEED = 0.02    # m/s: under the spine's standstill (0.05), so nothing moves
HALL_LET_GO_TIME = 1.5      # s: the brake's lag several times over


def hall_start() -> List[Step]:
    """
    Stand still HALL_STILL_TIME s (the IMU's offset), then let the hold brake go.

    A speed loop holds the brake while told 0; a speed-up straight from there starts
    against the brake letting go, and the throttle's dead time would hold the brake's
    lag. Told 0.02 m/s instead, the loop lets go and its throttle stays too small to move
    the car (the legacy ESP32 brakes under 0.05 m/s: there the brake stays on).
    """
    return [still(), Step(HALL_STILL, 0.0, until=Until.TIME, value=HALL_STILL_TIME),
            Step(HALL_LET_GO, HALL_LET_GO_SPEED, until=Until.TIME, value=HALL_LET_GO_TIME,
                 fenced=True)]


# The top speed held, a plateau before the command falls. 2.5 s: over
# safety.RUNAWAY_WINDOW, so the speed a second before the fall is the plateau's, not the
# climb's (the lags carry the climb on a little after the command falls: no false runaway).
HALL_HOLD = 'hold at'
HALL_HOLD_TIME = 2.5        # s


def hall_up(v: float, hold: float) -> List[Step]:
    """Speed up to v, then hold it HALL_HOLD_TIME s; both fenced."""
    s = speed_up_to(v, hold, f'speed up to {v:g}')
    return [replace(s, fenced=True),
            Step(f'{HALL_HOLD} {v:g}', v, until=Until.TIME, value=HALL_HOLD_TIME, fenced=True)]


def hall_brake(v: float) -> Step:
    return Step(f'brake from {v:g}', 0.0, until=Until.STOPPED, hold=0.5, timeout=15.0)


def hall_speed_up(s: Settings) -> List[Step]:
    v = s.cap
    return hall_start() + [*hall_up(v, 1.0), hall_brake(v)]


def hall_coast(s: Settings) -> List[Step]:
    v = s.cap
    # 0.3 m/s below for 0.2 s: past the speed loop's coast band (0.2) even through ego's
    # deceleration ramp, so the loop drops its integral and the coast has no throttle in it.
    return hall_start() + [
        *hall_up(v, 0.5),
        Step(HALL_RELEASE, COAST, until=Until.TIME, value=0.2, coast_offset=0.3,
             fenced=True),
        Step(f'coast from {v:g}', COAST, until=Until.BELOW, value=max(0.15, 0.25 * v),
             timeout=30.0, fenced=True),
        stop()]


def hall_brake_test(s: Settings) -> List[Step]:
    v = s.cap
    return hall_start() + [*hall_up(v, 1.0), hall_brake(v)]


def hall_steer(s: Settings) -> List[Step]:
    v = min(s.hall_creep, s.cap)
    a = round(min(s.hall_steer, s.max_steering), 4)
    steps = hall_start() + hall_up(v, 0.5)
    # Left, right twice as long, left: the heading comes back near where it started.
    for k, (angle, time) in enumerate(((a, 1.0), (-a, 2.0), (a, 1.0), (0.0, 1.0))):
        steps.append(Step(f'steer {k} {angle:+.3f}', v, angle, until=Until.TIME,
                          value=time * HALL_STEER_TIME, fenced=True))
    return steps + [stop()]


def circle_length(angle: float, wheelbase: float) -> float:
    """Return the length of one circle of base_link at this wheel angle (m)."""
    return 2.0 * math.pi * wheelbase / math.tan(max(abs(angle), 0.02))


@dataclass(frozen=True)
class Test:
    name: str
    number: str        # in docs/tests.md
    stage: int         # 3: on stands; 4: in the lot
    title: str
    build: Callable[[Settings], List[Step]]
    ego_steer_rate: Optional[float] = None   # rad/s: lift ego's steering rate limit meanwhile
    hallway: bool = False    # phase B: only with --hallway, fenced by the hallway's room


TESTS: Dict[str, Test] = {t.name: t for t in (
    Test('steer-sweep', '3.10', 3, 'steering sweep: command vs measured angle', steer_sweep),
    Test('steer-rate', '3.11', 3, 'steering speed: centre to full lock', steer_rate,
         ego_steer_rate=3.0),
    Test('steer-centre', '3.12', 3, 'centre: the angle when told straight; the deadband',
         steer_centre),
    Test('speed-up', '4.1', 4, 'speed up: time to 1, 2, 3 m/s; acceleration in g', speed_up),
    Test('coast', '4.2', 4, 'coast: neither throttle nor brake; the drag', coast),
    Test('brake', '4.3', 4, 'brake: stopping distance and deceleration in g', brake),
    Test('throttle-map', '4.4', 4, 'throttle map: throttle level vs acceleration',
         throttle_map),
    Test('circles', '4.5', 4, 'circles: the real wheel angle per command', circles),
    Test('straight', '4.7', 4, 'straight line: the sideways drift over 10 m', straight),
    Test('preflight', '3.P', 3, 'preflight: speed, brake, steering and every sensor at '
         'once, PASS / FAIL', preflight),
    Test('wheel-steps', '3.14', 3, 'throttle on free wheels: the wheel speed follows a few '
         'speed steps', wheel_steps),
    Test('hall-speed-up', 'B1', 4, 'hallway: stand still, speed up, brake (throttle lag, '
         'dead time, max_accel)', hall_speed_up, hallway=True),
    Test('hall-coast', 'B2', 4, 'hallway: stand still, speed up, coast (the drag)',
         hall_coast, hallway=True),
    Test('hall-brake', 'B3', 4, 'hallway: stand still, speed up, brake (brake lag, dead '
         'time, max_brake, stopping distance)', hall_brake_test, hallway=True),
    Test('hall-steer', 'B4', 4, 'hallway: stand still, roll, steer left, right, left on the '
         'ground (steering lag and dead time)', hall_steer, hallway=True),
)}
HALL_TESTS = tuple(name for name, t in TESTS.items() if t.hallway)


def estimate(steps: List[Step], s: Settings) -> Tuple[float, float]:
    """Return a generous (distance m, time s) for the whole test: the envelope."""
    distance = time = 0.0
    for st in steps:
        v = s.cap if st.speed is COAST else (st.speed or 0.0)
        if st.until is Until.TIME:
            t = st.value
        elif st.until is Until.DISTANCE:
            t = st.value / max(v, 0.1)
        elif st.until is Until.TURNED:
            t = circle_length(st.steering, s.wheelbase) / max(v, 0.1)
        elif st.until is Until.REACHED:
            t = v / 0.3 + st.hold        # 0.3 m/s^2: a sluggish car still gets there
        elif st.fenced and st.until is Until.BELOW:
            t = st.timeout               # a coast with little drag: the room ends it
        else:
            t = st.hold + 3.0
        if st.until in (Until.REACHED, Until.BELOW):
            dist = v * t
        elif st.until is Until.STOPPED:
            dist = s.cap * s.cap / (2 * 0.5)  # stopping at only 0.5 m/s^2
        else:
            dist = v * t
        distance += dist
        time += min(t, st.timeout)
    return distance * 1.3 + 5.0, time * 1.5 + 20.0


def with_ego_limit(settings: Settings, ego_max_speed: Optional[float]) -> Tuple[Settings, str]:
    """Lower the cap to ego's own max_speed (ego would cap it anyway), with a note."""
    if ego_max_speed and 0 < ego_max_speed < settings.cap:
        return (replace(settings, cap=ego_max_speed),
                f"ego's max_speed is {ego_max_speed:g} m/s, below the cap: using it "
                '(Ravensim: pixi run sim sim_speed:=3 for faster tests)')
    return settings, ''


@dataclass
class Plan:
    """A test, ready to run."""

    test: Test
    steps: List[Step] = field(default_factory=list)
    distance: float = 0.0   # m: the envelope's limit
    time: float = 0.0       # s: the envelope's limit


def plan(name, s: Settings, max_distance: Optional[float] = None,
         max_time: Optional[float] = None) -> Plan:
    """Return the steps and the envelope for one test: a name in TESTS, or a Test."""
    if isinstance(name, Test):
        test = name
    elif name in TESTS:
        test = TESTS[name]
    else:
        raise ValueError(f'no test {name!r}: pick one of {", ".join(TESTS)}')
    if test.hallway and s.room is None:
        raise ValueError(f'{test.name} runs in a hallway only: its room (--hallway) is needed')
    steps = test.build(s)
    distance, time = estimate(steps, s)
    if test.hallway:
        # The hallway's room is the fence: past it the test stops (the margin is left).
        distance = min(max_distance or s.room, s.room)
        return Plan(test, steps, distance, max_time or time)
    return Plan(test, steps, max_distance or distance, max_time or time)
