"""
The actuator tests' safety rules: who may run what, how fast, and when to stop.

No ROS here. The node (actuator_test.py) asks these functions, and stops the car whenever
one says no. Every rule errs towards "this is the real car".

| Rule | Where |
|---|---|
| What we're driving: REAL, SIM, BENCH, REPLAY, or UNKNOWN (= REAL) | source_kind |
| Whether a test may start, its speed cap, and whether to ask first | decide |
| The person types the test's name to start it on the real car | confirmed |
| Speed, distance and time limits per test, checked every tick | Envelope.breach |
| Someone else is driving (the joystick outranks the tests) | taken_over |
| The hallway: its top speed, room, checklist (phase B) | hallway_limits, HALLWAY_CHECKLIST |
"""

from __future__ import annotations

from dataclasses import dataclass
import enum
import math
from typing import Iterable, Mapping, Optional, Sequence, Tuple

REAL_CAP = 1.0         # m/s: never faster on the real car (ego.yaml's max_speed, 15 Oct)
STANDS_CAP = 0.3       # m/s with the wheels off the ground (docs/tests.md stage 3)
SIM_DEFAULT_CAP = 3.0  # m/s in Gazebo, unless --max-speed says otherwise
SIM_MAX_CAP = 5.0      # m/s: ego's own ceiling for a simulator (sim_max_speed)

# aav_interfaces/SensorSource's values.
SOURCE_REAL, SOURCE_SIM, SOURCE_BENCH, SOURCE_REPLAY = 0, 1, 2, 3


class Kind(enum.Enum):
    """What /system/source says we're driving."""

    REAL = 'REAL'
    SIM = 'SIM'
    BENCH = 'BENCH'
    REPLAY = 'REPLAY'
    UNKNOWN = 'UNKNOWN'   # nobody announced: treated as the real car


def source_kind(announcements: Mapping[str, Sequence[int]]) -> Kind:
    """
    Merge /system/source's announcements (announcer -> its entries' source values).

    Anything REAL makes it REAL, even beside a simulator: one real actuator is enough.
    """
    values = [v for entries in announcements.values() for v in entries]
    if not values:
        return Kind.UNKNOWN
    if SOURCE_REAL in values:
        return Kind.REAL
    if SOURCE_REPLAY in values:
        return Kind.REPLAY
    if SOURCE_BENCH in values:
        return Kind.BENCH
    return Kind.SIM


@dataclass(frozen=True)
class Decision:
    """Whether a test may run, and how."""

    allowed: bool
    reason: str            # why not, or a note on what was decided
    cap: float = 0.0       # m/s: the hard speed cap for this run
    confirm: bool = False  # ask the person to type the test's name first
    real: bool = False     # treat it as the real car


def decide(kind: Kind, sim: bool, stands: bool, stage: int,
           requested_cap: Optional[float] = None, bench: Optional[bool] = None) -> Decision:
    """
    Decide whether this test (stage 3 or 4 of docs/tests.md) may run, and its speed cap.

    sim: the user said --sim (Ravensim, simulated time). stands: --stands (wheels off the
    ground). requested_cap: --max-speed, or None for the default. bench: --bench (bench
    mode, this computer only); None leaves BENCH alone, as before --bench existed.
    """
    if requested_cap is not None and not (math.isfinite(requested_cap) and requested_cap > 0):
        return Decision(False, f'--max-speed must be above 0 m/s (got {requested_cap})')
    if bench and kind is not Kind.BENCH:
        return Decision(False, f'--bench, but /system/source says {kind.value}. Start bench '
                               'mode first (pixi run bench), on this ROS domain')
    if bench is False and kind is Kind.BENCH:
        return Decision(False, 'bench mode announces BENCH on /system/source: add --bench '
                               '(this computer only)')
    if kind is Kind.REPLAY:
        return Decision(False, '/system/source says REPLAY: a recording plays; there is '
                               'nothing to drive')
    if sim and kind is not Kind.SIM:
        return Decision(False, f'--sim, but /system/source says {kind.value}. '
                               'Start Ravensim first (pixi run sim ...), on this ROS domain')
    if not sim and kind is Kind.SIM:
        return Decision(False, 'Ravensim announces SIM on /system/source: add --sim '
                               '(simulated time, this computer only)')

    if kind in (Kind.REAL, Kind.UNKNOWN):
        if stage == 3 and not stands:
            return Decision(False, 'a stage-3 test on the real car runs with the wheels off '
                                   'the ground: put it on stands and add --stands')
        if stage == 4 and stands:
            return Decision(False, 'a stage-4 test needs the wheels on the ground, in an '
                                   'empty lot: drop --stands')
        if requested_cap is not None and requested_cap > REAL_CAP:
            return Decision(False, f'the real car is capped at {REAL_CAP} m/s '
                                   f'(asked {requested_cap})')
        cap = REAL_CAP if requested_cap is None else requested_cap
        if stands:
            cap = min(cap, STANDS_CAP)
        note = ('nobody announced /system/source: treated as the real car'
                if kind is Kind.UNKNOWN else 'the real car')
        return Decision(True, note, cap, confirm=True, real=True)

    if kind is Kind.BENCH:
        if requested_cap is not None and requested_cap > REAL_CAP:
            return Decision(False, f"the bench runs the real car's limits: at most {REAL_CAP} "
                                   f'm/s (asked {requested_cap})')
        cap = REAL_CAP if requested_cap is None else requested_cap
        return Decision(True, 'the bench (stand-ins)',
                        min(cap, STANDS_CAP) if stands else cap)

    # Gazebo
    cap = SIM_DEFAULT_CAP if requested_cap is None else requested_cap
    if cap > SIM_MAX_CAP:
        return Decision(False, f'Ravensim tops out at {SIM_MAX_CAP} m/s (ego sim_max_speed); '
                               f'asked {cap}')
    return Decision(True, 'Ravensim', min(cap, STANDS_CAP) if stands else cap)


def confirmed(typed: Optional[str], test: str) -> bool:
    """Return True if the person typed the test's name exactly (spaces aside)."""
    return typed is not None and typed.strip() == test


@dataclass(frozen=True)
class Envelope:
    """The limits one test runs inside. Crossing any of them stops the car at once."""

    max_speed: float     # m/s, measured
    max_distance: float  # m since the test started
    max_time: float      # s since the test started

    def breach(self, speed: float, distance: float, elapsed: float) -> Optional[str]:
        """Why the test must stop now, or None."""
        if not math.isfinite(speed):
            return 'the measured speed is not a number'
        if abs(speed) > self.max_speed:
            return f'speed {abs(speed):.2f} m/s is over the limit {self.max_speed:.2f} m/s'
        if distance > self.max_distance:
            return f'drove {distance:.1f} m, over the limit {self.max_distance:.1f} m'
        if elapsed > self.max_time:
            return f'ran {elapsed:.0f} s, over the limit {self.max_time:.0f} s'
        return None


def speed_limit(cap: float) -> float:
    """Return the measured speed that aborts a test capped at `cap` (some overshoot)."""
    return cap * 1.2 + 0.1


def taken_over(ours: Iterable[Tuple[float, float]], theirs: Tuple[float, float],
               tolerance: float = 1e-3) -> bool:
    """
    Whether twist_mux's output (theirs: speed, turn rate) is none of our recent commands.

    True means a higher-priority driver (the joystick, the keyboard) is driving.
    """
    return not any(abs(v - theirs[0]) <= tolerance and abs(w - theirs[1]) <= tolerance
                   for v, w in ours)


STANDS_CHECKLIST = (
    'All four wheels are OFF the ground, on stands',
    'Someone holds the e-stop, and knows to press it',
    "The 2025-26 team's drive software is stopped (only our stack drives)",
    'Nobody near the wheels or the steering linkage',
    f'This test caps the speed at {STANDS_CAP} m/s (docs/tests.md, stage 3)',
)

LOT_CHECKLIST = (
    'Stage 3 passed on stands, and the ESP32 timeout fix (test 3.9) is in',
    'A safety driver in the seat, holding the joystick: its deadman outranks this test',
    'Someone else holds the e-stop',
    'An empty lot: nobody in the way, and the space this test needs (shown above) is clear',
    'Walking pace first: start with the default speed cap',
)


# aav_safety's geofence states (geofence.py) that stop a test on the ground. The tests'
# slot (60) is above the geofence's lock (51), so the test stops itself.
GEOFENCE_STOPS = ('NEAR', 'OUTSIDE', 'LOST', 'JUMP')
GEOFENCE_QUIET = 1.0    # s without /safety/geofence/state: it went quiet


def geofence_stop(state: Optional[str], age: Optional[float], stands: bool, real: bool,
                  starting: bool = False, hallway: bool = False) -> Optional[str]:
    """
    Why the geofence stops this test (or keeps it from starting), or None.

    state: the last /safety/geofence/state ("NEAR: why..."), None if never heard; age: s
    since it arrived. On stands nothing is fenced. On the real car the geofence must be
    heard, and not be in stands mode. starting: also refuse STARTING (the pose not settled).
    hallway: indoors, no GPS (car.launch.py gps:=false stands:=true), so the geofence says
    STANDS: the hallway's own fence (its room, by the wheels) stands in for it. It must
    still be heard, and NEAR, OUTSIDE, LOST or JUMP still stop.
    """
    if stands:
        return None
    if state is None or age is None or age > GEOFENCE_QUIET:
        return ('no /safety/geofence/state: the geofence must run for a test on the ground'
                if real else None)
    name = state.split(':', 1)[0].strip()
    if name in GEOFENCE_STOPS or (starting and name == 'STARTING'):
        return f'the geofence says {state}'
    if name == 'STANDS' and real and not hallway:
        return ('the geofence is in stands mode (no fence) but this test is on the ground: '
                'start the car without stands:=true')
    return None


def checklist(stands: bool) -> Tuple[str, ...]:
    """Return what to check before a test on the real car (docs/tests.md, Before)."""
    return STANDS_CHECKLIST if stands else LOT_CHECKLIST


# --- live checks while driving (every test) ----------------------------------------

# UNSEEN_TIME: s of a command above 0 with the measured speed under UNSEEN_SPEED. Not
# less: 8 Hall clicks a turn say nothing until 2 clicks (0.24 m), 2-3 s of a slow start
# (the bench, 29 Oct).
UNSEEN_TIME = 4.0
UNSEEN_SPEED = 0.05    # m/s: the spine's own standstill threshold
RUNAWAY_WINDOW = 1.0   # s: the speed rose this long while the command didn't
RUNAWAY_RISE = 0.1     # m/s


def unseen(history: Sequence[Tuple[float, float, float]]) -> Optional[str]:
    """
    Why to stop when the wheels are told to turn but the speed sensor sees nothing.

    history: (t s, commanded m/s, measured m/s), oldest first. On a rear-wheel-drive car
    whose wheel sensors are on the front, on stands, this is what happens: the motor turns
    the rear wheels at a speed nobody measures, and the speed cap can't fire.
    """
    if not history:
        return None
    now = history[-1][0]
    if now - history[0][0] < UNSEEN_TIME:
        return None
    window = [h for h in history if now - h[0] <= UNSEEN_TIME]
    if all(abs(c) > UNSEEN_SPEED and abs(m) < UNSEEN_SPEED for _, c, m in window):
        return (f'told to drive for {UNSEEN_TIME:g} s, but the measured speed stays under '
                f"{UNSEEN_SPEED} m/s: the speed can't be seen (front wheel sensors on "
                "stands? a dead sensor?) or the car isn't moving (the spine not armed?), so "
                "the speed cap can't work")
    return None


def runaway(history: Sequence[Tuple[float, float, float]]) -> Optional[str]:
    """
    Why to stop when the speed rises while the command falls, or stays at a stop.

    A stuck throttle. A held command above 0 is left alone (a speed loop's overshoot rises
    above a steady command); the envelope's speed limit covers that.
    """
    if not history:
        return None
    now, cmd, speed = history[-1]
    old = [h for h in history if now - h[0] >= RUNAWAY_WINDOW]
    if not old:
        return None
    _, cmd0, speed0 = old[-1]
    falling = abs(cmd) < abs(cmd0) - 0.01 or (abs(cmd) < UNSEEN_SPEED and abs(cmd0) < UNSEEN_SPEED)
    if (falling and abs(speed) > abs(speed0) + RUNAWAY_RISE
            and abs(speed) > abs(cmd) + RUNAWAY_RISE):
        return (f'the speed rose {abs(speed0):.2f} -> {abs(speed):.2f} m/s while the command '
                f'fell ({abs(cmd0):.2f} -> {abs(cmd):.2f}): a stuck throttle?')
    return None


# --- the preflight (procedures.preflight; preflight.py judges it) -------------------

PREFLIGHT_CHECKLIST = (
    'All four wheels are OFF the ground, on stands; the car chocked',
    'The e-stop person holds the Kar-Tech e-stop: THEY are the stop (Ctrl-C and the '
    'joystick only stop the software)',
    'Someone watches the REAR (driven) wheels with a tachometer app, and calls out a '
    'spin faster than a walk',
    'The Rust ESP32 firmware, with session 6c and test 3.9 passed (it brakes 0.2 s '
    'after the link goes)',
    'This runs on the Orin (ssh + tmux) or a wired laptop, not over WiFi',
    "The 2025-26 team's drive software is stopped (only our stack drives)",
    'Nobody near the wheels or the steering linkage',
    f'The speed is capped at {STANDS_CAP} m/s; the run takes about 40 s',
)

DRIVEN_WHEELS = 'rear'   # the car is rear-wheel drive (docs/learn-car.md, "The moving parts")


def preflight_refusal(kind: Kind, drive: bool, speed_wheels: Optional[str],
                      no_timeout_lock: Optional[bool], speed_loop_mode: Optional[str]
                      ) -> Optional[str]:
    """
    Why the preflight may not drive the real car, or None.

    drive: False for --no-drive (the sensors only: nothing moves, so nothing refused).
    speed_wheels: vehicle.yaml's vehicle.speed_wheels, which wheels /vehicle/odom
    measures. no_timeout_lock: the legacy bridge's /safety/legacy_uart_bridge/lock (None:
    not heard). speed_loop_mode: the legacy bridge's speed_loop.mode (None: not heard).
    """
    if not drive or kind not in (Kind.REAL, Kind.UNKNOWN):
        return None
    if speed_wheels != DRIVEN_WHEELS:
        return (f'/vehicle/odom measures the {speed_wheels or "unknown"} wheels '
                '(vehicle.yaml vehicle.speed_wheels), but the car drives the '
                f'{DRIVEN_WHEELS} ones: on stands the speed reads 0 while the motor spins, '
                'so no speed cap can work. Run --no-drive (the sensors only) until a '
                'driven-wheel speed exists (TODO 49b, the hardware lead)')
    if no_timeout_lock:
        return ("the legacy bridge's no-timeout lock is ON: the C++ ESP32 has no command "
                'timeout, so a lost link leaves the throttle on. Flash the Rust firmware '
                '(car-day session 6) and pass test 3.9 first')
    if speed_loop_mode == 'pi':
        return ('the legacy bridge runs speed_loop.mode pi: on stands its loop chases a '
                "speed it can't reach and raises the throttle. Restart it with "
                'speed_loop.mode open')
    return None


# --- phase B, in a hallway (docs/testing-plan.md; TODO 248) ---------------------------

HALL_ACCEL = 0.5       # m/s^2 to speed up: slow, as the testing plan's table
HALL_DECEL = 0.5       # m/s^2 to stop: half the plan's 1.0, as the brake isn't measured yet
HALL_REACTION = 0.3    # s from the brake point to the brake biting
HALL_MARGIN = 3.0      # m: the default margin, left over at the far end
HALL_MIN_SPEED = 0.2   # m/s: a hallway too short for this is refused


@dataclass(frozen=True)
class Hallway:
    """What a hallway allows: its top speed and its room (the stop line from the start)."""

    length: float      # m, measured with a tape: start line to the far wall
    margin: float      # m left over at the far end, never driven into
    top: float         # m/s: the safe top speed (within REAL_CAP)
    room: float        # m: length - margin; the car must stand still before it

    def needed(self, speed: float) -> float:
        """Return the metres a run to `speed` and back to a stop takes, with the margin."""
        return hall_needed(speed, self.margin)


def hall_needed(speed: float, margin: float, accel: float = HALL_ACCEL,
                decel: float = HALL_DECEL, reaction: float = HALL_REACTION) -> float:
    """Return speed up + reaction + stop + margin (m): the plan's table, our numbers."""
    v = abs(speed)
    return v * v / (2 * accel) + v * reaction + v * v / (2 * decel) + margin


def hallway_limits(length: float, margin: float = HALL_MARGIN, cap: float = REAL_CAP,
                   accel: float = HALL_ACCEL, decel: float = HALL_DECEL,
                   reaction: float = HALL_REACTION) -> Tuple[Optional[Hallway], str]:
    """
    Return the hallway's limits, or None and why not.

    The top speed solves speed up + reaction + stop + margin = length, then is capped at
    `cap` (the shared 1.0 m/s on the real car).
    """
    if not (math.isfinite(length) and length > 0):
        return None, f'the hallway length must be above 0 m (got {length})'
    if not (math.isfinite(margin) and margin >= 1.0):
        return None, f'the margin must be at least 1 m (got {margin})'
    room = length - margin
    a = 1 / (2 * accel) + 1 / (2 * decel)
    v = (-reaction + math.sqrt(reaction * reaction + 4 * a * max(room, 0.0))) / (2 * a)
    top = round(min(v, cap), 2)
    if top < HALL_MIN_SPEED:
        return None, (f'a {length:g} m hallway with a {margin:g} m margin allows only '
                      f'{v:.2f} m/s: too short (needs {hall_needed(HALL_MIN_SPEED, margin):.1f} '
                      f'm for {HALL_MIN_SPEED} m/s)')
    return Hallway(length, margin, top, room), ''


HALLWAY_CHECKLIST = (
    'Phase A passed on stands (pixi run car-test --stands)',
    'The hallway is measured with a tape, start line to the far wall, and that is the '
    'length typed here',
    'The hallway is cleared, with a spotter at EACH end who stops people walking in',
    'A safety driver holds the joystick: its deadman outranks this test',
    'Someone else holds the e-stop (the Kar-Tech button) and walks beside the car',
    'The car started indoors: pixi run car <machine> gps:=false stands:=true (no GPS, so '
    "the geofence can't fence: this test's own fence, by the wheels, stops it)",
    'The car points straight down the hallway, its rear axle on the start line',
)
