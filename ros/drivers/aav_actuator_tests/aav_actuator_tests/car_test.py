"""
car-test: phase A (--stands) and phase B (--hallway) of docs/testing-plan.md. No ROS here.

    pixi run car-test --stands                       the real car, wheels off the ground
    pixi run car-test --stands --sim --no-manual     Ravensim as the stand-in (its own,
                                                     this computer only)
    pixi run car-test --hallway                      phase B: the transfer functions, in a
                                                     measured hallway (TODO 248)
    pixi run car-test --hallway --sim --no-manual    the same in Ravensim, a pretend hallway
    pixi run car-test --analyse <session>            a hallway session's report again

car_test_node.py runs the steps; this module holds what they are and how each is judged:

| Part | Where |
|---|---|
| The steps, in order; who does each on the car and in the sim | STEPS, Step, Manner |
| Which steps run (--only, --sim, --no-manual) | plan() |
| The command line (--stands required, --ground refused) | parse() |
| An actuator_test run: its arguments, its exit code as a result | tool_args(), tool_result() |
| The steering's vehicle.yaml lines, from its summary.txt | yaml_lines_of() |
| Does the watchdog name a sensor; a sensor fault's limit | named(), named_inputs(), limit_for() |
| A fault's result: who said why, how fast it stood still | judge_fault(), judge_heartbeat_car() |
| The report: the table, report.md | Result, table(), markdown() |
| The hallway: its passes, both ways, repeated (hallway.py fits) | hall_passes(), hall_args() |

Every rule errs towards stopping: any FAIL ends the run (the steps after it: NOT RUN).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import datetime
import enum
import os
import re
from typing import Iterable, List, Mapping, Optional, Sequence, Tuple

STILL_FOR = 0.3          # s under analysis.STOPPED: the car stands still
WATCHDOG_MARGIN = 1.0    # s after an input's max_age for the watchdog to say it
STILL_AFTER = 3.0        # s from the word (or the fault) to standing still
# s from the heartbeat stopping to standing still (sim: the backend's 0.2 s timeout, then
# the brake from the roll)
HEARTBEAT_LIMIT = 2.0
ESTOP_LIMIT = 2.0        # s from the e-stop to standing still
MANUAL_WAIT = 60.0       # s for a person's fault to show, once asked
# s the car must stay braked, still asked to roll, after the Orin's link is back (the
# ESP32's latch)
LATCH_HOLD = 2.0
ROLL = 0.2               # m/s: the slow roll each fault starts from (within the stands' cap)


class Status(enum.Enum):
    PASS = 'PASS'
    FAIL = 'FAIL'
    SKIPPED = 'SKIPPED'
    NOT_RUN = 'NOT RUN'


class How(enum.Enum):
    TOOL = 'tool'      # an actuator_test run, as a child process (its own report folder)
    FAULT = 'fault'    # a fault on purpose, from the runner itself


class Manner(enum.Enum):
    AUTO = 'auto'      # the runner does it
    MANUAL = 'manual'  # a person does it, when asked (skipped with --no-manual)
    NONE = 'none'      # not here (SKIPPED, with why)


@dataclass(frozen=True)
class Step:
    name: str          # for --only
    number: str        # in docs/testing-plan.md, A2
    title: str
    how: How
    passes: str        # passes when
    car: Manner
    sim: Manner
    args: Tuple[str, ...] = ()   # TOOL: actuator_test's arguments (--stands, --sim added)
    sensor: str = ''   # FAULT on a sensor: its sensor_watchdog input, and Ravensim's fault
    ask: str = ''      # MANUAL on the car: what the person does
    why_not: str = ''  # NONE: why not here


STEPS: Tuple[Step, ...] = (
    Step('preflight', '1', 'the preflight (actuator-test preflight --stands)', How.TOOL,
         'every line PASS', Manner.AUTO, Manner.AUTO, ('preflight',)),
    Step('sensors', '2', 'every sensor at its documented rate (preflight --no-drive)',
         How.TOOL, 'each at its rate (preflight.SENSORS)', Manner.AUTO, Manner.AUTO,
         ('preflight', '--no-drive')),
    Step('steer-sweep', '3a', 'steering sweep (3.10)', How.TOOL,
         'its vehicle.yaml lines printed', Manner.AUTO, Manner.AUTO, ('steer-sweep',)),
    Step('steer-rate', '3b', 'steering rate (3.11)', How.TOOL,
         'its vehicle.yaml lines printed', Manner.AUTO, Manner.AUTO, ('steer-rate',)),
    Step('steer-centre', '3c', 'steering centre (3.12)', How.TOOL,
         'its vehicle.yaml lines printed', Manner.AUTO, Manner.AUTO, ('steer-centre',)),
    Step('wheel-steps', '4', 'throttle on free wheels (3.14)', How.TOOL,
         'the wheel speed follows the steps', Manner.AUTO, Manner.AUTO, ('wheel-steps',)),
    Step('heartbeat', '5a', 'the Orin is lost: its heartbeat to the ESP32 stops', How.FAULT,
         'the ESP32 brakes and holds, on its own', Manner.MANUAL, Manner.AUTO,
         ask="pull the network cable between the Orin and the Motor Pi (the Orin's "
             'commands and heartbeat stop; the Pi keeps sending, the counter frozen)'),
    Step('gps', '5b', 'the GPS is lost', How.FAULT,
         'the sensor watchdog says why within its limit', Manner.MANUAL, Manner.AUTO,
         sensor='gps', ask="unplug the GPS's USB cable (or stop its driver)"),
    Step('imu', '5c', 'the IMU is lost', How.FAULT,
         'the sensor watchdog says why within its limit', Manner.MANUAL, Manner.AUTO,
         sensor='imu', ask="unplug the IMU's USB cable (or stop its driver)"),
    Step('camera', '5d', 'the front camera is lost', How.FAULT,
         'the sensor watchdog says why within its limit', Manner.MANUAL, Manner.AUTO,
         sensor='camera', ask="unplug the front left camera's cable (or stop its stream)"),
    Step('lidar', '5e', 'the lidar is lost', How.FAULT,
         'the sensor watchdog says why within its limit', Manner.MANUAL, Manner.AUTO,
         sensor='lidar', ask="unplug the lidar's network cable"),
    Step('pi-link', '5f', "a Pi is lost: the camera Pi's link", How.FAULT,
         'the car stops and something says why', Manner.MANUAL, Manner.NONE,
         ask="pull the camera Pi's network cable",
         why_not='no Pi in Ravensim: on the car only'),
    Step('cable', '5g', 'a cable pulled', How.FAULT,
         'the same as the software fault: it stops, and something says why',
         Manner.MANUAL, Manner.NONE,
         ask='pull one cable, as the hardware lead chooses (say which in the notes)',
         why_not='no cables in Ravensim: on the car only'),
    Step('estop', '5h', 'the e-stop', How.FAULT, 'everything stops', Manner.MANUAL,
         Manner.AUTO, ask='press the e-stop (the Kar-Tech button)'),
)
NAMES = tuple(s.name for s in STEPS)
GROUPS = {
    'steering': ('steer-sweep', 'steer-rate', 'steer-centre'),
    'faults': tuple(s.name for s in STEPS if s.how is How.FAULT),
}


def manner(step: Step, sim: bool) -> Manner:
    return step.sim if sim else step.car


def expand(only: Iterable[str]) -> List[str]:
    """Return the step names --only asks for: names and groups (steering, faults)."""
    out: List[str] = []
    for word in only:
        for name in GROUPS.get(word, (word,)):
            if name not in out:
                out.append(name)
    return out


def plan(only: Sequence[str], sim: bool, no_manual: bool
         ) -> List[Tuple[Step, Optional[str]]]:
    """
    Return every step in order, each with why it is skipped (None: it runs).

    The order is STEPS' whatever --only's order: the e-stop stays last, as it latches.
    """
    wanted = set(expand(only)) if only else set(NAMES)
    out: List[Tuple[Step, Optional[str]]] = []
    for step in STEPS:
        m = manner(step, sim)
        if step.name not in wanted:
            why = 'not asked for (--only)'
        elif m is Manner.NONE:
            why = step.why_not
        elif m is Manner.MANUAL and no_manual:
            why = '--no-manual: it needs a person'
        else:
            why = None
        out.append((step, why))
    return out


def parse(argv: Sequence[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog='car-test',
        description='Phase A of docs/testing-plan.md on stands (--stands): the preflight, '
                    'every sensor, the steering, throttle on free wheels, faults on purpose; '
                    'phase B in a hallway (--hallway): speed-up, coast, brake, steer, the '
                    'IMU with the wheels, the transfer functions with error bars. One report.')
    p.add_argument('--stands', action='store_true',
                   help='phase A: the wheels are off the ground')
    p.add_argument('--hallway', action='store_true',
                   help='phase B: on the ground, in a measured, cleared hallway')
    p.add_argument('--ground', action='store_true',
                   help='reserved for the open lot: refused for now')
    p.add_argument('--length', type=float,
                   help='--hallway: m, measured, start line to the far wall (asked if not '
                        f'given; --sim: {SIM_LENGTH:g})')
    p.add_argument('--margin', type=float, default=HALL_MARGIN,
                   help=f'--hallway: m left over at the far end (default {HALL_MARGIN:g})')
    p.add_argument('--repeats', type=int, default=3,
                   help='--hallway: each kind of pass this many times (default 3)')
    p.add_argument('--kinds', nargs='+', default=list(HALL_KINDS), metavar='KIND',
                   help=f'--hallway: which passes ({", ".join(HALL_KINDS)})')
    p.add_argument('--one-way', action='store_true',
                   help="--hallway: don't turn the car round between passes (the slope "
                        "then can't be told from the IMU's offset)")
    p.add_argument('--sim-actuators', default='',
                   help="--hallway --sim: the simulated car's own numbers, "
                        '"throttle_lag=0.3,drag=0.2" (the fits must find them; the drivers '
                        'still read vehicle.yaml)')
    p.add_argument('--wheel-noise', type=float, default=0.0,
                   help='--sim: m/s of noise on each simulated wheel speed')
    p.add_argument('--wheel-slip', type=float, default=0.0,
                   help='--sim: the simulated wheels read this share too far')
    p.add_argument('--analyse', metavar='SESSION',
                   help='a hallway session folder: its report again, from its files (no ROS)')
    p.add_argument('--sim', action='store_true',
                   help='start Ravensim as the stand-in (this computer only) and test it')
    p.add_argument('--only', nargs='+', default=[], metavar='STEP',
                   help=f'only these steps ({", ".join(NAMES)}; or the groups '
                        f'{", ".join(GROUPS)})')
    p.add_argument('--no-manual', action='store_true',
                   help='skip the steps that need a person (listed as SKIPPED)')
    p.add_argument('--out', help='the folder for reports (default ~/aav-tests, or '
                                 '$AAV_TESTS_DIR)')
    args = p.parse_args(argv)
    if args.analyse:
        return args
    if args.ground:
        p.error('--ground (the open lot) is refused for now: phase A on stands, then phase B '
                'in the hallway (--hallway), first (docs/testing-plan.md)')
    if args.stands == args.hallway:
        p.error('add --stands (phase A, wheels off the ground) or --hallway (phase B, in a '
                'measured hallway): one of them')
    if args.hallway:
        if args.only:
            p.error('--only is for --stands; --hallway takes --kinds')
        bad = [k for k in args.kinds if k not in HALL_KINDS]
        if bad:
            p.error(f'--kinds: no {", ".join(bad)} (pick from {", ".join(HALL_KINDS)})')
        if args.repeats < 1:
            p.error('--repeats must be at least 1')
        if args.length is not None and not args.length > 0:
            p.error('--length must be above 0 m')
        if not args.sim and (args.sim_actuators or args.wheel_noise or args.wheel_slip):
            p.error('--sim-actuators, --wheel-noise and --wheel-slip are for --sim only')
        if args.sim_actuators:
            try:
                sim_overrides(args.sim_actuators)
            except ValueError as e:
                p.error(f'--sim-actuators: {e}')
        if args.sim and args.length is None:
            args.length = SIM_LENGTH
        return args
    unknown = [w for w in args.only if w not in NAMES and w not in GROUPS]
    if unknown:
        p.error(f'--only: no step {", ".join(unknown)} (pick from {", ".join(NAMES)}, or '
                f'{", ".join(GROUPS)})')
    return args


def report_folder(base: Optional[str], sim: bool, now: datetime.datetime,
                  phase: str = 'stands') -> str:
    """Return the run's folder: <base>/<time>-<stands|hallway>[-sim] (not made here)."""
    base = os.path.expanduser(base or os.environ.get('AAV_TESTS_DIR', '~/aav-tests'))
    return os.path.join(base, now.strftime('%Y-%m-%d-%H%M%S') + f'-{phase}'
                        + ('-sim' if sim else ''))


# --- phase B, the hallway ------------------------------------------------------------

HALL_KINDS = ('coast', 'speed-up', 'brake', 'steer')   # coast first: its drag goes in the rest
HALL_MARGIN = 3.0       # m (safety.HALL_MARGIN)
SIM_LENGTH = 15.0       # m: Ravensim's pretend hallway
# The overrides the simulated car may take: the car model's actuator numbers.
SIM_KEYS = ('max_accel', 'throttle_lag', 'throttle_dead_time', 'max_brake', 'brake_lag',
            'brake_dead_time', 'drag', 'steer_rate', 'steer_lag', 'steer_dead_time',
            'steer_offset', 'steer_deadband', 'throttle_deadband')


def sim_overrides(text: str) -> Mapping[str, float]:
    """Parse --sim-actuators: 'name=value' pairs, by commas or spaces; names in SIM_KEYS."""
    out = {}
    for part in text.replace(',', ' ').split():
        name, eq, value = part.partition('=')
        if not eq:
            raise ValueError(f'not name=value: {part!r}')
        if name not in SIM_KEYS:
            raise ValueError(f'no actuator number {name!r} (pick from {", ".join(SIM_KEYS)})')
        try:
            out[name] = float(value)
        except ValueError:
            raise ValueError(f'{name}: not a number: {value!r}') from None
        if out[name] < 0:
            raise ValueError(f'{name} must not be negative')
    return out


@dataclass(frozen=True)
class HallPass:
    number: int        # 1, 2, ...
    kind: str          # coast, speed-up, brake, steer
    direction: str     # there, back

    @property
    def test(self) -> str:
        return f'hall-{self.kind}'

    @property
    def folder(self) -> str:
        return f'{self.number:02d}-{self.test}-{self.direction}'


def hall_passes(kinds: Sequence[str], repeats: int, both_ways: bool) -> List[HallPass]:
    """
    Return the passes in order: every kind (coast first), `repeats` times over.

    Both ways: the car faces 'there', then 'back' (turned round), then 'there'...: each
    pass starts where the last ended, and the IMU's offsets facing each way give the slope.
    """
    order = [k for k in HALL_KINDS if k in kinds]
    out = []
    for r in range(repeats):
        for k in order:
            n = len(out)
            out.append(HallPass(n + 1, k, 'back' if both_ways and n % 2 else 'there'))
    return out


def hall_args(p: HallPass, length: float, margin: float, sim: bool, out: str) -> List[str]:
    """actuator_test's arguments for one hallway pass."""
    args = [p.test, '--hallway', f'{length:g}', '--margin', f'{margin:g}', '--direction',
            p.direction, '--out', out]
    return args + (['--sim'] if sim else [])


def sim_launch_args(overrides: Mapping[str, float], wheel_noise: float,
                    wheel_slip: float) -> Tuple[str, ...]:
    """sim.launch.py's arguments for a pretend hallway: flat, empty, room behind and ahead."""
    actuators = ','.join(f'{k}={v:g}' for k, v in overrides.items())
    # Every pass starts where the last stopped, and in the sim nobody turns the car round:
    # start far back on the empty world's 400 m plane.
    # No numbers given: no actuators:= at all (launch refuses an empty one, 20 Nov: the
    # plain sim hallway never started).
    own = (f'actuators:={actuators}',) if actuators else ()
    return ('world:=empty', 'rviz:=false', 'nav:=false', 'cameras:=false',
            'lidar:=false', 'radar:=false', 'start_x:=-180') + own + (
            f'wheel_noise:={wheel_noise:g}', f'wheel_slip:={wheel_slip:g}')


# --- the tool steps ------------------------------------------------------------------

TOOL_CODES = {
    1: 'REFUSED: nothing moved (its output says why)',
    2: 'a wrong command line',
    3: 'ABORTED: a safety rule stopped it (its output says which)',
    4: 'an error in the test (its output has the trace)',
    5: 'a check FAILED (its table says which)',
    6: 'NOT STOPPED within 5 s: it set the e-stop',
}


# Sensors Ravensim doesn't have: its preflight steps skip them (--skip), and say so.
# /vehicle/battery is live on the bench only (docs/interfaces.yaml).
SIM_SKIP = ('battery',)


def tool_args(step: Step, sim: bool, out: str) -> List[str]:
    """Return actuator_test's arguments for a TOOL step: always --stands; --sim's own."""
    args = [*step.args, '--stands', '--out', out]
    if sim:
        args.append('--sim')
        if step.args[:1] == ('preflight',):
            args += ['--skip', *SIM_SKIP]
    return args


def tool_result(code: int, yaml: str = '', wheels: Optional[Tuple[bool, str]] = None,
                needs_yaml: bool = False) -> Tuple[Status, str]:
    """
    Turn an actuator_test run's exit code (and what it gave) into a result.

    yaml: its vehicle.yaml lines; needs_yaml: a steering test, which passes only with
    them. wheels: (follows, detail) for wheel-steps.
    """
    if code != 0:
        return Status.FAIL, TOOL_CODES.get(code, f'exit code {code}')
    if wheels is not None:
        follows, detail = wheels
        return (Status.PASS if follows else Status.FAIL), detail
    if needs_yaml:
        if not yaml.strip():
            return Status.FAIL, 'ran to the end, but no vehicle.yaml lines: the data ' \
                                "couldn't resolve them (its summary says why)"
        keys = [ln.split(':', 1)[0].strip() for ln in yaml.splitlines()
                if ln.startswith('  ') and ':' in ln]
        return Status.PASS, f'vehicle.yaml lines: {", ".join(keys)}'
    return Status.PASS, 'every line PASS (WARN and SKIP lines: its table)'


def yaml_lines_of(summary: str) -> str:
    """Return the vehicle.yaml lines a test printed into its summary.txt ('' if none)."""
    lines = summary.splitlines()
    for i, line in enumerate(lines):
        if line.startswith('# Paste into'):
            return '\n'.join(lines[i:]).strip()
    return ''


def only_folder(parent: str) -> Optional[str]:
    """Return the one run folder actuator_test made in `parent`, or None."""
    if not os.path.isdir(parent):
        return None
    found = sorted(d for d in os.listdir(parent) if os.path.isdir(os.path.join(parent, d)))
    return os.path.join(parent, found[-1]) if found else None


# --- the fault steps -----------------------------------------------------------------


def named_inputs(state: Optional[str]) -> List[str]:
    """
    Return the inputs a sensor_watchdog state line names as not OK, those that stop first.

    "STOP: gps STALE (nothing for 2.1 s (more than 2 s)); imu NAN (x)" -> ['gps', 'imu']
    (watchdog-core's summary(); "STOP - ..." as interfaces.yaml once wrote it, too).
    "OK" -> [].
    """
    head = re.match(r'(STOP|WARN)(: | - )', state or '')
    if not head:
        return []
    body = state[head.end():]
    return re.findall(r'(?:^|[,;]\s*)([a-z][a-z0-9_]*) [A-Z]{2,}', body)


def named(state: Optional[str], name: str) -> bool:
    """Whether the watchdog's line says STOP, naming this input."""
    return bool(state) and state.startswith('STOP') and name in named_inputs(state)


def limit_for(sensor: str, inputs: Mapping[str, Mapping]) -> Optional[float]:
    """Return how soon the watchdog must name this input (its max_age + margin), or None."""
    entry = inputs.get(sensor)
    if not entry or 'max_age' not in entry:
        return None
    return float(entry['max_age']) + WATCHDOG_MARGIN


@dataclass(frozen=True)
class Seen:
    """What happened after a fault went on, in s from `since` (None: never)."""

    since: str               # what the clock starts from, in words
    said: Optional[float]    # s until something said why
    by: str                  # who said it, in words ('' for nobody)
    still: Optional[float]   # s until the car stood still
    note: str = ''


def judge_fault(seen: Seen, said_limit: Optional[float], still_limit: float,
                need_word: bool = True) -> Tuple[Status, str]:
    """
    Judge one fault.

    need_word: something must say why within said_limit (None: any time), and the car be
    still within still_limit of that word; else still within still_limit of `since`.
    """
    said_ok = seen.said is not None and (said_limit is None or seen.said <= said_limit)
    if seen.said is None:
        said = f'nothing said why after {seen.since}'
    else:
        said = f'{seen.by} said why {seen.said:.2f} s after {seen.since}' + (
            f' (limit {said_limit:g} s)' if need_word and said_limit is not None else '')
    start = seen.said if need_word and seen.said is not None else 0.0
    if seen.still is None:
        still, still_ok = 'NEVER still', False
    else:
        took = seen.still - start
        still_ok = took <= still_limit
        still = (f'still {seen.still:.2f} s after {seen.since}, {took:.2f} s after the word '
                 f'(limit {still_limit:g} s)' if need_word else
                 f'still {seen.still:.2f} s after {seen.since} (limit {still_limit:g} s)')
    ok = still_ok and (said_ok or not need_word)
    parts = ([said] if need_word or seen.said is not None else []) + [still] + (
        [seen.note] if seen.note else [])
    return (Status.PASS if ok else Status.FAIL), '; '.join(parts)


def judge_heartbeat_car(stopped: Optional[float], top_after: Optional[float],
                        armed: Optional[bool], faults: Optional[int],
                        cmd_timeout_bit: int = 1) -> Tuple[Status, str]:
    """
    Judge the Orin's link pulled on the car: did the ESP32 brake on its own, and hold.

    stopped: s from the person's word to standing still (None: never seen). top_after: the
    top speed while the runner still asked to roll for LATCH_HOLD s after the link came
    back (None: no speed then). armed, faults: /vehicle/status after (None: not heard;
    the uplink is off).
    """
    parts = []
    if stopped is not None:
        parts.append(f'still {stopped:.2f} s after the link was pulled')
    else:
        parts.append("the speed wasn't seen still while the link was out")
    held = top_after is not None and top_after < 0.05
    if top_after is None:
        parts.append('no speed after the link came back')
    else:
        parts.append(f'link back, still asked to roll for {LATCH_HOLD:g} s: top '
                     f'{top_after:.2f} m/s ({"held: the ESP32 latched" if held else "NOT held"})')
    if armed is None:
        parts.append('no /vehicle/status (uplink off): the latch only from the speed')
    else:
        timeout = bool((faults or 0) & cmd_timeout_bit)
        parts.append(f'/vehicle/status: armed {armed}, CMD_TIMEOUT {timeout}')
        held = held and (not armed or timeout)
    return (Status.PASS if held else Status.FAIL), '; '.join(parts)


# --- the report ----------------------------------------------------------------------


@dataclass(frozen=True)
class Result:
    step: Step
    status: Status
    detail: str
    files: str = ''    # where its own files are, relative to the report folder


def failed(results: Sequence[Result]) -> bool:
    return any(r.status is Status.FAIL for r in results)


def counts(results: Sequence[Result]) -> Mapping[str, int]:
    return {s.value: sum(r.status is s for r in results) for s in Status}


def verdict(results: Sequence[Result]) -> str:
    c = counts(results)
    if c['FAIL']:
        return (f'PHASE A FAILED: {c["FAIL"]} step{"s" if c["FAIL"] != 1 else ""} failed. '
                'The car stays on stands')
    if c['SKIPPED'] or c['NOT RUN']:
        return (f'NO STEP FAILED, but {c["SKIPPED"] + c["NOT RUN"]} did not run: phase A is '
                'not passed until every step has')
    return 'PHASE A PASSED: every step PASS'


def table(results: Sequence[Result]) -> str:
    """Return the final table, for the terminal: one line per step."""
    width = max((len(r.step.name) for r in results), default=4)
    body = [f'  {r.status.value:<8} {r.step.number:<3} {r.step.name:<{width}}  {r.detail}'
            for r in results]
    return '\n'.join(['car-test --stands', '================='] + body + ['', verdict(results)])


def cell(text: str) -> str:
    return text.replace('|', '\\|').replace('\n', ' ')


def markdown(results: Sequence[Result], where: str, started: str, yaml: Mapping[str, str],
             notes: Sequence[str] = ()) -> str:
    """report.md: the summary for people (docs are for a dyslexic reader: short, a table)."""
    out = ['# car-test --stands', '',
           f'- **When:** {started}', f'- **Where:** {where}',
           f'- **Result:** {verdict(results)}', '',
           '## The steps', '',
           '| # | Step | Result | What happened | Passes when | Files |',
           '|---|---|---|---|---|---|']
    for r in results:
        files = f'`{r.files}`' if r.files else ''
        out.append(f'| {r.step.number} | {cell(r.step.title)} | **{r.status.value}** | '
                   f'{cell(r.detail)} | {cell(r.step.passes)} | {files} |')
    if yaml:
        out += ['', '## Lines for vehicle.yaml', '',
                'Printed by the steering tests. Check them; this tool never writes the file.', '']
        for name, lines in yaml.items():
            out += [f'### {name}', '', '```yaml', lines, '```', '']
    if notes:
        out += ['', '## Notes', ''] + [f'- {n}' for n in notes]
    return '\n'.join(out).rstrip() + '\n'
