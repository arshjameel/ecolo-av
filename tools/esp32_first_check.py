#!/usr/bin/env python3
"""
The first check after flashing the Rust firmware on the ESP32: does it work right.

`pixi run esp first-check [port]` (tools/esp32.sh first-check) reads the board's log on
the USB cable for 30 s, and prints a short table: PASS, FAIL, WARN or INFO for each check,
then one last line:

    SAFE TO GO ON to the next step
    NOT YET: the RC is lost; no roll back: the transmitter, then this check again
    STOP: <the check>, roll back (docs/esp32-bench.md)

It only reads: it sends nothing to the board. It reads both kinds of log the firmware
writes (src/main.rs): the text lines (the default build) and the JSON lines (`json-log`,
firmware/status/README.md).

| Check | PASS when |
|---|---|
| Log | lines came in at all |
| Build | the CONFIG line's build is this checkout's commit (or --build) |
| Features | `legacy-uart`; `uplink`, `json-log`, `wifi-status` allowed; never `qemu` |
| Last reset | a power-on, not a watchdog, a panic or a brown-out |
| Devices | all four I2C devices answered at boot (the DEVICES line) |
| Brake | at IDLE_BRAKE, as the CONFIG line says, in every status line |
| Throttle | 0 in every status line |
| Steering | centred in every status line |
| Mode | stopped, not armed (and latched, in the text FSM line). Braked with only the RC lost (safety Degraded, fault 256): WAIT, not a failure |
| Faults | none but 1 (command timeout) and 256 (RC lost: the transmitter is off) |
| Tick | the worst gap between ticks under 50 ms: half the watchdog's 100 ms |
| Restarts | none in the window (a second boot, a new boot_id, the uptime going back) |
| Pi link | says what the board sees: NO FRAMES is right while the Pi is off |

Exit codes: 0 safe to go on, 1 stop, 2 the port or file couldn't be read.

    python3 tools/esp32_first_check.py /dev/cu.usbserial-0001
    python3 tools/esp32_first_check.py --file saved.log --features qemu,uplink
    python3 tools/esp32_first_check.py --json /dev/ttyUSB0 --pi on
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import re
import select
import subprocess
import sys
import time
from typing import Dict, Iterable, List, Optional

ROOT = Path(__file__).resolve().parents[1]
SECONDS = 30.0
BAUD = 115200

# The firmware's numbers (spine-core's faults, src/main.rs's WATCHDOG_MS).
CMD_TIMEOUT = 0x0001
RC_SIGNAL_LOST = 0x0100
EXPECTED_FAULTS = CMD_TIMEOUT | RC_SIGNAL_LOST
FAULT_NAMES = {0x0001: 'command timeout', 0x0002: 'bad checksum', 0x0004: 'bad counter',
               0x0008: 'steering Jrk', 0x0010: 'brake Jrk', 0x0020: 'throttle DAC',
               0x0040: 'wheel encoder', 0x0080: 'CAN bus', 0x0100: 'RC lost',
               0x0200: 'overspeed'}
WORST_GAP_LIMIT_US = 50_000
REPORT_ONLY_FEATURES = {'json-log', 'wifi-status'}
# The car's build may carry these beside legacy-uart: the uplink is on by default since #23.
ALLOWED_FEATURES = REPORT_ONLY_FEATURES | {'uplink'}
DEVICE_NAMES = ('steering', 'brake', 'dac', 'expander')

# esp-hal's names for the ESP32's reset reasons (SocResetReason), and what they mean.
RESET_OK = {'ChipPowerOn', 'PowerOn'}
RESET_MEANS = {
    'CoreSw': 'a software reset: a panic? Its message is just above the boot line',
    'Cpu0Sw': 'a software reset: a panic? Its message is just above the boot line',
    'CoreMwdt0': 'a watchdog reset: a tick hung', 'CoreMwdt1': 'a watchdog reset: a tick hung',
    'CpuMwdt0': 'a watchdog reset: a tick hung', 'CoreRtcWdt': 'the RTC watchdog reset it',
    'Cpu0RtcWdt': 'the RTC watchdog reset it', 'SysRtcWdt': 'the RTC watchdog reset it',
    'SysBrownOut': 'a brown-out: the power dipped',
}

ANSI = re.compile(r'\x1b\[[0-9;]*m')
FIELD = re.compile(r'(\w+)=(\S+)')
TEXT_KINDS = ('STATUS', 'EVENT', 'FSM', 'LINK', 'TICK', 'HEALTH', 'CONFIG')
TEXT_MODES = {'Manual': 'stopped', 'Rc': 'rc', 'Computer': 'computer', 'Test': 'test'}
BOOT = re.compile(r'spine-esp32-umod \S+, link: .*the last reset: (?:Some\()?(\w+)')
DEVICES = re.compile(r'DEVICES (\d+)/(\d+) answered')


@dataclass
class Status:
    """One status line, in one form for both logs."""

    t: int
    mode: str               # stopped, rc, computer, test
    armed: bool
    faults: int
    throttle_zero: bool
    brake: Optional[float]  # 0..1 (text), or None
    brake_counts: Optional[int]
    steer_centred: Optional[bool]   # text: steer=0.000; JSON: needs the config's centre
    steer_counts: Optional[int]


@dataclass
class Log:
    """What the log said, in the order it said it."""

    lines: int = 0
    form: str = ''                          # text, json, or both
    config: Dict[str, object] = field(default_factory=dict)
    devices: Optional[dict] = None          # {'answered', 'expected', 'list': [...]}
    statuses: List[Status] = field(default_factory=list)
    latched: List[bool] = field(default_factory=list)    # text FSM, once Running: Latched?
    safety: List[str] = field(default_factory=list)      # text FSM, once Running: its safety
    lifecycle: Optional[str] = None                      # text FSM: the last one
    ticks: List[tuple] = field(default_factory=list)     # (t, worst_gap_us, overruns)
    link: List[str] = field(default_factory=list)        # the verdicts, in order
    boots: int = 0
    reset: Optional[str] = None
    restarts: List[str] = field(default_factory=list)
    last_up: Optional[int] = None
    boot_id: Optional[str] = None
    forms: set = field(default_factory=set)

    def uptime(self, t: int, what: str):
        """Note the board's clock: going back means it restarted."""
        if self.last_up is not None and t + 1000 < self.last_up:
            self.restarts.append(f'{what}: the uptime went back from {self.last_up} to {t} ms')
        self.last_up = t

    def boot(self, reset: str):
        if self.boots or self.statuses or self.config or self.forms:
            self.restarts.append(f'a second boot line (reset: {reset})')
        self.boots += 1
        self.reset = reset
        self.last_up = None


def _value(v: str):
    if v in ('true', 'false'):
        return v == 'true'
    for conv in (lambda s: int(s, 0), float):
        try:
            return conv(v)
        except ValueError:
            pass
    return v


def read_text(log: Log, text: str):
    """One text line (src/main.rs): STATUS, FSM, TICK, HEALTH, CONFIG, DEVICES, the boot."""
    boot = BOOT.search(text)
    if boot:
        log.boot(boot.group(1))
        return
    dev = DEVICES.search(text)
    if dev:
        log.forms.add('text')
        rest = text[dev.end():]
        found = []
        for name, value in FIELD.findall(rest):
            if name not in DEVICE_NAMES:
                continue
            parts = value.split(',')
            found.append({'name': name, 'address': int(parts[0], 16),
                          'answered': len(parts) > 1 and parts[1] == 'answered',
                          'feedback': parts[2] if len(parts) > 2 else None})
        log.devices = {'answered': int(dev.group(1)), 'expected': int(dev.group(2)),
                       'list': found}
        return
    for kind in TEXT_KINDS:
        at = text.find(kind + ' ')
        if at < 0 or ' t=' not in text[at:at + len(kind) + 4]:
            continue
        fields = {k: _value(v) for k, v in FIELD.findall(text[at + len(kind):])}
        if 't' not in fields:
            return
        log.forms.add('text')
        t = fields['t']
        if kind in ('STATUS', 'EVENT'):
            if 'faults' not in fields:
                return          # cut short
            log.uptime(t, kind)
            if kind == 'STATUS':
                log.statuses.append(Status(
                    t, TEXT_MODES.get(str(fields.get('mode')), str(fields.get('mode'))),
                    bool(fields.get('armed')), int(fields['faults']),
                    float(fields.get('throttle', 1)) == 0.0, float(fields.get('brake', -1)),
                    None, abs(float(fields.get('steer', 1))) < 0.0005, None))
        elif kind == 'FSM' and 'safety' in fields:
            # The first tick is the self-test's (safety Estop until it passes).
            log.lifecycle = str(fields.get('lifecycle'))
            if log.lifecycle == 'Running':
                log.latched.append(fields['safety'] == 'Latched')
                log.safety.append(str(fields['safety']))
        elif kind == 'TICK' and 'overruns' in fields:
            log.ticks.append((t, fields['worst_gap_us'], fields['overruns']))
        elif kind == 'HEALTH' and 'link' in fields:
            log.link.append(str(fields['link']))
        elif kind == 'CONFIG' and 'last_reset' in fields:
            log.config.update(dict(FIELD.findall(text[at + len(kind):])))
            log.config['idle_brake'] = float(log.config['idle_brake'])
            for k in ('steer_centre', 'brake_released', 'brake_full'):
                log.config[k] = int(log.config[k])
        return


def read_json(log: Log, line: dict):
    """One JSON line (firmware/status/README.md)."""
    t, kind = line.get('up_ms'), line.get('t')
    if not isinstance(t, int) or line.get('schema') != 1:
        return
    log.forms.add('json')
    log.uptime(t, kind)
    if kind == 'hb':
        if log.boot_id is not None and line.get('boot_id') != log.boot_id:
            log.restarts.append(f"a new boot_id ({log.boot_id} to {line.get('boot_id')})")
        log.boot_id = line.get('boot_id')
    elif kind == 'config':
        c = line
        log.config.update({
            'build': c.get('build'), 'features': c.get('features'),
            'last_reset': c.get('last_reset'), 'idle_brake': float(c.get('idle_brake', -1)),
            'steer_centre': c.get('steering', {}).get('centre'),
            'brake_released': c.get('brake', {}).get('released'),
            'brake_full': c.get('brake', {}).get('full'),
            'rc_fitted': c.get('brake', {}).get('rc_fitted')})
    elif kind == 'devices':
        found = [{'name': d.get('name'), 'address': d.get('address'),
                  'answered': d.get('answered'), 'feedback': d.get('feedback')}
                 for d in line.get('list', [])]
        log.devices = {'answered': line.get('answered'), 'expected': line.get('expected'),
                       'list': found}
    elif kind == 'health':
        if line.get('link'):
            log.link.append(line['link'])
    elif kind == 'status':
        out, state = line.get('out', {}), line.get('state', {})
        log.statuses.append(Status(
            t, state.get('mode', '?'), bool(state.get('armed')), int(state.get('faults', 0)),
            out.get('dac') == 0 and line.get('high', {}).get('dac', 0) == 0, None,
            out.get('brake_counts'), None, out.get('steer_counts')))
        tick = line.get('tick')
        if tick:
            log.ticks.append((t, tick.get('worst_gap_us', 0), tick.get('overruns', 0)))


def parse(lines: Iterable[str]) -> Log:
    """Read a whole log: text lines and JSON lines, mixed as the firmware writes them."""
    log = Log()
    for raw in lines:
        text = ANSI.sub('', raw).strip()
        if not text:
            continue
        log.lines += 1
        brace = text.find('{"t":')
        if brace >= 0:
            try:
                read_json(log, json.loads(text[brace:]))
                continue
            except ValueError:
                pass            # cut short, or not JSON after all
        read_text(log, text)
    log.form = ' and '.join(sorted(log.forms)) or 'none'
    if log.reset is None and log.config.get('last_reset'):
        log.reset = str(log.config['last_reset'])
    return log


# --- The checks -----------------------------------------------------------------------

@dataclass
class Row:
    check: str
    result: str         # PASS, FAIL, WAIT, WARN, INFO
    detail: str


def faults_named(bits: int) -> str:
    names = [name for bit, name in FAULT_NAMES.items() if bits & bit]
    return f'{bits:#06x} ({", ".join(names)})' if names else f'{bits:#06x}'


def judge(log: Log, build: Optional[str], features: Optional[str], pi: str) -> List[Row]:
    """Each check's row, in the order of the table."""
    rows: List[Row] = []

    def row(check, ok, detail, bad='FAIL'):
        rows.append(Row(check, 'PASS' if ok else bad, detail))

    row('Log', log.lines > 0 and log.forms,
        f'{log.lines} lines, {log.form} form' if log.lines else
        'nothing read: the port, the cable, or the board is not running')
    if not log.forms:
        return rows
    c = log.config

    if not c:
        row('Build', False, 'no CONFIG line: an older build? Flash this one (pixi run esp flash)')
    elif build:
        row('Build', c.get('build') == build,
            f"{c.get('build')}" + ('' if c.get('build') == build else
                                   f', but this checkout is {build}: flash it again'))
    else:
        rows.append(Row('Build', 'INFO', f"{c.get('build')} (no commit to compare with)"))

    if c:
        have = set(filter(None, str(c.get('features', '')).split(',')))
        if features is not None:
            want = set(filter(None, features.split(',')))
            row('Features', have == want, f"{','.join(sorted(have))}"
                + ('' if have == want else f", expected {','.join(sorted(want))}"))
        elif 'qemu' in have:
            row('Features', False, 'a QEMU build: stand-in devices, never on the car')
        else:
            row('Features', 'legacy-uart' in have and not have - ALLOWED_FEATURES
                - {'legacy-uart'}, ','.join(sorted(have)) + ' (expected legacy-uart; '
                'uplink, json-log and wifi-status allowed)')

    reset = log.reset
    row('Last reset', reset in RESET_OK,
        'power-on' if reset in RESET_OK else
        f"{reset}: {RESET_MEANS.get(str(reset), 'not a power-on: read the log above the boot')}"
        if reset else 'not seen: no boot or CONFIG line')

    d = log.devices
    if d is None:
        row('Devices', False, 'no DEVICES line: an older build?')
    else:
        parts = []
        for dev in d['list']:
            word = 'answered' if dev['answered'] else 'MISSING'
            fb = f", feedback {dev['feedback']}" if dev.get('feedback') else ''
            parts.append(f"{dev['name']} {dev['address']:#04x} {word}{fb}")
        row('Devices', d['answered'] == d['expected'] == 4,
            f"{d['answered']}/{d['expected']}: " + '; '.join(parts))

    st = log.statuses
    if not st:
        row('Status lines', False, 'none: is it the right baud (115200)?')
        return rows

    idle = c.get('idle_brake')
    if st[0].brake is not None:         # text: the spine's 0..1
        brakes = sorted({s.brake for s in st})
        ok = idle is not None and all(abs(b - idle) < 0.005 for b in brakes)
        row('Brake', ok, f"{', '.join(f'{b:.2f}' for b in brakes)} (IDLE_BRAKE {idle})")
    else:
        lo, hi = c.get('brake_released'), c.get('brake_full')
        want = None if None in (lo, hi, idle) else round(lo + idle * (hi - lo))
        got = sorted({s.brake_counts for s in st})
        row('Brake', want is not None and got == [want],
            f'counts {got} (IDLE_BRAKE {idle}: {want})')

    row('Throttle', all(s.throttle_zero for s in st),
        '0 in every line' if all(s.throttle_zero for s in st) else 'NOT 0: the throttle is on')

    if st[0].steer_centred is not None:
        ok = all(s.steer_centred for s in st)
        row('Steering', ok, 'centred' if ok else 'not centred')
    else:
        centre = c.get('steer_centre')
        got = sorted({s.steer_counts for s in st})
        row('Steering', got == [centre], f'counts {got} (centre {centre})')

    modes = sorted({s.mode for s in st})
    rc_fitted = str(log.config.get('rc_fitted', '')).lower() == 'true'
    armed = any(s.armed for s in st)
    latched = all(log.latched) if log.latched else None
    detail = f"{', '.join(modes)}, {'ARMED' if armed else 'not armed'}" + (
        '' if latched is None else (', latched' if latched else ', NOT latched'))
    if log.lifecycle not in (None, 'Running'):
        rows.append(Row('Mode', 'FAIL', detail + f': still in {log.lifecycle}, its self-test '
                        'never passed (a device?)'))
    elif modes == ['stopped'] and not armed and latched is not False:
        rows.append(Row('Mode', 'PASS', detail))
    elif (set(modes) <= {'stopped', 'rc'} and not armed and latched is not False
          and rc_fitted and all(s.throttle_zero for s in st)):
        # With a receiver fitted, normal mode's arbiter gives the RC the car at power-up,
        # braked, until the stick or the computer asks (spine-core RcOwner, Arsh, 21 Nov):
        # the mode says Rc while the throttle is off and the brake on (the rows above).
        rows.append(Row('Mode', 'PASS', detail + ': the RC holds the car, braked, until '
                        'the stick or the computer asks'))
    elif (modes == ['stopped'] and not armed and set(log.safety) <= {'Latched', 'Degraded'}
          and any(s.faults & RC_SIGNAL_LOST for s in st)):
        # A lost RC outranks the latch in the FSM line (Safety::of): the board is braked,
        # and nothing is wrong with the firmware. With a receiver fitted (21 Nov) this is
        # what a transmitter that is off, or not set up, looks like.
        rows.append(Row('Mode', 'WAIT', 'stopped, not armed, braked: the RC is lost (safety '
                        'Degraded). The transmitter on, CH6 low reading about 1100 (End '
                        'points CH6 80 %: docs/car-day.md T1-T4), the left stick in the '
                        'middle; then this check again'))
    elif set(modes) <= {'stopped', 'computer'} and 'ok' in log.link:
        rows.append(Row('Mode', 'WARN', detail + ": the Pi's frames armed it. For this "
                        "check, start with the Pi's bridge off"))
    else:
        rows.append(Row('Mode', 'FAIL', detail))

    bits = 0
    for s in st:
        bits |= s.faults
    extra = bits & ~EXPECTED_FAULTS
    note = ''
    if bits & RC_SIGNAL_LOST:
        note = '; 256: the RC transmitter is off, or not bound'
    row('Faults', not extra, faults_named(bits) + note + (
        '' if not extra else f'; NOT expected: {faults_named(extra)}'))

    if log.ticks:
        first, last = log.ticks[0], log.ticks[-1]
        late = last[2] - first[2] if last[2] >= first[2] else last[2]
        ok = last[1] < WORST_GAP_LIMIT_US
        rows.append(Row('Tick', 'PASS' if ok else 'FAIL',
                        f'worst gap {last[1] / 1000:.1f} ms (under {WORST_GAP_LIMIT_US // 1000}); '
                        f'{late} late ticks in the window'))
        if late and ok:
            rows.append(Row('Late ticks', 'WARN' if 'json' in log.forms else 'INFO',
                            f'{late} ticks over 10 ms late: '
                            + ("the text log's prints wait on the UART; the timing test "
                               '(the board README) says more' if 'json' not in log.forms
                               else 'with json-log there should be none')))
    else:
        row('Tick', False, 'no TICK line or tick field')

    row('Restarts', not log.restarts, '; '.join(log.restarts) or 'none in the window')

    verdict = log.link[-1] if log.link else None
    says = {
        None: 'no HEALTH line: an older build?',
        'ok': 'OK: frames with a moving counter',
        'no_frames': "NO FRAMES: right while the Pi's bridge is off",
        'stale': 'STALE: frames came, then stopped',
        'counter_frozen': "COUNTER FROZEN: is the Orin's output mux running?",
        'bad_frames': 'BAD FRAMES: the cable, or frame_version in legacy.yaml (2 for this build)',
        'can': 'a CAN build: the spine bridge judges the link',
    }.get(verdict, str(verdict))
    if pi == 'on':
        row('Pi link', verdict == 'ok', says)
    elif pi == 'off':
        row('Pi link', verdict == 'no_frames', says, bad='WARN')
    else:
        rows.append(Row('Pi link', 'INFO' if verdict in ('ok', 'no_frames', 'can') else 'WARN',
                        says))
    return rows


def verdict_line(rows: List[Row]) -> str:
    failed = next((r.check for r in rows if r.result == 'FAIL'), None)
    if failed:
        return f'STOP: {failed}, roll back (docs/esp32-bench.md)'
    if any(r.result == 'WAIT' for r in rows):
        return ('NOT YET: the RC is lost, so the board holds its brake. No roll back: set the '
                'transmitter (docs/car-day.md T1-T4), then run this check again')
    return 'SAFE TO GO ON to the next step'


def table(rows: List[Row]) -> str:
    width = max(len(r.check) for r in rows)
    lines = [f"{'Check':<{width}}  Result  What it saw", f"{'-' * width}  ------  {'-' * 11}"]
    lines += [f'{r.check:<{width}}  {r.result:<6}  {r.detail}' for r in rows]
    return '\n'.join(lines)


# --- Reading the port -----------------------------------------------------------------

def read_port(port: str, seconds: float, save: Optional[Path]) -> List[str]:
    """Read the serial port at 115200 8N1 for `seconds`; send nothing."""
    import termios
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0                                    # iflag: raw
        attrs[1] = 0                                    # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0                                    # lflag: no echo, not canonical
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
        data = b''
        end = time.monotonic() + seconds
        while (left := end - time.monotonic()) > 0:
            ready, _, _ = select.select([fd], [], [], min(left, 0.5))
            if ready:
                try:
                    data += os.read(fd, 4096)
                except BlockingIOError:
                    pass
    finally:
        os.close(fd)
    text = data.decode(errors='replace')
    if save:
        save.write_text(text)
    return text.splitlines()


def this_commit() -> Optional[str]:
    try:
        out = subprocess.run(['git', 'rev-parse', '--short=7', 'HEAD'], cwd=ROOT,
                             capture_output=True, text=True, check=True)
        return out.stdout.strip() or None
    except (OSError, subprocess.CalledProcessError):
        return None


def default_port() -> Optional[str]:
    for pattern in ('cu.usbserial-*', 'cu.SLAB_USBtoUART*', 'ttyUSB*', 'ttyACM*'):
        found = sorted(Path('/dev').glob(pattern))
        if found:
            return str(found[0])
    return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n\n')[0])
    ap.add_argument('port', nargs='?', help="the board's serial port (default: the first "
                    'USB serial port found)')
    ap.add_argument('--file', type=Path, help='read a saved log instead of the port')
    ap.add_argument('--seconds', type=float, default=SECONDS, help='how long to read the port')
    ap.add_argument('--save', type=Path, help='save what was read from the port here')
    ap.add_argument('--build', help="the commit expected (default: this checkout's)")
    ap.add_argument('--features', help='the exact features expected, comma-separated '
                    '(default: legacy-uart; uplink, json-log and wifi-status allowed)')
    ap.add_argument('--pi', choices=('any', 'on', 'off'), default='any',
                    help="is the Pi's bridge sending? on: the link must be OK")
    ap.add_argument('--json', action='store_true', help='print the result as JSON')
    args = ap.parse_args(argv)
    try:
        if args.file:
            lines = args.file.read_text(errors='replace').splitlines()
        else:
            port = args.port or default_port()
            if not port:
                print('no USB serial port found: plug the board in, or give its port '
                      '(vendor/cargo-tools/bin/espflash list-ports)', file=sys.stderr)
                return 2
            if not args.json:
                print(f'Reading {port} for {args.seconds:.0f} s; it sends nothing. '
                      'Close any other monitor (espflash) first.', flush=True)
            lines = read_port(port, args.seconds, args.save)
    except OSError as e:
        print(f'could not read the log: {e}', file=sys.stderr)
        return 2
    rows = judge(parse(lines), args.build or this_commit(), args.features, args.pi)
    final = verdict_line(rows)
    if args.json:
        print(json.dumps({'rows': [r.__dict__ for r in rows], 'verdict': final,
                          'safe': final.startswith('SAFE')}, indent=1))
    else:
        print(table(rows))
        print()
        print(final)
    return 0 if final.startswith('SAFE') else 1


if __name__ == '__main__':
    sys.exit(main())
