"""
The ESP32 firmware in QEMU: the heartbeat, the uplink and the RC's modes, checked.

`pixi run esp qemu-check` (tools/esp32.sh qemu-check) builds the Rust firmware with
`--features qemu,uplink`, makes a flash image, and runs this. No ROS here, so the tests run
anywhere (tools/tests/test_esp32_qemu.py). docs/esp32-bench.md, "Before the board: the
emulator", has the picture and the results.

  this script --frames--> unix socket --> QEMU UART1 RX --> the firmware's LegacyLink
  this script <-uplink--- unix socket <-- QEMU UART1 TX <-- its uplink frames (TODO 50)
  this script --iBUS----> unix socket --> QEMU UART2 RX --> the RC receiver's parser
  this script <--lines--- unix socket <-- QEMU UART0 <-- the firmware's log

The frames are the real protocol: aav_legacy's own `pack_frame_v2` (version 2, TODO 50),
at the bridge's 50 Hz, with ego's values (`to_legacy`) and the Orin's counter, which
moves by one a frame while the Orin is alive. The firmware logs four kinds of line
(src/main.rs):

  EVENT   t=<ms> mode=.. armed=.. estop=.. throttle=.. brake=.. ... faults=..  on a change
  STATUS  the same, every 200 ms
  LINK    t=<ms> frames=.. bad=.. skipped=.. repeats=.. counter=..  with each STATUS: the
          link's counts since boot
  DEVICES t=<ms> steering=<Jrk> brake=<Jrk> dac=<code> expander=..  what the stand-in
          devices were last told: the bytes the car's Jrks and DAC would get

Each cycle: stand-still frames (the hold brake: the arming edge's stop), drive frames,
a gap, then drive frames again with a moving counter but no stop (it must stay
latched). The next cycle's stand-still frames re-arm it. The gap is one of three kinds:

  silence   no frames at all: the Motor-Pi or its cable is gone
  frozen    drive frames at 50 Hz, the counter frozen: the Pi alive, the Orin gone
  bad_crc   drive frames at 50 Hz, the counter moving, every CRC wrong: a bad cable.
            Each must be ignored (and counted: LINK bad=), so the heartbeat stops too

Each must brake within 100-200 ms of the last good frame with a new counter. Each
cycle's uplink frames (judge_uplink) must come at ~50 Hz, echo a counter the Orin had
just sent, say computer and armed while it drives, and say stopped with fault 1 within
100-250 ms of the last new counter; in a frozen gap the echo must stay the frozen one.

Then the RC (TODO 48; RC_PHASES): iBUS frames every 7 ms with the mode switch (CH6) in
the middle, RC via ROS: the sticks come up, and only the frames drive; at the top, RC
direct: the sticks drive, whatever the frames say; then silence in each: fault 256 and
full brake within 100-250 ms; and back on RC via ROS it stays braked until a stop.

Times: `t` is the firmware's clock (esp-hal's Instant, from QEMU's virtual clock, which
follows the host's clock without -icount). The host notes when each line arrives;
the smallest (arrival - t) maps one clock to the other (ClockMap).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
import re
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from typing import Iterable, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros/can_bridge/aav_legacy'))
from aav_legacy.legacy_protocol import (  # noqa: E402
    LegacyMapping, MODE_COMPUTER, MODE_RC_DIRECT, MODE_STOPPED, pack_frame, pack_frame_v2,
    stop_command, SWITCH_NONE, SWITCH_RC_DIRECT, SWITCH_RC_VIA_ROS, to_legacy, Uplink,
    UplinkParser)
STOP_COMMAND = stop_command  # the bridge's stale stop, for the tests

# The firmware's numbers (spine-core's SpineConfig, spine-board's Calibration::TODAY).
TIMEOUT_MS = 100            # cmd_timeout_ms
LIMIT_MS = 200              # what the bench test asks for: within 0.2 s (esp32-bench.md 8)
CMD_TIMEOUT = 0x0001        # faults::CMD_TIMEOUT
BRAKE_FULL = 3500           # the brake Jrk's target, full on
BRAKE_RELEASED = 400

RC_SIGNAL_LOST = 0x0100     # faults::RC_SIGNAL_LOST
# The uplink is judged by when its frames reach this computer, which a loaded host delays
# (the firmware's own times, from its log, are judged against LIMIT_MS): the timeout, a
# tick, up to 20 ms to the next uplink frame, and room for the host's scheduling.
UPLINK_LIMIT_MS = 400
ECHO_WINDOW = 0.5           # s: an echo must be a counter sent this recently

# The RC (TODO 48): the mode switch on CH6 (index 5), the proposal (RcMap::THREE_POSITION).
SWITCH = {'computer': 1000, 'via_ros': 1500, 'direct': 2000}
FULL_STICK = 1000           # the throttle stick (CH3) all the way: below the middle is go
FORWARD = 1000              # the gear switch (CH5) on forward
IBUS_PERIOD = 0.007         # s: a FlySky receiver sends a frame about every 7 ms

RATE_HZ = 50.0              # legacy_uart_bridge's rate_hz
DRIVE = (0.6, 0.0)          # ego's command while driving: 0.6 m/s, straight
STAND_STILL = (0.0, 0.0)    # ego standing still: the hold brake, a stop
KINDS = ('silence', 'frozen', 'bad_crc')
EFUSE_REV3 = bytes.fromhex(  # chip revision 3, as ESP-IDF's `idf.py qemu` writes it
    '00000000000000000000000000800000000000000000100000000000000000000000000000000000'
    + '00' * 80 + '00000000')

ANSI = re.compile(r'\x1b\[[0-9;]*m')
FIELD = re.compile(r'(\w+)=(\S+)')


# --- The firmware's lines -------------------------------------------------------------

@dataclass
class Line:
    """One line of the firmware's log, and when it reached this computer."""

    host: float                 # s, time.monotonic()
    kind: str                   # EVENT, STATUS, LINK, DEVICES, or TEXT
    t: Optional[int] = None     # the firmware's ms since boot
    fields: dict = field(default_factory=dict)
    text: str = ''

    def get(self, key):
        """Return a field as a number or a word (bool for true/false)."""
        value = self.fields[key]
        if value in ('true', 'false'):
            return value == 'true'
        try:
            return int(value, 0)
        except ValueError:
            try:
                return float(value)
            except ValueError:
                return value


# A field near each kind's end (main.rs: log_line, the LINK and DEVICES lines). A line
# without it arrived cut short (QEMU's UART drops bytes on a busy computer): it is CUT,
# not judged, and the summary counts it.
LAST_FIELD = {'EVENT': 'faults', 'STATUS': 'faults', 'LINK': 'counter',
              'DEVICES': 'expander'}


def parse_line(text: str, host: float) -> Line:
    """Read one log line; anything that isn't EVENT, STATUS, LINK or DEVICES is TEXT."""
    text = ANSI.sub('', text).strip()
    for kind, last in LAST_FIELD.items():
        marker = kind + ' t='
        at = text.find(marker)
        if at >= 0:
            fields = dict(FIELD.findall(text[at + len(kind):]))
            if last not in fields:
                return Line(host, 'CUT', text=text)
            return Line(host, kind, int(fields['t']), fields, text)
    return Line(host, 'TEXT', text=text)


@dataclass
class ClockMap:
    """The firmware's clock against the host's, from the lines' arrival times."""

    offset: float               # s: host = firmware + offset, for the fastest line
    rate: float                 # firmware seconds per host second, over the run

    @classmethod
    def fit(cls, lines: Iterable[Line]) -> 'ClockMap':
        timed = [ln for ln in lines if ln.t is not None]
        if len(timed) < 2:
            raise ValueError('need two timed lines to map the clocks')
        offset = min(ln.host - ln.t / 1000 for ln in timed)
        first, last = timed[0], timed[-1]
        span = last.host - first.host
        rate = (last.t - first.t) / 1000 / span if span > 0 else 1.0
        return cls(offset, rate)

    @classmethod
    def fit_local(cls, lines: Iterable[Line], start: float, end: float,
                  margin: float = 1.0) -> 'ClockMap':
        """
        Fit the clocks on the lines that arrived between `start` and `end` (host s).

        One cycle's own map: on a loaded computer QEMU's clock can drift from the host's
        over a run, so the fastest line of the whole run (fit) can be seconds away.
        """
        near = [ln for ln in lines if start - margin <= ln.host <= end + margin]
        return cls.fit(near if len(near) >= 2 else lines)

    def to_firmware_ms(self, host: float) -> float:
        """Return the firmware's ms at a host time."""
        return (host - self.offset) * 1000


# --- One cycle, judged ----------------------------------------------------------------

@dataclass
class Cycle:
    """What was sent in one cycle (host times, s)."""

    stop_from: float
    drive_from: float
    last_frame: float           # the last good drive frame with a new counter
    resume_from: float          # drive frames again, without a stop
    end: float
    kind: str = 'silence'       # the gap: silence, frozen or bad_crc (KINDS)
    gap_frames: int = 0         # frames sent in the gap
    # The counters of the frames sent from resume_from to end. A line whose counter= is one
    # of them is the firmware's word while those frames were the latest: no clock needed.
    resume_counters: frozenset = frozenset()


@dataclass
class Result:
    """One cycle's findings. `None`: it didn't happen."""

    drove: bool = False                     # armed, throttle on, brake released
    dac_rose: int = 0                       # the DAC's highest code while driving
    timeout_fw_ms: Optional[float] = None   # last frame -> the timeout event, firmware clock
    timeout_host_ms: Optional[float] = None  # the same, as the host saw the line arrive
    brake_full_fw_ms: Optional[float] = None  # last frame -> brake Jrk 3500 and DAC 0
    fault_bit: bool = False                 # the timeout event has CMD_TIMEOUT
    stopped: bool = False                   # it says throttle 0, brake 1.00, not armed
    latched: Optional[bool] = None          # fresh frames without a stop didn't drive
    kind: str = 'silence'
    gap_frames: int = 0                     # frames sent in the gap
    counted: Optional[int] = None           # the firmware's count: LINK repeats or bad
    problems: List[str] = field(default_factory=list)

    @property
    def ok(self):
        """Return True if this cycle did all it should."""
        return not self.problems


def judge(cycle: Cycle, lines: List[Line], clock: ClockMap) -> Result:
    """Check one cycle against what the firmware logged."""
    r = Result(kind=cycle.kind, gap_frames=cycle.gap_frames)
    fw = clock.to_firmware_ms
    t_drive, t_last = fw(cycle.drive_from), fw(cycle.last_frame)
    t_resume, t_end = fw(cycle.resume_from), fw(cycle.end)
    timed = [ln for ln in lines if ln.t is not None]
    status = [ln for ln in timed if ln.kind in ('EVENT', 'STATUS')]
    devices = [ln for ln in timed if ln.kind == 'DEVICES']

    driving = [ln for ln in status if t_drive <= ln.t <= t_last
               and ln.get('armed') and ln.get('mode') == 'Computer'
               and ln.get('throttle') > 0 and ln.get('brake') == 0]
    r.drove = bool(driving)
    while_driving = [ln for ln in devices if t_drive <= ln.t <= t_last]
    r.dac_rose = max((ln.get('dac') for ln in while_driving), default=0)
    if not r.drove:
        r.problems.append('never drove: no armed Computer line with throttle on')
    elif not any(ln.get('brake') == BRAKE_RELEASED for ln in while_driving):
        r.problems.append('the brake Jrk was never released while driving')

    tripped = next((ln for ln in timed if ln.kind == 'EVENT' and ln.t > t_last
                    and ln.get('faults') & CMD_TIMEOUT), None)
    if tripped is None:
        r.problems.append('no timeout event after the frames stopped')
    else:
        r.fault_bit = True
        r.timeout_fw_ms = tripped.t - t_last
        r.timeout_host_ms = (tripped.host - cycle.last_frame) * 1000
        r.stopped = (tripped.get('throttle') == 0 and tripped.get('brake') == 1.0
                     and not tripped.get('armed'))
        if not r.stopped:
            r.problems.append(f'the timeout event did not stop the car: {tripped.text}')
        if not TIMEOUT_MS <= r.timeout_fw_ms <= LIMIT_MS:
            r.problems.append(f'timeout after {r.timeout_fw_ms:.0f} ms, '
                              f'not {TIMEOUT_MS}-{LIMIT_MS}')
    braked = next((ln for ln in devices if ln.t > t_last
                   and ln.get('brake') == BRAKE_FULL and ln.get('dac') == 0), None)
    if braked is None:
        r.problems.append('the brake Jrk never got 3500 with the DAC at 0')
    else:
        r.brake_full_fw_ms = braked.t - t_last
        if r.brake_full_fw_ms > LIMIT_MS:
            r.problems.append(f'full brake after {r.brake_full_fw_ms:.0f} ms')

    # Fresh frames, no stop first: it must stay stopped (esp32-bench.md step 9). Which
    # lines came while they were the latest: by their counter= where the firmware prints
    # it, so the next cycle's stop frames never count however the clocks drift (seen on a
    # loaded computer, 24 Oct); else by the clocks.
    # The counter is one byte, so a later cycle sends the same numbers again: take only the
    # first unbroken run of lines on these counters.
    if cycle.resume_counters and all('counter' in ln.fields for ln in status):
        resumed = []
        for ln in (ln for ln in status if ln.t >= t_resume - 500):
            if ln.fields.get('counter') is not None \
                    and ln.get('counter') in cycle.resume_counters:
                resumed.append(ln)
            elif resumed:
                break
        if resumed:
            t_resume, t_end = resumed[0].t, resumed[-1].t
    else:
        resumed = [ln for ln in status if t_resume <= ln.t <= t_end]
    moved = [ln for ln in resumed if ln.get('armed') or ln.get('throttle') > 0
             or ln.get('brake') < 1.0]
    moved_devices = [ln for ln in devices if t_resume <= ln.t <= t_end
                     and (ln.get('dac') > 0 or ln.get('brake') != BRAKE_FULL)]
    if not resumed:
        r.problems.append('no lines while the frames came back')
    else:
        r.latched = not moved and not moved_devices
        if not r.latched:
            r.problems.append('fresh frames without a stop drove again: '
                              + (moved or moved_devices)[0].text)
        elif not all(ln.get('faults') & CMD_TIMEOUT for ln in resumed):
            r.problems.append('the timeout fault bit cleared without a stop')

    # The gap's frames: each one ignored, and counted (LINK lines).
    if cycle.kind in ('frozen', 'bad_crc'):
        counter = 'repeats' if cycle.kind == 'frozen' else 'bad'
        links = [ln for ln in timed if ln.kind == 'LINK']
        before = [ln for ln in links if ln.t <= t_last]
        after = [ln for ln in links if ln.t >= t_resume + 30]
        if not before or not after:
            r.problems.append('no LINK lines around the gap')
        else:
            r.counted = after[0].get(counter) - before[-1].get(counter)
            if r.counted < cycle.gap_frames - 1:
                r.problems.append(f'{cycle.gap_frames} {cycle.kind} frames sent, only '
                                  f'{r.counted} counted as {counter}')
    return r


# --- The uplink and the RC (TODO 50, 48) ----------------------------------------------

@dataclass
class Sent:
    """One downlink frame the Orin sent: when (host s), its counter, and if it was good."""

    host: float
    counter: int
    good: bool = True


@dataclass
class UplinkResult:
    """One cycle's uplink: what came back from the ESP32 while it drove, and after."""

    rate_hz: float = 0.0                    # uplink frames a second while driving
    echo_ok: Optional[bool] = None          # every echo was a counter the Orin had just sent
    computer: Optional[bool] = None         # mode computer and armed while driving
    stopped_ms: Optional[float] = None      # last new counter -> an uplink saying stopped, fault 1
    frozen_echo: Optional[bool] = None      # in a frozen gap: the echo stayed the frozen counter
    problems: List[str] = field(default_factory=list)

    @property
    def ok(self):
        """Return True if the uplink did all it should."""
        return not self.problems


def judge_uplink(cycle: Cycle, ups: List[Tuple[float, Uplink]], sent: List[Sent]) -> UplinkResult:
    """Check one cycle's uplink frames (host times), against the frames the Orin sent."""
    r = UplinkResult()
    driving = [(h, u) for h, u in ups if cycle.drive_from + 0.1 <= h <= cycle.last_frame]
    span = cycle.last_frame - cycle.drive_from - 0.1
    r.rate_hz = len(driving) / span if span > 0 else 0.0
    if not 40 <= r.rate_hz <= 60:
        r.problems.append(f'{r.rate_hz:.0f} uplink frames a second while driving, not ~50')
    good = [s for s in sent if s.good]

    def recent(h):
        return {s.counter for s in good if h - ECHO_WINDOW <= s.host <= h}
    wrong = [(h, u.echo) for h, u in driving if u.echo not in recent(h)]
    r.echo_ok = bool(driving) and not wrong
    if wrong:
        r.problems.append(f'{len(wrong)} echoes not among the counters just sent, '
                          f'such as {wrong[0][1]}')
    r.computer = bool(driving) and all(u.mode == MODE_COMPUTER and u.armed for _, u in driving)
    if not r.computer:
        r.problems.append('the uplink did not say computer and armed while driving')
    stop = next(((h, u) for h, u in ups if h > cycle.last_frame
                 and u.faults & CMD_TIMEOUT and u.mode == MODE_STOPPED and not u.armed), None)
    if stop is None:
        r.problems.append('no uplink frame said stopped with fault 1 after the counter stopped')
    else:
        r.stopped_ms = (stop[0] - cycle.last_frame) * 1000
        if not TIMEOUT_MS <= r.stopped_ms <= UPLINK_LIMIT_MS:
            r.problems.append(f'the uplink said stopped after {r.stopped_ms:.0f} ms, '
                              f'not {TIMEOUT_MS}-{UPLINK_LIMIT_MS}')
    if cycle.kind == 'frozen':
        last = [s.counter for s in good if s.host <= cycle.last_frame][-1:]
        gap = [u.echo for h, u in ups if cycle.last_frame + 0.05 <= h <= cycle.resume_from]
        r.frozen_echo = bool(gap) and set(gap) == set(last)
        if not r.frozen_echo:
            r.problems.append(f'in the frozen gap the echo was {sorted(set(gap))}, not {last}')
    return r


def lost_uplinks(ups: List[Tuple[float, Uplink]]) -> int:
    """Return how many uplink frames went missing: gaps in the sequence numbers."""
    return sum((b.sequence - a.sequence - 1) & 0xFF for (_, a), (_, b) in zip(ups, ups[1:]))


def ibus_frame(channels: List[int]) -> bytes:
    """One iBUS servo frame, as the FlySky receiver sends it (crate ibus: encode)."""
    body = bytes([0x20, 0x40]) + struct.pack('<14H', *channels)
    return body + struct.pack('<H', (0xFFFF - sum(body)) & 0xFFFF)


def rc_channels(switch: str, throttle: int = FULL_STICK) -> List[int]:
    """Return the 14 channels: the mode switch (CH6) at `switch`, the throttle as given."""
    ch = [1500] * 14
    ch[5] = SWITCH[switch]
    ch[2] = throttle
    ch[4] = FORWARD
    return ch


@dataclass
class RcPhase:
    """One step of the RC check (host times, s)."""

    name: str                           # a key of RC_PHASES
    start: float
    end: float
    rc_stop: Optional[float] = None     # when the last iBUS frame went, if it stopped


@dataclass
class RcResult:
    """One RC phase's findings."""

    name: str
    seen: str = ''                      # what the uplink and the devices showed
    ms: Optional[float] = None          # for a lost RC: the last iBUS frame -> braked
    problems: List[str] = field(default_factory=list)

    @property
    def ok(self):
        """Return True if the phase showed all it should."""
        return not self.problems


# What each phase does, and what it must show.
RC_PHASES = {
    'via_ros_still': 'switch on RC via ROS, full stick; the frames say stand still',
    'via_ros_drive': 'switch on RC via ROS; the frames drive (as the ROS driver would)',
    'direct': 'switch on RC direct, full stick; the frames say stand still',
    'direct_lost': 'RC direct, then the receiver goes silent',
    'via_ros_lost': 'RC via ROS, the frames driving, then the receiver goes silent',
    'via_ros_back': 'the receiver back on RC via ROS, the frames still driving, no stop',
}


def judge_rc(phase: RcPhase, ups: List[Tuple[float, Uplink]], lines: List[Line],
             clock: ClockMap) -> RcResult:
    """Check one RC phase against the uplink and the firmware's DEVICES lines."""
    r = RcResult(phase.name)
    settle = phase.start + 0.15
    window = [u for h, u in ups if settle <= h <= phase.end]
    fw_start, fw_end = clock.to_firmware_ms(settle), clock.to_firmware_ms(phase.end)
    devices = [ln for ln in lines if ln.kind == 'DEVICES' and fw_start <= ln.t <= fw_end]
    dac_peak = max((ln.get('dac') for ln in devices), default=0)
    if not window:
        r.problems.append('no uplink frames')
        return r

    def every(pred, what):
        if not all(pred(u) for u in window):
            bad = next(u for u in window if not pred(u))
            r.problems.append(f'{what}: an uplink said mode {bad.mode}, switch {bad.switch}, '
                              f'faults {bad.faults:#06x}')
    name = phase.name
    if name in ('via_ros_still', 'via_ros_drive'):
        every(lambda u: u.switch == SWITCH_RC_VIA_ROS and u.mode == MODE_COMPUTER,
              'not RC via ROS with the computer driving')
        every(lambda u: u.rc_throttle == 1.0, 'the sticks did not come up')
        if name == 'via_ros_still' and dac_peak:
            r.problems.append(f'the DAC rose to {dac_peak}: the ESP32 drove from the sticks')
        if name == 'via_ros_drive' and not dac_peak:
            r.problems.append('the frames did not drive')
        r.seen = f'switch RC via ROS, mode computer, RC throttle 1.0 up, DAC peak {dac_peak}'
    elif name == 'direct':
        every(lambda u: u.switch == SWITCH_RC_DIRECT and u.mode == MODE_RC_DIRECT,
              'not RC direct')
        if not dac_peak:
            r.problems.append('the sticks did not drive')
        r.seen = f'switch RC direct, mode RC direct, DAC peak {dac_peak} (the sticks)'
    elif name in ('direct_lost', 'via_ros_lost'):
        lost = next(((h, u) for h, u in ups if h > phase.rc_stop
                     and u.faults & RC_SIGNAL_LOST and u.switch == SWITCH_NONE), None)
        braked = next((ln for ln in lines if ln.kind == 'DEVICES'
                       and ln.t > clock.to_firmware_ms(phase.rc_stop)
                       and ln.get('brake') == BRAKE_FULL and ln.get('dac') == 0), None)
        if lost is None:
            r.problems.append('no uplink frame said fault 256 and no signal')
        else:
            r.ms = (lost[0] - phase.rc_stop) * 1000
            if not TIMEOUT_MS <= r.ms <= UPLINK_LIMIT_MS:
                r.problems.append(f'fault 256 came up after {r.ms:.0f} ms')
        if braked is None:
            r.problems.append('the brake Jrk never got 3500 with the DAC at 0')
        elif braked.t - clock.to_firmware_ms(phase.rc_stop) > LIMIT_MS + 20:
            r.problems.append(f'full brake only after '
                              f'{braked.t - clock.to_firmware_ms(phase.rc_stop):.0f} ms')
        end = [u for h, u in ups if phase.end - 0.2 <= h <= phase.end]
        if not end or not all(u.brake_out == 1.0 and u.throttle_out == 0.0 for u in end):
            r.problems.append('not still braked at the end')
        r.seen = 'fault 256, switch no signal, brake full, DAC 0'
    elif name == 'via_ros_back':
        every(lambda u: u.switch == SWITCH_RC_VIA_ROS, 'the switch is not back on RC via ROS')
        every(lambda u: not u.armed and u.mode == MODE_STOPPED and u.throttle_out == 0.0,
              'it drove again without a stop')
        if dac_peak:
            r.problems.append(f'the DAC rose to {dac_peak} without a stop first')
        r.seen = 'switch RC via ROS, stopped, latched: drive frames do not drive'
    if r.problems:
        r.seen = 'not what it should (below)'
    return r


# --- The run --------------------------------------------------------------------------

class Ibus:
    """The RC receiver: an iBUS frame every 7 ms on QEMU's UART2, or silence."""

    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.channels: Optional[List[int]] = None
        self.last_sent: Optional[float] = None
        self.lock = threading.Lock()
        threading.Thread(target=self._send, daemon=True).start()

    def send(self, channels: Optional[List[int]]) -> Optional[float]:
        """Send these channels from now on (None: silence); return the last frame's time."""
        with self.lock:
            self.channels = channels
            return self.last_sent

    def _send(self):
        while True:
            with self.lock:
                frame = ibus_frame(self.channels) if self.channels else None
                if frame:
                    try:
                        self.sock.sendall(frame)
                    except OSError:
                        return
                    self.last_sent = time.monotonic()
            time.sleep(IBUS_PERIOD)


class Firmware:
    """QEMU running the flash image: its log, the Motor-Pi's UART and the RC's on sockets."""

    def __init__(self, qemu: Path, flash: Path, workdir: Path):
        self.lines: List[Line] = []
        self.uplinks: List[Tuple[float, Uplink]] = []
        self.lock = threading.Lock()
        efuse = workdir / 'efuse.bin'
        efuse.write_bytes(EFUSE_REV3)
        log_sock, motor_sock = workdir / 'log.sock', workdir / 'motor.sock'
        ibus_sock = workdir / 'ibus.sock'
        self.proc = subprocess.Popen(
            [str(qemu), '-machine', 'esp32', '-m', '4M', '-display', 'none',
             '-monitor', 'none', '-nic', 'none',
             '-drive', f'file={flash},if=mtd,format=raw',
             '-drive', f'file={efuse},if=none,format=raw,id=efuse',
             '-global', 'driver=nvram.esp32.efuse,property=drive,value=efuse',
             '-serial', f'unix:{log_sock},server=on,wait=on',      # UART0: the log
             '-serial', f'unix:{motor_sock},server=on,wait=on',    # UART1: the Motor-Pi
             '-serial', f'unix:{ibus_sock},server=on,wait=on'],    # UART2: the RC's iBUS
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
        self.log = self._connect(log_sock)
        self.motor = self._connect(motor_sock)
        self.ibus = Ibus(self._connect(ibus_sock))
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._read_uplink, daemon=True).start()

    def _connect(self, path: Path, wait: float = 20.0) -> socket.socket:
        deadline = time.monotonic() + wait
        while True:
            try:
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.connect(str(path))
                return s
            except OSError:
                s.close()
                if time.monotonic() > deadline or self.proc.poll() is not None:
                    out = self.proc.stdout.read().decode(errors='replace') if \
                        self.proc.poll() is not None else ''
                    raise RuntimeError(f'QEMU did not open {path.name}: {out}')
                time.sleep(0.05)

    def _read(self):
        buf = b''
        while True:
            try:
                chunk = self.log.recv(4096)
            except OSError:
                return
            if not chunk:
                return
            now = time.monotonic()
            buf += chunk
            *done, buf = buf.split(b'\n')
            with self.lock:
                self.lines.extend(parse_line(d.decode(errors='replace'), now) for d in done)

    def _read_uplink(self):
        """Read the ESP32's TX1: its uplink frames (feature `uplink`), as the bridge does."""
        parser = UplinkParser()
        while True:
            try:
                chunk = self.motor.recv(4096)
            except OSError:
                return
            if not chunk:
                return
            now = time.monotonic()
            found = parser.feed(chunk)
            with self.lock:
                self.uplinks.extend((now, u) for u in found)

    def snapshot(self) -> List[Line]:
        with self.lock:
            return list(self.lines)

    def uplink_snapshot(self) -> List[Tuple[float, Uplink]]:
        with self.lock:
            return list(self.uplinks)

    def wait_for(self, pred, timeout: float) -> Line:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            found = next((ln for ln in self.snapshot() if pred(ln)), None)
            if found:
                return found
            if self.proc.poll() is not None:
                break
            time.sleep(0.02)
        raise RuntimeError('timed out waiting for the firmware; its log:\n'
                           + '\n'.join(ln.text for ln in self.snapshot()[-30:]))

    def send_for(self, values: Tuple[float, float, float], seconds: float,
                 frames=None) -> Tuple[float, int]:
        """
        Send one frame every 20 ms for `seconds`; return when the last went, and how many.

        `frames()` makes each frame (version 2, the Orin's counter); by default the
        version 1 frame of `values`.
        """
        frame = pack_frame(*values)
        start = time.monotonic()
        n = 0
        last = start
        while True:
            due = start + n / RATE_HZ
            if due - start >= seconds:
                return last, n
            pause = due - time.monotonic()
            if pause > 0:
                time.sleep(pause)
            self.motor.sendall(frames() if frames else frame)
            last = time.monotonic()
            n += 1

    def stop(self):
        self.ibus.send(None)
        for s in (self.motor, self.log, self.ibus.sock):
            s.close()
        self.proc.kill()
        self.proc.wait()


class Orin:
    """The Orin's counter and the Motor-Pi's frames: legacy_uart_bridge, frame_version 2."""

    def __init__(self):
        self.counter = 0
        self.sent: List[Sent] = []

    def frames(self, values, move=True, armed=True, bad_crc=False):
        """Return a frame maker: each call, the next frame (the counter +1, or frozen)."""
        def make():
            if move:
                self.counter = (self.counter + 1) & 0xFF
            frame = pack_frame_v2(*values, counter=self.counter, armed=armed, estop=False)
            if bad_crc:
                frame = frame[:-1] + bytes([frame[-1] ^ 0xFF])
            self.sent.append(Sent(time.monotonic(), self.counter, not bad_crc))
            return frame
        return make


def run(qemu: Path, flash: Path, cycles: int, rc: bool = True):
    """
    Boot the firmware, run the cycles (their gaps in turn: KINDS), then the RC's modes.

    Return the cycles' results, their uplink results, the RC results, the clock map, the
    lines and the uplink frames.
    """
    mapping = LegacyMapping()
    drive = to_legacy(*DRIVE, mapping)
    still = to_legacy(*STAND_STILL, mapping)
    orin = Orin()
    rc_phases: List[RcPhase] = []
    with tempfile.TemporaryDirectory(prefix='esp-qemu-', dir='/tmp') as tmp:
        fw = Firmware(qemu, flash, Path(tmp))
        try:
            fw.wait_for(lambda ln: ln.kind == 'STATUS', timeout=30)
            plan = []
            for i in range(cycles):
                kind = KINDS[i % len(KINDS)]
                stop_from = time.monotonic()
                fw.send_for(still, 0.3, orin.frames(still))
                drive_from = time.monotonic()
                last, _ = fw.send_for(drive, 1.0, orin.frames(drive))
                if kind == 'silence':
                    time.sleep(0.6)
                    gap = 0
                else:
                    gap_frames = (orin.frames(drive, move=False) if kind == 'frozen'
                                  else orin.frames(drive, bad_crc=True))
                    _, gap = fw.send_for(drive, 0.6, gap_frames)
                resume_from = time.monotonic()
                first = len(orin.sent)
                fw.send_for(drive, 0.6, orin.frames(drive))
                counters = frozenset(s.counter for s in orin.sent[first:])
                plan.append(Cycle(stop_from, drive_from, last, resume_from, time.monotonic(),
                                  kind, gap, counters))
            if rc:
                rc_phases = rc_run(fw, orin, mapping)
            fw.ibus.send(None)
            time.sleep(0.3)
            lines = fw.snapshot()
            ups = fw.uplink_snapshot()
        finally:
            fw.stop()
    clock = ClockMap.fit(lines)
    results = [judge(c, lines, ClockMap.fit_local(lines, c.stop_from, c.end)) for c in plan]
    uplink = [judge_uplink(c, ups, orin.sent) for c in plan]
    rc_results = [judge_rc(p, ups, lines, ClockMap.fit_local(lines, p.start, p.end))
                  for p in rc_phases]
    return results, uplink, rc_results, clock, lines, ups


def rc_run(fw: Firmware, orin: Orin, mapping: LegacyMapping) -> List[RcPhase]:
    """Run the RC's three modes through the firmware: the phases of RC_PHASES, in order."""
    drive = to_legacy(*DRIVE, mapping)
    still = to_legacy(*STAND_STILL, mapping)
    phases: List[RcPhase] = []

    def phase(name, values, seconds, rc_stop=None):
        start = time.monotonic()
        fw.send_for(values, seconds, orin.frames(values))
        phases.append(RcPhase(name, start, time.monotonic(), rc_stop))

    fw.ibus.send(rc_channels('via_ros'))
    phase('via_ros_still', still, 1.0)          # also the arming stop
    phase('via_ros_drive', drive, 1.0)
    fw.ibus.send(rc_channels('direct'))
    phase('direct', still, 1.0)
    phase('direct_lost', still, 0.6, rc_stop=fw.ibus.send(None))
    fw.ibus.send(rc_channels('via_ros', throttle=1500))
    fw.send_for(still, 0.5, orin.frames(still))  # stand still: a stop, so it drives again
    fw.send_for(drive, 1.0, orin.frames(drive))
    phase('via_ros_lost', drive, 0.6, rc_stop=fw.ibus.send(None))
    fw.ibus.send(rc_channels('via_ros', throttle=1500))
    phase('via_ros_back', drive, 0.6)
    return phases


def table(results: List[Result], clock: ClockMap) -> str:
    """Return the results as a markdown table, for the docs."""
    def ms(v):
        return '-' if v is None else f'{v:.0f}'

    def gap(r):
        if r.kind == 'silence':
            return '-'
        return f'{r.gap_frames} sent, {"-" if r.counted is None else r.counted} counted'
    rows = ['| Cycle | Gap | Drove (DAC peak) | Timeout, firmware ms | Seen on host, ms '
            '| Full brake + DAC 0, ms | Fault bit 1 | Gap frames ignored | Latched until a stop '
            '| Result |',
            '|---|---|---|---|---|---|---|---|---|---|']
    for i, r in enumerate(results, 1):
        rows.append(
            f'| {i} | {r.kind} | {"yes" if r.drove else "no"} ({r.dac_rose}) '
            f'| {ms(r.timeout_fw_ms)} | {ms(r.timeout_host_ms)} | {ms(r.brake_full_fw_ms)} '
            f'| {"yes" if r.fault_bit else "no"} | {gap(r)} | {"yes" if r.latched else "no"} '
            f'| {"pass" if r.ok else "FAIL"} |')
    rows.append('')
    rows.append(f'Clocks: firmware {clock.rate:.3f} s per host s over the run.')
    return '\n'.join(rows)


def uplink_table(results: List[UplinkResult], kinds: List[str], lost: int) -> str:
    """Return the uplink's results as a markdown table."""
    def yes(v):
        return '-' if v is None else ('yes' if v else 'no')
    rows = ['| Cycle | Gap | Uplink frames a second | Echo = the counter sent | Computer, armed '
            '| Said stopped + fault 1, ms | Echo frozen in the gap | Result |',
            '|---|---|---|---|---|---|---|---|']
    for i, (r, kind) in enumerate(zip(results, kinds), 1):
        rows.append(f'| {i} | {kind} | {r.rate_hz:.0f} | {yes(r.echo_ok)} | {yes(r.computer)} '
                    f'| {"-" if r.stopped_ms is None else f"{r.stopped_ms:.0f}"} '
                    f'| {yes(r.frozen_echo)} | {"pass" if r.ok else "FAIL"} |')
    rows.append('')
    rows.append(f'Uplink frames lost over the run (gaps in the sequence): {lost}.')
    return '\n'.join(rows)


def rc_table(results: List[RcResult]) -> str:
    """Return the RC's results as a markdown table."""
    rows = ['| Step | Does | Seen | Lost RC -> fault 256, ms | Result |', '|---|---|---|---|---|']
    for r in results:
        rows.append(f'| {r.name} | {RC_PHASES[r.name]} | {r.seen} '
                    f'| {"-" if r.ms is None else f"{r.ms:.0f}"} | {"pass" if r.ok else "FAIL"} |')
    return '\n'.join(rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--qemu', type=Path, required=True, help='qemu-system-xtensa')
    ap.add_argument('--flash', type=Path, required=True, help='the merged 4 MB flash image')
    ap.add_argument('--cycles', type=int, default=6,
                    help='cycles, their gaps in turn: silence, frozen, bad_crc')
    ap.add_argument('--no-rc', action='store_true', help='skip the RC modes after the cycles')
    ap.add_argument('--log', type=Path, help='save the firmware log here')
    args = ap.parse_args(argv)
    try:
        results, uplink, rc, clock, lines, ups = run(args.qemu, args.flash, args.cycles,
                                                     rc=not args.no_rc)
    except ValueError as e:
        # Seen once on 24 Oct at load 17: QEMU printed almost nothing, so no clocks to map.
        print(f'FAIL: the firmware in QEMU said too little to judge ({e}). '
              'A starved computer? Run it again with less else running.')
        return 1
    if args.log:
        args.log.write_text(''.join(f'{ln.host:.4f} {ln.text}\n' for ln in lines))
    boot = next((ln.text for ln in lines if 'spine-esp32-umod' in ln.text), '(no boot line)')
    print(boot)
    cut = sum(ln.kind == 'CUT' for ln in lines)
    if cut:
        print(f'{cut} log lines arrived cut short (a busy computer): not judged')
    print(table(results, clock))
    lost = lost_uplinks(ups)
    print()
    print(uplink_table(uplink, [r.kind for r in results], lost))
    if rc:
        print()
        print(rc_table(rc))
    problems = [(f'cycle {i}', p) for i, r in enumerate(results, 1) for p in r.problems]
    problems += [(f'cycle {i} uplink', p) for i, r in enumerate(uplink, 1) for p in r.problems]
    problems += [(r.name, p) for r in rc for p in r.problems]
    if not ups:
        problems.append(('uplink', 'no uplink frames at all: is it built with --features uplink?'))
    elif lost:
        problems.append(('uplink', f'{lost} uplink frames lost'))
    for where, p in problems:
        print(f'{where}: {p}')
    print('PASS: it brakes when the counter of the Orin stops (no frames, frozen, bad CRCs), '
          'and stays braked until a stop; the uplink echoes the counter and says the mode; '
          'RC via ROS, RC direct, and a lost RC in each' if not problems else 'FAIL')
    return 0 if not problems else 1


if __name__ == '__main__':
    sys.exit(main())
