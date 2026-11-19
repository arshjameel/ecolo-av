"""
The recording kit's maths, no ROS (testing plan, phase C; TODO 249).

| Part | What |
|---|---|
| `STREAMS` | every topic a mapping drive records: its rate and size on the car, its group |
| `NEEDS` | what each later job needs: the NDT map, SLAM, labelling, the transfer functions |
| `bytes_per_second`, `gb_per_hour`, `disk_check` | how big a drive gets, and whether it fits |
| `parse_chronyc`, `clock_line` | one computer's clock offset from `chronyc tracking` |
| `stamp_lag_line` | a clock check through ROS: how old a message's stamp is when it arrives |
| `rate_line`, `missing_frames` | before: each topic at its rate; the sensor frames |
| `topic_stats`, `summarise`, `verdicts`, `markdown` | the after-summary |

tools/record_drive.py is the ROS part. The numbers are docs/bandwidth.md's and the preflight's
(aav_actuator_tests preflight.SENSORS); tools/tests/test_recording.py checks the maths.
"""

from dataclasses import dataclass, field
import re
import statistics
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

GB = 1e9
GAP_PERIODS = 3.0       # a gap is over 3 periods
MIN_GAP = 0.1           # s, and over this: shorter is the computer's own jitter
LOW_RATE = 0.8          # under 80 % of the expected rate: messages dropped
CLOCK_WARN = 0.010      # s: car-day.md's "System time under 0.01 s"
STAMP_LAG_MAX = 0.5     # s: a stamp older than this on arrival
STAMP_AHEAD = 0.020     # s: a stamp this far in the future: that computer's clock is ahead
WARN_HOURS = 1.0        # warn with under an hour of disk
REFUSE_MINUTES = 10.0   # refuse with under 10 minutes


@dataclass(frozen=True)
class Stream:
    """One recorded topic. rate 0: on change, or latched (no rate to check)."""

    topic: str
    group: str
    rate: float           # Hz on the car
    size: int             # bytes a message on the car (before compression)
    standin_rate: Optional[float] = None   # Hz in Ravensim, when it's checked there
    must: bool = False    # the before-check warns loudly when it's missing
    sim_only: bool = False
    car_only: bool = False


def _cameras():
    out = []
    for cam in ('front_left', 'front_right', 'rear'):
        base = f'/sensors/camera/{cam}'
        # JPEG, ~300 kB at 1080p (docs/bandwidth.md); Ravensim's are smaller.
        out.append(Stream(f'{base}/image_raw/compressed', 'cameras', 15.0, 300_000,
                          must=True))
        out.append(Stream(f'{base}/camera_info', 'cameras', 15.0, 400, must=True))
    return out


# The car's rates: the preflight's SENSORS where it has them (merged in by record_drive.py,
# with their stand-in rates), else interfaces.yaml's notes; sizes from docs/bandwidth.md
# or the message's own layout.
STREAMS: Tuple[Stream, ...] = (
    # the lidar: points for the map, and its packets (the truly raw: 10 % of the size)
    Stream('/sensors/lidar/points', 'lidar', 10.0, 922_000, must=True),
    Stream('/sensors/lidar/packets', 'lidar', 10.0, 92_000, car_only=True),
    # the IMU at its full rate
    Stream('/sensors/imu/data_raw', 'imu', 200.0, 330, must=True),
    Stream('/sensors/imu/data', 'imu', 200.0, 330),
    Stream('/sensors/imu/mag', 'imu', 200.0, 90, car_only=True),
    # GPS, raw
    Stream('/sensors/gps/fix', 'gps', 1.0, 150, must=True),
    Stream('/sensors/gps/fix_velocity', 'gps', 1.0, 380),
    Stream('/sensors/gps/vel', 'gps', 0.0, 90, car_only=True),
    Stream('/sensors/gps/time_reference', 'gps', 0.0, 60, car_only=True),
    Stream('/sensors/radar/points', 'radar', 10.0, 5_000),
    *_cameras(),
    # the wheels and the car's own state
    Stream('/vehicle/wheels', 'vehicle', 100.0, 70, must=True),
    Stream('/vehicle/odom', 'vehicle', 100.0, 720, must=True),
    Stream('/vehicle/steering', 'vehicle', 50.0, 60, must=True),
    Stream('/vehicle/throttle', 'vehicle', 50.0, 60),
    Stream('/vehicle/brake', 'vehicle', 50.0, 60),
    Stream('/vehicle/status', 'vehicle', 10.0, 120),
    Stream('/vehicle/joint_states', 'vehicle', 50.0, 120, car_only=True),
    Stream('/joint_states', 'vehicle', 50.0, 120),
    Stream('/vehicle/rc', 'vehicle', 50.0, 40, car_only=True),
    Stream('/vehicle/steering_actuator/raw', 'vehicle', 50.0, 40, car_only=True),
    Stream('/vehicle/brake_actuator/raw', 'vehicle', 50.0, 40, car_only=True),
    Stream('/vehicle/esp32/uplink_health', 'vehicle', 1.0, 60, car_only=True),
    Stream('/vehicle/battery', 'vehicle', 1.0, 200),
    Stream('/can/rx', 'vehicle', 200.0, 40, car_only=True),
    Stream('/can/tx', 'vehicle', 60.0, 40, car_only=True),
    # the commands, and who is driving
    Stream('/ego/actuator_cmd', 'commands', 50.0, 60, must=True),
    Stream('/control/cmd_vel', 'commands', 0.0, 70),
    *(Stream(f'/drivers/{d}/cmd_vel', 'commands', 0.0, 70)
      for d in ('joy', 'keyboard', 'rc', 'tests', 'scripted', 'point', 'nav', 'buttons')),
    Stream('/drivers/joy/joy', 'commands', 20.0, 100, car_only=True),
    Stream('/control/command_mux/state', 'commands', 10.0, 200, must=True),
    Stream('/safety/estop', 'safety', 0.0, 10),
    Stream('/safety/sensor_watchdog/state', 'safety', 10.0, 300, must=True),
    Stream('/safety/sensor_watchdog/lock', 'safety', 10.0, 10),
    Stream('/safety/output_mux/state', 'safety', 2.0, 100),
    Stream('/safety/geofence/state', 'safety', 0.0, 100),
    Stream('/system/source', 'safety', 0.0, 40),
    # where the car thinks it is (the map's poses on a real drive)
    Stream('/localization/odom/local', 'localization', 30.0, 720),
    Stream('/localization/odom/global', 'localization', 30.0, 720, must=True),
    Stream('/localization/gps/odom', 'localization', 1.0, 720),
    Stream('/localization/ndt/pose', 'localization', 0.0, 400),
    Stream('/localization/health', 'localization', 0.0, 200),
    # the frames
    Stream('/tf', 'frames', 100.0, 200, must=True),
    Stream('/tf_static', 'frames', 0.0, 2_000, must=True),
    Stream('/robot_description', 'frames', 0.0, 20_000),
    # Ravensim only: its clock and the truth (to score a map made from the car's poses)
    Stream('/clock', 'sim', 0.0, 12, sim_only=True),
    Stream('/vehicle/sim/truth', 'sim', 50.0, 720, sim_only=True),
)

# The sensor frames /tf_static must hold (vehicle.yaml's sensors).
SENSOR_FRAMES = ('lidar_link', 'imu_link', 'gps_link', 'radar_link',
                 'camera_left_link', 'camera_right_link', 'camera_rear_link')

# What each later job needs. Each item: any one of these topics, good.
NEEDS: Dict[str, Tuple[Tuple[str, ...], ...]] = {
    'NDT map': (('/sensors/lidar/points',),
                ('/localization/odom/global', '/vehicle/sim/truth'),
                ('/tf_static',)),
    'SLAM': (('/sensors/lidar/points',), ('/sensors/imu/data_raw',),
             ('/vehicle/odom', '/vehicle/wheels'), ('/tf_static',)),
    'labelling': (('/sensors/camera/front_left/image_raw/compressed',
                   '/sensors/camera/front_left/image_raw'),
                  ('/sensors/camera/front_left/camera_info',),
                  ('/sensors/lidar/points',), ('/tf',), ('/tf_static',)),
    'transfer functions': (('/ego/actuator_cmd',), ('/vehicle/steering',),
                           ('/vehicle/throttle',), ('/vehicle/brake',),
                           ('/vehicle/wheels',), ('/sensors/imu/data_raw',)),
}


def streams(sim: bool, raw_cameras: bool = False) -> List[Stream]:
    """Return the streams for the car (sim False) or Ravensim, cameras raw or JPEG."""
    out = []
    for s in STREAMS:
        if (s.sim_only and not sim) or (s.car_only and sim):
            continue
        if raw_cameras and s.topic.endswith('/image_raw/compressed'):
            # 1080p RGB: 6.2 MB a picture (docs/bandwidth.md's 93 MB/s at 15 Hz)
            s = Stream(s.topic[:-len('/compressed')], s.group, s.rate, 6_220_800,
                       s.standin_rate, s.must)
        out.append(s)
    return out


# --- how big ---------------------------------------------------------------------------

def bytes_per_second(rated: Sequence[Tuple[float, int]]) -> float:
    """Return the bytes a second of (rate Hz, bytes a message) pairs."""
    return sum(rate * size for rate, size in rated)


def gb_per_hour(rated: Sequence[Tuple[float, int]]) -> float:
    return bytes_per_second(rated) * 3600.0 / GB


def by_group(items: Sequence[Stream]) -> Dict[str, float]:
    """Return GB an hour per group of streams (the car's rates and sizes)."""
    out: Dict[str, float] = {}
    for s in items:
        out[s.group] = out.get(s.group, 0.0) + gb_per_hour([(s.rate, s.size)])
    return out


@dataclass(frozen=True)
class Check:
    """One before-check: OK, WARN (record anyway) or STOP (refuse, unless --force)."""

    status: str
    what: str
    detail: str


def disk_check(free_bytes: float, rate_bytes_per_s: float) -> Check:
    """Return how long the free disk lasts at a rate."""
    if rate_bytes_per_s <= 0:
        return Check('OK', 'disk', f'{free_bytes / GB:.0f} GB free')
    hours = free_bytes / rate_bytes_per_s / 3600.0
    text = (f'{free_bytes / GB:.0f} GB free; at {rate_bytes_per_s * 3600 / GB:.1f} GB/h '
            f'that is {hours * 60:.0f} min' if hours < 2 else
            f'{free_bytes / GB:.0f} GB free; at {rate_bytes_per_s * 3600 / GB:.1f} GB/h '
            f'that is {hours:.1f} h')
    if hours * 60 < REFUSE_MINUTES:
        return Check('STOP', 'disk', text)
    if hours < WARN_HOURS:
        return Check('WARN', 'disk', text + ' (under an hour)')
    return Check('OK', 'disk', text)


# --- the clocks ------------------------------------------------------------------------

def parse_chronyc(text: str) -> Optional[Dict[str, object]]:
    """
    Return chronyc tracking's offset (s, + = this clock ahead), leap status and source.

    None when the text isn't chronyc tracking's.
    """
    found = re.search(r'^System time\s*:\s*([0-9.]+) seconds (fast|slow) of NTP time',
                      text, re.M)
    if not found:
        return None
    offset = float(found.group(1)) * (1 if found.group(2) == 'fast' else -1)
    leap = re.search(r'^Leap status\s*:\s*(.+)$', text, re.M)
    ref = re.search(r'^Reference ID\s*:\s*(.+)$', text, re.M)
    return {'offset': offset, 'leap': leap.group(1).strip() if leap else '',
            'reference': ref.group(1).strip() if ref else ''}


def clock_line(computer: str, tracking: Optional[Dict[str, object]], why: str = '') -> Check:
    """Judge one computer's chronyc tracking (None: not read, why says why)."""
    if tracking is None:
        return Check('WARN', f'clock {computer}', f'not checked: {why}')
    offset = float(tracking['offset'])
    text = f"{offset * 1000:+.1f} ms from {tracking['reference'] or 'its source'}"
    if tracking['leap'] and tracking['leap'] != 'Normal':
        return Check('WARN', f'clock {computer}', f"{text}; leap status {tracking['leap']}: "
                     'not synchronised')
    if abs(offset) > CLOCK_WARN:
        return Check('WARN', f'clock {computer}', text + f' (over {CLOCK_WARN * 1000:.0f} ms)')
    return Check('OK', f'clock {computer}', text)


def stamp_lag_line(topic: str, lags: Sequence[float]) -> Optional[Check]:
    """
    Judge how old a topic's header stamps are on arrival (s; median).

    Negative: stamped in this computer's future, so the sender's clock is ahead. This
    checks the clocks through ROS, on any computer, chrony or not.
    """
    if not lags:
        return None
    lag = statistics.median(lags)
    text = f'stamps {lag * 1000:+.0f} ms old on arrival'
    if lag < -STAMP_AHEAD:
        return Check('WARN', f'stamps {topic}', text + ": its computer's clock is ahead")
    if lag > STAMP_LAG_MAX:
        return Check('WARN', f'stamps {topic}', text + ": late, or its computer's clock is "
                     'behind')
    return Check('OK', f'stamps {topic}', text)


# --- before: present, at its rate ------------------------------------------------------

def rate_line(stream: Stream, count: int, seconds: float, sim: bool) -> Check:
    """Judge one topic sampled for some seconds against its expected rate."""
    expected = stream.standin_rate if sim else stream.rate
    if count == 0:
        if stream.rate == 0 and not stream.must:
            return Check('OK', stream.topic, 'quiet (sends on change)')
        return Check('WARN', stream.topic, 'nothing heard')
    rate = count / seconds if seconds > 0 else 0.0
    if expected and rate < LOW_RATE * expected:
        return Check('WARN', stream.topic, f'{rate:.1f} Hz, expected {expected:g}')
    return Check('OK', stream.topic, f'{rate:.1f} Hz' if stream.rate or count > 1
                 else 'heard')


def missing_frames(frames: Sequence[str], needed: Sequence[str] = SENSOR_FRAMES
                   ) -> List[str]:
    return [f for f in needed if f not in set(frames)]


# --- after: the summary ----------------------------------------------------------------

@dataclass
class TopicStats:
    topic: str
    count: int
    rate: float              # Hz over the recording
    size: int                # bytes, before the file's compression
    gaps: int                # gaps over GAP_PERIODS periods
    worst_gap: float         # s
    expected: Optional[float] = None
    flags: List[str] = field(default_factory=list)

    @property
    def good(self) -> bool:
        return self.count > 0 and not self.flags


def topic_stats(topic: str, times: Sequence[float], sizes: Sequence[int], duration: float,
                expected: Optional[float] = None) -> TopicStats:
    """
    Return one topic's count, rate, size and gaps from its arrival times (s).

    The period is the expected rate's, else the median of its own; a gap is over 3 of
    them and over 0.1 s (a 50 Hz topic 60 ms late is the computer's jitter, not a hole).
    Fewer messages than 80 % of expected x the recording's length: dropped.
    """
    count = len(times)
    rate = count / duration if duration > 0 else 0.0
    ordered = sorted(times)
    steps = [b - a for a, b in zip(ordered, ordered[1:])]
    if expected:
        period = 1.0 / expected
    elif len(steps) >= 5:
        period = statistics.median(steps)
    else:
        period = 0.0
    long_gaps = [s for s in steps if period > 0 and s > max(GAP_PERIODS * period, MIN_GAP)]
    stats = TopicStats(topic, count, rate, sum(sizes), len(long_gaps),
                       max(steps, default=0.0), expected)
    if count == 0:
        stats.flags.append('none recorded')
    elif expected and duration > 0 and count < LOW_RATE * expected * duration:
        stats.flags.append(f'dropped: {count} of ~{expected * duration:.0f}')
    if long_gaps:
        stats.flags.append(f'{len(long_gaps)} gaps, worst {max(long_gaps):.2f} s')
    return stats


def summarise(seen: Mapping[str, Tuple[Sequence[float], Sequence[int]]],
              expected: Mapping[str, Optional[float]],
              duration: float) -> List[TopicStats]:
    """Return every recorded or expected topic's stats, sorted by name."""
    topics = sorted(set(seen) | set(expected))
    return [topic_stats(t, *seen.get(t, ((), ())), duration, expected.get(t))
            for t in topics]


def verdicts(stats: Sequence[TopicStats], frames: Sequence[str] = (),
             needs: Mapping[str, Tuple[Tuple[str, ...], ...]] = NEEDS) -> Dict[str, str]:
    """
    Return, per later job, 'good' or 'no: <what's wrong>'.

    Each of a job's items needs one of its topics recorded without flags (or, failing
    that, recorded at all: 'weak'). The NDT map and labelling need the sensors' frames.
    """
    by = {s.topic: s for s in stats}
    out = {}
    for job, items in needs.items():
        missing, weak = [], []
        for options in items:
            got = [by[t] for t in options if t in by and by[t].count > 0]
            if not got:
                missing.append(' or '.join(options))
            elif not any(s.good for s in got):
                weak.append(f'{got[0].topic} ({"; ".join(got[0].flags)})')
        if job in ('NDT map', 'labelling'):
            need = ('lidar_link',) if job == 'NDT map' else ('lidar_link', 'camera_left_link')
            lost = missing_frames(frames, need)
            if lost:
                missing.append('/tf_static frames ' + ', '.join(lost))
        if missing:
            out[job] = 'no: missing ' + '; '.join(missing)
        elif weak:
            out[job] = 'weak: ' + '; '.join(weak)
        else:
            out[job] = 'good'
    return out


def human(n: float) -> str:
    for unit, scale in (('GB', 1e9), ('MB', 1e6), ('kB', 1e3)):
        if n >= scale:
            return f'{n / scale:.1f} {unit}'
    return f'{n:.0f} B'


def markdown(stats: Sequence[TopicStats], verdict: Mapping[str, str], duration: float,
             on_disk: int, lost: int = 0, notes: Sequence[str] = ()) -> str:
    """Return the summary as a short page: the verdicts, then one row per topic."""
    total = sum(s.size for s in stats)
    lines = ['# The recording', '',
             f'{duration:.0f} s; {human(total)} of messages, {human(on_disk)} on disk'
             + (f' ({on_disk / total:.0%} after compression)' if total else '') + '.', '',
             '## Good for', '', '| Job | Verdict |', '|---|---|']
    lines += [f'| {job} | {v} |' for job, v in verdict.items()]
    lines += ['', '## Every topic', '',
              '| Topic | Count | Rate (Hz) | Expected | Gaps (> 3 periods) | Size | Flags |',
              '|---|---|---|---|---|---|---|']
    for s in stats:
        expected = f'{s.expected:g}' if s.expected else '-'
        lines.append(f'| `{s.topic}` | {s.count} | {s.rate:.1f} | {expected} | '
                     f'{s.gaps} (worst {s.worst_gap:.2f} s) | {human(s.size)} | '
                     f'{"; ".join(s.flags) or "-"} |')
    lines += ['', f'The recorder lost {lost} messages (its own count: the network and its '
              'cache).' if lost else 'The recorder lost no messages (its own count).']
    lines += [f'- {n}' for n in notes]
    return '\n'.join(lines) + '\n'


def lost_messages(log: str) -> int:
    """
    Return the messages rosbag2 says it lost: on the network, and from its full cache.

    It says so only when there were some (rosbag2_transport recorder.cpp, rosbag2_cpp
    message_cache.cpp).
    """
    found = re.findall(r'lost on the transport layer: (\d+)|Total lost: (\d+)', log)
    return sum(int(a or b) for a, b in found)
