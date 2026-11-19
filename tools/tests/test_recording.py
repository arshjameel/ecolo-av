"""recording: the size estimate, the disk, the clocks, rates, the summary and its verdicts."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import recording as r  # noqa: E402
import pytest  # noqa: E402, I100

TRACKING = """Reference ID    : C0A8018C (192.168.1.140)
Stratum         : 3
Ref time (UTC)  : Thu Nov 19 15:00:00 2026
System time     : 0.000412000 seconds slow of NTP time
Last offset     : -0.000210000 seconds
Leap status     : Normal
"""


def test_size_estimate():
    assert r.bytes_per_second([(10, 1000), (200, 100)]) == 30_000
    assert r.gb_per_hour([(10, 1_000_000)]) == pytest.approx(36.0)


def test_car_estimate_is_dominated_by_lidar_and_cameras():
    groups = r.by_group(r.streams(sim=False))
    assert groups['lidar'] == pytest.approx((922_000 + 92_000) * 10 * 3600 / 1e9)
    assert groups['cameras'] > 40.0          # three JPEG cameras at 15 Hz
    small = sum(v for k, v in groups.items() if k not in ('lidar', 'cameras'))
    assert small < 2.0


def test_streams_by_where():
    car = {s.topic for s in r.streams(sim=False)}
    sim = {s.topic for s in r.streams(sim=True)}
    assert '/sensors/lidar/packets' in car and '/sensors/lidar/packets' not in sim
    assert '/clock' in sim and '/clock' not in car
    raw = {s.topic: s for s in r.streams(sim=False, raw_cameras=True)}
    assert '/sensors/camera/rear/image_raw' in raw
    assert raw['/sensors/camera/rear/image_raw'].size > 6_000_000


def test_every_need_names_recorded_topics():
    topics = {s.topic for s in r.STREAMS}
    for items in r.NEEDS.values():
        for options in items:
            assert any(t in topics for t in options), options


def test_disk():
    rate = 10e9 / 3600                      # 10 GB an hour
    assert r.disk_check(100e9, rate).status == 'OK'
    assert r.disk_check(5e9, rate).status == 'WARN'       # 30 min
    assert r.disk_check(1e9, rate).status == 'STOP'       # 6 min
    assert '30 min' in r.disk_check(5e9, rate).detail


def test_chronyc():
    t = r.parse_chronyc(TRACKING)
    assert t['offset'] == pytest.approx(-0.000412)
    assert t['leap'] == 'Normal'
    assert r.clock_line('orin', t).status == 'OK'
    fast = r.parse_chronyc(TRACKING.replace('0.000412000 seconds slow', '0.050 seconds fast'))
    assert fast['offset'] == pytest.approx(0.05)
    assert r.clock_line('orin', fast).status == 'WARN'
    unsync = dict(t, leap='Not synchronised')
    assert r.clock_line('orin', unsync).status == 'WARN'
    assert r.parse_chronyc('506 Cannot talk to daemon') is None
    assert 'not checked' in r.clock_line('pi', None, 'no chronyc').detail


def test_stamp_lag():
    assert r.stamp_lag_line('/x', []) is None
    assert r.stamp_lag_line('/x', [0.01, 0.02, 0.015]).status == 'OK'
    assert 'ahead' in r.stamp_lag_line('/x', [-0.2, -0.3, -0.25]).detail
    assert r.stamp_lag_line('/x', [1.0, 1.2, 0.9]).status == 'WARN'


def test_rate_line():
    imu = r.Stream('/sensors/imu/data_raw', 'imu', 200.0, 330, 50.0, must=True)
    assert r.rate_line(imu, 1000, 5.0, sim=False).status == 'OK'
    assert r.rate_line(imu, 500, 5.0, sim=False).status == 'WARN'     # 100 Hz
    assert r.rate_line(imu, 250, 5.0, sim=True).status == 'OK'        # Ravensim's 50
    assert r.rate_line(imu, 0, 5.0, sim=True).status == 'WARN'
    estop = r.Stream('/safety/estop', 'safety', 0.0, 10)
    assert r.rate_line(estop, 0, 5.0, sim=False).status == 'OK'


def test_frames():
    assert r.missing_frames(['lidar_link', 'imu_link'], ('lidar_link', 'gps_link')) == \
        ['gps_link']


def test_topic_stats_gap_and_drops():
    times = [i * 0.1 for i in range(100)]            # 10 Hz for 10 s
    s = r.topic_stats('/l', times, [100] * 100, 10.0, 10.0)
    assert s.good and s.gaps == 0 and s.size == 10_000
    holed = times[:40] + times[60:]                 # 2 s missing
    s = r.topic_stats('/l', holed, [100] * 80, 10.0, 10.0)
    assert s.gaps == 1 and s.worst_gap == pytest.approx(2.1)
    assert not s.good and any('gaps' in f for f in s.flags)
    thin = times[::3]                               # 3.3 Hz against 10: dropped
    assert any('dropped' in f for f in r.topic_stats('/l', thin, [1] * 34, 10.0, 10.0).flags)
    # no expected rate: its own median period
    s = r.topic_stats('/x', times[:40] + times[60:], [1] * 80, 10.0)
    assert s.gaps == 1
    assert r.topic_stats('/none', [], [], 10.0, 10.0).flags == ['none recorded']


def _seen(rate, duration=10.0):
    n = int(rate * duration)
    return ([i / rate for i in range(n)], [100] * n)


def test_verdicts():
    every = {t: _seen(10.0) for items in r.NEEDS.values() for options in items
             for t in options[:1]}
    stats = r.summarise(every, dict.fromkeys(every, 10.0), 10.0)
    frames = list(r.SENSOR_FRAMES)
    assert set(r.verdicts(stats, frames).values()) == {'good'}
    # no frames: the map and the labels can't be placed
    v = r.verdicts(stats, [])
    assert v['NDT map'].startswith('no:') and 'lidar_link' in v['NDT map']
    assert v['SLAM'] == 'good'
    # no global pose, but the sim's truth: the map is still good
    del every['/localization/odom/global']
    every['/vehicle/sim/truth'] = _seen(10.0)
    stats = r.summarise(every, dict.fromkeys(every, 10.0), 10.0)
    assert r.verdicts(stats, frames)['NDT map'] == 'good'
    # the lidar with a gap: weak
    every['/sensors/lidar/points'] = ([t for t in _seen(10.0)[0] if not 4 < t < 6],
                                      [1] * 80)
    stats = r.summarise(every, dict.fromkeys(every, 10.0), 10.0)
    assert r.verdicts(stats, frames)['NDT map'].startswith('weak:')
    # no wheels at all
    del every['/vehicle/wheels']
    stats = r.summarise(every, {}, 10.0)
    assert 'wheels' in r.verdicts(stats, frames)['transfer functions']


def test_markdown_and_lost():
    stats = r.summarise({'/a': _seen(10.0)}, {'/a': 10.0, '/b': 5.0}, 10.0)
    page = r.markdown(stats, {'NDT map': 'good'}, 10.0, 5_000, 0)
    assert '| `/a` | 100 | 10.0 | 10 |' in page and 'none recorded' in page
    assert '| NDT map | good |' in page
    assert r.lost_messages('Recording stopped') == 0
    log = ('[WARN] [rosbag2_recorder]: Number of messages lost on the transport layer: 3\n'
           'Cache buffers lost messages per topic: \n\t/a: 2\nTotal lost: 2')
    assert r.lost_messages(log) == 5
    assert 'lost 5 messages' in r.markdown(stats, {}, 10.0, 1, 5)


def test_jitter_is_not_a_gap():
    times = [i * 0.02 for i in range(500)]          # 50 Hz
    times[100] += 0.05                              # 70 ms late: jitter
    s = r.topic_stats('/imu', times, [1] * 500, 10.0, 50.0)
    assert s.gaps == 0 and s.good
