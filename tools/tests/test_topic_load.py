"""topic_load: the rates, sizes and loads `pixi run measure bandwidth` prints, on fake messages."""

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pytest  # noqa: E402
import topic_load as t  # noqa: E402

CAMERA = 640 * 360 * 3 + 60     # a Gazebo picture: RGB pixels and a small header


def fed(times, size):
    """Return a Meter fed one message of `size` bytes at each time."""
    meter = t.Meter()
    for stamp in times:
        meter.add(stamp, size)
    return meter


def test_a_camera_at_10_hz():
    meters = {'/cam': fed([i / 10 for i in range(50)], CAMERA)}
    (row,) = t.summary(meters, 5.0, types={'/cam': 'sensor_msgs/msg/Image'},
                       publishers={'/cam': ['gz_road_bridge']})
    assert row.hz == pytest.approx(10.0)
    assert row.avg == CAMERA and row.largest == CAMERA
    assert row.mb_s == pytest.approx(CAMERA * 10 / 1e6)          # about 6.9 MB/s
    assert row.mbit_s == pytest.approx(8 * row.mb_s)
    assert row.worst_gap == pytest.approx(0.1)
    assert row.publishers == ['gz_road_bridge']


def test_the_worst_gap_finds_a_stall():
    times = [0.0, 0.02, 0.04, 0.54, 0.56]
    assert fed(times, 10).worst_gap == pytest.approx(0.5)


def test_sizes_vary_and_the_load_is_their_sum():
    meter = t.Meter()
    for i, size in enumerate([100, 300, 200]):
        meter.add(float(i), size)
    (row,) = t.summary({'/x': meter}, 2.0)
    assert row.avg == pytest.approx(200.0)
    assert row.largest == 300
    assert row.mb_s == pytest.approx(600 / 2.0 / 1e6)


def test_a_silent_topic_and_a_single_message():
    rows = t.summary({'/quiet': t.Meter(), '/once': fed([1.0], 50)}, 10.0)
    by_name = {r.topic: r for r in rows}
    assert by_name['/quiet'].hz == 0 and by_name['/quiet'].avg == 0
    assert math.isnan(by_name['/quiet'].worst_gap)
    assert by_name['/once'].hz == pytest.approx(0.1)       # once in the window, not nothing
    assert math.isnan(by_name['/once'].worst_gap)          # one message has no gap


def test_biggest_load_first():
    meters = {'/imu': fed([i / 200 for i in range(200)], 320),
              '/cam': fed([i / 10 for i in range(10)], CAMERA),
              '/gps': fed([i / 10 for i in range(10)], 140)}
    assert [r.topic for r in t.summary(meters, 1.0)] == ['/cam', '/imu', '/gps']


def test_a_window_must_have_length():
    with pytest.raises(ValueError):
        t.summary({}, 0.0)


def test_people_readable_numbers():
    assert t.size_text(512) == '512 B'
    assert t.size_text(3200) == '3.2 kB'
    assert t.size_text(691260) == '691 kB'
    assert t.size_text(2.7e6) == '2.7 MB'
    assert t.rate_text(0) == '0'
    assert t.rate_text(0.0014) == '0.001'
    assert t.rate_text(0.0002) == '<0.001'
    assert t.rate_text(9.81) == '9.81'
    assert t.rate_text(200.4) == '200'


def test_the_total_adds_every_topic():
    rows = t.summary({'/a': fed([0.0, 0.5], 1000), '/b': fed([0.0, 0.25, 0.5, 0.75], 10)}, 1.0)
    end = t.total(rows)
    assert end.hz == pytest.approx(6.0)
    assert end.mb_s == pytest.approx((2000 + 40) / 1e6)
    assert end.largest == 1000


def test_the_plain_table_lines_up_and_ends_with_the_total():
    rows = t.summary({'/cam': fed([i / 10 for i in range(10)], CAMERA),
                      '/quiet': t.Meter()}, 1.0, types={'/cam': 'sensor_msgs/msg/Image'})
    text = t.table(rows)
    lines = text.splitlines()
    assert lines[0].split()[:3] == ['topic', 'Hz', 'size']
    assert lines[2].startswith('/cam') and 'sensor_msgs/Image' in lines[2]
    assert lines[-1].startswith('TOTAL')
    assert '/quiet' in text
    assert '/quiet' not in t.table(rows, quiet=True)


def test_the_markdown_table():
    rows = t.summary({'/cam': fed([i / 10 for i in range(10)], CAMERA)}, 1.0)
    lines = t.table(rows, markdown=True).splitlines()
    assert lines[0].startswith('| topic | Hz |')
    assert lines[1].startswith('|---|---:|')
    assert lines[2].startswith('| `/cam` | 10 | 691 kB |')
    assert lines[-1].startswith('| **TOTAL** |')
    assert all(line.count('|') == lines[0].count('|') for line in lines)
