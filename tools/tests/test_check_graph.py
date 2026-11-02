"""check_graph: the localization health line, without ROS running."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import check_graph as cg  # noqa: E402  (it imports rclpy: run inside pixi)


def test_ok_is_one_line():
    assert cg.health_line('OK') == 'ok    localization health: OK'


def test_a_warn_lists_each_check_on_its_own_line_and_says_it_is_not_a_failure():
    line = cg.health_line('WARN gyro_bias WARN: 0.6 deg/s; tf STALE: nothing on /tf')
    assert line.splitlines() == [
        'WARN  localization health (warn only, not a failure):',
        '      gyro_bias WARN: 0.6 deg/s',
        '      tf STALE: nothing on /tf',
    ]


def test_nothing_heard():
    assert 'nothing heard yet' in cg.health_line(None)
