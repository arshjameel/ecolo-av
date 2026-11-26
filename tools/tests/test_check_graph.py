"""check_graph: the localization health line and the profiles' rules, without ROS running."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import check_graph as cg  # noqa: E402  (it imports rclpy: run inside pixi)
import yaml  # noqa: E402

TOPICS = yaml.safe_load((cg.ROOT / 'docs' / 'interfaces.yaml').read_text())['topics']


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


def test_the_virtual_car_must_have_every_car_topic_but_the_ones_marked_with_a_reason():
    car = {n for n, t in TOPICS.items() if cg.must_be_live(t, 'car')}
    virtual = {n for n, t in TOPICS.items() if cg.must_be_live(t, 'virtual')}
    marked = {n for n, t in TOPICS.items() if t.get('not_virtual')}
    assert car - virtual == marked
    # It is Ravensim's road world too.
    assert {n for n, t in TOPICS.items() if cg.must_be_live(t, 'road')} <= virtual


def test_only_a_car_topic_is_excused_and_never_quietly():
    for name, t in TOPICS.items():
        if 'not_virtual' in t:
            assert 'car' in t.get('live', []), name
            assert len(str(t['not_virtual'])) > 20, f'{name}: say why'


def test_a_simulator_is_never_asked_for_the_car_s_commands():
    assert not cg.must_be_live(TOPICS['/control/car/actuator_cmd'], 'virtual')
    assert cg.must_be_live(TOPICS['/control/car/actuator_cmd'], 'car')
    assert cg.must_be_live(TOPICS['/safety/self_test/status'], 'virtual')
