"""How a measured value follows what was asked (tools/step_response.py): no ROS, no files."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest  # noqa: E402
import step_response as sr  # noqa: E402

DT = 0.02


def lagged(asked, dead=0.1, lag=0.2, start=0.0):
    """Return a first-order answer to `asked`, after a dead time: what a motor does."""
    out, value = [], start
    hold = round(dead / DT)
    for i in range(len(asked)):
        target = asked[max(0, i - hold)]
        value += (target - value) * DT / lag
        out.append(value)
    return out


def run(levels, each=5.0):
    """Return (t, asked) holding each level for `each` seconds."""
    n = round(each / DT)
    asked = [level for level in levels for _ in range(n)]
    return [i * DT for i in range(len(asked))], asked


def test_the_steps_are_found_where_the_asked_value_jumps():
    _, asked = run([0.0, 0.1, 0.2, 0.2, 0.0])
    found = sr.steps(asked, least=0.02)
    assert [(s.asked_from, s.asked_to) for s in found] == [(0.0, 0.1), (0.1, 0.2), (0.2, 0.0)]
    assert [s.first for s in found] == [250, 500, 1000]
    assert [s.last for s in found] == [500, 1000, 1250]


def test_a_wobble_is_not_a_step_and_does_not_move_the_level():
    _, asked = run([0.0, 0.3])
    asked[100] = 0.003          # the brake's own blip while the wheels turn
    found = sr.steps(asked, least=0.05)
    assert len(found) == 1 and found[0].asked_from == 0.0 and found[0].asked_to == 0.3


def test_a_jump_written_over_two_samples_is_one_step():
    asked = [0.0] * 10 + [0.15] + [0.3] * 10
    found = sr.steps(asked, least=0.05)
    assert [(s.first, s.asked_from, s.asked_to) for s in found] == [(10, 0.0, 0.3)]


def test_the_delay_and_the_rise_of_a_known_lag_are_the_textbook_ones():
    t, asked = run([0.0, 1.0])
    measured = lagged(asked, dead=0.1, lag=0.2)
    (only,) = sr.responses(t, asked, measured, least=0.02)
    # 10 % of a first-order rise is at 0.105 lags, 90 % at 2.303: after the dead time.
    assert only.delay == pytest.approx(0.1 + 0.2 * -math.log(0.9), abs=2 * DT)
    assert only.rise == pytest.approx(0.2 * (math.log(10) - -math.log(0.9)), abs=2 * DT)
    assert only.end == pytest.approx(1.0, abs=1e-3) and only.off == pytest.approx(0.0, abs=1e-3)
    assert only.at == pytest.approx(5.0) and only.why == ''


def test_a_step_down_is_timed_the_same_way():
    t, asked = run([0.3, 0.0])
    measured = lagged(asked, dead=0.0, lag=0.1, start=0.3)
    (down,) = sr.responses(t, asked, measured, least=0.05)
    assert down.before == pytest.approx(0.3, abs=1e-6)
    assert down.delay is not None and down.rise == pytest.approx(0.22, abs=2 * DT)
    assert down.off == pytest.approx(0.0, abs=1e-3)


def test_a_value_that_never_gets_there_is_not_measured_and_says_why():
    t, asked = run([0.3, 0.15], each=4.0)
    measured = lagged(asked, dead=0.0, lag=8.0, start=0.3)      # free wheels coasting
    (slow,) = sr.responses(t, asked, measured, least=0.02)
    assert slow.delay is not None and slow.rise is None
    assert 'did not get 90 % of the way in the 4.0 s' in slow.why
    assert slow.off > 0.05, 'the end is reported as it is, not as if it had settled'
    stuck = sr.responses(t, asked, [0.3] * len(t), least=0.02)[0]
    assert stuck.delay is None and stuck.rise is None and 'did not move 10 %' in stuck.why


def test_a_value_already_there_is_not_timed():
    t, asked = run([0.0, 0.2])
    (there,) = sr.responses(t, asked, [0.2] * len(t), least=0.02)
    assert there.delay is None and there.why == 'the measured value was already there'
    assert there.off == pytest.approx(0.0)


def test_the_way_is_from_where_the_value_was_not_from_what_was_asked_before():
    """A step asked before the last one settled is timed from where the value really was."""
    t, asked = run([0.0, 1.0, 2.0], each=0.2)
    measured = lagged(asked, dead=0.0, lag=1.0)
    second = sr.responses(t, asked, measured, least=0.02)[1]
    assert second.before == pytest.approx(measured[19]) and second.before < 0.3


def test_nothing_is_smoothed_a_spike_in_the_samples_counts():
    t, asked = run([0.0, 1.0])
    measured = [0.0] * len(t)
    measured[260] = 0.5         # one sample: the 10 % mark is crossed there
    (only,) = sr.responses(t, asked, measured, least=0.02)
    assert only.delay == pytest.approx(10 * DT) and only.rise is None


def test_the_lists_must_match_and_the_period_is_the_usual_gap():
    with pytest.raises(ValueError):
        sr.responses([0.0, 0.1], [0.0], [0.0, 0.0], least=0.1)
    assert sr.period([0.0, 0.02, 0.04, 0.5, 0.52]) == pytest.approx(0.02)
    assert sr.period([1.0]) is None


def test_the_spread_is_the_middle_the_smallest_and_the_largest_of_what_exists():
    got = sr.spread([0.3, None, 0.1, 0.2])
    assert (got.median, got.low, got.high, got.n) == (0.2, 0.1, 0.3, 3)
    assert sr.spread([None, None]) is None


def test_the_wheels_are_still_from_the_last_sample_at_speed():
    t = [i * DT for i in range(10)]
    speed = [0.2, 0.2, 0.1, 0.05, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert sr.first_below(t, speed, 1, 0.02) == pytest.approx(4 * DT)
    assert sr.first_below(t, [0.0] * 10, 1, 0.02) == 0.0
    assert sr.first_below(t, [0.2] * 10, 1, 0.02) is None, 'the log ends: not measured'
