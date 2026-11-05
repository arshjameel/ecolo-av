"""The steering actuator's limits from its marked ends and centre (steering_limits)."""

import json
import math
from pathlib import Path

from aav_legacy.steering_limits import compute, main, MARGIN, MeasurementError
import pytest


def test_symmetric_limits_from_the_smaller_side_less_5_percent():
    # Today's guess turned real: left raises the target, the right side is shorter.
    r = compute(left=3500, centre=1950, right=500)
    assert (r.half, r.lo, r.hi, r.sign) == (1377, 573, 3327, 1)   # floor(1450 x 0.95)
    assert r.counts_per_rad is None and r.max_steering_angle is None
    assert r.warnings == []


def test_with_the_wheel_angles_it_gives_the_gain_and_the_steering_limit():
    r = compute(left=3500, centre=2000, right=500, left_deg=30.0, right_deg=30.0)
    gain = 1500 / math.radians(30)
    assert r.counts_per_rad == pytest.approx(gain)
    assert r.max_steering_angle == pytest.approx(1425 / gain)
    assert math.degrees(r.max_steering_angle) == pytest.approx(28.5)


def test_steering_left_lowering_the_target_gives_a_negative_gain():
    r = compute(left=600, centre=2000, right=3400, left_deg=28, right_deg=28)
    assert r.sign == -1 and r.counts_per_rad < 0
    assert (r.lo, r.hi) == (2000 - 1330, 2000 + 1330)


def test_flags_a_centre_off_the_middle_and_a_lopsided_travel():
    r = compute(left=3500, centre=2600, right=400)
    assert any('from the middle' in w for w in r.warnings)
    assert any('lopsided' in w for w in r.warnings)


def test_flags_clipped_feedback_and_gains_that_differ():
    r = compute(left=4095, centre=2048, right=0, left_deg=30, right_deg=20)
    assert sum('clipped' in w for w in r.warnings) == 2
    assert any('gains of the two sides differ' in w for w in r.warnings)


@pytest.mark.parametrize('left,centre,right,why', [
    (2000, 2000, 1950, "doesn't change"),       # 50 counts end to end
    (3500, 3600, 500, 'not between'),
    (3500, 500, 500, 'not between'),            # the centre is an end
    (5000, 2000, 500, 'counts are 0 to 4095'),
])
def test_measurements_that_cannot_give_limits(left, centre, right, why):
    with pytest.raises(MeasurementError, match=why):
        compute(left, centre, right)


def test_angles_must_be_positive():
    with pytest.raises(MeasurementError):
        compute(3500, 2000, 500, left_deg=-30, right_deg=30)


def test_the_command_line(capsys):
    assert main(['--left', '3500', '--centre', '1950', '--right', '500',
                 '--left-deg', '31', '--right-deg', '29']) == 0
    out = capsys.readouterr().out
    assert 'limits 573 to 3327' in out and 'min: 573' in out and 'max_steering_angle:' in out
    assert "hardware lead's" in out
    assert main(['--left', '2000', '--centre', '2000', '--right', '2010']) == 1
    assert 'ERROR' in capsys.readouterr().out


# --- The vectors shared with the hardware panel's Rust twin (panel-core steering_limits.rs) ---

VECTORS = Path(__file__).resolve().parent / 'steering_limits_vectors.json'
WARNING_KINDS = (('from the middle', 'centre_off'), ('lopsided', 'lopsided'),
                 ('left end reads', 'clipped_left'), ('right end reads', 'clipped_right'),
                 ('gains of the two sides', 'gains_differ'))
ERROR_KINDS = (('counts are 0 to', 'out_of_range'), ("doesn't change", 'no_span'),
               ('not between', 'centre_not_between'), ('positive degrees', 'angles'))


def kind(text, kinds):
    return next(k for key, k in kinds if key in text)


def test_the_vectors_shared_with_the_panel():
    doc = json.loads(VECTORS.read_text())
    assert doc['margin'] == MARGIN
    for case in doc['cases']:
        expect = case['expect']
        args = (case['left'], case['centre'], case['right'], case['left_deg'],
                case['right_deg'])
        if 'error' in expect:
            with pytest.raises(MeasurementError) as e:
                compute(*args)
            assert kind(str(e.value), ERROR_KINDS) == expect['error'], case['name']
            continue
        r = compute(*args)
        assert (r.lo, r.hi, r.half, r.sign) == (
            expect['lo'], expect['hi'], expect['half'], expect['sign']), case['name']
        for name in ('counts_per_rad', 'max_steering_angle'):
            want = expect[name]
            assert (getattr(r, name) is None) == (want is None), case['name']
            if want is not None:
                assert getattr(r, name) == pytest.approx(want), case['name']
        assert [kind(w, WARNING_KINDS) for w in r.warnings] == expect['warnings'], case['name']
