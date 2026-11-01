"""measure_map: the route covers vehicle.yaml, and chained readings add up to its numbers."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import measure_map as mm  # noqa: E402
import pytest  # noqa: E402

CFG = mm.load()


def leaves(tree, prefix=''):
    """Every dotted key with a plain value or a list under it."""
    for key, value in tree.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            yield from leaves(value, name + '.')
        else:
            yield name


def covered(key):
    gives = {s.gives for s in mm.ROUTE}
    if key in gives:
        return True
    parts = key.split('.')
    if parts[0] == 'sensors' and parts[-1] == 'rpy':
        return 'sensors.*.rpy' in mm.NOT_ON_THE_ROUTE
    return any('.'.join(parts[:i]) in mm.NOT_ON_THE_ROUTE for i in range(1, len(parts) + 1))


def test_every_yaml_number_has_a_place():
    missing = [k for k in leaves(CFG) if not covered(k)]
    assert missing == []


def test_every_sensor_and_body_key_is_on_the_route():
    gives = {s.gives for s in mm.ROUTE}
    for frame in CFG['sensors']:
        assert f'sensors.{frame}.xyz' in gives
    for key in CFG['vehicle']['body']:
        assert f'vehicle.body.{key}' in gives
    for key in ('track', 'wheelbase', 'wheel_diameter', 'wheel_width'):
        assert f'vehicle.{key}' in gives


def test_every_sensor_is_drawn():
    assert sorted(mm.SENSORS.values()) == sorted(CFG['sensors'])


def test_steps_are_numbered_in_order_and_start_at_a_reached_mark():
    assert [s.number for s in mm.ROUTE] == list(range(1, len(mm.ROUTE) + 1))
    reached = {'X'}
    for step in mm.ROUTE:
        assert step.start in reached, step
        assert step.start in mm.MARKS and step.end in mm.MARKS
        reached.add(step.end)
    assert set(mm.SHEET_ROW) == {s.number for s in mm.ROUTE}


def test_a_step_moves_only_along_the_axes_it_measures():
    pos = mm.positions(CFG)
    for step in mm.ROUTE:
        if not step.axes:
            continue
        d = mm.offset(pos, step)
        for axis, value in zip(mm.AXES, d):
            if axis not in step.axes:
                assert value == pytest.approx(0.0), (step.number, axis)


def test_chained_steps_add_up_to_the_yaml():
    values = mm.to_yaml_values(mm.readings_today(CFG))
    for key, value in values.items():
        want = mm.get(CFG, key.replace('check: ', '').replace(' (front)', ''))
        assert value == pytest.approx(want, abs=1e-6), key


def test_feet_inches_and_metres():
    assert mm.metres('5 ft 2 in') == pytest.approx(62 * 0.0254)
    assert mm.metres("5' 2\"") == pytest.approx(62 * 0.0254)
    assert mm.metres('62 in') == pytest.approx(1.5748)
    assert mm.metres('157 cm') == pytest.approx(1.57)
    assert mm.metres('1570mm') == pytest.approx(1.57)
    assert mm.metres(1.57) == pytest.approx(1.57)
    assert mm.metres('-3 in') == pytest.approx(-0.0762)
    assert mm.metres(2, default='ft') == pytest.approx(0.6096)
    assert mm.metres('2', default='in') == pytest.approx(0.0508)
    for bad in ('', 'two feet', '5 ft x', 'ft'):
        with pytest.raises(ValueError):
            mm.metres(bad)


def test_back_right_down_count_as_minus():
    got = mm.parse_readings({'units': 'cm', 8: {'back': 1, 'right': 2, 'up': 90},
                             7: '-20 cm'})
    assert got[8] == {'forward': pytest.approx(-0.01), 'left': pytest.approx(-0.02),
                      'up': pytest.approx(0.90)}
    assert got[7] == {'forward': pytest.approx(-0.20)}
    with pytest.raises(ValueError):
        mm.parse_readings({1: {'up': 1}})          # step 1 measures left only


def test_the_doc_s_worked_example():
    # docs/measure-the-car.md: step 1 '1 ft 3 in', step 12 1.50 m, step 14 forward
    # '2 ft 11 in' and up '1 ft'.
    readings = mm.parse_readings({1: '1 ft 3 in', 12: 1.50,
                                  14: {'forward': '2 ft 11 in', 'left': 0, 'up': '1 ft'}})
    values = mm.to_yaml_values(readings)
    assert values['vehicle.track'] == pytest.approx(0.762)
    assert values['sensors.lidar_link.xyz'] == pytest.approx([0.889, 0.0, 1.805])


def test_relative_to_earlier_marks():
    # The right camera is measured from the left one, so it carries the left one's spot.
    readings = mm.parse_readings({12: 1.4, 15: {'forward': 1.7, 'left': 0.3, 'up': 0.1},
                                  16: {'forward': 0, 'right': 0.6, 'up': 0}})
    values = mm.to_yaml_values(readings)
    assert values['sensors.camera_left_link.xyz'] == pytest.approx([1.7, 0.3, 1.5])
    assert values['sensors.camera_right_link.xyz'] == pytest.approx([1.7, -0.3, 1.5])


def test_guesses_come_from_the_comments(tmp_path):
    flagged = mm.guesses()
    assert mm.is_guess('vehicle.track', flagged)
    assert mm.is_guess('sensors.camera_right_link.xyz', flagged)
    assert not mm.is_guess('vehicle.max_reverse_speed', flagged)
    yaml_file = tmp_path / 'v.yaml'
    yaml_file.write_text('vehicle:\n  track: 0.8   # measured 30 Oct\n'
                         '  wheelbase: 1.6   # TODO\n', encoding='utf-8')
    flagged = mm.guesses(yaml_file)
    assert flagged == {'vehicle.wheelbase'}


def test_the_doc_holds_today_s_table():
    text = mm.DOC.read_text(encoding='utf-8')
    inside = text.split(mm.START, 1)[1].split(mm.END, 1)[0]
    assert inside.strip() == mm.sheet().strip(), 'run: pixi run measure-map'


def test_readings_file_round_trip(tmp_path, capsys):
    blank = tmp_path / 'readings.yaml'
    blank.write_text(mm.blank().replace('\n1:\n', '\n1: 1 ft 3 in\n'), encoding='utf-8')
    assert mm.main(['--readings', str(blank)]) == 0
    out = capsys.readouterr().out
    assert 'vehicle.track: 0.762' in out
    assert 'Not measured yet' in out


def test_the_pictures(tmp_path):
    pytest.importorskip('matplotlib')
    paths = mm.draw(CFG, mm.guesses(), tmp_path)
    assert sorted(p.name for p in paths) == [
        'measure-map-rear.png', 'measure-map-side.png', 'measure-map-top.png']
    assert all(p.stat().st_size > 10_000 for p in paths)
