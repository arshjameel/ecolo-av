"""A thing's words and colour on a screen (looks.py)."""

from aav_things import looks


def test_an_unknown_thing_shows_only_its_id():
    assert looks.caption(4, 'unknown', 1.0, 0.0, True) == '4 1.0 m/s R'
    assert looks.caption(4, '', 0.0, 0.0, False) == '4 0.0 m/s'


def test_a_named_thing_shows_its_name_first():
    assert looks.caption(3, 'person', 0.6, 0.8, False) == 'person 3 1.0 m/s'
    assert looks.caption(7, 'stop sign', 0.0, 0.0, False) == 'stop sign 7 0.0 m/s'


def test_each_kind_has_its_colour():
    assert looks.colour('person') == looks.PERSON
    assert looks.colour('Stop Sign') == looks.STOP_SIGN
    for vehicle in looks.VEHICLES:
        assert looks.colour(vehicle) == looks.VEHICLE
    assert looks.colour('bicycle') == looks.OTHER
    assert looks.colour('unknown') == looks.UNNAMED
    assert looks.colour('') == looks.UNNAMED


def test_coasting_is_grey_whatever_its_name():
    assert looks.colour('person', coasting=True) == looks.COASTING


def test_the_colours_differ():
    colours = {looks.PERSON, looks.VEHICLE, looks.STOP_SIGN, looks.OTHER, looks.UNNAMED,
               looks.COASTING}
    assert len(colours) == 6
