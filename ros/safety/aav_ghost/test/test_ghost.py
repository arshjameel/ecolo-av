import math

from aav_ghost.ghost import ahead_of, command, Ghost, Pose, ranges, ScanShape, to_base
import numpy
import pytest

SHAPE = ScanShape()     # 720 directions, as lidar_scan's
FRONT = 2.2             # m, base_link to the front bumper


def direction(angle):
    return int(round((angle - SHAPE.angle_min) / SHAPE.angle_increment))


def test_ahead_is_from_the_bumper_to_the_near_edge_on_the_cars_heading():
    x, y = ahead_of(Pose(1.0, 2.0, 0.0), FRONT, 10.0, 0.5)
    assert (x, y) == pytest.approx((1.0 + 2.2 + 10.0 + 0.25, 2.0))
    x, y = ahead_of(Pose(0.0, 0.0, math.pi / 2), FRONT, 5.0, 0.5)    # facing north (left)
    assert (x, y) == pytest.approx((0.0, 7.45))


def test_to_base_is_forward_and_left():
    pose = Pose(10.0, 5.0, math.pi / 2)              # facing +y
    assert to_base(10.0, 8.0, pose) == pytest.approx((3.0, 0.0))    # 3 m ahead
    assert to_base(9.0, 5.0, pose) == pytest.approx((0.0, 1.0))     # 1 m to the left


def test_the_scan_sees_the_ghosts_near_side_only_where_it_is():
    ghost = Ghost(10.0, 0.0, width=0.5)
    r = ranges(ghost, Pose(0.0, 0.0, 0.0))
    assert r[direction(0.0)] == pytest.approx(9.75)          # its near edge
    hit = numpy.isfinite(r)
    # 0.5 m wide at 10 m: about 2.9 degrees, so 5 or 6 beams of 0.5 degrees
    assert 5 <= hit.sum() <= 7
    assert all(abs(SHAPE.angle_min + i * SHAPE.angle_increment) < math.radians(2)
               for i in numpy.flatnonzero(hit))


def test_close_up_enough_beams_for_the_stop_zone():
    # the stop zone needs 3 points: a person 1 m past the bumper gives many.
    r = ranges(Ghost(FRONT + 1.25, 0.0), Pose(0.0, 0.0, 0.0))
    assert numpy.isfinite(r).sum() >= 10


def test_the_ghost_stays_put_as_the_car_drives_and_turns():
    ghost = Ghost(10.0, 0.0)
    r = ranges(ghost, Pose(5.0, 0.0, 0.0))
    assert r[direction(0.0)] == pytest.approx(4.75)
    r = ranges(ghost, Pose(10.0, -5.0, 0.0))             # it's now 5 m to the left
    assert r[direction(math.pi / 2)] == pytest.approx(4.75)
    assert numpy.isinf(r[direction(0.0)])


def test_behind_the_car_and_far_away():
    r = ranges(Ghost(-6.0, 0.0), Pose(0.0, 0.0, 0.0))
    assert numpy.isfinite(r[0]) or numpy.isfinite(r[-1])     # straight behind wraps
    assert numpy.isinf(ranges(Ghost(40.0, 0.0), Pose(0.0, 0.0, 0.0))).all()


def test_inside_the_ghost_every_beam_is_blocked():
    r = ranges(Ghost(0.1, 0.0, width=1.0), Pose(0.0, 0.0, 0.0))
    assert (r == SHAPE.range_min).all()


@pytest.mark.parametrize('words, values', [
    (['ahead', '10'], {'ahead': 10.0, 'enabled': True}),
    (['at', '20', '-1.5'], {'ahead': -1.0, 'x': 20.0, 'y': -1.5, 'enabled': True}),
    (['size', '0.3', '1.2'], {'width': 0.3, 'height': 1.2}),
    (['off'], {'enabled': False}),
    (['on'], {'enabled': True}),
])
def test_the_command_line(words, values):
    assert command(words) == values


@pytest.mark.parametrize('words', [[], ['ahead'], ['ahead', '-3'], ['ahead', 'ten'],
                                   ['at', '1'], ['size', '0', '1'], ['off', '1'], ['boo']])
def test_a_bad_command_line_gives_the_usage(words):
    with pytest.raises(ValueError, match='ghost ahead'):
        command(words)
