"""Names for the things from the camera's boxes (naming.py, TODO 192)."""

from aav_detector.ground import Lens, Mount
from aav_things.naming import (best_label, Camera, iou, Namer, project, Seen, thing_box,
                               UNKNOWN)
import pytest

LENS = Lens(500.0, 500.0, 320.0, 240.0, 640, 480)
LEVEL = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
# The front left camera, as vehicle.yaml has it, looking straight ahead (x forward).
FRONT = Mount((1.75, 0.0, 1.55), LEVEL, optical=False)
# The same camera in an optical frame (z forward, x right, y down): base_link's x = its z.
OPTICAL = Mount((1.75, 0.0, 1.55), ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
                optical=True)


def test_a_point_ahead_lands_where_the_pinhole_says():
    u, v = project(LENS, FRONT, (6.0, 0.0, 1.55))     # level with the camera, dead ahead
    assert (u, v) == pytest.approx((320.0, 240.0))
    u, v = project(LENS, FRONT, (6.0, 1.0, 1.55))     # 1 m left: to the left in the picture
    assert u < 320.0
    assert project(LENS, FRONT, (1.0, 0.0, 1.0)) is None    # behind the camera


def test_an_optical_frame_projects_the_same():
    for p in [(6.0, 0.5, 0.2), (10.0, -2.0, 1.0)]:
        assert project(LENS, OPTICAL, p) == pytest.approx(project(LENS, FRONT, p))


def test_a_person_ahead_is_named_by_the_box_round_them():
    person = thing_box(LENS, FRONT, 6.0, 0.0, 0.5, 0.5, 1.8)
    assert person is not None
    u0, v0, u1, v1 = person
    assert u0 < 320.0 < u1 and v0 < 240.0 < v1
    seen = [Seen('car', 0.9, (500.0, 300.0, 640.0, 480.0)),
            Seen('person', 0.8, (u0 + 3, v0 - 5, u1 + 2, v1 + 4))]
    assert best_label(person, seen, 0.3) == 'person'
    assert best_label(person, seen[:1], 0.3) is None


def test_iou():
    assert iou((0, 0, 10, 10), (0, 0, 10, 10)) == pytest.approx(1.0)
    assert iou((0, 0, 10, 10), (5, 0, 15, 10)) == pytest.approx(1 / 3)
    assert iou((0, 0, 1, 1), (2, 2, 3, 3)) == 0.0


def test_a_name_is_held_a_while_then_let_go():
    namer = Namer(hold=3.0, max_age=0.5)
    box = thing_box(LENS, FRONT, 6.0, 0.0, 0.5, 0.5, 1.8)
    namer.cameras['front'] = Camera(LENS, FRONT, [Seen('person', 0.8, box)], stamp=10.0)
    assert namer.name(7, 6.0, 0.0, 0.5, 0.5, 1.8, 10.1) == 'person'
    # The picture goes stale (no camera sees it now): the name holds 3 s, then unknown.
    assert namer.name(7, 6.0, 0.0, 0.5, 0.5, 1.8, 12.0) == 'person'
    assert namer.name(7, 6.0, 0.0, 0.5, 0.5, 1.8, 13.5) == UNKNOWN
    # A thing off to the side never takes the person's name.
    assert namer.name(8, 6.0, 4.0, 0.5, 0.5, 1.8, 10.1) == UNKNOWN
