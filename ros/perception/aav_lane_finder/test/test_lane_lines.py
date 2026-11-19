import math

from aav_lane_finder.birds_eye import BirdsEye, Grid, Lens, Mount
from aav_lane_finder.lane_lines import (draw, lane_from, LaneFinder, Line, MAX_RADIUS,
                                        paint_mask, paint_of, PAINT_UNKNOWN, PAINT_WHITE,
                                        PAINT_YELLOW, Settings, STRAIGHT)
import numpy as np
from scenes import BENCH_CAMERA, picture

SETTINGS = Settings()
LOOKAHEAD = SETTINGS.lookahead


def finder(camera=BENCH_CAMERA, pitch=0.0):
    view = BirdsEye(Lens.pinhole(640, 360, 343.0), Mount(camera, (0.0, pitch, 0.0)), Grid())
    return LaneFinder(view, SETTINGS)


def find(**scene):
    return finder().find(picture(**scene))


# --- the six pictures, with known answers ---

def test_a_straight_lane_with_the_car_in_the_middle():
    lane = find()
    assert lane.left and lane.right
    assert lane.left_confidence > 0.8 and lane.right_confidence > 0.8
    assert abs(lane.offset) < 0.03
    assert abs(lane.width - 3.5) < 0.05
    assert abs(lane.left.c - 1.75) < 0.05 and abs(lane.right.c + 1.75) < 0.05
    assert lane.mean_radius > STRAIGHT and lane.curve_direction == 0.0
    assert abs(lane.steering_hint) < 0.01


def test_each_line_says_its_paint():
    lane = find(yellow=('left',))
    assert (lane.left_paint, lane.right_paint) == (PAINT_YELLOW, PAINT_WHITE)
    lane = find(lines=('right',))
    assert (lane.left_paint, lane.right_paint) == (PAINT_UNKNOWN, PAINT_WHITE)
    # Warm light on white paint is still white; a grey picture's lines are white.
    assert paint_of(np.array([[200, 230, 240]] * 4), SETTINGS) == PAINT_WHITE
    assert paint_of(np.empty((0, 3)), SETTINGS) == PAINT_UNKNOWN


def test_an_offset_car_left_of_the_centre_is_plus():
    lane = find(offset=0.5)
    assert abs(lane.offset - 0.5) < 0.03
    assert lane.steering_hint < 0.0, 'steer right, back to the centre'
    lane = find(offset=-0.4)
    assert abs(lane.offset + 0.4) < 0.03
    assert lane.steering_hint > 0.0


def test_a_heading_error_shows_in_the_offset_ahead_and_the_slope():
    heading = 0.05   # rad, turned left: the car's nose points left of the lane
    lane = find(heading=heading)
    # Centred at base_link, so the car's axis is L tan(heading) left of the centre at L.
    assert abs(lane.offset - LOOKAHEAD * math.tan(heading)) < 0.03
    assert abs(lane.left.b + math.tan(heading)) < 0.01
    assert abs(lane.right.b + math.tan(heading)) < 0.01


def test_a_curve_gives_its_radius_and_direction():
    for radius, direction in ((60.0, 1.0), (-60.0, -1.0)):
        lane = find(radius=radius)
        assert lane.left and lane.right
        assert abs(lane.mean_radius - abs(radius)) < 0.2 * abs(radius), lane.mean_radius
        assert lane.curve_direction == direction
        # The lane's centre bends away from the car's axis: x^2 / 2R at the look-ahead.
        bend = abs(radius) - math.sqrt(radius ** 2 - LOOKAHEAD ** 2)
        assert abs(lane.offset + direction * bend) < 0.03
    # The inside line is the tighter one.
    lane = find(radius=60.0)
    assert lane.left_radius < lane.right_radius


def test_one_line_missing_puts_the_centre_half_a_lane_beside_the_other():
    lane = find(offset=0.3, lines=('right',))
    assert lane.left is None and lane.left_confidence == 0.0
    assert lane.right and lane.right_confidence > 0.8
    assert abs(lane.offset - 0.3) < 0.03
    assert abs(lane.width - SETTINGS.lane_width) < 1e-9
    lane = find(offset=-0.2, lines=('left',))
    assert lane.right is None and lane.left
    assert abs(lane.offset + 0.2) < 0.03


def test_no_lines_no_lane():
    lane = find(lines=())
    assert lane.left is None and lane.right is None
    assert lane.left_confidence == 0.0 and lane.right_confidence == 0.0
    assert lane.offset == 0.0 and lane.steering_hint == 0.0
    assert lane.mean_radius == MAX_RADIUS and lane.curve_direction == 0.0


# --- more of the same, and the parts ---

def test_the_lane_width_it_saw_is_used_when_a_line_goes():
    f = finder()
    for _ in range(20):
        f.find(picture(lane_width=3.0))
    assert abs(f.width - 3.0) < 0.05
    lane = f.find(picture(offset=0.2, lane_width=3.0, lines=('right',)))
    assert abs(lane.offset - 0.2) < 0.04
    assert abs(lane.width - 3.0) < 0.05


def test_a_camera_tilted_down_and_higher_up_gives_the_same_answer():
    camera = (1.5, 0.0, 1.6)
    f = finder(camera, pitch=0.12)
    lane = f.find(picture(offset=0.3, camera=camera, pitch=0.12))
    assert abs(lane.offset - 0.3) < 0.03
    assert abs(lane.width - 3.5) < 0.05


def test_paint_must_be_brighter_than_both_sides():
    row = np.array([[95] * 10 + [235] * 3 + [95] * 10 + [130] * 10], np.uint8)
    mask = paint_mask(row, 0.05, SETTINGS)[0]
    assert mask[10:13].all(), 'the paint'
    assert mask.sum() == 3, 'not the step from tarmac to grass'


def test_lane_from_lines_square_to_the_car():
    left, right = Line(0.0, 0.0, 1.25, 1.0), Line(0.0, 0.0, -2.25, 1.0)
    lane = lane_from(left, right, 3.5, SETTINGS, 1.0, 1.0)
    assert abs(lane.offset - 0.5) < 1e-9 and abs(lane.width - 3.5) < 1e-9
    assert lane.left_radius == lane.right_radius == MAX_RADIUS


def test_a_lane_seen_at_an_angle_is_measured_square_to_it():
    slope = math.tan(0.3)
    left = Line(0.0, slope, 1.75 / math.cos(0.3), 1.0)
    right = Line(0.0, slope, -1.75 / math.cos(0.3), 1.0)
    assert abs(lane_from(left, right, 3.5, SETTINGS).width - 3.5) < 1e-9
    one = lane_from(left, None, 3.5, SETTINGS)
    both = lane_from(left, right, 3.5, SETTINGS)
    assert abs(one.offset - both.offset) < 1e-9


def test_a_parabola_has_the_radius_of_its_bend():
    line = Line(1.0 / 100.0, 0.0, 0.0, 1.0)   # y = x^2 / 100: radius 50 m at x = 0
    assert abs(1.0 / line.curvature(0.0) - 50.0) < 1e-9


def test_the_debug_picture_is_drawn():
    f = finder()
    lane = f.find(picture(offset=0.2))
    image = draw(f.last, lane, f.view.grid)
    assert image.shape == (2 * f.view.grid.rows, 2 * f.view.grid.cols, 3)
    assert len(f.last.windows) == 2 * SETTINGS.windows
