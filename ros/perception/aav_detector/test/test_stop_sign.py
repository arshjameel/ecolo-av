from aav_detector.stop_sign import (biggest_sign, CLEAR, GAZEBO_FOCAL_PX, lidar_sign,
                                    PASSING, sign_distance_m, sign_point,
                                    STOPPING, StopRule, StopSignWatcher)
import pytest

# 3.5 m: 73.5 px with Gazebo's lens.
RULE = StopRule(stop_distance_m=3.5, min_score=0.5, hold_s=4.0, ignore_s=10.0)
NEAR = [('stop sign', 0.9, 100.0)]
FAR = [('stop sign', 0.9, 30.0)]


def test_a_far_sign_does_nothing_a_near_one_locks():
    w = StopSignWatcher(RULE)
    assert not w.update(FAR, 0.0)
    assert w.state == CLEAR
    assert w.update(NEAR, 1.0)
    assert w.state == STOPPING


def test_it_holds_then_lets_go_and_ignores_the_same_sign():
    w = StopSignWatcher(RULE)
    w.update(NEAR, 0.0)
    assert w.update(NEAR, 3.9)            # still holding
    assert not w.update(NEAR, 4.0)        # let go...
    assert w.state == PASSING
    assert not w.update(NEAR, 13.9)       # ...and drive past it
    assert not w.update([], 14.0)
    assert w.state == CLEAR
    assert w.update(NEAR, 15.0), 'the next sign counts again'


def test_timers_run_without_new_boxes():
    w = StopSignWatcher(RULE)
    w.update(NEAR, 0.0)
    assert not w.tick(5.0)
    assert w.state == PASSING


def test_low_scores_and_other_labels_never_lock():
    w = StopSignWatcher(RULE)
    assert not w.update([('stop sign', 0.3, 200.0), ('person', 0.99, 300.0)], 0.0)


def test_the_2024_25_models_label_counts_too():
    assert StopSignWatcher(RULE).update([('Stop_Sign', 0.9, 100.0)], 0.0)


def test_the_default_is_55_px_with_gazebos_lens():
    rule = StopRule()
    assert rule.focal_px == GAZEBO_FOCAL_PX == 343.0
    assert rule.min_height_px() == pytest.approx(55.0, abs=0.5)     # the rule until 24 Oct
    assert sign_distance_m(55.0, 343.0) == pytest.approx(4.68, abs=0.01)
    assert sign_distance_m(0.0, 343.0) == float('inf')


def box_at(distance_m, focal_px):
    """Return a 0.75 m stop sign's box, distance_m ahead of a camera with this lens."""
    return [('stop sign', 0.9, focal_px * 0.75 / distance_m)]


@pytest.mark.parametrize('focal_px', [343.0, 950.0, 1400.0])     # Gazebo; 1280x720 lenses
def test_the_distance_carries_over_to_any_lens(focal_px):
    for distance, stops in ((8.0, False), (4.9, False), (4.6, True), (2.0, True)):
        w = StopSignWatcher(StopRule())
        assert w.update(box_at(distance, focal_px), 0.0, focal_px) == stops, distance


def test_a_bigger_lens_needs_a_bigger_box():
    # 100 px is 2.6 m with Gazebo's lens, but 7.1 m with a 950 px one: too far yet.
    assert StopSignWatcher(StopRule()).update([('stop sign', 0.9, 100.0)], 0.0)
    assert not StopSignWatcher(StopRule()).update([('stop sign', 0.9, 100.0)], 0.0, 950.0)
    assert StopRule().min_height_px(950.0) == pytest.approx(151.6, abs=0.1)
    # No lens given: the rule's own (Gazebo's, or focal_px from the parameters).
    assert StopSignWatcher(StopRule(focal_px=950.0)).update(box_at(4.0, 950.0), 0.0)


def test_the_signs_point_in_its_cameras_frame():
    # 55 px tall with Gazebo's lens: 4.68 m along the axis; its centre 34.3 px right of cx.
    ahead = sign_distance_m(55.0, GAZEBO_FOCAL_PX)
    x, y, z = sign_point(354.3, 55.0, GAZEBO_FOCAL_PX, fx=343.0, cx=320.0)
    assert (x, z) == (pytest.approx(ahead), 0.0) and y == pytest.approx(-0.1 * ahead)
    ox, oy, oz = sign_point(354.3, 55.0, GAZEBO_FOCAL_PX, fx=343.0, cx=320.0, optical=True)
    assert (ox, oy, oz) == (pytest.approx(0.1 * ahead), 0.0, pytest.approx(ahead))
    # No lens yet: straight ahead.
    assert sign_point(500.0, 55.0, GAZEBO_FOCAL_PX)[1] == 0.0


def test_the_biggest_sign_box():
    boxes = [('stop sign', 0.9, 30.0, 100.0), ('stop sign', 0.9, 60.0, 300.0),
             ('person', 0.9, 200.0, 50.0), ('stop sign', 0.2, 90.0, 10.0)]
    assert biggest_sign(boxes, 0.5) == (300.0, 60.0)
    assert biggest_sign([], 0.5) is None


def test_the_lidar_gives_the_named_signs_distance():
    """TODO 192 step 2: the stop sign the lidar sees beats the pixel height's guess."""
    things = [('person', 6.1, 0.2), ('stop_sign', 7.9, 1.6), ('stop sign', 20.0, 1.5)]
    assert lidar_sign((7.0, 1.4), things) == (7.9, 1.6)
    assert lidar_sign((7.0, 1.4), things[:1]) is None          # no sign named: the camera's
    assert lidar_sign((12.0, 1.4), things, gate_m=3.0) is None  # too far from the guess
