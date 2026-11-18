//! `aav_lidar`'s `test_scan.py` and `test_ground.py`, ported.

use std::f64::consts::PI;

use super::ground::{ground_hit, max_safe_roll, vlp16_elevations, worst_ground_height};
use super::*;

/// vehicle.yaml's lidar_link: 0.9 m ahead of base_link, 1.8 m up, level.
fn roof() -> Mount {
    Mount::new([0.9, 0.0, 1.8], [0.0; 3])
}

fn to_base(mount: &Mount, points: &[[f64; 3]]) -> Vec<Point> {
    points
        .iter()
        .map(|&[x, y, z]| mount.to_base(&Point::new(x, y, z)))
        .collect()
}

fn direction(shape: &ScanShape, angle: f64) -> usize {
    let n = shape.count() as i64;
    (((angle - shape.angle_min) / shape.angle_increment).round_ties_even() as i64).rem_euclid(n)
        as usize
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn seven_hundred_and_twenty_directions_all_the_way_round() {
    let shape = ScanShape::default();
    assert_eq!(shape.count(), 720);
    assert!(flatten(&[], &shape).iter().all(|r| r.is_infinite()));
}

#[test]
fn a_post_ahead_is_its_nearest_point_from_base_link() {
    let shape = ScanShape::default();
    let post = [[5.0, 0.0, -1.0], [5.2, 0.0, -1.5], [5.0, 0.0, -1.79]];
    let ranges = flatten(&to_base(&roof(), &post), &shape);
    let ahead = direction(&shape, 0.0);
    assert!(close(ranges[ahead], 5.9, 1e-9)); // 5 m + the lidar's 0.9 m
    assert_eq!(ranges.iter().filter(|r| r.is_finite()).count(), 1);
}

#[test]
fn the_road_and_what_is_overhead_are_not_obstacles() {
    let points = [
        [8.0, 1.0, -1.75],  // the road, 0.05 m up
        [10.0, -2.0, -1.6], // 0.2 m up
        [4.0, 0.0, 0.5],    // a branch, 2.3 m up
        [f64::INFINITY, f64::INFINITY, f64::INFINITY],
        [f64::NAN, 0.0, 0.0],
    ];
    let ranges = flatten(&to_base(&roof(), &points), &ScanShape::default());
    assert!(ranges.iter().all(|r| r.is_infinite()));
}

#[test]
fn left_is_positive_and_behind_wraps() {
    let shape = ScanShape::default();
    let points = to_base(&roof(), &[[0.0, 3.0, -1.0], [-4.0, 0.001, -1.0]]);
    let ranges = flatten(&points, &shape);
    assert!(ranges[direction(&shape, 3.0_f64.atan2(0.9))].is_finite());
    assert!(ranges[0].is_finite()); // just under +pi: straight behind, as -pi is
}

#[test]
fn each_point_goes_to_its_nearest_direction() {
    let shape = ScanShape::default();
    let angle = 0.4_f64.to_radians();
    let mount = Mount::new([0.0, 0.0, 1.8], [0.0; 3]);
    let points = to_base(&mount, &[[10.0 * angle.cos(), 10.0 * angle.sin(), -1.0]]);
    let ranges = flatten(&points, &shape);
    assert!(ranges[direction(&shape, 0.5_f64.to_radians())].is_finite());
    assert_eq!(
        direction(&shape, 0.5_f64.to_radians()),
        direction(&shape, 0.0) + 1
    );
}

#[test]
fn a_turned_lidar_turns_its_points() {
    let mount = Mount::new([0.0, 0.0, 1.0], [0.0, 0.0, PI / 2.0]);
    let p = mount.to_base(&Point::new(0.0, -5.0, 0.0));
    assert!(close(p.x, 5.0, 1e-12) && close(p.y, 0.0, 1e-12) && close(p.z, 1.0, 1e-12));
}

#[test]
fn the_car_itself_is_not_an_obstacle() {
    let shape = ScanShape {
        body: Body {
            back: -0.2,
            front: 1.8,
            half_width: 0.5,
        },
        ..ScanShape::default()
    };
    // Its roof, 1.5 m up, and a point past the bumper.
    let points = to_base(
        &roof(),
        &[[-0.5, 0.0, -0.3], [0.5, 0.4, -0.3], [1.5, 0.0, -1.0]],
    );
    let ranges = flatten(&points, &shape);
    assert_eq!(ranges.iter().filter(|r| r.is_finite()).count(), 1);
    assert!(close(ranges[direction(&shape, 0.0)], 2.4, 1e-9));
}

#[test]
fn a_part_circle_drops_what_is_outside_it() {
    // Only ahead, ±45 degrees: a point behind has no direction.
    let shape = ScanShape {
        angle_min: -PI / 4.0,
        angle_max: PI / 4.0,
        ..ScanShape::default()
    };
    let ranges = flatten(&to_base(&roof(), &[[-5.0, 0.0, -1.0]]), &shape);
    assert!(ranges.iter().all(|r| r.is_infinite()));
}

#[test]
fn a_bad_shape_is_refused() {
    let zero = ScanShape {
        angle_increment: 0.0,
        ..ScanShape::default()
    };
    assert_eq!(zero.checked(), Err(ShapeError::Increment(0.0)));
    let nan = ScanShape {
        min_height: f64::NAN,
        ..ScanShape::default()
    };
    assert_eq!(nan.checked(), Err(ShapeError::NotFinite));
    assert!(ScanShape::default().checked().is_ok());
}

#[test]
fn body_box_is_the_outline_and_its_margins() {
    let corners = [[-0.5, 0.6], [1.9, 0.6], [1.9, -0.6], [-0.5, -0.6]];
    let body = body_box(&corners);
    assert!(close(body.back, -0.8, 1e-12));
    assert!(close(body.front, 2.0, 1e-12));
    assert!(close(body.half_width, 0.9, 1e-12));
}

// --- ground.py ----------------------------------------------------------------------

const ROOF: [f64; 3] = [0.9, 0.0, 1.8];
const RIGHT: f64 = -PI / 2.0;

#[test]
fn level_the_road_is_at_base_links_height_where_the_beam_lands() {
    let hit = ground_hit(0.0, [0.0, 0.0, 1.8], (-9.0_f64).to_radians(), 0.0).unwrap();
    assert!(close(hit.height, 0.0, 1e-12));
    assert!(close(hit.range, 1.8 / 9.0_f64.to_radians().tan(), 1e-9));
}

#[test]
fn a_beam_at_or_above_level_never_lands() {
    assert!(ground_hit(0.0, ROOF, 1.0_f64.to_radians(), 0.0).is_none());
    assert!(ground_hit(0.0, ROOF, 0.0, RIGHT).is_none());
}

#[test]
fn rolled_right_the_road_on_the_right_rises_by_distance_times_tan_roll() {
    let roll = 2.0_f64.to_radians();
    let hit = ground_hit(roll, [0.0, 0.0, 1.8], (-7.0_f64).to_radians(), RIGHT).unwrap();
    assert!(close(
        hit.height,
        hit.range * roll.tan(),
        1e-9 * hit.height.abs()
    ));
    let left = ground_hit(roll, [0.0, 0.0, 1.8], (-7.0_f64).to_radians(), -RIGHT).unwrap();
    assert!(left.height < 0.0);
}

#[test]
fn a_beam_just_above_level_lands_when_rolled_past_it() {
    let hit = ground_hit(2.0_f64.to_radians(), ROOF, 1.0_f64.to_radians(), RIGHT).unwrap();
    let expected = 1.8 / 1.0_f64.to_radians().tan();
    assert!(close(hit.range, expected, 0.05 * expected));
}

#[test]
fn the_todos_numbers_two_degrees_lifts_the_road_about_0_7_m_by_20_m() {
    let worst = worst_ground_height(
        1.95_f64.to_radians(),
        ROOF,
        30.0,
        &vlp16_elevations(),
        1.0_f64.to_radians(),
    );
    assert!(close(worst.height, 0.7, 0.05));
    assert!(worst.range > 18.0 && worst.range < 24.0);
}

#[test]
fn level_nothing_on_the_road_counts() {
    let worst = worst_ground_height(0.0, ROOF, 30.0, &vlp16_elevations(), 1.0_f64.to_radians());
    assert!(close(worst.height, 0.0, 1e-9));
}

#[test]
fn rolling_either_way_is_the_same() {
    let step = 1.0_f64.to_radians();
    let e = vlp16_elevations();
    let a = worst_ground_height(1.5_f64.to_radians(), ROOF, 12.0, &e, step).height;
    let b = worst_ground_height(-1.5_f64.to_radians(), ROOF, 12.0, &e, step).height;
    assert!(close(a, b, 1e-6));
}

#[test]
fn the_cars_lidar_is_safe_to_about_1_46_degrees_at_the_costmaps_12_m() {
    let e = vlp16_elevations();
    let limit = 10.0_f64.to_radians();
    let safe = max_safe_roll(ROOF, 0.3, 12.0, &e, limit).to_degrees();
    assert!(close(safe, 1.46, 0.03));
    assert!(max_safe_roll(ROOF, 0.3, 8.0, &e, limit).to_degrees() > 2.0);
    assert!(max_safe_roll(ROOF, 0.3, 30.0, &e, limit).to_degrees() < safe);
}

#[test]
fn the_vlp16_has_16_beams_2_degrees_apart() {
    let e = vlp16_elevations();
    assert_eq!(e.len(), 16);
    assert!(close(e[0].to_degrees(), -15.0, 1e-12));
}
