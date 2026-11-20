//! The lane-centre planner (`path:=lane`) with the real lattice for its joins: on a free
//! map with an L of road, the path follows the right lane's centre between the joins and
//! starts and ends where asked; off the lane map, blocked, or too short, the lattice's.

use costmap::cost::LETHAL;
use costmap::lanes::EDGE_INSET;
use costmap::{Grid, Road};
use lattice_planner::jerk_pass::JerkSettings;
use lattice_planner::lane::{Fallback, How, LaneRouter, LaneSettings};
use lattice_planner::{scenes, Pose};

/// A free 140 m x 140 m map (0.25 m cells, origin (-20, -20)).
fn free() -> Grid {
    Grid::new((-20.0, -20.0), scenes::RESOLUTION, 560, 560, 0)
}

/// East along y = 0 to x = 100, then north along x = 100; two-way, 7 m.
fn ell() -> LaneRouter {
    let road = |name: &str, points: Vec<(f64, f64)>| Road {
        name: name.to_owned(),
        points,
        width: 7.0,
        lanes: 0,
        oneway: false,
        edge_inset: EDGE_INSET,
    };
    LaneRouter::new(
        vec![
            road("east", vec![(0.0, 0.0), (100.0, 0.0)]),
            road("north", vec![(100.0, 0.0), (100.0, 100.0)]),
        ],
        LaneSettings::default(),
    )
}

fn never() -> bool {
    false
}

#[test]
fn along_the_lane_between_the_joins() {
    let planner = scenes::planner(false);
    let grid = free();
    let start = Pose::new(5.0, 0.5, 0.0);
    let goal = Pose::new(101.5, 70.0, std::f64::consts::FRAC_PI_2);
    let got = ell()
        .plan(&planner, &grid, &start, &goal, &never)
        .expect("a path");
    let How::Lane(metres) = got.how else {
        panic!("the lattice's: {:?}", got.how);
    };
    assert!(metres > 100.0, "{metres}");
    let poses = &got.plan.poses;
    let first = poses[0];
    let last = poses[poses.len() - 1];
    assert!(
        (first.x - start.x).hypot(first.y - start.y) < 0.3,
        "{first:?}"
    );
    assert!((last.x - goal.x).hypot(last.y - goal.y) < 0.3, "{last:?}");
    // On the east road past the join: the right lane's centre, y = -1.5.
    let east: Vec<&Pose> = poses.iter().filter(|p| p.x > 20.0 && p.x < 90.0).collect();
    assert!(!east.is_empty());
    assert!(east.iter().all(|p| (p.y + 1.5).abs() < 0.05), "{east:?}");
    // No big jumps between poses: the joins meet the line.
    let gap = poses
        .windows(2)
        .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
        .fold(0.0, f64::max);
    assert!(gap < 1.0, "{gap}");
}

#[test]
fn a_car_on_its_lane_is_joined_straight() {
    // Driving the lane (0.3 m off its centre, 10 degrees off): no lattice search at all,
    // so no back-and-forth at a replan; the path starts at the car.
    let planner = scenes::planner(true);
    let grid = free();
    let start = Pose::new(30.0, -1.2, -10f64.to_radians());
    let goal = Pose::new(101.5, 70.0, std::f64::consts::FRAC_PI_2);
    let got = ell()
        .plan(&planner, &grid, &start, &goal, &never)
        .expect("a path");
    assert!(matches!(got.how, How::Lane(_)), "{:?}", got.how);
    assert_eq!(got.plan.poses[0], start);
    let gap = got
        .plan
        .poses
        .windows(2)
        .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
        .fold(0.0, f64::max);
    assert!(gap <= 0.3, "{gap}");
    // The goal on the line's end: joined straight too, so no search either end.
    assert_eq!(got.plan.poses[got.plan.poses.len() - 1], goal);
    assert_eq!(got.plan.iterations, 0);
}

#[test]
fn off_the_map_short_or_blocked_is_the_lattices() {
    let planner = scenes::planner(false);
    let mut grid = free();
    let lanes = ell();
    let how = |grid: &Grid, s: Pose, g: Pose| {
        lanes
            .plan(&planner, grid, &s, &g, &never)
            .expect("a path")
            .how
    };
    // The lot: 50 m off every road.
    assert_eq!(
        how(
            &grid,
            Pose::new(10.0, 50.0, 0.0),
            Pose::new(60.0, -1.5, 0.0)
        ),
        How::Lattice(Fallback::OffMap)
    );
    // 15 m of road, the car 2 m off its lane: no room for two 8 m joins.
    assert_eq!(
        how(&grid, Pose::new(10.0, 0.5, 0.0), Pose::new(25.0, 0.5, 0.0)),
        How::Lattice(Fallback::Short)
    );
    // On its lane already, the goal too: joined straight, so 15 m is enough; 5 m isn't.
    assert!(matches!(
        how(
            &grid,
            Pose::new(10.0, -1.5, 0.0),
            Pose::new(25.0, -1.5, 0.0)
        ),
        How::Lane(_)
    ));
    assert_eq!(
        how(
            &grid,
            Pose::new(10.0, -1.5, 0.0),
            Pose::new(15.0, -1.5, 0.0)
        ),
        How::Lattice(Fallback::Short)
    );
    // The whole road walled off at x = 50 (kerb to kerb and more): no pass, the lattice's.
    for x in 276..280 {
        for y in 60..100 {
            grid.set(x, y, LETHAL);
        }
    }
    assert_eq!(
        how(&grid, Pose::new(5.0, -1.5, 0.0), Pose::new(90.0, -1.5, 0.0)),
        How::Lattice(Fallback::Blocked)
    );
}

#[test]
fn a_box_parked_in_the_lane_is_passed_on_the_left_and_the_lane_kept_elsewhere() {
    // Ben, 16 Nov: keep right, leave the lane only to pass. A 2 m x 1 m box on the right
    // lane's centre (y -2 to -1) at x 49-51; the lane's centre y = -1.5, the centre line 0.
    let planner = scenes::planner(false);
    let mut grid = free();
    for x in 276..284 {
        for y in 72..76 {
            grid.set(x, y, LETHAL);
        }
    }
    // Inflated, as the global costmap is: the outline check reads it.
    scenes::inflation().apply(&mut grid);
    let start = Pose::new(5.0, -1.5, 0.0);
    let goal = Pose::new(90.0, -1.5, 0.0);
    let got = ell()
        .plan(&planner, &grid, &start, &goal, &never)
        .expect("a path");
    let How::Pass(_, shift) = got.how else {
        panic!("{:?}", got.how);
    };
    // The least that clears it, and no more than to the other lane's middle.
    assert!(shift > 0.9 && shift < 2.0, "{shift}");
    for p in &got.plan.poses {
        // On the lane's centre except near the box; never past the left kerb (y 3.5).
        if p.x < 30.0 || p.x > 75.0 {
            assert!((p.y + 1.5).abs() < 0.05, "{p:?}");
        }
        assert!(p.y < 3.5 - 0.5, "{p:?}");
    }
    let beside = got
        .plan
        .poses
        .iter()
        .filter(|p| (49.0..51.0).contains(&p.x))
        .map(|p| p.y)
        .fold(f64::INFINITY, f64::min);
    assert!((beside - (-1.5 + shift)).abs() < 0.05, "{beside}");
    // Every pose fits.
    assert!(got
        .plan
        .poses
        .iter()
        .all(|p| planner.fits(&grid, p).expect("cells")));
    // With passing off: the lattice, as before 16 Nov.
    let off = LaneRouter::new(
        ell().roads().to_vec(),
        LaneSettings {
            pass: None,
            ..LaneSettings::default()
        },
    );
    let got = off
        .plan(&planner, &grid, &start, &goal, &never)
        .expect("a path");
    assert_eq!(got.how, How::Lattice(Fallback::Blocked));
}

#[test]
fn a_box_parked_in_the_lane_is_passed_within_the_jerk_limit_too() {
    // TODO 214: the same box, the pass a jerk-limited QP at 2 m/s (a challenger).
    let planner = scenes::planner(false);
    let mut grid = free();
    for x in 276..284 {
        for y in 72..76 {
            grid.set(x, y, LETHAL);
        }
    }
    scenes::inflation().apply(&mut grid);
    let start = Pose::new(5.0, -1.5, 0.0);
    let goal = Pose::new(90.0, -1.5, 0.0);
    let jerky = LaneRouter::new(
        ell().roads().to_vec(),
        LaneSettings {
            pass_jerk: Some((JerkSettings::default(), 2.0)),
            ..LaneSettings::default()
        },
    );
    let got = jerky
        .plan(&planner, &grid, &start, &goal, &never)
        .expect("a path");
    let How::Pass(_, shift) = got.how else {
        panic!("{:?}", got.how);
    };
    assert!(shift > 0.9 && shift < 2.0, "{shift}");
    assert!(got
        .plan
        .poses
        .iter()
        .all(|p| planner.fits(&grid, p).expect("cells")));
    for p in &got.plan.poses {
        if p.x < 25.0 || p.x > 80.0 {
            assert!((p.y + 1.5).abs() < 0.05, "{p:?}");
        }
    }
}

#[test]
fn a_goal_off_the_costmap_is_planned_to_where_the_lane_leaves_it() {
    // TODO 175: a costmap of only the chunks near the car, 70 m tall from (-20, -20); the
    // goal up the north road at y = 70, off it. The path keeps to the lane as far as 10 m
    // inside the edge (steps::EDGE_MARGIN), and says it was cut; the route is whole.
    let planner = scenes::planner(false);
    let near = Grid::new((-20.0, -20.0), scenes::RESOLUTION, 560, 280, 0);
    let start = Pose::new(5.0, 0.5, 0.0);
    let goal = Pose::new(101.5, 70.0, std::f64::consts::FRAC_PI_2);
    let got = ell()
        .plan(&planner, &near, &start, &goal, &never)
        .expect("a path");
    assert!(got.cut);
    assert!(matches!(got.how, How::Lane(_)), "{:?}", got.how);
    let last = got.plan.poses[got.plan.poses.len() - 1];
    assert!((last.x - 101.5).abs() < 0.1, "{last:?}");
    assert!(last.y > 35.0 && last.y <= 40.0, "{last:?}");
    let end = got.route[got.route.len() - 1];
    assert!(end.y > 69.0, "the route reaches the goal: {end:?}");
    // On the whole map: not cut.
    let whole = ell()
        .plan(&planner, &free(), &start, &goal, &never)
        .expect("a path");
    assert!(!whole.cut);
}

#[test]
fn off_the_lane_map_a_far_goal_is_a_step_along_the_straight_line() {
    let planner = scenes::planner(false);
    let near = Grid::new((-20.0, -20.0), scenes::RESOLUTION, 280, 280, 0);
    let start = Pose::new(0.0, 30.0, 0.0);
    let goal = Pose::new(300.0, 30.0, 0.0);
    let got = ell()
        .plan(&planner, &near, &start, &goal, &never)
        .expect("a path");
    assert!(got.cut);
    assert_eq!(got.how, How::Lattice(Fallback::OffMap));
    let last = got.plan.poses[got.plan.poses.len() - 1];
    assert!(last.x > 35.0 && last.x <= 40.0, "{last:?}");
    assert_eq!(got.route.len(), 2);
}

#[test]
fn a_loop_round_the_block_to_a_goal_close_by_is_the_lattices() {
    // A one-way U: east along y = 0, north at x = 100, west along y = 30. The goal 33 m
    // north of the car, on the west road: by the lanes 200 m round, so the lattice's.
    let one_way = |name: &str, points: Vec<(f64, f64)>| Road {
        name: name.to_owned(),
        points,
        width: 7.0,
        lanes: 0,
        oneway: true,
        edge_inset: EDGE_INSET,
    };
    let u = LaneRouter::new(
        vec![
            one_way("east", vec![(0.0, 0.0), (100.0, 0.0)]),
            one_way("north", vec![(100.0, 0.0), (100.0, 30.0)]),
            one_way("west", vec![(100.0, 30.0), (0.0, 30.0)]),
        ],
        LaneSettings::default(),
    );
    let planner = scenes::planner(true);
    let start = Pose::new(10.0, 0.0, 0.0);
    let goal = Pose::new(20.0, 30.0, std::f64::consts::PI);
    let got = u
        .plan(&planner, &free(), &start, &goal, &never)
        .expect("a path");
    assert_eq!(got.how, How::Lattice(Fallback::Detour));
    // Further along the same road, the lane is taken: 160 m by the lanes, 120 m straight.
    let far = Pose::new(10.0, 30.0, std::f64::consts::PI);
    let from = Pose::new(90.0, 0.0, 0.0);
    let got = u
        .plan(&planner, &free(), &from, &far, &never)
        .expect("a path");
    assert!(matches!(got.how, How::Lane(_)), "{:?}", got.how);
}
