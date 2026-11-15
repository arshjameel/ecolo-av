//! Hybrid-A* (`Planner::hybrid`): its moves (Nav2's `HybridMotionTable`), its heading
//! bins, the obstacle estimate it downsamples, and its plans on the example lot's tour,
//! against our lattice on the same legs.

use std::f64::consts::{PI, SQRT_2, TAU};

use costmap::cost::LETHAL;
use costmap::Grid;
use lattice_planner::heuristic::Obstacles;
use lattice_planner::{
    cusps, scenes, tightest_turn, Hybrid, Lattice, PlanError, Planner, Pose, Settings,
};

/// The car's planning radius (vehicle.plan_turning_radius), m; the maps' cells, m.
const RADIUS: f64 = 3.572;
const CELL: f64 = 0.25;

#[test]
fn the_moves_are_three_arcs_from_each_heading() {
    let lattice = Lattice::hybrid(RADIUS, 72, CELL, false).unwrap();
    assert_eq!(lattice.headings.len(), 72);
    assert!((lattice.headings[1] - TAU / 72.0).abs() < 1e-12);
    assert!((lattice.turning_radius - RADIUS).abs() < 1e-12);
    let r = RADIUS / CELL;
    // The least turn whose chord leaves the cell is 5.6 degrees: two 5 degree bins.
    let angle = 2.0 * TAU / 72.0;
    for (h, moves) in lattice.moves.iter().enumerate() {
        assert_eq!(moves.len(), 3, "heading {h}");
        let theta = lattice.headings[h];
        let (straight, left, right) = (&moves[0], &moves[1], &moves[2]);
        assert_eq!(straight.end_heading, h);
        assert_eq!(left.end_heading, (h + 2) % 72);
        assert_eq!(right.end_heading, (h + 70) % 72);
        assert!(left.left_turn && !right.left_turn && !straight.left_turn);
        assert_eq!(straight.arc_length, 0.0);
        // Each is one pose, its end; every chord leaves the cell (over a diagonal).
        for m in moves {
            assert_eq!(m.poses.len(), 1);
            let (x, y) = m.end();
            let chord = x.hypot(y) / CELL;
            assert!(chord >= SQRT_2, "chord {chord}");
            assert!((chord - r * (2.0 * (angle / 2.0).sin())).abs() < 1e-9);
        }
        // The straight move goes where the car faces.
        let (x, y) = straight.end();
        assert!((y.atan2(x) - theta).sin().abs() < 1e-9 && x * theta.cos() + y * theta.sin() > 0.0);
        // The left arc ends on the circle round the car's left turning centre, and is
        // R x angle long.
        let centre = (-theta.sin() * RADIUS, theta.cos() * RADIUS);
        let (x, y) = left.end();
        assert!(((x - centre.0).hypot(y - centre.1) - RADIUS).abs() < 1e-9);
        assert!((left.length - RADIUS * angle).abs() < 1e-9);
        let centre = (theta.sin() * RADIUS, -theta.cos() * RADIUS);
        let (x, y) = right.end();
        assert!(((x - centre.0).hypot(y - centre.1) - RADIUS).abs() < 1e-9);
    }
}

#[test]
fn interpolation_adds_the_gentler_turns() {
    let lattice = Lattice::hybrid(RADIUS, 72, CELL, true).unwrap();
    let moves = &lattice.moves[0];
    // Straight, full left and right (2 bins), then 1 bin each way at the same chord.
    assert_eq!(moves.len(), 5);
    let ends: Vec<usize> = moves.iter().map(|m| m.end_heading).collect();
    assert_eq!(ends, vec![0, 2, 70, 1, 71]);
    let chord = |i: usize| {
        let (x, y) = moves[i].end();
        x.hypot(y)
    };
    assert!((chord(3) - chord(1)).abs() < 1e-9);
    // The gentle arc is wider than full lock.
    assert!(moves[3].length / (TAU / 72.0) > RADIUS);
}

#[test]
fn a_tight_radius_turns_one_bin_and_bad_settings_are_refused() {
    // 1 m on 0.25 m cells: the least turn is 20.4 degrees, so 5 bins; 200 m: under one.
    let tight = Lattice::hybrid(1.0, 72, CELL, false).unwrap();
    assert_eq!(tight.moves[0][1].end_heading, 5);
    let wide = Lattice::hybrid(200.0, 72, CELL, false).unwrap();
    assert_eq!(wide.moves[0][1].end_heading, 1);
    // A radius under one cell becomes one cell (as Nav2).
    let tiny = Lattice::hybrid(0.01, 72, CELL, false).unwrap();
    assert!((tiny.turning_radius - CELL).abs() < 1e-12);
    assert!(Lattice::hybrid(RADIUS, 71, CELL, false).is_err());
    assert!(Lattice::hybrid(RADIUS, 0, CELL, false).is_err());
    assert!(Lattice::hybrid(RADIUS, 72, 0.0, false).is_err());
    assert!(Lattice::hybrid(RADIUS, 72, f64::NAN, false).is_err());
}

#[test]
fn headings_bin_to_the_nearest_of_72() {
    let lattice = Lattice::hybrid(RADIUS, 72, CELL, false).unwrap();
    let deg = |d: f64| d.to_radians();
    assert_eq!(lattice.nearest_heading(0.0), 0);
    assert_eq!(lattice.nearest_heading(deg(2.4)), 0);
    assert_eq!(lattice.nearest_heading(deg(2.6)), 1);
    assert_eq!(lattice.nearest_heading(deg(-2.6)), 71);
    assert_eq!(lattice.nearest_heading(deg(358.0)), 0);
    assert_eq!(lattice.nearest_heading(PI), 36);
    assert_eq!(lattice.nearest_heading(-PI), 36);
    assert_eq!(lattice.nearest_heading(deg(90.0)), 18);
    assert_eq!(lattice.opposite(18), 54);
}

#[test]
fn the_downsampled_obstacle_estimate_stays_close_and_goes_round_walls() {
    // 30 x 30 m, a wall across the middle with a gap at the top.
    let mut grid = Grid::new((0.0, 0.0), CELL, 120, 120, 0);
    for y in 0..100 {
        for x in 58..62 {
            grid.set(x, y, LETHAL);
        }
    }
    let (start, goal) = ((20, 40), (100, 40));
    let mut full = Obstacles::new(&grid, start, goal, 2.0, false);
    let mut half = Obstacles::new(&grid, start, goal, 2.0, true);
    let a = full.get(&grid, start.0, start.1);
    let b = half.get(&grid, start.0, start.1);
    // Round the wall: well over the straight 80 cells.
    assert!(a > 120.0, "{a}");
    // The blocks cost their cheapest cell, doubled: within a few cells of the full search.
    assert!((a - b).abs() < 0.05 * a, "full {a}, downsampled {b}");
    // Behind a wall from top to bottom: unreachable, 0.
    for y in 100..120 {
        for x in 58..62 {
            grid.set(x, y, LETHAL);
        }
    }
    let mut shut = Obstacles::new(&grid, start, goal, 2.0, true);
    assert_eq!(shut.get(&grid, start.0, start.1), 0.0);
}

#[test]
fn other_cells_make_the_moves_again() {
    // The lattice file is for 0.25 m cells only; Hybrid-A* plans on any.
    let mut grid = Grid::new((0.0, 0.0), 0.5, 80, 40, 0);
    scenes::inflation().apply(&mut grid);
    let (start, goal) = (Pose::new(5.0, 10.0, 0.0), Pose::new(30.0, 10.0, 0.0));
    assert_eq!(
        scenes::planner(false)
            .plan(&grid, &start, &goal, &|| false)
            .unwrap_err(),
        PlanError::WrongResolution
    );
    let hybrid = scenes::hybrid_planner(false);
    let plan = hybrid.plan(&grid, &start, &goal, &|| false).unwrap();
    let last = plan.poses.last().unwrap();
    assert!((last.x - goal.x).hypot(last.y - goal.y) < 0.6);
    // And back on 0.25 m cells.
    let scene = scenes::all().remove(0);
    assert!(hybrid
        .plan(&scene.grid, &scene.start, &scene.goal, &|| false)
        .is_ok());
}

#[test]
fn settings_are_nav2_yamls_gridbased_block() {
    let s = Settings::hybrid();
    assert_eq!(s.change_penalty, 0.0);
    assert_eq!(s.non_straight_penalty, 1.2);
    assert_eq!(s.reverse_penalty, 2.0);
    assert_eq!(s.cost_penalty, 2.0);
    let h = Hybrid::new(RADIUS);
    assert_eq!(h.angle_quantization_bins, 72);
    assert!(!h.allow_primitive_interpolation);
    assert!(h.downsample_obstacle_heuristic);
    let planner = Planner::hybrid(h, s, &scenes::outline(), None, CELL).unwrap();
    assert!(planner.describe().starts_with("Hybrid-A* (DUBIN)"));
    assert_eq!(planner.turning_radius(), RADIUS);
}

/// Plans the lot tour's legs with `planner`: each leg's plan, or the error.
fn tour(planner: &Planner) -> Vec<Result<lattice_planner::Plan, PlanError>> {
    let lot = scenes::map("example_lot").unwrap();
    let mut from = scenes::LOT_START;
    scenes::LOT_TOUR
        .iter()
        .map(|&(_, x, y, deg)| {
            let goal = Pose::new(x, y, deg.to_radians());
            let plan = planner.plan(&lot, &from, &goal, &|| false);
            from = goal;
            plan
        })
        .collect()
}

#[test]
fn the_lot_tour_forward_only_and_reversing() {
    for reversing in [false, true] {
        let hybrid = tour(&scenes::hybrid_planner(reversing));
        let lattice = tour(&scenes::planner(reversing));
        for (((name, x, y, _), h), l) in scenes::LOT_TOUR.iter().zip(&hybrid).zip(&lattice) {
            let what = format!("{name} (reversing {reversing})");
            let h = h.as_ref().unwrap_or_else(|e| panic!("{what}: {e}"));
            let l = l
                .as_ref()
                .unwrap_or_else(|e| panic!("{what}, the lattice: {e}"));
            let last = h.poses.last().unwrap();
            assert!(
                (last.x - x).hypot(last.y - y) < 0.6,
                "{what}: ends at {last:?}"
            );
            // The search's own path keeps to the radius.
            let turn = tightest_turn(&h.raw, 1.0);
            assert!(turn > 0.95 * RADIUS, "{what}: a {turn:.2} m turn");
            if !reversing {
                assert_eq!(cusps(&h.raw), 0, "{what}: reverses");
            }
            // About as long as the lattice's: within a fifth.
            let (hl, ll) = (
                lattice_planner::length(&h.poses),
                lattice_planner::length(&l.poses),
            );
            assert!(
                (hl - ll).abs() < 0.2 * ll,
                "{what}: {hl:.1} m against {ll:.1} m"
            );
            assert!(h.total_time < 2.0, "{what}: {} s", h.total_time);
        }
    }
}

#[test]
fn the_turn_back_reverses_and_is_shorter() {
    // (35, 42, 0) to (20, 42, pi): forward only, a loop round the lot; reversing, a turn
    // with a change of direction, half as long.
    let lot = scenes::map("example_lot").unwrap();
    let (from, to) = (Pose::new(35.0, 42.0, 0.0), Pose::new(20.0, 42.0, PI));
    let forward = scenes::hybrid_planner(false)
        .plan(&lot, &from, &to, &|| false)
        .unwrap();
    let back = scenes::hybrid_planner(true)
        .plan(&lot, &from, &to, &|| false)
        .unwrap();
    assert_eq!(cusps(&forward.raw), 0);
    assert!(cusps(&back.raw) >= 1);
    let (f, b) = (
        lattice_planner::length(&forward.poses),
        lattice_planner::length(&back.poses),
    );
    assert!(b < 0.7 * f, "reversing {b:.1} m, forward {f:.1} m");
    // Some of it is driven backwards: a step against the way the car faces.
    assert!(back.raw.windows(2).any(|w| {
        let (dx, dy) = (w[1].x - w[0].x, w[1].y - w[0].y);
        dx * w[0].yaw.cos() + dy * w[0].yaw.sin() < -0.01
    }));
}
