//! The planner on the hand-made maps: a path where one exists, none where none does, and
//! every path one the car can drive (clear of walls, starting at the start, ending at the
//! goal, no turn much tighter than the car's).

use costmap::cost::LETHAL;
use costmap::footprint;
use lattice_planner::{cusps, length, scenes, tightest_turn, PlanError, Pose};

/// The car's outline is off every lethal cell at this pose.
fn clear(grid: &costmap::Grid, p: &Pose) -> bool {
    let pose = costmap::Pose2 {
        x: p.x,
        y: p.y,
        yaw: p.yaw,
    };
    let placed = footprint::placed(&scenes::CAR, &pose);
    let cells = footprint::cells_under(grid, &placed);
    !cells.is_empty() && cells.iter().all(|&c| grid.data[c] < LETHAL)
}

#[test]
fn each_scene_with_and_without_reversing() {
    each_scene(scenes::planner);
}

#[test]
fn each_scene_with_and_without_reversing_hybrid_a_star() {
    each_scene(scenes::hybrid_planner);
}

/// Every scene, forward only and reversing, with the planner `make` gives.
fn each_scene(make: fn(bool) -> lattice_planner::Planner) {
    let lattice = scenes::lattice();
    for reversing in [false, true] {
        let planner = make(reversing);
        for scene in scenes::all() {
            let expect = if reversing {
                scene.reverse_ok
            } else {
                scene.forward_ok
            };
            let what = format!("{} (reversing {reversing})", scene.name);
            match planner.plan(&scene.grid, &scene.start, &scene.goal, &|| false) {
                Ok(plan) => {
                    assert!(expect, "{what}: found a path where none should be");
                    let (first, last) = (plan.poses[0], *plan.poses.last().unwrap());
                    assert!(
                        (first.x - scene.start.x).hypot(first.y - scene.start.y) < 0.5,
                        "{what}: starts at {first:?}"
                    );
                    assert!(
                        (last.x - scene.goal.x).hypot(last.y - scene.goal.y) < 0.6,
                        "{what}: ends at {last:?}"
                    );
                    for p in &plan.raw {
                        assert!(
                            clear(&scene.grid, p),
                            "{what}: the car hits a wall at {p:?}"
                        );
                    }
                    let turn = tightest_turn(&plan.raw, 1.0);
                    assert!(
                        turn > 0.8 * lattice.turning_radius,
                        "{what}: a {turn:.2} m turn"
                    );
                    if !reversing {
                        assert_eq!(cusps(&plan.raw), 0, "{what}: reverses");
                    }
                    assert!(plan.total_time < 2.0, "{what}: {} s", plan.total_time);
                    assert!(length(&plan.poses) > 0.0);
                }
                Err(e) => {
                    assert!(!expect, "{what}: no path ({e})");
                    assert!(
                        matches!(
                            e,
                            PlanError::NoValidPath | PlanError::Timeout | PlanError::StartOccupied
                        ),
                        "{what}: {e}"
                    );
                }
            }
        }
    }
}

#[test]
fn reversing_out_of_the_parking_space_backs_out_first() {
    backs_out_first(scenes::planner);
}

#[test]
fn reversing_out_of_the_parking_space_backs_out_first_hybrid_a_star() {
    backs_out_first(scenes::hybrid_planner);
}

fn backs_out_first(make: fn(bool) -> lattice_planner::Planner) {
    let scene = scenes::all()
        .into_iter()
        .find(|s| s.name == "parking-lot exit")
        .unwrap();
    let plan = make(true)
        .plan(&scene.grid, &scene.start, &scene.goal, &|| false)
        .unwrap();
    // Its first metre runs behind the car (the car faces +x).
    let p = plan
        .raw
        .iter()
        .find(|p| (p.x - 12.0).hypot(p.y - 10.0) > 1.0)
        .unwrap();
    assert!(p.x < 12.0, "first move {p:?}");
    assert!(cusps(&plan.raw) >= 1);
}

#[test]
fn off_the_map_and_cancelled() {
    let scene = scenes::all().remove(0);
    let planner = scenes::planner(false);
    let off = Pose::new(-5.0, 0.0, 0.0);
    assert_eq!(
        planner
            .plan(&scene.grid, &off, &scene.goal, &|| false)
            .unwrap_err(),
        PlanError::StartOutsideMap
    );
    assert_eq!(
        planner
            .plan(&scene.grid, &scene.start, &off, &|| false)
            .unwrap_err(),
        PlanError::GoalOutsideMap
    );
    assert_eq!(
        planner
            .plan(&scene.grid, &scene.start, &scene.goal, &|| true)
            .unwrap_err(),
        PlanError::Cancelled
    );
}

#[test]
fn through_poses_joins_the_legs_and_says_which_leg_failed() {
    // Open ground (the straight scene): start, a pose half way, then the scene's goal.
    let scene = scenes::all().remove(0);
    let planner = scenes::planner(false);
    let middle = Pose::new(
        (scene.start.x + scene.goal.x) / 2.0,
        (scene.start.y + scene.goal.y) / 2.0,
        scene.goal.yaw,
    );
    let plan = planner
        .plan_through(&scene.grid, &scene.start, &[middle, scene.goal], &|| false)
        .unwrap();
    let last = *plan.poses.last().unwrap();
    assert!((last.x - scene.goal.x).hypot(last.y - scene.goal.y) < 0.6);
    // It passes the middle pose.
    let nearest = plan
        .poses
        .iter()
        .map(|p| (p.x - middle.x).hypot(p.y - middle.y))
        .fold(f64::INFINITY, f64::min);
    assert!(nearest < 0.6, "{nearest:.2} m from the middle pose");
    // No repeated pose where the legs join.
    assert!(plan
        .poses
        .windows(2)
        .all(|w| (w[0].x - w[1].x).hypot(w[0].y - w[1].y) > 1e-6));
    // Off the map on the second leg: leg 1.
    let off = Pose::new(-5.0, 0.0, 0.0);
    assert_eq!(
        planner
            .plan_through(&scene.grid, &scene.start, &[middle, off], &|| false)
            .unwrap_err(),
        (1, PlanError::GoalOutsideMap)
    );
    assert_eq!(
        planner
            .plan_through(&scene.grid, &scene.start, &[], &|| false)
            .unwrap_err(),
        (0, PlanError::NoValidPath)
    );
}

/// TODO 174: priced, a change of gear is rare. sim-check --three-point's 6.5 m street turns
/// with 3 cusps at most, the 8 m road with 2 (Nav2's way, unpriced: 6 and 5).
#[test]
fn a_priced_change_of_gear_turns_in_a_street_with_few_cusps() {
    let settings = lattice_planner::Settings {
        allow_reverse: true,
        cusp_penalty: 4.0,
        ..lattice_planner::Settings::default()
    };
    let planner = lattice_planner::Planner::new(
        scenes::lattice(),
        settings,
        &scenes::outline(),
        Some(&scenes::inflation()),
    );
    for (name, most) in [("three-point turn", 3), ("u-turn in a road", 2)] {
        let scene = scenes::all()
            .into_iter()
            .find(|s| s.name == name)
            .expect("the scene");
        let plan = planner
            .plan(&scene.grid, &scene.start, &scene.goal, &|| false)
            .expect("a path");
        let n = lattice_planner::cusps(&plan.poses);
        assert!(n <= most, "{name}: {n} cusps");
    }
}
