//! Hybrid-A* against our State Lattice, offline, on the same maps:
//!
//! ```sh
//! cargo run --release -p lattice-planner --example hybrid
//! ```
//!
//! The lot tour (`measure controllers --tours lot`'s four legs on `example_lot`, the map
//! the simulator plans on), then the hand-made maps, each forward only (Dubins) and
//! reversing (Reeds-Shepp). Per plan: its length, the planning time (the best of 3), the
//! tightest turn (m, over 1 m spans, between cusps; the planners plan at 3.57 m, the car
//! turns at 2.86 m at full lock), smoothed and as the search found it, the
//! cusps (changes of direction) and the search's iterations. Both use nav2.yaml's blocks:
//! the lattice's `Lattice`, Hybrid-A*'s `GridBased`.

use lattice_planner::{cusps, length, scenes, tightest_turn, Plan, Planner, Pose};

/// The best of `runs` plans by total time, or the error.
fn best_of(
    planner: &Planner,
    grid: &costmap::Grid,
    from: &Pose,
    to: &Pose,
    runs: usize,
) -> Result<Plan, lattice_planner::PlanError> {
    let mut best: Option<Plan> = None;
    for _ in 0..runs {
        let p = planner.plan(grid, from, to, &|| false)?;
        if best.as_ref().is_none_or(|b| p.total_time < b.total_time) {
            best = Some(p);
        }
    }
    best.ok_or(lattice_planner::PlanError::NoValidPath)
}

fn row(
    search: &str,
    reversing: bool,
    what: &str,
    result: &Result<Plan, lattice_planner::PlanError>,
) {
    match result {
        Ok(p) => println!(
            "| {search} | {reversing} | {what} | {:.1} | {:.0} | {:.2} | {:.2} | {} | {} |",
            length(&p.poses),
            p.total_time * 1e3,
            tightest_turn(&p.poses, 1.0),
            tightest_turn(&p.raw, 1.0),
            cusps(&p.poses),
            p.iterations,
        ),
        Err(e) => println!("| {search} | {reversing} | {what} | {e} | | | | | |"),
    }
}

fn main() {
    let lot = scenes::map("example_lot").expect("the example lot");
    println!("The lot tour (example_lot):");
    println!();
    println!(
        "| search | reversing | leg | length m | plan ms | tightest turn m | before smoothing m | cusps | iterations |"
    );
    println!("|---|---|---|---|---|---|---|---|---|");
    for reversing in [false, true] {
        for (search, planner) in [
            ("lattice", scenes::planner(reversing)),
            ("hybrid", scenes::hybrid_planner(reversing)),
        ] {
            let mut from = scenes::LOT_START;
            let (mut metres, mut seconds) = (0.0, 0.0);
            for (name, x, y, deg) in scenes::LOT_TOUR {
                let goal = Pose::new(x, y, deg.to_radians());
                let result = best_of(&planner, &lot, &from, &goal, 3);
                if let Ok(p) = &result {
                    metres += length(&p.poses);
                    seconds += p.total_time;
                }
                row(search, reversing, name, &result);
                from = goal;
            }
            println!(
                "| {search} | {reversing} | **whole tour** | {metres:.1} | {:.0} | | | | |",
                seconds * 1e3
            );
        }
    }
    println!();
    println!("The hand-made maps (scenes.rs):");
    println!();
    println!("| search | reversing | scene | length m | plan ms | tightest turn m | before smoothing m | cusps | iterations |");
    println!("|---|---|---|---|---|---|---|---|---|");
    for reversing in [false, true] {
        for (search, planner) in [
            ("lattice", scenes::planner(reversing)),
            ("hybrid", scenes::hybrid_planner(reversing)),
        ] {
            for scene in scenes::all() {
                let result = best_of(&planner, &scene.grid, &scene.start, &scene.goal, 3);
                row(search, reversing, scene.name, &result);
            }
        }
    }
}
