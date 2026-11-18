//! The campus tour's legs, planned and timed on the real campus map (TODO 152: ours took
//! about 180 ms against Nav2's 150 ms on the long legs). Needs `pixi run campus` first.
//! `cargo run --release -p lattice-planner --example campus`
//!
//! `CUSP_PENALTY=4` prices a change of gear as the sim's planner does (default 0: Nav2's
//! way); `WHOLE=1` plans on the whole campus map (6400 x 6400 cells), as the sim's global
//! costmap is, not the tour's 1280 x 1280 corner.

use lattice_planner::{scenes, Planner, Pose, Settings};

fn main() -> Result<(), String> {
    let number = |name: &str| std::env::var(name).ok().and_then(|v| v.parse::<f64>().ok());
    let cusp_penalty = number("CUSP_PENALTY").unwrap_or(0.0);
    let grid = if number("WHOLE").is_some_and(|w| w > 0.0) {
        scenes::campus_area((-900.0, -900.0), (700.0, 700.0))?
    } else {
        scenes::campus()?
    };
    println!("cusp_penalty {cusp_penalty} m");
    println!("campus map: {} x {} cells", grid.width, grid.height);
    println!(
        "| leg | reversing | result | length m | iterations | search ms (best of 5) | total ms |"
    );
    println!("|---|---|---|---|---|---|---|");
    for reversing in [false, true] {
        let settings = Settings {
            allow_reverse: reversing,
            cusp_penalty,
            ..Settings::default()
        };
        let planner = Planner::new(
            scenes::lattice(),
            settings,
            &scenes::outline(),
            Some(&scenes::inflation()),
        );
        let mut from = scenes::CAMPUS_START;
        for (name, x, y, deg) in scenes::CAMPUS_TOUR {
            let goal = Pose::new(x, y, deg.to_radians());
            let runs: Vec<_> = (0..5)
                .map(|_| planner.plan(&grid, &from, &goal, &|| false))
                .collect();
            match runs
                .iter()
                .filter_map(|r| r.as_ref().ok())
                .min_by(|a, b| a.search_time.total_cmp(&b.search_time))
            {
                Some(p) => println!(
                    "| {name} | {reversing} | path | {:.1} | {} | {:.1} | {:.1} |",
                    lattice_planner::length(&p.poses),
                    p.iterations,
                    p.search_time * 1e3,
                    p.total_time * 1e3
                ),
                None => println!(
                    "| {name} | {reversing} | {:?} | | | | |",
                    runs[0].as_ref().err()
                ),
            }
            from = goal;
        }
    }
    Ok(())
}
