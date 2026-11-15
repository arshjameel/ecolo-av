//! The turns in a street, by the price of a change of gear (`cusp_penalty`, TODO 174):
//! `cargo run --release -p lattice-planner --example cusps`.

use lattice_planner::{cusps, length, scenes, Planner, Settings};

fn main() {
    println!("| scene | cusp_penalty m | result | length m | cusps | search ms |");
    println!("|---|---|---|---|---|---|");
    for penalty in [0.0, 1.0, 2.0, 4.0, 8.0] {
        let settings = Settings {
            allow_reverse: true,
            cusp_penalty: penalty,
            ..Settings::default()
        };
        let planner = Planner::new(
            scenes::lattice(),
            settings,
            &scenes::outline(),
            Some(&scenes::inflation()),
        );
        for scene in scenes::all()
            .into_iter()
            .filter(|s| !s.forward_ok && s.reverse_ok)
        {
            match planner.plan(&scene.grid, &scene.start, &scene.goal, &|| false) {
                Ok(p) => println!(
                    "| {} | {penalty} | path | {:.1} | {} | {:.1} |",
                    scene.name,
                    length(&p.poses),
                    cusps(&p.poses),
                    p.search_time * 1e3
                ),
                Err(e) => println!("| {} | {penalty} | {e} | | | |", scene.name),
            }
        }
    }
}
