//! How fast is the diffusion model from Rust here, on each provider this build has?
//!
//!     cargo run --release -p diffusion-core --features coreml --example bench -- [MODEL] [RUNS]
//!
//! MODEL: default `models/diffusion/planner.onnx` (from the repo's root). For each
//! provider: the time to load, its share of the graph, and the median and 90th-percentile
//! ms for one plan (16 samples, 10 steps) over RUNS runs (default 50) after 5 warm-up
//! runs; and the largest gap from the CPU's paths.

use std::time::Instant;

use diffusion_core::{Model, Provider, CHANNELS, COND, POINTS, SIZE};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let model = args
        .next()
        .unwrap_or_else(|| "models/diffusion/planner.onnx".to_owned());
    let runs: usize = args.next().map_or(Ok(50), |s| s.parse())?;
    let crop: Vec<f32> = (0..SIZE * SIZE)
        .map(|i| if (i / SIZE) % 37 < 3 { 1.0 } else { 0.0 })
        .collect();
    let cond = [-0.4, 0.0, 0.5, 0.2, 0.0, 1.0, 1.0, 0.25];
    assert_eq!(cond.len(), COND);
    let noise: Vec<f32> = (0..16 * POINTS * CHANNELS)
        .map(|i| ((i * 7919) % 2000) as f32 / 1000.0 - 1.0)
        .collect();

    println!(
        "| rust, provider | load (s) | nodes taken | ms a plan, median | 90th | gap from the CPU |"
    );
    println!("|---|---|---|---|---|---|");
    let mut reference: Option<Vec<f32>> = None;
    for provider in Provider::built() {
        let start = Instant::now();
        let mut m = Model::open(&model, provider)?;
        let load = start.elapsed().as_secs_f64();
        let paths = m.sample(&crop, &cond, &noise)?;
        for _ in 0..5 {
            m.sample(&crop, &cond, &noise)?;
        }
        let mut times = Vec::with_capacity(runs);
        for _ in 0..runs {
            let start = Instant::now();
            m.sample(&crop, &cond, &noise)?;
            times.push(start.elapsed().as_secs_f64() * 1000.0);
        }
        times.sort_by(f64::total_cmp);
        let at = |q: f64| times[((times.len() - 1) as f64 * q).round() as usize];
        let gap = reference.as_ref().map_or(0.0, |r| {
            r.iter()
                .zip(&paths)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f32::max)
        });
        let taken = match (provider, m.coverage()) {
            (Provider::Cpu, _) => "all".to_owned(),
            (_, Some(c)) => format!("{} of {}, {} piece(s)", c.taken, c.nodes, c.partitions),
            (_, None) => "not logged".to_owned(),
        };
        println!(
            "| {provider} | {load:.1} | {taken} | {:.1} | {:.1} | {gap:.1e} |",
            at(0.5),
            at(0.9)
        );
        reference.get_or_insert(paths);
    }
    Ok(())
}
