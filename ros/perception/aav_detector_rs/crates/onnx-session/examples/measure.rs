//! Any ONNX model from Rust, on each provider this build has: the same answers? how much
//! of the graph? how fast? The Rust half of `pixi run measure onnx`.
//!
//!     cargo run --release -p onnx-session --features coreml --example measure -- \
//!         models/diffusion/planner.onnx [RUNS]
//!
//! The inputs: made-up float32 numbers (0 to 1, the same each run) in each input's shape
//! (a dimension left open is 1). For each provider: the time to load, its share of the
//! graph (ONNX Runtime's log), which operators fell back to the CPU (one profiled run),
//! the median ms of RUNS runs (default 30) after 5 warm-up runs, and the largest gap from
//! the CPU's answers.

use std::time::Instant;

use onnx_session::{open, placements, Options, Provider};
use ort::session::Session;
use ort::value::{DynValue, Tensor};

type Inputs = Vec<(String, Vec<i64>, Vec<f32>)>;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let model = args.next().ok_or("usage: measure <model.onnx> [RUNS]")?;
    let runs: usize = args.next().map_or(Ok(30), |s| s.parse())?;

    let mut reference: Option<Vec<Vec<f32>>> = None;
    println!("| provider | load (s) | nodes taken | pieces | ms, median | max gap from the CPU |");
    println!("|---|---|---|---|---|---|");
    let mut fallbacks = Vec::new();
    for provider in Provider::built() {
        let start = Instant::now();
        let mut opened = open(&model, provider, &Options::default())?;
        let load = start.elapsed().as_secs_f64();
        let inputs = made_up(&opened.session)?;
        let answers = run(&mut opened.session, &inputs)?;
        for _ in 0..5 {
            run(&mut opened.session, &inputs)?;
        }
        let mut times: Vec<f64> = (0..runs)
            .map(|_| {
                let start = Instant::now();
                run(&mut opened.session, &inputs).map(|_| start.elapsed().as_secs_f64() * 1e3)
            })
            .collect::<Result<_, _>>()?;
        times.sort_by(f64::total_cmp);
        let median = times.get(times.len() / 2).copied().unwrap_or(f64::NAN);
        let gap = reference.as_ref().map_or(0.0, |r| max_gap(r, &answers));
        let (taken, pieces) = match (provider, opened.coverage) {
            (Provider::Cpu, _) => ("all".to_owned(), "1".to_owned()),
            (_, Some(c)) => (
                format!("{} of {} ({:.1} %)", c.taken, c.nodes, 100.0 * c.share()),
                c.partitions.to_string(),
            ),
            (_, None) => ("not logged".to_owned(), "?".to_owned()),
        };
        println!("| rust {provider} | {load:.1} | {taken} | {pieces} | {median:.2} | {gap:.2e} |");
        if reference.is_none() {
            reference = Some(answers);
        }

        let prefix = std::env::temp_dir().join(format!("onnx-session-{provider}"));
        let options = Options {
            profile: Some(prefix),
            ..Options::default()
        };
        let mut profiled = open(&model, provider, &options)?;
        run(&mut profiled.session, &inputs)?;
        let file = profiled.session.end_profiling()?;
        let p = placements(&std::fs::read_to_string(&file)?)?;
        let ops: Vec<String> = p.cpu_ops.iter().map(|(o, n)| format!("{o} {n}")).collect();
        fallbacks.push(format!(
            "- {provider}: the CPU ran {}",
            if ops.is_empty() {
                "nothing".to_owned()
            } else {
                ops.join(", ")
            }
        ));
    }
    println!();
    for line in fallbacks {
        println!("{line}");
    }
    Ok(())
}

/// Made-up float32 inputs in the model's shapes, 0 to 1, the same every time.
fn made_up(session: &Session) -> Result<Inputs, Box<dyn std::error::Error>> {
    let mut seed: u32 = 12345;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as f32 / u32::MAX as f32
    };
    session
        .inputs()
        .iter()
        .map(|input| {
            let shape: Vec<i64> = input
                .dtype()
                .tensor_shape()
                .ok_or_else(|| format!("input {} isn't a tensor", input.name()))?
                .iter()
                .map(|&d| if d > 0 { d } else { 1 })
                .collect();
            let len = shape.iter().product::<i64>() as usize;
            Ok((
                input.name().to_owned(),
                shape,
                (0..len).map(|_| next()).collect(),
            ))
        })
        .collect()
}

/// Run once; every output as float32, in the model's order.
fn run(session: &mut Session, inputs: &Inputs) -> Result<Vec<Vec<f32>>, ort::Error> {
    let feed: Vec<(String, DynValue)> = inputs
        .iter()
        .map(|(name, shape, data)| {
            Tensor::from_array((shape.clone(), data.clone())).map(|t| (name.clone(), t.into_dyn()))
        })
        .collect::<Result<_, _>>()?;
    let outputs = session.run(feed)?;
    outputs
        .iter()
        .map(|(_, v)| v.try_extract_tensor::<f32>().map(|(_, d)| d.to_vec()))
        .collect()
}

fn max_gap(a: &[Vec<f32>], b: &[Vec<f32>]) -> f64 {
    a.iter()
        .zip(b)
        .flat_map(|(x, y)| x.iter().zip(y))
        .map(|(x, y)| f64::from((x - y).abs()))
        .fold(0.0, f64::max)
}
