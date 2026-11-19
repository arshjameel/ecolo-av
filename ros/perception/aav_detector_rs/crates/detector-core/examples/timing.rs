//! How fast is the detector here, and which provider runs which nodes?
//!
//!     cargo run --release -p detector-core --features ort,coreml --example timing -- [RUNS]
//!
//! For each provider this build has (the CPU; Core ML with `coreml`): the time to load,
//! then, after 5 warm-up runs, the median and 90th-percentile time per picture over RUNS
//! passes (default 30) of the 640 x 360 test pictures: resize, model and decode together.
//! Then one profiled run: ONNX Runtime's trace says which provider ran each node.
//! The model: $DETECTOR_ONNX, else models/rf-detr-nano.onnx.

use std::collections::BTreeMap;
use std::time::Instant;

use detector_core::{Detector, Picture, PixelOrder, Provider};

#[path = "../tests/common/mod.rs"]
mod common;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let runs: usize = std::env::args().nth(1).map_or(Ok(30), |s| s.parse())?;
    let model = common::model();
    let pictures: Vec<_> = common::pictures()
        .into_iter()
        .filter(|(_, _, w, h)| (*w, *h) == (640, 360))
        .collect();
    let mut providers = vec![Provider::Cpu];
    if cfg!(all(feature = "coreml", target_os = "macos")) {
        providers.push(Provider::CoreMl);
    }
    println!("| provider | load (s) | ms per picture, median | 90th percentile |");
    println!("|---|---|---|---|");
    for &provider in &providers {
        let start = Instant::now();
        let mut detector = Detector::open(&model, provider)?;
        let load = start.elapsed().as_secs_f64();
        let (name, rgb, w, h) = &pictures[0];
        let first = Picture::packed(rgb, *w, *h, PixelOrder::Rgb)?;
        for _ in 0..5 {
            detector.detect(&first, 0.5)?;
        }
        let mut times = Vec::with_capacity(runs * pictures.len());
        for _ in 0..runs {
            for (_, rgb, w, h) in &pictures {
                let picture = Picture::packed(rgb, *w, *h, PixelOrder::Rgb)?;
                let start = Instant::now();
                detector.detect(&picture, 0.5)?;
                times.push(start.elapsed().as_secs_f64() * 1000.0);
            }
        }
        times.sort_by(f64::total_cmp);
        let at = |q: f64| times[((times.len() - 1) as f64 * q).round() as usize];
        println!(
            "| rust ort-{provider} | {load:.1} | {:.1} | {:.1} |",
            at(0.5),
            at(0.9)
        );
        let found = detector.detect(&first, 0.5)?;
        for d in found {
            eprintln!("  {name}: {} {:.2} {:?}", d.class, d.score, d.corners);
        }
    }

    println!("\nWhich provider ran which nodes (one profiled run):\n");
    for &provider in &providers {
        let prefix = std::env::temp_dir().join(format!("detector-core-{provider}"));
        let mut detector = Detector::open_with(&model, provider, Some(&prefix))?;
        let (_, rgb, w, h) = &pictures[0];
        detector.detect(&Picture::packed(rgb, *w, *h, PixelOrder::Rgb)?, 0.5)?;
        let file = detector.end_profiling()?;
        let counts = placements(&std::fs::read_to_string(&file)?)?;
        let line: Vec<String> = counts.iter().map(|(p, n)| format!("{p}: {n}")).collect();
        println!("- {provider}: {} ({file})", line.join(", "));
        for note in detector.notes() {
            println!("  - onnxruntime: {note}");
        }
    }
    Ok(())
}

/// Count the nodes each provider ran, from an ONNX Runtime profile (its JSON trace).
/// A provider that fuses a part of the graph (Core ML) counts that part as one node.
fn placements(trace: &str) -> Result<BTreeMap<String, usize>, serde_json::Error> {
    let events: Vec<serde_json::Value> = serde_json::from_str(trace)?;
    let mut nodes: BTreeMap<String, String> = BTreeMap::new();
    for e in &events {
        if e["cat"] == "Node" {
            if let (Some(name), Some(provider)) =
                (e["name"].as_str(), e["args"]["provider"].as_str())
            {
                let node = name.trim_end_matches("_kernel_time").to_owned();
                nodes.insert(node, provider.to_owned());
            }
        }
    }
    let mut counts = BTreeMap::new();
    for provider in nodes.into_values() {
        *counts.entry(provider).or_insert(0) += 1;
    }
    Ok(counts)
}
