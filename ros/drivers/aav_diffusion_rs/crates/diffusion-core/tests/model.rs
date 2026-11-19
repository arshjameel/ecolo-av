//! The model itself, if `models/diffusion/planner.onnx` is there (`pixi run diffusion
//! train`); without it these tests pass with a note, as the detector's do.

use std::path::PathBuf;

use diffusion_core::{Model, Provider, CHANNELS, COND, POINTS, SIZE};

fn model() -> Option<PathBuf> {
    let path = std::env::var_os("DIFFUSION_ONNX")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../../../../models/diffusion/planner.onnx")
        });
    if path.exists() {
        Some(path)
    } else {
        eprintln!("no {}: the model's tests skipped", path.display());
        None
    }
}

fn inputs(samples: usize) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
    let mut seed: u32 = 99;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as f32 / u32::MAX as f32
    };
    let crop = (0..SIZE * SIZE)
        .map(|_| if next() > 0.9 { 1.0 } else { 0.0 })
        .collect();
    let cond = vec![-0.4, 0.0, 0.5, 0.2, 0.0, 1.0, 1.0, 0.25];
    assert_eq!(COND, 8);
    let noise = (0..samples * POINTS * CHANNELS)
        .map(|_| 2.0 * next() - 1.0)
        .collect();
    (crop, cond, noise)
}

#[test]
fn every_provider_draws_the_cpus_paths() {
    let Some(path) = model() else { return };
    let (crop, cond, noise) = inputs(16);
    let mut cpu = Model::open(&path, Provider::Cpu).unwrap();
    let want = cpu.sample(&crop, &cond, &noise).unwrap();
    assert_eq!(want.len(), noise.len());
    for provider in Provider::built().into_iter().skip(1) {
        let mut m = Model::open(&path, provider).unwrap();
        let got = m.sample(&crop, &cond, &noise).unwrap();
        let gap = want
            .iter()
            .zip(&got)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f32::max);
        // In the window's half-sides: 1e-3 is 1.6 cm in the smallest window (32 m).
        assert!(gap < 1e-3, "{provider}: {gap}");
    }
}

#[test]
fn any_number_of_samples() {
    let Some(path) = model() else { return };
    let mut m = Model::open(&path, Provider::Cpu).unwrap();
    let (crop, cond, noise) = inputs(20);
    let paths = m.sample(&crop, &cond, &noise).unwrap();
    assert_eq!(paths.len(), 20 * POINTS * CHANNELS);
    // The first 16 as one batch alone give the same paths.
    let first = m
        .sample(&crop, &cond, &noise[..16 * POINTS * CHANNELS])
        .unwrap();
    assert_eq!(&paths[..first.len()], &first[..]);
}

#[test]
fn a_wrong_file_is_an_error_not_a_crash() {
    let not_a_model = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml");
    assert!(Model::open(not_a_model, Provider::Cpu).is_err());
}
