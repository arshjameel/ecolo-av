//! The test pictures, the model's place, and the Python's boxes.

use std::path::{Path, PathBuf};

/// This crate's folder.
pub fn here() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// The ONNX model: $DETECTOR_ONNX, else the repository's models/rf-detr-nano.onnx (made by
/// tools/export_model.py --format onnx; not in git).
pub fn model() -> PathBuf {
    std::env::var_os("DETECTOR_ONNX")
        .map(PathBuf::from)
        .unwrap_or_else(|| here().join("../../../../../models/rf-detr-nano.onnx"))
}

/// The test pictures, by file name, sorted: RGB bytes, width, height.
pub fn pictures() -> Vec<(String, Vec<u8>, usize, usize)> {
    let mut files: Vec<PathBuf> = std::fs::read_dir(here().join("tests/pictures"))
        .expect("tests/pictures")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "png"))
        .collect();
    files.sort();
    files.iter().map(|f| load(f)).collect()
}

fn load(file: &Path) -> (String, Vec<u8>, usize, usize) {
    let rgb = image::open(file).expect("a PNG").to_rgb8();
    let (w, h) = (rgb.width() as usize, rgb.height() as usize);
    let name = file.file_name().unwrap().to_string_lossy().into_owned();
    (name, rgb.into_raw(), w, h)
}
