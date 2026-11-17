//! The Rust finds the Python's boxes: the same ONNX model, on the same saved pictures.
//!
//! tests/python_boxes.json is ONNX Runtime's answer in Python, through aav_detector's
//! detr.py (tools/onnx_check.py --write-fixtures; that script also checks those boxes
//! against PyTorch's). Same boxes: every box of either over 0.3 has a twin of the same
//! class in the other, IoU at least 0.95 and scores within 0.02. A box within 0.05 of
//! the threshold may miss (its twin can fall just under).
//!
//! Needs the model (not in git): without it the test says so and passes.
#![cfg(feature = "ort")]

mod common;

use detector_core::{Corners, Detector, Picture, PixelOrder, Provider};

const THRESHOLD: f32 = 0.3;
const IOU_MIN: f32 = 0.95;
const SCORE_GAP: f32 = 0.02;

struct Expected {
    corners: Corners,
    score: f32,
    label: String,
}

fn expected(picture: &str) -> Vec<Expected> {
    let text = std::fs::read_to_string(common::here().join("tests/python_boxes.json"))
        .expect("tests/python_boxes.json");
    let all: serde_json::Value = serde_json::from_str(&text).expect("JSON");
    let boxes = all[picture]
        .as_array()
        .unwrap_or_else(|| panic!("{picture} has no boxes in python_boxes.json"));
    boxes
        .iter()
        .map(|b| {
            let c: Vec<f32> = b["corners"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap() as f32)
                .collect();
            Expected {
                corners: Corners {
                    x1: c[0],
                    y1: c[1],
                    x2: c[2],
                    y2: c[3],
                },
                score: b["score"].as_f64().unwrap() as f32,
                label: b["label"].as_str().unwrap().to_owned(),
            }
        })
        .collect()
}

fn providers() -> Vec<Provider> {
    let mut all = vec![Provider::Cpu];
    if cfg!(all(feature = "coreml", target_os = "macos")) {
        all.push(Provider::CoreMl);
    }
    all
}

#[test]
fn same_boxes_as_the_python() {
    let model = common::model();
    if !model.exists() {
        eprintln!(
            "SKIPPED: {} is missing (tools/export_model.py --format onnx)",
            model.display()
        );
        return;
    }
    for provider in providers() {
        let mut detector = Detector::open(&model, provider).expect("the model loads");
        let mut worst_iou: f32 = 1.0;
        let mut worst_gap: f32 = 0.0;
        for (name, rgb, w, h) in common::pictures() {
            let picture = Picture::packed(&rgb, w, h, PixelOrder::Rgb).unwrap();
            let found = detector.detect(&picture, THRESHOLD).unwrap();
            let want = expected(&name);
            for d in &found {
                let label = d.class.to_string();
                let twin = want.iter().filter(|e| e.label == label).max_by(|a, b| {
                    a.corners
                        .iou(&d.corners)
                        .total_cmp(&b.corners.iou(&d.corners))
                });
                match twin {
                    Some(e) if e.corners.iou(&d.corners) >= IOU_MIN => {
                        worst_iou = worst_iou.min(e.corners.iou(&d.corners));
                        worst_gap = worst_gap.max((e.score - d.score).abs());
                    }
                    _ => assert!(
                        d.score < THRESHOLD + 0.05,
                        "{provider}, {name}: {label} {:.2} at {:?} isn't in the Python's boxes",
                        d.score,
                        d.corners
                    ),
                }
            }
            for e in &want {
                let twin = found.iter().any(|d| {
                    d.class.to_string() == e.label && d.corners.iou(&e.corners) >= IOU_MIN
                });
                assert!(
                    twin || e.score < THRESHOLD + 0.05,
                    "{provider}, {name}: the Python's {} {:.2} is missing",
                    e.label,
                    e.score
                );
            }
        }
        eprintln!("{provider}: worst IoU {worst_iou:.4}, worst score gap {worst_gap:.4}");
        assert!(
            worst_gap <= SCORE_GAP,
            "{provider}: scores differ by {worst_gap}"
        );
    }
}

#[test]
fn a_wrong_model_is_an_error_not_a_crash() {
    let not_a_model = common::here().join("tests/python_boxes.json");
    assert!(Detector::open(not_a_model, Provider::Cpu).is_err());
}
