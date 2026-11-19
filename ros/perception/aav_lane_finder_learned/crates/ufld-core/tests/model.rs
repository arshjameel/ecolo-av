//! The model itself on Ravensim's pictures (feature `ort`). Skips, saying so, without
//! `models/lane-ufld.onnx` (outside git: `tools/lane_train.py` makes it), or another model
//! named by `AAV_LANE_MODEL`.
#![cfg(feature = "ort")]

use std::path::PathBuf;

use lane_core::Paint;
use lane_core::{Encoding, Lens, Mount, Picture, Settings};
use ufld_core::{decode, input, paints, Learned, Model, Provider};

/// The front left camera on the car (vehicle.yaml camera_left_link), as Ravensim draws it.
const MOUNT: Mount = Mount {
    xyz: [1.75, 0.29, 1.55],
    rpy: [0.0, 0.0, 0.0],
};

fn model_file() -> Option<PathBuf> {
    let path = std::env::var_os("AAV_LANE_MODEL").map_or_else(
        || PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../../../models/lane-ufld.onnx"),
        PathBuf::from,
    );
    if path.exists() {
        Some(path)
    } else {
        eprintln!("SKIP: no {} (tools/lane_train.py makes it)", path.display());
        None
    }
}

fn rgb(path: &str) -> (Vec<u8>, usize, usize) {
    let decoder = png::Decoder::new(std::io::BufReader::new(
        std::fs::File::open(path).expect("picture"),
    ));
    let mut reader = decoder.read_info().expect("png");
    let mut buf = vec![0; reader.output_buffer_size().expect("size")];
    let info = reader.next_frame(&mut buf).expect("frame");
    buf.truncate(info.buffer_size());
    let rgb = match info.color_type {
        png::ColorType::Rgb => buf,
        png::ColorType::Rgba => buf.chunks(4).flat_map(|p| [p[0], p[1], p[2]]).collect(),
        other => panic!("{path}: {other:?}"),
    };
    (rgb, info.width as usize, info.height as usize)
}

fn lane_of(model: &mut Model, data: &[u8], w: usize, h: usize) -> lane_core::Lane {
    let picture = Picture::packed(data, w, h, Encoding::Rgb8).unwrap();
    let a = model.run(&input(&picture, model.meta()).unwrap()).unwrap();
    let lines = decode(&a.loc, &a.exist, model.meta(), 0.5).unwrap();
    let paints = a
        .colour
        .map(|c| paints(&c, model.meta()).unwrap())
        .unwrap_or_default();
    let mut learned = Learned::new(model.meta().clone(), Settings::default());
    learned
        .lane(&lines, &paints, &Lens::pinhole(w, h, 343.0), &MOUNT)
        .unwrap()
}

#[test]
fn finds_ravensims_lane_and_nothing_on_a_blank_road() {
    let Some(path) = model_file() else { return };
    let mut model = Model::open(&path, Provider::Cpu).expect("the model");
    let dir = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../aav_lane_finder_rs/crates/lane-core/tests/pictures/"
    );
    let mut both = 0;
    for name in [
        "ravensim_frame_00",
        "ravensim_frame_02",
        "ravensim_frame_04",
    ] {
        let (data, w, h) = rgb(&format!("{dir}{name}.png"));
        let lane = lane_of(&mut model, &data, w, h);
        eprintln!(
            "{name}: left {:?} ({:?}) right {:?} ({:?}) offset {:.2} m width {:.2} m",
            lane.left.map(|l| l.c),
            lane.left_paint,
            lane.right.map(|r| r.c),
            lane.right_paint,
            lane.offset,
            lane.width
        );
        // The campus, University Drive: the yellow dashed centre on the left, the white
        // edge line on the right.
        if model.has_colour() {
            if lane.left.is_some() {
                assert_eq!(lane.left_paint, Paint::Yellow, "{name}: the centre line");
            }
            if lane.right.is_some() {
                assert_eq!(lane.right_paint, Paint::White, "{name}: the edge line");
            }
        }
        assert!(
            lane.left.is_some() || lane.right.is_some(),
            "{name}: no line"
        );
        if let (Some(l), Some(r)) = (lane.left, lane.right) {
            both += 1;
            assert!(
                l.c > 0.0 && r.c < 0.0,
                "{name}: left is left, right is right"
            );
            assert!((2.5..=5.0).contains(&lane.width), "{name}: {}", lane.width);
        }
    }
    assert!(both >= 2, "both lines in most of Ravensim's pictures");
    // Grey tarmac and a sky: no paint, no lane.
    let (w, h) = (640, 360);
    let blank: Vec<u8> = (0..h)
        .flat_map(|y| {
            let c = if y < 180 {
                [170u8, 200, 230]
            } else {
                [90, 90, 92]
            };
            (0..w).flat_map(move |_| c)
        })
        .collect();
    let lane = lane_of(&mut model, &blank, w, h);
    assert!(lane.left.is_none() && lane.right.is_none(), "{lane:?}");
}

#[cfg(feature = "coreml")]
#[test]
fn core_ml_answers_as_the_cpu_does() {
    let Some(path) = model_file() else { return };
    let mut cpu = Model::open(&path, Provider::Cpu).expect("cpu");
    let mut coreml = Model::open(&path, Provider::CoreMl).expect("coreml");
    let dir = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../aav_lane_finder_rs/crates/lane-core/tests/pictures/"
    );
    let (data, w, h) = rgb(&format!("{dir}ravensim_frame_00.png"));
    let (a, b) = (
        lane_of(&mut cpu, &data, w, h),
        lane_of(&mut coreml, &data, w, h),
    );
    assert_eq!(a.left.is_some(), b.left.is_some());
    assert_eq!(a.right.is_some(), b.right.is_some());
    assert!(
        (a.offset - b.offset).abs() < 0.05,
        "{} {}",
        a.offset,
        b.offset
    );
}
