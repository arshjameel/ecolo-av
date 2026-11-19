//! The Python's answers (gen_vectors.py) on the same pictures.
//!
//! What must agree, and how closely: what matters for driving, not the last bit.
//!
//! | What | Allowed |
//! |---|---|
//! | each line found or not, and the lane's curve direction | the same |
//! | the offset, the lane's width, the finder's remembered width | 2 cm |
//! | each line's heading at the look-ahead, the steering hint | 0.5 degrees |
//! | each line's curvature at the look-ahead | 0.001 1/m (a 1 km radius) |
//! | each line's confidence | 0.02 |
//! | the paint mask's pixels, the bird's-eye view's grey sum | 0.5 % |
//! | where a point lands in the picture | 1e-6 px; seen or not the same |
//!
//! In practice the warp is OpenCV's to the grey level, so the answers agree far closer;
//! the test prints the largest differences.

use std::path::{Path, PathBuf};

use lane_core::{
    project, BirdsEye, Distortion, Encoding, Grid, Lane, LaneFinder, Lens, Line, Mount, Paint,
    Picture, Settings,
};
use serde_json::Value;

const OFFSET: f64 = 0.02;
const ANGLE: f64 = 0.5 * std::f64::consts::PI / 180.0;
const CURVATURE: f64 = 0.001;
const CONFIDENCE: f64 = 0.02;
const PIXELS: f64 = 0.005;

fn here() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests")
}

fn num(v: &Value) -> f64 {
    v.as_f64().unwrap_or_else(|| panic!("not a number: {v}"))
}

fn nums<const N: usize>(v: &Value) -> [f64; N] {
    let a = v.as_array().expect("a list");
    std::array::from_fn(|i| num(&a[i]))
}

fn lens(v: &Value) -> Lens {
    let d: Vec<f64> = v["distortion"]
        .as_array()
        .unwrap()
        .iter()
        .map(num)
        .collect();
    Lens {
        width: v["width"].as_u64().unwrap() as usize,
        height: v["height"].as_u64().unwrap() as usize,
        fx: num(&v["fx"]),
        fy: num(&v["fy"]),
        cx: num(&v["cx"]),
        cy: num(&v["cy"]),
        distortion: if d.iter().any(|&k| k != 0.0) {
            Some(Distortion::from_coefficients(&d).unwrap())
        } else {
            None
        },
    }
}

fn mount(v: &Value) -> Mount {
    Mount {
        xyz: nums(&v["xyz"]),
        rpy: nums(&v["rpy"]),
    }
}

fn grid(v: &Value) -> Grid {
    Grid {
        x_min: num(&v["x_min"]),
        x_max: num(&v["x_max"]),
        y_max: num(&v["y_max"]),
        resolution: num(&v["resolution"]),
    }
}

fn settings(v: &Value) -> Settings {
    Settings {
        paint_contrast: num(&v["paint_contrast"]),
        paint_width: num(&v["paint_width"]),
        windows: v["windows"].as_u64().unwrap() as usize,
        window_margin: num(&v["window_margin"]),
        min_pixels: v["min_pixels"].as_u64().unwrap() as usize,
        min_span: num(&v["min_span"]),
        fit_scale: num(&v["fit_scale"]),
        min_confidence: num(&v["min_confidence"]),
        lane_width: num(&v["lane_width"]),
        min_lane_width: num(&v["min_lane_width"]),
        max_lane_width: num(&v["max_lane_width"]),
        lookahead: num(&v["lookahead"]),
        wheelbase: num(&v["wheelbase"]),
        yellow_ratio: v["yellow_ratio"].as_f64().unwrap_or(0.75),
    }
}

/// A PNG as packed bgr8, as cv2.imread gives it.
fn read_png(name: &str) -> (Vec<u8>, usize, usize) {
    let file = std::fs::File::open(here().join("pictures").join(name)).expect(name);
    let mut reader = png::Decoder::new(std::io::BufReader::new(file))
        .read_info()
        .expect("a PNG");
    let mut buf = vec![0; reader.output_buffer_size().expect("a size")];
    let info = reader.next_frame(&mut buf).expect("a frame");
    assert_eq!(info.bit_depth, png::BitDepth::Eight, "{name}");
    let (w, h) = (info.width as usize, info.height as usize);
    let bgr = match info.color_type {
        png::ColorType::Rgb => buf[..w * h * 3]
            .chunks(3)
            .flat_map(|p| [p[2], p[1], p[0]])
            .collect(),
        png::ColorType::Grayscale => buf[..w * h].iter().flat_map(|&g| [g, g, g]).collect(),
        other => panic!("{name}: {other:?}"),
    };
    (bgr, w, h)
}

/// gen_vectors.py's `noisy`: the same generator, byte for byte.
fn noisy(bytes: &mut [u8], seed: u64) {
    let (amplitude, mut x) = (20i32, seed);
    for b in bytes.iter_mut() {
        x = (x * 1_103_515_245 + 12_345) & 0x7FFF_FFFF;
        let n = ((x >> 16) % 41) as i32 - amplitude;
        *b = (i32::from(*b) + n).clamp(0, 255) as u8;
    }
}

#[derive(Default)]
struct Worst {
    offset: f64,
    angle: f64,
    curvature: f64,
    confidence: f64,
    pixels: f64,
    uv: f64,
}

fn line(v: &Value) -> Option<Line> {
    (!v.is_null()).then(|| Line {
        a: num(&v["a"]),
        b: num(&v["b"]),
        c: num(&v["c"]),
        confidence: num(&v["confidence"]),
    })
}

fn close(worst: &mut f64, got: f64, want: f64, allowed: f64, what: &str) {
    let d = (got - want).abs();
    *worst = worst.max(d);
    assert!(d <= allowed, "{what}: {got} against the Python's {want}");
}

fn compare(got: &Lane, want: &Value, s: &Settings, worst: &mut Worst, at: &str) {
    let x = s.lookahead;
    for (side, mine) in [("left", &got.left), ("right", &got.right)] {
        let theirs = line(&want[side]);
        assert_eq!(
            mine.is_some(),
            theirs.is_some(),
            "{at}: the {side} line found or not"
        );
        if let (Some(m), Some(t)) = (mine, theirs) {
            let what = format!("{at}: the {side} line");
            close(&mut worst.offset, m.y(x), t.y(x), OFFSET, &what);
            close(
                &mut worst.angle,
                m.slope(x).atan(),
                t.slope(x).atan(),
                ANGLE,
                &what,
            );
            close(
                &mut worst.curvature,
                m.curvature(x),
                t.curvature(x),
                CURVATURE,
                &what,
            );
        }
    }
    let at = |what: &str| format!("{at}: {what}");
    close(
        &mut worst.offset,
        got.offset,
        num(&want["offset"]),
        OFFSET,
        &at("offset"),
    );
    close(
        &mut worst.offset,
        got.width,
        num(&want["width"]),
        OFFSET,
        &at("width"),
    );
    close(
        &mut worst.angle,
        got.steering_hint,
        num(&want["steering_hint"]),
        ANGLE,
        &at("steering hint"),
    );
    for (mine, key) in [
        (got.left_confidence, "left_confidence"),
        (got.right_confidence, "right_confidence"),
    ] {
        close(
            &mut worst.confidence,
            mine,
            num(&want[key]),
            CONFIDENCE,
            &at(key),
        );
    }
    for (mine, key) in [
        (got.left_radius, "left_radius"),
        (got.right_radius, "right_radius"),
        (got.mean_radius, "mean_radius"),
    ] {
        close(
            &mut worst.curvature,
            1.0 / mine,
            1.0 / num(&want[key]),
            CURVATURE,
            &at(key),
        );
    }
    for (mine, key) in [
        (got.left_paint, "left_paint"),
        (got.right_paint, "right_paint"),
    ] {
        let want = want[key]
            .as_u64()
            .map_or(Paint::Unknown, |c| Paint::from_code(c as u8));
        assert_eq!(mine, want, "{}", at(key));
    }
    assert_eq!(
        got.curve_direction,
        num(&want["curve_direction"]),
        "{}",
        at("curve")
    );
}

fn vectors() -> Value {
    let text = std::fs::read_to_string(here().join("vectors.json")).expect("vectors.json");
    serde_json::from_str(&text).expect("JSON")
}

#[test]
fn the_same_lanes_as_the_python() {
    let all = vectors();
    let mut worst = Worst::default();
    let mut frames = 0;
    for case in all["cases"].as_array().unwrap() {
        let name = case["name"].as_str().unwrap();
        let s = settings(&case["settings"]);
        let view = BirdsEye::new(
            lens(&case["lens"]),
            mount(&case["mount"]),
            grid(&case["grid"]),
        );
        match case["nearest_seen"].as_f64() {
            Some(want) => assert!((view.nearest_seen().unwrap() - want).abs() < 1e-9, "{name}"),
            None => assert!(view.nearest_seen().is_none(), "{name}"),
        }
        let mut finder = LaneFinder::new(view, s);
        for (k, frame) in case["frames"].as_array().unwrap().iter().enumerate() {
            let at = format!("{name}, frame {k}");
            let (mut bgr, w, h) = read_png(frame["picture"].as_str().unwrap());
            if let Some(seed) = frame["noise"].as_u64().filter(|&s| s != 0) {
                noisy(&mut bgr, seed);
            }
            let picture = Picture::packed(&bgr, w, h, Encoding::Bgr8).unwrap();
            let lane = finder.find(&picture).expect(&at);
            let last = finder.last().unwrap();
            for (mine, key) in [
                (last.mask.count() as f64, "mask"),
                (last.bev.grey().map(f64::from).sum::<f64>(), "bev_sum"),
            ] {
                let want = num(&frame[key]);
                let d = (mine - want).abs() / want.max(1.0);
                worst.pixels = worst.pixels.max(d);
                assert!(
                    d <= PIXELS,
                    "{at}: {key} {mine} against the Python's {want}"
                );
            }
            compare(&lane, &frame["lane"], &s, &mut worst, &at);
            close(
                &mut worst.offset,
                finder.width,
                num(&frame["width_after"]),
                OFFSET,
                &format!("{at}: the width remembered"),
            );
            frames += 1;
        }
    }
    for p in all["project"].as_array().unwrap() {
        let points: Vec<[f64; 3]> = p["points"].as_array().unwrap().iter().map(nums).collect();
        let got = project(&points, &lens(&p["lens"]), &mount(&p["mount"]));
        for (mine, want) in got.iter().zip(p["uv"].as_array().unwrap()) {
            match (mine, want.is_null()) {
                (Some(uv), false) => {
                    let want: [f64; 2] = nums(want);
                    for (m, t) in uv.iter().zip(want) {
                        close(&mut worst.uv, *m, t, 1e-6, "a point in the picture");
                    }
                }
                (None, true) => {}
                _ => panic!("seen or not differs: {mine:?} against {want}"),
            }
        }
    }
    println!(
        "{frames} frames; the largest differences: {:.2e} m, {:.2e} rad, {:.2e} 1/m, \
         confidence {:.2e}, pixels {:.2e} (a share), {:.2e} px",
        worst.offset, worst.angle, worst.curvature, worst.confidence, worst.pixels, worst.uv
    );
    assert!(frames >= 40, "only {frames} frames");
}
