//! The Python's answers: `gen_vectors.py` ran `aav_lidar`'s `scan.py` on these clouds.
//!
//! Every range must be the same float32 as the Python's, but for a point whose angle is
//! within a hair (1e-9) of half way between two directions: `atan2`'s last bit may round
//! it either way, so there the next direction over may hold it.

use std::{fs, path::Path};

use lidar_scan::{flatten, Body, Mount, Point, ScanShape};
use serde_json::Value;

fn number(v: &Value) -> f64 {
    match v {
        Value::String(s) if s == "inf" => f64::INFINITY,
        Value::String(s) if s == "-inf" => f64::NEG_INFINITY,
        Value::String(_) => f64::NAN,
        v => v.as_f64().expect("a number"),
    }
}

fn numbers(v: &Value) -> Vec<f64> {
    v.as_array().expect("a list").iter().map(number).collect()
}

fn three(v: &Value) -> [f64; 3] {
    let n = numbers(v);
    [n[0], n[1], n[2]]
}

#[test]
fn the_same_ranges_as_the_python() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/vectors.json");
    let all: Value =
        serde_json::from_str(&fs::read_to_string(path).expect("vectors.json")).expect("JSON");
    let cases = all["cases"].as_array().expect("cases");
    assert!(cases.len() >= 7);
    let (mut ranges, mut off_by_one) = (0, 0);
    for case in cases {
        let name = case["name"].as_str().expect("name");
        let s = numbers(&case["shape"]);
        let shape = ScanShape {
            min_height: s[0],
            max_height: s[1],
            angle_min: s[2],
            angle_max: s[3],
            angle_increment: s[4],
            range_min: s[5],
            range_max: s[6],
            body: Body {
                back: s[7],
                front: s[8],
                half_width: s[9],
            },
        };
        let mount = Mount::new(three(&case["xyz"]), three(&case["rpy"]));
        // float32 values, as the lidar sends them.
        let points: Vec<Point> = numbers(&case["points"])
            .into_iter()
            .map(|v| f64::from(v as f32))
            .collect::<Vec<_>>()
            .as_chunks::<3>()
            .0
            .iter()
            .map(|p| mount.to_base(&Point::new(p[0], p[1], p[2])))
            .collect();
        let want: Vec<f32> = numbers(&case["ranges"])
            .into_iter()
            .map(|r| r as f32)
            .collect();
        let got: Vec<f32> = flatten(&points, &shape)
            .into_iter()
            .map(|r| r as f32)
            .collect();
        assert_eq!(got.len(), want.len(), "{name}");
        for (i, (g, w)) in got.iter().zip(&want).enumerate() {
            ranges += 1;
            if g == w {
                continue;
            }
            // A point on a boundary: the neighbour holds what this one lost.
            let n = want.len();
            let near = [(i + 1) % n, (i + n - 1) % n];
            assert!(
                near.iter().any(|&j| got[j] == *w || want[j] == *g),
                "{name}: range {i} is {g}, the Python's {w}"
            );
            off_by_one += 1;
        }
    }
    // A boundary point is rare: at most one range in a thousand.
    assert!(
        off_by_one * 1000 <= ranges,
        "{off_by_one} of {ranges} differ"
    );
    println!("{ranges} ranges, {off_by_one} one direction over");
}
