//! The crop and scale give the numbers Pillow gave the model in training
//! (`gen_pillow.py` writes `pillow.json`): every pixel's sum and 64 pixels, exactly.

use lane_core::{Encoding, Picture};
use ufld_core::{input, Meta};

fn meta() -> Meta {
    Meta::from_json(
        r#"{"width": 640, "height": 360, "crop_top": 168, "input_w": 480, "input_h": 144,
        "rows": [192, 356], "cells": 100, "lanes": ["left2", "left", "right", "right2"]}"#,
    )
    .unwrap()
}

/// `gen_pillow.py`'s `picture`.
fn picture(w: usize, h: usize) -> Vec<u8> {
    (0..h)
        .flat_map(|y| {
            (0..w).flat_map(move |x| {
                [
                    ((x * 7 + y * 3) % 256) as u8,
                    ((x * x + y * 5) % 256) as u8,
                    ((x ^ y) % 256) as u8,
                ]
            })
        })
        .collect()
}

#[test]
fn the_same_numbers_as_pillow() {
    let want: serde_json::Value =
        serde_json::from_str(include_str!("pillow.json")).expect("pillow.json");
    let meta = meta();
    let plane = meta.input_w * meta.input_h;
    for (w, h) in [(640, 360), (1280, 720)] {
        let data = picture(w, h);
        let p = Picture::packed(&data, w, h, Encoding::Rgb8).unwrap();
        let x = input(&p, &meta).unwrap();
        let level = |c: usize, px: usize, py: usize| {
            (x[c * plane + py * meta.input_w + px] * 255.0).round() as i64
        };
        let case = &want[format!("{w}x{h}")];
        for (c, sum) in case["sums"].as_array().unwrap().iter().enumerate() {
            let ours: i64 = (0..plane)
                .map(|i| level(c, i % meta.input_w, i / meta.input_w))
                .sum();
            assert_eq!(ours, sum.as_i64().unwrap(), "{w}x{h} channel {c}'s sum");
        }
        for px in case["pixels"].as_array().unwrap() {
            let v: Vec<i64> = px
                .as_array()
                .unwrap()
                .iter()
                .map(|n| n.as_i64().unwrap())
                .collect();
            let (x0, y0) = (v[0] as usize, v[1] as usize);
            for c in 0..3 {
                assert_eq!(
                    level(c, x0, y0),
                    v[2 + c],
                    "{w}x{h} ({x0}, {y0}) channel {c}"
                );
            }
        }
    }
}
