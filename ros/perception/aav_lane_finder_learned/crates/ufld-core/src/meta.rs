//! What the model was made for: the JSON `tools/lane_train.py` writes beside the `.onnx`.

use std::path::Path;

use serde::Deserialize;

use crate::{Error, Result};

/// The model's picture, input and answers.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Meta {
    /// px: the camera's pictures it was trained on.
    pub width: usize,
    pub height: usize,
    /// px: rows above this are cut off (sky).
    pub crop_top: usize,
    /// px: the model's input, rows `crop_top..height` scaled to this.
    pub input_w: usize,
    pub input_h: usize,
    /// px: the row anchors, in the picture's rows.
    pub rows: Vec<f64>,
    /// The columns each row's line is placed in, across the picture's width.
    pub cells: usize,
    /// The lines, in the model's order: `left2`, `left`, `right`, `right2`.
    pub lanes: Vec<String>,
    /// m ahead: the span the lines are fitted over.
    #[serde(default = "fit_x")]
    pub fit_x: [f64; 2],
}

fn fit_x() -> [f64; 2] {
    [4.0, 25.0]
}

impl Meta {
    /// Read and check the JSON text.
    pub fn from_json(text: &str) -> Result<Self> {
        let meta: Self = serde_json::from_str(text).map_err(|e| Error::Meta(e.to_string()))?;
        meta.check()?;
        Ok(meta)
    }

    /// The JSON beside a model: `models/lane-ufld.onnx` → `models/lane-ufld.json`.
    pub fn beside(model: &Path) -> Result<Self> {
        let path = model.with_extension("json");
        let text = std::fs::read_to_string(&path)
            .map_err(|e| Error::Meta(format!("{}: {e}", path.display())))?;
        Self::from_json(&text)
    }

    fn check(&self) -> Result<()> {
        let ok = self.width > 0
            && self.crop_top < self.height
            && self.input_w > 0
            && self.input_h > 0
            && self.cells > 1
            && !self.rows.is_empty()
            && self
                .rows
                .iter()
                .all(|&r| r >= self.crop_top as f64 && r < self.height as f64)
            && self.fit_x[1] > self.fit_x[0];
        if !ok {
            return Err(Error::Meta(format!("impossible sizes: {self:?}")));
        }
        self.slot("left")?;
        self.slot("right")?;
        Ok(())
    }

    /// The model's index for a line (`left`, `right`...).
    pub fn slot(&self, name: &str) -> Result<usize> {
        self.lanes
            .iter()
            .position(|l| l == name)
            .ok_or_else(|| Error::Meta(format!("no line called {name} in {:?}", self.lanes)))
    }

    /// How many numbers each answer has: loc (cells x rows x lanes), exist (2 x rows x lanes).
    pub fn sizes(&self) -> (usize, usize) {
        let rl = self.rows.len() * self.lanes.len();
        (self.cells * rl, 2 * rl)
    }
}

#[cfg(test)]
pub(crate) fn example() -> Meta {
    Meta {
        width: 640,
        height: 360,
        crop_top: 168,
        input_w: 480,
        input_h: 144,
        rows: (0..42).map(|k| 192.0 + 4.0 * k as f64).collect(),
        cells: 100,
        lanes: ["left2", "left", "right", "right2"]
            .map(String::from)
            .to_vec(),
        fit_x: [4.0, 25.0],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_the_trainers_json() {
        let text = r#"{"width": 640, "height": 360, "crop_top": 168, "input_w": 480,
            "input_h": 144, "rows": [192, 196], "cells": 100,
            "lanes": ["left2", "left", "right", "right2"], "mean": [0.5], "scores": {}}"#;
        let meta = Meta::from_json(text).unwrap();
        assert_eq!(meta.fit_x, [4.0, 25.0]);
        assert_eq!(meta.slot("right").unwrap(), 2);
        assert_eq!(meta.sizes(), (100 * 2 * 4, 2 * 2 * 4));
    }

    #[test]
    fn refuses_rows_outside_the_crop() {
        let mut m = example();
        m.rows.push(100.0);
        let text = serde_json::json!({"width": m.width, "height": m.height,
            "crop_top": m.crop_top, "input_w": m.input_w, "input_h": m.input_h,
            "rows": m.rows, "cells": m.cells, "lanes": m.lanes});
        assert!(Meta::from_json(&text.to_string()).is_err());
    }
}
