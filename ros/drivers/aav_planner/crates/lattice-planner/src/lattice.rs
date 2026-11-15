//! The lattice primitives file (`tools/lattice_primitives.py`, Nav2's format): the car's
//! moves from each heading, as Nav2's `LatticeMotionTable` reads them.

use std::f64::consts::TAU;

use serde_json::Value;

use crate::curves::wrap;

/// One move: from a heading, a short drivable curve to another lattice point.
#[derive(Debug, Clone, PartialEq)]
pub struct Primitive {
    /// The heading it starts from (an index into `Lattice::headings`).
    pub start_heading: usize,
    /// The heading it ends on.
    pub end_heading: usize,
    pub left_turn: bool,
    /// m driven.
    pub length: f64,
    /// m of it on an arc (0: a straight move).
    pub arc_length: f64,
    /// The poses along it, from the start (not included) to the end, metres and radians,
    /// relative to the start point.
    pub poses: Vec<(f64, f64, f64)>,
}

impl Primitive {
    /// The end point, m from the start.
    pub fn end(&self) -> (f64, f64) {
        self.poses.last().map_or((0.0, 0.0), |p| (p.0, p.1))
    }
}

/// The whole file.
#[derive(Debug, Clone, PartialEq)]
pub struct Lattice {
    /// m: the turning radius the moves were made for.
    pub turning_radius: f64,
    /// m: the grid the moves land on.
    pub resolution: f64,
    /// rad, in [0, 2 pi): each heading's angle.
    pub headings: Vec<f64>,
    /// The moves from each heading, `moves[h]`.
    pub moves: Vec<Vec<Primitive>>,
}

/// What can be wrong with a primitives file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LatticeError(pub String);

impl std::fmt::Display for LatticeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "lattice file: {}", self.0)
    }
}

impl std::error::Error for LatticeError {}

fn number(v: &Value, key: &str) -> Result<f64, LatticeError> {
    v[key]
        .as_f64()
        .ok_or_else(|| LatticeError(format!("{key}: a number is needed")))
}

impl Lattice {
    /// Read the file's text.
    pub fn parse(text: &str) -> Result<Self, LatticeError> {
        let json: Value =
            serde_json::from_str(text).map_err(|e| LatticeError(format!("not JSON: {e}")))?;
        let meta = &json["lattice_metadata"];
        let model = meta["motion_model"].as_str().unwrap_or("");
        if model != "ackermann" {
            return Err(LatticeError(format!(
                "motion_model {model:?}: only ackermann is ported"
            )));
        }
        let headings: Vec<f64> = meta["heading_angles"]
            .as_array()
            .ok_or_else(|| LatticeError("heading_angles: a list is needed".into()))?
            .iter()
            .map(|a| a.as_f64().map(|a| a.rem_euclid(TAU)))
            .collect::<Option<_>>()
            .ok_or_else(|| LatticeError("heading_angles: numbers are needed".into()))?;
        if headings.is_empty() || !headings.len().is_multiple_of(2) {
            return Err(LatticeError(
                "heading_angles: an even number of headings is needed".into(),
            ));
        }
        let mut moves = vec![Vec::new(); headings.len()];
        for p in json["primitives"]
            .as_array()
            .ok_or_else(|| LatticeError("primitives: a list is needed".into()))?
        {
            let index = |key: &str| -> Result<usize, LatticeError> {
                p[key]
                    .as_u64()
                    .map(|i| i as usize)
                    .filter(|&i| i < headings.len())
                    .ok_or_else(|| LatticeError(format!("{key}: a heading index is needed")))
            };
            let poses = p["poses"]
                .as_array()
                .ok_or_else(|| LatticeError("poses: a list is needed".into()))?
                .iter()
                .map(|q| Some((q[0].as_f64()?, q[1].as_f64()?, q[2].as_f64()?)))
                .collect::<Option<Vec<_>>>()
                .ok_or_else(|| LatticeError("poses: [x, y, yaw] each".into()))?;
            if poses.is_empty() {
                return Err(LatticeError("a primitive with no poses".into()));
            }
            let start = index("start_angle_index")?;
            moves[start].push(Primitive {
                start_heading: start,
                end_heading: index("end_angle_index")?,
                left_turn: p["left_turn"].as_bool().unwrap_or(false),
                length: number(p, "trajectory_length")?,
                arc_length: number(p, "arc_length")?,
                poses,
            });
        }
        Ok(Self {
            turning_radius: number(meta, "turning_radius")?,
            resolution: number(meta, "grid_resolution")?,
            headings,
            moves,
        })
    }

    /// Hybrid-A*'s moves as a lattice (Nav2's `HybridMotionTable::initDubin`): from each of
    /// `bins` headings, straight on, and full lock left and right, each one chord just over
    /// a cell's diagonal long (so every move leaves its cell), at `radius` m on `resolution`
    /// m cells. The turn is a whole number of heading bins. With `interpolation` (Nav2's
    /// `allow_primitive_interpolation`), also the gentler turns of fewer bins, each with the
    /// same chord. Reversing (Reeds-Shepp) needs nothing more: the search drives the facing
    /// heading's moves backwards, which are Nav2's `REVERSE`, `REV_LEFT` and `REV_RIGHT`.
    ///
    /// Each move has one pose, its end: Hybrid-A* checks the car only where a move ends.
    pub fn hybrid(
        radius: f64,
        bins: usize,
        resolution: f64,
        interpolation: bool,
    ) -> Result<Self, LatticeError> {
        if bins < 2 || !bins.is_multiple_of(2) {
            return Err(LatticeError(format!(
                "angle_quantization_bins {bins}: an even number of 2 or more is needed"
            )));
        }
        if resolution.is_nan() || resolution <= 0.0 {
            return Err(LatticeError(format!(
                "resolution {resolution}: more than 0 is needed"
            )));
        }
        // Nav2 raises a radius under one cell to one cell.
        let r = (radius / resolution).max(1.0);
        let bin = TAU / bins as f64;
        // The smallest turn whose chord leaves the cell, rounded up to whole bins.
        let least = 2.0 * (std::f64::consts::SQRT_2 / (2.0 * r)).min(1.0).asin();
        let increments = if least < bin {
            1
        } else {
            (least / bin).ceil() as usize
        };
        let angle = increments as f64 * bin;
        let dx = r * angle.sin();
        let dy = r - r * angle.cos();
        let chord = dx.hypot(dy);
        // (bins turned, x, y, cells driven): left; right mirrors it.
        let mut turns = vec![(increments, dx, dy, r * angle)];
        if interpolation {
            for i in 1..increments {
                let a = i as f64 * bin;
                let rn = chord / (2.0 * (a / 2.0).sin());
                turns.push((i, rn * a.sin(), rn - rn * a.cos(), rn * a));
            }
        }
        let headings: Vec<f64> = (0..bins).map(|h| h as f64 * bin).collect();
        let moves = headings
            .iter()
            .enumerate()
            .map(|(h, &theta)| {
                let (s, c) = theta.sin_cos();
                let make = |x: f64, y: f64, turn: isize, cells: f64| {
                    let end = (h as isize + turn).rem_euclid(bins as isize) as usize;
                    Primitive {
                        start_heading: h,
                        end_heading: end,
                        left_turn: turn > 0,
                        length: cells * resolution,
                        arc_length: if turn == 0 { 0.0 } else { cells * resolution },
                        poses: vec![(
                            (x * c - y * s) * resolution,
                            (x * s + y * c) * resolution,
                            headings[end],
                        )],
                    }
                };
                let mut out = vec![make(chord, 0.0, 0, chord)];
                for &(i, x, y, cells) in &turns {
                    out.push(make(x, y, i as isize, cells));
                    out.push(make(x, -y, -(i as isize), cells));
                }
                out
            })
            .collect();
        Ok(Self {
            turning_radius: r * resolution,
            resolution,
            headings,
            moves,
        })
    }

    /// The heading index nearest an angle (Nav2's `getClosestAngularBin`).
    pub fn nearest_heading(&self, yaw: f64) -> usize {
        self.headings
            .iter()
            .enumerate()
            .min_by(|a, b| wrap(yaw - a.1).abs().total_cmp(&wrap(yaw - b.1).abs()))
            .map_or(0, |(i, _)| i)
    }

    /// The heading facing the other way.
    pub fn opposite(&self, heading: usize) -> usize {
        (heading + self.headings.len() / 2) % self.headings.len()
    }
}
