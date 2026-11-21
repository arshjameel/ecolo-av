//! Lane lines in the bird's-eye view: find the paint, follow each line, say where the car is.
//!
//! The offset is + when the car is left of the lane's centre (the 2025-26 lane follower
//! steered right by it, back to the centre). It is measured `lookahead` metres ahead of
//! base_link, because a PID on the offset at the rear axle weaves; a few metres ahead,
//! the offset also says which way the car is pointing.

use nalgebra::{DMatrix, DVector};

use crate::birds_eye::{BirdsEye, Grid};
use crate::picture::{Bev, Picture};
use crate::Result;

/// m: what a straight line, or a line not seen, reports as its radius.
pub const MAX_RADIUS: f64 = 9999.0;
/// m: a bigger radius counts as straight (curve_direction 0).
pub const STRAIGHT: f64 = 500.0;

/// What the finder looks for. The node makes each one a parameter of the same name.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// Grey levels the paint is brighter than the road, both sides.
    pub paint_contrast: f64,
    /// m: the widest paint it looks for.
    pub paint_width: f64,
    /// Sliding windows per line, near to far.
    pub windows: usize,
    /// m either side of a window's centre.
    pub window_margin: f64,
    /// Paint pixels for a window to count.
    pub min_pixels: usize,
    /// m: a line must be seen over this much road.
    pub min_span: f64,
    /// m: the fit's scatter that halves (about) the confidence.
    pub fit_scale: f64,
    /// Below this, a line isn't detected.
    pub min_confidence: f64,
    /// m: assumed until both lines are seen.
    pub lane_width: f64,
    /// m: two "lines" closer than this are one line.
    pub min_lane_width: f64,
    /// m
    pub max_lane_width: f64,
    /// m ahead of base_link: where the offset is measured.
    pub lookahead: f64,
    /// m, for steering_hint (vehicle.yaml).
    pub wheelbase: f64,
    /// Paint is yellow when its blue is below this share of the lower of its red and green.
    pub yellow_ratio: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            paint_contrast: 40.0,
            paint_width: 0.30,
            windows: 12,
            window_margin: 0.5,
            min_pixels: 8,
            min_span: 2.0,
            fit_scale: 0.15,
            min_confidence: 0.3,
            lane_width: 3.5,
            min_lane_width: 2.5,
            max_lane_width: 5.0,
            lookahead: 3.0,
            wheelbase: 1.57,
            yellow_ratio: 0.75,
        }
    }
}

/// A line's paint (TODO 264): the costmap prices a yellow centre line apart from a white
/// edge line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Paint {
    /// Not detected, or no pixels to tell.
    #[default]
    Unknown,
    White,
    Yellow,
}

impl Paint {
    /// LaneGeometry's `PAINT_*` constant.
    pub fn code(self) -> u8 {
        match self {
            Self::Unknown => 0,
            Self::White => 1,
            Self::Yellow => 2,
        }
    }

    /// The paint of a `PAINT_*` code; an unknown code is Unknown.
    pub fn from_code(code: u8) -> Self {
        match code {
            1 => Self::White,
            2 => Self::Yellow,
            _ => Self::Unknown,
        }
    }
}

/// A line's paint from its pixels (blue, green, red): yellow paint is red and green with
/// little blue; white has as much blue as the others. A grey picture's lines are white.
pub fn paint_of<'a>(pixels: impl IntoIterator<Item = &'a [u8; 3]>, s: &Settings) -> Paint {
    let (mut n, mut blue, mut red_green) = (0usize, 0.0, 0.0);
    for p in pixels {
        n += 1;
        blue += f64::from(p[0]);
        red_green += f64::from(p[1].min(p[2]));
    }
    match n {
        0 => Paint::Unknown,
        _ if blue < s.yellow_ratio * red_green => Paint::Yellow,
        _ => Paint::White,
    }
}

/// One lane line: y = a x² + b x + c in base_link (m), and the finder's confidence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub confidence: f64,
}

impl Line {
    pub fn y(&self, x: f64) -> f64 {
        (self.a * x + self.b) * x + self.c
    }

    pub fn slope(&self, x: f64) -> f64 {
        2.0 * self.a * x + self.b
    }

    /// 1 / radius (1/m) at x: + when the line bends left.
    pub fn curvature(&self, x: f64) -> f64 {
        2.0 * self.a / (1.0 + self.slope(x).powi(2)).powf(1.5)
    }
}

/// What the finder reports, in LaneGeometry's units and signs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lane {
    /// None: not detected.
    pub left: Option<Line>,
    pub right: Option<Line>,
    /// 0 to 1, also for a line found but not trusted.
    pub left_confidence: f64,
    pub right_confidence: f64,
    /// m: the car from the lane's centre, + = left of it.
    pub offset: f64,
    /// m
    pub width: f64,
    /// m; MAX_RADIUS when straight or not seen.
    pub left_radius: f64,
    pub right_radius: f64,
    pub mean_radius: f64,
    /// +1 bends left, -1 right, 0 straight (or no lane).
    pub curve_direction: f64,
    /// rad, + = left: pure pursuit to the lane's centre.
    pub steering_hint: f64,
    /// Each line's paint; Unknown when not detected.
    pub left_paint: Paint,
    pub right_paint: Paint,
}

/// One sliding window, in bird's-eye pixels, and whether it found paint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Window {
    pub top: usize,
    pub bottom: usize,
    pub left: f64,
    pub right: f64,
    pub hit: bool,
}

/// The paint found: one flag per bird's-eye pixel, row by row.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Mask {
    pub rows: usize,
    pub cols: usize,
    pub on: Vec<bool>,
}

impl Mask {
    /// The paint's (row, column)s, row by row, as numpy's `nonzero`.
    pub fn pixels(&self) -> Vec<(usize, usize)> {
        let cols = self.cols.max(1);
        self.on
            .iter()
            .enumerate()
            .filter(|(_, &on)| on)
            .map(|(i, _)| (i / cols, i % cols))
            .collect()
    }

    pub fn count(&self) -> usize {
        self.on.iter().filter(|&&on| on).count()
    }
}

/// The last picture's working, for the debug picture.
#[derive(Debug, Clone)]
pub struct View {
    pub bev: Bev,
    pub mask: Mask,
    pub windows: Vec<Window>,
}

/// A mask of the paint: pixels brighter than the road on both sides, nearby.
///
/// A pixel counts when it is paint_contrast grey levels brighter than the pixels just
/// beyond paint_width / 2 on its left and on its right. A step from road to grass, or to
/// a shadow, has a dark side, so it never counts; nor do wide bright patches. Off the
/// edge counts as white, so the edges never count.
pub fn paint_mask(grey: &[u8], cols: usize, resolution: f64, s: &Settings) -> Vec<bool> {
    let w = (s.paint_width / resolution / 2.0).ceil() as usize + 1;
    let cols = cols.max(1);
    grey.chunks(cols)
        .flat_map(|row| {
            (0..row.len()).map(move |c| {
                let g = i16::from(row[c]);
                let left = c.checked_sub(w).map_or(255, |l| i16::from(row[l]));
                let right = row.get(c + w).map_or(255, |&r| i16::from(r));
                f64::from((g - left).min(g - right)) >= s.paint_contrast
            })
        })
        .collect()
}

/// numpy's `convolve(a, ones(k), 'same')`: the middle max(len, k) of the full sums.
fn box_sum(a: &[f64], k: usize) -> Vec<f64> {
    let (n, k) = (a.len(), k.max(1));
    let full: Vec<f64> = (0..n + k - 1)
        .map(|i| {
            let lo = (i + 1).saturating_sub(k);
            a[lo..=i.min(n - 1)].iter().sum()
        })
        .collect();
    let start = (n.min(k) - 1) / 2;
    full[start..start + n.max(k)].to_vec()
}

/// The columns where the left and right lines start: the paint nearest the car.
pub fn starts(pixels: &[(usize, usize)], grid: &Grid, s: &Settings) -> [Option<f64>; 2] {
    let (rows, cols) = (grid.rows(), grid.cols());
    // The car's centre line.
    let split = ((grid.col_of_y(0.0) + 0.5).round_ties_even().max(0.0) as usize).min(cols);
    let k = ((s.paint_width / grid.resolution).round_ties_even() as usize).max(1);
    [(0, split), (split, cols)].map(|(lo, hi)| {
        if hi <= lo {
            return None;
        }
        // The nearest third of the view first, then all of it, for a line that starts
        // further out (the car is turned, or the line is hidden near the car).
        [rows * 2 / 3, 0].into_iter().find_map(|nearest| {
            let mut hist = vec![0.0; hi - lo];
            for &(_, c) in pixels
                .iter()
                .filter(|(r, c)| *r >= nearest && (lo..hi).contains(c))
            {
                hist[c - lo] += 1.0;
            }
            let hist = box_sum(&hist, k);
            let best = hist.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            // The first of equals, as numpy's argmax.
            (best >= s.min_pixels as f64)
                .then(|| hist.iter().position(|&v| v == best))
                .flatten()
                .map(|i| (lo + i) as f64)
        })
    })
}

/// Climb one line from `start` with sliding windows: its paint pixels (indices into
/// `pixels`, which is row by row), and the windows.
pub fn follow(
    pixels: &[(usize, usize)],
    start: f64,
    grid: &Grid,
    s: &Settings,
) -> (Vec<usize>, Vec<Window>) {
    let n = s.windows.max(1);
    // numpy's linspace(rows, 0, n + 1), rounded half to even.
    let rows = grid.rows() as f64;
    let step = -rows / n as f64;
    let edges: Vec<usize> = (0..=n)
        .map(|k| match k {
            k if k == n => 0,
            k => (k as f64 * step + rows).round_ties_even().max(0.0) as usize,
        })
        .collect();
    let margin = s.window_margin / grid.resolution;
    let (mut centre, mut shift, mut last): (f64, f64, Option<(usize, f64)>) = (start, 0.0, None);
    let mut picked = Vec::new();
    let mut windows = Vec::with_capacity(n);
    for (k, pair) in edges.windows(2).enumerate() {
        let (bottom, top) = (pair[0], pair[1]);
        // The rows top..bottom are one run of `pixels`.
        let from = pixels.partition_point(|&(r, _)| r < top);
        let to = pixels.partition_point(|&(r, _)| r < bottom);
        let inside: Vec<usize> = (from..to)
            .filter(|&i| (pixels[i].1 as f64 - centre).abs() <= margin)
            .collect();
        let hit = inside.len() >= s.min_pixels;
        windows.push(Window {
            top,
            bottom,
            left: centre - margin,
            right: centre + margin,
            hit,
        });
        if hit {
            let sum: usize = inside.iter().map(|&i| pixels[i].1).sum();
            let now = sum as f64 / inside.len() as f64;
            if let Some((k0, before)) = last {
                shift = (now - before) / (k - k0) as f64; // the line's drift per window
            }
            last = Some((k, now));
            picked.extend(inside);
            centre = now + shift;
        } else {
            centre += shift; // carry on the way the line was going, through a gap
        }
    }
    (picked, windows)
}

/// numpy's `polyfit`: least squares on the Vandermonde matrix, its columns scaled to
/// unit length first; highest power first.
fn polyfit(x: &[f64], y: &[f64], degree: usize) -> Option<Vec<f64>> {
    let m = degree + 1;
    let mut lhs = DMatrix::from_fn(x.len(), m, |i, j| x[i].powi((degree - j) as i32));
    let scale: Vec<f64> = lhs.column_iter().map(|c| c.norm()).collect();
    for (mut column, s) in lhs.column_iter_mut().zip(&scale) {
        column /= *s;
    }
    let rhs = DVector::from_column_slice(y);
    let eps = x.len() as f64 * f64::EPSILON;
    let c = lhs.svd(true, true).solve(&rhs, eps).ok()?;
    Some(c.iter().zip(&scale).map(|(c, s)| c / s).collect())
}

/// Fit y = a x² + b x + c through one line's paint (m); None if too little was seen.
pub fn fit_line(x: &[f64], y: &[f64], hits: usize, s: &Settings) -> Option<Line> {
    let lo = x.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = x.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if hits < 2 || x.len() < 3 || hi - lo < s.min_span {
        return None;
    }
    // A short stretch can't tell a bend from noise: fit it straight.
    let (a, b, c) = if hi - lo >= 2.0 * s.min_span {
        let p = polyfit(x, y, 2)?;
        (p[0], p[1], p[2])
    } else {
        let p = polyfit(x, y, 1)?;
        (0.0, p[0], p[1])
    };
    let line = Line {
        a,
        b,
        c,
        confidence: 0.0,
    };
    let squares: f64 = x
        .iter()
        .zip(y)
        .map(|(&x, &y)| (y - line.y(x)).powi(2))
        .sum();
    let scatter = (squares / x.len() as f64).sqrt();
    let confidence = (hits as f64 / s.windows as f64) * (-(scatter / s.fit_scale).powi(2)).exp();
    Some(Line {
        confidence: confidence.min(1.0),
        ..line
    })
}

pub fn radius(curvature: f64) -> f64 {
    if curvature.abs() < 1.0 / MAX_RADIUS {
        MAX_RADIUS
    } else {
        1.0 / curvature.abs()
    }
}

/// The lane's centre at x (m, + = left), from the lines detected; None with no line.
///
/// With one line, the centre is half of `width` beside it, square to the line.
pub fn centre_y(left: Option<&Line>, right: Option<&Line>, width: f64, x: f64) -> Option<f64> {
    match (left, right) {
        (Some(l), Some(r)) => Some((l.y(x) + r.y(x)) / 2.0),
        (Some(l), None) => Some(l.y(x) - width / 2.0 * (1.0 + l.slope(x).powi(2)).sqrt()),
        (None, Some(r)) => Some(r.y(x) + width / 2.0 * (1.0 + r.slope(x).powi(2)).sqrt()),
        (None, None) => None,
    }
}

/// The car's place in the lane from the lines detected (None = not detected).
pub fn lane_from(
    left: Option<Line>,
    right: Option<Line>,
    width: f64,
    s: &Settings,
    left_confidence: f64,
    right_confidence: f64,
) -> Lane {
    let x = s.lookahead;
    let Some(centre) = centre_y(left.as_ref(), right.as_ref(), width, x) else {
        return Lane {
            left: None,
            right: None,
            left_confidence,
            right_confidence,
            offset: 0.0,
            width,
            left_radius: MAX_RADIUS,
            right_radius: MAX_RADIUS,
            mean_radius: MAX_RADIUS,
            curve_direction: 0.0,
            steering_hint: 0.0,
            left_paint: Paint::Unknown,
            right_paint: Paint::Unknown,
        };
    };
    let width = match (&left, &right) {
        (Some(l), Some(r)) => {
            let slope = (l.slope(x) + r.slope(x)) / 2.0;
            (l.y(x) - r.y(x)) / (1.0 + slope * slope).sqrt()
        }
        _ => width,
    };
    let bends: Vec<f64> = [left, right]
        .iter()
        .flatten()
        .map(|l| l.curvature(x))
        .collect();
    let mean = bends.iter().sum::<f64>() / bends.len() as f64;
    let mean_radius = radius(mean);
    let curve_direction = if mean_radius > STRAIGHT {
        0.0
    } else {
        1.0f64.copysign(mean)
    };
    // Pure pursuit: the front-wheel angle whose arc reaches the lane's centre at x.
    let reach = x.hypot(centre);
    let steering_hint = (2.0 * s.wheelbase * centre.atan2(x).sin() / reach).atan();
    Lane {
        left,
        right,
        left_confidence,
        right_confidence,
        offset: -centre,
        width,
        left_radius: left.map_or(MAX_RADIUS, |l| radius(l.curvature(x))),
        right_radius: right.map_or(MAX_RADIUS, |r| radius(r.curvature(x))),
        mean_radius,
        curve_direction,
        steering_hint,
        left_paint: Paint::Unknown,
        right_paint: Paint::Unknown,
    }
}

/// Finds the lane in camera pictures, one at a time. Remembers the lane's width.
#[derive(Debug, Clone)]
pub struct LaneFinder {
    view: BirdsEye,
    settings: Settings,
    /// m: the lane's width, smoothed over the frames that saw both lines.
    pub width: f64,
    last: Option<View>,
}

impl LaneFinder {
    pub fn new(view: BirdsEye, settings: Settings) -> Self {
        Self {
            view,
            width: settings.lane_width,
            settings,
            last: None,
        }
    }

    pub fn view(&self) -> &BirdsEye {
        &self.view
    }

    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// The last picture's working, for the debug picture.
    pub fn last(&self) -> Option<&View> {
        self.last.as_ref()
    }

    /// The lane in one picture (the lens's size).
    pub fn find(&mut self, picture: &Picture<'_>) -> Result<Lane> {
        let grid = *self.view.grid();
        let s = self.settings;
        let bev = self.view.warp(picture)?;
        let grey: Vec<u8> = bev.grey().collect();
        let on: Vec<bool> = paint_mask(&grey, bev.cols, grid.resolution, &s)
            .into_iter()
            .zip(self.view.valid())
            .map(|(paint, &seen)| paint && seen)
            .collect();
        let mask = Mask {
            rows: bev.rows,
            cols: bev.cols,
            on,
        };
        let pixels = mask.pixels();
        let mut windows = Vec::new();
        let lines = starts(&pixels, &grid, &s).map(|start| {
            let (found, ws) = follow(&pixels, start?, &grid, &s);
            let hits = ws.iter().filter(|w| w.hit).count();
            windows.extend(ws);
            let x: Vec<f64> = found
                .iter()
                .map(|&i| grid.x_of_row(pixels[i].0 as f64))
                .collect();
            let y: Vec<f64> = found
                .iter()
                .map(|&i| grid.y_of_col(pixels[i].1 as f64))
                .collect();
            let paint = paint_of(
                found
                    .iter()
                    .map(|&i| &bev.pixels[pixels[i].0 * bev.cols + pixels[i].1]),
                &s,
            );
            Some((fit_line(&x, &y, hits, &s)?, paint))
        });
        let paints = lines.map(|l| l.map_or(Paint::Unknown, |(_, p)| p));
        let lines = lines.map(|l| l.map(|(line, _)| line));
        let mut confidence = lines.map(|l| l.map_or(0.0, |l| l.confidence));
        let [mut left, mut right] = lines.map(|l| l.filter(|l| l.confidence >= s.min_confidence));
        if let (Some(l), Some(r)) = (left, right) {
            let apart = l.y(s.lookahead) - r.y(s.lookahead);
            if !(s.min_lane_width..=s.max_lane_width).contains(&apart) {
                // One "line" is something else, or both found the same line: keep the
                // surer one.
                if l.confidence >= r.confidence {
                    right = None;
                    confidence[1] = 0.0;
                } else {
                    left = None;
                    confidence[0] = 0.0;
                }
            }
        }
        let lane = Lane {
            left_paint: if left.is_some() {
                paints[0]
            } else {
                Paint::Unknown
            },
            right_paint: if right.is_some() {
                paints[1]
            } else {
                Paint::Unknown
            },
            ..lane_from(left, right, self.width, &s, confidence[0], confidence[1])
        };
        if left.is_some() && right.is_some() {
            self.width += 0.2 * (lane.width - self.width); // smooth: one frame can be off
        }
        self.last = Some(View { bev, mask, windows });
        Ok(lane)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paint_must_be_brighter_than_both_sides() {
        let row: Vec<u8> = [[95u8; 10].as_slice(), &[235; 3], &[95; 10], &[130; 10]].concat();
        let mask = paint_mask(&row, row.len(), 0.05, &Settings::default());
        assert!(mask[10..13].iter().all(|&m| m), "the paint");
        assert_eq!(
            mask.iter().filter(|&&m| m).count(),
            3,
            "not the step to grass"
        );
    }

    #[test]
    fn lane_from_lines_square_to_the_car() {
        let s = Settings::default();
        let left = Line {
            a: 0.0,
            b: 0.0,
            c: 1.25,
            confidence: 1.0,
        };
        let right = Line {
            a: 0.0,
            b: 0.0,
            c: -2.25,
            confidence: 1.0,
        };
        let lane = lane_from(Some(left), Some(right), 3.5, &s, 1.0, 1.0);
        assert!((lane.offset - 0.5).abs() < 1e-9 && (lane.width - 3.5).abs() < 1e-9);
        assert_eq!(lane.left_radius, MAX_RADIUS);
        assert_eq!(lane.right_radius, MAX_RADIUS);
    }

    #[test]
    fn a_lane_seen_at_an_angle_is_measured_square_to_it() {
        let s = Settings::default();
        let (slope, c) = (0.3f64.tan(), 1.75 / 0.3f64.cos());
        let left = Line {
            a: 0.0,
            b: slope,
            c,
            confidence: 1.0,
        };
        let right = Line {
            a: 0.0,
            b: slope,
            c: -c,
            confidence: 1.0,
        };
        let both = lane_from(Some(left), Some(right), 3.5, &s, 0.0, 0.0);
        assert!((both.width - 3.5).abs() < 1e-9);
        let one = lane_from(Some(left), None, 3.5, &s, 0.0, 0.0);
        assert!((one.offset - both.offset).abs() < 1e-9);
    }

    #[test]
    fn a_parabola_has_the_radius_of_its_bend() {
        let line = Line {
            a: 0.01,
            b: 0.0,
            c: 0.0,
            confidence: 1.0,
        };
        assert!((1.0 / line.curvature(0.0) - 50.0).abs() < 1e-9);
    }

    #[test]
    fn no_lines_no_lane() {
        let lane = lane_from(None, None, 3.5, &Settings::default(), 0.0, 0.0);
        assert_eq!(lane.offset, 0.0);
        assert_eq!(lane.mean_radius, MAX_RADIUS);
        assert_eq!(lane.curve_direction, 0.0);
    }

    #[test]
    fn box_sum_is_numpys_same_convolution() {
        // np.convolve([1, 2, 3, 4, 5], np.ones(2), 'same') = [1, 3, 5, 7, 9]
        assert_eq!(
            box_sum(&[1.0, 2.0, 3.0, 4.0, 5.0], 2),
            vec![1.0, 3.0, 5.0, 7.0, 9.0]
        );
        // np.convolve([1, 2, 3, 4, 5], np.ones(3), 'same') = [3, 6, 9, 12, 9]
        assert_eq!(
            box_sum(&[1.0, 2.0, 3.0, 4.0, 5.0], 3),
            vec![3.0, 6.0, 9.0, 12.0, 9.0]
        );
        // A shorter array than the box: numpy swaps them.
        assert_eq!(box_sum(&[1.0, 2.0], 3), vec![1.0, 3.0, 3.0]);
    }

    #[test]
    fn yellow_paint_is_told_from_white() {
        let s = Settings::default();
        let yellow = [[40u8, 200, 230]; 5];
        let white = [[235u8, 235, 235]; 5];
        let warm_white = [[200u8, 230, 240]; 5];
        assert_eq!(paint_of(&yellow, &s), Paint::Yellow);
        assert_eq!(paint_of(&white, &s), Paint::White);
        assert_eq!(paint_of(&warm_white, &s), Paint::White);
        assert_eq!(paint_of(&[], &s), Paint::Unknown);
        for p in [Paint::Unknown, Paint::White, Paint::Yellow] {
            assert_eq!(Paint::from_code(p.code()), p);
        }
    }

    #[test]
    fn a_straight_fit_through_a_straight_line() {
        let x: Vec<f64> = (0..60).map(|i| 4.0 + 0.05 * f64::from(i)).collect();
        let y: Vec<f64> = x.iter().map(|x| 0.1 * x + 1.0).collect();
        let line = fit_line(&x, &y, 12, &Settings::default()).unwrap();
        assert!(line.a == 0.0 && (line.b - 0.1).abs() < 1e-9 && (line.c - 1.0).abs() < 1e-9);
        assert!((line.confidence - 1.0).abs() < 1e-9);
    }
}
