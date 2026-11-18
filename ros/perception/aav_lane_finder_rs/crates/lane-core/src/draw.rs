//! The debug picture: the bird's-eye view with the paint, the windows and the lines on it.

use crate::birds_eye::Grid;
use crate::font::glyph;
use crate::lines::{centre_y, Lane, View};

/// A picture, packed bgr8.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Drawing {
    pub width: usize,
    pub height: usize,
    pub bgr: Vec<u8>,
}

type Bgr = [u8; 3];

impl Drawing {
    fn put(&mut self, x: i64, y: i64, colour: Bgr) {
        if x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height {
            let i = 3 * (y as usize * self.width + x as usize);
            self.bgr[i..i + 3].copy_from_slice(&colour);
        }
    }

    /// A rectangle's outline, one pixel wide, corners included (cv2.rectangle).
    fn rectangle(&mut self, (x0, y0): (i64, i64), (x1, y1): (i64, i64), colour: Bgr) {
        let (x0, x1) = (x0.min(x1), x0.max(x1));
        let (y0, y1) = (y0.min(y1), y0.max(y1));
        for x in x0..=x1 {
            self.put(x, y0, colour);
            self.put(x, y1, colour);
        }
        for y in y0..=y1 {
            self.put(x0, y, colour);
            self.put(x1, y, colour);
        }
    }

    /// A line `thick` pixels wide: a dot every pixel along it (no anti-aliasing).
    fn line(&mut self, (x0, y0): (i64, i64), (x1, y1): (i64, i64), colour: Bgr, thick: i64) {
        let steps = (x1 - x0).abs().max((y1 - y0).abs()).max(1);
        let r = thick / 2;
        for k in 0..=steps {
            let x = x0 + ((x1 - x0) * k + steps / 2).div_euclid(steps);
            let y = y0 + ((y1 - y0) * k + steps / 2).div_euclid(steps);
            for dy in -r..=r {
                for dx in -r..=r {
                    if thick == 1 || dx * dx + dy * dy <= r * r {
                        self.put(x + dx, y + dy, colour);
                    }
                }
            }
        }
    }

    fn polyline(&mut self, points: &[(i64, i64)], colour: Bgr, thick: i64) {
        for pair in points.windows(2) {
            self.line(pair[0], pair[1], colour, thick);
        }
    }

    /// Words, 5 x 7 pixels a character, the baseline at y.
    fn text(&mut self, words: &str, (x, y): (i64, i64), colour: Bgr) {
        for (i, c) in words.chars().enumerate() {
            let left = x + 6 * i as i64;
            for (row, bits) in glyph(c).into_iter().enumerate() {
                for col in 0..5 {
                    if bits & (0x10 >> col) != 0 {
                        self.put(left + col, y - 7 + row as i64, colour);
                    }
                }
            }
        }
    }
}

/// The bird's-eye view with the paint (yellow), the windows (green when they found
/// paint), the lines (left blue, right red), the car's centre line (white) and the lane's
/// (green), `scale` times the view's size. aav_lane_finder's `draw`, without its
/// anti-aliasing, in a smaller font.
pub fn draw(view: &View, lane: &Lane, grid: &Grid, scale: usize) -> Drawing {
    let (rows, cols, s) = (view.bev.rows, view.bev.cols, scale.max(1));
    let mut d = Drawing {
        width: cols * s,
        height: rows * s,
        bgr: vec![0; 3 * rows * cols * s * s],
    };
    for (i, (p, &paint)) in view.bev.pixels.iter().zip(&view.mask.on).enumerate() {
        let colour = if paint {
            [0, 220, 255]
        } else {
            p.map(|v| (f64::from(v) * 0.6) as u8)
        };
        let (r, c) = (i / cols, i % cols);
        for y in r * s..(r + 1) * s {
            for x in c * s..(c + 1) * s {
                d.put(x as i64, y as i64, colour);
            }
        }
    }
    let si = s as i64;
    for w in &view.windows {
        let colour = if w.hit { [0, 200, 0] } else { [90, 90, 90] };
        d.rectangle(
            ((w.left * s as f64) as i64, w.top as i64 * si),
            ((w.right * s as f64) as i64, w.bottom as i64 * si - 1),
            colour,
        );
    }
    let xs: Vec<f64> = (0..40)
        .map(|k| grid.x_min + (grid.x_max - grid.x_min) * f64::from(k) / 39.0)
        .collect();
    let points = |f: &dyn Fn(f64) -> f64| -> Vec<(i64, i64)> {
        xs.iter()
            .map(|&x| {
                let u = (grid.col_of_y(f(x)) + 0.5) * s as f64;
                let v = (grid.row_of_x(x) + 0.5) * s as f64;
                (u.round_ties_even() as i64, v.round_ties_even() as i64)
            })
            .collect()
    };
    if let Some(l) = &lane.left {
        d.polyline(&points(&|x| l.y(x)), [255, 120, 0], 2);
    }
    if let Some(r) = &lane.right {
        d.polyline(&points(&|x| r.y(x)), [0, 0, 255], 2);
    }
    d.polyline(&points(&|_| 0.0), [255, 255, 255], 1);
    let seen = lane.left.is_some() || lane.right.is_some();
    if seen {
        let (l, r, w) = (lane.left.as_ref(), lane.right.as_ref(), lane.width);
        d.polyline(
            &points(&|x| centre_y(l, r, w, x).unwrap_or(0.0)),
            [0, 255, 0],
            1,
        );
    }
    let words = if seen {
        format!("offset {:+.2} m  width {:.2} m", lane.offset, lane.width)
    } else {
        "no lane".to_string()
    };
    d.text(&words, (6, 18), [255, 255, 255]);
    d
}
