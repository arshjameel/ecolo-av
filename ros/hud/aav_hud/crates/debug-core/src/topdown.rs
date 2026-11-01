//! The seeing screen's two top-downs: the map frame seen from above, in a rectangle of
//! the window. x to the right, y up (REP-103 seen from above), so the screen's y is
//! turned over.
//!
//! [`View::fit`] shows a whole area (the NDT map, the cars on it); [`View::around`]
//! shows a square of a given width round one point (NDT's match round the car).

use map_core::geom::Point;

/// A rectangle of the window, in window points: left, top, width, height.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
}

impl Rect {
    pub fn new(x: f32, y: f32, w: f32, h: f32) -> Self {
        Self { x, y, w, h }
    }

    /// Whether a window point is inside.
    pub fn contains(&self, at: [f32; 2]) -> bool {
        at[0] >= self.x && at[0] <= self.x + self.w && at[1] >= self.y && at[1] <= self.y + self.h
    }
}

/// From the map frame (metres) to window points.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct View {
    /// The map point at the rectangle's middle.
    pub centre: Point,
    /// Window points per metre.
    pub scale: f32,
    pub rect: Rect,
}

/// Room left round a fitted area, as a share of it.
const MARGIN: f32 = 0.06;
/// The smallest area [`View::fit`] shows, metres: a lone car isn't drawn a screen wide.
const MIN_SPAN: f32 = 20.0;

impl View {
    /// The whole of `bounds` (lowest x, y, then highest x, y; metres) in `rect`, with a
    /// margin, the same scale both ways.
    pub fn fit(bounds: [f32; 4], rect: Rect) -> Self {
        let [x0, y0, x1, y1] = bounds;
        let span_x = (x1 - x0).abs().max(MIN_SPAN) * (1.0 + 2.0 * MARGIN);
        let span_y = (y1 - y0).abs().max(MIN_SPAN) * (1.0 + 2.0 * MARGIN);
        let scale = (rect.w / span_x).min(rect.h / span_y);
        Self {
            centre: Point::new(f64::from(x0 + x1) / 2.0, f64::from(y0 + y1) / 2.0),
            scale,
            rect,
        }
    }

    /// A square `width` metres across round `centre`, filling the rectangle's shorter side.
    pub fn around(centre: Point, width: f32, rect: Rect) -> Self {
        Self {
            centre,
            scale: rect.w.min(rect.h) / width.max(1e-3),
            rect,
        }
    }

    /// A map point in window points.
    pub fn to_screen(&self, p: Point) -> [f32; 2] {
        let dx = (p.x - self.centre.x) as f32 * self.scale;
        let dy = (p.y - self.centre.y) as f32 * self.scale;
        [
            self.rect.x + self.rect.w / 2.0 + dx,
            self.rect.y + self.rect.h / 2.0 - dy,
        ]
    }

    /// Whether a map point is drawn inside the rectangle.
    pub fn shows(&self, p: Point) -> bool {
        self.rect.contains(self.to_screen(p))
    }

    /// Half the shown width and height, in metres: what to look up round the centre.
    pub fn half_span(&self) -> f64 {
        f64::from(self.rect.w.max(self.rect.h) / 2.0 / self.scale.max(1e-6))
    }

    /// The points of an ellipse (map frame: centre, half-axes, the long one's heading)
    /// round its edge, in window points: `n` of them, the first repeated at the end.
    pub fn ellipse(
        &self,
        centre: [f32; 2],
        half_axes: [f32; 2],
        angle: f32,
        n: usize,
    ) -> Vec<[f32; 2]> {
        let (s, c) = angle.sin_cos();
        (0..=n)
            .map(|i| {
                let t = i as f32 / n.max(1) as f32 * std::f32::consts::TAU;
                let (a, b) = (half_axes[0] * t.cos(), half_axes[1] * t.sin());
                let p = Point::new(
                    f64::from(centre[0] + a * c - b * s),
                    f64::from(centre[1] + a * s + b * c),
                );
                self.to_screen(p)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn y_goes_up_the_screen() {
        let v = View::around(
            Point::new(10.0, 5.0),
            20.0,
            Rect::new(0.0, 0.0, 200.0, 100.0),
        );
        // 100 points for 20 m: 5 per metre; the centre in the middle.
        assert_eq!(v.scale, 5.0);
        assert_eq!(v.to_screen(Point::new(10.0, 5.0)), [100.0, 50.0]);
        assert_eq!(v.to_screen(Point::new(12.0, 6.0)), [110.0, 45.0]);
        assert!(v.shows(Point::new(10.0, 14.0)));
        assert!(!v.shows(Point::new(10.0, 16.0)));
    }

    #[test]
    fn a_fitted_map_keeps_its_shape_and_a_margin() {
        let v = View::fit([0.0, 0.0, 100.0, 50.0], Rect::new(0.0, 0.0, 400.0, 400.0));
        assert_eq!(v.centre, Point::new(50.0, 25.0));
        let left = v.to_screen(Point::new(0.0, 25.0));
        let right = v.to_screen(Point::new(100.0, 25.0));
        assert!(left[0] > 0.0 && right[0] < 400.0);
        assert!((right[0] - left[0] - 400.0 / 1.12).abs() < 0.01);
        // A single point is shown 20 m across, not infinitely close.
        let one = View::fit([3.0, 3.0, 3.0, 3.0], Rect::new(0.0, 0.0, 224.0, 224.0));
        assert!((one.scale - 10.0).abs() < 1e-4);
    }

    #[test]
    fn an_ellipse_goes_round_its_axes() {
        let v = View::around(
            Point::new(0.0, 0.0),
            10.0,
            Rect::new(0.0, 0.0, 100.0, 100.0),
        );
        let e = v.ellipse([0.0, 0.0], [2.0, 1.0], std::f32::consts::FRAC_PI_2, 4);
        assert_eq!(e.len(), 5);
        // Turned a quarter: the long axis points up the screen (map +y).
        assert!((e[0][0] - 50.0).abs() < 1e-3 && (e[0][1] - 30.0).abs() < 1e-3);
        assert!((e[1][0] - 40.0).abs() < 1e-3 && (e[1][1] - 50.0).abs() < 1e-3);
        assert!((e[0][0] - e[4][0]).abs() < 1e-3 && (e[0][1] - e[4][1]).abs() < 1e-3);
    }
}
