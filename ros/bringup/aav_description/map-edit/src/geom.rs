//! The shapes the editor works with: a hand alignment, a box that may be turned, and a
//! height band. Metres and radians (REP-103); every key in the files says its unit.

use serde::{Deserialize, Serialize};

pub use map_chunks::Point;

/// A move of the whole map by hand: turn by `yaw_rad` about the map's (0, 0), then shift
/// by (`x_m`, `y_m`). Applied to a point `p`: `R(yaw) p + (x, y)`.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Align {
    /// Shift east-ish, along the map's x (m).
    pub x_m: f64,
    /// Shift along the map's y (m).
    pub y_m: f64,
    /// Turn, counter-clockwise (rad).
    pub yaw_rad: f64,
}

impl Align {
    /// No move at all.
    pub const IDENTITY: Self = Self {
        x_m: 0.0,
        y_m: 0.0,
        yaw_rad: 0.0,
    };

    /// A move from its three numbers.
    #[must_use]
    pub const fn new(x_m: f64, y_m: f64, yaw_rad: f64) -> Self {
        Self { x_m, y_m, yaw_rad }
    }

    /// True if it moves nothing.
    #[must_use]
    pub fn is_identity(&self) -> bool {
        *self == Self::IDENTITY
    }

    /// True if every number is finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.x_m.is_finite() && self.y_m.is_finite() && self.yaw_rad.is_finite()
    }

    /// Where `p` goes.
    #[must_use]
    pub fn apply(&self, p: Point) -> Point {
        let (s, c) = self.yaw_rad.sin_cos();
        Point::new(c * p.x - s * p.y + self.x_m, s * p.x + c * p.y + self.y_m)
    }

    /// Where `p` came from: the opposite of [`Align::apply`].
    #[must_use]
    pub fn invert(&self, p: Point) -> Point {
        let (s, c) = self.yaw_rad.sin_cos();
        let (dx, dy) = (p.x - self.x_m, p.y - self.y_m);
        Point::new(c * dx + s * dy, -s * dx + c * dy)
    }

    /// The same move, then a shift by (`dx`, `dy`) m.
    #[must_use]
    pub fn moved(self, dx: f64, dy: f64) -> Self {
        Self::new(self.x_m + dx, self.y_m + dy, self.yaw_rad)
    }

    /// The same move, then a turn by `by` rad about `pivot` (a point after the move): the
    /// map turns in place on the screen, not about a far-away (0, 0).
    #[must_use]
    pub fn turned_about(self, pivot: Point, by: f64) -> Self {
        self.then(&Self::new(0.0, 0.0, by).about(pivot))
    }

    /// This move, then `after`.
    #[must_use]
    pub fn then(&self, after: &Align) -> Self {
        let shift = after.apply(Point::new(self.x_m, self.y_m));
        Self::new(shift.x, shift.y, self.yaw_rad + after.yaw_rad)
    }

    /// The turn of this move (its shift ignored) about `pivot` rather than (0, 0).
    fn about(self, pivot: Point) -> Self {
        let (s, c) = self.yaw_rad.sin_cos();
        Self::new(
            pivot.x - (c * pivot.x - s * pivot.y),
            pivot.y - (s * pivot.x + c * pivot.y),
            self.yaw_rad,
        )
    }
}

/// A box on the map, maybe turned: its middle, its sides and its turn.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Region {
    /// The middle (x, y m).
    pub centre_m: [f64; 2],
    /// The sides (along its own x, along its own y; m).
    pub size_m: [f64; 2],
    /// Its turn, counter-clockwise from the map's x (rad).
    #[serde(default)]
    pub yaw_rad: f64,
}

impl Region {
    /// The box between two corners, lined up with the map.
    #[must_use]
    pub fn from_corners(a: Point, b: Point) -> Self {
        Self {
            centre_m: [(a.x + b.x) / 2.0, (a.y + b.y) / 2.0],
            size_m: [(a.x - b.x).abs(), (a.y - b.y).abs()],
            yaw_rad: 0.0,
        }
    }

    /// The middle.
    #[must_use]
    pub fn centre(&self) -> Point {
        Point::new(self.centre_m[0], self.centre_m[1])
    }

    /// A box with sides and a middle that are numbers, and sides above zero.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.centre_m
            .iter()
            .chain(&self.size_m)
            .chain([&self.yaw_rad])
            .all(|v| v.is_finite())
            && self.size_m.iter().all(|&s| s > 0.0)
    }

    /// True if `p` is inside or on the edge.
    #[must_use]
    pub fn contains(&self, p: Point) -> bool {
        let (s, c) = self.yaw_rad.sin_cos();
        let (dx, dy) = (p.x - self.centre_m[0], p.y - self.centre_m[1]);
        let (u, v) = (c * dx + s * dy, -s * dx + c * dy);
        u.abs() <= self.size_m[0] / 2.0 && v.abs() <= self.size_m[1] / 2.0
    }

    /// The four corners, counter-clockwise from the box's own lower left.
    #[must_use]
    pub fn corners(&self) -> [Point; 4] {
        let (s, c) = self.yaw_rad.sin_cos();
        let (hx, hy) = (self.size_m[0] / 2.0, self.size_m[1] / 2.0);
        [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)].map(|(u, v)| {
            Point::new(
                self.centre_m[0] + c * u - s * v,
                self.centre_m[1] + s * u + c * v,
            )
        })
    }

    /// The smallest box lined up with the map that holds it: (lower left, upper right).
    #[must_use]
    pub fn bounds(&self) -> (Point, Point) {
        bounds_of(self.corners()).unwrap_or((self.centre(), self.centre()))
    }

    /// The same box after `align` moves the map.
    #[must_use]
    pub fn aligned(&self, align: &Align) -> Self {
        let c = align.apply(self.centre());
        Self {
            centre_m: [c.x, c.y],
            size_m: self.size_m,
            yaw_rad: self.yaw_rad + align.yaw_rad,
        }
    }

    /// The box `align` moves onto this one: from the screen's frame back to the map's.
    #[must_use]
    pub fn unaligned(&self, align: &Align) -> Self {
        let c = align.invert(self.centre());
        Self {
            centre_m: [c.x, c.y],
            size_m: self.size_m,
            yaw_rad: self.yaw_rad - align.yaw_rad,
        }
    }
}

/// Heights from `min_m` to `max_m` above the map's z = 0 (the ground, for map-drive's maps).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HeightBand {
    /// The lowest (m).
    pub min_m: f64,
    /// The highest (m).
    pub max_m: f64,
}

impl HeightBand {
    /// A band from its two heights.
    #[must_use]
    pub const fn new(min_m: f64, max_m: f64) -> Self {
        Self { min_m, max_m }
    }

    /// True if `z` is in it, ends included.
    #[must_use]
    pub fn contains(&self, z: f64) -> bool {
        (self.min_m..=self.max_m).contains(&z)
    }

    /// Two numbers, the lower first.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.min_m.is_finite() && self.max_m.is_finite() && self.min_m <= self.max_m
    }
}

/// The box lined up with the map around some points; `None` if there are none.
pub fn bounds_of(points: impl IntoIterator<Item = Point>) -> Option<(Point, Point)> {
    points.into_iter().fold(None, |b, p| {
        Some(match b {
            None => (p, p),
            Some((lo, hi)) => (
                Point::new(lo.x.min(p.x), lo.y.min(p.y)),
                Point::new(hi.x.max(p.x), hi.y.max(p.y)),
            ),
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: Point, b: Point) -> bool {
        (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9
    }

    #[test]
    fn align_round_trips() {
        let a = Align::new(3.0, -2.0, 0.4);
        let p = Point::new(10.0, 5.0);
        assert!(near(a.invert(a.apply(p)), p));
        assert!(near(Align::IDENTITY.apply(p), p));
    }

    #[test]
    fn a_quarter_turn() {
        let a = Align::new(1.0, 0.0, std::f64::consts::FRAC_PI_2);
        assert!(near(a.apply(Point::new(1.0, 0.0)), Point::new(1.0, 1.0)));
    }

    #[test]
    fn then_is_one_after_the_other() {
        let a = Align::new(3.0, -2.0, 0.4);
        let b = Align::new(-1.0, 7.0, -1.1);
        let p = Point::new(4.0, 9.0);
        assert!(near(a.then(&b).apply(p), b.apply(a.apply(p))));
    }

    #[test]
    fn a_turn_about_a_point_keeps_it() {
        let a = Align::new(2.0, 1.0, 0.1);
        let pivot = Point::new(30.0, 20.0);
        let turned = a.turned_about(pivot, 0.5);
        let under = a.invert(pivot);
        assert!(near(turned.apply(under), pivot));
        assert!((turned.yaw_rad - 0.6).abs() < 1e-12);
    }

    #[test]
    fn a_turned_box_holds_its_own_points() {
        let r = Region {
            centre_m: [10.0, 10.0],
            size_m: [4.0, 1.0],
            yaw_rad: std::f64::consts::FRAC_PI_4,
        };
        assert!(r.contains(Point::new(11.0, 11.0)));
        assert!(!r.contains(Point::new(11.0, 9.0)));
        for c in r.corners() {
            assert!(r.contains(Point::new(
                c.x + (10.0 - c.x) * 1e-6,
                c.y + (10.0 - c.y) * 1e-6
            )));
        }
    }

    #[test]
    fn boxes_follow_the_map() {
        let a = Align::new(5.0, 1.0, 0.3);
        let r = Region::from_corners(Point::new(0.0, 0.0), Point::new(2.0, 4.0));
        let p = Point::new(1.5, 3.0);
        assert!(r.contains(p));
        assert!(r.aligned(&a).contains(a.apply(p)));
        let back = r.aligned(&a).unaligned(&a);
        assert!(near(back.centre(), r.centre()));
        assert!(back.yaw_rad.abs() < 1e-12);
    }

    #[test]
    fn bad_boxes_and_bands() {
        let flat = Region::from_corners(Point::new(0.0, 0.0), Point::new(2.0, 0.0));
        assert!(!flat.is_valid());
        assert!(!HeightBand::new(2.0, 1.0).is_valid());
        assert!(HeightBand::new(0.0, 1.0).contains(1.0));
    }
}
