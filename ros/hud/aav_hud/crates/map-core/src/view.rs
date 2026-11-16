//! From the map to the screen and back: north up, one scale for x and y.
//!
//! Three ways to look:
//!
//! | Mode | Centre | Scale |
//! |---|---|---|
//! | [`Mode::FollowCar`] (the start) | the car | yours: zoom with the wheel or + / - |
//! | [`Mode::WholeMap`] | the map's middle | the whole map fits |
//! | [`Mode::Free`] | where you dragged it | yours |
//!
//! F switches between following the car and the whole map. Dragging, or zooming the
//! whole map, lets go of both (Free).

use crate::{geom::Point, grid::Bounds};

/// The scale: screen pixels per metre.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct PixelsPerMetre(pub f64);

impl PixelsPerMetre {
    /// The most zoomed out: 1 px = 50 m (60 km across a big screen: the street map).
    pub const MIN: Self = Self(0.02);
    /// The most zoomed in: 1 m = 400 px.
    pub const MAX: Self = Self(400.0);

    fn clamped(self) -> Self {
        Self(self.0.clamp(Self::MIN.0, Self::MAX.0))
    }
}

/// A point on the screen, in pixels from the window's top left.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ScreenPoint {
    /// px, right.
    pub x: f64,
    /// px, down.
    pub y: f64,
}

impl ScreenPoint {
    /// A point from x and y (px).
    #[must_use]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// The part of the window the map is drawn in, in pixels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Viewport {
    /// The left edge (px).
    pub x: f64,
    /// The top edge (px).
    pub y: f64,
    /// px.
    pub width: f64,
    /// px.
    pub height: f64,
}

impl Viewport {
    /// The middle.
    #[must_use]
    pub fn centre(&self) -> ScreenPoint {
        ScreenPoint::new(self.x + self.width / 2.0, self.y + self.height / 2.0)
    }

    /// True if `p` is inside.
    #[must_use]
    pub fn contains(&self, p: ScreenPoint) -> bool {
        (self.x..self.x + self.width).contains(&p.x)
            && (self.y..self.y + self.height).contains(&p.y)
    }
}

/// What the view keeps in sight.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// Keeps the car in the middle.
    #[default]
    FollowCar,
    /// Fits the whole map.
    WholeMap,
    /// Stays where the person moved it.
    Free,
}

impl Mode {
    /// The words on the screen.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::FollowCar => "FOLLOWING THE CAR",
            Self::WholeMap => "WHOLE MAP",
            Self::Free => "FREE VIEW",
        }
    }
}

/// How many metres across the window shows when it starts following the car.
pub const FOLLOW_SPAN_M: f64 = 60.0;
/// Space kept around the whole map, as a share of the window.
const FIT_MARGIN: f64 = 0.04;

/// Where the view looks, and how close.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct View {
    /// The map point at the viewport's middle.
    pub centre: Point,
    /// Pixels per metre: the zoom.
    pub scale: PixelsPerMetre,
    /// What it keeps in sight.
    pub mode: Mode,
    /// The scale to go back to when following the car again; None until first set.
    follow_scale: Option<PixelsPerMetre>,
}

impl Default for View {
    fn default() -> Self {
        Self {
            centre: Point::default(),
            scale: PixelsPerMetre(10.0),
            mode: Mode::FollowCar,
            follow_scale: None,
        }
    }
}

impl View {
    /// A map point on the screen. North (+y) is up.
    #[must_use]
    pub fn to_screen(&self, p: Point, vp: &Viewport) -> ScreenPoint {
        let c = vp.centre();
        let k = self.scale.0;
        ScreenPoint::new(
            c.x + (p.x - self.centre.x) * k,
            c.y - (p.y - self.centre.y) * k,
        )
    }

    /// A screen point on the map: the opposite of [`View::to_screen`].
    #[must_use]
    pub fn to_map(&self, s: ScreenPoint, vp: &Viewport) -> Point {
        let c = vp.centre();
        let k = self.scale.0;
        Point::new(
            self.centre.x + (s.x - c.x) / k,
            self.centre.y - (s.y - c.y) / k,
        )
    }

    /// The map area the viewport shows.
    #[must_use]
    pub fn visible(&self, vp: &Viewport) -> Bounds {
        let a = self.to_map(ScreenPoint::new(vp.x, vp.y + vp.height), vp);
        let b = self.to_map(ScreenPoint::new(vp.x + vp.width, vp.y), vp);
        Bounds { min: a, max: b }
    }

    /// Each frame: follow the car, or fit the map, as the mode says. Following with no
    /// car yet shows the whole map until it comes.
    pub fn update(&mut self, car: Option<Point>, map: Option<Bounds>, vp: &Viewport) {
        match (self.mode, car, map) {
            (Mode::FollowCar, Some(car), _) => {
                self.scale = *self
                    .follow_scale
                    .get_or_insert(PixelsPerMetre(vp.width / FOLLOW_SPAN_M).clamped());
                self.centre = car;
            }
            (Mode::FollowCar | Mode::WholeMap, _, Some(map)) => self.fit(map, vp),
            _ => {}
        }
    }

    /// Fits a box in the viewport, centred.
    pub fn fit(&mut self, b: Bounds, vp: &Viewport) {
        let (w, h) = b.size();
        let k = (vp.width / w.max(1e-3)).min(vp.height / h.max(1e-3)) * (1.0 - 2.0 * FIT_MARGIN);
        self.scale = PixelsPerMetre(k).clamped();
        self.centre = b.centre();
    }

    /// Sets the scale (clamped), and keeps it for following the car too.
    pub fn set_scale(&mut self, scale: PixelsPerMetre) {
        self.scale = scale.clamped();
        self.follow_scale = Some(self.scale);
    }

    /// F: following the car <-> the whole map. From a free view, back to the car.
    pub fn toggle_follow(&mut self) {
        self.mode = match self.mode {
            Mode::FollowCar => Mode::WholeMap,
            Mode::WholeMap | Mode::Free => Mode::FollowCar,
        };
    }

    /// Zooms by `factor` (2 = twice as close). Following the car, it stays on the car.
    /// Otherwise the map point under `about` (the mouse) stays where it is, and the view
    /// becomes free.
    pub fn zoom(&mut self, factor: f64, about: Option<ScreenPoint>, vp: &Viewport) {
        if !(factor.is_finite() && factor > 0.0) {
            return;
        }
        let new_scale = PixelsPerMetre(self.scale.0 * factor).clamped();
        if self.mode == Mode::FollowCar {
            self.scale = new_scale;
            self.follow_scale = Some(new_scale);
            return;
        }
        let about = about.unwrap_or_else(|| vp.centre());
        let fixed = self.to_map(about, vp);
        self.scale = new_scale;
        // Move the centre so `fixed` is under `about` again.
        let moved = self.to_map(about, vp);
        self.centre = self.centre + (fixed - moved);
        self.mode = Mode::Free;
    }

    /// Centres the view on a map point (a place picked in the search), at the same
    /// scale: the view becomes free; F goes back to the car.
    pub fn look_at(&mut self, p: Point) {
        if p.is_finite() {
            self.centre = p;
            self.mode = Mode::Free;
        }
    }

    /// Drags the map by (dx, dy) screen pixels: the view becomes free.
    pub fn pan(&mut self, dx: f64, dy: f64) {
        if dx == 0.0 && dy == 0.0 {
            return;
        }
        let k = self.scale.0;
        self.centre = self.centre + Point::new(-dx / k, dy / k);
        self.mode = Mode::Free;
    }
}

/// The largest of 1, 2, 5, 10, 20, 50... that is at most `limit`; None below 0.01.
#[must_use]
pub fn nice_at_most(limit: f64) -> Option<f64> {
    if !(limit.is_finite() && limit >= 0.01) {
        return None;
    }
    let decade = 10f64.powf(limit.log10().floor());
    [5.0, 2.0, 1.0]
        .into_iter()
        .map(|m| m * decade)
        .find(|v| *v <= limit * (1.0 + 1e-9))
}

/// The scale bar: a round number of metres, and its length in pixels, no longer than
/// `max_px`.
#[must_use]
pub fn scale_bar(scale: PixelsPerMetre, max_px: f64) -> Option<(f64, f64)> {
    let metres = nice_at_most(max_px / scale.0)?;
    Some((metres, metres * scale.0))
}

/// The grid lines' spacing in metres: a round number, at least `min_px` apart.
#[must_use]
pub fn grid_step(scale: PixelsPerMetre, min_px: f64) -> f64 {
    let at_least = min_px / scale.0;
    // The smallest 1-2-5 number at or above `at_least`.
    let decade = 10f64.powf(at_least.log10().floor());
    [1.0, 2.0, 5.0, 10.0]
        .into_iter()
        .map(|m| m * decade)
        .find(|v| *v >= at_least * (1.0 - 1e-9))
        .unwrap_or(10.0 * decade)
}

/// The grid lines' positions from `from` to `to`, `step` apart, on multiples of `step`.
pub fn grid_lines(from: f64, to: f64, step: f64) -> impl Iterator<Item = f64> {
    let first = (from / step).ceil() as i64;
    let last = (to / step).floor() as i64;
    (first..=last).map(move |i| i as f64 * step)
}

#[cfg(test)]
mod tests {
    use super::*;

    const VP: Viewport = Viewport {
        x: 0.0,
        y: 0.0,
        width: 800.0,
        height: 600.0,
    };

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn north_is_up_and_back_again() {
        let v = View {
            centre: Point::new(10.0, 20.0),
            scale: PixelsPerMetre(10.0),
            ..View::default()
        };
        assert_eq!(
            v.to_screen(Point::new(10.0, 20.0), &VP),
            ScreenPoint::new(400.0, 300.0)
        );
        // 1 m north is 10 px up; 1 m east is 10 px right.
        assert_eq!(
            v.to_screen(Point::new(11.0, 21.0), &VP),
            ScreenPoint::new(410.0, 290.0)
        );
        let p = Point::new(-3.5, 7.25);
        let back = v.to_map(v.to_screen(p, &VP), &VP);
        assert!(close(back.x, p.x) && close(back.y, p.y));
    }

    #[test]
    fn following_keeps_the_car_in_the_middle() {
        let mut v = View::default();
        v.update(Some(Point::new(5.0, 6.0)), None, &VP);
        assert_eq!(v.centre, Point::new(5.0, 6.0));
        assert!(close(v.scale.0, 800.0 / FOLLOW_SPAN_M));
        // Zooming keeps following, and the new scale stays.
        v.zoom(2.0, Some(ScreenPoint::new(0.0, 0.0)), &VP);
        v.update(Some(Point::new(7.0, 6.0)), None, &VP);
        assert_eq!(v.mode, Mode::FollowCar);
        assert_eq!(v.centre, Point::new(7.0, 6.0));
        assert!(close(v.scale.0, 2.0 * 800.0 / FOLLOW_SPAN_M));
    }

    #[test]
    fn the_whole_map_fits() {
        let mut v = View::default();
        v.toggle_follow();
        assert_eq!(v.mode, Mode::WholeMap);
        let map = Bounds {
            min: Point::new(-10.0, -10.0),
            max: Point::new(70.0, 50.0),
        };
        v.update(Some(Point::new(0.0, 0.0)), Some(map), &VP);
        assert_eq!(v.centre, Point::new(30.0, 20.0));
        // 80 m x 60 m in 800 x 600 px: 10 px/m, less the margin.
        assert!(close(v.scale.0, 10.0 * (1.0 - 2.0 * FIT_MARGIN)));
        let seen = v.visible(&VP);
        assert!(seen.min.x < -10.0 && seen.max.y > 50.0);
        v.toggle_follow();
        assert_eq!(v.mode, Mode::FollowCar);
    }

    #[test]
    fn following_with_no_car_shows_the_map() {
        let mut v = View::default();
        let map = Bounds {
            min: Point::new(0.0, 0.0),
            max: Point::new(10.0, 10.0),
        };
        v.update(None, Some(map), &VP);
        assert_eq!(v.centre, Point::new(5.0, 5.0));
        assert_eq!(v.mode, Mode::FollowCar);
    }

    #[test]
    fn zooming_a_free_view_keeps_the_point_under_the_mouse() {
        let mut v = View {
            mode: Mode::WholeMap,
            ..View::default()
        };
        let mouse = ScreenPoint::new(100.0, 500.0);
        let under = v.to_map(mouse, &VP);
        v.zoom(1.5, Some(mouse), &VP);
        assert_eq!(v.mode, Mode::Free);
        let after = v.to_map(mouse, &VP);
        assert!(close(after.x, under.x) && close(after.y, under.y));
        // It never goes past its limits, and refuses nonsense.
        v.zoom(1e9, None, &VP);
        assert_eq!(v.scale, PixelsPerMetre::MAX);
        v.zoom(f64::NAN, None, &VP);
        assert_eq!(v.scale, PixelsPerMetre::MAX);
    }

    #[test]
    fn dragging_moves_the_map_with_the_mouse() {
        let mut v = View::default();
        let p = Point::new(1.0, 1.0);
        let before = v.to_screen(p, &VP);
        v.pan(30.0, -20.0);
        let after = v.to_screen(p, &VP);
        assert!(close(after.x - before.x, 30.0) && close(after.y - before.y, -20.0));
        assert_eq!(v.mode, Mode::Free);
    }

    #[test]
    fn round_numbers() {
        assert_eq!(nice_at_most(7.3), Some(5.0));
        assert_eq!(nice_at_most(2.0), Some(2.0));
        assert_eq!(nice_at_most(19.9), Some(10.0));
        assert_eq!(nice_at_most(0.3), Some(0.2));
        assert_eq!(nice_at_most(0.0), None);
        // 10 px/m, at most 160 px: 10 m, 100 px.
        assert_eq!(scale_bar(PixelsPerMetre(10.0), 160.0), Some((10.0, 100.0)));
        // Lines at least 50 px apart at 10 px/m: every 5 m.
        assert_eq!(grid_step(PixelsPerMetre(10.0), 50.0), 5.0);
        assert_eq!(grid_step(PixelsPerMetre(10.0), 51.0), 10.0);
        assert_eq!(grid_step(PixelsPerMetre(100.0), 50.0), 0.5);
        let lines: Vec<f64> = grid_lines(-7.0, 12.0, 5.0).collect();
        assert_eq!(lines, [-5.0, 0.0, 5.0, 10.0]);
    }

    #[test]
    fn looking_at_a_place_frees_the_view() {
        let mut v = View::default();
        v.look_at(Point::new(120.0, -40.0));
        assert_eq!(v.centre, Point::new(120.0, -40.0));
        assert_eq!(v.mode, Mode::Free);
        v.look_at(Point::new(f64::NAN, 0.0));
        assert_eq!(v.centre, Point::new(120.0, -40.0));
    }
}
