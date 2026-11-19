//! The lanes layer from the camera (TODO 264): where there is no lane map, the lane
//! finder's two fitted lines (`/perception/lane/geometry`, in base_link) are priced with
//! **the same cost shape** as the lane map's roads ([`lanes::across`]).
//!
//! Ben, 19 Nov: "We use the perception system to identify the lines and then we bake it
//! into the cost map. So curb, high cost, yllow line med cost, lright side low cost".
//!
//! The car's own lane is the one it is in: the road's centre line is its **left** line,
//! so the car keeps right of it. Only a **yellow right** line says otherwise: the car is
//! then in the oncoming lane, and the yellow line is the centre (keep right of it). With
//! one line seen, a white right line is the edge line, the centre one lane further left.
//!
//! ```text
//!          y (left)
//!   off_road ┤░░░░░░░░░░░░░░░░░░░░  assumed: an oncoming lane as wide as ours
//!       kerb ┤══════════════════════
//! wrong_lane ┤
//!       line ┤── ── ── ── ── ── ──  the left line seen (yellow, or the centre by default)
//!          0 ┤ ●──── the car's lane ───►  x (ahead), from -behind to ahead
//!       line ┤────────────────────  the right line seen (white: the edge line)
//!       kerb ┤══════════════════════  edge_inset past it
//!   off_road ┤░░░░░░░░░░░░░░░░░░░░  off_road_band, then nothing
//! ```
//!
//! Every cost stays below INSCRIBED ([`Lanes::new`] checks the ramp): a line is never a
//! wall. Cars and people stay walls through the obstacle layers. Where the lane map
//! draws a road, the map wins ([`lanes::merged`]).

use crate::grid::{Bounds, Grid};
use crate::lanes::{self, across, LaneError, Lanes, Ramp, Road, NO_ROAD};
use crate::transform::Pose2;

/// A line's paint, as `aav_lane_geometry/LaneGeometry`'s `PAINT_*` says it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Paint {
    /// Not said.
    #[default]
    Unknown,
    /// White: an edge line, or a line between lanes going the same way.
    White,
    /// Yellow: the centre line of a two-way road.
    Yellow,
}

impl Paint {
    /// The paint of a `PAINT_*` code (0 unknown, 1 white, 2 yellow); any other is Unknown.
    #[must_use]
    pub fn from_code(code: u8) -> Self {
        match code {
            1 => Self::White,
            2 => Self::Yellow,
            _ => Self::Unknown,
        }
    }
}

/// One fitted lane line in base_link: y = a x² + b x + c (m), and its paint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeenLine {
    /// 1/m.
    pub a: f64,
    /// Slope.
    pub b: f64,
    /// m.
    pub c: f64,
    /// Its paint.
    pub paint: Paint,
}

impl SeenLine {
    fn y(&self, x: f64) -> f64 {
        (self.a * x + self.b) * x + self.c
    }

    fn slope(&self, x: f64) -> f64 {
        2.0 * self.a * x + self.b
    }

    fn finite(&self) -> bool {
        self.a.is_finite() && self.b.is_finite() && self.c.is_finite()
    }
}

/// What one camera picture showed: the lines trusted (the node gates them by confidence),
/// and the lane's width.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seen {
    /// The car's lane's left line.
    pub left: Option<SeenLine>,
    /// Its right line.
    pub right: Option<SeenLine>,
    /// m, between the two (the finder's measure, or its remembered one).
    pub width: f64,
}

/// Where the camera's lanes are drawn, and the road they assume.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraSettings {
    /// m behind base_link the lines are carried back (the finder sees from about 4 m
    /// ahead; the car's own cells need a cost too).
    pub behind: f64,
    /// m ahead of base_link: no further than the finder looks.
    pub ahead: f64,
    /// m from the edge line to the kerb.
    pub edge_inset: f64,
    /// m: a lane width outside these is clamped to them.
    pub min_width: f64,
    /// m.
    pub max_width: f64,
}

impl Default for CameraSettings {
    /// The lane finder's own view (4-16 m ahead, `lane_finder.yaml`) and lane widths
    /// (`min_lane_width`, `max_lane_width`); the lane map's edge inset.
    fn default() -> Self {
        Self {
            behind: 2.0,
            ahead: 16.0,
            edge_inset: lanes::EDGE_INSET,
            min_width: 2.5,
            max_width: 5.0,
        }
    }
}

/// The road's centre line in base_link: a seen line, moved `shift` m to its left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Centre {
    line: SeenLine,
    shift: f64,
    /// m: one lane.
    pub width: f64,
}

impl Centre {
    /// The centre line's y at x, and its slope.
    #[must_use]
    pub fn at(&self, x: f64) -> (f64, f64) {
        let slope = self.line.slope(x);
        (self.line.y(x) + self.shift * slope.hypot(1.0), slope)
    }
}

/// The camera's lanes: the ramp, and where to draw.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraLanes {
    ramp: Ramp,
    settings: CameraSettings,
}

impl CameraLanes {
    /// Checked: every cost below INSCRIBED, a sane view.
    ///
    /// # Errors
    ///
    /// [`LaneError::BadRamp`] for a ramp that would make a wall, or settings not finite,
    /// an empty view or a width range upside down.
    pub fn new(ramp: Ramp, settings: CameraSettings) -> Result<Self, LaneError> {
        Lanes::new(Vec::new(), ramp)?;
        let s = &settings;
        let finite = [s.behind, s.ahead, s.edge_inset, s.min_width, s.max_width]
            .iter()
            .all(|v| v.is_finite());
        if !finite || s.behind + s.ahead <= 0.0 || s.edge_inset < 0.0 {
            return Err(LaneError::BadRamp(
                "camera: behind + ahead above 0, edge_inset 0 or more",
            ));
        }
        if !(0.0 < s.min_width && s.min_width <= s.max_width) {
            return Err(LaneError::BadRamp("camera: 0 < min_width <= max_width"));
        }
        Ok(Self { ramp, settings })
    }

    /// The ramp.
    #[must_use]
    pub fn ramp(&self) -> Ramp {
        self.ramp
    }

    /// The settings.
    #[must_use]
    pub fn settings(&self) -> CameraSettings {
        self.settings
    }

    /// The road's centre line from what was seen; None without a line.
    #[must_use]
    pub fn centre(&self, seen: &Seen) -> Option<Centre> {
        let s = &self.settings;
        let width = if seen.width.is_finite() {
            seen.width.clamp(s.min_width, s.max_width)
        } else {
            lanes::LANE_WIDTH
        };
        let left = seen.left.filter(SeenLine::finite);
        let right = seen.right.filter(SeenLine::finite);
        let (line, shift) = match (left, right) {
            // A yellow line on the right: the car is in the oncoming lane.
            (_, Some(r)) if r.paint == Paint::Yellow => (r, 0.0),
            (Some(l), _) => (l, 0.0),
            // Only the edge line: the centre one lane to its left.
            (None, Some(r)) => (r, width),
            (None, None) => return None,
        };
        Some(Centre { line, shift, width })
    }

    /// The road the centre line implies: two lanes of `width`, two-way, the edge inset.
    fn road(&self, width: f64) -> Road {
        let inset = self.settings.edge_inset;
        Road {
            name: "camera".to_owned(),
            points: vec![(0.0, 0.0), (1.0, 0.0)],
            width: 2.0 * (width + inset),
            lanes: 2,
            oneway: false,
            edge_inset: inset,
        }
    }

    /// The cost at (x, y) in base_link; None outside the view or past the off-road band.
    #[must_use]
    pub fn cost_at(&self, centre: &Centre, x: f64, y: f64) -> Option<u8> {
        self.cost_on(&self.road(centre.width), centre, x, y)
    }

    fn cost_on(&self, road: &Road, centre: &Centre, x: f64, y: f64) -> Option<u8> {
        if x < -self.settings.behind || x > self.settings.ahead {
            return None;
        }
        let (cy, slope) = centre.at(x);
        // m right of the centre line, square to it.
        let s = (cy - y) / slope.hypot(1.0);
        across(&self.ramp, road, s)
    }

    /// Draw what was seen into `grid` (the global costmap's camera grid, in map frame),
    /// `car`: base_link in the grid's frame at the picture's stamp. Each cell of the
    /// returned area is set: a cost, or [`NO_ROAD`] where the lanes don't reach. None:
    /// nothing to draw (no line, or the view is off the grid).
    pub fn draw(&self, seen: &Seen, car: &Pose2, grid: &mut Grid) -> Option<Bounds> {
        let centre = self.centre(seen)?;
        let road = self.road(centre.width);
        let area = self.area(&centre, &road, car, grid)?;
        let to_car = car.inverse();
        for cy in area.y0..area.y1 {
            for cx in area.x0..area.x1 {
                let (x, y) = to_car.apply(grid.centre(cx, cy));
                let cost = self.cost_on(&road, &centre, x, y).unwrap_or(NO_ROAD);
                grid.set(cx, cy, cost);
            }
        }
        Some(area)
    }

    /// The grid's cells the lanes may reach: the view's box in base_link, round the
    /// centre line's whole width, into the grid.
    fn area(&self, centre: &Centre, road: &Road, car: &Pose2, grid: &Grid) -> Option<Bounds> {
        let s = &self.settings;
        let reach = road.width / 2.0 + self.ramp.off_road_band;
        let steps = ((s.ahead + s.behind).ceil() as usize).max(1);
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for k in 0..=steps {
            let x = -s.behind + (s.ahead + s.behind) * k as f64 / steps as f64;
            let (cy, slope) = centre.at(x);
            let r = reach * slope.hypot(1.0);
            lo = lo.min(cy - r);
            hi = hi.max(cy + r);
        }
        if !(lo.is_finite() && hi.is_finite()) {
            return None;
        }
        let corners = [
            (-s.behind, lo),
            (-s.behind, hi),
            (s.ahead, lo),
            (s.ahead, hi),
        ]
        .map(|p| car.apply(p));
        let (mut x0, mut y0, mut x1, mut y1) = (
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        );
        for (x, y) in corners {
            x0 = x0.min(x);
            y0 = y0.min(y);
            x1 = x1.max(x);
            y1 = y1.max(y);
        }
        let res = grid.resolution;
        let cell = |v: f64, origin: f64, n: usize| {
            ((v - origin) / res).floor().clamp(0.0, n as f64) as usize
        };
        let area = Bounds {
            x0: cell(x0, grid.origin_x, grid.width),
            y0: cell(y0, grid.origin_y, grid.height),
            x1: (cell(x1, grid.origin_x, grid.width) + 1).min(grid.width),
            y1: (cell(y1, grid.origin_y, grid.height) + 1).min(grid.height),
        };
        (area.x0 < area.x1 && area.y0 < area.y1).then_some(area)
    }
}

/// Every cell of `area` back to [`NO_ROAD`] (the camera's old drawing, before a new one).
pub fn clear(grid: &mut Grid, area: Bounds) {
    for cy in area.y0..area.y1 {
        let start = cy * grid.width;
        grid.data[start + area.x0..start + area.x1].fill(NO_ROAD);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cost::INSCRIBED;

    fn line(c: f64, paint: Paint) -> SeenLine {
        SeenLine {
            a: 0.0,
            b: 0.0,
            c,
            paint,
        }
    }

    /// A straight 3.5 m lane, the car in its middle: left line at +1.75, right at -1.75.
    fn both(left: Paint, right: Paint) -> Seen {
        Seen {
            left: Some(line(1.75, left)),
            right: Some(line(-1.75, right)),
            width: 3.5,
        }
    }

    fn cam() -> CameraLanes {
        CameraLanes::new(Ramp::default(), CameraSettings::default()).expect("good settings")
    }

    fn cost(c: &CameraLanes, seen: &Seen, x: f64, y: f64) -> Option<u8> {
        c.cost_at(&c.centre(seen).expect("a centre line"), x, y)
    }

    #[test]
    fn the_right_side_is_low_the_yellow_line_medium_the_kerb_high() {
        let c = cam();
        let r = Ramp::default();
        let seen = both(Paint::Yellow, Paint::White);
        // The car's lane's centre: 0.
        assert_eq!(cost(&c, &seen, 5.0, 0.0), Some(0));
        // Its lines: `line`; past the yellow one, the oncoming lane: `wrong_lane`.
        assert_eq!(cost(&c, &seen, 5.0, 1.75), Some(r.line));
        assert_eq!(cost(&c, &seen, 5.0, 3.5), Some(r.wrong_lane));
        assert_eq!(cost(&c, &seen, 5.0, -1.75), Some(r.line));
        // The white edge line, then up to the kerb 0.5 m past it, then off the road.
        let kerb = cost(&c, &seen, 5.0, -2.2).expect("a cost");
        assert!(kerb > r.line && kerb <= r.kerb, "{kerb}");
        assert_eq!(cost(&c, &seen, 5.0, -2.5), Some(r.off_road));
        assert_eq!(cost(&c, &seen, 5.0, -4.0), None);
        // Every cost below a wall's.
        for i in -60..=60 {
            let y = 0.1 * f64::from(i);
            assert!(cost(&c, &seen, 5.0, y).is_none_or(|v| v < INSCRIBED), "{y}");
        }
        // Rising from the lane's centre to the yellow line, and on to the kerb.
        let across: Vec<u8> = (0..=24)
            .map(|i| cost(&c, &seen, 5.0, -0.1 * f64::from(i)).unwrap_or(0))
            .collect();
        assert!(across.windows(2).all(|p| p[0] <= p[1]), "{across:?}");
        // Only inside the view.
        assert_eq!(cost(&c, &seen, -2.5, 0.0), None);
        assert_eq!(cost(&c, &seen, 16.5, 0.0), None);
    }

    #[test]
    fn the_map_and_the_camera_price_a_road_alike() {
        // The lane map's two-way road, 3.5 m lanes, the same edge inset, driven along:
        // the same costs across as the camera's lines on it.
        let c = cam();
        let road = c.road(3.5);
        let lanes = Lanes::new(vec![road.clone()], Ramp::default()).expect("a good road");
        let seen = both(Paint::Yellow, Paint::White);
        // s: m right of the road's centre line, the camera's left line (y = 1.75).
        for i in -50..=50 {
            let s = 0.1 * f64::from(i);
            assert_eq!(
                lanes.cost_across(&road, s, lanes::Way::Along),
                cost(&c, &seen, 3.0, 1.75 - s),
                "{s}"
            );
        }
    }

    #[test]
    fn a_yellow_right_line_puts_the_car_in_the_oncoming_lane() {
        // The car drifted over: the yellow line is on its right, so its own lane is one
        // further right, and where it is costs `wrong_lane`.
        let c = cam();
        let seen = both(Paint::White, Paint::Yellow);
        let r = Ramp::default();
        assert_eq!(cost(&c, &seen, 5.0, 0.0), Some(r.wrong_lane));
        assert_eq!(cost(&c, &seen, 5.0, -3.5), Some(0));
    }

    #[test]
    fn one_line_seen() {
        let c = cam();
        // Only the edge line: the centre line is a lane to its left.
        let edge = Seen {
            left: None,
            right: Some(line(-1.75, Paint::White)),
            width: 3.5,
        };
        assert_eq!(cost(&c, &edge, 5.0, 0.0), Some(0));
        assert_eq!(cost(&c, &edge, 5.0, 3.5), Some(Ramp::default().wrong_lane));
        // Only the left line, whatever its paint: the centre line.
        let centre = Seen {
            left: Some(line(1.75, Paint::Unknown)),
            right: None,
            width: 3.5,
        };
        assert_eq!(cost(&c, &centre, 5.0, 0.0), Some(0));
        // None: nothing.
        let none = Seen {
            left: None,
            right: None,
            width: 3.5,
        };
        assert!(c.centre(&none).is_none());
    }

    #[test]
    fn a_bend_follows_the_lines() {
        // The lane bends left (radius 50 m): 10 m ahead, its centre is 1 m left.
        let c = cam();
        let bend = |c0: f64| SeenLine {
            a: 0.01,
            b: 0.0,
            c: c0,
            paint: Paint::Unknown,
        };
        let seen = Seen {
            left: Some(bend(1.75)),
            right: Some(bend(-1.75)),
            width: 3.5,
        };
        assert!(cost(&c, &seen, 10.0, 1.0).expect("a cost") <= 2);
        assert!(cost(&c, &seen, 10.0, -0.75).expect("a cost") > 30);
    }

    #[test]
    fn drawn_on_the_map_where_the_car_is() {
        // The car at (20, 10) in the map, facing north: its left is west (-x).
        let c = cam();
        let mut grid = Grid::new((0.0, 0.0), 0.25, 160, 160, NO_ROAD);
        let car = Pose2::new(20.0, 10.0, std::f64::consts::FRAC_PI_2);
        let area = c
            .draw(&both(Paint::Yellow, Paint::White), &car, &mut grid)
            .expect("drawn");
        let at = |x: f64, y: f64| grid.cost_at(x, y, NO_ROAD);
        let r = Ramp::default();
        // 5 m ahead (north): the lane's centre 0, the oncoming lane west of it.
        assert!(at(20.0, 15.0) <= 7);
        assert_eq!(at(16.5, 15.0), r.wrong_lane);
        assert_eq!(at(22.6, 15.0), r.off_road);
        // Past the view: nothing.
        assert_eq!(at(20.0, 27.0), NO_ROAD);
        assert_eq!(at(26.0, 15.0), NO_ROAD);
        // Cleared, the area is nothing again.
        clear(&mut grid, area);
        assert!(grid.data.iter().all(|&v| v == NO_ROAD));
    }

    #[test]
    fn bad_settings_are_refused() {
        let wall = Ramp {
            wrong_lane: INSCRIBED,
            ..Ramp::default()
        };
        assert!(CameraLanes::new(wall, CameraSettings::default()).is_err());
        let upside_down = CameraSettings {
            min_width: 5.0,
            max_width: 3.0,
            ..CameraSettings::default()
        };
        assert!(CameraLanes::new(Ramp::default(), upside_down).is_err());
    }

    #[test]
    fn paint_codes_are_lane_geometrys() {
        assert_eq!(Paint::from_code(0), Paint::Unknown);
        assert_eq!(Paint::from_code(1), Paint::White);
        assert_eq!(Paint::from_code(2), Paint::Yellow);
        assert_eq!(Paint::from_code(9), Paint::Unknown);
    }
}
