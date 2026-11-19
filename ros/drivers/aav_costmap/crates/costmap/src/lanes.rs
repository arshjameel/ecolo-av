//! The lanes layer: the roads drawn as a soft cost, lowest at the right lane's centre
//! (TODO 148, decision D2 in `docs/designs/2026-11-13_one-shape.md`). Ours, not Nav2's.
//!
//! One road seen from above, the car driving left to right (`s`: metres right of the
//! road's centre line, the way the car drives):
//!
//! ```text
//!  off_road ┤░░░░░░░░░░░░░░░░░░░░░░░░░░  off_road_band past the kerb, then nothing
//!      kerb ┤══════════════════════════  left kerb      ╲
//!           │                                            ramp up to `kerb`
//!wrong_lane ┤── ── ── ── ── ── ── ── ──  centre line    flat: the oncoming lane
//!      line ┤                                           ╲
//!         0 ┤ ●──── the car's lane centre ────►           ramp, 0 at the centre
//!      line ┤──────────────────────────  edge line      ╱
//!      kerb ┤══════════════════════════  right kerb     ramp up to `kerb`
//! ```
//!
//! Which lane is "right" depends on the way the car drives the road, so the layer is drawn
//! per route: [`Lanes::set_route`] takes the planner's newest path and marks each road
//! segment it runs along as [`Way::Along`] or [`Way::Against`] its drawn direction. A
//! segment no route runs along is [`Way::Unknown`]: on a two-way road both lanes' centres
//! are then low, so the first plan is free to take either, and the next keeps right.
//! The lane's place and width are `aav_description`'s `lane_map.py` `lane_offset`, so the
//! costs and the road ahead agree.
//!
//! Every cost stays below [`INSCRIBED`]: a lane is never a wall. The planner pays for
//! crossing a line only when it gains more (Ben, 13 Nov: lane lines "it can cross but only
//! if it needs to and to stay on the right side").

use std::fmt;

use crate::cost::{INSCRIBED, NO_INFORMATION};
use crate::grid::{Bounds, Grid};

/// m: one lane, when a road doesn't say how many it has (`lane_map.py` LANE_WIDTH).
pub const LANE_WIDTH: f64 = 3.5;
/// m: kerb to edge line (`lane_map.py` EDGE_INSET).
pub const EDGE_INSET: f64 = 0.5;
/// rad: a route runs along a segment if their headings agree within this (`MAX_TURN`).
const MAX_TURN: f64 = std::f64::consts::FRAC_PI_4;
/// m past the kerb a route point may be and still be on that road (`lane_map.py` MARGIN).
const MARGIN: f64 = 0.5;
/// Marks a cell no road reaches while drawing (never a cost: every cost is < INSCRIBED).
const NONE: u8 = NO_INFORMATION;
/// A lanes grid's cell no road (and no camera line) reaches: [`merged`] reads it as 0.
pub const NO_ROAD: u8 = NONE;

/// One road: its centre line (map frame, m), its width kerb to kerb, how its lanes run.
#[derive(Debug, Clone, PartialEq)]
pub struct Road {
    /// Its name, for messages.
    pub name: String,
    /// The centre line, drawn in the road's own direction.
    pub points: Vec<(f64, f64)>,
    /// m, kerb to kerb.
    pub width: f64,
    /// Every lane, both ways; 0: width / [`LANE_WIDTH`], rounded.
    pub lanes: u32,
    /// All its lanes run the drawn way.
    pub oneway: bool,
    /// m from the kerb to the edge line.
    pub edge_inset: f64,
}

impl Road {
    /// Every lane, both ways.
    #[must_use]
    pub fn lane_count(&self) -> u32 {
        if self.lanes > 0 {
            self.lanes
        } else {
            ((self.width / LANE_WIDTH).round() as u32).max(1)
        }
    }

    /// The car's lane: its centre (m right of the road's centre, the way the car drives)
    /// and its width. One lane: 0 and the whole painted width (`lane_map.py` lane_offset).
    #[must_use]
    pub fn lane(&self) -> (f64, f64) {
        let painted = self.width - 2.0 * self.edge_inset;
        let n = self.lane_count();
        if n <= 1 {
            return (0.0, painted);
        }
        let width = if self.oneway {
            painted / f64::from(n)
        } else {
            (painted / 2.0) / f64::from((n / 2).max(1))
        };
        (painted / 2.0 - width / 2.0, width)
    }
}

/// The costs across a road (Nav2's 0-252 scale; each below [`INSCRIBED`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ramp {
    /// At the car's lane's own lines; 0 at its centre, straight in between. Also the
    /// other lanes the car's way (a one-way road's, a wide road's inner lane).
    pub line: u8,
    /// Over the centre line, in the oncoming lanes.
    pub wrong_lane: u8,
    /// At the kerb, rising straight from the edge line.
    pub kerb: u8,
    /// Past the kerb, for `off_road_band` m.
    pub off_road: u8,
    /// m past the kerb that cost `off_road`; further out the layer adds nothing.
    pub off_road_band: f64,
}

impl Default for Ramp {
    /// 15 Nov, the second guess (the first, 50 / 100 / 150 / 150 combined by the higher,
    /// left the car in the road's middle): the oncoming lane costs about twice the lane's
    /// own line; the kerb and past it more. With the planner's `cost_penalty` (2), a metre
    /// in the oncoming lane costs about 2.2 m of travel, a metre on the right lane's
    /// centre 1 m.
    fn default() -> Self {
        Self {
            line: 80,
            wrong_lane: 150,
            kerb: 200,
            off_road: 200,
            off_road_band: 1.0,
        }
    }
}

/// The way a route drives a road segment, against the way the road was drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Way {
    /// The drawn way.
    Along,
    /// Against the drawn way.
    Against,
    /// No route runs along it.
    Unknown,
}

/// Why a lane map can't be drawn.
#[derive(Debug, Clone, PartialEq)]
pub enum LaneError {
    /// A road with fewer than two points, a width not above 0, or a number not finite.
    BadRoad(String),
    /// A ramp cost at or above INSCRIBED (a lane would be a wall), or a bad band.
    BadRamp(&'static str),
}

impl fmt::Display for LaneError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadRoad(name) => write!(
                f,
                "road {name:?}: needs 2 or more points, a width above 0, finite numbers"
            ),
            Self::BadRamp(what) => write!(f, "lane ramp: {what}"),
        }
    }
}

impl std::error::Error for LaneError {}

/// The roads, the ramp, and the way the newest route drives each segment.
#[derive(Debug, Clone, PartialEq)]
pub struct Lanes {
    roads: Vec<Road>,
    ramp: Ramp,
    /// ways[road][segment].
    ways: Vec<Vec<Way>>,
}

impl Lanes {
    /// The lane map, checked. Every segment starts [`Way::Unknown`].
    ///
    /// # Errors
    ///
    /// [`LaneError`] for a bad road or a bad ramp.
    pub fn new(roads: Vec<Road>, ramp: Ramp) -> Result<Self, LaneError> {
        for r in &roads {
            let finite = r.points.iter().all(|p| p.0.is_finite() && p.1.is_finite());
            let inset_ok = r.edge_inset.is_finite() && r.edge_inset >= 0.0;
            if r.points.len() < 2 || r.width <= 0.0 || !r.width.is_finite() || !finite || !inset_ok
            {
                return Err(LaneError::BadRoad(r.name.clone()));
            }
        }
        if [ramp.line, ramp.wrong_lane, ramp.kerb, ramp.off_road]
            .iter()
            .any(|&c| c >= INSCRIBED)
        {
            return Err(LaneError::BadRamp(
                "every cost below 253 (a lane is never a wall)",
            ));
        }
        if ramp.off_road_band < 0.0 || !ramp.off_road_band.is_finite() {
            return Err(LaneError::BadRamp("off_road_band: 0 m or more"));
        }
        let ways = roads
            .iter()
            .map(|r| vec![Way::Unknown; r.points.len() - 1])
            .collect();
        Ok(Self { roads, ramp, ways })
    }

    /// The roads.
    #[must_use]
    pub fn roads(&self) -> &[Road] {
        &self.roads
    }

    /// The ramp.
    #[must_use]
    pub fn ramp(&self) -> Ramp {
        self.ramp
    }

    /// The way the newest route drives segment `segment` of road `road`.
    #[must_use]
    pub fn way(&self, road: usize, segment: usize) -> Option<Way> {
        self.ways.get(road)?.get(segment).copied()
    }

    /// A new route (map frame, m, in driving order): each segment it runs along takes its
    /// way; every other one goes back to Unknown. Returns the area (map frame, m:
    /// x0, y0, x1, y1) whose costs changed, or None if no segment's way did.
    pub fn set_route(&mut self, route: &[(f64, f64)]) -> Option<(f64, f64, f64, f64)> {
        let mut ways: Vec<Vec<Way>> = self
            .roads
            .iter()
            .map(|r| vec![Way::Unknown; r.points.len() - 1])
            .collect();
        // Every metre along the route, at least: a plan's points are 5 cm apart, but a
        // route drawn by hand may skip a whole segment between two.
        let mut steps = Vec::new();
        for (p, q) in route.iter().zip(route.iter().skip(1)) {
            let (dx, dy) = (q.0 - p.0, q.1 - p.1);
            let length = dx.hypot(dy);
            if length < 1e-6 {
                continue;
            }
            let heading = dy.atan2(dx);
            let n = length.ceil().max(1.0) as usize;
            steps.extend((0..=n).map(|k| {
                let t = k as f64 / n as f64;
                ((p.0 + t * dx, p.1 + t * dy), heading)
            }));
        }
        for (p, heading) in steps {
            for (ri, road) in self.roads.iter().enumerate() {
                let reach = road.width / 2.0 + MARGIN;
                for (si, (a, b)) in road
                    .points
                    .iter()
                    .zip(road.points.iter().skip(1))
                    .enumerate()
                {
                    if foot(p, *a, *b).2 > reach {
                        continue;
                    }
                    let turn = wrap(heading - (b.1 - a.1).atan2(b.0 - a.0)).abs();
                    if turn <= MAX_TURN {
                        ways[ri][si] = Way::Along;
                    } else if turn >= std::f64::consts::PI - MAX_TURN {
                        ways[ri][si] = Way::Against;
                    }
                }
            }
        }
        let mut changed: Option<(f64, f64, f64, f64)> = None;
        for (ri, (new, old)) in ways.iter().zip(&self.ways).enumerate() {
            if new != old {
                let b = self.road_box(ri);
                changed = Some(match changed {
                    None => b,
                    Some(c) => (c.0.min(b.0), c.1.min(b.1), c.2.max(b.2), c.3.max(b.3)),
                });
            }
        }
        self.ways = ways;
        changed
    }

    /// m: how far from a road's centre line its costs reach.
    fn reach(&self, road: &Road) -> f64 {
        road.width / 2.0 + self.ramp.off_road_band
    }

    /// The area a road's costs cover (map frame, m: x0, y0, x1, y1).
    fn road_box(&self, ri: usize) -> (f64, f64, f64, f64) {
        let road = &self.roads[ri];
        let pad = self.reach(road);
        road.points.iter().fold(
            (
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ),
            |b, p| {
                (
                    b.0.min(p.0 - pad),
                    b.1.min(p.1 - pad),
                    b.2.max(p.0 + pad),
                    b.3.max(p.1 + pad),
                )
            },
        )
    }

    /// The cost `s` m right of a road's centre line (as drawn), driven `way`; None past
    /// the off-road band.
    #[must_use]
    pub fn cost_across(&self, road: &Road, s: f64, way: Way) -> Option<u8> {
        match way {
            Way::Along => self.profile(road, s),
            Way::Against => self.profile(road, -s),
            // A one-way road is driven its own way; a two-way one either way, so each
            // lane's centre is low until a route says which.
            Way::Unknown if road.oneway => self.profile(road, s),
            Way::Unknown => match (self.profile(road, s), self.profile(road, -s)) {
                (Some(a), Some(b)) => Some(a.min(b)),
                (a, b) => a.or(b),
            },
        }
    }

    /// The cost `s` m right of the centre line, the way the car drives.
    fn profile(&self, road: &Road, s: f64) -> Option<u8> {
        across(&self.ramp, road, s)
    }

    /// Draw the cells in `area` of `grid` (a grid on the map, in map frame): each the
    /// lowest any road gives it, 0 where none reaches.
    pub fn draw(&self, grid: &mut Grid, area: Bounds) {
        self.draw_marked(grid, area, 0);
    }

    /// [`Lanes::draw`], with `none` where no road reaches ([`NO_ROAD`]: the camera's
    /// lanes may fill those cells; see [`merged`]).
    pub fn draw_marked(&self, grid: &mut Grid, area: Bounds, none: u8) {
        if area.width() == 0 || area.height() == 0 {
            return;
        }
        let (w, h) = (area.width(), area.height());
        let mut part = vec![NONE; w * h];
        let res = grid.resolution;
        for (ri, road) in self.roads.iter().enumerate() {
            let pad = self.reach(road);
            for (si, (a, b)) in road
                .points
                .iter()
                .zip(road.points.iter().skip(1))
                .enumerate()
            {
                let way = self.ways[ri][si];
                // The segment's cells, padded, cut to the area.
                let cells = |lo: f64, hi: f64, origin: f64, first: usize, end: usize| {
                    let from = ((lo - pad - origin) / res).floor().max(first as f64);
                    let to = ((hi + pad - origin) / res).ceil().min(end as f64);
                    (from as usize, to.max(from) as usize)
                };
                let (x0, x1) = cells(a.0.min(b.0), a.0.max(b.0), grid.origin_x, area.x0, area.x1);
                let (y0, y1) = cells(a.1.min(b.1), a.1.max(b.1), grid.origin_y, area.y0, area.y1);
                for cy in y0..y1 {
                    let row = (cy - area.y0) * w;
                    for cx in x0..x1 {
                        let p = grid.centre(cx, cy);
                        let (fx, fy, d) = foot(p, *a, *b);
                        if d > pad {
                            continue;
                        }
                        // + right of the drawn direction.
                        let cross = (b.0 - a.0) * (p.1 - fy) - (b.1 - a.1) * (p.0 - fx);
                        let s = if cross > 0.0 { -d } else { d };
                        if let Some(c) = self.cost_across(road, s, way) {
                            let cell = &mut part[row + cx - area.x0];
                            if *cell == NONE || c < *cell {
                                *cell = c;
                            }
                        }
                    }
                }
            }
        }
        for (y, row) in (area.y0..area.y1).zip(part.chunks_exact(w)) {
            let start = y * grid.width + area.x0;
            for (out, &c) in grid.data[start..start + w].iter_mut().zip(row) {
                *out = if c == NONE { none } else { c };
            }
        }
    }
}

/// The cost `s` m right of a road's centre line, the way the car drives; None past the
/// off-road band. **The one cost shape**: the lane map ([`Lanes`]) and the camera's lines
/// (`camera_lanes`) both price a road through it, so the two agree.
#[must_use]
pub fn across(r: &Ramp, road: &Road, s: f64) -> Option<u8> {
    let half = road.width / 2.0;
    if s.abs() > half {
        return (s.abs() - half <= r.off_road_band).then_some(r.off_road);
    }
    let painted = (half - road.edge_inset).max(0.0);
    let (centre, width) = road.lane();
    let lane_half = width / 2.0;
    let cost = if (s - centre).abs() <= lane_half {
        lerp(0, r.line, (s - centre).abs() / lane_half.max(1e-9))
    } else if s > centre {
        // Toward the right kerb, from the car's lane's edge line.
        lerp(r.line, r.kerb, ramp_t(s, centre + lane_half, half))
    } else {
        // Left of the car's lane: its own way's other lanes, then (two-way) the
        // oncoming ones, then up to the left kerb.
        let lanes = if road.oneway || s >= 0.0 || road.lane_count() <= 1 {
            r.line
        } else {
            r.wrong_lane
        };
        if s >= -painted {
            lanes
        } else {
            lerp(lanes, r.kerb, ramp_t(-s, painted, half))
        }
    };
    Some(cost)
}

/// The most the lanes and the inflation make together: one below inscribed, so a lane
/// never becomes a wall.
pub const MAX_SUM: u8 = INSCRIBED - 1;

/// The lanes' costs onto a master grid's cells: **added** to the inflation, up to
/// [`MAX_SUM`]; an unknown cell or an obstacle (the car's inscribed circle and worse)
/// stays as it is.
///
/// Added, not the higher of the two (Nav2's `updateWithMax`): on the campus the lidar marks
/// both kerbs, and the inflation from them is lowest in the road's middle. With the higher
/// of the two, the right lane's centre (59 from the kerb 2 m away) cost more than the
/// centre line (50), and the car kept to the middle (15 Nov, the first runs).
pub fn combine(master: &mut [u8], lanes: &[u8]) {
    for (m, &l) in master.iter_mut().zip(lanes) {
        if *m < INSCRIBED {
            *m = m.saturating_add(l).min(MAX_SUM.max(*m));
        }
    }
}

/// One cell's lane cost from the lane map's grid and the camera's (each [`NO_ROAD`] where
/// it has nothing): **the map wins** where it has a road; else the camera's; else 0.
#[must_use]
pub fn merged(map: u8, camera: u8) -> u8 {
    match (map, camera) {
        (NO_ROAD, NO_ROAD) => 0,
        (NO_ROAD, c) => c,
        (m, _) => m,
    }
}

/// The nearest point to `p` on segment a-b, and its distance.
fn foot(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> (f64, f64, f64) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let n2 = dx * dx + dy * dy;
    let t = if n2 > 1e-12 {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / n2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let (fx, fy) = (a.0 + t * dx, a.1 + t * dy);
    (fx, fy, (p.0 - fx).hypot(p.1 - fy))
}

fn wrap(angle: f64) -> f64 {
    angle.sin().atan2(angle.cos())
}

/// How far `x` is from `from` to `to`, 0 to 1.
fn ramp_t(x: f64, from: f64, to: f64) -> f64 {
    if to - from <= 1e-9 {
        1.0
    } else {
        ((x - from) / (to - from)).clamp(0.0, 1.0)
    }
}

fn lerp(a: u8, b: u8, t: f64) -> u8 {
    let t = t.clamp(0.0, 1.0);
    (f64::from(a) + (f64::from(b) - f64::from(a)) * t).round() as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A two-way road, 7 m kerb to kerb (two 3 m lanes inside 0.5 m of tarmac), along
    /// y = 0 from x = 0 to 40: the campus's commonest.
    fn two_way() -> Road {
        Road {
            name: "two-way".to_owned(),
            points: vec![(0.0, 0.0), (20.0, 0.0), (40.0, 0.0)],
            width: 7.0,
            lanes: 0,
            oneway: false,
            edge_inset: EDGE_INSET,
        }
    }

    fn lanes(road: Road) -> Lanes {
        Lanes::new(vec![road], Ramp::default()).expect("a good road")
    }

    /// The cost at (x, y) on a 0.25 m map 60 m x 20 m round the road.
    fn drawn(l: &Lanes) -> Grid {
        let mut g = Grid::new((-10.0, -10.0), 0.25, 240, 80, 0);
        let all = Bounds {
            x0: 0,
            y0: 0,
            x1: 240,
            y1: 80,
        };
        l.draw(&mut g, all);
        g
    }

    /// The most a 0.25 m cell's centre can be from the lane's (0.125 m of 1.5 m: line / 12).
    const LOW: u8 = 7;

    fn at(g: &Grid, x: f64, y: f64) -> u8 {
        g.cost_at(x, y, 0)
    }

    #[test]
    fn the_lane_is_lane_maps() {
        // lane_map.py's lane_offset for the same roads.
        assert_eq!(two_way().lane(), (1.5, 3.0));
        let one = Road {
            width: 4.5,
            lanes: 1,
            ..two_way()
        };
        assert_eq!(one.lane(), (0.0, 3.5));
        let oneway = Road {
            width: 10.5,
            oneway: true,
            ..two_way()
        };
        assert_eq!(oneway.lane_count(), 3);
        let (c, w) = oneway.lane();
        assert!((w - 9.5 / 3.0).abs() < 1e-9 && (c - (4.75 - w / 2.0)).abs() < 1e-9);
    }

    #[test]
    fn driven_along_the_right_lane_is_lowest() {
        let mut l = lanes(two_way());
        assert!(l.set_route(&[(0.0, 0.0), (40.0, 0.0)]).is_some());
        // Driving east, right is south: the lane's centre at y = -1.5.
        let g = drawn(&l);
        assert!(at(&g, 10.0, -1.5) <= LOW, "the lane centre");
        let r = Ramp::default();
        // Its lines: the centre line (y = 0) and the edge line (y = -3).
        assert!((i32::from(at(&g, 10.0, -0.05)) - i32::from(r.line)).abs() <= 7);
        assert!((i32::from(at(&g, 10.0, -2.95)) - i32::from(r.line)).abs() <= 7);
        // The oncoming lane, the kerb, off the road, then nothing.
        assert_eq!(at(&g, 10.0, 1.5), r.wrong_lane);
        assert!(at(&g, 10.0, -3.4) > r.line && at(&g, 10.0, -3.4) <= r.kerb);
        assert_eq!(at(&g, 10.0, 4.2), r.off_road);
        assert_eq!(at(&g, 10.0, -6.0), 0);
        // Rising all the way from the centre to the right kerb.
        let across: Vec<u8> = (0..=8)
            .map(|i| at(&g, 10.0, -1.5 - 0.25 * f64::from(i)))
            .collect();
        assert!(across.windows(2).all(|p| p[0] <= p[1]), "{across:?}");
    }

    #[test]
    fn the_other_way_swaps_the_lanes() {
        let mut l = lanes(two_way());
        l.set_route(&[(40.0, 0.0), (0.0, 0.0)]);
        assert_eq!(l.way(0, 0), Some(Way::Against));
        let g = drawn(&l);
        assert!(at(&g, 10.0, 1.5) <= LOW, "the lane centre");
        assert_eq!(at(&g, 10.0, -1.5), Ramp::default().wrong_lane);
    }

    #[test]
    fn without_a_route_both_lanes_are_low() {
        let g = drawn(&lanes(two_way()));
        assert!(at(&g, 10.0, 1.5) <= LOW, "the lane centre");
        assert!(at(&g, 10.0, -1.5) <= LOW, "the lane centre");
        assert!(at(&g, 10.0, 0.0) >= Ramp::default().line - 7);
        // A one-way road keeps its own right lane.
        let oneway = Road {
            oneway: true,
            ..two_way()
        };
        let g = drawn(&lanes(oneway));
        assert!(at(&g, 10.0, -1.5) <= LOW, "the lane centre");
        assert_eq!(at(&g, 10.0, 1.5), Ramp::default().line);
    }

    #[test]
    fn a_route_across_the_road_says_nothing() {
        let mut l = lanes(two_way());
        assert_eq!(l.set_route(&[(10.0, -8.0), (10.0, 8.0)]), None);
        assert_eq!(l.way(0, 0), Some(Way::Unknown));
        // Only the half it runs along takes a way; the change covers that road.
        let changed = l
            .set_route(&[(25.0, -1.5), (39.0, -1.5)])
            .expect("a change");
        assert_eq!(
            (l.way(0, 0), l.way(0, 1)),
            (Some(Way::Unknown), Some(Way::Along))
        );
        assert!(changed.0 <= -4.5 && changed.2 >= 44.5, "{changed:?}");
        // The same route again changes nothing.
        assert_eq!(l.set_route(&[(25.0, -1.5), (39.0, -1.5)]), None);
    }

    #[test]
    fn an_area_draws_as_the_whole_does() {
        let mut l = lanes(two_way());
        l.set_route(&[(0.0, 0.0), (40.0, 0.0)]);
        let whole = drawn(&l);
        let mut g = whole.clone();
        let area = Bounds {
            x0: 50,
            y0: 20,
            x1: 130,
            y1: 70,
        };
        g.data.iter_mut().for_each(|c| *c = 7);
        l.draw(&mut g, area);
        for cy in area.y0..area.y1 {
            for cx in area.x0..area.x1 {
                assert_eq!(g.get(cx, cy), whole.get(cx, cy));
            }
        }
        assert_eq!(g.get(0, 0), 7, "outside the area untouched");
    }

    #[test]
    fn obstacles_and_unknown_stay() {
        let mut master = vec![
            0,
            10,
            200,
            210,
            252,
            INSCRIBED,
            crate::cost::LETHAL,
            NO_INFORMATION,
        ];
        combine(&mut master, &[50; 8]);
        // Added, never past MAX_SUM; obstacles and unknown untouched.
        assert_eq!(
            master,
            vec![
                50,
                60,
                250,
                MAX_SUM,
                MAX_SUM,
                INSCRIBED,
                crate::cost::LETHAL,
                NO_INFORMATION
            ]
        );
    }

    #[test]
    fn between_two_inflated_kerbs_the_right_lane_is_still_lowest() {
        // The campus: the lidar marks both kerbs of a 7 m road; the global inflation
        // (3 m, scaling 1, inscribed 0.55 m) is lowest in the road's middle.
        let mut l = lanes(two_way());
        l.set_route(&[(0.0, 0.0), (40.0, 0.0)]);
        let road = two_way();
        let inflation = |d: f64| -> u8 {
            if d > 3.0 {
                0
            } else {
                (252.0 * (-(d - 0.55)).exp()).min(252.0) as u8
            }
        };
        let total = |s: f64| {
            let mut m = [inflation(3.5 - s).max(inflation(3.5 + s))];
            let lane = l.cost_across(&road, s, Way::Along).unwrap_or(0);
            combine(&mut m, &[lane]);
            m[0]
        };
        // s: m right of the centre line. The cheapest place is inside the right lane.
        let across: Vec<(f64, u8)> = (-12..=12)
            .map(|i| (0.25 * f64::from(i), total(0.25 * f64::from(i))))
            .collect();
        let best = across.iter().min_by_key(|(_, c)| *c).map(|(s, _)| *s);
        assert!(
            matches!(best, Some(s) if (0.75..=2.25).contains(&s)),
            "{across:?}"
        );
        assert!(total(1.5) < total(0.0) && total(0.0) < total(-1.5));
    }

    #[test]
    fn bad_maps_are_refused() {
        let short = Road {
            points: vec![(0.0, 0.0)],
            ..two_way()
        };
        assert!(matches!(
            Lanes::new(vec![short], Ramp::default()),
            Err(LaneError::BadRoad(_))
        ));
        let wall = Ramp {
            kerb: INSCRIBED,
            ..Ramp::default()
        };
        assert!(matches!(
            Lanes::new(vec![two_way()], wall),
            Err(LaneError::BadRamp(_))
        ));
    }
}
