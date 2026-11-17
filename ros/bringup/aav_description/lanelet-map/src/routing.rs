//! The route level on the lanelets (TODO 175, step 6): where a pose meets a lane, the best
//! way by lanes between two such places, its line and the stop lines on it.
//!
//! ```text
//!  car ─▶ places_near ─▶ Place (lanelet, m along) ─┐
//!                                                  ├─▶ route_between (Dijkstra) ─▶ Route
//!  goal ─▶ places_near ─▶ Place ───────────────────┘        │
//!                                    route_line ◀───────────┤ the lanes' centres, joined
//!                                    stops_between ◀────────┘ the stop lines met
//! ```
//!
//! A change of lane keeps the car's place: it goes on from the same share of the way along
//! the lane beside, at [`LANE_CHANGE_COST`] m more, and the line eases across over
//! [`LANE_CHANGE_LENGTH`] m.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::{distance_to_line, Id, Lanelet, LaneletMap, Point, LANE_CHANGE_COST};

/// m over which a route's line eases from one lane's centre to the next's at a change.
pub const LANE_CHANGE_LENGTH: f64 = 20.0;

/// Where a point meets one lanelet's centre line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Place {
    /// The lanelet.
    pub lanelet: Id,
    /// m along its centre line from its start.
    pub along: f64,
    /// m from the point to the centre line.
    pub off: f64,
    /// rad: the way the lane runs there (x east, counter-clockwise).
    pub heading: f64,
    /// The nearest point on the centre line.
    pub foot: Point,
}

/// A way by lanes between two places ([`LaneletMap::route_between`]).
#[derive(Debug, Clone, PartialEq)]
pub struct Route {
    /// m driven, each lane change counted as [`LANE_CHANGE_COST`] m more.
    pub cost: f64,
    /// The lanelets in driving order: each one follows, or sits beside, the one before.
    pub lanelets: Vec<Id>,
    /// m along each lanelet where the route joins it: the start's place on the first,
    /// 0 on one that follows, the car's place on one changed into.
    pub entries: Vec<f64>,
}

/// A lanelet in the search's queue, the least cost first; `done` is the goal itself,
/// reached at this cost.
#[derive(Debug, PartialEq)]
struct Queued {
    cost: f64,
    id: Id,
    done: bool,
}

impl Eq for Queued {}

impl Ord for Queued {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .total_cmp(&self.cost)
            .then(self.done.cmp(&other.done))
            .then(other.id.cmp(&self.id))
    }
}

impl PartialOrd for Queued {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// m along a lanelet of `to_len` m at the same share of the way as `along` on one of
/// `from_len` m.
fn scaled(along: f64, from_len: f64, to_len: f64) -> f64 {
    if from_len > 0.0 {
        (along / from_len * to_len).clamp(0.0, to_len)
    } else {
        0.0
    }
}

/// Ease `line` (starting in the new lane, right after a change) in from `step` beside it:
/// moved by all of `step` at its start, none [`LANE_CHANGE_LENGTH`] m on (a smoothstep).
fn ease(line: &mut [Point], step: Point) {
    let mut s = 0.0;
    let mut before = line.first().copied();
    for p in line.iter_mut() {
        if let Some(b) = before {
            s += b.distance(*p);
        }
        before = Some(*p);
        if s >= LANE_CHANGE_LENGTH {
            break;
        }
        let w = s / LANE_CHANGE_LENGTH;
        let keep = 1.0 - w * w * (3.0 - 2.0 * w);
        p.x += keep * step.x;
        p.y += keep * step.y;
    }
}

/// The search's state: per lanelet, its best cost at its start (a point `s` m along it
/// costs that plus s), where the route joins it, and the lanelet before.
#[derive(Default)]
struct Search {
    best: HashMap<Id, f64>,
    entry: HashMap<Id, f64>,
    came: HashMap<Id, Id>,
    open: BinaryHeap<Queued>,
}

impl Search {
    /// Reach lanelet `n` at `cost` (at its start), joining it `at` m along, from `before`;
    /// the goal's own entry too, if `n` is the goal's lanelet and `at` isn't past it.
    fn reach(&mut self, n: Id, cost: f64, at: f64, before: Option<Id>, to: &Place) {
        if self.best.get(&n).is_some_and(|&old| cost >= old) {
            return;
        }
        self.best.insert(n, cost);
        self.entry.insert(n, at);
        match before {
            Some(b) => self.came.insert(n, b),
            None => self.came.remove(&n),
        };
        self.open.push(Queued {
            cost,
            id: n,
            done: false,
        });
        if n == to.lanelet && at <= to.along {
            self.open.push(Queued {
                cost: cost + to.along,
                id: n,
                done: true,
            });
        }
    }
}

impl LaneletMap {
    /// Where (x, y) meets lanelet `id`'s centre line; `None` with no such lanelet.
    #[must_use]
    pub fn place_on(&self, id: Id, x: f64, y: f64) -> Option<Place> {
        let l = self.lanelets.get(&id)?;
        let p = Point::new(x, y);
        let mut walked = 0.0;
        let mut best: Option<Place> = None;
        for w in l.centre.windows(2) {
            let (a, b) = (w[0], w[1]);
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let span = dx.hypot(dy);
            let t = if span > 0.0 {
                (((p.x - a.x) * dx + (p.y - a.y) * dy) / (span * span)).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let foot = Point::new(a.x + t * dx, a.y + t * dy);
            let off = p.distance(foot);
            if best.is_none_or(|b| off < b.off) {
                best = Some(Place {
                    lanelet: id,
                    along: walked + t * span,
                    off,
                    heading: dy.atan2(dx),
                    foot,
                });
            }
            walked += span;
        }
        best.or_else(|| {
            let only = *l.centre.first()?;
            Some(Place {
                lanelet: id,
                along: 0.0,
                off: p.distance(only),
                heading: 0.0,
                foot: only,
            })
        })
    }

    /// Every lanelet whose centre line passes within `within` m of (x, y), nearest first
    /// (of two as near, the smaller id).
    #[must_use]
    pub fn places_near(&self, x: f64, y: f64, within: f64) -> Vec<Place> {
        let mut ids: Vec<Id> = self
            .index
            .locate_within_distance([x, y], within * within)
            .map(|s| s.lanelet)
            .collect();
        ids.sort_unstable();
        ids.dedup();
        let mut places: Vec<Place> = ids
            .into_iter()
            .filter_map(|id| self.place_on(id, x, y))
            .collect();
        places.sort_by(|a, b| a.off.total_cmp(&b.off).then(a.lanelet.cmp(&b.lanelet)));
        places
    }

    /// The point `along` m down lanelet `id`'s centre line (clamped to its ends).
    #[must_use]
    pub fn point_at(&self, id: Id, along: f64) -> Option<Point> {
        let centre = &self.lanelets.get(&id)?.centre;
        let mut walked = 0.0;
        for w in centre.windows(2) {
            let span = w[0].distance(w[1]);
            if walked + span >= along && span > 0.0 {
                let t = ((along - walked) / span).clamp(0.0, 1.0);
                return Some(Point::new(
                    w[0].x + t * (w[1].x - w[0].x),
                    w[0].y + t * (w[1].y - w[0].y),
                ));
            }
            walked += span;
        }
        centre.last().copied()
    }

    /// m from lanelet `id`'s centre, `along` m down it, to its left bound and to its right.
    #[must_use]
    pub fn half_widths(&self, id: Id, along: f64) -> Option<(f64, f64)> {
        let l = self.lanelets.get(&id)?;
        let p = self.point_at(id, along)?;
        let side = |line: Id| {
            self.lines
                .get(&line)
                .map_or(0.0, |line| distance_to_line(p, &line.points))
        };
        Some((side(l.left), side(l.right)))
    }

    /// The best way by lanes from `from` to `to`, never backwards along a lane: Dijkstra
    /// over the lanelets, the cost the metres driven plus [`LANE_CHANGE_COST`] a change.
    /// On one lanelet with `to` ahead: that stretch alone. `None`: no way.
    #[must_use]
    pub fn route_between(&self, from: &Place, to: &Place) -> Option<Route> {
        if from.lanelet == to.lanelet && to.along >= from.along {
            return Some(Route {
                cost: to.along - from.along,
                lanelets: vec![from.lanelet],
                entries: vec![from.along],
            });
        }
        let length = |id: Id| self.lanelets.get(&id).map_or(0.0, Lanelet::length);
        let mut s = Search::default();
        // The start's own lanelet isn't a state: a goal behind on it is reached round.
        let start = -from.along;
        for &n in self.following(from.lanelet) {
            s.reach(n, start + length(from.lanelet), 0.0, None, to);
        }
        for &n in self.beside(from.lanelet) {
            let at = scaled(from.along, length(from.lanelet), length(n));
            s.reach(n, LANE_CHANGE_COST - at, at, None, to);
        }
        let mut found = None;
        while let Some(Queued { cost, id, done }) = s.open.pop() {
            let best = s.best.get(&id).copied().unwrap_or(f64::INFINITY);
            if done {
                // Still the goal's best entry (not one a cheaper way replaced).
                if (best + to.along - cost).abs() < 1e-9 {
                    found = Some(cost);
                    break;
                }
                continue;
            }
            if cost > best {
                continue;
            }
            let here = s.entry.get(&id).copied().unwrap_or(0.0);
            for &n in self.following(id) {
                s.reach(n, cost + length(id), 0.0, Some(id), to);
            }
            for &n in self.beside(id) {
                let at = scaled(here, length(id), length(n));
                // The cost at n's start: the cost here, the change, less n's part skipped.
                s.reach(n, cost + here + LANE_CHANGE_COST - at, at, Some(id), to);
            }
        }
        let cost = found?;
        let mut lanelets = vec![to.lanelet];
        while let Some(&before) = s.came.get(lanelets.last()?) {
            if lanelets.len() > self.lanelets.len() {
                return None;
            }
            lanelets.push(before);
        }
        let mut entries: Vec<f64> = lanelets
            .iter()
            .map(|id| s.entry.get(id).copied().unwrap_or(0.0))
            .collect();
        lanelets.push(from.lanelet);
        entries.push(from.along);
        lanelets.reverse();
        entries.reverse();
        Some(Route {
            cost,
            lanelets,
            entries,
        })
    }

    /// Does the route go from its `k`-th lanelet to the next by a change of lane?
    fn changes_after(&self, route: &Route, k: usize) -> bool {
        route
            .lanelets
            .get(k + 1)
            .is_some_and(|next| !self.following(route.lanelets[k]).contains(next))
    }

    /// A route's line, ending at `to`: the lanes' centre lines, a change of lane eased
    /// across over [`LANE_CHANGE_LENGTH`] m.
    #[must_use]
    pub fn route_line(&self, route: &Route, to: &Place) -> Vec<Point> {
        let n = route.lanelets.len();
        let mut line: Vec<Point> = Vec::new();
        // Each change: where in the line the new lane starts, and the step across.
        let mut changes: Vec<(usize, Point)> = Vec::new();
        for (k, (&id, &start)) in route.lanelets.iter().zip(&route.entries).enumerate() {
            let end = if k + 1 == n {
                to.along
            } else if self.changes_after(route, k) {
                // Changed out of where it joined: none of it driven.
                start
            } else {
                self.lanelet(id).map_or(0.0, Lanelet::length)
            };
            let piece = self.centre_between(id, start, end);
            if k > 0 && self.changes_after(route, k - 1) {
                // The lane before ended where the car was; this one starts beside it.
                if let (Some(q), Some(p)) = (line.pop(), piece.first()) {
                    changes.push((line.len(), Point::new(q.x - p.x, q.y - p.y)));
                }
            }
            for p in piece {
                if line.last().is_none_or(|q| q.distance(p) > 1e-3) {
                    line.push(p);
                }
            }
        }
        for (from, step) in changes {
            ease(&mut line[from..], step);
        }
        line
    }

    /// Lanelet `id`'s centre line from `start` to `end` m along: both ends, and the line's
    /// own points between (about a metre apart).
    fn centre_between(&self, id: Id, start: f64, end: f64) -> Vec<Point> {
        let Some(l) = self.lanelets.get(&id) else {
            return Vec::new();
        };
        let mut out: Vec<Point> = self.point_at(id, start).into_iter().collect();
        let mut walked = 0.0;
        for w in l.centre.windows(2) {
            walked += w[0].distance(w[1]);
            if walked > start && walked < end {
                out.push(w[1]);
            }
        }
        if end > start {
            out.extend(self.point_at(id, end));
        }
        out
    }

    /// The stop lines a route meets on its way to `to`, in order: (lanelet, rule, the
    /// line's middle). One behind where the route joins a lanelet, or past `to`, is left
    /// out.
    #[must_use]
    pub fn stops_between(&self, route: &Route, to: &Place) -> Vec<(Id, Id, Point)> {
        let last = route.lanelets.len().saturating_sub(1);
        self.stops_on(&route.lanelets)
            .into_iter()
            .filter(|&(id, _, p)| {
                let along = self.place_on(id, p.x, p.y).map_or(0.0, |pl| pl.along);
                route
                    .lanelets
                    .iter()
                    .zip(&route.entries)
                    .enumerate()
                    .any(|(k, (&l, &entry))| {
                        l == id && along + 1e-6 >= entry && (k < last || along <= to.along + 1e-6)
                    })
            })
            .collect()
    }
}
