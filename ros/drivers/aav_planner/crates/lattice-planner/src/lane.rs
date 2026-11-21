//! The lane-centre planner (`path:=lane`, TODO 148 and 169; the Fable review's change 7):
//! on a mapped road the path is the right lane's centre, found by routing along the lane
//! map; the lattice only joins the car to it and it to the goal. Autoware's idea (follow
//! the lane's centre, leave it only to go round), on our `lanes.yaml`.
//!
//! ```text
//!   car ─lattice─▶ entry ════ the right lane's centre, by the road graph ════ exit ─lattice─▶ goal
//!         (JOIN m along the lane)                                         (JOIN m before)
//! ```
//!
//! | Step | What |
//! |---|---|
//! | the graph | each road's centre-line points are nodes; a segment is an edge each way the road runs (one way: its drawn way). Roads join where they share a point (OpenStreetMap's shared nodes) |
//! | where on it | the start and the goal on their nearest segment: on the road, the way they face; off it but within [`LaneSettings::reach`], either way |
//! | the route | Dijkstra, shortest along the centre lines |
//! | the line | each segment moved right by its lane's centre (`Road::lane`, as the lanes layer and `lane_map.py`), corners rounded at the planner's turning radius: circle arcs, or with `lane_corners:=clothoid` clothoid, arc, clothoid (continuous curvature, [`LaneSettings::transition`]; a challenger, TODO 211) |
//! | the ends | the lattice, from the car to the line `join` m on, and from `join` m before its end to the goal; a car already on its lane (within [`LaneSettings::snap`]) straight onto the line `ease` m ahead, and a goal on the line's end straight too |
//!
//! Something parked on the line (Ben, 16 Nov: keep right, leave the lane only to pass):
//! only the blocked stretch moves left, by the least that clears it, and back
//! ([`crate::pass`]); the rest stays the lane's centre ([`How::Pass`]).
//!
//! It falls back to the lattice for the whole path, and says why ([`Fallback`]), when:
//! the start or the goal is off the lane map (the lot), the route is too short for both
//! joins, the line is blocked with no pass (the other lane taken too, or the goal in the
//! blocked stretch), or a join has no path.
//!
//! A goal off the costmap (a global costmap made of the chunks round the car, TODO 175):
//! the lane line still runs the whole way, but the path stops on it where it leaves the
//! costmap ([`crate::steps`]), and the next plan goes on from there ([`LanePlan::cut`]).

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use costmap::{Grid, Road};
use lanelet_map::{LaneletMap, Place};

use crate::jerk_pass::{self, JerkSettings};
use crate::pass::{self, PassSettings};
use crate::steps::{self, EDGE_MARGIN};
use crate::{curves::wrap, Plan, PlanError, Planner, Pose};

/// m past a road's kerb a pose still counts as on that road (`lane_map.py` MARGIN).
const ON_ROAD_MARGIN: f64 = 0.5;
/// A join from off the lanes to them, in metres of lane: each metre off counts this much
/// (Lanelet2: the lane picked near an off-road start or goal).
const JOIN_WEIGHT: f64 = 2.0;
/// m of clothoid at each end of a corner with `lane_corners:=clothoid` (TODO 211): the
/// steering actuator turns 0.8 rad/s (vehicle.yaml), about 0.5 1/m of curvature a second
/// on the 1.56 m wheelbase; 4 m of clothoid up to the 3.6 m radius asks a third of that at
/// 2.5 m/s.
pub const CLOTHOID_TRANSITION: f64 = 4.0;
/// m: the sideways step the joins must make room for: one lane.
const ONE_LANE: f64 = 3.5;
/// m a pass keeps from the left kerb, beyond half the car.
const KERB_MARGIN: f64 = 0.25;

/// How the lane planner joins and when it gives up on the lane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaneSettings {
    /// m along the lane line from the car's place on it to where the lattice joins it
    /// (and from the join before the goal to the goal's place). At least an S-bend of one
    /// lane at the turning radius, plus 2 m.
    pub join: f64,
    /// m: a start or goal off every road, but this near one's centre line, still joins it.
    pub reach: f64,
    /// m: a lane stretch between the two joins shorter than this isn't worth it.
    pub min_lane: f64,
    /// m between the lane line's poses.
    pub spacing: f64,
    /// m of clothoid at each end of a corner, from straight up to the turning radius's
    /// curvature (TODO 211, [`CLOTHOID_TRANSITION`]); 0 (the default): circle arcs, the
    /// curvature jumping. The clothoids are a challenger: on University Drive they tied.
    pub transition: f64,
    /// m and rad: a car (or goal) this near the lane line's end, and facing its way, is
    /// joined to it straight, not by the lattice: the path starts at the car and goes on
    /// at the line `ease` m ahead (the goal: the line's end, then the goal).
    pub snap: (f64, f64),
    /// m along the line where a snapped car's path meets it.
    pub ease: f64,
    /// A lane route longer than `detour.0` times the straight distance plus `detour.1` m
    /// is a loop round the block to reach a goal close by: the lattice's instead.
    pub detour: (f64, f64),
    /// The same on a Lanelet2 map, tighter: its lanes run one way each, so a goal across
    /// the road is reached round the block more often than on lanes.yaml's two-way roads.
    pub detour_lanelets: (f64, f64),
    /// Something parked on the line: pass it ([`crate::pass`]); None: the lattice, all of
    /// it, as before 16 Nov.
    pub pass: Option<PassSettings>,
    /// The pass's shape: None, the half cosine ([`crate::pass`]); Some, the jerk-limited
    /// QP ([`crate::jerk_pass`], TODO 214, a challenger) at this speed (m/s).
    pub pass_jerk: Option<(JerkSettings, f64)>,
}

impl Default for LaneSettings {
    /// 16 Nov: the first sim runs joined the car by the lattice at every replan (once a
    /// second), and on the bends the lattice backed up now and then (4 reversals a leg);
    /// a car already on its lane is now joined straight.
    fn default() -> Self {
        Self {
            join: 8.0,
            reach: 20.0,
            min_lane: 5.0,
            spacing: 0.25,
            transition: 0.0,
            snap: (0.6, 20f64.to_radians()),
            ease: 2.0,
            // 16 Nov (TODO 175): on the campus tour a replan 10 m short of the goal (off
            // the road, at 45 degrees to it) gave a 430 m loop of lanes; the car drove it.
            detour: (2.0, 40.0),
            // 16 Nov (TODO 175): replans along the campus legs on Lanelet2 gave 153-234 m
            // loops to goals 44-103 m away, inside (2, 40); the sim drove them.
            detour_lanelets: (1.5, 20.0),
            pass: Some(PassSettings::default()),
            pass_jerk: None,
        }
    }
}

/// Why the whole path is the lattice's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fallback {
    /// The start or the goal is off the lane map, or no route joins them.
    OffMap,
    /// The route is too short for the two joins and a stretch of lane.
    Short,
    /// The lane line runs over an obstacle (lethal or inscribed), and no pass fits (or
    /// passing is off): the lattice goes round.
    Blocked,
    /// The lattice found no join, at one end or the other.
    JoinFailed,
    /// The lane route loops far round to a goal close by ([`LaneSettings::detour`]).
    Detour,
}

impl Fallback {
    /// One word, for the status.
    #[must_use]
    pub const fn word(self) -> &'static str {
        match self {
            Self::OffMap => "off_map",
            Self::Short => "short",
            Self::Blocked => "blocked",
            Self::JoinFailed => "join_failed",
            Self::Detour => "detour",
        }
    }
}

/// How a path was made.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum How {
    /// Along the lane: this many metres of it between the two joins.
    Lane(f64),
    /// Along the lane, passing something parked on it: the metres, and the most it moved
    /// left (m).
    Pass(f64, f64),
    /// The lattice's, all of it, and why.
    Lattice(Fallback),
}

/// A path and how it was made.
#[derive(Debug, Clone)]
pub struct LanePlan {
    /// The path, as [`Planner::plan`] gives it.
    pub plan: Plan,
    /// The lane, or the lattice and why.
    pub how: How,
    /// The goal is off the costmap: the path ends where the way there leaves it.
    pub cut: bool,
    /// The route level's whole way to the goal: the lane line, or start and goal off it.
    pub route: Vec<Pose>,
    /// The stop lines the route meets (map frame; a Lanelet2 map only), in order.
    pub stops: Vec<(f64, f64)>,
}

/// Where a pose meets the lane map: a point on one road's centre line, and the way along.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Anchor {
    road: usize,
    segment: usize,
    /// 0 to 1 along the segment, in its drawn direction.
    t: f64,
    foot: (f64, f64),
    /// The drawn way (else against it).
    along: bool,
}

#[derive(Debug, Clone, Copy)]
struct Edge {
    from: usize,
    to: usize,
    road: usize,
    length: f64,
}

/// The lane map as a graph, and how to join it.
#[derive(Debug, Clone)]
pub struct LaneRouter {
    settings: LaneSettings,
    graph: Graph,
}

/// The route level's map (`lane_graph:=`).
#[derive(Debug, Clone)]
enum Graph {
    /// `lanes.yaml`: each road's centre line, the right lane offset from it.
    Roads(RoadGraph),
    /// Lanelet2: each lane its own lanelet, with followers, neighbours and stop lines.
    Lanelets(Box<LaneletMap>),
}

/// `lanes.yaml`'s roads as a graph of their centre-line points.
#[derive(Debug, Clone)]
struct RoadGraph {
    roads: Vec<Road>,
    /// Every centre-line point, once.
    nodes: Vec<(f64, f64)>,
    /// node_of[road][point].
    node_of: Vec<Vec<usize>>,
    /// Each way along each segment.
    edges: Vec<Edge>,
    /// out[node]: the edges leaving it.
    out: Vec<Vec<usize>>,
}

/// A point's key: centimetres (roads that share a point join there).
fn key(p: (f64, f64)) -> (i64, i64) {
    ((p.0 * 100.0).round() as i64, (p.1 * 100.0).round() as i64)
}

/// The nearest point to `p` on segment a-b: (t, foot, distance).
fn foot(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> (f64, (f64, f64), f64) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let n2 = dx * dx + dy * dy;
    let t = if n2 > 1e-12 {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / n2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let f = (a.0 + t * dx, a.1 + t * dy);
    (t, f, (p.0 - f.0).hypot(p.1 - f.1))
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    (b.0 - a.0).hypot(b.1 - a.1)
}

/// A line, and the stop lines met on it (map frame).
type WithStops<P> = (Vec<P>, Vec<(f64, f64)>);

/// A route: its metres, and its points each with the road (index) it was reached on.
type Route = (f64, Vec<((f64, f64), usize)>);

/// The nearest segment: its distance, road, segment, how far along (0-1), and the foot.
type Nearest = (f64, usize, usize, f64, (f64, f64));

/// Dijkstra's queue entry: the smallest cost first.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Queued(f64, usize);

impl Eq for Queued {}

impl Ord for Queued {
    fn cmp(&self, other: &Self) -> Ordering {
        other.0.total_cmp(&self.0).then(other.1.cmp(&self.1))
    }
}

impl PartialOrd for Queued {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl LaneRouter {
    /// The graph of `roads` (each with two or more points; others are skipped).
    #[must_use]
    pub fn new(roads: Vec<Road>, settings: LaneSettings) -> Self {
        let roads: Vec<Road> = roads.into_iter().filter(|r| r.points.len() >= 2).collect();
        let mut index: HashMap<(i64, i64), usize> = HashMap::new();
        let mut nodes = Vec::new();
        let node_of: Vec<Vec<usize>> = roads
            .iter()
            .map(|r| {
                r.points
                    .iter()
                    .map(|&p| {
                        *index.entry(key(p)).or_insert_with(|| {
                            nodes.push(p);
                            nodes.len() - 1
                        })
                    })
                    .collect()
            })
            .collect();
        let mut edges = Vec::new();
        let mut out = vec![Vec::new(); nodes.len()];
        for (ri, (road, ids)) in roads.iter().zip(&node_of).enumerate() {
            for (pair, ps) in ids.windows(2).zip(road.points.windows(2)) {
                if pair[0] == pair[1] {
                    continue;
                }
                let length = distance(ps[0], ps[1]);
                let ways: &[(usize, usize)] = if road.oneway {
                    &[(0, 1)]
                } else {
                    &[(0, 1), (1, 0)]
                };
                for &(a, b) in ways {
                    out[pair[a]].push(edges.len());
                    edges.push(Edge {
                        from: pair[a],
                        to: pair[b],
                        road: ri,
                        length,
                    });
                }
            }
        }
        Self {
            settings,
            graph: Graph::Roads(RoadGraph {
                roads,
                nodes,
                node_of,
                edges,
                out,
            }),
        }
    }

    /// The route level on a Lanelet2 map (`lane_graph:=lanelet2`, TODO 175): routed by
    /// lanes (A lanelet's followers, its neighbours over a dashed line, one way each), the
    /// line each lane's own centre, the stop lines on the way found.
    #[must_use]
    pub fn from_lanelets(map: LaneletMap, settings: LaneSettings) -> Self {
        Self {
            settings,
            graph: Graph::Lanelets(Box::new(map)),
        }
    }

    /// The roads (`lanes.yaml`); none on a Lanelet2 map.
    #[must_use]
    pub fn roads(&self) -> &[Road] {
        match &self.graph {
            Graph::Roads(g) => &g.roads,
            Graph::Lanelets(_) => &[],
        }
    }

    /// What it routes on, for the log: "N roads (lanes.yaml)" or "N lanelets (Lanelet2)".
    #[must_use]
    pub fn describe(&self) -> String {
        match &self.graph {
            Graph::Roads(g) => format!("{} roads (lanes.yaml)", g.roads.len()),
            Graph::Lanelets(m) => format!("{} lanelets (Lanelet2)", m.lanelets().count()),
        }
    }

    /// The settings.
    #[must_use]
    pub fn settings(&self) -> LaneSettings {
        self.settings
    }

    /// The lane's centre from `start` to `goal` (before rounding), and the stop lines met
    /// on it (Lanelet2 only); None off the lane map.
    fn centre_line(&self, start: &Pose, goal: &Pose) -> Option<WithStops<(f64, f64)>> {
        match &self.graph {
            Graph::Roads(g) => g
                .centre_line(start, goal, self.settings.reach)
                .map(|l| (l, Vec::new())),
            Graph::Lanelets(m) => lanelet_line(m, start, goal, self.settings.reach),
        }
    }

    /// m left of the lane's centre the car may go near `p` (see [`RoadGraph::room_left`]).
    fn room_left(&self, p: &Pose, half_width: f64) -> f64 {
        match &self.graph {
            Graph::Roads(g) => g.room_left(p, half_width),
            Graph::Lanelets(m) => lanelet_room_left(m, p, half_width),
        }
    }

    /// The right lane's centre from `start` to `goal`, a pose every
    /// [`LaneSettings::spacing`] m, its corners rounded at `radius` m; None off the lane map.
    #[must_use]
    pub fn lane_line(&self, start: &Pose, goal: &Pose, radius: f64) -> Option<Vec<Pose>> {
        self.line_and_stops(start, goal, radius)
            .map(|(line, _)| line)
    }

    /// [`LaneRouter::lane_line`], and the stop lines met on the way (Lanelet2 only).
    fn line_and_stops(&self, start: &Pose, goal: &Pose, radius: f64) -> Option<WithStops<Pose>> {
        let (points, stops) = self.centre_line(start, goal)?;
        if points.len() < 2 {
            return None;
        }
        let rounded = round_corners(
            &points,
            radius,
            self.settings.transition,
            self.settings.spacing,
        );
        Some((resample(&rounded, self.settings.spacing), stops))
    }
}

/// On a Lanelet2 map, where a pose meets the lanes: the lanes within `reach`, nearest
/// first. On a lane (within its half width and half a metre): the lanes there running the
/// way the pose faces (at a junction, the lanes through it overlap; the route picks).
/// Off every lane: every lane near (up to 12), either way; the route picks, counting the
/// join (`JOIN_WEIGHT`).
fn lanelet_places(m: &LaneletMap, pose: &Pose, reach: f64) -> Vec<Place> {
    let near = m.places_near(pose.x, pose.y, reach);
    let facing = |p: &Place| wrap(pose.yaw - p.heading).abs() <= std::f64::consts::FRAC_PI_2;
    let on: Vec<Place> = near
        .iter()
        .filter(|p| {
            let (left, right) = m.half_widths(p.lanelet, p.along).unwrap_or((1.75, 1.75));
            p.off <= left.max(right) + ON_ROAD_MARGIN
        })
        .copied()
        .collect();
    if !on.is_empty() {
        // On the road: the lanes running the way the pose faces.
        return on.into_iter().filter(facing).take(6).collect();
    }
    // Off every lane: each lane near, either way; the route picks, its join counted too.
    near.into_iter().take(12).collect()
}

/// The lanes' centre from `start` to `goal` on a Lanelet2 map, and the stop lines met;
/// None off the map or with no way by lanes.
fn lanelet_line(
    m: &LaneletMap,
    start: &Pose,
    goal: &Pose,
    reach: f64,
) -> Option<WithStops<(f64, f64)>> {
    let froms = lanelet_places(m, start, reach);
    let tos = lanelet_places(m, goal, reach);
    // A join off the lane costs twice its length: the lattice's way there is no straight
    // line. A lane passing 10 m from an off-road goal beats a loop to one 6 m from it.
    let key = |(r, t): &(lanelet_map::Route, Place)| {
        r.cost
            + JOIN_WEIGHT
                * (t.off
                    + froms
                        .iter()
                        .find(|f| f.lanelet == r.lanelets[0])
                        .map_or(0.0, |f| f.off))
    };
    let (route, to) = froms
        .iter()
        .flat_map(|f| {
            tos.iter()
                .filter_map(move |t| Some((m.route_between(f, t)?, *t)))
        })
        .min_by(|a, b| key(a).total_cmp(&key(b)))?;
    let line = m
        .route_line(&route, &to)
        .iter()
        .map(|p| (p.x, p.y))
        .collect();
    let stops = m
        .stops_between(&route, &to)
        .iter()
        .map(|(_, _, p)| (p.x, p.y))
        .collect();
    Some((line, stops))
}

/// m left of the lane's centre the car may go near `p` on a Lanelet2 map: across its own
/// lane and every lane beside on its left (either way, side by side), to the last one's
/// far edge, less `half_width` and [`KERB_MARGIN`]. 0 off every lane.
fn lanelet_room_left(m: &LaneletMap, p: &Pose, half_width: f64) -> f64 {
    let Some(here) = m.places_near(p.x, p.y, ONE_LANE).into_iter().next() else {
        return 0.0;
    };
    let (left, _) = m
        .half_widths(here.lanelet, here.along)
        .unwrap_or((0.0, 0.0));
    // Step left a lane at a time: a lane there, side by side with this one, widens the road.
    let (nx, ny) = (-here.heading.sin(), here.heading.cos());
    let mut edge = left;
    for _ in 0..4 {
        let (x, y) = (
            here.foot.x + nx * (edge + 0.5),
            here.foot.y + ny * (edge + 0.5),
        );
        let beside = m.places_near(x, y, ONE_LANE).into_iter().find(|q| {
            let (a, b) = m.half_widths(q.lanelet, q.along).unwrap_or((0.0, 0.0));
            q.lanelet != here.lanelet
                && q.off <= a.max(b) + 0.1
                && wrap(q.heading - here.heading)
                    .abs()
                    .min(std::f64::consts::PI - wrap(q.heading - here.heading).abs())
                    < 0.35
        });
        let Some(q) = beside else { break };
        let (a, b) = m.half_widths(q.lanelet, q.along).unwrap_or((0.0, 0.0));
        let centre = (q.foot.x - here.foot.x) * nx + (q.foot.y - here.foot.y) * ny;
        let further = centre
            + if wrap(q.heading - here.heading).abs() < 1.0 {
                a
            } else {
                b
            };
        if further <= edge + 0.1 {
            break;
        }
        edge = further;
    }
    (edge - half_width - KERB_MARGIN).max(0.0)
}

impl RoadGraph {
    /// Where `pose` meets the lane map: on a road, the way it faces (none if that's
    /// against a one-way road); off every road but within `reach`, both ways (a two-way
    /// road) or the road's way.
    fn anchors(&self, pose: &Pose, reach: f64) -> Vec<Anchor> {
        let p = (pose.x, pose.y);
        let mut best: Option<Nearest> = None;
        for (ri, road) in self.roads.iter().enumerate() {
            for (si, w) in road.points.windows(2).enumerate() {
                let (t, f, d) = foot(p, w[0], w[1]);
                if best.is_none_or(|b| d < b.0) {
                    best = Some((d, ri, si, t, f));
                }
            }
        }
        let Some((d, road, segment, t, foot)) = best else {
            return Vec::new();
        };
        let r = &self.roads[road];
        let (a, b) = (r.points[segment], r.points[segment + 1]);
        let heading = (b.1 - a.1).atan2(b.0 - a.0);
        let anchor = |along| Anchor {
            road,
            segment,
            t,
            foot,
            along,
        };
        if d <= r.width / 2.0 + ON_ROAD_MARGIN {
            let along = wrap(pose.yaw - heading).abs() <= std::f64::consts::FRAC_PI_2;
            if !along && r.oneway {
                return Vec::new();
            }
            vec![anchor(along)]
        } else if d <= reach {
            if r.oneway {
                vec![anchor(true)]
            } else {
                vec![anchor(true), anchor(false)]
            }
        } else {
            Vec::new()
        }
    }

    /// The node an anchor drives toward, the metres to it, and the node behind (the
    /// anchor's segment's other end: no U-turn back over it).
    fn leaving(&self, a: &Anchor) -> (usize, f64, usize) {
        let r = &self.roads[a.road];
        let length = distance(r.points[a.segment], r.points[a.segment + 1]);
        let (p, q) = (
            self.node_of[a.road][a.segment],
            self.node_of[a.road][a.segment + 1],
        );
        if a.along {
            (q, (1.0 - a.t) * length, p)
        } else {
            (p, a.t * length, q)
        }
    }

    /// The node an anchor is reached from, the metres from it, and the node beyond (the
    /// route mustn't come from there: that would pass the goal and turn back).
    fn arriving(&self, a: &Anchor) -> (usize, f64, usize) {
        let r = &self.roads[a.road];
        let length = distance(r.points[a.segment], r.points[a.segment + 1]);
        let (p, q) = (
            self.node_of[a.road][a.segment],
            self.node_of[a.road][a.segment + 1],
        );
        if a.along {
            (p, a.t * length, q)
        } else {
            (q, (1.0 - a.t) * length, p)
        }
    }

    /// The shortest route from `from` to `to` along the centre lines, never turning back
    /// on a segment (Dijkstra over the segments, not the points): its metres, and its
    /// points each with the road it was reached on.
    fn route_between(&self, from: &Anchor, to: &Anchor) -> Option<Route> {
        if from.road == to.road && from.segment == to.segment && from.along == to.along {
            let ahead = if from.along {
                to.t >= from.t
            } else {
                to.t <= from.t
            };
            if ahead {
                let r = &self.roads[from.road];
                let length = distance(r.points[from.segment], r.points[from.segment + 1]);
                return Some((
                    (to.t - from.t).abs() * length,
                    vec![(from.foot, from.road), (to.foot, to.road)],
                ));
            }
        }
        let (source, first, behind) = self.leaving(from);
        let (target, last, beyond) = self.arriving(to);
        let start = (from.foot, from.road);
        let end = (to.foot, to.road);
        if source == target && behind != beyond {
            return Some((
                first + last,
                vec![start, (self.nodes[source], from.road), end],
            ));
        }
        let mut cost = vec![f64::INFINITY; self.edges.len()];
        let mut came: Vec<Option<usize>> = vec![None; self.edges.len()];
        let mut queue = BinaryHeap::new();
        for &e in &self.out[source] {
            if self.edges[e].to != behind {
                cost[e] = first + self.edges[e].length;
                queue.push(Queued(cost[e], e));
            }
        }
        let mut found = None;
        while let Some(Queued(c, e)) = queue.pop() {
            if c > cost[e] {
                continue;
            }
            let edge = self.edges[e];
            if edge.to == target && edge.from != beyond {
                found = Some(e);
                break;
            }
            for &n in &self.out[edge.to] {
                let next_edge = self.edges[n];
                if next_edge.to == edge.from {
                    continue;
                }
                let next = c + next_edge.length;
                if next < cost[n] {
                    cost[n] = next;
                    came[n] = Some(e);
                    queue.push(Queued(next, n));
                }
            }
        }
        let found = found?;
        let mut chain = Vec::new();
        let mut e = Some(found);
        while let Some(i) = e {
            chain.push((self.nodes[self.edges[i].to], self.edges[i].road));
            e = came[i];
        }
        chain.push((self.nodes[source], from.road));
        chain.push(start);
        chain.reverse();
        chain.push(end);
        chain.dedup_by(|b, a| distance(a.0, b.0) < 1e-3);
        Some((cost[found] + last, chain))
    }

    /// m left of the right lane's centre the car may go near `p`: to the left kerb of the
    /// nearest road, less `half_width` and [`KERB_MARGIN`]. 0 off every road.
    fn room_left(&self, p: &Pose, half_width: f64) -> f64 {
        let nearest = self
            .roads
            .iter()
            .flat_map(|r| {
                r.points
                    .windows(2)
                    .map(move |w| (foot((p.x, p.y), w[0], w[1]).2, r))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0));
        nearest.map_or(0.0, |(_, r)| {
            (r.lane().0 + r.width / 2.0 - half_width - KERB_MARGIN).max(0.0)
        })
    }

    /// The right lane's centre from `start` to `goal`, corners not yet rounded; None off
    /// the lane map.
    fn centre_line(&self, start: &Pose, goal: &Pose, reach: f64) -> Option<Vec<(f64, f64)>> {
        let froms = self.anchors(start, reach);
        let tos = self.anchors(goal, reach);
        let best = froms
            .iter()
            .flat_map(|f| tos.iter().filter_map(move |t| self.route_between(f, t)))
            .min_by(|a, b| a.0.total_cmp(&b.0))?;
        let centre = best.1;
        if centre.len() < 2 {
            return None;
        }
        let offsets: Vec<f64> = centre[1..]
            .iter()
            .map(|(_, road)| self.roads[*road].lane().0)
            .collect();
        let points: Vec<(f64, f64)> = centre.iter().map(|(p, _)| *p).collect();
        Some(offset(&points, &offsets))
    }
}

impl LaneRouter {
    /// A path from `start` to `goal`: the lane's centre joined by `planner` at both ends, or
    /// `planner`'s alone (and why). `cancelled` as [`Planner::plan`].
    ///
    /// # Errors
    ///
    /// The lattice's [`PlanError`] when it plans the whole path and finds none.
    pub fn plan(
        &self,
        planner: &Planner,
        grid: &Grid,
        start: &Pose,
        goal: &Pose,
        cancelled: &dyn Fn() -> bool,
    ) -> Result<LanePlan, PlanError> {
        let lattice = |why, route: Vec<Pose>| {
            steps::plan_toward(planner, grid, start, goal, cancelled).map(|(plan, cut)| LanePlan {
                plan,
                how: How::Lattice(why),
                cut,
                route,
                stops: Vec::new(),
            })
        };
        let radius = planner.turning_radius();
        let Some((mut line, stops)) = self.line_and_stops(start, goal, radius) else {
            return lattice(Fallback::OffMap, vec![*start, *goal]);
        };
        let route = line.clone();
        let (times, plus) = match self.graph {
            Graph::Roads(_) => self.settings.detour,
            Graph::Lanelets(_) => self.settings.detour_lanelets,
        };
        let straight = (goal.x - start.x).hypot(goal.y - start.y);
        if arc_lengths(&line).last().copied().unwrap_or(0.0) > times * straight + plus {
            return lattice(Fallback::Detour, vec![*start, *goal]);
        }
        // A goal off the costmap: the line as far as it stays on it, the path ending there.
        let mut cut = false;
        if !steps::inside(grid, goal.x, goal.y, 0.0) {
            if let Some(k) = steps::last_inside(grid, &line, EDGE_MARGIN) {
                line.truncate(k + 1);
                cut = true;
            }
        }
        let lattice = |why| {
            lattice(why, route.clone()).map(|p| LanePlan {
                stops: stops.clone(),
                ..p
            })
        };
        // Something parked on the line: pass it, the rest the lane's centre.
        let mut passing = 0.0;
        if let Some(settings) = &self.settings.pass {
            let along = arc_lengths(&line);
            // Off the grid nothing is known: the search's own check decides there.
            let fits = |p: &Pose| {
                !steps::inside(grid, p.x, p.y, 0.0) || planner.fits(grid, p).unwrap_or(true)
            };
            let half = planner.half_width();
            let room = |p: &Pose| self.room_left(p, half);
            let passed = match &self.settings.pass_jerk {
                Some((jerk, speed)) => {
                    jerk_pass::plan(&line, &along, radius, *speed, settings, jerk, &fits, &room)
                }
                None => pass::plan(&line, &along, radius, settings, &fits, &room),
            };
            match passed {
                Ok(None) => {}
                Ok(Some(p)) => {
                    passing = p.shift();
                    line = p.line;
                }
                Err(pass::NoRoom) => return lattice(Fallback::Blocked),
            }
        }
        let target = if cut { line[line.len() - 1] } else { *goal };
        let join = self.settings.join.max(s_bend(radius, ONE_LANE) + 2.0);
        let along = arc_lengths(&line);
        let total = along.last().copied().unwrap_or(0.0);
        // On the line already (the car driving its lane, a goal at the lane's centre): a
        // straight join; else the lattice's, `join` m along.
        let (snap_m, snap_yaw) = self.settings.snap;
        let near = |p: &Pose, q: &Pose| {
            (p.x - q.x).hypot(p.y - q.y) <= snap_m && wrap(p.yaw - q.yaw).abs() <= snap_yaw
        };
        let snap_start = near(start, &line[0]);
        let snap_goal = cut || near(goal, &line[line.len() - 1]);
        let head_m = if snap_start { self.settings.ease } else { join };
        let tail_m = if snap_goal { 0.0 } else { join };
        if total - head_m - tail_m < self.settings.min_lane {
            return lattice(Fallback::Short);
        }
        let first = along.partition_point(|&s| s < head_m);
        let last = along
            .partition_point(|&s| s <= total - tail_m)
            .max(first + 1)
            - 1;
        let middle = &line[first..=last];
        let xy: Vec<(f64, f64)> = middle.iter().map(|p| (p.x, p.y)).collect();
        if !grid.blocked_along(&xy, 0).is_empty() {
            return lattice(Fallback::Blocked);
        }
        let (entry, exit) = (middle[0], middle[middle.len() - 1]);
        let straight = |from: &Pose, to: &Pose| {
            let mut poses = resample(&[(from.x, from.y), (to.x, to.y)], self.settings.spacing);
            if let Some(f) = poses.first_mut() {
                *f = *from;
            }
            if let Some(l) = poses.last_mut() {
                *l = *to;
            }
            Plan {
                raw: poses.clone(),
                poses,
                iterations: 0,
                search_time: 0.0,
                total_time: 0.0,
            }
        };
        let head = if snap_start {
            Ok(straight(start, &entry))
        } else {
            planner.plan(grid, start, &entry, cancelled)
        };
        let tail = if snap_goal {
            Ok(straight(&exit, &target))
        } else {
            planner.plan(grid, &exit, goal, cancelled)
        };
        let (Ok(head), Ok(tail)) = (head, tail) else {
            if cancelled() {
                return Err(PlanError::Cancelled);
            }
            return lattice(Fallback::JoinFailed);
        };
        let mut poses = head.poses.clone();
        poses.extend(middle.iter().skip(1));
        poses.extend(tail.poses.iter().skip(1));
        let mut raw = head.raw;
        raw.extend(middle.iter().skip(1));
        raw.extend(tail.raw.iter().skip(1));
        Ok(LanePlan {
            plan: Plan {
                poses,
                raw,
                iterations: head.iterations + tail.iterations,
                search_time: head.search_time + tail.search_time,
                total_time: head.total_time + tail.total_time,
            },
            how: if passing > 0.0 {
                How::Pass(along[last] - along[first], passing)
            } else {
                How::Lane(along[last] - along[first])
            },
            cut,
            route,
            stops,
        })
    }

    /// Through each of `goals` in turn, a [`LaneRouter::plan`] a leg, joined (as
    /// [`Planner::plan_through`]). How: the lane if any leg used it (its metres summed),
    /// else the first leg's fallback. A leg cut at the costmap's edge is the last.
    ///
    /// # Errors
    ///
    /// The failed leg's number (0 = start to the first goal) and its [`PlanError`].
    pub fn plan_through(
        &self,
        planner: &Planner,
        grid: &Grid,
        start: &Pose,
        goals: &[Pose],
        cancelled: &dyn Fn() -> bool,
    ) -> Result<LanePlan, (usize, PlanError)> {
        if goals.is_empty() {
            return Err((0, PlanError::NoValidPath));
        }
        let mut whole = Plan {
            poses: Vec::new(),
            raw: Vec::new(),
            iterations: 0,
            search_time: 0.0,
            total_time: 0.0,
        };
        let mut lane = 0.0;
        let mut shift: f64 = 0.0;
        let mut fallback = None;
        let mut from = *start;
        let mut cut = false;
        let mut route = Vec::new();
        let mut stops = Vec::new();
        for (leg, goal) in goals.iter().enumerate() {
            let p = self
                .plan(planner, grid, &from, goal, cancelled)
                .map_err(|e| (leg, e))?;
            match p.how {
                How::Lane(m) => lane += m,
                How::Pass(m, d) => {
                    lane += m;
                    shift = shift.max(d);
                }
                How::Lattice(why) => {
                    fallback.get_or_insert(why);
                }
            }
            let skip = usize::from(leg > 0);
            whole.poses.extend(p.plan.poses.iter().skip(skip));
            whole.raw.extend(p.plan.raw.iter().skip(skip));
            whole.iterations += p.plan.iterations;
            whole.search_time += p.plan.search_time;
            whole.total_time += p.plan.total_time;
            route.extend(p.route.iter().skip(usize::from(!route.is_empty())));
            stops.extend(p.stops);
            from = *goal;
            if p.cut {
                cut = true;
                break;
            }
        }
        let how = match fallback {
            Some(why) if lane <= 0.0 => How::Lattice(why),
            _ if shift > 0.0 => How::Pass(lane, shift),
            _ => How::Lane(lane),
        };
        Ok(LanePlan {
            plan: whole,
            how,
            cut,
            route,
            stops,
        })
    }
}

/// The roads of a lanes block, as `lane_map.costmap_block` writes it (the costmap's
/// `lane_costs` block too): `{"roads": [{name, points: [[x, y]...], width, lanes,
/// oneway, edge_inset}]}`. `lanes`, `oneway` and `edge_inset` may be left out.
///
/// # Errors
///
/// What is missing or not a number.
pub fn roads_from_json(block: &serde_json::Value) -> Result<Vec<Road>, String> {
    block["roads"]
        .as_array()
        .ok_or("roads: a list is needed")?
        .iter()
        .enumerate()
        .map(|(i, r)| {
            let points = r["points"]
                .as_array()
                .and_then(|ps| {
                    ps.iter()
                        .map(|p| Some((p[0].as_f64()?, p[1].as_f64()?)))
                        .collect::<Option<Vec<_>>>()
                })
                .ok_or(format!("road {i}: points, a list of [x, y]"))?;
            Ok(Road {
                name: r["name"].as_str().unwrap_or_default().to_owned(),
                points,
                width: r["width"]
                    .as_f64()
                    .ok_or(format!("road {i}: width, in m"))?,
                lanes: r["lanes"]
                    .as_u64()
                    .map_or(0, |n| u32::try_from(n).unwrap_or(0)),
                oneway: r["oneway"].as_bool().unwrap_or(false),
                edge_inset: r["edge_inset"]
                    .as_f64()
                    .unwrap_or(costmap::lanes::EDGE_INSET),
            })
        })
        .collect()
}

/// m ahead a car needs to step `offset` m sideways at `radius` (two arcs; `vehicle.py`
/// `s_bend_length`). Past two radii: 4 radii, a guess for a loop.
fn s_bend(radius: f64, offset: f64) -> f64 {
    let d = offset.abs();
    if d > 2.0 * radius {
        4.0 * radius
    } else {
        (4.0 * radius * d - d * d).sqrt()
    }
}

/// The polyline `points` moved right by `offsets[k]` along segment k (m; + right, the way
/// it runs). Each corner where the two offset lines meet (a mitre), with the two offsets
/// averaged; a near U-turn takes the next segment's.
fn offset(points: &[(f64, f64)], offsets: &[f64]) -> Vec<(f64, f64)> {
    let normals: Vec<(f64, f64)> = points
        .windows(2)
        .map(|w| {
            let (dx, dy) = (w[1].0 - w[0].0, w[1].1 - w[0].1);
            let l = dx.hypot(dy).max(1e-12);
            (dy / l, -dx / l)
        })
        .collect();
    let n = points.len();
    (0..n)
        .map(|k| {
            let p = points[k];
            if k == 0 {
                return (
                    p.0 + offsets[0] * normals[0].0,
                    p.1 + offsets[0] * normals[0].1,
                );
            }
            if k == n - 1 {
                let (o, m) = (offsets[k - 1], normals[k - 1]);
                return (p.0 + o * m.0, p.1 + o * m.1);
            }
            let (a, b) = (normals[k - 1], normals[k]);
            let o = 0.5 * (offsets[k - 1] + offsets[k]);
            let denom = 1.0 + a.0 * b.0 + a.1 * b.1;
            if denom < 0.2 {
                (p.0 + o * b.0, p.1 + o * b.1)
            } else {
                (p.0 + o * (a.0 + b.0) / denom, p.1 + o * (a.1 + b.1) / denom)
            }
        })
        .collect()
}

/// Each inner corner of `points` replaced by a continuous-curvature turn (Fraichard and
/// Scheuer's CC turn): a clothoid (Euler spiral) from straight up to the curvature of
/// `radius` m over `transition` m, an arc of `radius` m, and a clothoid back to straight.
/// A small corner is the two clothoids only, meeting below that curvature. The curvature
/// never jumps, so the steering doesn't (TODO 211); `transition` 0 gives the circle arcs
/// used before 17 Nov. Where the segments beside a corner are too short for the turn (it
/// may use at most half of each), the whole turn shrinks to fit. A point every `spacing`
/// m or less.
///
/// ```text
///   curvature
///   1/radius ┤      ┌──────┐
///            │     ╱  arc   ╲
///          0 ┼────╱          ╲────
///              clothoid   clothoid
/// ```
fn round_corners(
    points: &[(f64, f64)],
    radius: f64,
    transition: f64,
    spacing: f64,
) -> Vec<(f64, f64)> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut out = vec![points[0]];
    for w in points.windows(3) {
        let (a, v, b) = (w[0], w[1], w[2]);
        let (l1, l2) = (distance(a, v), distance(v, b));
        if l1 < 1e-9 || l2 < 1e-9 {
            out.push(v);
            continue;
        }
        let u1 = ((v.0 - a.0) / l1, (v.1 - a.1) / l1);
        let u2 = ((b.0 - v.0) / l2, (b.1 - v.1) / l2);
        let turn = wrap(u2.1.atan2(u2.0) - u1.1.atan2(u1.0));
        let half = 0.5 * turn.abs();
        if half < 1e-3 || half > 0.5 * std::f64::consts::PI - 1e-3 {
            out.push(v);
            continue;
        }
        let turn_curve = CcTurn::new(turn.abs(), radius, transition);
        let tangent = turn_curve.tangent();
        let scale = (0.5 * l1 / tangent).min(0.5 * l2 / tangent).min(1.0);
        let start = (v.0 - u1.0 * tangent * scale, v.1 - u1.1 * tangent * scale);
        let side = turn.signum();
        // The turn's own frame: x along u1, y to the turn's side.
        out.extend(
            turn_curve
                .points(spacing / scale)
                .into_iter()
                .map(|(x, y)| {
                    let (x, y) = (x * scale, side * y * scale);
                    (start.0 + x * u1.0 - y * u1.1, start.1 + x * u1.1 + y * u1.0)
                }),
        );
    }
    out.push(points[points.len() - 1]);
    out.dedup_by(|b, a| distance(*a, *b) < 1e-6);
    out
}

/// A continuous-curvature turn to the left by `turn` rad, starting at the origin facing
/// +x: a clothoid of `clothoid` m, an arc of `arc` m, the clothoid mirrored.
#[derive(Debug, Clone, Copy, PartialEq)]
struct CcTurn {
    turn: f64,
    /// 1/m per m: how fast the curvature grows along each clothoid (0: no clothoid).
    sharpness: f64,
    clothoid: f64,
    arc: f64,
    /// 1/m along the arc (the clothoids' peak).
    curvature: f64,
}

impl CcTurn {
    fn new(turn: f64, radius: f64, transition: f64) -> Self {
        let most = 1.0 / radius.max(1e-6);
        if transition <= 1e-9 {
            return Self {
                turn,
                sharpness: 0.0,
                clothoid: 0.0,
                arc: turn / most,
                curvature: most,
            };
        }
        let sharpness = most / transition;
        // Each full clothoid turns most * transition / 2.
        if turn >= most * transition {
            Self {
                turn,
                sharpness,
                clothoid: transition,
                arc: (turn - most * transition) / most,
                curvature: most,
            }
        } else {
            // Two clothoids only, each turning half: sharpness * l^2 / 2 = turn / 2.
            let clothoid = (turn / sharpness).sqrt();
            Self {
                turn,
                sharpness,
                clothoid,
                arc: 0.0,
                curvature: sharpness * clothoid,
            }
        }
    }

    fn length(&self) -> f64 {
        2.0 * self.clothoid + self.arc
    }

    /// The heading `s` m along.
    fn heading(&self, s: f64) -> f64 {
        let (c, a) = (self.clothoid, self.arc);
        if s <= c {
            0.5 * self.sharpness * s * s
        } else if s <= c + a {
            0.5 * self.sharpness * c * c + self.curvature * (s - c)
        } else {
            let left = (self.length() - s).max(0.0);
            self.turn - 0.5 * self.sharpness * left * left
        }
    }

    /// The curvature `s` m along (1/m).
    #[cfg(test)]
    fn curvature_at(&self, s: f64) -> f64 {
        let (c, a) = (self.clothoid, self.arc);
        if s <= c {
            self.sharpness * s
        } else if s <= c + a {
            self.curvature
        } else {
            self.sharpness * (self.length() - s).max(0.0)
        }
    }

    /// Points along the turn, at most `spacing` m apart, both ends included (the
    /// midpoint rule on the exact heading: millimetres off over a corner).
    fn points(&self, spacing: f64) -> Vec<(f64, f64)> {
        let length = self.length();
        let step = spacing.clamp(1e-3, 0.05);
        let n = (length / step).ceil().max(1.0) as usize;
        let h = length / n as f64;
        let every = ((spacing / h).floor() as usize).max(1);
        let mut out = vec![(0.0, 0.0)];
        let mut p = (0.0, 0.0);
        for i in 0..n {
            let th = self.heading((i as f64 + 0.5) * h);
            p = (p.0 + h * th.cos(), p.1 + h * th.sin());
            if (i + 1) % every == 0 || i + 1 == n {
                out.push(p);
            }
        }
        out
    }

    /// m from the corner's vertex back to where the turn starts (and on to where it ends).
    fn tangent(&self) -> f64 {
        let (x, y) = self.points(0.05).last().copied().unwrap_or((0.0, 0.0));
        x - y / self.turn.tan()
    }
}

/// A pose every `spacing` m along `points` (and the last point), each facing along it.
fn resample(points: &[(f64, f64)], spacing: f64) -> Vec<Pose> {
    let mut out = Vec::new();
    let mut carry = 0.0;
    let mut heading = 0.0;
    for w in points.windows(2) {
        let (a, b) = (w[0], w[1]);
        let l = distance(a, b);
        if l < 1e-9 {
            continue;
        }
        heading = (b.1 - a.1).atan2(b.0 - a.0);
        let mut s = carry;
        while s < l {
            let t = s / l;
            out.push(Pose::new(
                a.0 + t * (b.0 - a.0),
                a.1 + t * (b.1 - a.1),
                heading,
            ));
            s += spacing;
        }
        carry = s - l;
    }
    if let Some(&end) = points.last() {
        if out.last().is_none_or(|p| distance((p.x, p.y), end) > 1e-6) {
            out.push(Pose::new(end.0, end.1, heading));
        }
    }
    out
}

/// The metres along `poses` to each one.
fn arc_lengths(poses: &[Pose]) -> Vec<f64> {
    let mut s = 0.0;
    let mut prev: Option<&Pose> = None;
    poses
        .iter()
        .map(|p| {
            if let Some(q) = prev {
                s += (p.x - q.x).hypot(p.y - q.y);
            }
            prev = Some(p);
            s
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use costmap::lanes::EDGE_INSET;

    fn road(name: &str, points: &[(f64, f64)], oneway: bool) -> Road {
        Road {
            name: name.to_owned(),
            points: points.to_vec(),
            width: 7.0,
            lanes: 0,
            oneway,
            edge_inset: EDGE_INSET,
        }
    }

    /// An L: east along y = 0 from x = 0 to 100, then north along x = 100 to y = 100;
    /// two-way, 7 m (the right lane's centre 1.5 m right of the centre line).
    fn ell() -> LaneRouter {
        LaneRouter::new(
            vec![
                road("east", &[(0.0, 0.0), (50.0, 0.0), (100.0, 0.0)], false),
                road("north", &[(100.0, 0.0), (100.0, 100.0)], false),
            ],
            LaneSettings::default(),
        )
    }

    fn max_off(line: &[Pose], f: impl Fn(&Pose) -> f64) -> f64 {
        line.iter().map(|p| f(p).abs()).fold(0.0, f64::max)
    }

    #[test]
    fn a_straight_road_keeps_right() {
        let r = ell();
        let line = r
            .lane_line(&Pose::new(10.0, 0.5, 0.0), &Pose::new(40.0, -1.0, 0.0), 5.0)
            .expect("a line");
        // Driving east, right is south: y = -1.5 the whole way.
        assert!(max_off(&line, |p| p.y + 1.5) < 1e-6);
        assert!((line[0].x - 10.0).abs() < 1e-6);
        assert!((line[line.len() - 1].x - 40.0).abs() < 1e-6);
        // Back west: the other lane, y = +1.5.
        let back = r
            .lane_line(&Pose::new(40.0, 0.0, 3.1), &Pose::new(10.0, 0.0, 3.1), 5.0)
            .expect("a line");
        assert!(max_off(&back, |p| p.y - 1.5) < 1e-6);
        assert!(back
            .iter()
            .all(|p| wrap(p.yaw - std::f64::consts::PI).abs() < 1e-6));
    }

    #[test]
    fn round_the_corner_by_the_graph() {
        let r = ell();
        let line = r
            .lane_line(
                &Pose::new(10.0, 0.0, 0.0),
                &Pose::new(100.0, 60.0, 1.57),
                5.0,
            )
            .expect("a line");
        let first = line[0];
        let last = line[line.len() - 1];
        assert!((first.y + 1.5).abs() < 1e-6, "{first:?}");
        // Driving north, right is east: x = 101.5.
        assert!(
            (last.x - 101.5).abs() < 1e-6 && (last.y - 60.0).abs() < 1e-6,
            "{last:?}"
        );
        // Evenly spaced, and the corner rounded: no step turns more than spacing / radius.
        for w in line.windows(2) {
            let step = (w[1].x - w[0].x).hypot(w[1].y - w[0].y);
            assert!(step <= 0.25 + 1e-6, "{step}");
        }
        let sharpest = line
            .windows(2)
            .map(|w| wrap(w[1].yaw - w[0].yaw).abs())
            .fold(0.0, f64::max);
        assert!(sharpest < 0.25 / 5.0 * 2.0 + 1e-3, "{sharpest}");
        // Never off the road: within the right lane's 3 m, by a margin for the corner.
        for p in &line {
            let d = if p.x < 100.0 {
                p.y.abs()
            } else {
                (p.x - 100.0).abs()
            };
            assert!(d <= 3.0, "{p:?}");
        }
    }

    #[test]
    fn one_way_roads_are_driven_their_way_only() {
        let r = LaneRouter::new(
            vec![road("oneway", &[(0.0, 0.0), (100.0, 0.0)], true)],
            LaneSettings::default(),
        );
        assert!(r
            .lane_line(&Pose::new(10.0, 0.0, 0.0), &Pose::new(60.0, 0.0, 0.0), 5.0)
            .is_some());
        assert!(r
            .lane_line(&Pose::new(60.0, 0.0, 3.0), &Pose::new(10.0, 0.0, 3.0), 5.0)
            .is_none());
        // The goal behind the start: no way round on one road.
        assert!(r
            .lane_line(&Pose::new(60.0, 0.0, 0.0), &Pose::new(10.0, 0.0, 0.0), 5.0)
            .is_none());
    }

    #[test]
    fn off_the_map_is_none_near_it_joins() {
        let r = ell();
        // 50 m off: the lot.
        assert!(r
            .lane_line(&Pose::new(10.0, 50.0, 0.0), &Pose::new(60.0, 0.0, 0.0), 5.0)
            .is_none());
        // 12 m off, facing anywhere: joins the nearer way to the goal.
        let line = r
            .lane_line(
                &Pose::new(10.0, -12.0, 1.57),
                &Pose::new(60.0, 0.0, 0.0),
                5.0,
            )
            .expect("a line");
        assert!(line.iter().all(|p| (p.y + 1.5).abs() < 1e-6));
    }

    #[test]
    fn shared_points_join_roads() {
        let Graph::Roads(r) = ell().graph else {
            panic!("a lanes.yaml graph");
        };
        assert_eq!(r.nodes.len(), 4, "the corner is one node");
        assert_eq!(r.edges.len(), 6);
    }

    #[test]
    fn offset_and_corners_by_hand() {
        // A right angle, moved 1 m right: the mitre at (9, 1)... driving east then north,
        // right is south then east: the corner at (11, -1).
        let moved = offset(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)], &[1.0, 1.0]);
        assert!((moved[1].0 - 11.0).abs() < 1e-9 && (moved[1].1 + 1.0).abs() < 1e-9);
        let round = round_corners(&moved, 2.0, 0.0, 0.1);
        // The arc stays inside the mitre's corner, at the radius from its centre (9, 1).
        for p in &round[1..round.len() - 1] {
            assert!(p.0 <= 11.0 + 1e-9 && p.1 >= -1.0 - 1e-9);
        }
        let s = resample(&round, 0.5);
        assert!(s
            .windows(2)
            .all(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y) <= 0.5 + 1e-9));
        assert!((s_bend(5.0, 3.5) - (70.0f64 - 12.25).sqrt()).abs() < 1e-9);
    }

    #[test]
    fn clothoid_corners_keep_the_curvature_continuous() {
        // A right angle, the 5 m radius, 4 m clothoids: clothoid, arc, clothoid.
        let big = CcTurn::new(0.5 * std::f64::consts::PI, 5.0, 4.0);
        assert!((big.clothoid - 4.0).abs() < 1e-12 && big.arc > 0.0);
        assert!((big.heading(big.length()) - big.turn).abs() < 1e-12);
        // A small corner: two clothoids only, peaking below 1 / radius.
        let small = CcTurn::new(0.2, 5.0, 4.0);
        assert!(small.arc == 0.0 && small.curvature < 0.2);
        for t in [big, small] {
            let n = 400;
            let h = t.length() / f64::from(n);
            let ks: Vec<f64> = (0..=n).map(|i| t.curvature_at(f64::from(i) * h)).collect();
            assert!(ks[0].abs() < 1e-12 && ks[n as usize].abs() < 1e-9);
            assert!(ks.iter().all(|&k| k <= 0.2 + 1e-12));
            // No jump: each step changes the curvature by at most sharpness * step.
            assert!(ks
                .windows(2)
                .all(|w| (w[1] - w[0]).abs() <= t.sharpness * h + 1e-9));
        }
        // On a lane line: the end leaves along the second segment, at the tangent's
        // distance from the corner, and the heading's change per metre (the curvature)
        // never steps by more than the clothoid allows.
        let line = round_corners(&[(0.0, 0.0), (30.0, 0.0), (30.0, 30.0)], 5.0, 4.0, 0.25);
        let t = big.tangent();
        assert!(t > 5.0, "longer than the arc's own 5 m: {t}");
        assert!(line.iter().any(|p| distance(*p, (30.0 - t, 0.0)) < 1e-6));
        assert!(line.iter().any(|p| distance(*p, (30.0, t)) < 0.01), "{t}");
        let poses = resample(&line, 0.25);
        let ks: Vec<f64> = poses
            .windows(2)
            .map(|w| wrap(w[1].yaw - w[0].yaw) / 0.25)
            .collect();
        let jump = ks
            .windows(2)
            .map(|w| (w[1] - w[0]).abs())
            .fold(0.0, f64::max);
        let arcs = resample(
            &round_corners(&[(0.0, 0.0), (30.0, 0.0), (30.0, 30.0)], 5.0, 0.0, 0.25),
            0.25,
        );
        let arc_jump = arcs
            .windows(3)
            .map(|w| (wrap(w[2].yaw - w[1].yaw) - wrap(w[1].yaw - w[0].yaw)).abs() / 0.25)
            .fold(0.0, f64::max);
        assert!(jump < 0.05, "{jump}");
        assert!(
            arc_jump > 0.08 && arc_jump > 4.0 * jump,
            "{arc_jump} {jump}"
        );
    }
}
