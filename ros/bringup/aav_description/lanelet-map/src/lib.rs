//! Lanelet2 maps in Rust, with no ROS: step 6 of the three levels
//! (`docs/designs/2026-11-15_three-levels.md`, TODO 175). Ben, 14 Nov: "convert lanelet2
//! to rust". Our own code, after Lanelet2's file format (Poggenhans et al., 2018); no
//! crate for it exists.
//!
//! ```text
//!  map.osm ─▶ LaneletMap::from_osm ─▶ points, lines, lanelets, rules
//!                                   ─▶ the routing graph: which lane follows which,
//!                                      which sits beside (a lane change over a dashed line)
//!  from, to ─▶ route ─▶ [lanelet ids] ─▶ centre lines, the stop lines on the way
//! ```
//!
//! A lanelet is one stretch of one lane: a left and a right line, driven from their first
//! points to their last. Lanelet B **follows** A when A's lines end where B's start; B is
//! **beside** A when they share a line; a lane change is allowed over a dashed one.
//!
//! Positions: a point's `local_x`, `local_y` tags (Autoware's maps carry them), or else its
//! lat/lon put in east and north of the `origin` (WGS-84, exactly as our GPS datum does).
//! Metres, x east, y north (REP-103's map frame).
//!
//! ```
//! use lanelet_map::{LaneletMap, Origin};
//! let osm = r#"<osm>
//!   <node id="1" lat="0" lon="0"><tag k="local_x" v="0"/><tag k="local_y" v="0"/></node>
//!   <node id="2" lat="0" lon="0"><tag k="local_x" v="10"/><tag k="local_y" v="0"/></node>
//!   <node id="3" lat="0" lon="0"><tag k="local_x" v="0"/><tag k="local_y" v="3"/></node>
//!   <node id="4" lat="0" lon="0"><tag k="local_x" v="10"/><tag k="local_y" v="3"/></node>
//!   <way id="10"><nd ref="3"/><nd ref="4"/><tag k="type" v="line_thin"/></way>
//!   <way id="11"><nd ref="1"/><nd ref="2"/><tag k="type" v="line_thin"/></way>
//!   <relation id="100"><member type="way" role="left" ref="10"/>
//!     <member type="way" role="right" ref="11"/><tag k="type" v="lanelet"/></relation>
//! </osm>"#;
//! let map = LaneletMap::from_osm(osm, Origin::default()).unwrap();
//! assert_eq!(map.lanelets().count(), 1);
//! assert_eq!(map.nearest(5.0, 1.0), Some(100));
//! ```
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

use std::cmp::Ordering;
use std::collections::{BTreeMap, BinaryHeap, HashMap};
use std::fmt;

/// An id from the `.osm` file.
pub type Id = i64;

/// A point on the map (m).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    /// m, east.
    pub x: f64,
    /// m, north.
    pub y: f64,
}

impl Point {
    /// The point (`x`, `y`).
    #[must_use]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// The straight distance to `other` (m).
    #[must_use]
    pub fn distance(self, other: Point) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
}

/// Where lat/lon points are put on the map: the map frame's origin (our GPS datum).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Origin {
    /// Degrees north.
    pub lat: f64,
    /// Degrees east.
    pub lon: f64,
}

const WGS84_A: f64 = 6_378_137.0; // m, the equator's radius
const WGS84_F: f64 = 1.0 / 298.257_223_563;

/// A lat/lon (degrees, on the ellipsoid's surface) as earth-centred x, y, z (m).
fn ecef(lat: f64, lon: f64) -> [f64; 3] {
    let e2 = WGS84_F * (2.0 - WGS84_F);
    let (phi, lambda) = (lat.to_radians(), lon.to_radians());
    let n = WGS84_A / (1.0 - e2 * phi.sin().powi(2)).sqrt();
    [
        n * phi.cos() * lambda.cos(),
        n * phi.cos() * lambda.sin(),
        n * (1.0 - e2) * phi.sin(),
    ]
}

impl Origin {
    /// East and north of the origin (m): WGS-84 to earth-centred to the origin's local
    /// east-north-up, exactly (as tools/campus_world.py's geodetic_to_enu).
    fn project(&self, lat: f64, lon: f64) -> Point {
        let (o, p) = (ecef(self.lat, self.lon), ecef(lat, lon));
        let d = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
        let (phi, lambda) = (self.lat.to_radians(), self.lon.to_radians());
        let east = -lambda.sin() * d[0] + lambda.cos() * d[1];
        let north =
            -phi.sin() * lambda.cos() * d[0] - phi.sin() * lambda.sin() * d[1] + phi.cos() * d[2];
        Point::new(east, north)
    }
}

/// How a line is painted (Lanelet2's `type` and `subtype` tags).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LineKind {
    /// A painted line: dashed lets a car change lanes over it, solid doesn't.
    Paint {
        /// Dashed, not solid.
        dashed: bool,
    },
    /// A stop line.
    StopLine,
    /// A curb: never crossed.
    Curb,
    /// A line only on the map (no paint): crossed freely.
    Virtual,
    /// Any other `type`, kept as written.
    Other(String),
}

impl LineKind {
    fn from_tags(kind: Option<&str>, subtype: Option<&str>) -> Self {
        match kind {
            Some("line_thin" | "line_thick") => Self::Paint {
                dashed: matches!(subtype, Some("dashed" | "dashed_solid")),
            },
            Some("stop_line") => Self::StopLine,
            Some("curbstone") => Self::Curb,
            Some("virtual") => Self::Virtual,
            other => Self::Other(other.unwrap_or("").to_string()),
        }
    }

    /// May a car change lanes over this line?
    #[must_use]
    pub fn crossable(&self) -> bool {
        matches!(self, Self::Paint { dashed: true } | Self::Virtual)
    }
}

/// A line of points (a way).
#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    /// The way's id.
    pub id: Id,
    /// Its nodes' ids, in order.
    pub nodes: Vec<Id>,
    /// Its nodes on the map, in order.
    pub points: Vec<Point>,
    /// What it is.
    pub kind: LineKind,
    /// The `subtype` tag as written (a sign's kind, for a `traffic_sign` way).
    pub subtype: Option<String>,
}

/// One stretch of one lane.
#[derive(Debug, Clone, PartialEq)]
pub struct Lanelet {
    /// The relation's id.
    pub id: Id,
    /// Its left bound (a [`Line`]).
    pub left: Id,
    /// Its right bound (a [`Line`]).
    pub right: Id,
    /// The middle of the lane, from start to end.
    pub centre: Vec<Point>,
    /// m/s, from a `speed_limit` tag in km/h, if any.
    pub speed_limit: Option<f64>,
    /// The rules that bind on this lanelet (stop signs, right of way).
    pub rules: Vec<Id>,
}

impl Lanelet {
    /// The centre line's length (m).
    #[must_use]
    pub fn length(&self) -> f64 {
        self.centre.windows(2).map(|w| w[0].distance(w[1])).sum()
    }
}

/// What a rule says.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RuleKind {
    /// A sign (Lanelet2's `traffic_sign`), e.g. a stop sign: `de206`, `usR1-1` or `stop`.
    Sign(String),
    /// Some lanes have the right of way, others yield.
    RightOfWay,
    /// Every way stops, the first to arrive goes first.
    AllWayStop,
    /// Traffic lights.
    TrafficLight,
    /// Another kind, by its subtype.
    Other(String),
}

/// A rule (a regulatory element) and where it binds.
#[derive(Debug, Clone, PartialEq)]
pub struct Rule {
    /// The relation's id.
    pub id: Id,
    /// What it says.
    pub kind: RuleKind,
    /// The lines to stop at (`ref_line`).
    pub stop_lines: Vec<Id>,
    /// Right of way: the lanelets that go first, and those that yield to them.
    pub right_of_way: Vec<Id>,
    /// The lanelets that yield.
    pub yields: Vec<Id>,
}

impl Rule {
    /// Does this rule make a car stop at its line?
    #[must_use]
    pub fn is_stop(&self) -> bool {
        match &self.kind {
            RuleKind::Sign(sign) => {
                let sign = sign.to_ascii_lowercase();
                sign == "stop" || sign == "de206" || sign == "usr1-1" || sign.contains("stop")
            }
            RuleKind::AllWayStop => true,
            _ => false,
        }
    }
}

/// Why a map couldn't be read.
#[derive(Debug, Clone, PartialEq)]
pub enum MapError {
    /// The XML can't be read.
    Xml(String),
    /// An element lacks a part it must have (an id, a lat, a member).
    Missing {
        /// `node`, `way` or `relation`.
        element: &'static str,
        /// Its id.
        id: Id,
        /// The part missing.
        what: &'static str,
    },
    /// A way or relation names an id the file doesn't have.
    Unknown {
        /// `way` or `relation`.
        element: &'static str,
        /// Its id.
        id: Id,
        /// The id it names.
        refers: Id,
    },
}

impl fmt::Display for MapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Xml(e) => write!(f, "not a readable .osm: {e}"),
            Self::Missing { element, id, what } => write!(f, "{element} {id} has no {what}"),
            Self::Unknown {
                element,
                id,
                refers,
            } => {
                write!(
                    f,
                    "{element} {id} refers to {refers}, which the map doesn't have"
                )
            }
        }
    }
}

impl std::error::Error for MapError {}

/// A whole Lanelet2 map: its lines, lanelets, rules and routing graph.
#[derive(Debug, Clone)]
pub struct LaneletMap {
    lines: BTreeMap<Id, Line>,
    lanelets: BTreeMap<Id, Lanelet>,
    rules: BTreeMap<Id, Rule>,
    /// Lanelet -> the lanelets that follow it.
    next: BTreeMap<Id, Vec<Id>>,
    /// Lanelet -> the lanelets beside it a car may change into.
    beside: BTreeMap<Id, Vec<Id>>,
    /// Every centre line's segments, in an R-tree (TODO 201): `nearest` looks at the few
    /// near a point, not every lanelet's whole line.
    index: rstar::RTree<Segment>,
}

/// One segment of a lanelet's centre line, in the R-tree.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Segment {
    lanelet: Id,
    a: Point,
    b: Point,
}

impl Segment {
    /// The segments of each lanelet's centre line (a line of one point: a point).
    fn of(lanelets: &BTreeMap<Id, Lanelet>) -> Vec<Self> {
        lanelets
            .values()
            .flat_map(|l| {
                let pairs: Vec<(Point, Point)> = match l.centre.as_slice() {
                    [only] => vec![(*only, *only)],
                    line => line.windows(2).map(|w| (w[0], w[1])).collect(),
                };
                pairs.into_iter().map(move |(a, b)| Segment {
                    lanelet: l.id,
                    a,
                    b,
                })
            })
            .collect()
    }
}

impl rstar::RTreeObject for Segment {
    type Envelope = rstar::AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        rstar::AABB::from_corners([self.a.x, self.a.y], [self.b.x, self.b.y])
    }
}

impl rstar::PointDistance for Segment {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        distance_to_line(Point::new(point[0], point[1]), &[self.a, self.b]).powi(2)
    }
}

/// Points along the centre line: one every this far, at least (m).
const CENTRE_STEP: f64 = 1.0;
/// A lane change's cost, in metres of driving: a longer way round in the same lane wins
/// over a change shorter than this.
pub const LANE_CHANGE_COST: f64 = 10.0;

mod routing;
pub use routing::{Place, Route, LANE_CHANGE_LENGTH};

impl LaneletMap {
    /// Read a Lanelet2 `.osm` file's text.
    ///
    /// # Errors
    ///
    /// [`MapError`]: bad XML, a part missing, or an id the file doesn't have.
    pub fn from_osm(text: &str, origin: Origin) -> Result<Self, MapError> {
        let doc = roxmltree::Document::parse(text).map_err(|e| MapError::Xml(e.to_string()))?;
        let root = doc.root_element();
        let tags = |n: roxmltree::Node| -> HashMap<String, String> {
            n.children()
                .filter(|c| c.has_tag_name("tag"))
                .filter_map(|c| {
                    Some((c.attribute("k")?.to_string(), c.attribute("v")?.to_string()))
                })
                .collect()
        };
        let id_of = |n: roxmltree::Node, element: &'static str| -> Result<Id, MapError> {
            n.attribute("id")
                .and_then(|v| v.parse().ok())
                .ok_or(MapError::Missing {
                    element,
                    id: 0,
                    what: "id",
                })
        };

        let mut points: HashMap<Id, Point> = HashMap::new();
        for n in root.children().filter(|n| n.has_tag_name("node")) {
            let id = id_of(n, "node")?;
            let t = tags(n);
            let local = |k: &str| t.get(k).and_then(|v| v.parse::<f64>().ok());
            let point = match (local("local_x"), local("local_y")) {
                (Some(x), Some(y)) => Point::new(x, y),
                _ => {
                    let number = |k: &'static str| {
                        n.attribute(k).and_then(|v| v.parse::<f64>().ok()).ok_or(
                            MapError::Missing {
                                element: "node",
                                id,
                                what: k,
                            },
                        )
                    };
                    origin.project(number("lat")?, number("lon")?)
                }
            };
            points.insert(id, point);
        }

        let mut lines = BTreeMap::new();
        for w in root.children().filter(|n| n.has_tag_name("way")) {
            let id = id_of(w, "way")?;
            let nodes: Vec<Id> = w
                .children()
                .filter(|c| c.has_tag_name("nd"))
                .filter_map(|c| c.attribute("ref")?.parse().ok())
                .collect();
            let line_points = nodes
                .iter()
                .map(|n| {
                    points.get(n).copied().ok_or(MapError::Unknown {
                        element: "way",
                        id,
                        refers: *n,
                    })
                })
                .collect::<Result<Vec<_>, _>>()?;
            let t = tags(w);
            let kind = LineKind::from_tags(
                t.get("type").map(String::as_str),
                t.get("subtype").map(String::as_str),
            );
            let subtype = t.get("subtype").cloned();
            lines.insert(
                id,
                Line {
                    id,
                    nodes,
                    points: line_points,
                    kind,
                    subtype,
                },
            );
        }

        let mut lanelets = BTreeMap::new();
        let mut rules = BTreeMap::new();
        for r in root.children().filter(|n| n.has_tag_name("relation")) {
            let id = id_of(r, "relation")?;
            let t = tags(r);
            let members = |role: &str| -> Vec<Id> {
                r.children()
                    .filter(|c| c.has_tag_name("member") && c.attribute("role") == Some(role))
                    .filter_map(|c| c.attribute("ref")?.parse().ok())
                    .collect()
            };
            match t.get("type").map(String::as_str) {
                Some("lanelet") => {
                    let one = |role: &'static str| {
                        members(role).first().copied().ok_or(MapError::Missing {
                            element: "lanelet",
                            id,
                            what: role,
                        })
                    };
                    let (left, right) = (one("left")?, one("right")?);
                    let line = |l: Id| {
                        lines.get(&l).ok_or(MapError::Unknown {
                            element: "lanelet",
                            id,
                            refers: l,
                        })
                    };
                    let centre = centre_line(&line(left)?.points, &line(right)?.points);
                    let speed_limit = t
                        .get("speed_limit")
                        .and_then(|v| v.trim_end_matches("km/h").trim().parse::<f64>().ok())
                        .map(|kmh| kmh / 3.6);
                    lanelets.insert(
                        id,
                        Lanelet {
                            id,
                            left,
                            right,
                            centre,
                            speed_limit,
                            rules: members("regulatory_element"),
                        },
                    );
                }
                Some("regulatory_element") => {
                    let kind = match t.get("subtype").map(String::as_str) {
                        Some("traffic_sign") => RuleKind::Sign(
                            members("refers")
                                .iter()
                                .find_map(|w| lines.get(w)?.subtype.clone())
                                .or_else(|| t.get("sign_type").cloned())
                                .unwrap_or_else(|| "unknown".to_string()),
                        ),
                        Some("right_of_way") => RuleKind::RightOfWay,
                        Some("all_way_stop") => RuleKind::AllWayStop,
                        Some("traffic_light") => RuleKind::TrafficLight,
                        other => RuleKind::Other(other.unwrap_or("").to_string()),
                    };
                    rules.insert(
                        id,
                        Rule {
                            id,
                            kind,
                            stop_lines: members("ref_line"),
                            right_of_way: members("right_of_way"),
                            yields: members("yield"),
                        },
                    );
                }
                _ => {}
            }
        }
        let mut map = Self {
            lines,
            lanelets,
            rules,
            next: BTreeMap::new(),
            beside: BTreeMap::new(),
            index: rstar::RTree::new(),
        };
        map.index = rstar::RTree::bulk_load(Segment::of(&map.lanelets));
        map.link();
        Ok(map)
    }

    /// Build the routing graph: who follows whom, who sits beside whom.
    fn link(&mut self) {
        let ends = |l: &Lanelet, lines: &BTreeMap<Id, Line>| {
            let line = |id: Id| lines.get(&id).map(|line| &line.nodes);
            Some((
                *line(l.left)?.first()?,
                *line(l.left)?.last()?,
                *line(l.right)?.first()?,
                *line(l.right)?.last()?,
            ))
        };
        let mut starts: HashMap<(Id, Id), Vec<Id>> = HashMap::new();
        for l in self.lanelets.values() {
            if let Some((left_first, _, right_first, _)) = ends(l, &self.lines) {
                starts
                    .entry((left_first, right_first))
                    .or_default()
                    .push(l.id);
            }
        }
        for l in self.lanelets.values() {
            if let Some((_, left_last, _, right_last)) = ends(l, &self.lines) {
                let after = starts
                    .get(&(left_last, right_last))
                    .cloned()
                    .unwrap_or_default();
                self.next
                    .insert(l.id, after.into_iter().filter(|&n| n != l.id).collect());
            }
        }
        // Beside: my left line is your right line (you are on my left), or the other way.
        for a in self.lanelets.values() {
            let crossable = |line: Id| self.lines.get(&line).is_some_and(|l| l.kind.crossable());
            let beside: Vec<Id> = self
                .lanelets
                .values()
                .filter(|b| b.id != a.id)
                .filter(|b| {
                    (a.left == b.right && crossable(a.left))
                        || (a.right == b.left && crossable(a.right))
                })
                .map(|b| b.id)
                .collect();
            self.beside.insert(a.id, beside);
        }
    }

    /// Every lanelet.
    pub fn lanelets(&self) -> impl Iterator<Item = &Lanelet> {
        self.lanelets.values()
    }

    /// The lanelet `id`.
    #[must_use]
    pub fn lanelet(&self, id: Id) -> Option<&Lanelet> {
        self.lanelets.get(&id)
    }

    /// The line `id`.
    #[must_use]
    pub fn line(&self, id: Id) -> Option<&Line> {
        self.lines.get(&id)
    }

    /// The rule `id`.
    #[must_use]
    pub fn rule(&self, id: Id) -> Option<&Rule> {
        self.rules.get(&id)
    }

    /// The lines a screen draws: every lanelet's painted sides and every stop line, each
    /// once, in id order; virtual lines (only on the map) left out.
    #[must_use]
    pub fn drawn_lines(&self) -> Vec<&Line> {
        let mut ids: Vec<Id> = self
            .lanelets
            .values()
            .flat_map(|l| [l.left, l.right])
            .chain(
                self.rules
                    .values()
                    .flat_map(|r| r.stop_lines.iter().copied()),
            )
            .collect();
        ids.sort_unstable();
        ids.dedup();
        ids.iter()
            .filter_map(|id| self.lines.get(id))
            .filter(|line| line.kind != LineKind::Virtual)
            .collect()
    }

    /// The lanelets that follow `id`.
    pub fn following(&self, id: Id) -> &[Id] {
        self.next.get(&id).map_or(&[], Vec::as_slice)
    }

    /// The lanelets beside `id` that a car may change into.
    pub fn beside(&self, id: Id) -> &[Id] {
        self.beside.get(&id).map_or(&[], Vec::as_slice)
    }

    /// The lanelet whose centre line passes nearest (x, y); of two as near (where one
    /// lane ends and the next begins), the smaller id.
    #[must_use]
    pub fn nearest(&self, x: f64, y: f64) -> Option<Id> {
        let mut near = self.index.nearest_neighbor_iter_with_distance_2(&[x, y]);
        let (first, best) = near.next()?;
        let tied = near.take_while(|&(_, d)| d <= best).map(|(s, _)| s.lanelet);
        tied.chain([first.lanelet]).min()
    }

    /// `nearest` the slow way, every lanelet's whole line: the R-tree's yardstick.
    #[cfg(test)]
    fn nearest_by_scan(&self, x: f64, y: f64) -> Option<Id> {
        let p = Point::new(x, y);
        self.lanelets
            .values()
            .map(|l| (l.id, distance_to_line(p, &l.centre)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(id, _)| id)
    }

    /// The best way by lanes from lanelet `from` to lanelet `to` (A*: driving length, a
    /// lane change counted as `LANE_CHANGE_COST` m). `None` if there is no way.
    #[must_use]
    pub fn route(&self, from: Id, to: Id) -> Option<Vec<Id>> {
        let goal = self.lanelets.get(&to)?.centre.last().copied()?;
        let estimate = |id: Id| {
            self.lanelets
                .get(&id)
                .and_then(|l| l.centre.last())
                .map_or(0.0, |p| p.distance(goal))
        };
        let length = |id: Id| self.lanelets.get(&id).map_or(0.0, Lanelet::length);
        let mut best: HashMap<Id, f64> = HashMap::from([(from, 0.0)]);
        let mut came: HashMap<Id, Id> = HashMap::new();
        let mut open = BinaryHeap::from([Open {
            cost: estimate(from),
            id: from,
        }]);
        while let Some(Open { id, .. }) = open.pop() {
            if id == to {
                let mut path = vec![to];
                while let Some(&before) = came.get(path.last()?) {
                    path.push(before);
                }
                path.reverse();
                return Some(path);
            }
            let here = best[&id];
            let steps = self
                .following(id)
                .iter()
                .map(|&n| (n, length(id)))
                .chain(self.beside(id).iter().map(|&n| (n, LANE_CHANGE_COST)));
            for (n, step) in steps {
                let cost = here + step;
                if best.get(&n).is_none_or(|&old| cost < old) {
                    best.insert(n, cost);
                    came.insert(n, id);
                    open.push(Open {
                        cost: cost + estimate(n),
                        id: n,
                    });
                }
            }
        }
        None
    }

    /// The stop lines a route meets, in order: (lanelet, the rule, the line's middle).
    #[must_use]
    pub fn stops_on(&self, route: &[Id]) -> Vec<(Id, Id, Point)> {
        route
            .iter()
            .filter_map(|id| self.lanelets.get(id))
            .flat_map(|l| {
                l.rules
                    .iter()
                    .filter_map(|r| self.rules.get(r))
                    .filter(|r| r.is_stop())
                    .flat_map(move |r| {
                        r.stop_lines.iter().filter_map(move |line| {
                            let points = &self.lines.get(line)?.points;
                            let (a, b) = (points.first()?, points.last()?);
                            Some((l.id, r.id, Point::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)))
                        })
                    })
            })
            .collect()
    }
}

/// An entry in A*'s open list: the least cost first.
#[derive(Debug, PartialEq)]
struct Open {
    cost: f64,
    id: Id,
}

impl Eq for Open {}

impl Ord for Open {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .total_cmp(&self.cost)
            .then(self.id.cmp(&other.id))
    }
}

impl PartialOrd for Open {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// The middle of two lines: each resampled to the same number of points by length.
fn centre_line(left: &[Point], right: &[Point]) -> Vec<Point> {
    let longest = line_length(left).max(line_length(right));
    let n = ((longest / CENTRE_STEP).ceil() as usize).max(1) + 1;
    let (l, r) = (resample(left, n), resample(right, n));
    l.iter()
        .zip(&r)
        .map(|(a, b)| Point::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0))
        .collect()
}

fn line_length(line: &[Point]) -> f64 {
    line.windows(2).map(|w| w[0].distance(w[1])).sum()
}

/// `n` points evenly along a line by length (its first and last among them).
fn resample(line: &[Point], n: usize) -> Vec<Point> {
    let (Some(&first), Some(&last)) = (line.first(), line.last()) else {
        return Vec::new();
    };
    let total = line_length(line);
    if total <= 0.0 || n < 2 {
        return vec![first; n.max(1)];
    }
    let mut out = Vec::with_capacity(n);
    let (mut segment, mut walked) = (0, 0.0);
    for k in 0..n {
        let want = total * k as f64 / (n - 1) as f64;
        while segment + 2 < line.len() && walked + line[segment].distance(line[segment + 1]) < want
        {
            walked += line[segment].distance(line[segment + 1]);
            segment += 1;
        }
        let (a, b) = (line[segment], line[(segment + 1).min(line.len() - 1)]);
        let span = a.distance(b);
        let t = if span > 0.0 {
            ((want - walked) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        out.push(Point::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
    }
    if let Some(end) = out.last_mut() {
        *end = last;
    }
    out
}

/// The shortest distance from `p` to a line.
fn distance_to_line(p: Point, line: &[Point]) -> f64 {
    match line {
        [] => f64::INFINITY,
        [only] => p.distance(*only),
        _ => line
            .windows(2)
            .map(|w| {
                let (a, b) = (w[0], w[1]);
                let (dx, dy) = (b.x - a.x, b.y - a.y);
                let span = dx * dx + dy * dy;
                let t = if span > 0.0 {
                    (((p.x - a.x) * dx + (p.y - a.y) * dy) / span).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                p.distance(Point::new(a.x + t * dx, a.y + t * dy))
            })
            .fold(f64::INFINITY, f64::min),
    }
}

#[cfg(test)]
mod tests;
