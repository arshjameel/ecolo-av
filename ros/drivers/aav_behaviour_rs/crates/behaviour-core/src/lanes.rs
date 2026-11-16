//! The lane map: which lane the car drives in, so the road ahead's line is the lane's
//! centre (`aav_description/lane_map.py`: the parts the behaviour node uses).
//!
//! ```text
//!    ══════════════════════════════  left kerb   ─┐
//!    ── ── ── ── ── ── ── ── ── ──   centre line  │ left_hard
//!    ● ──────── the car's lane centre ───────►    ┼─ left_soft = right_soft = lane / 2
//!    ─────────────────────────────   edge line  ──┤ right_soft
//!    ══════════════════════════════  right kerb ──┘ right_hard
//! ```
//!
//! | The road | The car's lane |
//! |---|---|
//! | one lane | the whole lane: the road's centre |
//! | two-way, `n` lanes | the rightmost of the `n / 2` its way, as the car faces |
//! | one-way, `n` lanes | the rightmost |

use std::collections::HashMap;
use std::f64::consts::PI;
use std::fmt;
use std::path::Path;

use serde_yaml_ng::Value;

use crate::paths::Point;
use crate::road::{resample, RoadError, Strip};
use crate::{hypot, py_max, py_min, py_sum};

/// m: one lane, when the file doesn't say how many.
pub const LANE_WIDTH: f64 = 3.5;
/// m: kerb to edge line.
pub const EDGE_INSET: f64 = 0.5;
/// rad: the route and the road agree within this, or it's not that road (45°, as
/// Python's `math.radians`).
pub const MAX_TURN: f64 = 45.0 * (PI / 180.0);
/// m: the index's squares.
pub const CELL: f64 = 20.0;
/// m each side: the lane line eases from one road to the next over this.
pub const SMOOTH: f64 = 3.0;
/// m beyond the kerb a route point may be and still be on the road.
pub const MARGIN: f64 = 0.5;
/// m left of its lane's centre a route point is, while the planner passes something
/// parked, before it counts as the pass.
pub const PASS_LEFT: f64 = 0.2;

/// One road: its centre line (map frame, m), its width kerb to kerb, how its lanes run.
#[derive(Debug, Clone, PartialEq)]
pub struct Road {
    /// Its name (or its kind).
    pub name: String,
    /// The centre line.
    pub points: Vec<Point>,
    /// m, kerb to kerb.
    pub width: f64,
    /// Every lane, both ways; 0: width / LANE_WIDTH, rounded.
    pub lanes: i64,
    /// One way.
    pub oneway: bool,
    /// m from the kerb to the edge line.
    pub edge_inset: f64,
}

impl Road {
    /// How many lanes, both ways.
    #[must_use]
    pub fn lane_count(&self) -> i64 {
        if self.lanes > 0 {
            self.lanes
        } else {
            // Python's round(): halves to even.
            ((self.width / LANE_WIDTH).round_ties_even() as i64).max(1)
        }
    }
}

/// The car's lane at one place: where its centre is, and its edges (m, all positive).
#[derive(Debug, Clone, PartialEq)]
pub struct Lane {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad: the way the car drives it.
    pub heading: f64,
    /// m.
    pub left_soft: f64,
    /// m.
    pub right_soft: f64,
    /// m.
    pub left_hard: f64,
    /// m.
    pub right_hard: f64,
    /// The road's name.
    pub road: String,
}

impl Lane {
    fn edge(&self, edge: Edge) -> f64 {
        match edge {
            Edge::LeftHard => self.left_hard,
            Edge::RightHard => self.right_hard,
            Edge::LeftSoft => self.left_soft,
            Edge::RightSoft => self.right_soft,
        }
    }
}

/// (the car's lane centre, its width): metres right of the road's centre (+ = right).
#[must_use]
pub fn lane_offset(road: &Road) -> (f64, f64) {
    let painted = road.width - 2.0 * road.edge_inset;
    let n = road.lane_count();
    if n <= 1 {
        return (0.0, painted);
    }
    let width = if road.oneway {
        painted / n as f64
    } else {
        (painted / 2.0) / (n / 2).max(1) as f64
    };
    (painted / 2.0 - width / 2.0, width)
}

fn wrap(angle: f64) -> f64 {
    angle.sin().atan2(angle.cos())
}

fn cell(v: f64) -> i64 {
    (v / CELL).floor() as i64
}

/// The roads, with a grid index so a lookup reads only the segments near a point.
#[derive(Debug, Clone, Default)]
pub struct LaneMap {
    roads: Vec<Road>,
    /// (i, j) -> (road, segment), in the order they were added.
    cells: HashMap<(i64, i64), Vec<(usize, usize)>>,
}

impl LaneMap {
    /// The map of these roads (a road with fewer than two points, or no width, is left out).
    #[must_use]
    pub fn new(roads: Vec<Road>) -> Self {
        let roads: Vec<Road> = roads
            .into_iter()
            .filter(|r| r.points.len() >= 2 && r.width > 0.0)
            .collect();
        let mut cells: HashMap<(i64, i64), Vec<(usize, usize)>> = HashMap::new();
        for (ri, road) in roads.iter().enumerate() {
            let pad = road.width / 2.0 + MARGIN;
            for (si, w) in road.points.windows(2).enumerate() {
                let (a, b) = (w[0], w[1]);
                let (x0, x1) = (py_min(a.0, b.0), py_max(a.0, b.0));
                let (y0, y1) = (py_min(a.1, b.1), py_max(a.1, b.1));
                for i in cell(x0 - pad)..=cell(x1 + pad) {
                    for j in cell(y0 - pad)..=cell(y1 + pad) {
                        cells.entry((i, j)).or_default().push((ri, si));
                    }
                }
            }
        }
        LaneMap { roads, cells }
    }

    /// The roads.
    #[must_use]
    pub fn roads(&self) -> &[Road] {
        &self.roads
    }

    /// The car's lane at (x, y), driving at `heading` (rad), or `None` off every road.
    ///
    /// The road is the nearest whose tarmac (plus MARGIN) holds the point and that runs
    /// within MAX_TURN of `heading`, either way along it.
    #[must_use]
    pub fn lane_at(&self, x: f64, y: f64, heading: f64) -> Option<Lane> {
        let mut best: Option<(f64, &Road, f64, f64, f64)> = None;
        for &(ri, si) in self.cells.get(&(cell(x), cell(y)))? {
            let road = &self.roads[ri];
            let (a, b) = (road.points[si], road.points[si + 1]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let n2 = dx * dx + dy * dy;
            if n2 < 1e-12 {
                continue;
            }
            let t = py_max(0.0, py_min(1.0, ((x - a.0) * dx + (y - a.1) * dy) / n2));
            let (fx, fy) = (a.0 + t * dx, a.1 + t * dy);
            let d = hypot(x - fx, y - fy);
            if d > road.width / 2.0 + MARGIN {
                continue;
            }
            let direction = dy.atan2(dx);
            let turn = wrap(heading - direction);
            let along = if turn.abs() <= MAX_TURN {
                true
            } else if turn.abs() >= PI - MAX_TURN {
                false
            } else {
                continue;
            };
            if best.is_none_or(|b| d < b.0) {
                let way = if along {
                    direction
                } else {
                    wrap(direction + PI)
                };
                best = Some((d, road, fx, fy, way));
            }
        }
        let (_, road, fx, fy, way) = best?;
        let (right, width) = lane_offset(road);
        // Right of the way the car drives: (sin, -cos) of its heading.
        let (cx, cy) = (fx + right * way.sin(), fy - right * way.cos());
        let half = road.width / 2.0;
        Some(Lane {
            x: cx,
            y: cy,
            heading: way,
            left_soft: width / 2.0,
            right_soft: width / 2.0,
            left_hard: half + right,
            right_hard: half - right,
            road: road.name.clone(),
        })
    }
}

/// A lanes.yaml that can't be read.
#[derive(Debug)]
pub enum LanesError {
    /// The file.
    Read(std::io::Error),
    /// Its YAML.
    Yaml(serde_yaml_ng::Error),
    /// A road's field.
    Field {
        /// Which road (0 the first).
        road: usize,
        /// What is wrong.
        what: String,
    },
}

impl fmt::Display for LanesError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LanesError::Read(e) => write!(f, "lanes.yaml: {e}"),
            LanesError::Yaml(e) => write!(f, "lanes.yaml: {e}"),
            LanesError::Field { road, what } => write!(f, "lanes.yaml, road {road}: {what}"),
        }
    }
}

impl std::error::Error for LanesError {}

fn number(v: &Value) -> Option<f64> {
    v.as_f64().or_else(|| v.as_i64().map(|i| i as f64))
}

/// The lane map in a lanes.yaml (the format in lane_map.py's docstring).
///
/// # Errors
///
/// [`LanesError`] if the file can't be read, isn't YAML, or a road lacks its points or width.
pub fn load(path: impl AsRef<Path>) -> Result<LaneMap, LanesError> {
    let text = std::fs::read_to_string(path).map_err(LanesError::Read)?;
    from_yaml(&text)
}

/// The lane map from lanes.yaml's text.
///
/// # Errors
///
/// As [`load`].
pub fn from_yaml(text: &str) -> Result<LaneMap, LanesError> {
    let data: Value = serde_yaml_ng::from_str(text).map_err(LanesError::Yaml)?;
    let mut roads = Vec::new();
    let list = match data.get("roads") {
        Some(Value::Sequence(list)) => list.as_slice(),
        _ => &[],
    };
    for (i, r) in list.iter().enumerate() {
        let field = |what: &str| LanesError::Field {
            road: i,
            what: what.to_string(),
        };
        // str(r.get('name') or r.get('kind', '')).
        let text_of = |v: Option<&Value>| match v {
            Some(Value::String(s)) => s.clone(),
            Some(Value::Number(n)) => n.to_string(),
            _ => String::new(),
        };
        let mut name = text_of(r.get("name"));
        if name.is_empty() {
            name = text_of(r.get("kind"));
        }
        let points = r
            .get("points")
            .and_then(Value::as_sequence)
            .ok_or_else(|| field("no points"))?
            .iter()
            .map(|p| {
                let xy = p.as_sequence()?;
                Some((number(xy.first()?)?, number(xy.get(1)?)?))
            })
            .collect::<Option<Vec<Point>>>()
            .ok_or_else(|| field("a point isn't [x, y]"))?;
        let width = r
            .get("width")
            .and_then(number)
            .ok_or_else(|| field("no width"))?;
        let lanes = match r.get("lanes") {
            None | Some(Value::Null) => 0,
            Some(v) => number(v)
                .ok_or_else(|| field("lanes isn't a number"))?
                .trunc() as i64,
        };
        let oneway = r.get("oneway").and_then(Value::as_bool).unwrap_or(false);
        let edge_inset = match r.get("edge_inset") {
            None | Some(Value::Null) => EDGE_INSET,
            Some(v) => number(v).ok_or_else(|| field("edge_inset isn't a number"))?,
        };
        roads.push(Road {
            name,
            points,
            width,
            lanes,
            oneway,
            edge_inset,
        });
    }
    Ok(LaneMap::new(roads))
}

/// m (x, y) is left of the lane's centre (+ left), the way the lane runs.
fn left_of(lane: &Lane, x: f64, y: f64) -> f64 {
    (x - lane.x) * -lane.heading.sin() + (y - lane.y) * lane.heading.cos()
}

/// Where a route point goes in [`lane_line`].
#[derive(Debug, Clone, PartialEq)]
enum Target {
    /// Off every road: it stays.
    Off,
    /// Part of a pass: it stays.
    Passing,
    /// On this road's lane centre.
    Road(String),
}

/// The car's lane centre along the route ahead of the car: points, map frame.
///
/// Each route point ahead (from the car's foot, every `spacing` m, `smooth` m past
/// `length`) moves onto its lane's centre. Off every road it stays where the route is.
/// Where the road changes, the points are averaged over `smooth` m each side. `passing`:
/// a point more than PASS_LEFT left of its lane's centre stays where the route is.
///
/// # Errors
///
/// [`RoadError::Spacing`] unless `spacing` > 0.
pub fn lane_line(
    lanes: &LaneMap,
    route: &[Point],
    car: Point,
    spacing: f64,
    length: f64,
    smooth: f64,
    passing: bool,
) -> Result<Vec<Point>, RoadError> {
    let ahead = resample(route, car.0, car.1, spacing, length + smooth)?;
    let targets: Vec<(f64, f64, Target)> = ahead
        .iter()
        .map(|&(x, y, h)| match lanes.lane_at(x, y, h) {
            None => (x, y, Target::Off),
            Some(lane) if passing && left_of(&lane, x, y) > PASS_LEFT => (x, y, Target::Passing),
            Some(lane) => (lane.x, lane.y, Target::Road(lane.road)),
        })
        .collect();
    let k = (smooth / spacing).round_ties_even().max(0.0) as usize;
    let out = targets
        .iter()
        .enumerate()
        .map(|(i, (x, y, road))| {
            let window = &targets[i.saturating_sub(k)..(i + k + 1).min(targets.len())];
            let one_road = matches!(road, Target::Road(_)) && window.iter().all(|w| w.2 == *road);
            if one_road {
                (*x, *y)
            } else {
                let n = window.len() as f64;
                (
                    py_sum(window.iter().map(|w| w.0)) / n,
                    py_sum(window.iter().map(|w| w.1)) / n,
                )
            }
        })
        .collect();
    Ok(out)
}

/// The four edges, in the Python's order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Edge {
    LeftHard,
    RightHard,
    LeftSoft,
    RightSoft,
}

/// The strip with its edges from the lane map: NaN where a point is on no road.
///
/// `passing`: a point more than PASS_LEFT left of its lane's centre has its left soft edge
/// opened to the hard one (the kerb).
#[must_use]
pub fn with_edges(mut strip: Strip, lanes: &LaneMap, passing: bool) -> Strip {
    if strip.is_empty() {
        return strip;
    }
    let n = strip.len();
    let mut edges = [
        Vec::with_capacity(n),
        Vec::with_capacity(n),
        Vec::with_capacity(n),
        Vec::with_capacity(n),
    ];
    let names = [
        Edge::LeftHard,
        Edge::RightHard,
        Edge::LeftSoft,
        Edge::RightSoft,
    ];
    for i in 0..n {
        let (x, y, h) = (strip.x[i], strip.y[i], strip.heading[i]);
        let lane = lanes.lane_at(x, y, h);
        for (k, name) in names.into_iter().enumerate() {
            let Some(lane) = &lane else {
                edges[k].push(f64::NAN);
                continue;
            };
            // The point may sit off the lane's centre: its edges are measured from where it is.
            let (nx, ny) = (-lane.heading.sin(), lane.heading.cos());
            let left = (x - lane.x) * nx + (y - lane.y) * ny;
            let edge = if passing && name == Edge::LeftSoft && left > PASS_LEFT {
                Edge::LeftHard
            } else {
                name
            };
            let is_left = matches!(name, Edge::LeftHard | Edge::LeftSoft);
            let value = lane.edge(edge) + if is_left { -left } else { left };
            edges[k].push(py_max(0.0, value));
        }
    }
    let [lh, rh, ls, rs] = edges;
    strip.left_hard = lh;
    strip.right_hard = rh;
    strip.left_soft = ls;
    strip.right_soft = rs;
    strip
}

#[cfg(test)]
mod tests {
    use super::*;

    const TWO_WAY: &str =
        "roads:\n- name: road\n  width: 8.0\n  lanes: 2\n  points: [[-20.0, 0.0], [100.0, 0.0]]\n";

    #[test]
    fn the_car_drives_in_the_right_lane_whichever_way_it_goes() {
        let map = from_yaml(TWO_WAY).unwrap();
        let east = map.lane_at(10.0, 0.5, 0.0).unwrap();
        // Painted 7 m, two lanes of 3.5 m: the right one's centre 1.75 m right.
        assert!((east.y + 1.75).abs() < 1e-12 && east.heading == 0.0);
        let west = map.lane_at(10.0, 0.5, PI).unwrap();
        assert!((west.y - 1.75).abs() < 1e-12);
        assert!(map.lane_at(10.0, 0.5, PI / 2.0).is_none());
        assert!(map.lane_at(10.0, 9.0, 0.0).is_none());
    }

    #[test]
    fn the_line_moves_onto_the_lane_and_the_edges_follow() {
        let map = from_yaml(TWO_WAY).unwrap();
        let route = [(0.0, 0.0), (30.0, 0.0)];
        let line = lane_line(&map, &route, (0.0, 0.0), 0.5, 10.0, SMOOTH, false).unwrap();
        assert!(line.iter().all(|p| (p.1 + 1.75).abs() < 1e-12));
        let strip = Strip {
            spacing: 0.5,
            x: vec![5.0],
            y: vec![-1.75],
            heading: vec![0.0],
            ..Strip::default()
        };
        let s = with_edges(strip, &map, false);
        assert!((s.left_soft[0] - 1.75).abs() < 1e-12);
        assert!((s.left_hard[0] - 5.75).abs() < 1e-12 && (s.right_hard[0] - 2.25).abs() < 1e-12);
    }

    #[test]
    fn a_bad_file_says_which_road() {
        let e = from_yaml("roads:\n- name: x\n  points: [[0, 0], [1, 0]]\n").unwrap_err();
        assert!(e.to_string().contains("road 0: no width"));
        assert!(from_yaml("{}").unwrap().roads().is_empty());
    }
}
