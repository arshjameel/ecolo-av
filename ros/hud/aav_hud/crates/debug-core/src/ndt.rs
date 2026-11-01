//! NDT (aav_ndt) on the debug screen: its status in plain words, and its map's occupied
//! cells for the faint wireframes (key M).
//!
//! `/localization/ndt/status` is a diagnostic_msgs/DiagnosticStatus: level OK, WARN or
//! ERROR (LOST); message `TRACKING`, `WAITING` or `LOST`, then `: why` for a poor fit;
//! values `score_per_point`, `overlap`, `iterations`, `match_ms`, `points`,
//! `poor_in_row`, `restarts`, `clouds_dropped`, `x`, `y`, `yaw` (aav_ndt's README).
//!
//! The map is the PCD `pixi run ndt-map drive` writes (map frame, ground removed), read
//! with ndt-core's own reader and snapped to cells (1 m by default).

use std::{
    collections::{HashMap, HashSet},
    fmt,
    fs::File,
    path::Path,
};

use map_core::geom::Point;

use crate::cubes;

/// What NDT is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NdtState {
    /// No start pose yet.
    Waiting,
    /// Matching: the last fit was good, or a few poor ones in a row.
    Tracking,
    /// Too many poor fits in a row: it starts again from ekf_global.
    Lost,
}

impl NdtState {
    /// From the message's first word.
    pub fn from_word(word: &str) -> Option<Self> {
        match word {
            "WAITING" => Some(Self::Waiting),
            "TRACKING" => Some(Self::Tracking),
            "LOST" => Some(Self::Lost),
            _ => None,
        }
    }

    pub fn word(self) -> &'static str {
        match self {
            Self::Waiting => "WAITING",
            Self::Tracking => "TRACKING",
            Self::Lost => "LOST",
        }
    }
}

/// How the last cloud went (the message's level).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fit {
    /// Level OK: a good fit, a pose published.
    Good,
    /// Level WARN: a poor fit, or no start yet.
    Poor,
    /// Level ERROR: LOST.
    Lost,
}

impl Fit {
    /// diagnostic_msgs' levels: 0 OK, 1 WARN, 2 ERROR (3 STALE counts as lost).
    pub fn from_level(level: u8) -> Self {
        match level {
            0 => Self::Good,
            1 => Self::Poor,
            _ => Self::Lost,
        }
    }
}

/// What the panel shows of NDT.
#[derive(Debug, Clone, PartialEq)]
pub struct NdtStatus {
    pub fit: Fit,
    /// `None` if the message's first word is none of NDT's.
    pub state: Option<NdtState>,
    /// The message after its first word: why a fit was poor, or empty.
    pub why: String,
    /// The fit's score per point (good fits score about 3; below 2 is poor).
    pub score_per_point: Option<f64>,
    /// The match alone, wall clock, milliseconds.
    pub match_ms: Option<f64>,
    /// The share of the scan's points that landed in map cubes.
    pub overlap: Option<f64>,
    pub iterations: Option<u64>,
    /// Points matched, after the filters.
    pub points: Option<u64>,
    pub poor_in_row: Option<u64>,
    /// Restarts from ekf_global after LOST.
    pub restarts: Option<u64>,
    /// Clouds that arrived while one was being matched.
    pub clouds_dropped: Option<u64>,
}

impl NdtStatus {
    /// From the message's level, its `message` and its key-value pairs. Unreadable
    /// values are left out.
    pub fn from_message<'a>(
        level: u8,
        message: &str,
        values: impl IntoIterator<Item = (&'a str, &'a str)>,
    ) -> Self {
        let (word, rest) = message
            .split_once(':')
            .map_or((message, ""), |(w, r)| (w, r));
        let mut s = Self {
            fit: Fit::from_level(level),
            state: NdtState::from_word(word.trim()),
            why: rest.trim().to_owned(),
            score_per_point: None,
            match_ms: None,
            overlap: None,
            iterations: None,
            points: None,
            poor_in_row: None,
            restarts: None,
            clouds_dropped: None,
        };
        let number = |v: &str| v.trim().parse::<f64>().ok().filter(|x| x.is_finite());
        let count = |v: &str| v.trim().parse::<u64>().ok();
        for (key, value) in values {
            match key {
                "score_per_point" => s.score_per_point = number(value),
                "match_ms" => s.match_ms = number(value),
                "overlap" => s.overlap = number(value),
                "iterations" => s.iterations = count(value),
                "points" => s.points = count(value),
                "poor_in_row" => s.poor_in_row = count(value),
                "restarts" => s.restarts = count(value),
                "clouds_dropped" => s.clouds_dropped = count(value),
                _ => {}
            }
        }
        s
    }

    /// The panel's big word: TRACKING, WAITING, LOST, or the message itself.
    pub fn headline(&self) -> &'static str {
        match (self.fit, self.state) {
            (Fit::Lost, _) | (_, Some(NdtState::Lost)) => "LOST",
            (_, Some(s)) => s.word(),
            (_, None) => "?",
        }
    }

    /// The numbers, in one short line: "3.41 per point   12.3 ms".
    pub fn numbers(&self) -> String {
        let mut parts = Vec::new();
        if let Some(s) = self.score_per_point {
            parts.push(format!("{s:.2} per point"));
        }
        if let Some(ms) = self.match_ms {
            parts.push(format!("{ms:.1} ms"));
        }
        parts.join("   ")
    }

    /// The smaller numbers: overlap, iterations, restarts, dropped clouds.
    pub fn details(&self) -> String {
        let mut parts = Vec::new();
        if let Some(o) = self.overlap {
            parts.push(format!("overlap {:.0}%", o * 100.0));
        }
        if let Some(i) = self.iterations {
            parts.push(format!("{i} steps"));
        }
        if let Some(r) = self.restarts {
            parts.push(format!("{r} restart{}", if r == 1 { "" } else { "s" }));
        }
        if let Some(d) = self.clouds_dropped.filter(|&d| d > 0) {
            parts.push(format!("{d} clouds dropped"));
        }
        parts.join(", ")
    }
}

/// NDT's map, as the screen has it.
#[derive(Debug, Clone, PartialEq)]
pub enum MapFile {
    /// No file at the path: say how to make one.
    Missing {
        path: String,
    },
    /// The file is there but can't be read.
    Unreadable {
        path: String,
        why: String,
    },
    Loaded {
        path: String,
        cells: MapCells,
    },
}

impl MapFile {
    /// What `pixi run ndt-map drive` makes, said once for the panel and the README.
    pub const HOW_TO_MAKE: &'static str =
        "make it with pixi run ndt-map drive (about 6 min), then pixi run debugdemo";

    /// Reads `path` (a PCD) into cells of `size` metres. An empty path is missing.
    pub fn read(path: &str, size: f64) -> Self {
        let owned = path.to_owned();
        if path.trim().is_empty() || !Path::new(path).is_file() {
            return Self::Missing { path: owned };
        }
        let read = File::open(path)
            .map_err(|e| e.to_string())
            .and_then(|f| ndt_core::pcd::read(f).map_err(|e| e.to_string()));
        match read {
            Ok(points) => Self::Loaded {
                path: owned,
                cells: MapCells::from_points(points.iter().map(|p| [p.x, p.y, p.z]), size),
            },
            Err(why) => Self::Unreadable { path: owned, why },
        }
    }

    pub fn cells(&self) -> Option<&MapCells> {
        match self {
            Self::Loaded { cells, .. } => Some(cells),
            _ => None,
        }
    }
}

impl fmt::Display for MapFile {
    /// One line for the panel.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Missing { path } => write!(f, "no NDT map at {path}: {}", Self::HOW_TO_MAKE),
            Self::Unreadable { path, why } => write!(f, "NDT map {path}: {why}"),
            Self::Loaded { path, cells } => {
                write!(f, "map {path}: {} cubes of {} m", cells.len(), cells.size)
            }
        }
    }
}

/// The map's occupied cells: their centres in the map frame; and the map seen from
/// above, for the seeing screen's two top-downs.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MapCells {
    /// One cell's edge, metres.
    pub size: f32,
    pub centres: Vec<[f32; 3]>,
    /// The map from above: its points, thinned, and each column's ellipse.
    pub footprint: Footprint,
}

/// One column of the map (a cell seen from above, every height) as NDT sees it: the
/// mean and the spread of its points, drawn as an ellipse.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ellipse {
    /// The points' mean, map frame, metres.
    pub centre: [f32; 2],
    /// The two half-axes at two standard deviations, metres, the long one first.
    pub half_axes: [f32; 2],
    /// The long axis' heading, radians from +x (counter-clockwise).
    pub angle: f32,
}

/// The thinnest an ellipse is drawn, metres: a wall seen edge-on still shows.
pub const MIN_HALF_AXIS: f32 = 0.04;
/// The top-down's points: at most one per square of this size, metres.
pub const TOP_DOWN_STEP: f64 = 0.15;

impl Ellipse {
    /// From one column's running sums. None for fewer than [`MIN_POINTS`] points.
    fn from_sums(s: &Sums) -> Option<Self> {
        if s.n < MIN_POINTS {
            return None;
        }
        let k = f64::from(s.n);
        let (mx, my) = (s.sum[0] / k, s.sum[1] / k);
        let xx = (s.sum_sq[0] / k - mx * mx).max(0.0);
        let xy = s.sum_sq[1] / k - mx * my;
        let yy = (s.sum_sq[2] / k - my * my).max(0.0);
        // The 2 x 2 covariance's eigenvalues, and the long axis' angle.
        let mean = (xx + yy) / 2.0;
        let spread = (((xx - yy) / 2.0).powi(2) + xy * xy).sqrt();
        let (long, short) = (mean + spread, (mean - spread).max(0.0));
        let angle = 0.5 * (2.0 * xy).atan2(xx - yy);
        let half = |var: f64| ((2.0 * var.sqrt()) as f32).max(MIN_HALF_AXIS);
        Some(Self {
            centre: [mx as f32, my as f32],
            half_axes: [half(long), half(short)],
            angle: angle as f32,
        })
    }
}

/// One column's running sums, for its ellipse.
#[derive(Debug, Clone, Copy, Default)]
struct Sums {
    n: u32,
    sum: [f64; 2],
    /// x x, x y, y y.
    sum_sq: [f64; 3],
}

impl Sums {
    fn add(&mut self, x: f64, y: f64) {
        self.n += 1;
        self.sum[0] += x;
        self.sum[1] += y;
        self.sum_sq[0] += x * x;
        self.sum_sq[1] += x * y;
        self.sum_sq[2] += y * y;
    }
}

/// The map from above.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Footprint {
    /// The map's points, at most one per [`TOP_DOWN_STEP`] square, map frame.
    pub points: Vec<[f32; 2]>,
    /// One ellipse per column with at least [`MIN_POINTS`] points.
    pub ellipses: Vec<Ellipse>,
    /// The points' corners: lowest x and y, then highest x and y. None for no points.
    pub bounds: Option<[f32; 4]>,
    /// The columns that have an ellipse, by cell index.
    columns: HashSet<[i32; 2]>,
    /// One column's edge, metres.
    size: f64,
}

impl Footprint {
    /// Whether `p` (map frame) falls in one of the map's columns: a scan point that
    /// lands on the map.
    pub fn covers(&self, p: Point) -> bool {
        self.size > 0.0
            && self
                .columns
                .contains(&[cubes::cell(p.x, self.size), cubes::cell(p.y, self.size)])
    }

    /// The ellipses whose centres are within `half` metres of `around`, in x and in y.
    pub fn ellipses_near(&self, around: Point, half: f64) -> impl Iterator<Item = &Ellipse> {
        self.ellipses.iter().filter(move |e| {
            (f64::from(e.centre[0]) - around.x).abs() <= half
                && (f64::from(e.centre[1]) - around.y).abs() <= half
        })
    }
}

/// A cell needs this many of the map's points to count: a stray point isn't a wall.
pub const MIN_POINTS: u32 = 3;

impl MapCells {
    /// The cells of `size` metres holding at least [`MIN_POINTS`] of `points` (map frame).
    /// Points that aren't finite are skipped.
    pub fn from_points(points: impl IntoIterator<Item = [f64; 3]>, size: f64) -> Self {
        let mut counts: HashMap<[i32; 3], u32> = HashMap::new();
        let mut columns: HashMap<[i32; 2], Sums> = HashMap::new();
        let mut thinned: HashSet<[i32; 2]> = HashSet::new();
        let mut top = Vec::new();
        let mut bounds: Option<[f32; 4]> = None;
        for p in points
            .into_iter()
            .filter(|p| p.iter().all(|v| v.is_finite()))
        {
            let [ix, iy, iz] = cubes::cell_of(p, size);
            *counts.entry([ix, iy, iz]).or_default() += 1;
            columns.entry([ix, iy]).or_default().add(p[0], p[1]);
            let square = [
                cubes::cell(p[0], TOP_DOWN_STEP),
                cubes::cell(p[1], TOP_DOWN_STEP),
            ];
            if thinned.insert(square) {
                let (x, y) = (p[0] as f32, p[1] as f32);
                top.push([x, y]);
                bounds = Some(match bounds {
                    None => [x, y, x, y],
                    Some([a, b, c, d]) => [a.min(x), b.min(y), c.max(x), d.max(y)],
                });
            }
        }
        let mut ellipses: Vec<([i32; 2], Ellipse)> = columns
            .iter()
            .filter_map(|(&c, s)| Ellipse::from_sums(s).map(|e| (c, e)))
            .collect();
        ellipses.sort_by_key(|&(c, _)| c);
        let footprint = Footprint {
            points: top,
            columns: ellipses.iter().map(|&(c, _)| c).collect(),
            ellipses: ellipses.into_iter().map(|(_, e)| e).collect(),
            bounds,
            size,
        };
        let mut centres: Vec<[f32; 3]> = counts
            .into_iter()
            .filter(|&(_, n)| n >= MIN_POINTS)
            .map(|(c, _)| c.map(|i| ((f64::from(i) + 0.5) * size) as f32))
            .collect();
        // A steady order, so a redraw never flickers between equal cells.
        centres.sort_by(|a, b| {
            a.iter()
                .zip(b)
                .fold(std::cmp::Ordering::Equal, |o, (x, y)| {
                    o.then(x.total_cmp(y))
                })
        });
        Self {
            size: size as f32,
            centres,
            footprint,
        }
    }

    pub fn len(&self) -> usize {
        self.centres.len()
    }

    pub fn is_empty(&self) -> bool {
        self.centres.is_empty()
    }

    /// At most `max` cells, the nearest `to` (map frame, seen from above).
    pub fn nearest(&self, to: Point, max: usize) -> Vec<[f32; 3]> {
        let mut v = self.centres.clone();
        cubes::keep_nearest_by(&mut v, |c| *c, to, max);
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tracking() -> NdtStatus {
        NdtStatus::from_message(
            0,
            "TRACKING",
            [
                ("points", "812"),
                ("poor_in_row", "0"),
                ("restarts", "1"),
                ("clouds_dropped", "0"),
                ("match_ms", "12.34"),
                ("score_per_point", "3.412"),
                ("overlap", "0.870"),
                ("iterations", "7"),
                ("x", "1.0"),
            ],
        )
    }

    #[test]
    fn reads_a_good_fit() {
        let s = tracking();
        assert_eq!((s.fit, s.state), (Fit::Good, Some(NdtState::Tracking)));
        assert_eq!(s.headline(), "TRACKING");
        assert_eq!(s.numbers(), "3.41 per point   12.3 ms");
        assert_eq!(s.details(), "overlap 87%, 7 steps, 1 restart");
        assert_eq!(s.points, Some(812));
    }

    #[test]
    fn reads_a_poor_fit_and_lost() {
        let poor = NdtStatus::from_message(
            1,
            "TRACKING: a poor fit (score per point too low)",
            [("score_per_point", "1.2")],
        );
        assert_eq!(poor.fit, Fit::Poor);
        assert_eq!(poor.why, "a poor fit (score per point too low)");
        let lost = NdtStatus::from_message(2, "LOST: a poor fit (out of iterations)", []);
        assert_eq!(lost.headline(), "LOST");
        let waiting = NdtStatus::from_message(1, "WAITING: no start pose yet", []);
        assert_eq!(waiting.headline(), "WAITING");
        assert_eq!(waiting.numbers(), "");
        let odd = NdtStatus::from_message(0, "hello", [("match_ms", "nan")]);
        assert_eq!((odd.headline(), odd.match_ms), ("?", None));
    }

    #[test]
    fn a_map_keeps_cells_with_enough_points() {
        let wall = (0..30).map(|i| [0.5, f64::from(i) * 0.1, 1.2]);
        let stray = [[10.2, 10.2, 1.0], [f64::NAN, 0.0, 0.0]];
        let cells = MapCells::from_points(wall.chain(stray), 1.0);
        // 3 m of wall: cells y = 0, 1, 2; the stray point isn't one.
        assert_eq!(
            cells.centres,
            vec![[0.5, 0.5, 1.5], [0.5, 1.5, 1.5], [0.5, 2.5, 1.5]]
        );
        assert_eq!(
            cells.nearest(Point::new(0.0, 3.0), 1),
            vec![[0.5, 2.5, 1.5]]
        );
    }

    #[test]
    fn a_wall_is_a_long_thin_ellipse_along_it() {
        // A wall along y = 0.5, x from 0 to 0.9, at two heights: one column.
        let wall = (0..10).flat_map(|i| [0.3, 1.1].map(|z| [f64::from(i) * 0.1, 0.5, z]));
        let cells = MapCells::from_points(wall, 1.0);
        assert_eq!(cells.footprint.ellipses.len(), 1);
        let e = cells.footprint.ellipses[0];
        assert!((e.centre[0] - 0.45).abs() < 1e-5 && (e.centre[1] - 0.5).abs() < 1e-5);
        // Along x: 2 sigma of 0, 0.1 ... 0.9 is about 0.57 m; across it, the sliver.
        assert!((e.half_axes[0] - 0.574).abs() < 0.01, "{:?}", e.half_axes);
        assert_eq!(e.half_axes[1], MIN_HALF_AXIS);
        assert!(e.angle.abs() < 1e-5);
        assert!(cells.footprint.covers(Point::new(0.9, 0.1)));
        assert!(!cells.footprint.covers(Point::new(1.1, 0.1)));
        assert_eq!(cells.footprint.bounds, Some([0.0, 0.5, 0.9, 0.5]));
        // Thinned to one point per 15 cm square, whatever the height.
        assert_eq!(cells.footprint.points.len(), 7);
    }

    #[test]
    fn a_diagonal_wall_leans_at_45_degrees() {
        let wall = (0..30).map(|i| {
            let t = f64::from(i) * 0.03;
            [t, t, 1.0]
        });
        let cells = MapCells::from_points(wall, 1.0);
        let e = cells.footprint.ellipses[0];
        assert!(
            (e.angle - std::f32::consts::FRAC_PI_4).abs() < 1e-4,
            "{}",
            e.angle
        );
        let fp = &cells.footprint;
        assert_eq!(fp.ellipses_near(Point::new(3.0, 3.0), 1.0).count(), 0);
        assert_eq!(fp.ellipses_near(Point::new(0.5, 0.5), 1.0).count(), 1);
    }

    #[test]
    fn a_missing_map_says_how_to_make_one() {
        let m = MapFile::read("/no/such/map.pcd", 1.0);
        assert!(matches!(m, MapFile::Missing { .. }));
        assert!(m.to_string().contains("pixi run ndt-map drive"));
        assert!(matches!(MapFile::read("", 1.0), MapFile::Missing { .. }));
    }

    #[test]
    fn a_pcd_file_becomes_cells() {
        let dir = std::env::temp_dir().join(format!("debug-core-ndt-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("a temp folder");
        let path = dir.join("lot.pcd");
        let points: Vec<ndt_core::Point> = (0..10)
            .map(|i| ndt_core::Point::new(2.1, 0.05 * f64::from(i), 0.8))
            .collect();
        let mut file = File::create(&path).expect("a temp file");
        ndt_core::pcd::write(&mut file, &points).expect("written");
        drop(file);
        let m = MapFile::read(path.to_str().expect("utf-8"), 1.0);
        let cells = m.cells().expect("loaded");
        assert_eq!(cells.centres, vec![[2.5, 0.5, 0.5]]);
        std::fs::write(&path, "not a pcd").expect("written");
        assert!(matches!(
            MapFile::read(path.to_str().expect("utf-8"), 1.0),
            MapFile::Unreadable { .. }
        ));
        let _ = std::fs::remove_dir_all(dir);
    }
}
