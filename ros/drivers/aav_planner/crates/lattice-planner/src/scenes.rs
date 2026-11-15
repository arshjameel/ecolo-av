//! Hand-made maps for the tests and the examples: our car, our lattice file and
//! nav2.yaml's global costmap (0.25 m cells, 3 m of inflation at scaling 1.0); and the
//! example lot (`map`) with the lot tour's legs (`LOT_TOUR`).

use costmap::cost::{LETHAL, NO_INFORMATION};
use costmap::{footprint, Grid, Inflation};

use crate::{Hybrid, Lattice, Planner, Pose, Settings};

/// vehicle.yaml's body (2.0 m x 1.0 m, the rear axle 0.2 m from the back), as
/// `vehicle.footprint_points` gives it.
pub const CAR: [(f64, f64); 4] = [(1.8, 0.5), (1.8, -0.5), (-0.2, -0.5), (-0.2, 0.5)];
/// nav2.yaml's global costmap.
pub const PADDING: f64 = 0.05;
pub const RESOLUTION: f64 = 0.25;
pub const INFLATION_RADIUS: f64 = 3.0;
pub const COST_SCALING: f64 = 1.0;

/// Our lattice file (aav_navigation's config/lattice_ackermann.json).
pub fn lattice() -> Lattice {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../aav_navigation/config/lattice_ackermann.json"
    );
    let text = std::fs::read_to_string(path).expect("aav_navigation's lattice file");
    Lattice::parse(&text).expect("a good lattice file")
}

pub fn outline() -> Vec<(f64, f64)> {
    footprint::padded(&CAR, PADDING)
}

pub fn inflation() -> Inflation {
    Inflation {
        inflation_radius: INFLATION_RADIUS,
        cost_scaling_factor: COST_SCALING,
        inscribed_radius: footprint::radii(&outline()).0,
    }
}

/// A planner as navigation.launch.py starts ours.
pub fn planner(reversing: bool) -> Planner {
    let settings = Settings {
        allow_reverse: reversing,
        ..Settings::default()
    };
    Planner::new(lattice(), settings, &outline(), Some(&inflation()))
}

/// Hybrid-A* as navigation.launch.py starts ours with `planner_search:=hybrid`: nav2.yaml's
/// `GridBased` block, at the lattice file's radius (both are vehicle.plan_turning_radius).
pub fn hybrid_planner(reversing: bool) -> Planner {
    let settings = Settings {
        allow_reverse: reversing,
        ..Settings::hybrid()
    };
    let spec = Hybrid::new(lattice().turning_radius);
    Planner::hybrid(spec, settings, &outline(), Some(&inflation()), RESOLUTION)
        .expect("nav2.yaml's Hybrid-A* block is good")
}

/// `measure controllers --tours lot`: the start, then each leg's goal (x, y, heading
/// degrees), on `example_lot`.
pub const LOT_START: Pose = Pose::new(0.0, 0.0, 0.0);
pub const LOT_TOUR: [(&str, f64, f64, f64); 4] = [
    ("a left turn", 15.0, 15.0, 90.0),
    ("round the building", 35.0, 42.0, 0.0),
    ("turn back", 20.0, 42.0, 180.0),
    ("home", 0.0, 0.0, 0.0),
];

/// aav_navigation's map `name` (`maps/<name>.pgm`, binary P5, with `example_lot.yaml`'s
/// 0.25 m cells and origin (-10, -10)), read as map_server's trinary mode does (occupied
/// above 0.65, free under 0.25, else unknown), then inflated as nav2.yaml's global costmap.
pub fn map(name: &str) -> Result<Grid, String> {
    let path = format!(
        "{}/../../../aav_navigation/maps/{name}.pgm",
        env!("CARGO_MANIFEST_DIR")
    );
    read_pgm(&path, (-10.0, -10.0), None)
}

/// `measure controllers --tours campus`: the start (lot P4, `maps/campus/place.yaml`), then each leg's goal (x, y,
/// heading degrees), on `pixi run campus`'s map.
pub const CAMPUS_START: Pose = Pose::new(49.25, -70.75, 0.0);
pub const CAMPUS_TOUR: [(&str, f64, f64, f64); 3] = [
    ("out of the lot", 73.0, -38.0, 45.0),
    ("along the road", 165.0, -111.0, -40.0),
    ("round the bend", 139.0, -190.0, 109.0),
];

/// The campus map (`pixi run campus`: `maps/campus/campus.pgm`, 0.25 m cells, origin
/// (-900, -900)), cut to the campus tour's area, x -60..260 m and y -260..60 m, and
/// inflated as nav2.yaml's global costmap. An error if the map isn't made yet.
pub fn campus() -> Result<Grid, String> {
    let path = format!(
        "{}/../../../../../maps/campus/campus.pgm",
        env!("CARGO_MANIFEST_DIR")
    );
    read_pgm(
        &path,
        (-900.0, -900.0),
        Some(((-60.0, -260.0), (260.0, 60.0))),
    )
}

/// A map_server PGM with its origin (m), the part `crop` keeps ((x0, y0), (x1, y1) m; None:
/// all of it), read as map_server's trinary mode does, then inflated.
fn read_pgm(
    path: &str,
    origin: (f64, f64),
    crop: Option<((f64, f64), (f64, f64))>,
) -> Result<Grid, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    // The header: P5, comments, width, height, maxval, one whitespace byte, then the data.
    let mut fields = Vec::new();
    let mut i = 0;
    let at = |i: usize| bytes.get(i).copied().ok_or("the PGM ends in its header");
    while fields.len() < 4 {
        while at(i)?.is_ascii_whitespace() {
            i += 1;
        }
        if at(i)? == b'#' {
            while at(i)? != b'\n' {
                i += 1;
            }
            continue;
        }
        let start = i;
        while !at(i)?.is_ascii_whitespace() {
            i += 1;
        }
        fields.push(String::from_utf8_lossy(&bytes[start..i]).into_owned());
    }
    if fields[0] != "P5" {
        return Err(format!("{path}: not a binary PGM (P5)"));
    }
    let number = |s: &str| s.parse::<usize>().map_err(|e| format!("{path}: {s}: {e}"));
    let (w, h, max) = (
        number(&fields[1])?,
        number(&fields[2])?,
        number(&fields[3])?,
    );
    let data = bytes.get(i + 1..i + 1 + w * h).ok_or("the PGM is short")?;
    let max = max as f64;
    // The cells kept: x from c0 to c1, y (up from the map's bottom) from r0 to r1.
    let cell = |m: f64, o: f64, n: usize| (((m - o) / RESOLUTION).max(0.0) as usize).min(n);
    let ((c0, r0), (c1, r1)) = match crop {
        None => ((0, 0), (w, h)),
        Some(((x0, y0), (x1, y1))) => (
            (cell(x0, origin.0, w), cell(y0, origin.1, h)),
            (cell(x1, origin.0, w), cell(y1, origin.1, h)),
        ),
    };
    let corner = (
        origin.0 + c0 as f64 * RESOLUTION,
        origin.1 + r0 as f64 * RESOLUTION,
    );
    let mut grid = Grid::new(corner, RESOLUTION, c1 - c0, r1 - r0, 0);
    for up in r0..r1 {
        // The PGM's first row is the map's top.
        let row = h - 1 - up;
        for cx in c0..c1 {
            let occupied = (max - f64::from(data[row * w + cx])) / max;
            let cost = if occupied > 0.65 {
                LETHAL
            } else if occupied < 0.25 {
                0
            } else {
                NO_INFORMATION
            };
            grid.set(cx - c0, up - r0, cost);
        }
    }
    inflation().apply(&mut grid);
    Ok(grid)
}

/// m: a narrow campus street's width, kerb to kerb (`angled_road`).
pub const ROAD_WIDTH: f64 = 6.5;

/// A straight road `length` m long, `ROAD_WIDTH` wide, at `angle` (rad) from (10, 10): walls
/// everywhere further than half the width from its centre line, inflated. At an angle
/// between two of the lattice's headings its plan is a staircase (TODO 168).
pub fn angled_road(angle: f64, length: f64) -> Grid {
    let (w, h) = (length + 20.0, length * angle.sin().abs() + 30.0);
    let r = RESOLUTION;
    let mut grid = Grid::new((0.0, 0.0), r, (w / r) as usize, (h / r) as usize, 0);
    let (c, s) = (angle.cos(), angle.sin());
    for cy in 0..grid.height {
        for cx in 0..grid.width {
            let (x, y) = ((cx as f64 + 0.5) * r - 10.0, (cy as f64 + 0.5) * r - 10.0);
            if (-x * s + y * c).abs() > ROAD_WIDTH / 2.0 {
                grid.set(cx, cy, LETHAL);
            }
        }
    }
    inflated(grid)
}

/// One scene: a map, a start, a goal, and whether a path must exist with and without
/// reversing.
pub struct Scene {
    pub name: &'static str,
    pub grid: Grid,
    pub start: Pose,
    pub goal: Pose,
    pub forward_ok: bool,
    pub reverse_ok: bool,
}

/// An empty map, `w` x `h` m, origin at (0, 0).
fn open(w: f64, h: f64) -> Grid {
    Grid::new(
        (0.0, 0.0),
        RESOLUTION,
        (w / RESOLUTION) as usize,
        (h / RESOLUTION) as usize,
        0,
    )
}

/// Mark a box (m) lethal.
fn wall(grid: &mut Grid, x0: f64, y0: f64, x1: f64, y1: f64) {
    let r = grid.resolution;
    for cy in (y0 / r) as usize..(y1 / r).ceil() as usize {
        for cx in (x0 / r) as usize..(x1 / r).ceil() as usize {
            if cx < grid.width && cy < grid.height {
                grid.set(cx, cy, LETHAL);
            }
        }
    }
}

fn inflated(mut grid: Grid) -> Grid {
    inflation().apply(&mut grid);
    grid
}

/// The scenes.
pub fn all() -> Vec<Scene> {
    let mut out = Vec::new();

    out.push(Scene {
        name: "straight",
        grid: inflated(open(40.0, 20.0)),
        start: Pose::new(5.0, 10.0, 0.0),
        goal: Pose::new(30.0, 10.0, 0.0),
        forward_ok: true,
        reverse_ok: true,
    });

    out.push(Scene {
        name: "u-turn",
        grid: inflated(open(40.0, 30.0)),
        start: Pose::new(15.0, 12.0, 0.0),
        goal: Pose::new(15.0, 18.0, std::f64::consts::PI),
        forward_ok: true,
        reverse_ok: true,
    });

    // A U-turn in an 8 m road (walls each side): too narrow to turn forward, so only a
    // three-point turn (reversing) gets round.
    let mut road = open(60.0, 20.0);
    wall(&mut road, 0.0, 0.0, 60.0, 6.0);
    wall(&mut road, 0.0, 14.0, 60.0, 20.0);
    out.push(Scene {
        name: "u-turn in a road",
        grid: inflated(road),
        start: Pose::new(20.0, 8.5, 0.0),
        goal: Pose::new(20.0, 11.5, std::f64::consts::PI),
        forward_ok: false,
        reverse_ok: true,
    });

    // Parked nose-in between two cars, a wall ahead: only reversing gets out.
    let mut lot = open(30.0, 30.0);
    wall(&mut lot, 10.0, 6.0, 15.0, 8.4); // the car on the right
    wall(&mut lot, 10.0, 11.6, 15.0, 14.0); // the car on the left
    wall(&mut lot, 15.0, 5.0, 16.0, 15.0); // the wall ahead
    out.push(Scene {
        name: "parking-lot exit",
        grid: inflated(lot),
        start: Pose::new(12.0, 10.0, 0.0),
        goal: Pose::new(4.0, 22.0, std::f64::consts::FRAC_PI_2),
        forward_ok: false,
        reverse_ok: true,
    });

    // A wall across with a 2.5 m gap: through it.
    let mut gap = open(40.0, 30.0);
    wall(&mut gap, 19.0, 0.0, 21.0, 13.75);
    wall(&mut gap, 19.0, 16.25, 21.0, 30.0);
    out.push(Scene {
        name: "narrow gap",
        grid: inflated(gap),
        start: Pose::new(8.0, 15.0, 0.0),
        goal: Pose::new(32.0, 15.0, 0.0),
        forward_ok: true,
        reverse_ok: true,
    });

    // The same wall with a 1.0 m gap: the car (1.1 m wide padded) can't pass.
    let mut shut = open(40.0, 30.0);
    wall(&mut shut, 19.0, 0.0, 21.0, 14.5);
    wall(&mut shut, 19.0, 15.5, 21.0, 30.0);
    out.push(Scene {
        name: "no path",
        grid: inflated(shut),
        start: Pose::new(8.0, 15.0, 0.0),
        goal: Pose::new(32.0, 15.0, 0.0),
        forward_ok: false,
        reverse_ok: false,
    });
    out
}
