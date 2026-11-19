//! A made-up lot for the tests: a walled yard with boxes in it (none of it symmetric, so
//! one pose fits best), a scan cast from a true pose, and AMCL set up as amcl.yaml has it.

use amcl_core::{
    Amcl, AmclSettings, Cell, DifferentialMotion, DistanceField, FilterSettings, FreeSpace,
    LikelihoodField, Metres, OccupancyMap, Pose2, Radians, Scan,
};

pub const RES: f64 = 0.1;
pub const RANGE_MAX: f64 = 30.0;

/// 30 m x 20 m, origin at (-5, -5): walls round it, and boxes of different sizes.
pub fn yard() -> OccupancyMap {
    let (w, h) = (300, 200);
    let boxes: [(f64, f64, f64, f64); 5] = [
        (2.0, 3.0, 4.0, 4.0), // x0, y0, x1, y1 in metres from the corner
        (12.0, 15.0, 13.0, 17.5),
        (20.0, 4.0, 24.0, 5.0),
        (7.0, 13.0, 8.0, 14.0),
        (26.0, 14.0, 27.5, 16.0),
    ];
    let cells = (0..w * h)
        .map(|k| {
            let (i, j) = (k % w, k / w);
            let (x, y) = ((i as f64 + 0.5) * RES, (j as f64 + 0.5) * RES);
            let wall = i < 2 || j < 2 || i >= w - 2 || j >= h - 2;
            let in_box = boxes
                .iter()
                .any(|&(x0, y0, x1, y1)| x >= x0 && x <= x1 && y >= y0 && y <= y1);
            if wall || in_box {
                Cell::Occupied
            } else {
                Cell::Free
            }
        })
        .collect();
    OccupancyMap::new(w, h, Metres(RES), Pose2::new(-5.0, -5.0, 0.0), cells).expect("the yard")
}

/// A 720-beam scan from `truth`, by stepping along each beam until it meets a wall.
pub fn cast(map: &OccupancyMap, truth: &Pose2) -> Scan {
    let n = 720;
    let inc = std::f64::consts::TAU / n as f64;
    let angle_min = -std::f64::consts::PI;
    let ranges: Vec<f32> = (0..n)
        .map(|k| {
            let a = truth.yaw + angle_min + k as f64 * inc;
            let (s, c) = a.sin_cos();
            let mut r = 0.0;
            while r < RANGE_MAX {
                r += RES / 4.0;
                let hit = map
                    .cell_of(truth.x + r * c, truth.y + r * s)
                    .and_then(|(i, j)| map.cell(i, j))
                    .is_none_or(|cell| cell == Cell::Occupied);
                if hit {
                    return r as f32;
                }
            }
            f32::INFINITY
        })
        .collect();
    Scan::from_ranges(
        &ranges,
        angle_min,
        inc,
        (0.2, RANGE_MAX),
        (0.2, RANGE_MAX),
        Pose2::default(),
    )
}

/// AMCL on the yard, with amcl.yaml's tuning, and recovery's rates.
pub fn amcl(map: &OccupancyMap, recovery: (f64, f64), seed: u64) -> Amcl<LikelihoodField> {
    let model = LikelihoodField::new(DistanceField::new(map, Metres(2.0)), 0.9, 0.1, 0.2, 180);
    let settings = AmclSettings {
        filter: FilterSettings {
            alpha_slow: recovery.0,
            alpha_fast: recovery.1,
            ..FilterSettings::default()
        },
        motion: DifferentialMotion {
            alpha1: 0.05,
            alpha2: 0.01,
            alpha3: 0.02,
            alpha4: 0.01,
        },
        update_min_d: Metres(0.25),
        update_min_a: Radians(0.2),
        resample_interval: 2,
        seed,
        ..AmclSettings::default()
    };
    Amcl::new(settings, model, FreeSpace::new(map)).expect("settings")
}

/// A drive round the yard's open middle: (x, y, yaw) every 0.3 m, a loop.
pub fn route(steps: usize) -> Vec<Pose2> {
    (0..steps)
        .map(|k| {
            // An ellipse round (10, 5), 8 m by 4 m, anticlockwise.
            let t = k as f64 * 0.3 / 6.0;
            let (s, c) = t.sin_cos();
            let (x, y) = (10.0 + 8.0 * c, 5.0 + 4.0 * s);
            let yaw = (4.0 * c).atan2(-8.0 * s);
            Pose2::new(x, y, yaw)
        })
        .collect()
}
