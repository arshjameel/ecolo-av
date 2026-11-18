//! Body roll and the roof lidar: when does flat ground look like an obstacle? No ROS.
//!
//! A port of `aav_lidar`'s `ground.py` (TODO 52e). In a turn the body leans outwards on
//! its springs, and `base_link` leans with it: on the outside of the turn the flat road
//! rises in `base_link` by about (how far out) × tan(roll), and the lowest beams' road
//! points can reach `min_height`: an obstacle that isn't there.
//!
//! | Function | Answers |
//! |---|---|
//! | [`ground_hit`] | one beam: where it meets the flat road, as `base_link` sees it |
//! | [`worst_ground_height`] | every beam all the way round: the highest road point within a range |
//! | [`max_safe_roll`] | the most roll before that point reaches `min_height` |

use std::f64::consts::PI;

/// The VLP-16's 16 lasers, degrees: -15 to +15, 2 apart.
#[must_use]
pub fn vlp16_elevations() -> Vec<f64> {
    (-15..=15)
        .step_by(2)
        .map(|e| f64::from(e).to_radians())
        .collect()
}

/// Where one beam meets the flat road, as `base_link` sees it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    /// m, flat from `base_link`.
    pub range: f64,
    /// m above `base_link`: what `scan.rs` compares with its heights.
    pub height: f64,
}

/// One beam's road point, or `None` if it never comes down to the road.
///
/// `roll`: the body's lean (rad, REP-103: + = the right side down), about x through
/// `base_link`; `lidar`: the lidar's place in `base_link` (m), level with the body;
/// `elevation`, `azimuth`: the beam's direction in the lidar's frame (rad).
#[must_use]
pub fn ground_hit(roll: f64, lidar: [f64; 3], elevation: f64, azimuth: f64) -> Option<Hit> {
    let [lx, ly, lz] = lidar;
    let (dx, dy, dz) = (
        elevation.cos() * azimuth.cos(),
        elevation.cos() * azimuth.sin(),
        elevation.sin(),
    );
    // A point (y, z) in base_link is sin(roll) y + cos(roll) z above the road.
    let (s, c) = roll.sin_cos();
    let start = s * ly + c * lz;
    let down = s * dy + c * dz;
    if down >= 0.0 {
        return None;
    }
    let t = -start / down;
    let (x, y, z) = (lx + t * dx, ly + t * dy, lz + t * dz);
    Some(Hit {
        range: x.hypot(y),
        height: z,
    })
}

/// The highest road point within `max_range`: every beam, all the way round.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Worst {
    /// m: `-inf` when no beam reaches the road within range.
    pub height: f64,
    /// m, flat from `base_link`: NaN with no hit.
    pub range: f64,
    /// rad: the beam's.
    pub elevation: f64,
    /// rad: the beam's.
    pub azimuth: f64,
}

/// The highest road point within `max_range`, every beam `step` rad apart all round.
#[must_use]
pub fn worst_ground_height(
    roll: f64,
    lidar: [f64; 3],
    max_range: f64,
    elevations: &[f64],
    step: f64,
) -> Worst {
    let mut best = Worst {
        height: f64::NEG_INFINITY,
        range: f64::NAN,
        elevation: f64::NAN,
        azimuth: f64::NAN,
    };
    let turns = (2.0 * PI / step).round_ties_even() as u32;
    for &elevation in elevations {
        for i in 0..turns {
            let azimuth = f64::from(i) * step - PI;
            if let Some(hit) = ground_hit(roll, lidar, elevation, azimuth) {
                if hit.range <= max_range && hit.height > best.height {
                    best = Worst {
                        height: hit.height,
                        range: hit.range,
                        elevation,
                        azimuth,
                    };
                }
            }
        }
    }
    best
}

/// The most roll (rad, up to `limit`) at which no road point within `max_range` reaches
/// `min_height`; the same either way round.
#[must_use]
pub fn max_safe_roll(
    lidar: [f64; 3],
    min_height: f64,
    max_range: f64,
    elevations: &[f64],
    limit: f64,
) -> f64 {
    let (mut low, mut high) = (0.0, limit);
    for _ in 0..30 {
        let mid = 0.5 * (low + high);
        let worst = worst_ground_height(mid, lidar, max_range, elevations, 1.0_f64.to_radians());
        if worst.height < min_height {
            low = mid;
        } else {
            high = mid;
        }
    }
    low
}
