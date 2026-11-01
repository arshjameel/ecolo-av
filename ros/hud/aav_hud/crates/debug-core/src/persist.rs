//! Cubes that stay and fade: the "occupancy network" look (key K).
//!
//! Each scan's cubes are kept in odom, by their grid cell, with the stamp of the last
//! scan that saw them. A cube not seen again fades over `keep_for` (2 s by default) and
//! then goes. Age is measured in the clouds' own stamps, never the window's clock
//! (Gazebo stamps sim time), so when the lidar stops, nothing fades: the panel's lidar
//! row says STALE instead. Over `max_cubes`, the nearest to the car are kept.

use std::{collections::HashMap, time::Duration};

use crate::{
    cubes::{self, Band, Cube, CubeScan},
    frames::Stamp,
};

/// How faded a kept cube is: four steps, so the screen draws a few batches, not one
/// colour per cube.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Fade {
    /// Seen in the newest scan (or within the first quarter of `keep_for`).
    New,
    Recent,
    Older,
    /// About to go.
    Oldest,
}

impl Fade {
    /// Every step, newest first.
    pub const ALL: [Fade; 4] = [Fade::New, Fade::Recent, Fade::Older, Fade::Oldest];

    /// The step of a cube last seen `age` ago; `None` once it is older than `keep_for`.
    pub fn of(age: Duration, keep_for: Duration) -> Option<Self> {
        if age > keep_for {
            return None;
        }
        let quarter = keep_for.as_secs_f64() / 4.0;
        let step = if quarter > 0.0 {
            (age.as_secs_f64() / quarter) as usize
        } else {
            0
        };
        Some(Self::ALL[step.min(3)])
    }

    /// How much of its colour it keeps: 1 new, less as it fades (towards the background).
    pub fn brightness(self) -> f32 {
        match self {
            Self::New => 1.0,
            Self::Recent => 0.7,
            Self::Older => 0.45,
            Self::Oldest => 0.25,
        }
    }
}

/// A cube to draw, and how faded it is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FadedCube {
    /// Its centre in odom, metres.
    pub centre: [f32; 3],
    pub band: Band,
    pub fade: Fade,
}

impl FadedCube {
    /// A cube from this scan: not faded at all.
    pub fn new(c: &Cube) -> Self {
        Self {
            centre: c.centre,
            band: c.band,
            fade: Fade::New,
        }
    }
}

/// The kept cubes.
#[derive(Debug, Clone)]
pub struct KeptCubes {
    keep_for: Duration,
    max: usize,
    /// The edge of every cube; a scan of another size starts again.
    size: f32,
    /// Each cell: its centre, band, and the stamp of the last scan that saw it.
    cells: HashMap<[i32; 3], (Cube, Stamp)>,
    /// The newest scan's stamp.
    latest: Option<Stamp>,
    drawn: Vec<FadedCube>,
    cut: usize,
}

impl KeptCubes {
    /// Cubes kept for `keep_for` after they were last seen; at most `max` drawn.
    pub fn new(keep_for: Duration, max: usize) -> Self {
        Self {
            keep_for,
            max,
            size: 0.0,
            cells: HashMap::new(),
            latest: None,
            drawn: Vec::new(),
            cut: 0,
        }
    }

    /// Adds a scan: its cubes are seen now; older ones fade, the oldest go.
    pub fn add(&mut self, scan: &CubeScan) {
        // A clock that went back (the sim restarted, a bag looped) or another cube
        // size: start again.
        let went_back = self.latest.is_some_and(|l| scan.stamp < l);
        if went_back || (scan.size - self.size).abs() > f32::EPSILON {
            self.cells.clear();
            self.size = scan.size;
        }
        self.latest = Some(scan.stamp);
        let size = f64::from(scan.size);
        for c in &scan.cubes {
            let key = cubes::cell_of(c.centre.map(f64::from), size);
            self.cells.insert(key, (*c, scan.stamp));
        }
        let (now, keep_for) = (scan.stamp, self.keep_for);
        let age = |seen: Stamp| Duration::from_secs_f64(now.seconds_since(seen).max(0.0));
        self.cells
            .retain(|_, (_, seen)| Fade::of(age(*seen), keep_for).is_some());
        self.drawn = self
            .cells
            .values()
            .filter_map(|(c, seen)| {
                Fade::of(age(*seen), keep_for).map(|fade| FadedCube {
                    centre: c.centre,
                    band: c.band,
                    fade,
                })
            })
            .collect();
        self.cut =
            cubes::keep_nearest_by(&mut self.drawn, |c| c.centre, scan.car.position, self.max);
    }

    /// What to draw: at most `max`, the nearest the car at the newest scan.
    pub fn cubes(&self) -> &[FadedCube] {
        &self.drawn
    }

    /// How many kept cubes are left out over `max` (the farthest).
    pub fn cut(&self) -> usize {
        self.cut
    }

    /// How many cells are kept (some may be cut from the drawing).
    pub fn len(&self) -> usize {
        self.cells.len()
    }

    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use map_core::geom::Pose;

    fn scan(t_ms: i64, xs: &[f32]) -> CubeScan {
        CubeScan {
            stamp: Stamp(t_ms * 1_000_000),
            car: Pose::default(),
            cubes: xs
                .iter()
                .map(|&x| Cube {
                    centre: [x + 0.1, 0.1, 1.1],
                    band: Band::Waist,
                })
                .collect(),
            size: 0.2,
            ..CubeScan::default()
        }
    }

    fn fades(k: &KeptCubes) -> Vec<(i32, Fade)> {
        let mut v: Vec<(i32, Fade)> = k
            .cubes()
            .iter()
            .map(|c| ((c.centre[0] * 10.0).round() as i32, c.fade))
            .collect();
        v.sort_by_key(|&(x, _)| x);
        v
    }

    #[test]
    fn steps_fade_then_go() {
        let two = Duration::from_secs(2);
        assert_eq!(Fade::of(Duration::ZERO, two), Some(Fade::New));
        assert_eq!(
            Fade::of(Duration::from_millis(600), two),
            Some(Fade::Recent)
        );
        assert_eq!(
            Fade::of(Duration::from_millis(1100), two),
            Some(Fade::Older)
        );
        assert_eq!(Fade::of(two, two), Some(Fade::Oldest));
        assert_eq!(Fade::of(Duration::from_millis(2001), two), None);
        assert!(Fade::ALL
            .windows(2)
            .all(|w| w[0].brightness() > w[1].brightness()));
    }

    #[test]
    fn a_cube_not_seen_again_fades_and_goes() {
        let mut k = KeptCubes::new(Duration::from_secs(2), 100);
        k.add(&scan(0, &[1.0, 2.0]));
        assert_eq!(fades(&k), vec![(11, Fade::New), (21, Fade::New)]);
        // 1.2 s later only the cube at x = 1 is seen again.
        k.add(&scan(1200, &[1.0]));
        assert_eq!(fades(&k), vec![(11, Fade::New), (21, Fade::Older)]);
        // 2.1 s after the first: the one at x = 2 has gone.
        k.add(&scan(2100, &[1.0]));
        assert_eq!(fades(&k), vec![(11, Fade::New)]);
        assert_eq!(k.len(), 1);
    }

    #[test]
    fn the_same_cell_is_kept_once() {
        let mut k = KeptCubes::new(Duration::from_secs(2), 100);
        k.add(&scan(0, &[1.0]));
        k.add(&scan(100, &[1.0]));
        assert_eq!(k.cubes().len(), 1);
    }

    #[test]
    fn over_the_cap_the_nearest_are_drawn() {
        let mut k = KeptCubes::new(Duration::from_secs(2), 3);
        k.add(&scan(0, &[8.0, 9.0]));
        k.add(&scan(100, &[1.0, 2.0, 3.0]));
        assert_eq!(k.cut(), 2);
        assert!(k.cubes().iter().all(|c| c.centre[0] < 4.0));
        assert_eq!(k.len(), 5);
    }

    #[test]
    fn a_clock_that_goes_back_starts_again() {
        let mut k = KeptCubes::new(Duration::from_secs(2), 100);
        k.add(&scan(5000, &[1.0, 2.0]));
        k.add(&scan(100, &[3.0]));
        assert_eq!(fades(&k), vec![(31, Fade::New)]);
    }
}
