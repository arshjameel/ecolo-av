//! The ground ring by ring (TODO 215, a challenger to [`crate::fit_ground`]'s one plane):
//! Patchwork++'s idea (S. Lee et al., IROS 2022, URL-KAIST; BSD-2-Clause, the idea only).
//!
//! One plane for the whole cloud can't follow a road that bends up a hill or falls to a
//! gutter, so the champion keeps its obstacle floor high (`min_height` 0.3 m) and a 0.3 m
//! kerb is lost against it. Here the ground is a plane **per patch**, so each point is
//! measured against the ground right under it and the floor can come down
//! ([`PatchSettings::min_height`], 0.15 m): the kerb stands out.
//!
//! ```text
//!            zone 3 (2 rings, 32 sectors)
//!        zone 2 (4 rings, 54 sectors)            Concentric zones round base_link: near
//!     zone 1 (4 rings, 32 sectors)               patches small (dense points), far ones
//!   zone 0 (2 rings, 16 sectors)  ● car          bigger (sparse); one plane in each
//! ```
//!
//! | Step, per patch | What |
//! |---|---|
//! | seeds | the lowest points: under the mean of the lowest `lpr_points` plus `seed_band` |
//! | plane | a plane through them by PCA (the normal: the covariance's least eigenvector); then through every point within `ground_band` of it, `iterations` times |
//! | uprightness | the normal within 45° of up, or the patch is not ground (a wall) |
//! | elevation | in the first `elevation.len()` rings, ground higher than `elevation[ring]` is not ground... |
//! | flatness | ...unless it is flatter than `flatness[ring]` (a pavement, a car park's level) |
//! | out | ground: points within `ground_band` of an accepted plane; obstacles: the rest, `min_height` to `max_height` above their patch's plane (a rejected patch's or an empty one's: the whole cloud's road, [`crate::fit_ground`]) |

use std::f64::consts::{PI, TAU};

use crate::{fit_ground, nearest_per_cube, square_centres, usable, CubeShape, Cubes, Plane, Point};

/// How the cloud is cut into patches, and what counts as ground. Metres, in `base_link`.
#[derive(Clone, Debug, PartialEq)]
pub struct PatchSettings {
    /// Each zone's inner edge, then the last zone's outer edge (flat from `base_link`).
    pub zone_edges: Vec<f64>,
    /// Rings in each zone.
    pub rings: Vec<usize>,
    /// Sectors in each zone.
    pub sectors: Vec<usize>,
    /// Fewer points in a patch: no plane of its own.
    pub min_points: usize,
    /// The lowest points whose mean the seeds sit under (Patchwork's LPR).
    pub lpr_points: usize,
    /// m above that mean: still a seed.
    pub seed_band: f64,
    /// m either side of a patch's plane: ground.
    pub ground_band: f64,
    /// Fits a patch.
    pub iterations: usize,
    /// The plane's normal's up part, at least (cos 45°).
    pub uprightness: f64,
    /// m: the most a near ring's ground may sit above `base_link`'s ground, ring by ring.
    pub elevation: Vec<f64>,
    /// m²: an elevated ring's patch flatter than this (its least eigenvalue) is still ground.
    pub flatness: Vec<f64>,
    /// m above a patch's plane: lower is the ground (the champion's 0.3 for one plane).
    pub min_height: f64,
}

impl Default for PatchSettings {
    /// Patchwork++'s numbers (zones 4, rings 2-4-4-4, sectors 16-32-54-32, uprightness
    /// 0.707, th_seeds and th_dist 0.125, flatness 0.0005-0.001), its zones scaled to our
    /// lidar's 30 m reach.
    fn default() -> Self {
        Self {
            zone_edges: vec![1.5, 6.0, 12.0, 20.0, 30.0],
            rings: vec![2, 4, 4, 2],
            sectors: vec![16, 32, 54, 32],
            min_points: 10,
            lpr_points: 20,
            seed_band: 0.125,
            ground_band: 0.125,
            iterations: 3,
            uprightness: (PI / 4.0).cos(),
            elevation: vec![0.5, 0.75, 0.9, 1.1],
            flatness: vec![0.0005, 0.000_725, 0.001, 0.001],
            min_height: 0.15,
        }
    }
}

/// Where a point falls: its zone, ring (counted over all zones) and sector.
fn patch_of(settings: &PatchSettings, p: &Point) -> Option<(usize, usize, usize)> {
    let r = p.flat();
    let edges = &settings.zone_edges;
    let zone = edges.windows(2).position(|w| r >= w[0] && r < w[1])?;
    let (inner, outer) = (edges[zone], edges[zone + 1]);
    let rings = settings.rings.get(zone).copied()?.max(1);
    let sectors = settings.sectors.get(zone).copied()?.max(1);
    let ring = (((r - inner) / (outer - inner) * rings as f64) as usize).min(rings - 1);
    let angle = p.y.atan2(p.x).rem_euclid(TAU);
    let sector = ((angle / TAU * sectors as f64) as usize).min(sectors - 1);
    let before: usize = settings.rings.iter().take(zone).sum();
    Some((zone, before + ring, sector))
}

/// A plane fitted by PCA: its mean, unit normal (up) and the covariance's eigenvalues,
/// least first.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PcaPlane {
    /// The points' mean.
    pub mean: Point,
    /// The unit normal, its z ≥ 0.
    pub normal: [f64; 3],
    /// The covariance's eigenvalues, least first (m²).
    pub eigenvalues: [f64; 3],
}

impl PcaPlane {
    /// Through `points` (3 or more), or `None`.
    #[must_use]
    pub fn fit(points: &[Point]) -> Option<Self> {
        if points.len() < 3 {
            return None;
        }
        let n = points.len() as f64;
        let mean = points.iter().fold(Point::default(), |m, p| {
            Point::new(m.x + p.x / n, m.y + p.y / n, m.z + p.z / n)
        });
        let mut c = [[0.0; 3]; 3];
        for p in points {
            let d = [p.x - mean.x, p.y - mean.y, p.z - mean.z];
            for (i, row) in c.iter_mut().enumerate() {
                for (j, cell) in row.iter_mut().enumerate() {
                    *cell += d[i] * d[j] / n;
                }
            }
        }
        let (values, vectors) = symmetric_eigen(c);
        let least = (0..3).min_by(|&a, &b| values[a].total_cmp(&values[b]))?;
        let mut normal = [vectors[0][least], vectors[1][least], vectors[2][least]];
        if normal[2] < 0.0 {
            normal = normal.map(|v| -v);
        }
        let mut sorted = values;
        sorted.sort_by(f64::total_cmp);
        Some(Self {
            mean,
            normal,
            eigenvalues: sorted,
        })
    }

    /// m from the plane, along its normal (+ above).
    #[must_use]
    pub fn distance(&self, p: &Point) -> f64 {
        let [a, b, c] = self.normal;
        a * (p.x - self.mean.x) + b * (p.y - self.mean.y) + c * (p.z - self.mean.z)
    }

    /// As z = a x + b y + c (`None` for a vertical plane).
    #[must_use]
    pub fn as_plane(&self) -> Option<Plane> {
        let [nx, ny, nz] = self.normal;
        if nz.abs() < 1e-6 {
            return None;
        }
        let (a, b) = (-nx / nz, -ny / nz);
        Some(Plane {
            a,
            b,
            c: self.mean.z - a * self.mean.x - b * self.mean.y,
        })
    }
}

/// The eigenvalues and eigenvectors (columns) of a symmetric 3 x 3 matrix, by Jacobi's
/// rotations.
fn symmetric_eigen(mut a: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off < 1e-15 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for row in &mut a {
                let (akp, akq) = (row[p], row[q]);
                row[p] = c * akp - s * akq;
                row[q] = s * akp + c * akq;
            }
            let (rp, rq) = (a[p], a[q]);
            a[p] = std::array::from_fn(|k| c * rp[k] - s * rq[k]);
            a[q] = std::array::from_fn(|k| s * rp[k] + c * rq[k]);
            for row in &mut v {
                let (vp, vq) = (row[p], row[q]);
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    ([a[0][0], a[1][1], a[2][2]], v)
}

/// What one patch's ground turned out to be.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Verdict {
    /// Too few points for a plane of its own.
    Sparse,
    /// Ground: the plane every point is measured against.
    Ground(PcaPlane),
    /// A plane, but not ground (a wall, a roof): measured against the whole cloud's road.
    NotGround(PcaPlane),
}

/// One patch's ground, by Patchwork++'s checks (`ring`: counted over every zone).
#[must_use]
pub fn judge(points: &[Point], ring: usize, settings: &PatchSettings) -> Verdict {
    if points.len() < settings.min_points.max(3) {
        return Verdict::Sparse;
    }
    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| a.z.total_cmp(&b.z));
    let lowest = settings.lpr_points.clamp(1, sorted.len());
    let lpr = sorted[..lowest].iter().map(|p| p.z).sum::<f64>() / lowest as f64;
    let mut ground: Vec<Point> = sorted
        .iter()
        .copied()
        .filter(|p| p.z < lpr + settings.seed_band)
        .collect();
    let mut plane = None;
    for _ in 0..settings.iterations.max(1) {
        let Some(fit) = PcaPlane::fit(&ground) else {
            break;
        };
        ground = sorted
            .iter()
            .copied()
            .filter(|p| fit.distance(p).abs() < settings.ground_band)
            .collect();
        plane = Some(fit);
    }
    let Some(plane) = plane else {
        return Verdict::Sparse;
    };
    if plane.normal[2] < settings.uprightness {
        return Verdict::NotGround(plane);
    }
    if let Some(&limit) = settings.elevation.get(ring) {
        let flat = settings
            .flatness
            .get(ring)
            .is_some_and(|&f| plane.eigenvalues[0] < f);
        if plane.mean.z > limit && !flat {
            return Verdict::NotGround(plane);
        }
    }
    Verdict::Ground(plane)
}

/// One cloud's obstacle cubes and ground squares from its points in `base_link`, the
/// ground found patch by patch (`ground:=patchwork`): a drop-in for [`crate::cubes`].
/// `shape.min_height` is the floor where a point has no patch of its own (the whole
/// cloud's road, as the champion); `settings.min_height` where it has.
#[must_use]
pub fn cubes(points: &[Point], shape: &CubeShape, settings: &PatchSettings) -> Cubes {
    let good = usable(points, shape);
    let road = fit_ground(&good, shape);
    let global = road.plane();

    let mut patches: std::collections::HashMap<(usize, usize, usize), Vec<usize>> =
        std::collections::HashMap::new();
    for (i, p) in good.iter().enumerate() {
        if let Some(key) = patch_of(settings, p) {
            patches.entry(key).or_default().push(i);
        }
    }
    // Each point's plane and floor, the whole cloud's where it has no patch of its own.
    let mut under: Vec<(Option<Plane>, f64)> = vec![(None, shape.min_height); good.len()];
    for ((_, ring, _), members) in &patches {
        let mine: Vec<Point> = members.iter().map(|&i| good[i]).collect();
        if let Verdict::Ground(plane) = judge(&mine, *ring, settings) {
            let local = plane.as_plane();
            for &i in members {
                under[i] = (local, settings.min_height);
            }
        }
    }
    let mut high = Vec::new();
    let mut on_road = Vec::new();
    for (p, (plane, floor)) in good.iter().zip(&under) {
        let base = plane.unwrap_or(global);
        let above = p.z - base.height(p.x, p.y);
        let band = if plane.is_some() {
            settings.ground_band
        } else {
            shape.ground_band
        };
        if above.abs() <= band {
            on_road.push(*p);
        } else if (*floor..=shape.max_height).contains(&above) {
            high.push(*p);
        }
    }
    let ground = square_centres(&on_road, shape.size)
        .into_iter()
        .map(|c| Point {
            z: global.height(c.x, c.y),
            ..c
        })
        .collect();
    Cubes {
        obstacles: nearest_per_cube(&high, shape.size),
        ground,
        road,
        points: good.len(),
    }
}

/// Which ground finder the voxels node runs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Ground {
    /// One plane for the cloud ([`crate::fit_ground`]): the champion.
    #[default]
    Plane,
    /// A plane a patch ([`cubes`]): TODO 215, a challenger.
    Patchwork,
}

impl std::str::FromStr for Ground {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.trim() {
            "" | "plane" => Ok(Self::Plane),
            "patchwork" => Ok(Self::Patchwork),
            other => Err(format!("ground:={other}: plane or patchwork")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cubes as champion;

    /// A grid of points every `step` m over the ring 2..20 m round base_link, its height
    /// `z(x, y)`.
    fn ground(step: f64, z: impl Fn(f64, f64) -> f64) -> Vec<Point> {
        let n = (20.0 / step) as i32;
        let mut out = Vec::new();
        for i in -n..=n {
            for j in -n..=n {
                let (x, y) = (f64::from(i) * step, f64::from(j) * step);
                let r = x.hypot(y);
                if (2.0..20.0).contains(&r) {
                    out.push(Point::new(x, y, z(x, y)));
                }
            }
        }
        out
    }

    fn near_kerb(p: &Point) -> bool {
        (2.9..3.6).contains(&p.y)
    }

    #[test]
    fn pca_finds_a_tilted_planes_normal() {
        let pts: Vec<Point> = (0..100)
            .map(|i| {
                let (x, y) = (f64::from(i % 10), f64::from(i / 10));
                Point::new(x, y, 0.1 * x + 0.05 * y + 1.0)
            })
            .collect();
        let fit = PcaPlane::fit(&pts).expect("a plane");
        let plane = fit.as_plane().expect("not vertical");
        assert!((plane.a - 0.1).abs() < 1e-9 && (plane.b - 0.05).abs() < 1e-9);
        assert!((plane.c - 1.0).abs() < 1e-9);
        assert!(fit.eigenvalues[0].abs() < 1e-12);
        assert!(fit.distance(&Point::new(0.0, 0.0, 2.0)) > 0.0);
    }

    #[test]
    fn a_wall_is_not_ground_and_a_flat_road_is() {
        let s = PatchSettings::default();
        let road: Vec<Point> = (0..50)
            .map(|i| Point::new(f64::from(i % 10) * 0.1, f64::from(i / 10) * 0.1, 0.0))
            .collect();
        assert!(matches!(judge(&road, 0, &s), Verdict::Ground(_)));
        let wall: Vec<Point> = (0..50)
            .map(|i| Point::new(5.0, f64::from(i % 10) * 0.1, f64::from(i / 10) * 0.3))
            .collect();
        assert!(matches!(judge(&wall, 0, &s), Verdict::NotGround(_)));
        assert_eq!(judge(&road[..5], 0, &s), Verdict::Sparse);
    }

    #[test]
    fn a_raised_rough_patch_near_the_car_is_not_ground_but_a_flat_one_is() {
        let s = PatchSettings::default();
        // 1 m up in ring 0: a car's bonnet, bumpy.
        let rough: Vec<Point> = (0..50)
            .map(|i| {
                let (x, y) = (f64::from(i % 10) * 0.1, f64::from(i / 10) * 0.1);
                Point::new(x, y, 1.0 + 0.1 * (7.0 * x).sin() * (5.0 * y).cos())
            })
            .collect();
        assert!(matches!(judge(&rough, 0, &s), Verdict::NotGround(_)));
        let level: Vec<Point> = rough.iter().map(|p| Point::new(p.x, p.y, 1.0)).collect();
        assert!(matches!(judge(&level, 0, &s), Verdict::Ground(_)));
    }

    #[test]
    fn a_kerb_stands_out_ring_by_ring() {
        // The road at 0, a 0.3 m kerb at y = 3 m and the pavement beyond it, its face
        // every 5 cm; a point every 0.1 m.
        let mut cloud = ground(0.1, |_, y| if y >= 3.0 { 0.3 } else { 0.0 });
        for i in 0..200 {
            for k in 1..6 {
                cloud.push(Point::new(
                    f64::from(i) * 0.1 - 10.0,
                    3.0,
                    f64::from(k) * 0.05,
                ));
            }
        }
        let shape = CubeShape::default();
        let found = cubes(&cloud, &shape, &PatchSettings::default());
        let at_kerb = found.obstacles.iter().filter(|p| near_kerb(p)).count();
        assert!(at_kerb >= 20, "{at_kerb} cubes at the kerb");
        // Nothing on the road itself.
        let on_road = found.obstacles.iter().filter(|p| p.y < 2.5).count();
        assert_eq!(
            on_road,
            0,
            "{:?}",
            found.obstacles.iter().find(|p| p.y < 2.5)
        );
        // The same kerb a little lower than the champion's floor: patchwork still sees it.
        let low: Vec<Point> = cloud
            .iter()
            .map(|p| Point::new(p.x, p.y, p.z * (0.25 / 0.3)))
            .collect();
        let ours = cubes(&low, &shape, &PatchSettings::default());
        let theirs = champion(&low, &shape);
        let ours_at = ours.obstacles.iter().filter(|p| near_kerb(p)).count();
        let theirs_at = theirs.obstacles.iter().filter(|p| near_kerb(p)).count();
        assert!(ours_at >= 20 && theirs_at == 0, "{ours_at} vs {theirs_at}");
    }

    #[test]
    fn a_road_bending_up_a_hill_is_no_obstacle() {
        // Flat, then curving up ahead (0.004 x² past 5 m: 0.9 m up at 20 m, 7 degrees).
        let hill = |x: f64, _: f64| {
            if x > 5.0 {
                0.004 * (x - 5.0).powi(2)
            } else {
                0.0
            }
        };
        let cloud = ground(0.1, hill);
        let shape = CubeShape::default();
        let ours = cubes(&cloud, &shape, &PatchSettings::default());
        let theirs = champion(&cloud, &shape);
        assert_eq!(ours.obstacles.len(), 0, "{:?}", ours.obstacles.first());
        assert!(
            theirs.obstacles.len() > 50,
            "one plane: {} false cubes",
            theirs.obstacles.len()
        );
    }

    #[test]
    fn patches_cover_the_zones_and_the_names_parse() {
        let s = PatchSettings::default();
        assert_eq!(patch_of(&s, &Point::new(1.0, 0.0, 0.0)), None);
        assert_eq!(patch_of(&s, &Point::new(2.0, 0.0, 0.0)), Some((0, 0, 0)));
        assert_eq!(
            patch_of(&s, &Point::new(0.0, 7.0, 0.0)).map(|k| k.1),
            Some(2)
        );
        assert_eq!(
            patch_of(&s, &Point::new(29.0, 0.0, 0.0)).map(|k| k.1),
            Some(11)
        );
        assert_eq!(patch_of(&s, &Point::new(31.0, 0.0, 0.0)), None);
        assert_eq!("patchwork".parse(), Ok(Ground::Patchwork));
        assert_eq!("".parse(), Ok(Ground::Plane));
        assert!("ransac".parse::<Ground>().is_err());
    }
}
