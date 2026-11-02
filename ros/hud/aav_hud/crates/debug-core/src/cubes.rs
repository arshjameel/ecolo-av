//! One scan's cubes: the lidar's points, placed on the ground (odom) and snapped to a
//! grid of cubes (20 cm by default), coloured by height.
//!
//! Each point goes lidar_link -> base_link (the static transform) -> odom (the local
//! EKF's pose at the cloud's stamp, [`crate::poses`]). A cube is there if any point fell
//! in it: one `HashSet` of cube indices per scan. Heights are above base_link: the EKF
//! is 2D (no roll or pitch), so a pitched car tilts the cubes. The VLP-16's 0.1 s sweep
//! is ignored (Gazebo's lidar has none).

use std::collections::HashSet;

use map_core::geom::{Point, Pose};

use crate::frames::{Stamp, Transform3, Vec3};

/// A length, in metres.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Metres(pub f64);

/// What the grid needs from the settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CubeSettings {
    /// One cube's edge.
    pub size: Metres,
    /// More cubes than this: the nearest to the car are kept.
    pub max_cubes: usize,
    /// Cubes lower than this above base_link are ground.
    pub ground_height: Metres,
    /// The car itself and its sensors, from above in base_link: points inside are the
    /// lidar seeing its own car, and make no cube. `None` keeps them.
    pub own: Option<OwnBox>,
}

/// A box from above in base_link (m): the car and the sensors on it, with room for the
/// lidar's noise. aav_description's `vehicle.self_box` works it out; the launch file
/// passes it (`--own-box`), so the rule lives in one place. Without it, Gazebo's lidar
/// put the roof's corners and the front cameras in every scan, and the kept cubes left a
/// trail behind the car (30 Oct).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OwnBox {
    pub min: [f64; 2],
    pub max: [f64; 2],
}

impl OwnBox {
    /// Is this base_link point inside, seen from above?
    pub fn contains(&self, p: Vec3) -> bool {
        (self.min[0]..=self.max[0]).contains(&p[0]) && (self.min[1]..=self.max[1]).contains(&p[1])
    }

    /// Reads `x_min,y_min,x_max,y_max` (m), as `--own-box` takes it.
    pub fn parse(text: &str) -> Option<Self> {
        let v: Vec<f64> = text
            .split(',')
            .map(|t| t.trim().parse().ok())
            .collect::<Option<_>>()?;
        match v[..] {
            [x0, y0, x1, y1] if v.iter().all(|n| n.is_finite()) && x0 < x1 && y0 < y1 => {
                Some(Self {
                    min: [x0, y0],
                    max: [x1, y1],
                })
            }
            _ => None,
        }
    }
}

/// How high a cube is above base_link: its colour on the screen.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Band {
    /// Below `ground_height`: the ground, drawn darker.
    Ground,
    /// Up to 0.75 m: kerbs, cones, a box.
    Knee,
    /// Up to 1.25 m: car bodies, walls.
    Waist,
    /// Up to 1.75 m: people.
    Head,
    /// Up to 2.5 m.
    High,
    /// Above 2.5 m: trees, roofs, signs.
    Above,
}

impl Band {
    /// Every band, lowest first: the legend's order.
    pub const ALL: [Band; 6] = [
        Band::Ground,
        Band::Knee,
        Band::Waist,
        Band::Head,
        Band::High,
        Band::Above,
    ];

    /// The band of a cube whose centre is `height` above base_link.
    pub fn of(height: Metres, ground: Metres) -> Self {
        let h = height.0;
        if h < ground.0 {
            Self::Ground
        } else if h < 0.75 {
            Self::Knee
        } else if h < 1.25 {
            Self::Waist
        } else if h < 1.75 {
            Self::Head
        } else if h < 2.5 {
            Self::High
        } else {
            Self::Above
        }
    }

    /// The legend's words.
    pub fn label(self) -> &'static str {
        match self {
            Self::Ground => "ground",
            Self::Knee => "< 0.75 m",
            Self::Waist => "< 1.25 m",
            Self::Head => "< 1.75 m",
            Self::High => "< 2.5 m",
            Self::Above => "higher",
        }
    }
}

/// One cube: its centre in odom (metres) and its band.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cube {
    pub centre: [f32; 3],
    pub band: Band,
}

/// One scan's cubes, ready to draw.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CubeScan {
    /// The cloud's stamp.
    pub stamp: Stamp,
    /// Where base_link was at that stamp, in odom.
    pub car: Pose,
    pub cubes: Vec<Cube>,
    /// The edge of every cube, metres.
    pub size: f32,
    /// Points that were numbers (Gazebo sends inf where a ray hit nothing).
    pub points: usize,
    /// Points skipped: not finite.
    pub skipped: usize,
    /// Cubes left out over `max_cubes` (the farthest).
    pub cut: usize,
}

/// The cubes of one cloud.
///
/// `points` are in the cloud's frame; `cloud_to_base` takes them to base_link;
/// `car` is base_link's pose in odom at the cloud's stamp.
pub fn build(
    points: impl IntoIterator<Item = [f32; 3]>,
    cloud_to_base: &Transform3,
    car: Pose,
    stamp: Stamp,
    settings: &CubeSettings,
) -> CubeScan {
    let size = settings.size.0;
    let to_odom = Transform3::from_planar(car).then_after(cloud_to_base);
    let mut cells: HashSet<[i32; 3]> = HashSet::new();
    let (mut used, mut skipped) = (0, 0);
    for p in points {
        if !p.iter().all(|v| v.is_finite()) {
            skipped += 1;
            continue;
        }
        used += 1;
        let p = p.map(f64::from);
        if settings
            .own
            .is_some_and(|own| own.contains(cloud_to_base.apply(p)))
        {
            continue;
        }
        let q = to_odom.apply(p);
        cells.insert(cell_of(q, size));
    }
    let mut cubes: Vec<Cube> = cells
        .into_iter()
        .map(|c| {
            let centre: Vec3 = c.map(|i| (f64::from(i) + 0.5) * size);
            Cube {
                centre: centre.map(|v| v as f32),
                band: Band::of(Metres(centre[2]), settings.ground_height),
            }
        })
        .collect();
    let cut = keep_nearest(&mut cubes, car, settings.max_cubes);
    CubeScan {
        stamp,
        car,
        cubes,
        size: size as f32,
        points: used,
        skipped,
        cut,
    }
}

/// The cube index along one axis. Saturates far out (a point 400,000 km away).
pub(crate) fn cell(v: f64, size: f64) -> i32 {
    let i = (v / size).floor();
    if i >= f64::from(i32::MAX) {
        i32::MAX
    } else if i <= f64::from(i32::MIN) {
        i32::MIN
    } else {
        i as i32
    }
}

/// Keeps the `max` cubes nearest the car (seen from above); returns how many went.
fn keep_nearest(cubes: &mut Vec<Cube>, car: Pose, max: usize) -> usize {
    keep_nearest_by(cubes, |c| c.centre, car.position, max)
}

/// Keeps the `max` items whose `centre` is nearest `to` (seen from above); returns how
/// many went. The order of those kept is not kept.
pub(crate) fn keep_nearest_by<T>(
    items: &mut Vec<T>,
    centre: impl Fn(&T) -> [f32; 3],
    to: Point,
    max: usize,
) -> usize {
    if items.len() <= max {
        return 0;
    }
    let (x, y) = (to.x as f32, to.y as f32);
    let far = |c: &T| {
        let [cx, cy, _] = centre(c);
        let (dx, dy) = (cx - x, cy - y);
        dx * dx + dy * dy
    };
    if max > 0 {
        items.select_nth_unstable_by(max - 1, |a, b| far(a).total_cmp(&far(b)));
    }
    let cut = items.len() - max;
    items.truncate(max);
    cut
}

/// The cube index along one axis of a point (or a cube's centre), `size` apart.
pub(crate) fn cell_of(p: [f64; 3], size: f64) -> [i32; 3] {
    p.map(|v| cell(v, size))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn settings(max: usize) -> CubeSettings {
        CubeSettings {
            size: Metres(0.2),
            max_cubes: max,
            ground_height: Metres(0.25),
            own: None,
        }
    }

    #[test]
    fn the_cars_own_roof_and_cameras_make_no_cube() {
        // From the roof lidar (0.5 m ahead of base_link, 1.8 up): a roof corner and a
        // camera (the 30 Oct ghost), and a wall 3 m ahead.
        let own = OwnBox::parse("-0.39, -0.65, 1.99, 0.65").expect("four numbers");
        let points = [[-0.71, 0.49, -0.33], [1.35, -0.27, -0.26], [2.5, 0.0, -1.0]];
        let mut with = settings(100);
        with.own = Some(own);
        let kept = build(points, &lidar_on_roof(), Pose::default(), Stamp(0), &with);
        assert_eq!(kept.cubes.len(), 1);
        assert!((kept.cubes[0].centre[0] - 3.1).abs() < 1e-5);
        let all = build(
            points,
            &lidar_on_roof(),
            Pose::default(),
            Stamp(0),
            &settings(100),
        );
        assert_eq!(all.cubes.len(), 3);
    }

    #[test]
    fn an_own_box_needs_four_numbers_low_then_high() {
        assert!(OwnBox::parse("1,2,3").is_none());
        assert!(OwnBox::parse("1,2,0,3").is_none());
        assert!(OwnBox::parse("a,b,c,d").is_none());
        assert_eq!(
            OwnBox::parse("-1,-2,3,4"),
            Some(OwnBox {
                min: [-1.0, -2.0],
                max: [3.0, 4.0]
            })
        );
    }

    fn lidar_on_roof() -> Transform3 {
        Transform3 {
            translation: [0.5, 0.0, 1.8],
            ..Transform3::IDENTITY
        }
    }

    #[test]
    fn points_in_one_cube_make_one_cube() {
        let scan = build(
            [[0.01, 0.01, 0.01], [0.05, 0.1, 0.15], [0.19, 0.19, 0.19]],
            &Transform3::IDENTITY,
            Pose::default(),
            Stamp(7),
            &settings(100),
        );
        assert_eq!(scan.cubes.len(), 1);
        assert_eq!(scan.points, 3);
        let c = scan.cubes[0];
        assert!((c.centre[0] - 0.1).abs() < 1e-6 && (c.centre[2] - 0.1).abs() < 1e-6);
        assert_eq!(c.band, Band::Ground);
        assert_eq!(scan.stamp, Stamp(7));
    }

    #[test]
    fn a_wall_ahead_of_a_car_facing_north_is_north() {
        // The car at (10, 0), facing north. The lidar (0.5 m ahead of base_link, 1.8 m
        // up) sees a point 5 m ahead of itself and 0.7 m below.
        let car = Pose::new(10.0, 0.0, FRAC_PI_2);
        let scan = build(
            [[5.0, 0.0, -0.7]],
            &lidar_on_roof(),
            car,
            Stamp(0),
            &settings(100),
        );
        let c = scan.cubes[0].centre;
        // 5.5 m ahead of base_link: map (10, 5.5), 1.1 m up.
        assert!((c[0] - 10.1).abs() < 0.11, "{c:?}");
        assert!((c[1] - 5.5).abs() < 0.11, "{c:?}");
        assert!((c[2] - 1.1).abs() < 0.11, "{c:?}");
        assert_eq!(scan.cubes[0].band, Band::Waist);
    }

    #[test]
    fn rays_that_hit_nothing_are_skipped() {
        let scan = build(
            [
                [f32::INFINITY, 0.0, 0.0],
                [f32::NAN, 1.0, 1.0],
                [1.0, 1.0, 1.0],
            ],
            &Transform3::IDENTITY,
            Pose::default(),
            Stamp(0),
            &settings(100),
        );
        assert_eq!((scan.points, scan.skipped, scan.cubes.len()), (1, 2, 1));
    }

    #[test]
    fn over_the_cap_the_nearest_stay() {
        let points: Vec<[f32; 3]> = (0..10).map(|i| [i as f32, 0.0, 1.0]).collect();
        let scan = build(
            points,
            &Transform3::IDENTITY,
            Pose::default(),
            Stamp(0),
            &settings(3),
        );
        assert_eq!((scan.cubes.len(), scan.cut), (3, 7));
        assert!(scan.cubes.iter().all(|c| c.centre[0] < 3.0));
    }

    #[test]
    fn bands_go_up() {
        let g = Metres(0.25);
        let heights = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0];
        let bands: Vec<Band> = heights.iter().map(|&h| Band::of(Metres(h), g)).collect();
        assert_eq!(bands, Band::ALL.to_vec());
        assert_eq!(Band::of(Metres(-1.0), g), Band::Ground);
    }

    #[test]
    fn negative_coordinates_snap_down() {
        assert_eq!(cell(-0.01, 0.2), -1);
        assert_eq!(cell(0.19, 0.2), 0);
        assert_eq!(cell(1e30, 0.2), i32::MAX);
    }
}
