//! Made-up worlds for the tests: boxes on flat ground, and a lidar like Gazebo's (16
//! beams from -15° to 15°, 1800 steps a turn, 3 cm of range noise) that sees them.

#![allow(dead_code)]

use ndt_core::{Mount, Point, Pose2};

/// A small random number generator (xorshift64*): the tests need no crate for it.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }

    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Normal, mean 0, standard deviation `sd` (Box-Muller).
    pub fn normal(&mut self, sd: f64) -> f64 {
        let u = self.uniform().max(1e-300);
        let v = self.uniform();
        sd * (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// A box standing on the ground, sides along x and y (as every one in the test lot).
#[derive(Clone, Copy, Debug)]
pub struct Block {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl Block {
    /// From Gazebo's way: the centre (x, y) and the size; it stands on the ground.
    pub fn at(x: f64, y: f64, size: [f64; 3]) -> Self {
        Self {
            min: [x - size[0] / 2.0, y - size[1] / 2.0, 0.0],
            max: [x + size[0] / 2.0, y + size[1] / 2.0, size[2]],
        }
    }

    /// How far along the ray from `o` in direction `d` it first hits the box (slabs).
    fn hit(&self, o: &[f64; 3], d: &[f64; 3]) -> Option<f64> {
        let mut near = f64::NEG_INFINITY;
        let mut far = f64::INFINITY;
        for i in 0..3 {
            if d[i].abs() < 1e-12 {
                if o[i] < self.min[i] || o[i] > self.max[i] {
                    return None;
                }
                continue;
            }
            let (a, b) = ((self.min[i] - o[i]) / d[i], (self.max[i] - o[i]) / d[i]);
            near = near.max(a.min(b));
            far = far.min(a.max(b));
        }
        (near <= far && near > 0.0).then_some(near)
    }
}

/// Boxes on flat ground at z = 0.
pub struct World {
    pub blocks: Vec<Block>,
}

impl World {
    /// Gazebo's test lot (aav_gazebo worlds/test_lot.sdf): the walls, the building and the
    /// parked cars, and a few posts and parked cars more, so it isn't all long walls.
    pub fn test_lot() -> Self {
        let mut blocks = vec![
            Block::at(30.0, -9.5, [80.0, 1.0, 1.5]),  // wall_south
            Block::at(30.0, 49.5, [80.0, 1.0, 1.5]),  // wall_north
            Block::at(69.5, 20.0, [1.0, 60.0, 1.5]),  // wall_east
            Block::at(-9.5, -2.5, [1.0, 15.0, 1.5]),  // wall_west_a
            Block::at(-9.5, 29.5, [1.0, 41.0, 1.5]),  // wall_west_b
            Block::at(30.0, 27.5, [20.0, 15.0, 4.0]), // building
            Block::at(51.0, 15.0, [2.0, 30.0, 1.5]),  // parked_cars
        ];
        blocks.extend(
            [
                (5.0, 8.0),
                (12.0, -4.0),
                (20.0, 10.0),
                (40.0, 5.0),
                (60.0, 38.0),
            ]
            .map(|(x, y)| Block::at(x, y, [0.2, 0.2, 1.2])),
        );
        blocks.extend(
            [(-2.0, 20.0), (15.0, 40.0), (45.0, 40.0)]
                .map(|(x, y)| Block::at(x, y, [4.5, 1.8, 1.5])),
        );
        Self { blocks }
    }

    /// A small yard: three walls, two parked cars, two posts; nothing repeats.
    pub fn yard() -> Self {
        Self {
            blocks: vec![
                Block::at(0.0, 15.0, [40.0, 0.5, 2.0]),
                Block::at(-20.0, 0.0, [0.5, 30.0, 2.0]),
                Block::at(10.0, -12.0, [25.0, 0.5, 1.0]),
                Block::at(8.0, 5.0, [4.5, 1.8, 1.5]),
                Block::at(-6.0, -4.0, [1.8, 4.5, 1.5]),
                Block::at(3.0, -6.0, [0.3, 0.3, 1.2]),
                Block::at(-10.0, 8.0, [0.3, 0.3, 1.2]),
            ],
        }
    }

    /// The lidar on the car's roof: where it sits on `base_link`.
    pub fn mount() -> ([f64; 3], Mount) {
        let xyz = [0.9, 0.0, 1.8];
        (xyz, Mount::from_xyz_rpy(xyz, [0.0, 0.0, 0.0]))
    }

    /// One turn of the lidar with the car at `car` (map frame): points in `lidar_link`,
    /// inf for a ray that hits nothing within 100 m.
    pub fn scan(&self, car: &Pose2, steps: usize, noise: f64, rng: &mut Rng) -> Vec<Point> {
        let ([mx, my, mz], _) = Self::mount();
        let lidar = car.apply(&Point::new(mx, my, mz));
        let o = [lidar.x, lidar.y, lidar.z];
        let mut out = Vec::with_capacity(steps * 16);
        for beam in 0..16 {
            let elevation = (-15.0 + 2.0 * beam as f64).to_radians();
            for step in 0..steps {
                let azimuth =
                    -std::f64::consts::PI + std::f64::consts::TAU * step as f64 / steps as f64;
                // In lidar_link, then turned into the map (the lidar faces base_link's way).
                let local = [
                    elevation.cos() * azimuth.cos(),
                    elevation.cos() * azimuth.sin(),
                    elevation.sin(),
                ];
                let (s, c) = car.yaw.sin_cos();
                let d = [
                    c * local[0] - s * local[1],
                    s * local[0] + c * local[1],
                    local[2],
                ];
                let ground = (d[2] < 0.0).then(|| -o[2] / d[2]);
                let range = self
                    .blocks
                    .iter()
                    .filter_map(|b| b.hit(&o, &d))
                    .chain(ground)
                    .fold(f64::INFINITY, f64::min);
                if !(0.5..=100.0).contains(&range) {
                    out.push(Point::new(f64::INFINITY, f64::INFINITY, f64::INFINITY));
                    continue;
                }
                let r = range + rng.normal(noise);
                out.push(Point::new(r * local[0], r * local[1], r * local[2]));
            }
        }
        out
    }
}

/// Scans along `route`, put into the map frame with the true poses, ground out above
/// 0.3 m, thinned to 10 cm: what `tools/ndt_map.py` does with a bag.
pub fn map_from_route(world: &World, route: &[Pose2], rng: &mut Rng) -> Vec<Point> {
    let (_, mount) = World::mount();
    let points: Vec<Point> = route
        .iter()
        .flat_map(|car| {
            world
                .scan(car, 1800, 0.03, rng)
                .into_iter()
                .filter(|p| p.iter().all(|v| v.is_finite()))
                .map(|p| car.apply(&mount.to_base_link(&p)))
                .filter(|p| p.z > 0.3)
                .collect::<Vec<_>>()
        })
        .collect();
    ndt_core::cloud::thin(&points, 0.1)
}

/// Poses every `step` m along straight lines between the corners.
pub fn route(corners: &[(f64, f64)], step: f64) -> Vec<Pose2> {
    corners
        .windows(2)
        .flat_map(|w| {
            let ((x0, y0), (x1, y1)) = (w[0], w[1]);
            let length = (x1 - x0).hypot(y1 - y0);
            let yaw = (y1 - y0).atan2(x1 - x0);
            let n = (length / step).ceil() as usize;
            (0..n).map(move |i| {
                let t = i as f64 / n as f64;
                Pose2::new(x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw)
            })
        })
        .collect()
}
