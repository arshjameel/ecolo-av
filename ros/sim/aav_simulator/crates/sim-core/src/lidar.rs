//! Lidars by ray casting: the roof lidar and the radar stand-in, as Gazebo's `gpu_lidar`.
//!
//! One ray per (ring, step), ring by ring from the lowest beam, each ring from the
//! smallest angle; Gaussian noise on each hit's range; `+inf` past the far limit and `-inf`
//! nearer than the near one, as Gazebo's. The cloud is packed as Gazebo packs it
//! (gz-sensors' GpuLidarSensor, memory aligned): x, y, z (float32) at 0, 4, 8, intensity
//! (float32) at 16, ring (uint16) at 24, 32 bytes a point, `height` rings of `width`
//! points: an organised cloud, so `ros_gz_bridge`'s readers read ours the same. Ours adds
//! `speed` (float32) in Gazebo's padding at 20: how fast what the ray hit moves along the
//! ray, over the ground (m/s, + away; 0 for the still world), so the radar stand-in can
//! give a person walking their own Doppler speed (TODO 194). Gazebo's cloud has no such
//! field: there everything stands still.

use crate::math::{Pose, Vec3};
use crate::noise::Noise;
use crate::scene::{Ray, Scene};

/// A placed model's velocity over the ground (m/s, world frame) by its name; `None`: still.
pub type Motion<'a> = &'a (dyn Fn(&str) -> Option<Vec3> + Sync);

/// A lidar's scan pattern and range, as the URDF's `<lidar>` block says.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LidarSpec {
    /// Steps round (`<horizontal><samples>`), from `h_min` to `h_max` inclusive.
    pub h_samples: usize,
    pub h_min: f64,
    pub h_max: f64,
    /// Rings (`<vertical><samples>`), from `v_min` to `v_max` inclusive.
    pub v_samples: usize,
    pub v_min: f64,
    pub v_max: f64,
    /// m.
    pub range_min: f64,
    pub range_max: f64,
    /// m: the range noise's standard deviation.
    pub noise: f64,
    /// Hz.
    pub rate: f64,
}

impl LidarSpec {
    /// The simulated roof lidar (aav.sensors.gazebo.xacro): 1800 x 16, ±15°, 0.5-100 m,
    /// 3 cm, 10 Hz, like a VLP-16.
    pub const ROOF: LidarSpec = LidarSpec {
        h_samples: 1800,
        h_min: -std::f64::consts::PI,
        h_max: std::f64::consts::PI,
        v_samples: 16,
        v_min: -15.0 * std::f64::consts::PI / 180.0,
        v_max: 15.0 * std::f64::consts::PI / 180.0,
        range_min: 0.5,
        range_max: 100.0,
        noise: 0.03,
        rate: 10.0,
    };

    /// The radar stand-in: the TI IWR6843AOP's view, 61 x 3, ±60°, -1° to +7°, 0.3-8.4 m,
    /// 5 cm, 10 Hz.
    pub const RADAR: LidarSpec = LidarSpec {
        h_samples: 61,
        h_min: -60.0 * std::f64::consts::PI / 180.0,
        h_max: 60.0 * std::f64::consts::PI / 180.0,
        v_samples: 3,
        v_min: -std::f64::consts::PI / 180.0,
        v_max: 7.0 * std::f64::consts::PI / 180.0,
        range_min: 0.3,
        range_max: 8.4,
        noise: 0.05,
        rate: 10.0,
    };

    fn step(min: f64, max: f64, n: usize) -> f64 {
        if n > 1 {
            (max - min) / (n - 1) as f64
        } else {
            0.0
        }
    }

    /// Each ring's elevation and each step's azimuth, in Gazebo's float32 sums.
    fn angles(&self) -> (Vec<f32>, Vec<f32>) {
        let rings = (0..self.v_samples)
            .scan(self.v_min as f32, |a, _| {
                let now = *a;
                *a += Self::step(self.v_min, self.v_max, self.v_samples) as f32;
                Some(now)
            })
            .collect();
        let steps = (0..self.h_samples)
            .scan(self.h_min as f32, |a, _| {
                let now = *a;
                *a += Self::step(self.h_min, self.h_max, self.h_samples) as f32;
                Some(now)
            })
            .collect();
        (rings, steps)
    }
}

/// One scan: the range of every ray, ring by ring.
#[derive(Debug, Clone, PartialEq)]
pub struct Scan {
    pub spec: LidarSpec,
    /// `v_samples` x `h_samples` ranges (m); `inf` for nothing, `-inf` for too near.
    pub ranges: Vec<f32>,
    /// m/s, one per ray: how fast what it hit moves along it over the ground (+ away).
    pub speeds: Vec<f32>,
}

impl Scan {
    /// Cast every ray of `spec` from `sensor` (the lidar's pose in the world).
    ///
    /// The rings are shared between `threads` workers: the roof lidar's 28,800 rays take a
    /// few milliseconds this way.
    pub fn cast(
        spec: &LidarSpec,
        scene: &Scene,
        sensor: &Pose,
        noise: &mut Noise,
        threads: usize,
    ) -> Scan {
        Self::cast_moving(spec, scene, sensor, noise, threads, &|_| None)
    }

    /// As [`Scan::cast`], with each placed model's `motion`: a ray that hits a moving one
    /// gets its speed along the ray.
    pub fn cast_moving(
        spec: &LidarSpec,
        scene: &Scene,
        sensor: &Pose,
        noise: &mut Noise,
        threads: usize,
        motion: Motion,
    ) -> Scan {
        let (rings, steps) = spec.angles();
        let mut ranges = vec![f32::INFINITY; rings.len() * steps.len()];
        let mut speeds = vec![0.0f32; ranges.len()];
        let width = steps.len().max(1);
        let chunk = rings.len().div_ceil(threads.max(1)).max(1);
        // One noise stream per ring, forked in order: a seed gives the same scan on any
        // number of threads (one stream per worker made the noise depend on `threads`).
        let mut streams: Vec<Noise> = rings.iter().map(|_| noise.fork()).collect();
        std::thread::scope(|s| {
            for (((rows, speed_rows), ring_angles), ring_noise) in ranges
                .chunks_mut(width * chunk)
                .zip(speeds.chunks_mut(width * chunk))
                .zip(rings.chunks(chunk))
                .zip(streams.chunks_mut(chunk))
            {
                let steps = &steps;
                s.spawn(move || {
                    for (((row, speed_row), &elevation), n) in rows
                        .chunks_mut(width)
                        .zip(speed_rows.chunks_mut(width))
                        .zip(ring_angles)
                        .zip(ring_noise.iter_mut())
                    {
                        let (se, ce) = (elevation as f64).sin_cos();
                        for ((cell, speed), &azimuth) in
                            row.iter_mut().zip(speed_row.iter_mut()).zip(steps)
                        {
                            let (sa, ca) = (azimuth as f64).sin_cos();
                            let local = Vec3::new(ce * ca, ce * sa, se);
                            let ray = Ray {
                                origin: sensor.position,
                                direction: sensor.rotation.apply(local),
                            };
                            let hit = scene.cast_named(&ray, spec.range_max);
                            *cell = match hit {
                                None => f32::INFINITY,
                                Some((d, _)) if d < spec.range_min => f32::NEG_INFINITY,
                                Some((d, _)) => (d + n.gauss(spec.noise))
                                    .clamp(spec.range_min, spec.range_max)
                                    as f32,
                            };
                            if let Some(v) = hit.and_then(|(_, name)| name).and_then(motion) {
                                *speed = ray.direction.dot(v) as f32;
                            }
                        }
                    }
                });
            }
        });
        Scan {
            spec: *spec,
            ranges,
            speeds,
        }
    }

    /// The points of the rays that hit something, in the sensor's frame.
    pub fn hits(&self) -> impl Iterator<Item = (Vec3, u16)> + '_ {
        let (rings, steps) = self.spec.angles();
        let width = steps.len();
        self.ranges
            .iter()
            .enumerate()
            .filter(|(_, r)| r.is_finite())
            .map(move |(i, &r)| {
                let (ring, step) = (i / width, i % width);
                let (e, a) = (rings[ring], steps[step]);
                let p = Vec3::new(
                    (r * e.cos() * a.cos()) as f64,
                    (r * e.cos() * a.sin()) as f64,
                    (r * e.sin()) as f64,
                );
                (p, ring as u16)
            })
    }

    /// The cloud's bytes, packed as Gazebo packs them (see the module's notes).
    pub fn pack(&self) -> Vec<u8> {
        let (rings, steps) = self.spec.angles();
        let mut data = Vec::with_capacity(self.ranges.len() * POINT_STEP);
        for (ring, &e) in rings.iter().enumerate() {
            for (step, &a) in steps.iter().enumerate() {
                let r = self.ranges[ring * steps.len() + step];
                let mut point = [0u8; POINT_STEP];
                let xyz = [r * e.cos() * a.cos(), r * e.cos() * a.sin(), r * e.sin()];
                for (k, v) in xyz.iter().enumerate() {
                    point[4 * k..4 * k + 4].copy_from_slice(&v.to_le_bytes());
                }
                point[16..20].copy_from_slice(&0.0f32.to_le_bytes());
                let speed = self.speeds[ring * steps.len() + step];
                point[20..24].copy_from_slice(&speed.to_le_bytes());
                point[24..26].copy_from_slice(&(ring as u16).to_le_bytes());
                data.extend_from_slice(&point);
            }
        }
        data
    }

    /// Whether every ray hit something within range (PointCloud2's `is_dense`).
    pub fn dense(&self) -> bool {
        self.ranges.iter().all(|r| r.is_finite())
    }
}

/// Bytes per point in the packed cloud.
pub const POINT_STEP: usize = 32;

/// A packed cloud's fields: name, byte offset, PointField datatype (7 float32, 4 uint16).
pub const FIELDS: [(&str, u32, u8); 6] = [
    ("x", 0, 7),
    ("y", 4, 7),
    ("z", 8, 7),
    ("intensity", 16, 7),
    ("speed", 20, 7),
    ("ring", 24, 4),
];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Rot3;
    use crate::scene::Mesh;

    #[test]
    fn a_level_ray_sees_a_wall_ten_metres_ahead() {
        let mut scene = Scene::new(Vec::new());
        scene.place(
            "wall",
            &Mesh::cuboid(Vec3::new(1.0, 20.0, 5.0)),
            &Pose::new(Vec3::new(10.5, 0.0, 0.0), Rot3::IDENTITY),
        );
        let spec = LidarSpec {
            h_samples: 3,
            h_min: -0.1,
            h_max: 0.1,
            v_samples: 1,
            v_min: 0.0,
            v_max: 0.0,
            noise: 0.0,
            ..LidarSpec::ROOF
        };
        let scan = Scan::cast(&spec, &scene, &Pose::IDENTITY, &mut Noise::new(1), 2);
        assert!((scan.ranges[1] - 10.0).abs() < 1e-4, "{:?}", scan.ranges);
        assert!(scan.ranges[0] > 10.0 && scan.ranges[0] < 10.1);
    }

    #[test]
    fn a_ray_on_a_moving_model_gets_its_speed_along_the_ray() {
        // TODO 194: a person walking across, 4 m ahead and 3 m right, at 0.8 m/s to the
        // left: the ray to them closes at 0.8 x 3/5 = 0.48 m/s. The wall stands still.
        let mut scene = Scene::new(Vec::new());
        let at = Vec3::new(4.0, -3.0, 0.0);
        scene.place(
            "walker",
            &Mesh::cuboid(Vec3::new(0.2, 0.2, 0.2)),
            &Pose::new(at, Rot3::IDENTITY),
        );
        scene.place(
            "wall",
            &Mesh::cuboid(Vec3::new(1.0, 20.0, 5.0)),
            &Pose::new(Vec3::new(10.5, 0.0, 0.0), Rot3::IDENTITY),
        );
        let towards = (-3.0f64).atan2(4.0);
        let spec = LidarSpec {
            h_samples: 2,
            h_min: towards,
            h_max: 0.0,
            v_samples: 1,
            v_min: 0.0,
            v_max: 0.0,
            noise: 0.0,
            ..LidarSpec::ROOF
        };
        let walking = |name: &str| (name == "walker").then_some(Vec3::new(0.0, 0.8, 0.0));
        let scan = Scan::cast_moving(
            &spec,
            &scene,
            &Pose::IDENTITY,
            &mut Noise::new(1),
            1,
            &walking,
        );
        assert!((scan.ranges[0] - 4.9).abs() < 0.05, "{:?}", scan.ranges);
        assert!((scan.speeds[0] + 0.48).abs() < 1e-4, "{:?}", scan.speeds);
        assert_eq!(scan.speeds[1], 0.0);
        // Packed at byte 20, as FIELDS says.
        let packed = scan.pack();
        let speed = f32::from_le_bytes(packed[20..24].try_into().expect("4 bytes"));
        assert!((speed + 0.48).abs() < 1e-4);
        assert!(FIELDS.contains(&("speed", 20, 7)));
    }

    #[test]
    fn nothing_there_is_inf_and_too_near_is_minus_inf() {
        let mut scene = Scene::new(Vec::new());
        scene.place(
            "box",
            &Mesh::cuboid(Vec3::new(0.2, 0.2, 0.2)),
            &Pose::new(Vec3::new(0.3, 0.0, 0.0), Rot3::IDENTITY),
        );
        let spec = LidarSpec {
            h_samples: 2,
            h_min: 0.0,
            h_max: std::f64::consts::PI,
            v_samples: 1,
            v_min: 0.0,
            v_max: 0.0,
            ..LidarSpec::ROOF
        };
        let scan = Scan::cast(&spec, &scene, &Pose::IDENTITY, &mut Noise::new(1), 1);
        assert_eq!(scan.ranges, vec![f32::NEG_INFINITY, f32::INFINITY]);
        assert!(!scan.dense());
    }

    #[test]
    fn the_roof_lidar_sees_flat_ground_where_its_lower_beams_land() {
        let scene = Scene::new(Mesh::plane(400.0, 400.0).triangles);
        let lidar = Pose::new(Vec3::new(0.0, 0.0, 1.52), Rot3::IDENTITY);
        let spec = LidarSpec {
            noise: 0.0,
            ..LidarSpec::ROOF
        };
        let scan = Scan::cast(&spec, &scene, &lidar, &mut Noise::new(1), 4);
        assert_eq!(scan.ranges.len(), 1800 * 16);
        // The lowest ring, 15° down: 1.52 / sin 15° = 5.87 m.
        let lowest = scan.ranges[900];
        assert!((lowest - 5.873).abs() < 0.01, "{lowest}");
        // The level and upward rings see nothing.
        assert!(scan.ranges[15 * 1800..].iter().all(|r| r.is_infinite()));
        assert_eq!(scan.pack().len(), 1800 * 16 * POINT_STEP);
    }

    #[test]
    fn a_seed_repeats_the_scan() {
        let scene = Scene::new(Mesh::plane(400.0, 400.0).triangles);
        let lidar = Pose::new(Vec3::new(0.0, 0.0, 1.52), Rot3::IDENTITY);
        let a = Scan::cast(&LidarSpec::ROOF, &scene, &lidar, &mut Noise::new(9), 4);
        let b = Scan::cast(&LidarSpec::ROOF, &scene, &lidar, &mut Noise::new(9), 4);
        assert_eq!(a, b);
    }

    #[test]
    fn the_noise_does_not_depend_on_the_threads() {
        let scene = Scene::new(Mesh::plane(400.0, 400.0).triangles);
        let lidar = Pose::new(Vec3::new(0.0, 0.0, 1.52), Rot3::IDENTITY);
        let one = Scan::cast(&LidarSpec::ROOF, &scene, &lidar, &mut Noise::new(9), 1);
        for threads in [2, 3, 4, 7, 16] {
            let many = Scan::cast(
                &LidarSpec::ROOF,
                &scene,
                &lidar,
                &mut Noise::new(9),
                threads,
            );
            assert_eq!(one, many, "{threads} threads");
        }
        // And the noise is there: the lowest ring isn't the noiseless 5.873 m everywhere.
        assert!(one.ranges[..1800].windows(2).any(|w| w[0] != w[1]));
    }
}
