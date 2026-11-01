//! Pretend data for `--demo`: a walled lot with boxes, a car driving a loop, a 16-beam
//! lidar ray-cast against it, and the localizers a little apart (NDT too, LOST for a
//! moment every 30 s), AMCL's particles, and the lot itself as NDT's map. No ROS.
//!
//! `--bench-cubes N` swaps the lidar for N cubes spread round the car: the drawing test.

use std::{
    f64::consts::{PI, TAU},
    time::{Duration, Instant},
};

use debug_core::{
    cubes::{self, Cube, CubeScan, CubeSettings},
    frames::{Stamp, Transform3},
    ndt::{MapCells, NdtStatus},
    state::{Localizer, Source, Update},
    swap::{Reply, Tracker},
    track::TrackStatus,
};
use hud_core::{
    units::{Fraction, MetresPerSecond, Radians},
    vehicle::{Faults, Mode, VehicleStatus},
};
use map_core::geom::{Point, Pose};

use crate::source::DataSource;

/// The loop the car drives: a circle, metres and m/s.
const CENTRE: Point = Point::new(12.0, 0.0);
const RADIUS: f64 = 9.0;
const SPEED: f64 = 2.5;
/// The lidar on the roof, in base_link.
const LIDAR: [f64; 3] = [0.6, 0.0, 1.8];
/// Where odom sits on the map (map -> odom).
const ODOM_IN_MAP: Pose = Pose::new(3.0, -2.0, 0.08);

/// An axis-aligned box: min and max corners, odom.
#[derive(Clone, Copy)]
struct Aabb {
    min: [f64; 3],
    max: [f64; 3],
}

impl Aabb {
    const fn new(min: [f64; 3], max: [f64; 3]) -> Self {
        Self { min, max }
    }

    /// The distance along the ray to the box, if it hits (slab method).
    fn hit(&self, o: [f64; 3], d: [f64; 3]) -> Option<f64> {
        let (mut near, mut far) = (0.0_f64, f64::INFINITY);
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
            if near > far {
                return None;
            }
        }
        Some(near)
    }
}

/// The lot: four walls, boxes, a post, a parked van.
const WORLD: [Aabb; 10] = [
    Aabb::new([-10.0, -16.0, 0.0], [34.0, -15.6, 2.4]),
    Aabb::new([-10.0, 15.6, 0.0], [34.0, 16.0, 2.4]),
    Aabb::new([-10.4, -16.0, 0.0], [-10.0, 16.0, 2.4]),
    Aabb::new([34.0, -16.0, 0.0], [34.4, 16.0, 2.4]),
    Aabb::new([11.0, -1.0, 0.0], [13.0, 1.0, 1.0]),
    Aabb::new([24.0, 6.0, 0.0], [25.0, 7.0, 0.6]),
    Aabb::new([0.0, 8.0, 0.0], [5.0, 10.0, 2.1]),
    Aabb::new([20.0, -12.0, 0.0], [20.3, -11.7, 3.5]),
    Aabb::new([-4.0, -6.0, 0.0], [-3.0, -5.0, 1.6]),
    Aabb::new([26.0, -4.0, 0.0], [27.0, -3.0, 1.2]),
];

/// The lidar's beams: 16, from -15 to +15 degrees, as a VLP-16.
const BEAMS: usize = 16;
const AZIMUTHS: usize = 900;
const RANGE: f64 = 60.0;

/// One sweep from base_link's pose `car`: the points in the lidar's own frame.
fn sweep(car: Pose) -> Vec<[f32; 3]> {
    let lidar = Transform3::from_planar(car).apply(LIDAR);
    let mut points = Vec::with_capacity(BEAMS * AZIMUTHS);
    for b in 0..BEAMS {
        let elevation = (-15.0 + 2.0 * b as f64).to_radians();
        for a in 0..AZIMUTHS {
            let azimuth = TAU * a as f64 / AZIMUTHS as f64;
            // In the lidar's frame, then turned onto the ground with the car.
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
            let ground = (d[2] < 0.0).then(|| -lidar[2] / d[2]);
            let t = WORLD
                .iter()
                .filter_map(|w| w.hit(lidar, d))
                .chain(ground)
                .filter(|&t| t > 0.3)
                .fold(f64::INFINITY, f64::min);
            if t <= RANGE {
                points.push(local.map(|v| (v * t) as f32));
            } else {
                // As Gazebo's: inf where a ray hit nothing.
                points.push([f32::INFINITY; 3]);
            }
        }
    }
    points
}

/// The lot as NDT's map: points on every box's sides and top, 0.25 m apart, in the
/// map frame, above 0.3 m (as `ndt-map` leaves the ground out).
fn lot_map(size: f64) -> MapCells {
    let step = 0.25;
    let along = |a: f64, b: f64| {
        let n = ((b - a) / step).ceil() as usize;
        (0..=n).map(move |i| (a + i as f64 * step).min(b))
    };
    let points = WORLD.iter().flat_map(|w| {
        let (min, max) = (w.min, w.max);
        along(min[0], max[0]).flat_map(move |x| {
            along(min[1], max[1]).flat_map(move |y| {
                along(min[2].max(0.3), max[2]).filter_map(move |z| {
                    let on_side = [x == min[0] || x == max[0], y == min[1] || y == max[1]];
                    (on_side.iter().any(|&b| b) || z == max[2]).then(|| {
                        let p = ODOM_IN_MAP.transform(Point::new(x, y));
                        [p.x, p.y, z]
                    })
                })
            })
        })
    });
    MapCells::from_points(points, size)
}

/// A number from -1 to 1 that looks random, the same for the same `i`.
fn jitter(i: u64) -> f64 {
    let mut x = i
        .wrapping_mul(0x9E37_79B9_7F4A_7C15)
        .wrapping_add(0x2545_F491_4F6C_DD1D);
    x ^= x >> 29;
    x = x.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    x ^= x >> 32;
    (x % 20_001) as f64 / 10_000.0 - 1.0
}

/// AMCL's particles round its pose: 300, spread about 0.4 m and 4 degrees.
fn particles(amcl: Pose, t: f64) -> Vec<Pose> {
    let tick = (t * 2.0) as u64;
    (0..300u64)
        .map(|i| {
            let k = tick * 1000 + i * 3;
            Pose::new(
                amcl.position.x + 0.4 * jitter(k),
                amcl.position.y + 0.4 * jitter(k + 1),
                amcl.yaw + 0.07 * jitter(k + 2),
            )
        })
        .collect()
}

/// NDT's status at `t`: tracking, but LOST for 2 s every 30 s, so the panel shows both.
fn ndt_status(t: f64) -> NdtStatus {
    let lost = t % 30.0 > 28.0;
    let restarts = (t / 30.0) as u64;
    if lost {
        return NdtStatus::from_message(
            2,
            "LOST: a poor fit (score per point too low)",
            [
                ("score_per_point", "1.31"),
                ("match_ms", "21.40"),
                ("restarts", &restarts.to_string()),
            ],
        );
    }
    let score = format!("{:.3}", 3.3 + 0.2 * (t * 0.9).sin());
    let ms = format!("{:.2}", 9.0 + 2.0 * (t * 1.7).cos());
    NdtStatus::from_message(
        0,
        "TRACKING",
        [
            ("score_per_point", score.as_str()),
            ("match_ms", ms.as_str()),
            ("overlap", "0.880"),
            ("iterations", "6"),
            ("restarts", &restarts.to_string()),
            ("clouds_dropped", "0"),
        ],
    )
}

/// The car on its loop at `t` seconds: base_link in odom.
fn car_at(t: f64) -> Pose {
    let angle = SPEED * t / RADIUS;
    let p = CENTRE + Point::new(angle.cos(), angle.sin()) * RADIUS;
    Pose::new(p.x, p.y, angle + PI / 2.0)
}

/// odom to map.
fn in_map(p: Pose) -> Pose {
    Pose {
        position: ODOM_IN_MAP.transform(p.position),
        yaw: p.yaw + ODOM_IN_MAP.yaw,
    }
}

/// `--bench-cubes`: `n` cubes over a 100 m square round the origin, five layers high.
pub fn bench_scan(n: usize, size: f32) -> CubeScan {
    let side = ((n / 5) as f64).sqrt().ceil().max(1.0) as usize;
    let cubes = (0..n)
        .map(|i| {
            let (layer, cell) = (i % 5, i / 5);
            let (gx, gy) = (cell % side, cell / side);
            let step = 100.0 / side as f32;
            Cube {
                centre: [
                    gx as f32 * step - 50.0,
                    gy as f32 * step - 50.0,
                    0.1 + layer as f32 * 0.6,
                ],
                band: cubes::Band::ALL[layer % cubes::Band::ALL.len()],
            }
        })
        .collect();
    CubeScan {
        cubes,
        size,
        points: n,
        ..CubeScan::default()
    }
}

/// The demo's clock and its news.
pub struct Demo {
    start: Instant,
    last_scan: Option<Instant>,
    last_news: Option<Instant>,
    settings: CubeSettings,
    bench: Option<CubeScan>,
    tracker: Tracker,
    replies: Vec<Update>,
}

impl Demo {
    pub fn new(start: Instant, settings: CubeSettings, bench_cubes: Option<usize>) -> Self {
        let size = settings.size.0 as f32;
        Self {
            start,
            last_scan: None,
            last_news: None,
            settings,
            bench: bench_cubes.map(|n| bench_scan(n, size)),
            tracker: Tracker::PurePursuit,
            replies: Vec::new(),
        }
    }

    fn t(&self, now: Instant) -> f64 {
        now.duration_since(self.start).as_secs_f64()
    }
}

/// True every `period` since `last`, which it moves on.
fn due(last: &mut Option<Instant>, now: Instant, period: Duration) -> bool {
    if last.is_some_and(|l| now.duration_since(l) < period) {
        return false;
    }
    *last = Some(now);
    true
}

impl DataSource for Demo {
    fn label(&self) -> &'static str {
        "DEMO"
    }

    fn drain(&mut self, now: Instant) -> Vec<Update> {
        if !due(&mut self.last_news, now, Duration::from_millis(50)) {
            return std::mem::take(&mut self.replies);
        }
        let t = self.t(now);
        let car = car_at(t);
        let truth = in_map(car);
        // The global EKF wanders round the truth; AMCL jitters.
        let global = Pose::new(
            truth.position.x + 0.35 * (t * 0.3).sin(),
            truth.position.y + 0.25 * (t * 0.23).cos(),
            truth.yaw + 0.02,
        );
        let amcl = Pose::new(
            truth.position.x - 0.15 + 0.1 * (t * 3.1).sin(),
            truth.position.y + 0.1 * (t * 2.3).cos(),
            truth.yaw - 0.01,
        );
        let ndt = Pose::new(
            truth.position.x + 0.03 * (t * 1.3).sin(),
            truth.position.y - 0.02,
            truth.yaw + 0.003,
        );
        let ndt_lost = t % 30.0 > 28.0;
        let pose = |who, pose, frame: &str| Update::Pose {
            who,
            pose,
            frame: frame.into(),
            speed: Some(SPEED),
        };
        let angle = SPEED * t / RADIUS;
        let plan: Vec<Point> = (0..80)
            .map(|i| {
                let a = angle + f64::from(i) * 0.04;
                ODOM_IN_MAP.transform(CENTRE + Point::new(a.cos(), a.sin()) * (RADIUS + 0.1))
            })
            .collect();
        let steer = (1.57 / RADIUS).atan() as f32;
        let heard = |source, detail: &str| Update::Heard {
            source,
            detail: detail.into(),
        };
        let mut news = std::mem::take(&mut self.replies);
        news.extend([
            Update::MapToOdom(ODOM_IN_MAP),
            pose(Localizer::Local, car, "odom"),
            pose(Localizer::Global, global, "map"),
            pose(Localizer::Truth, truth, "map"),
            pose(Localizer::Amcl, amcl, "map"),
            Update::Particles {
                poses: particles(amcl, t),
                frame: "map".into(),
            },
            Update::Ndt(ndt_status(t)),
            Update::Plan {
                points: plan,
                frame: "map".into(),
            },
            heard(
                Source::Imu,
                &format!("yaw rate {:+.1} deg/s", (SPEED / RADIUS).to_degrees()),
            ),
            heard(Source::Gps, "fix, 3 m"),
            heard(Source::Wheels, &format!("{SPEED:.2} m/s")),
            heard(Source::WheelOdom, &format!("{SPEED:.2} m/s")),
            Update::Vehicle(VehicleStatus {
                mode: Mode::Computer,
                armed: true,
                estop_active: false,
                speed: MetresPerSecond(SPEED as f32),
                steering_angle: Radians(steer),
                throttle: Fraction::new(0.2),
                brake: Fraction::new(0.0),
                faults: Faults(0),
            }),
            Update::Track(TrackStatus {
                tracker: self.tracker.name().into(),
                cross_track: Some(0.04 * (t * 0.7).sin()),
                loop_ms: Some(0.3),
                missed_ticks: Some(0),
                steer: Some(f64::from(steer)),
                left: None,
            }),
        ]);
        // NDT publishes a pose only for a good fit.
        if !ndt_lost {
            news.push(pose(Localizer::Ndt, ndt, "map"));
        }
        news
    }

    fn built_in_map(&self) -> Option<MapCells> {
        Some(lot_map(1.0))
    }

    fn swap_tracker(&mut self, tracker: Tracker) -> anyhow::Result<()> {
        self.tracker = tracker;
        self.replies.push(Update::SwapReply(Reply {
            asked: tracker.name(),
            outcome: Ok(()),
        }));
        Ok(())
    }

    fn take_scan(&mut self) -> Option<CubeScan> {
        let now = Instant::now();
        if let Some(bench) = &self.bench {
            return due(&mut self.last_scan, now, Duration::from_millis(100))
                .then(|| bench.clone());
        }
        if !due(&mut self.last_scan, now, Duration::from_millis(100)) {
            return None;
        }
        let t = self.t(now);
        let car = car_at(t);
        let lidar = Transform3 {
            translation: LIDAR,
            ..Transform3::IDENTITY
        };
        let stamp = Stamp((t * 1e9) as i64);
        Some(cubes::build(sweep(car), &lidar, car, stamp, &self.settings))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use debug_core::cubes::Metres;

    #[test]
    fn a_ray_hits_the_box_in_front() {
        let b = Aabb::new([5.0, -1.0, 0.0], [6.0, 1.0, 2.0]);
        assert_eq!(b.hit([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]), Some(5.0));
        assert_eq!(b.hit([0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]), None);
        assert_eq!(b.hit([0.0, 0.0, 3.0], [1.0, 0.0, 0.0]), None);
    }

    #[test]
    fn the_sweep_sees_walls_and_ground() {
        let settings = CubeSettings {
            size: Metres(0.2),
            max_cubes: 100_000,
            ground_height: Metres(0.25),
        };
        let car = car_at(0.0);
        let lidar = Transform3 {
            translation: LIDAR,
            ..Transform3::IDENTITY
        };
        let scan = cubes::build(sweep(car), &lidar, car, Stamp(0), &settings);
        assert!(scan.cubes.len() > 1000, "{}", scan.cubes.len());
        assert!(scan.cubes.iter().any(|c| c.band == cubes::Band::Ground));
        assert!(scan.cubes.iter().any(|c| c.band != cubes::Band::Ground));
    }

    #[test]
    fn the_lot_is_its_own_map() {
        let map = lot_map(1.0);
        // The four walls alone are about 150 m long and 3 cells high.
        assert!(map.len() > 300, "{}", map.len());
        assert!(map.centres.iter().all(|c| c[2] > 0.0));
    }

    #[test]
    fn ndt_is_lost_for_a_moment() {
        assert_eq!(ndt_status(5.0).headline(), "TRACKING");
        assert_eq!(ndt_status(29.0).headline(), "LOST");
        assert!(particles(Pose::default(), 0.0)
            .iter()
            .all(|p| p.position.x.abs() <= 0.4));
    }

    #[test]
    fn the_drawing_test_makes_n_cubes() {
        assert_eq!(bench_scan(20_000, 0.2).cubes.len(), 20_000);
    }
}
