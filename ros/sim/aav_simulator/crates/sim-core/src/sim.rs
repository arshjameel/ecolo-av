//! The simulator's loop, with no ROS: step the car, and say which sensor speaks when.
//!
//! Time is whole nanoseconds from the start, stepped by a fixed `step` (1 ms, as Gazebo's
//! worlds). A stream with period p speaks at every multiple of p, so the rates are exact on
//! the simulated clock, whatever the computer's speed. What it says is an [`Output`]: the
//! node turns each into a message, stamped with [`Simulator::time_ns`].
//!
//! Cameras are not here: they plug in through [`SensorPlugin`] (see its notes).

use std::path::PathBuf;

use crate::car::{CarModel, CarState, Command};
use crate::geo::Geodetic;
use crate::ground::Ground;
use crate::lidar::{LidarSpec, Scan};
use crate::math::{Pose, Rot3, Vec3};
use crate::noise::Noise;
use crate::world::World;

/// m/s²: SDF's default gravity, which our worlds keep.
pub const GRAVITY: f64 = 9.8;

/// The car's size, for the wheels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geometry {
    pub wheelbase: f64,
    pub track: f64,
    pub wheel_radius: f64,
}

/// The car's outline, for hitting things (vehicle.yaml's `body:`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Body {
    /// m, bumper to bumper.
    pub length: f64,
    pub width: f64,
    /// m from the rear bumper to the rear axle (base_link).
    pub rear_overhang: f64,
    pub height: f64,
}

impl Body {
    /// Heights (m) the outline is felt at: over a 0.30 m kerb, which the wheels climb in
    /// Gazebo or not; a wall, a post, a box or a parked car meets one of them.
    const FEELERS: [f64; 3] = [0.35, 0.8, 1.3];

    /// The outline's four corners in base_link, anticlockwise from the rear right.
    fn corners(&self) -> [Vec3; 4] {
        let (back, front) = (-self.rear_overhang, self.length - self.rear_overhang);
        let half = self.width / 2.0;
        [
            Vec3::new(back, -half, 0.0),
            Vec3::new(front, -half, 0.0),
            Vec3::new(front, half, 0.0),
            Vec3::new(back, half, 0.0),
        ]
    }

    /// Whether anything in the scene crosses the outline, with the car at `base`.
    ///
    /// Rays along each side and across both diagonals, at each feeler height: something
    /// that reaches into the car crosses one of them first. The diagonals catch a thing
    /// exactly as wide as the car and in line with it, whose faces lie along the sides.
    /// Cheap (18 short rays a step), and enough to stop at a wall.
    pub fn touches(&self, scene: &crate::scene::Scene, base: &Pose) -> bool {
        let c = self.corners();
        const LINES: [(usize, usize); 6] = [(0, 1), (1, 2), (2, 3), (3, 0), (0, 2), (1, 3)];
        Self::FEELERS
            .iter()
            .filter(|&&z| z < self.height)
            .any(|&z| {
                LINES.iter().any(|&(i, j)| {
                    let a = base.apply(Vec3::new(c[i].x, c[i].y, z));
                    let b = base.apply(Vec3::new(c[j].x, c[j].y, z));
                    let line = b - a;
                    let length = line.length();
                    let ray = crate::scene::Ray {
                        origin: a,
                        direction: line * (1.0 / length),
                    };
                    scene.cast(&ray, length).is_some()
                })
            })
    }
}

/// Where each sensor sits on the car, from `base_link` (vehicle.yaml's `sensors:`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mounts {
    pub imu: Pose,
    pub gps: Pose,
    pub lidar: Pose,
    pub radar: Pose,
}

/// How often each stream speaks, Hz. The defaults are Gazebo's (aav.gazebo.xacro).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rates {
    /// `/clock`: one per physics step, 1 kHz, as Gazebo (its clock is not
    /// throttled), so a sensor's stamp is never ahead of the nodes' clocks by more than a step.
    pub clock: f64,
    /// The AckermannSteering system's odometry (`odom_publish_frequency`).
    pub odom: f64,
    /// The OdometryPublisher's truth.
    pub truth: f64,
    pub imu: f64,
    pub navsat: f64,
    /// The wheels' joint states. Gazebo sends them every physics step; the car's wheel
    /// nodes, 100 Hz.
    pub joints: f64,
}

impl Default for Rates {
    fn default() -> Self {
        Self {
            clock: 1000.0,
            odom: 50.0,
            truth: 50.0,
            imu: 50.0,
            navsat: 5.0,
            joints: 100.0,
        }
    }
}

/// The IMU's and GPS's noise (aav.gazebo.xacro's, standard deviations).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorNoise {
    /// rad/s.
    pub gyro: f64,
    /// m/s².
    pub accel: f64,
    /// Degrees, added to latitude and longitude each (4.5e-6: about 0.5 m).
    pub gps_horizontal_deg: f64,
    /// m.
    pub gps_vertical: f64,
}

impl Default for SensorNoise {
    fn default() -> Self {
        Self {
            gyro: 0.02,
            accel: 0.2,
            gps_horizontal_deg: 4.5e-6,
            gps_vertical: 1.0,
        }
    }
}

/// Everything the loop needs besides the world and the car.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// ns per step.
    pub step_ns: u64,
    pub geometry: Geometry,
    pub mounts: Mounts,
    pub rates: Rates,
    pub noise: SensorNoise,
    /// The roof lidar, if the car has one in this run.
    pub lidar: Option<LidarSpec>,
    /// The radar stand-in, if it runs.
    pub radar: Option<LidarSpec>,
    /// The noise's seed: the same seed, the same sensor streams.
    pub seed: u64,
    /// Workers for the lidar's rays.
    pub threads: usize,
    /// The world file, for plug-ins that draw it themselves (cameras).
    pub world_file: Option<PathBuf>,
    /// The car's outline: it stops at what it hits, as Gazebo's does. None: it drives
    /// through everything.
    pub body: Option<Body>,
    /// m: the highest step a wheel rolls up (`ground::Ground`): the car rides the world's
    /// ground, tilts on slopes, and stops at a higher kerb. None: flat ground at z = 0.
    pub ground_step: Option<f64>,
}

/// One IMU reading, in `imu_link`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuReading {
    /// The sensor's orientation in the world, (x, y, z, w).
    pub orientation: [f64; 4],
    /// rad/s.
    pub angular_velocity: Vec3,
    /// m/s²: what an accelerometer reads, gravity included (+9.8 up at rest).
    pub linear_acceleration: Vec3,
}

/// The wheels' joints, in the URDF's names (aav.urdf.xacro).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Joints {
    /// rad and rad/s: rear_left, rear_right, front_left, front_right wheel joints.
    pub wheel_angle: [f64; 4],
    pub wheel_rate: [f64; 4],
    /// rad: front_left, front_right steering joints (Ackermann).
    pub steering: [f64; 2],
}

impl Joints {
    pub const WHEELS: [&'static str; 4] = [
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
        "front_left_wheel_joint",
        "front_right_wheel_joint",
    ];
    pub const STEERING: [&'static str; 2] =
        ["front_left_steering_joint", "front_right_steering_joint"];
}

/// The car's motion as an odometry message carries it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Odometry {
    /// The pose in the message's frame.
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    /// m/s and rad/s, in base_link.
    pub speed: f64,
    pub yaw_rate: f64,
    /// m and rad: the height and tilt on the ground (0 for the wheels' odometry).
    pub z: f64,
    pub roll: f64,
    pub pitch: f64,
}

/// Something to publish now.
#[derive(Debug, Clone, PartialEq)]
pub enum Output {
    Clock,
    /// The wheels' odometry, from where the car started (`/gazebo/odom`, odom frame).
    Odom(Odometry),
    /// Where the car really is, in the map (`/vehicle/sim/truth`).
    Truth(Odometry),
    Imu(ImuReading),
    NavSat(Geodetic),
    Joints(Joints),
    Lidar(Scan),
    Radar(Scan),
    /// The car has just hit something, here (x, y in the map), and stopped.
    Hit {
        x: f64,
        y: f64,
    },
}

/// What a plug-in sees when it's its turn.
pub struct Frame<'a> {
    pub time_ns: u64,
    pub world: &'a World,
    /// The world file, for a plug-in that loads its own copy (with colours).
    pub world_file: Option<&'a std::path::Path>,
    pub car: CarState,
    /// `base_link` in the world (= map) frame; a sensor's pose is this then its mount.
    pub base_link: Pose,
}

/// A sensor the simulator doesn't make itself: the cameras (0.3: another crate).
///
/// The simulator calls `sample` at every multiple of `period_ns` on its clock, after the
/// car has moved, with the world and the car's pose. The plug-in makes its picture and
/// publishes it itself (it owns its publishers), so sim-core needs no ROS and no Bevy.
/// It runs inside the loop: a slow one slows the clock, so keep it quick or hand the work
/// to a thread of its own and drop frames, as a real camera would.
pub trait SensorPlugin: Send {
    fn name(&self) -> &str;
    fn period_ns(&self) -> u64;
    fn sample(&mut self, frame: &Frame<'_>);
}

fn period_ns(hz: f64, step_ns: u64) -> u64 {
    if hz <= 0.0 {
        return 0;
    }
    // A whole number of steps, at least one.
    let steps = ((1e9 / hz) / step_ns as f64).round().max(1.0) as u64;
    steps * step_ns
}

/// The loop's state.
pub struct Simulator {
    settings: Settings,
    world: World,
    car: Box<dyn CarModel>,
    plugins: Vec<Box<dyn SensorPlugin>>,
    time_ns: u64,
    start: CarState,
    odom: Odometry,
    joints: Joints,
    imu_noise: Noise,
    gps_noise: Noise,
    lidar_noise: Noise,
    radar_noise: Noise,
    /// When the car last pushed against something (ns): a new hit is one after a second
    /// clear of it, not each step it keeps pushing.
    last_contact_ns: Option<u64>,
    /// Each wheel's ground height (m): rear right, rear left, front right, front left.
    wheel_heights: [f64; 4],
}

impl Simulator {
    /// A simulator with the car at (x, y, yaw) in the world.
    pub fn new(
        settings: Settings,
        world: World,
        mut car: Box<dyn CarModel>,
        start: (f64, f64, f64),
    ) -> Self {
        car.reset(start.0, start.1, start.2);
        let mut seed = Noise::new(settings.seed);
        let state = car.state();
        Self {
            imu_noise: seed.fork(),
            gps_noise: seed.fork(),
            lidar_noise: seed.fork(),
            radar_noise: seed.fork(),
            settings,
            world,
            car,
            plugins: Vec::new(),
            time_ns: 0,
            start: state,
            odom: Odometry {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
                speed: 0.0,
                yaw_rate: 0.0,
                z: 0.0,
                roll: 0.0,
                pitch: 0.0,
            },
            joints: Joints::default(),
            last_contact_ns: None,
            wheel_heights: [0.0; 4],
        }
    }

    /// Add a sensor plug-in (a camera).
    pub fn add_plugin(&mut self, plugin: Box<dyn SensorPlugin>) {
        self.plugins.push(plugin);
    }

    pub fn time_ns(&self) -> u64 {
        self.time_ns
    }

    pub fn car(&self) -> CarState {
        self.car.state()
    }

    pub fn car_model(&self) -> &dyn CarModel {
        self.car.as_ref()
    }

    pub fn world(&self) -> &World {
        &self.world
    }

    /// The world, to place or remove an object while running.
    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }

    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// `base_link` in the world: on the ground under the rear axle, tilted to the wheels
    /// (flat at z = 0 without `ground_step`).
    pub fn base_link(&self) -> Pose {
        let s = self.car.state();
        match self.ground() {
            Some(g) => g.pose(s.x, s.y, s.yaw, self.wheel_heights),
            None => Pose::new(Vec3::new(s.x, s.y, 0.0), Rot3::from_yaw(s.yaw)),
        }
    }

    fn ground(&self) -> Option<Ground> {
        let g = self.settings.geometry;
        self.settings.ground_step.map(|max_step| Ground {
            max_step,
            wheelbase: g.wheelbase,
            track: g.track,
        })
    }

    /// Put the car at (x, y, yaw), standing still, its actuators at rest (Gazebo's
    /// `set_pose`; the node's SetEntityState). The truth and wheel odometry jump with it,
    /// as in Gazebo; on a ground with steps the wheels settle again.
    pub fn place_car(&mut self, x: f64, y: f64, yaw: f64) {
        self.car.reset(x, y, yaw);
        self.settle();
    }

    /// The wheels' heights where the car stands, found from a little up.
    fn settle(&mut self) {
        if let Some(g) = self.ground() {
            let s = self.car.state();
            let anywhere = Ground {
                max_step: f64::INFINITY,
                ..g
            };
            if let Ok(h) = anywhere.settle(&self.world.scene, s.x, s.y, s.yaw, [0.4; 4]) {
                self.wheel_heights = h;
            }
        }
    }

    /// The first outputs, at time 0: the clock and where the car stands.
    pub fn start(&mut self) -> Vec<Output> {
        // Find the ground from a little up: the car may start on a raised lot.
        self.settle();
        let mut out = vec![Output::Clock];
        self.speak(&mut out, true);
        out
    }

    /// Move on one step under `command`; return what is due now.
    pub fn step(&mut self, command: &Command) -> Vec<Output> {
        let dt = self.settings.step_ns as f64 * 1e-9;
        let before = self.car.state();
        let mut s = self.car.step(command, dt);
        self.time_ns += self.settings.step_ns;
        let mut out = Vec::new();
        let mut heights = self.wheel_heights;
        let mut blocked = false;
        if let Some(g) = self.ground() {
            match g.settle(&self.world.scene, s.x, s.y, s.yaw, self.wheel_heights) {
                Ok(h) => heights = h,
                Err(_) => blocked = true,
            }
        }
        if let Some(body) = self.settings.body {
            let at = match self.ground() {
                Some(g) => g.pose(s.x, s.y, s.yaw, heights),
                None => Pose::new(Vec3::new(s.x, s.y, 0.0), Rot3::from_yaw(s.yaw)),
            };
            blocked = blocked || body.touches(&self.world.scene, &at);
        }
        if !blocked {
            self.wheel_heights = heights;
        }
        {
            if blocked {
                // Back to where it was, standing: a wall stops the car, as Gazebo's physics.
                self.car.hold(&before);
                s = self.car.state();
                let new = self
                    .last_contact_ns
                    .is_none_or(|t| self.time_ns - t > 1_000_000_000);
                if new {
                    out.push(Output::Hit { x: s.x, y: s.y });
                }
                self.last_contact_ns = Some(self.time_ns);
            }
        }
        self.integrate(&s, dt);
        self.speak(&mut out, false);
        out
    }

    fn due(&self, hz: f64) -> bool {
        let p = period_ns(hz, self.settings.step_ns);
        p > 0 && self.time_ns.is_multiple_of(p)
    }

    fn speak(&mut self, out: &mut Vec<Output>, at_start: bool) {
        let r = self.settings.rates;
        if !at_start && self.due(r.clock) {
            out.push(Output::Clock);
        }
        if self.due(r.odom) {
            out.push(Output::Odom(self.odom));
        }
        if self.due(r.truth) {
            let s = self.car.state();
            let base = self.base_link();
            let (roll, pitch, _) = base.rotation.rpy();
            out.push(Output::Truth(Odometry {
                x: s.x,
                y: s.y,
                yaw: s.yaw,
                speed: s.speed,
                yaw_rate: s.yaw_rate,
                z: base.position.z,
                roll,
                pitch,
            }));
        }
        if self.due(r.imu) {
            out.push(Output::Imu(self.imu()));
        }
        if self.due(r.navsat) {
            out.push(Output::NavSat(self.navsat()));
        }
        if self.due(r.joints) {
            out.push(Output::Joints(self.joints));
        }
        let base = self.base_link();
        let threads = self.settings.threads;
        if let Some(spec) = self.settings.lidar {
            if self.due(spec.rate) {
                let pose = base.then(&self.settings.mounts.lidar);
                out.push(Output::Lidar(Scan::cast(
                    &spec,
                    &self.world.scene,
                    &pose,
                    &mut self.lidar_noise,
                    threads,
                )));
            }
        }
        if let Some(spec) = self.settings.radar {
            if self.due(spec.rate) {
                let pose = base.then(&self.settings.mounts.radar);
                out.push(Output::Radar(Scan::cast(
                    &spec,
                    &self.world.scene,
                    &pose,
                    &mut self.radar_noise,
                    threads,
                )));
            }
        }
        let frame = Frame {
            time_ns: self.time_ns,
            world: &self.world,
            world_file: self.settings.world_file.as_deref(),
            car: self.car.state(),
            base_link: base,
        };
        for p in &mut self.plugins {
            let period = p.period_ns();
            if period > 0 && frame.time_ns.is_multiple_of(period) {
                p.sample(&frame);
            }
        }
    }

    /// The wheels' odometry and the joints, after the car moved by a step.
    fn integrate(&mut self, s: &CarState, dt: f64) {
        // The odometry: the car's motion from where it started, in its start heading.
        let (dx, dy) = (s.x - self.start.x, s.y - self.start.y);
        let (sin0, cos0) = self.start.yaw.sin_cos();
        self.odom = Odometry {
            x: cos0 * dx + sin0 * dy,
            y: -sin0 * dx + cos0 * dy,
            yaw: crate::math::wrap(s.yaw - self.start.yaw),
            speed: s.speed,
            yaw_rate: s.yaw_rate,
            z: 0.0,
            roll: 0.0,
            pitch: 0.0,
        };
        let g = self.settings.geometry;
        let half = g.track / 2.0;
        // Each wheel's ground speed along its own heading.
        let rear = [s.speed - s.yaw_rate * half, s.speed + s.yaw_rate * half];
        let side = s.yaw_rate * g.wheelbase;
        let front = [
            (rear[0].powi(2) + side.powi(2)).sqrt().copysign(s.speed),
            (rear[1].powi(2) + side.powi(2)).sqrt().copysign(s.speed),
        ];
        let speeds = [rear[0], rear[1], front[0], front[1]];
        for (i, v) in speeds.iter().enumerate() {
            let rate = v / g.wheel_radius;
            self.joints.wheel_rate[i] = rate;
            self.joints.wheel_angle[i] = crate::math::wrap(self.joints.wheel_angle[i] + rate * dt);
        }
        self.joints.steering = wheel_angles(s.steering, g.wheelbase, g.track);
    }

    fn imu(&mut self) -> ImuReading {
        let s = self.car.state();
        let mount = self.settings.mounts.imu;
        let world = self.base_link().then(&mount);
        let into_imu = mount.rotation.transpose();
        let n = self.settings.noise;
        let rate = into_imu.apply(Vec3::new(0.0, 0.0, s.yaw_rate));
        // The specific force at the sensor: the car's acceleration (along, and the turn's
        // pull toward the centre, at the sensor's place ahead of the axle) and gravity.
        let r = mount.position;
        // Gravity's pull, as the tilted car feels it (straight up on flat ground).
        let up = self
            .base_link()
            .rotation
            .transpose()
            .apply(Vec3::new(0.0, 0.0, GRAVITY));
        let accel = Vec3::new(
            s.accel - s.yaw_rate * s.yaw_rate * r.x + up.x,
            s.speed * s.yaw_rate + up.y,
            up.z,
        );
        let accel = into_imu.apply(accel);
        let g = &mut self.imu_noise;
        ImuReading {
            orientation: world.rotation.quaternion(),
            angular_velocity: Vec3::new(
                rate.x + g.gauss(n.gyro),
                rate.y + g.gauss(n.gyro),
                rate.z + g.gauss(n.gyro),
            ),
            linear_acceleration: Vec3::new(
                accel.x + g.gauss(n.accel),
                accel.y + g.gauss(n.accel),
                accel.z + g.gauss(n.accel),
            ),
        }
    }

    fn navsat(&mut self) -> Geodetic {
        let antenna = self.base_link().apply(self.settings.mounts.gps.position);
        let mut fix = self.world.datum.to_geodetic(antenna);
        let n = self.settings.noise;
        let g = &mut self.gps_noise;
        fix.latitude += g.gauss(n.gps_horizontal_deg);
        fix.longitude += g.gauss(n.gps_horizontal_deg);
        fix.altitude += g.gauss(n.gps_vertical);
        fix
    }
}

/// Each front wheel's angle for the centre wheel's `steering` (Ackermann), left then right:
/// both point at the same turning centre.
pub fn wheel_angles(steering: f64, wheelbase: f64, track: f64) -> [f64; 2] {
    if steering.abs() < 1e-9 {
        return [0.0, 0.0];
    }
    let radius = wheelbase / steering.tan();
    let half = track / 2.0;
    [
        (wheelbase / (radius - half)).atan(),
        (wheelbase / (radius + half)).atan(),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::car::TwistFollower;

    fn settings() -> Settings {
        let at = |x, z| Pose::from_xyz_rpy([x, 0.0, z], [0.0; 3]);
        Settings {
            step_ns: 1_000_000,
            geometry: Geometry {
                wheelbase: 1.561,
                track: 0.75,
                wheel_radius: 0.18,
            },
            mounts: Mounts {
                imu: at(0.49, 1.44),
                gps: at(0.0, 1.4),
                lidar: at(0.49, 1.52),
                radar: at(1.8, 0.45),
            },
            rates: Rates::default(),
            noise: SensorNoise::default(),
            lidar: Some(LidarSpec::ROOF),
            radar: None,
            seed: 1,
            threads: 2,
            world_file: None,
            body: Some(Body {
                length: 2.0,
                width: 1.0,
                rear_overhang: 0.2,
                height: 1.5,
            }),
            ground_step: None,
        }
    }

    #[test]
    fn a_box_as_wide_as_the_car_and_in_line_still_stops_it() {
        let body = settings().body.expect("a body");
        let mut scene = crate::scene::Scene::new(Vec::new());
        // The bench's 1 m box, its near face 0.1 m inside the front bumper (x = 1.8).
        scene.place(
            "box",
            &crate::scene::Mesh::cuboid(Vec3::new(1.0, 1.0, 1.0)),
            &Pose::new(Vec3::new(2.2, 0.0, 0.5), Rot3::IDENTITY),
        );
        assert!(body.touches(&scene, &Pose::IDENTITY));
        let back = Pose::new(Vec3::new(-0.2, 0.0, 0.0), Rot3::IDENTITY);
        assert!(!body.touches(&scene, &back));
    }

    #[test]
    fn the_car_can_be_placed_anywhere_and_stands_still_there() {
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut sim = Simulator::new(settings(), World::flat(400.0), car, (0.0, 0.0, 0.0));
        let go = Command::Twist {
            speed: 1.0,
            yaw_rate: 0.0,
        };
        for _ in 0..1000 {
            sim.step(&go);
        }
        assert!(sim.car().speed > 0.5);
        sim.place_car(10.0, -4.0, 1.0);
        let s = sim.car();
        assert_eq!((s.x, s.y, s.yaw), (10.0, -4.0, 1.0));
        assert_eq!(s.speed, 0.0);
        let stop = Command::Twist {
            speed: 0.0,
            yaw_rate: 0.0,
        };
        for _ in 0..100 {
            sim.step(&stop);
        }
        assert!((sim.car().x - 10.0).abs() < 1e-6);
    }

    #[test]
    fn the_car_stops_at_a_wall_and_says_so() {
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut world = World::flat(400.0);
        // A wall whose near face is 3 m ahead of the front bumper (at x = 1.8).
        world.scene.place(
            "wall",
            &crate::scene::Mesh::cuboid(Vec3::new(1.0, 10.0, 2.0)),
            &Pose::new(Vec3::new(5.3, 0.0, 1.0), Rot3::IDENTITY),
        );
        let mut sim = Simulator::new(settings(), world, car, (0.0, 0.0, 0.0));
        let go = Command::Twist {
            speed: 1.0,
            yaw_rate: 0.0,
        };
        let mut hits = 0;
        for _ in 0..6000 {
            hits += sim
                .step(&go)
                .iter()
                .filter(|o| matches!(o, Output::Hit { .. }))
                .count();
        }
        let s = sim.car();
        assert!(
            s.x > 2.9 && s.x <= 3.0,
            "the bumper at the wall: x = {}",
            s.x
        );
        assert_eq!(s.speed, 0.0);
        assert_eq!(hits, 1, "one hit, however long it pushes");
    }

    #[test]
    fn a_kerb_stops_the_wheels_and_a_step_onto_a_mat_lifts_the_truth() {
        let mut s = settings();
        s.body = None; // only the wheels feel the ground
        s.ground_step = Some(0.1);
        let mut world = World::flat(400.0);
        // A 5 cm mat from 3 m on, then a 0.3 m kerb from 10 m on.
        world.scene.place(
            "mat",
            &crate::scene::Mesh::cuboid(Vec3::new(4.0, 10.0, 0.05)),
            &Pose::new(Vec3::new(5.0, 0.0, 0.025), Rot3::IDENTITY),
        );
        world.scene.place(
            "kerb",
            &crate::scene::Mesh::cuboid(Vec3::new(10.0, 10.0, 0.3)),
            &Pose::new(Vec3::new(15.0, 0.0, 0.15), Rot3::IDENTITY),
        );
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut sim = Simulator::new(s, world, car, (0.0, 0.0, 0.0));
        sim.start();
        let go = Command::Twist {
            speed: 1.0,
            yaw_rate: 0.0,
        };
        let mut on_mat = None;
        let mut hits = 0;
        for _ in 0..12_000 {
            for o in sim.step(&go) {
                match o {
                    Output::Truth(t) if (4.0..5.0).contains(&t.x) => on_mat = Some(t),
                    Output::Hit { .. } => hits += 1,
                    _ => {}
                }
            }
        }
        let t = on_mat.expect("the car crossed the mat");
        assert!((t.z - 0.05).abs() < 1e-9 && t.pitch.abs() < 1e-9, "{t:?}");
        // The front wheels (1.561 m ahead) stop at the kerb's face, at x = 10.
        let x = sim.car().x;
        assert!(x > 8.4 && x < 8.44, "the front wheels at the kerb: x = {x}");
        assert_eq!(hits, 1);
    }

    #[test]
    fn each_stream_speaks_at_its_rate() {
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut sim = Simulator::new(settings(), World::flat(400.0), car, (0.0, 0.0, 0.0));
        let mut counts = std::collections::HashMap::new();
        let still = Command::Twist {
            speed: 0.0,
            yaw_rate: 0.0,
        };
        for _ in 0..1000 {
            for o in sim.step(&still) {
                let name = match o {
                    Output::Clock => "clock",
                    Output::Odom(_) => "odom",
                    Output::Truth(_) => "truth",
                    Output::Imu(_) => "imu",
                    Output::NavSat(_) => "navsat",
                    Output::Joints(_) => "joints",
                    Output::Lidar(_) => "lidar",
                    Output::Radar(_) => "radar",
                    Output::Hit { .. } => "hit",
                };
                *counts.entry(name).or_insert(0) += 1;
            }
        }
        assert_eq!(sim.time_ns(), 1_000_000_000);
        for (name, n) in [
            ("clock", 1000),
            ("odom", 50),
            ("truth", 50),
            ("imu", 50),
            ("navsat", 5),
            ("joints", 100),
            ("lidar", 10),
        ] {
            assert_eq!(counts.get(name), Some(&n), "{name}");
        }
        assert!(!counts.contains_key("radar"));
    }

    #[test]
    fn at_rest_the_imu_feels_gravity_and_the_gps_is_at_the_datum() {
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut s = settings();
        s.noise = SensorNoise {
            gyro: 0.0,
            accel: 0.0,
            gps_horizontal_deg: 0.0,
            gps_vertical: 0.0,
        };
        let mut sim = Simulator::new(s, World::flat(400.0), car, (0.0, 0.0, 0.0));
        let imu = sim.imu();
        assert!((imu.linear_acceleration.z - GRAVITY).abs() < 1e-12);
        let fix = sim.navsat();
        assert!((fix.latitude - 45.386).abs() < 1e-9);
        assert!((fix.altitude - 71.4).abs() < 1e-6);
    }

    #[test]
    fn driving_east_moves_the_truth_and_the_odometry_alike() {
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut sim = Simulator::new(
            settings(),
            World::flat(400.0),
            car,
            (2.0, 3.0, std::f64::consts::FRAC_PI_2),
        );
        let go = Command::Twist {
            speed: 1.0,
            yaw_rate: 0.0,
        };
        let mut last = None;
        for _ in 0..2000 {
            for o in sim.step(&go) {
                if let Output::Odom(odom) = o {
                    last = Some(odom);
                }
            }
        }
        let s = sim.car();
        assert!(
            (s.x - 2.0).abs() < 1e-6 && s.y > 4.8,
            "facing north: ({}, {})",
            s.x,
            s.y
        );
        let odom = last.expect("odometry");
        assert!((odom.x - (s.y - 3.0)).abs() < 1e-6 && odom.y.abs() < 1e-6);
    }

    #[test]
    fn ackermann_inner_wheel_turns_more() {
        let [left, right] = wheel_angles(0.3, 1.561, 0.75);
        assert!(left > 0.3 && right < 0.3 && right > 0.0);
    }
}
