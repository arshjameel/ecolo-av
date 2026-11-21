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
use crate::scene::Mesh;
use crate::world::World;

/// m/s²: SDF's default gravity, which our worlds keep.
pub const GRAVITY: f64 = 9.8;

/// The car's size, for the wheels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geometry {
    /// m: rear axle to front axle.
    pub wheelbase: f64,
    /// m: left wheel to right wheel.
    pub track: f64,
    /// m.
    pub wheel_radius: f64,
}

/// The car's outline, for hitting things (vehicle.yaml's `body:`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Body {
    /// m, bumper to bumper.
    pub length: f64,
    /// m, side to side.
    pub width: f64,
    /// m from the rear bumper to the rear axle (base_link).
    pub rear_overhang: f64,
    /// m, ground to roof.
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
    #[must_use]
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
    /// The IMU (`imu_link`).
    pub imu: Pose,
    /// The GPS antenna.
    pub gps: Pose,
    /// The roof lidar.
    pub lidar: Pose,
    /// The radar.
    pub radar: Pose,
}

/// How often each stream speaks, Hz. The defaults are Gazebo's (its URDF, until 16 Nov).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rates {
    /// `/clock`: one per physics step, 1 kHz, as Gazebo (its clock is not
    /// throttled), so a sensor's stamp is never ahead of the nodes' clocks by more than a step.
    pub clock: f64,
    /// The AckermannSteering system's odometry (`odom_publish_frequency`).
    pub odom: f64,
    /// The OdometryPublisher's truth.
    pub truth: f64,
    /// The IMU.
    pub imu: f64,
    /// The GPS fix.
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

/// The IMU's, GPS's and wheels' noise (Gazebo's URDF's until 16 Nov, standard deviations). The
/// wheels' is ours (Gazebo's odometry has none), off by default (Ben, 14 Nov).
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
    /// m/s, on each wheel speed reading while the wheels turn (0: none). The odometry
    /// then integrates the readings and drifts, as on the car.
    pub wheel_speed: f64,
    /// A fraction: the wheels read this much more than the ground moved (slip, worn
    /// tyres, a wrong radius; 0: none, 0.02: 2 % too far).
    pub wheel_slip: f64,
}

impl SensorNoise {
    /// Whether the wheels' odometry is read from noisy wheels rather than the truth.
    #[must_use]
    pub fn wheels_noisy(&self) -> bool {
        self.wheel_speed > 0.0 || self.wheel_slip != 0.0
    }
}

impl Default for SensorNoise {
    fn default() -> Self {
        Self {
            gyro: 0.02,
            accel: 0.2,
            gps_horizontal_deg: 4.5e-6,
            gps_vertical: 1.0,
            wheel_speed: 0.0,
            wheel_slip: 0.0,
        }
    }
}

/// Everything the loop needs besides the world and the car.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// ns per step.
    pub step_ns: u64,
    /// The car's size, for the wheels.
    pub geometry: Geometry,
    /// Where each sensor sits on the car.
    pub mounts: Mounts,
    /// How often each stream speaks.
    pub rates: Rates,
    /// How noisy each sensor is.
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
    /// rad/s: the same wheels, in the same order.
    pub wheel_rate: [f64; 4],
    /// rad: front_left, front_right steering joints (Ackermann).
    pub steering: [f64; 2],
}

impl Joints {
    /// The wheel joints' names, in `wheel_angle`'s order.
    pub const WHEELS: [&'static str; 4] = [
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
        "front_left_wheel_joint",
        "front_right_wheel_joint",
    ];
    /// The steering joints' names, in `steering`'s order.
    pub const STEERING: [&'static str; 2] =
        ["front_left_steering_joint", "front_right_steering_joint"];
}

/// The car's motion as an odometry message carries it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Odometry {
    /// The pose in the message's frame.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad, + left.
    pub yaw: f64,
    /// m/s and rad/s, in base_link.
    pub speed: f64,
    /// rad/s, + left.
    pub yaw_rate: f64,
    /// m and rad: the height and tilt on the ground (0 for the wheels' odometry).
    pub z: f64,
    /// rad.
    pub roll: f64,
    /// rad.
    pub pitch: f64,
}

/// Something to publish now.
#[derive(Debug, Clone, PartialEq)]
pub enum Output {
    /// The clock has moved on a step (`/clock`).
    Clock,
    /// The wheels' odometry, from where the car started (`/vehicle/sim/odom`, odom frame).
    Odom(Odometry),
    /// Where the car really is, in the map (`/vehicle/sim/truth`).
    Truth(Odometry),
    /// An IMU reading.
    Imu(ImuReading),
    /// A GPS fix.
    NavSat(Geodetic),
    /// The wheels' joint states.
    Joints(Joints),
    /// A lidar scan.
    Lidar(Scan),
    /// A radar scan.
    Radar(Scan),
    /// The car has just hit something, here (x, y in the map), and stopped.
    Hit {
        /// m, east.
        x: f64,
        /// m, north.
        y: f64,
    },
}

/// What a plug-in sees when it's its turn.
#[derive(Debug)]
pub struct Frame<'a> {
    /// ns on the simulator's clock.
    pub time_ns: u64,
    /// The world, as it is now.
    pub world: &'a World,
    /// The world file, for a plug-in that loads its own copy (with colours).
    pub world_file: Option<&'a std::path::Path>,
    /// Where the car is and what it's doing.
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
    /// The plug-in's name, for logs.
    fn name(&self) -> &str;
    /// ns between two samples.
    fn period_ns(&self) -> u64;
    /// Make one sample of the world as it is in `frame`.
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
// Debug by hand: the car model and the plug-ins are trait objects, shown by name.
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
    wheel_noise: Noise,
    /// m, m, rad: how far the wheels' odometry has drifted from the truth's (noisy
    /// wheels only), in the odometry's frame.
    drift: [f64; 3],
    /// When the car last pushed against something (ns): a new hit is one after a second
    /// clear of it, not each step it keeps pushing.
    last_contact_ns: Option<u64>,
    /// Each wheel's ground height (m): rear right, rear left, front right, front left.
    wheel_heights: [f64; 4],
    /// Placed models that move on their own (`set_motion`): a person walking.
    movers: Vec<Mover>,
    /// The car's motion at the last IMU reading: the next one averages over the time
    /// between, as a real IMU's filter does (None: the first, or the car was placed).
    imu_last: Option<ImuMemory>,
}

/// What the IMU remembers from its last reading.
#[derive(Debug, Clone, Copy, PartialEq)]
struct ImuMemory {
    time_ns: u64,
    /// m/s, forward.
    speed: f64,
    /// rad/s.
    yaw_rate: f64,
    /// `base_link`'s rotation in the world.
    rotation: Rot3,
}

impl std::fmt::Debug for Simulator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let plugins: Vec<&str> = self.plugins.iter().map(|p| p.name()).collect();
        f.debug_struct("Simulator")
            .field("settings", &self.settings)
            .field("time_ns", &self.time_ns)
            .field("plugins", &plugins)
            .field("movers", &self.movers.len())
            .finish_non_exhaustive()
    }
}

/// A placed model moving at a steady velocity in the world: a person crossing, a car
/// going by (TODO 170). The lidar, the radar and the car's body meet it where it is now.
#[derive(Debug, Clone)]
struct Mover {
    name: String,
    mesh: Mesh,
    /// m and rad, in the world: where it is now.
    x: f64,
    y: f64,
    z: f64,
    yaw: f64,
    /// m/s and rad/s, in the world.
    velocity: Velocity,
}

/// A placed model's steady motion, in the world frame (Gazebo's SetEntityState twist).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Velocity {
    /// m/s, east and north.
    pub x: f64,
    /// m/s, north.
    pub y: f64,
    /// rad/s, turning left.
    pub yaw_rate: f64,
}

impl Velocity {
    /// Whether it moves at all.
    #[must_use]
    pub fn is_still(&self) -> bool {
        self.x == 0.0 && self.y == 0.0 && self.yaw_rate == 0.0
    }
}

impl Simulator {
    /// A simulator with the car at (x, y, yaw) in the world.
    #[must_use]
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
            // Forked last: the other sensors' noise stays as it was.
            wheel_noise: seed.fork(),
            drift: [0.0; 3],
            movers: Vec::new(),
            imu_last: None,
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

    /// ns since the simulator started.
    #[must_use]
    pub fn time_ns(&self) -> u64 {
        self.time_ns
    }

    /// Where the car is and what it's doing.
    #[must_use]
    pub fn car(&self) -> CarState {
        self.car.state()
    }

    /// The car model it drives.
    #[must_use]
    pub fn car_model(&self) -> &dyn CarModel {
        self.car.as_ref()
    }

    /// The world, as it is now.
    #[must_use]
    pub fn world(&self) -> &World {
        &self.world
    }

    /// The world, to place or remove an object while running.
    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }

    /// The settings it runs with.
    #[must_use]
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// `base_link` in the world: on the ground under the rear axle, tilted to the wheels
    /// (flat at z = 0 without `ground_step`).
    #[must_use]
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

    /// Put a model in the world at `pose` (x, y, z and yaw used), moving at `velocity` from
    /// now on (still: it stands there). Replaces any model of the same name.
    pub fn place_model(&mut self, name: &str, mesh: Mesh, pose: &Pose, velocity: Velocity) {
        self.movers.retain(|m| m.name != name);
        self.world.scene.place(name, &mesh, pose);
        if !velocity.is_still() {
            let at = pose.position;
            let f = pose.rotation.apply(Vec3::new(1.0, 0.0, 0.0));
            self.movers.push(Mover {
                name: name.to_string(),
                mesh,
                x: at.x,
                y: at.y,
                z: at.z,
                yaw: f.y.atan2(f.x),
                velocity,
            });
        }
    }

    /// Take a placed model away; false if there was none of that name.
    pub fn remove_model(&mut self, name: &str) -> bool {
        self.movers.retain(|m| m.name != name);
        self.world.scene.remove(name)
    }

    /// Where a moving model is now (x, y, yaw in the world), if it moves.
    #[must_use]
    pub fn mover(&self, name: &str) -> Option<(f64, f64, f64)> {
        self.movers
            .iter()
            .find(|m| m.name == name)
            .map(|m| (m.x, m.y, m.yaw))
    }

    /// Move each moving model on by dt.
    fn move_models(&mut self, dt: f64) {
        for m in &mut self.movers {
            m.x += m.velocity.x * dt;
            m.y += m.velocity.y * dt;
            m.yaw = crate::math::wrap(m.yaw + m.velocity.yaw_rate * dt);
            let pose = Pose::new(Vec3::new(m.x, m.y, m.z), Rot3::from_yaw(m.yaw));
            self.world.scene.place(&m.name, &m.mesh, &pose);
        }
    }

    /// Put the car at (x, y, yaw), standing still, its actuators at rest (Gazebo's
    /// `set_pose`; the node's SetEntityState). The truth and wheel odometry jump with it,
    /// as in Gazebo; on a ground with steps the wheels settle again.
    pub fn place_car(&mut self, x: f64, y: f64, yaw: f64) {
        self.car.reset(x, y, yaw);
        self.settle();
        // A jump, not a motion: the IMU starts afresh.
        self.imu_last = None;
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
        self.move_models(dt);
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
        // What a ray hits moving (a person walking) gives the radar its own speed (TODO 194).
        let movers = &self.movers;
        let motion = |name: &str| {
            movers
                .iter()
                .find(|m| m.name == name)
                .map(|m| Vec3::new(m.velocity.x, m.velocity.y, 0.0))
        };
        if let Some(spec) = self.settings.lidar {
            if self.due(spec.rate) {
                let pose = base.then(&self.settings.mounts.lidar);
                out.push(Output::Lidar(Scan::cast_moving(
                    &spec,
                    &self.world.scene,
                    &pose,
                    &mut self.lidar_noise,
                    threads,
                    &motion,
                )));
            }
        }
        if let Some(spec) = self.settings.radar {
            if self.due(spec.rate) {
                let pose = base.then(&self.settings.mounts.radar);
                out.push(Output::Radar(Scan::cast_moving(
                    &spec,
                    &self.world.scene,
                    &pose,
                    &mut self.radar_noise,
                    threads,
                    &motion,
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
        if self.settings.noise.wheels_noisy() {
            self.read_wheels(s, dt);
        }
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

    /// The wheels' odometry from noisy, slipping wheels: the truth's plus a drift that
    /// grows from the gap between what the wheels read and how the car moved.
    fn read_wheels(&mut self, s: &CarState, dt: f64) {
        let n = self.settings.noise;
        // A wheel that doesn't turn reads nothing: no noise standing still.
        let turning = s.speed.abs() > 0.01;
        let speed = if turning {
            s.speed * (1.0 + n.wheel_slip) + self.wheel_noise.gauss(n.wheel_speed)
        } else {
            0.0
        };
        // Ackermann odometry turns by the wheels' speed and the steering: the same error.
        let yaw_rate = if turning {
            s.yaw_rate * speed / s.speed
        } else {
            s.yaw_rate
        };
        let true_yaw = self.odom.yaw;
        let read_yaw = true_yaw + self.drift[2];
        self.drift[0] += (speed * read_yaw.cos() - s.speed * true_yaw.cos()) * dt;
        self.drift[1] += (speed * read_yaw.sin() - s.speed * true_yaw.sin()) * dt;
        self.drift[2] += (yaw_rate - s.yaw_rate) * dt;
        self.odom.x += self.drift[0];
        self.odom.y += self.drift[1];
        self.odom.yaw = crate::math::wrap(self.odom.yaw + self.drift[2]);
        self.odom.speed = speed;
        self.odom.yaw_rate = yaw_rate;
    }

    /// The IMU: what a real one on the car feels (S3, 18 Nov). Its turn rates on all three
    /// axes come from how `base_link` turned since the last reading (slopes too); its
    /// acceleration from how the speed and the turn rate changed since then, the turn's
    /// pull at the sensor's place, and gravity as the tilted car feels it. Averaging over
    /// the time between readings is what a real IMU's low-pass filter does; an
    /// instant's acceleration would alias the car model's 1 ms steps (it catches each new
    /// command in a step or two at up to 10 m/s²).
    fn imu(&mut self) -> ImuReading {
        let s = self.car.state();
        let mount = self.settings.mounts.imu;
        let base = self.base_link();
        let world = base.then(&mount);
        let into_imu = mount.rotation.transpose();
        let now = ImuMemory {
            time_ns: self.time_ns,
            speed: s.speed,
            yaw_rate: s.yaw_rate,
            rotation: base.rotation,
        };
        let motion = match self.imu_last.replace(now) {
            Some(before) if now.time_ns > before.time_ns => {
                Motion::between(&before, &now, (now.time_ns - before.time_ns) as f64 * 1e-9)
            }
            _ => Motion {
                accel: s.accel,
                yaw_accel: 0.0,
                rate: Vec3::new(0.0, 0.0, s.yaw_rate),
            },
        };
        // Gravity's pull, as the tilted car feels it (straight up on flat ground).
        let up = base
            .rotation
            .transpose()
            .apply(Vec3::new(0.0, 0.0, GRAVITY));
        let accel = sensor_acceleration(
            motion.accel,
            s.speed,
            s.yaw_rate,
            motion.yaw_accel,
            mount.position,
        ) + up;
        let rate = into_imu.apply(motion.rate);
        let accel = into_imu.apply(accel);
        let n = self.settings.noise;
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

/// The car's motion between two IMU readings, in `base_link`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Motion {
    /// m/s², forward: the mean over the time between.
    accel: f64,
    /// rad/s².
    yaw_accel: f64,
    /// rad/s, about x, y and z: the mean turn.
    rate: Vec3,
}

impl Motion {
    fn between(before: &ImuMemory, now: &ImuMemory, dt: f64) -> Motion {
        Motion {
            accel: (now.speed - before.speed) / dt,
            yaw_accel: (now.yaw_rate - before.yaw_rate) / dt,
            rate: body_rate(&before.rotation, &now.rotation, dt),
        }
    }
}

/// rad/s about the body's own x, y and z: the turn from `before` to `now` (both the body's
/// rotation in the world) over `dt` s. A gyro's reading.
#[must_use]
pub fn body_rate(before: &Rot3, now: &Rot3, dt: f64) -> Vec3 {
    // The turn in the body's frame, as an axis and an angle.
    let r = before.transpose().then(now).m;
    let skew = Vec3::new(r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]) * 0.5;
    let sin = skew.length();
    let cos = ((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0);
    let angle = sin.atan2(cos);
    let scale = if sin > 1e-12 { angle / sin } else { 1.0 };
    skew * (scale / dt)
}

/// m/s² at a point `at` (m, in `base_link`) of a car on the ground whose rear axle speeds
/// up at `accel` while it drives at `speed` and turns at `yaw_rate`, itself changing at
/// `yaw_accel`: a rigid body's a + α × r + ω × (ω × r), without gravity.
#[must_use]
pub fn sensor_acceleration(
    accel: f64,
    speed: f64,
    yaw_rate: f64,
    yaw_accel: f64,
    at: Vec3,
) -> Vec3 {
    let w2 = yaw_rate * yaw_rate;
    Vec3::new(
        accel - yaw_accel * at.y - w2 * at.x,
        speed * yaw_rate + yaw_accel * at.x - w2 * at.y,
        0.0,
    )
}

/// Each front wheel's angle for the centre wheel's `steering` (Ackermann), left then right:
/// both point at the same turning centre.
#[must_use]
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
            ..SensorNoise::default()
        };
        let mut sim = Simulator::new(s, World::flat(400.0), car, (0.0, 0.0, 0.0));
        let imu = sim.imu();
        assert!((imu.linear_acceleration.z - GRAVITY).abs() < 1e-12);
        let fix = sim.navsat();
        assert!((fix.latitude - 45.386).abs() < 1e-9);
        assert!((fix.altitude - 71.4).abs() < 1e-6);
    }

    fn quiet() -> Settings {
        let mut s = settings();
        s.noise = SensorNoise {
            gyro: 0.0,
            accel: 0.0,
            gps_horizontal_deg: 0.0,
            gps_vertical: 0.0,
            ..SensorNoise::default()
        };
        s.lidar = None;
        s
    }

    /// Run `secs` s, the command from `command(t)` changed every 20 ms (sim_adapter's
    /// 50 Hz); return every IMU reading with its time (s).
    fn imu_readings(
        sim: &mut Simulator,
        secs: f64,
        command: impl Fn(f64) -> Command,
    ) -> Vec<(f64, ImuReading)> {
        let mut out = Vec::new();
        let mut now = command(0.0);
        for k in 0..(secs * 1000.0) as usize {
            if k % 20 == 0 {
                now = command(k as f64 * 1e-3);
            }
            for o in sim.step(&now) {
                if let Output::Imu(r) = o {
                    out.push((sim.time_ns() as f64 * 1e-9, r));
                }
            }
        }
        out
    }

    #[test]
    fn speeding_up_the_imu_reads_the_cars_acceleration_not_the_steps() {
        // sim_adapter ramps 1 m/s² at 50 Hz; the follower catches each new speed within
        // 2 ms at 10 m/s². A real IMU reads 1 m/s² all along (S3).
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut sim = Simulator::new(quiet(), World::flat(400.0), car, (0.0, 0.0, 0.0));
        let ramp = |t: f64| Command::Twist {
            speed: t.min(2.0),
            yaw_rate: 0.0,
        };
        let readings = imu_readings(&mut sim, 1.5, ramp);
        for (t, r) in readings.iter().filter(|(t, _)| (0.1..1.4).contains(t)) {
            let a = r.linear_acceleration;
            assert!((a.x - 1.0).abs() < 1e-6, "at {t:.2} s: {a:?}");
            assert!(a.y.abs() < 1e-9 && (a.z - GRAVITY).abs() < 1e-9, "{a:?}");
        }
    }

    #[test]
    fn in_a_steady_turn_the_imu_feels_the_pull_and_the_turn() {
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut sim = Simulator::new(quiet(), World::flat(400.0), car, (0.0, 0.0, 0.0));
        let (v, r) = (2.0, 0.4);
        let turn = |_| Command::Twist {
            speed: v,
            yaw_rate: r,
        };
        let readings = imu_readings(&mut sim, 3.0, turn);
        let (_, last) = readings.last().expect("readings");
        let w = last.angular_velocity;
        let a = last.linear_acceleration;
        let x = 0.49; // the IMU's place ahead of the rear axle (settings())
        assert!(
            (w.z - r).abs() < 1e-3 && w.x.abs() < 1e-9 && w.y.abs() < 1e-9,
            "{w:?}"
        );
        assert!(
            (a.y - v * r).abs() < 1e-2,
            "the pull toward the centre: {a:?}"
        );
        assert!(
            (a.x + r * r * x).abs() < 1e-2,
            "ahead of the axle, pulled in: {a:?}"
        );
    }

    #[test]
    fn a_gyro_reads_each_axis_of_a_turn() {
        let dt = 0.02;
        let level = Rot3::from_yaw(1.0);
        let pitched = Rot3::from_rpy(0.0, 0.01, 1.0);
        let w = body_rate(&level, &pitched, dt);
        assert!(
            (w.y - 0.5).abs() < 1e-3 && w.x.abs() < 1e-3 && w.z.abs() < 1e-3,
            "{w:?}"
        );
        let turned = Rot3::from_yaw(1.0 + 0.008);
        let w = body_rate(&level, &turned, dt);
        assert!((w.z - 0.4).abs() < 1e-9 && w.x.abs() < 1e-12, "{w:?}");
        let w = body_rate(&level, &level, dt);
        assert_eq!(w, Vec3::new(0.0, 0.0, 0.0));
        // A big turn (nearly half a circle) is still read by its angle.
        let far = Rot3::from_yaw(1.0 + 3.0);
        assert!((body_rate(&level, &far, 1.0).z - 3.0).abs() < 1e-9);
    }

    #[test]
    fn the_acceleration_at_the_sensor_is_a_rigid_bodys() {
        let at = Vec3::new(0.5, 0.2, 1.4);
        let a = sensor_acceleration(1.0, 2.0, 0.0, 0.0, at);
        assert_eq!(a, Vec3::new(1.0, 0.0, 0.0));
        // Turning: the pull toward the centre, v r, and the point's own circle.
        let a = sensor_acceleration(0.0, 2.0, 0.5, 0.0, at);
        assert!((a.x + 0.25 * 0.5).abs() < 1e-12 && (a.y - (1.0 - 0.25 * 0.2)).abs() < 1e-12);
        // Turning faster: a point ahead of the axle swings left.
        let a = sensor_acceleration(0.0, 0.0, 0.0, 1.0, at);
        assert!((a.y - 0.5).abs() < 1e-12 && (a.x + 0.2).abs() < 1e-12);
    }

    #[test]
    fn a_placed_car_gives_the_imu_no_jolt() {
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut sim = Simulator::new(quiet(), World::flat(400.0), car, (0.0, 0.0, 0.0));
        let go = |_| Command::Twist {
            speed: 1.0,
            yaw_rate: 0.2,
        };
        imu_readings(&mut sim, 2.0, go);
        sim.place_car(30.0, 5.0, -2.0);
        let still = |_| Command::Twist {
            speed: 0.0,
            yaw_rate: 0.0,
        };
        for (t, r) in imu_readings(&mut sim, 0.5, still) {
            let (a, w) = (r.linear_acceleration, r.angular_velocity);
            assert!(
                a.x.abs() < 1e-9 && w.z.abs() < 1e-9,
                "at {t:.2} s: {a:?} {w:?}"
            );
        }
    }

    #[test]
    fn rolling_onto_a_mat_the_gyro_feels_the_pitch() {
        let mut s = quiet();
        s.body = None;
        s.ground_step = Some(0.1);
        let mut world = World::flat(400.0);
        world.scene.place(
            "mat",
            &crate::scene::Mesh::cuboid(Vec3::new(4.0, 10.0, 0.05)),
            &Pose::new(Vec3::new(5.0, 0.0, 0.025), Rot3::IDENTITY),
        );
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut sim = Simulator::new(s, world, car, (0.0, 0.0, 0.0));
        sim.start();
        let go = |_| Command::Twist {
            speed: 1.0,
            yaw_rate: 0.0,
        };
        let readings = imu_readings(&mut sim, 6.0, go);
        let most = readings
            .iter()
            .map(|(_, r)| r.angular_velocity.y.abs())
            .fold(0.0, f64::max);
        // The front wheels step up 5 cm, then the rear: the nose goes up, then down.
        assert!(most > 0.1, "the pitch rate on the mat's edge: {most}");
        let (_, end) = readings.last().expect("readings");
        assert!(
            end.angular_velocity.y.abs() < 1e-9,
            "level again on the mat"
        );
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

    /// Drive a curve for `secs` s; return the truth's distance and the last odometry.
    fn drive_with_wheels(noise: SensorNoise, seed: u64, secs: usize) -> (f64, Odometry) {
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut s = settings();
        s.noise = noise;
        s.seed = seed;
        let mut sim = Simulator::new(s, World::flat(400.0), car, (0.0, 0.0, 0.0));
        let go = Command::Twist {
            speed: 1.0,
            yaw_rate: 0.1,
        };
        let mut last = None;
        let mut travelled = 0.0;
        for _ in 0..secs * 1000 {
            for o in sim.step(&go) {
                if let Output::Odom(odom) = o {
                    last = Some(odom);
                }
            }
            travelled += sim.car().speed.abs() * 0.001;
        }
        (travelled, last.expect("odometry"))
    }

    #[test]
    fn a_walker_crosses_and_the_world_meets_it_where_it_is() {
        use crate::scene::{Mesh, Ray};
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut sim = Simulator::new(settings(), World::flat(400.0), car, (0.0, 0.0, 0.0));
        let post = Mesh::cuboid(Vec3::new(0.5, 0.5, 1.8));
        let start = Pose::new(Vec3::new(10.0, -3.0, 0.9), Rot3::from_yaw(0.0));
        let walk = Velocity {
            x: 0.0,
            y: 1.5,
            yaw_rate: 0.0,
        };
        sim.place_model("walker", post, &start, walk);
        let ahead = Ray {
            origin: Vec3::new(0.0, 0.0, 1.0),
            direction: Vec3::new(1.0, 0.0, 0.0),
        };
        assert!(
            sim.world().scene.cast(&ahead, 50.0).is_none(),
            "not in the lane yet"
        );
        let still = Command::Twist {
            speed: 0.0,
            yaw_rate: 0.0,
        };
        for _ in 0..2000 {
            sim.step(&still);
        }
        let (x, y, _) = sim.mover("walker").expect("it moves");
        assert!(
            (x - 10.0).abs() < 1e-9 && (y - 0.0).abs() < 1e-6,
            "at ({x}, {y})"
        );
        let hit = sim
            .world()
            .scene
            .cast(&ahead, 50.0)
            .expect("in the lane now");
        assert!((hit - 9.75).abs() < 1e-6, "its near face: {hit}");
        assert!(sim.remove_model("walker"));
        assert!(sim.mover("walker").is_none());
        assert!(sim.world().scene.cast(&ahead, 50.0).is_none());
    }

    #[test]
    fn a_still_model_is_placed_once_and_never_moves() {
        use crate::scene::Mesh;
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut sim = Simulator::new(settings(), World::flat(400.0), car, (0.0, 0.0, 0.0));
        let at = Pose::new(Vec3::new(5.0, 0.0, 0.5), Rot3::from_yaw(0.0));
        sim.place_model(
            "box",
            Mesh::cuboid(Vec3::new(1.0, 1.0, 1.0)),
            &at,
            Velocity::default(),
        );
        assert!(sim.mover("box").is_none());
    }

    #[test]
    fn slipping_wheels_read_too_far() {
        let slip = SensorNoise {
            wheel_slip: 0.05,
            ..SensorNoise::default()
        };
        let (_, truth) = drive_with_wheels(SensorNoise::default(), 1, 10);
        let (travelled, read) = drive_with_wheels(slip, 1, 10);
        assert!(
            read.speed > 1.04 && read.speed < 1.06,
            "speed {}",
            read.speed
        );
        let (dx, dy) = (read.x - truth.x, read.y - truth.y);
        let gap = (dx * dx + dy * dy).sqrt();
        // About 5 % of the way, give or take the turn's own error.
        assert!(
            gap > 0.03 * travelled && gap < 0.08 * travelled,
            "drift {gap} m"
        );
    }

    #[test]
    fn noisy_wheels_drift_the_same_way_for_the_same_seed() {
        let noisy = SensorNoise {
            wheel_speed: 0.1,
            ..SensorNoise::default()
        };
        let (_, truth) = drive_with_wheels(SensorNoise::default(), 1, 10);
        let (_, a) = drive_with_wheels(noisy, 7, 10);
        let (_, b) = drive_with_wheels(noisy, 7, 10);
        let (_, c) = drive_with_wheels(noisy, 8, 10);
        assert_eq!(a, b);
        assert_ne!(a, c);
        assert!(a != truth, "noise on, the odometry is not the truth");
    }

    #[test]
    fn wheels_standing_still_read_nothing() {
        let noisy = SensorNoise {
            wheel_speed: 0.1,
            wheel_slip: 0.05,
            ..SensorNoise::default()
        };
        let car = Box::new(TwistFollower::new(1.561, 0.5));
        let mut s = settings();
        s.noise = noisy;
        let mut sim = Simulator::new(s, World::flat(400.0), car, (0.0, 0.0, 0.0));
        let still = Command::Twist {
            speed: 0.0,
            yaw_rate: 0.0,
        };
        for _ in 0..1000 {
            for o in sim.step(&still) {
                if let Output::Odom(odom) = o {
                    assert_eq!((odom.x, odom.y, odom.speed), (0.0, 0.0, 0.0));
                }
            }
        }
    }

    #[test]
    fn ackermann_inner_wheel_turns_more() {
        let [left, right] = wheel_angles(0.3, 1.561, 0.75);
        assert!(left > 0.3 && right < 0.3 && right > 0.0);
    }
}
