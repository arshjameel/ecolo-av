//! simulator: Ravensim, our own simulator. It took the place of `gz sim` and its
//! `ros_gz_bridge`s (Gazebo went on 16 Nov).
//!
//! It publishes what Gazebo's bridges publish, with the same names, types, frames, rates
//! and QoS, so `sim_adapter`, `sim_imu`, `sim_gps`, `sim_radar` and the whole
//! stack above them run unchanged (docs/own-simulator.md):
//!
//! Subscribes  vehicle/sim/cmd_vel          geometry_msgs/Twist: the speed and turn rate
//!                                     sim_adapter's actuator model gives (Gazebo's
//!                                     AckermannSteering system's input), kept until the next
//! Publishes   /clock                  rosgraph_msgs/Clock, clock_hz (1000, as Gazebo), the simulated time
//!             vehicle/sim/odom             nav_msgs/Odometry, 50 Hz, odom -> base_link: the
//!                                     wheels' odometry from where the car started
//!             vehicle/sim/truth       nav_msgs/Odometry, 50 Hz, map -> base_link: where the
//!                                     car really is (for checks only)
//!             vehicle/sim/imu              sensor_msgs/Imu, 50 Hz, imu_link: orientation, turn
//!                                     rates and specific force (gravity +9.8 up), each
//!                                     averaged since the last reading as a real IMU's
//!                                     (sim-core sim.rs), with Gazebo's old noise
//!             vehicle/sim/navsat           sensor_msgs/NavSatFix, 5 Hz, gps_link: the antenna on
//!                                     the world's datum, with its noise
//!             joint_states            sensor_msgs/JointState, 100 Hz: the four wheels' spin
//!                                     and the two steering joints (robot_state_publisher's TF)
//!             sensors/lidar/points    sensor_msgs/PointCloud2, 10 Hz, lidar_link, reliable:
//!                                     1800 x 16 rays, organised, packed as Gazebo's (lidar)
//!             vehicle/sim/radar/points     sensor_msgs/PointCloud2, 10 Hz, radar_link, best
//!                                     effort: the radar stand-in's 61 x 3 rays (radar).
//!                                     Both clouds add `speed` (float32, at 20): what each
//!                                     ray hit moves along it, for the radar's Doppler
//! Subscribes  vehicle/sim/cameras/free_until  rosgraph_msgs/Clock: with camera_lockstep, the
//!                                     cameras' next picture's sim time; at a truth stamped
//!                                     that late the clock holds until they say a later one
//!                                     (sim-core lockstep.rs: software rendering, no GPU)
//!             vehicle/sim/entities    visualization_msgs/MarkerArray, latched: every model
//!                                     placed while running (obstacle, spawn_entity), each a
//!                                     MESH_RESOURCE marker: ns its name, mesh_resource its
//!                                     SDF file (file://), its pose in the map. For the
//!                                     cameras (sim_cameras), which draw them
//! Serves      vehicle/sim/spawn_entity  simulation_interfaces/SpawnEntity: put a model (an SDF
//!                                     file, or model://name) in the world, at initial_pose
//!                                     (map frame; x, y and yaw), as Gazebo's create service
//!             vehicle/sim/delete_entity  simulation_interfaces/DeleteEntity: take it away
//!             vehicle/sim/set_entity_state  simulation_interfaces/SetEntityState: entity
//!                                     `aav`, the car, put at state.pose (map frame; x, y,
//!                                     yaw), standing still, as Gazebo's set_pose (a twist
//!                                     is refused). A placed model: put at state.pose,
//!                                     then moving at state.twist (map frame: linear x,
//!                                     y, angular z), as a person walking (TODO 170)
//!
//! Parameters  world (an SDF file), models (folders for model://, ':'-separated),
//!             start_x, start_y, start_yaw, wheelbase, track, wheel_radius, max_steering,
//!             imu_xyz, imu_rpy, gps_xyz, lidar_xyz, lidar_rpy, radar_xyz, radar_rpy
//!             (vehicle.yaml's), lidar, radar (true: that sensor runs), obstacle (the bench's
//!             box at obstacle_x, obstacle_y, from obstacle_model), body_length,
//!             body_width, rear_overhang, body_height (the car stops at what its outline
//!             meets, as Gazebo's; body_length 0: it drives through), ground_step (m: the
//!             highest step a wheel rolls up, 0.1; the car rides the world's ground, tilts on
//!             slopes, stops at a higher kerb; 0: flat ground), seed, wheel_noise (m/s on
//!             each wheel reading, 0: off), wheel_slip (a fraction, 0: off), step (s),
//!             clock_hz, real_time_factor (1.0; 0: as fast as it can), threads, camera_lockstep
//!             (false; true: wait for the cameras, above), camera_timeout (s, wall: 10)
//!
//! The simulated time runs in a thread of its own, paced against the wall clock; the
//! executor only takes the commands. Cameras plug in through sim-core's SensorPlugin.

use std::{
    collections::BTreeMap,
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

use anyhow::{anyhow, Context as _, Result};
use rclrs::*;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::geometry_msgs::msg::{
    Point, Pose as PoseMsg, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
};
use ros_env::nav_msgs::msg::Odometry as OdometryMsg;
use ros_env::rosgraph_msgs::msg::Clock;
use ros_env::sensor_msgs::msg::{
    Imu, JointState, NavSatFix, NavSatStatus, PointCloud2, PointField,
};
use ros_env::simulation_interfaces::msg::Result as SimResult;
use ros_env::simulation_interfaces::srv::{
    DeleteEntity, DeleteEntity_Request, DeleteEntity_Response, SetEntityState,
    SetEntityState_Request, SetEntityState_Response, SpawnEntity, SpawnEntity_Request,
    SpawnEntity_Response,
};
use ros_env::std_msgs::msg::Header;
use ros_env::visualization_msgs::msg::{Marker, MarkerArray};
use sim_core::car::{Command, TwistFollower};
use sim_core::lidar::{LidarSpec, Scan, FIELDS, POINT_STEP};
use sim_core::lockstep::{CameraGate, Waited};
use sim_core::math::{Pose, Rot3};
use sim_core::node::{self, Pace, Placement, Refusal, SetState, Wait, CAR};
use sim_core::scene::Mesh;
use sim_core::sim::{
    Body, Geometry, ImuReading, Joints, Mounts, Odometry, Output, Rates, SensorNoise, Settings,
    Simulator, Velocity,
};
use sim_core::world::{Surfaces, World};

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("simulator")?;
    let params = Params::declare(&node)?;

    let world_file = PathBuf::from(params.world.get().as_ref());
    let models: Vec<PathBuf> = params
        .models
        .get()
        .split(':')
        .filter(|s| !s.is_empty())
        .map(PathBuf::from)
        .collect();
    let started = Instant::now();
    let mut world = World::load(&world_file, &models, Surfaces::Visual)
        .with_context(|| format!("world:={}", world_file.display()))?;
    for w in &world.warnings {
        log_warn!(node.logger(), "{}: {w}", world_file.display());
    }
    if params.obstacle.get() {
        let model = PathBuf::from(params.obstacle_model.get().as_ref());
        let mesh = World::model_mesh(&model, &models, Surfaces::Visual)?;
        let at = Pose::from_xyz_rpy(
            [params.obstacle_x.get(), params.obstacle_y.get(), 0.0],
            [0.0; 3],
        );
        world.scene.place("obstacle", &mesh, &at);
    }
    log_info!(
        node.logger(),
        "world {}: {} models, {} triangles, read in {:.2} s",
        world_file.display(),
        world.models.len(),
        world.scene.triangle_count(),
        started.elapsed().as_secs_f64()
    );

    let wheel_noise = node::wheel_noise(params.wheel_noise.get())?;
    let wheel_slip = node::wheel_slip(params.wheel_slip.get())?;
    let rtf = node::real_time_factor(params.real_time_factor.get())?;
    let settings = Settings {
        step_ns: node::step_ns(params.step.get())?,
        geometry: Geometry {
            wheelbase: params.wheelbase.get(),
            track: params.track.get(),
            wheel_radius: params.wheel_radius.get(),
        },
        mounts: Mounts {
            imu: node::mount(&params.imu_xyz.get(), &params.imu_rpy.get())?,
            gps: node::mount(&params.gps_xyz.get(), &[0.0; 3])?,
            lidar: node::mount(&params.lidar_xyz.get(), &params.lidar_rpy.get())?,
            radar: node::mount(&params.radar_xyz.get(), &params.radar_rpy.get())?,
        },
        rates: Rates {
            clock: params.clock_hz.get(),
            ..Rates::default()
        },
        noise: SensorNoise {
            wheel_speed: wheel_noise,
            wheel_slip,
            ..SensorNoise::default()
        },
        lidar: params.lidar.get().then_some(LidarSpec::ROOF),
        radar: params.radar.get().then_some(LidarSpec::RADAR),
        seed: params.seed.get() as u64,
        threads: params.threads.get().max(1) as usize,
        world_file: Some(world_file.clone()),
        body: (params.body_length.get() > 0.0).then(|| Body {
            length: params.body_length.get(),
            width: params.body_width.get(),
            rear_overhang: params.rear_overhang.get(),
            height: params.body_height.get(),
        }),
        ground_step: (params.ground_step.get() > 0.0).then(|| params.ground_step.get()),
    };
    let car = Box::new(TwistFollower::new(
        params.wheelbase.get(),
        params.max_steering.get(),
    ));
    let start = (
        params.start_x.get(),
        params.start_y.get(),
        params.start_yaw.get(),
    );
    let sim = Simulator::new(settings, world, car, start);

    let out = Outputs {
        clock: node.create_publisher("/clock")?,
        odom: node.create_publisher("vehicle/sim/odom")?,
        truth: node.create_publisher("vehicle/sim/truth")?,
        imu: node.create_publisher("vehicle/sim/imu")?,
        navsat: node.create_publisher("vehicle/sim/navsat")?,
        joints: node.create_publisher("joint_states")?,
        // Only the sensors this run has: a publisher with nothing behind it still shows in
        // the graph, and the sensor mux would doubt a lidar nobody announces.
        lidar: if params.lidar.get() {
            Some(node.create_publisher("sensors/lidar/points")?)
        } else {
            None
        },
        radar: if params.radar.get() {
            Some(node.create_publisher("vehicle/sim/radar/points".best_effort().keep_last(5))?)
        } else {
            None
        },
        logger: node.logger().clone(),
    };

    // The last command, as Gazebo's AckermannSteering keeps it, and the objects to place.
    let shared = Arc::new(Mutex::new(Shared {
        command: Command::Twist {
            speed: 0.0,
            yaw_rate: 0.0,
        },
        changes: Vec::new(),
        placed: BTreeMap::new(),
        entities: node.create_publisher("vehicle/sim/entities".reliable().transient_local())?,
    }));
    if params.obstacle.get() {
        let at = Pose::from_xyz_rpy(
            [params.obstacle_x.get(), params.obstacle_y.get(), 0.0],
            [0.0; 3],
        );
        let file = PathBuf::from(params.obstacle_model.get().as_ref());
        if let Ok(mut s) = shared.lock() {
            s.placed.insert("obstacle".into(), (file, at));
            s.announce();
        }
    } else if let Ok(s) = shared.lock() {
        s.announce();
    }
    let latest = Arc::clone(&shared);
    let _cmd = node.create_subscription("vehicle/sim/cmd_vel", move |msg: Twist| {
        if let Ok(mut s) = latest.lock() {
            s.command = Command::Twist {
                speed: msg.linear.x,
                yaw_rate: msg.angular.z,
            };
        }
    })?;
    let spawning = Arc::clone(&shared);
    let search = models.clone();
    let _spawn = node.create_service::<SpawnEntity, _>(
        "vehicle/sim/spawn_entity",
        move |request: SpawnEntity_Request| spawn(&spawning, &search, &request),
    )?;
    let placing = Arc::clone(&shared);
    let moving_models = models.clone();
    let _set_state = node.create_service::<SetEntityState, _>(
        "vehicle/sim/set_entity_state",
        move |request: SetEntityState_Request| set_state(&placing, &moving_models, &request),
    )?;
    let deleting = Arc::clone(&shared);
    let _delete = node.create_service::<DeleteEntity, _>(
        "vehicle/sim/delete_entity",
        move |request: DeleteEntity_Request| delete(&deleting, &request),
    )?;

    // Lockstep with the cameras (software rendering): the clock waits for their pictures.
    let gate = params.camera_lockstep.get().then(|| {
        Arc::new(CameraGate::new(Duration::from_secs_f64(
            params.camera_timeout.get(),
        )))
    });
    if gate.is_some() {
        log_info!(
            node.logger(),
            "camera lockstep on: the clock waits for each picture (at most {:.0} s)",
            params.camera_timeout.get()
        );
    }
    let _free_until = match &gate {
        Some(gate) => {
            let gate = Arc::clone(gate);
            Some(node.create_subscription(
                "vehicle/sim/cameras/free_until",
                move |msg: Clock| {
                    gate.free_until(node::nanoseconds(msg.clock.sec, msg.clock.nanosec));
                },
            )?)
        }
        None => None,
    };

    let stop = Arc::new(AtomicBool::new(false));
    let logger = node.logger().clone();
    let running = Arc::clone(&stop);
    let looping = std::thread::Builder::new()
        .name("simulator".into())
        .spawn(move || run(sim, out, shared, running, rtf, gate, logger))?;

    let commands = Arc::clone(executor.commands());
    let halting = Arc::clone(&stop);
    ctrlc::set_handler(move || {
        halting.store(true, Ordering::Relaxed);
        commands.halt_spinning();
    })?;
    executor.spin(SpinOptions::default()).first_error()?;
    stop.store(true, Ordering::Relaxed);
    looping
        .join()
        .map_err(|_| anyhow!("the simulator's loop panicked"))?
}

/// The loop: step, publish what's due, and keep the simulated time with the wall's.
fn run(
    mut sim: Simulator,
    out: Outputs,
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    rtf: f64,
    gate: Option<Arc<CameraGate>>,
    logger: Logger,
) -> Result<()> {
    let first = sim.start();
    out.publish(&sim, first)?;
    let wall = Instant::now();
    let mut pace = Pace::new(rtf, wall.elapsed(), sim.time_ns());
    let mut report = Instant::now();
    let mut report_ns = 0u64;
    let mut held = (0u32, Duration::ZERO); // lockstep: waits, and their wall time
    while !stop.load(Ordering::Relaxed) {
        let (now_command, changes) = {
            let mut s = shared
                .lock()
                .map_err(|_| anyhow!("the command's lock is poisoned"))?;
            (s.command, std::mem::take(&mut s.changes))
        };
        for change in changes {
            match change {
                Change::Place {
                    name,
                    mesh,
                    pose,
                    velocity,
                } => sim.place_model(&name, mesh, &pose, velocity),
                Change::PlaceCar(x, y, yaw) => sim.place_car(x, y, yaw),
                Change::Remove(name) => {
                    sim.remove_model(&name);
                }
            }
        }
        let outputs = sim.step(&now_command);
        let pose_out = outputs.iter().any(|o| matches!(o, Output::Truth(_)));
        out.publish(&sim, outputs)?;
        if let (Some(gate), true) = (&gate, pose_out) {
            // The cameras draw from the truth: hold the clock until they're past it.
            match gate.wait_past(sim.time_ns()) {
                Waited::Free => {}
                Waited::Answered(wait) => held = (held.0 + 1, held.1 + wait),
                Waited::TimedOut => log_warn!(
                    &logger,
                    "camera lockstep: no word from the cameras; running free until they speak"
                ),
            }
        }
        // Too far behind, the pace runs on from here, slower than asked (sim-core node.rs).
        if let Wait::Sleep(ahead) = pace.after_step(sim.time_ns(), wall.elapsed()) {
            std::thread::sleep(ahead);
        }
        if report.elapsed() >= Duration::from_secs(30) {
            let simulated = (sim.time_ns() - report_ns) as f64 * 1e-9;
            let lockstep = if gate.is_some() {
                format!(
                    "; camera lockstep held the clock {} times, {:.1} s",
                    held.0,
                    held.1.as_secs_f64()
                )
            } else {
                String::new()
            };
            log_info!(
                &logger,
                "real-time factor {:.2} over the last {:.0} s{lockstep}",
                simulated / report.elapsed().as_secs_f64(),
                report.elapsed().as_secs_f64()
            );
            held = (0, Duration::ZERO);
            report = Instant::now();
            report_ns = sim.time_ns();
        }
    }
    Ok(())
}

/// What the services and the command's subscription hand to the loop.
struct Shared {
    command: Command,
    changes: Vec<Change>,
    /// What is placed so far, by name: its SDF file and pose. A delete says whether there
    /// was one; the cameras hear the lot on vehicle/sim/entities.
    placed: BTreeMap<String, (PathBuf, Pose)>,
    entities: Publisher<MarkerArray>,
}

impl Shared {
    /// Every placed model, latched, for the cameras: a DELETEALL, then one marker each.
    fn announce(&self) {
        let clear = Marker {
            action: Marker::DELETEALL,
            ..Default::default()
        };
        let markers = std::iter::once(clear)
            .chain(
                self.placed
                    .iter()
                    .enumerate()
                    .map(|(i, (name, (file, pose)))| {
                        let [x, y, z, w] = pose.rotation.quaternion();
                        Marker {
                            header: Header {
                                frame_id: "map".into(),
                                ..Default::default()
                            },
                            ns: name.clone(),
                            id: i as i32,
                            type_: Marker::MESH_RESOURCE,
                            action: Marker::ADD,
                            pose: PoseMsg {
                                position: Point {
                                    x: pose.position.x,
                                    y: pose.position.y,
                                    z: pose.position.z,
                                },
                                orientation: Quaternion { x, y, z, w },
                            },
                            mesh_resource: format!("file://{}", file.display()),
                            ..Default::default()
                        }
                    }),
            )
            .collect();
        let _ = self.entities.publish(MarkerArray { markers });
    }
}

/// An object to put in the world, or take away, before the next step.
enum Change {
    /// The car, moved: x, y (m), yaw (rad), in the map.
    PlaceCar(f64, f64, f64),
    Place {
        name: String,
        mesh: Mesh,
        pose: Pose,
        /// Still, or walking on at this velocity (set_entity_state's twist).
        velocity: Velocity,
    },
    Remove(String),
}

fn sim_result(result: u8, error_message: impl Into<String>) -> SimResult {
    SimResult {
        result,
        error_message: error_message.into(),
    }
}

/// SpawnEntity: read the model now (the loop mustn't wait on a file), place it next step.
fn spawn(
    shared: &Mutex<Shared>,
    models: &[PathBuf],
    request: &SpawnEntity_Request,
) -> SpawnEntity_Response {
    let refuse = |code: u8, why: String| SpawnEntity_Response {
        result: sim_result(code, why),
        entity_name: String::new(),
    };
    let file = match node::model_file(&request.uri, models) {
        Ok(file) => file,
        Err(why) => {
            let code = match why {
                Refusal::NoUri => SpawnEntity_Response::NO_RESOURCE,
                _ => SpawnEntity_Response::MISSING_ASSETS,
            };
            return refuse(code, why.to_string());
        }
    };
    let mesh = match World::model_mesh(&file, models, Surfaces::Visual) {
        Ok(mesh) => mesh,
        Err(e) => return refuse(SpawnEntity_Response::RESOURCE_PARSE_ERROR, e.to_string()),
    };
    let name = node::model_name(&request.name, &file);
    let pose = node::spawn_pose(&placement(&request.initial_pose.pose));
    let Ok(mut s) = shared.lock() else {
        let why = "the simulator's lock is poisoned".to_string();
        return refuse(SimResult::RESULT_OPERATION_FAILED, why);
    };
    if s.placed.contains_key(&name) && !request.allow_renaming {
        return refuse(
            SpawnEntity_Response::NAME_NOT_UNIQUE,
            format!("{name} is there already"),
        );
    }
    s.placed.insert(name.clone(), (file, pose));
    s.announce();
    s.changes.push(Change::Place {
        name: name.clone(),
        mesh,
        pose,
        velocity: Velocity::default(),
    });
    SpawnEntity_Response {
        result: sim_result(SimResult::RESULT_OK, ""),
        entity_name: name,
    }
}

/// DeleteEntity: take a placed object away next step.
fn delete(shared: &Mutex<Shared>, request: &DeleteEntity_Request) -> DeleteEntity_Response {
    let result = match shared.lock() {
        Ok(mut s) => {
            if s.placed.remove(&request.entity).is_some() {
                s.announce();
                s.changes.push(Change::Remove(request.entity.clone()));
                sim_result(SimResult::RESULT_OK, "")
            } else {
                sim_result(
                    SimResult::RESULT_NOT_FOUND,
                    format!("nothing called {} was placed", request.entity),
                )
            }
        }
        Err(_) => sim_result(
            SimResult::RESULT_OPERATION_FAILED,
            "the simulator's lock is poisoned",
        ),
    };
    DeleteEntity_Response { result }
}

/// A ROS pose as sim-core's plain one.
fn placement(p: &PoseMsg) -> Placement {
    let q = &p.orientation;
    Placement {
        position: [p.position.x, p.position.y, p.position.z],
        orientation: [q.x, q.y, q.z, q.w],
    }
}

/// SetEntityState: the car at a pose next step, standing still (Gazebo's set_pose); or a
/// placed model at a pose, then moving at the twist (map frame: linear x, y, angular z),
/// as a person walking (TODO 170). The cameras see where it was put. What is accepted:
/// sim-core node.rs `set_state`.
fn set_state(
    shared: &Mutex<Shared>,
    models: &[PathBuf],
    request: &SetEntityState_Request,
) -> SetEntityState_Response {
    let answer = |code: u8, why: String| SetEntityState_Response {
        result: sim_result(code, why),
    };
    let t = &request.state.twist;
    let twist = [
        t.linear.x,
        t.linear.y,
        t.linear.z,
        t.angular.x,
        t.angular.y,
        t.angular.z,
    ];
    let asked = node::set_state(
        &request.entity,
        &request.state.header.frame_id,
        &placement(&request.state.pose),
        &twist,
    );
    let change = match asked {
        Ok(change) => change,
        Err(why) => {
            let code = match why {
                Refusal::BadPose | Refusal::NotFinite => SetEntityState_Response::INVALID_POSE,
                _ => SimResult::RESULT_OPERATION_FAILED,
            };
            return answer(code, why.to_string());
        }
    };
    let Ok(mut s) = shared.lock() else {
        let why = "the simulator's lock is poisoned".to_string();
        return answer(SimResult::RESULT_OPERATION_FAILED, why);
    };
    match change {
        SetState::Car { x, y, yaw } => s.changes.push(Change::PlaceCar(x, y, yaw)),
        SetState::Model {
            name,
            pose,
            velocity,
        } => {
            let Some((file, _)) = s.placed.get(&name).cloned() else {
                let why = format!("nothing called {name} was placed (the car is {CAR})");
                return answer(SimResult::RESULT_NOT_FOUND, why);
            };
            let mesh = match World::model_mesh(&file, models, Surfaces::Visual) {
                Ok(mesh) => mesh,
                Err(e) => return answer(SimResult::RESULT_OPERATION_FAILED, e.to_string()),
            };
            s.placed.insert(name.clone(), (file, pose));
            s.announce();
            s.changes.push(Change::Place {
                name,
                mesh,
                pose,
                velocity,
            });
        }
    }
    answer(SimResult::RESULT_OK, String::new())
}

fn stamp(ns: u64) -> Time {
    let (sec, nanosec) = node::stamp(ns);
    Time { sec, nanosec }
}

fn header(ns: u64, frame: &str) -> Header {
    Header {
        stamp: stamp(ns),
        frame_id: frame.into(),
    }
}

fn quaternion(q: [f64; 4]) -> Quaternion {
    Quaternion {
        x: q[0],
        y: q[1],
        z: q[2],
        w: q[3],
    }
}

/// Everything the loop publishes.
struct Outputs {
    clock: Publisher<Clock>,
    odom: Publisher<OdometryMsg>,
    truth: Publisher<OdometryMsg>,
    imu: Publisher<Imu>,
    navsat: Publisher<NavSatFix>,
    joints: Publisher<JointState>,
    lidar: Option<Publisher<PointCloud2>>,
    radar: Option<Publisher<PointCloud2>>,
    logger: Logger,
}

impl Outputs {
    fn publish(&self, sim: &Simulator, outputs: Vec<Output>) -> Result<()> {
        let ns = sim.time_ns();
        for o in outputs {
            match o {
                Output::Clock => self.clock.publish(Clock { clock: stamp(ns) })?,
                Output::Odom(odom) => self.odom.publish(odometry(ns, "odom", &odom))?,
                Output::Truth(truth) => self.truth.publish(odometry(ns, "map", &truth))?,
                Output::Imu(reading) => self.imu.publish(imu(ns, &reading))?,
                Output::NavSat(fix) => self.navsat.publish(NavSatFix {
                    header: header(ns, "gps_link"),
                    status: NavSatStatus {
                        status: NavSatStatus::STATUS_FIX,
                        service: NavSatStatus::SERVICE_GPS,
                    },
                    latitude: fix.latitude,
                    longitude: fix.longitude,
                    altitude: fix.altitude,
                    ..Default::default()
                })?,
                Output::Joints(j) => self.joints.publish(joints(ns, &j))?,
                Output::Lidar(scan) => {
                    if let Some(lidar) = &self.lidar {
                        lidar.publish(cloud(ns, "lidar_link", &scan))?;
                    }
                }
                Output::Radar(scan) => {
                    if let Some(radar) = &self.radar {
                        radar.publish(cloud(ns, "radar_link", &scan))?;
                    }
                }
                Output::Hit { x, y } => log_warn!(
                    &self.logger,
                    "the car hit something at ({x:.2}, {y:.2}) and stopped"
                ),
            }
        }
        Ok(())
    }
}

fn odometry(ns: u64, frame: &str, o: &Odometry) -> OdometryMsg {
    OdometryMsg {
        header: header(ns, frame),
        child_frame_id: "base_link".into(),
        pose: PoseWithCovariance {
            pose: PoseMsg {
                position: Point {
                    x: o.x,
                    y: o.y,
                    z: o.z,
                },
                orientation: quaternion(Rot3::from_rpy(o.roll, o.pitch, o.yaw).quaternion()),
            },
            ..Default::default()
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: Vector3 {
                    x: o.speed,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: o.yaw_rate,
                },
            },
            ..Default::default()
        },
    }
}

fn imu(ns: u64, r: &ImuReading) -> Imu {
    let v = |p: sim_core::math::Vec3| Vector3 {
        x: p.x,
        y: p.y,
        z: p.z,
    };
    Imu {
        header: header(ns, "imu_link"),
        orientation: quaternion(r.orientation),
        angular_velocity: v(r.angular_velocity),
        linear_acceleration: v(r.linear_acceleration),
        ..Default::default()
    }
}

fn joints(ns: u64, j: &Joints) -> JointState {
    JointState {
        header: header(ns, ""),
        name: Joints::WHEELS
            .iter()
            .chain(&Joints::STEERING)
            .map(|s| s.to_string())
            .collect(),
        position: j.wheel_angle.iter().chain(&j.steering).copied().collect(),
        velocity: j.wheel_rate.iter().chain(&[0.0, 0.0]).copied().collect(),
        effort: Vec::new(),
    }
}

fn cloud(ns: u64, frame: &str, scan: &Scan) -> PointCloud2 {
    let width = scan.spec.h_samples as u32;
    PointCloud2 {
        header: header(ns, frame),
        height: scan.spec.v_samples as u32,
        width,
        fields: FIELDS
            .iter()
            .map(|&(name, offset, datatype)| PointField {
                name: name.into(),
                offset,
                datatype,
                count: 1,
            })
            .collect(),
        is_bigendian: false,
        point_step: POINT_STEP as u32,
        row_step: POINT_STEP as u32 * width,
        data: scan.pack(),
        is_dense: scan.dense(),
    }
}

/// The node's parameters (read once). Keep alive as long as the node: rclrs undeclares a
/// parameter when its handle is dropped.
struct Params {
    world: ReadOnlyParameter<Arc<str>>,
    models: ReadOnlyParameter<Arc<str>>,
    start_x: ReadOnlyParameter<f64>,
    start_y: ReadOnlyParameter<f64>,
    start_yaw: ReadOnlyParameter<f64>,
    wheelbase: ReadOnlyParameter<f64>,
    track: ReadOnlyParameter<f64>,
    wheel_radius: ReadOnlyParameter<f64>,
    max_steering: ReadOnlyParameter<f64>,
    imu_xyz: ReadOnlyParameter<Arc<[f64]>>,
    imu_rpy: ReadOnlyParameter<Arc<[f64]>>,
    gps_xyz: ReadOnlyParameter<Arc<[f64]>>,
    lidar_xyz: ReadOnlyParameter<Arc<[f64]>>,
    lidar_rpy: ReadOnlyParameter<Arc<[f64]>>,
    radar_xyz: ReadOnlyParameter<Arc<[f64]>>,
    radar_rpy: ReadOnlyParameter<Arc<[f64]>>,
    lidar: ReadOnlyParameter<bool>,
    radar: ReadOnlyParameter<bool>,
    obstacle: ReadOnlyParameter<bool>,
    obstacle_x: ReadOnlyParameter<f64>,
    obstacle_y: ReadOnlyParameter<f64>,
    obstacle_model: ReadOnlyParameter<Arc<str>>,
    body_length: ReadOnlyParameter<f64>,
    ground_step: ReadOnlyParameter<f64>,
    body_width: ReadOnlyParameter<f64>,
    rear_overhang: ReadOnlyParameter<f64>,
    body_height: ReadOnlyParameter<f64>,
    seed: ReadOnlyParameter<i64>,
    wheel_noise: ReadOnlyParameter<f64>,
    wheel_slip: ReadOnlyParameter<f64>,
    step: ReadOnlyParameter<f64>,
    clock_hz: ReadOnlyParameter<f64>,
    real_time_factor: ReadOnlyParameter<f64>,
    threads: ReadOnlyParameter<i64>,
    camera_lockstep: ReadOnlyParameter<bool>,
    camera_timeout: ReadOnlyParameter<f64>,
}

impl Params {
    fn declare(node: &Node) -> Result<Self> {
        let number = |name: &str, default: f64| -> Result<ReadOnlyParameter<f64>> {
            Ok(node.declare_parameter(name).default(default).read_only()?)
        };
        let xyz = |name: &str, default: [f64; 3]| -> Result<ReadOnlyParameter<Arc<[f64]>>> {
            Ok(node
                .declare_parameter(name)
                .default(Arc::from(default.as_slice()))
                .read_only()?)
        };
        let flag = |name: &str, default: bool| -> Result<ReadOnlyParameter<bool>> {
            Ok(node.declare_parameter(name).default(default).read_only()?)
        };
        let text = |name: &str, default: &str| -> Result<ReadOnlyParameter<Arc<str>>> {
            Ok(node
                .declare_parameter(name)
                .default(Arc::from(default))
                .read_only()?)
        };
        Ok(Self {
            world: text("world", "")?,
            models: text("models", "")?,
            start_x: number("start_x", 0.0)?,
            start_y: number("start_y", 0.0)?,
            start_yaw: number("start_yaw", 0.0)?,
            wheelbase: number("wheelbase", 1.561)?,
            track: number("track", 0.75)?,
            wheel_radius: number("wheel_radius", 0.18)?,
            max_steering: number("max_steering", 0.5)?,
            imu_xyz: xyz("imu_xyz", [0.0; 3])?,
            imu_rpy: xyz("imu_rpy", [0.0; 3])?,
            gps_xyz: xyz("gps_xyz", [0.0; 3])?,
            lidar_xyz: xyz("lidar_xyz", [0.0; 3])?,
            lidar_rpy: xyz("lidar_rpy", [0.0; 3])?,
            radar_xyz: xyz("radar_xyz", [0.0; 3])?,
            radar_rpy: xyz("radar_rpy", [0.0; 3])?,
            lidar: flag("lidar", false)?,
            radar: flag("radar", false)?,
            obstacle: flag("obstacle", false)?,
            obstacle_x: number("obstacle_x", 15.5)?,
            obstacle_y: number("obstacle_y", 0.0)?,
            obstacle_model: text("obstacle_model", "")?,
            body_length: number("body_length", 2.0)?,
            ground_step: number("ground_step", 0.1)?,
            body_width: number("body_width", 1.0)?,
            rear_overhang: number("rear_overhang", 0.2)?,
            body_height: number("body_height", 1.5)?,
            seed: node.declare_parameter("seed").default(1).read_only()?,
            wheel_noise: number("wheel_noise", 0.0)?,
            wheel_slip: number("wheel_slip", 0.0)?,
            step: number("step", 0.001)?,
            clock_hz: number("clock_hz", 1000.0)?,
            real_time_factor: number("real_time_factor", 1.0)?,
            threads: node.declare_parameter("threads").default(4).read_only()?,
            camera_lockstep: flag("camera_lockstep", false)?,
            camera_timeout: number("camera_timeout", 10.0)?,
        })
    }
}
