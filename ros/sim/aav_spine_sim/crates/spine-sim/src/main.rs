//! spine_sim: the spine, on a laptop (bench mode), or in Gazebo (the spine profile).
//!
//! The firmware's real logic (spine-core) runs here on the CAN frames spine_bridge sends,
//! and drives a car instead of motors. It answers with the frames the real spine sends, so
//! nothing above it can tell the difference. Which car is the `car` parameter:
//!
//!   model   (the bench) a model of the car, in bench-car, on this computer's clock
//!   gazebo  (sim.launch.py spine:=true) Gazebo's car: the spine's outputs go to
//!           gazebo_adapter's actuator model, which moves Gazebo's car and sends back what
//!           it measured. The spine runs on Gazebo's clock (use_sim_time), 10 ms a tick.
//!
//! Subscribes  /can/tx                       can_msgs/Frame    what spine_bridge puts on the bus
//!             /vehicle/bench/hw_estop       std_msgs/Bool     true = the hardware e-stop is pressed
//!             /vehicle/bench/driver_faults  std_msgs/UInt16   pretend a driver failed (fault bits)
//!             car:=gazebo only:
//!             /vehicle/gazebo/wheels        aav_interfaces/WheelSpeeds   Gazebo's wheels: the
//!                                           front two for the wheel nodes, the rear two's mean
//!                                           for the spine's measured speed (its encoder)
//!             /vehicle/gazebo/steering      aav_interfaces/ActuatorState the model's angle
//! Publishes   /can/rx                       can_msgs/Frame    VEHICLE_FEEDBACK 50 Hz, VEHICLE_STATUS 10 Hz,
//!                                                             and the front wheel nodes' WHEEL_SPEED_L
//!                                                             and WHEEL_SPEED_R 100 Hz (wheel-core)
//!             car:=model only:
//!             /vehicle/bench/truth          nav_msgs/Odometry where the model car really is (map
//!                                                             frame, 50 Hz), for the pretend sensors
//!             car:=gazebo only:
//!             /vehicle/gazebo/spine_outputs aav_interfaces/SpineOutputs  throttle, brake, reverse
//!                                           and steering target, every tick (100 Hz)
//! Parameters  car (model), wheelbase, track, wheel_radius (m), max_steering (rad), start_x,
//!             start_y (m), start_yaw (rad), speed_loop (default: the spine's speed loop
//!             profile, default, smooth or lqi: spine-core SpeedLoopProfile, TODO 27, TODO 3)
//!
//! A thread runs the spine at 100 Hz, like the board's main loop; the main thread spins.

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};

use anyhow::{anyhow, bail, Result};
use bench_car::{
    Bench, Car, MeasuredCar, PlantConfig, Pose, SpeedLoopProfile, SpineConfig, BENCH_WHEELS,
};
use rclrs::*;
use ros_env::aav_interfaces::msg::{ActuatorState, SpineOutputs, WheelSpeeds};
use ros_env::builtin_interfaces::msg::Time;
use ros_env::can_msgs::msg::Frame;
use ros_env::nav_msgs::msg::Odometry;
use ros_env::std_msgs::msg::{Bool, UInt16};

/// The spine's tick, as on the board (ms).
const TICK_MS: u32 = 10;
/// Gazebo's clock: more than this many ticks behind is a stall or a reset, not catch-up.
const MAX_CATCH_UP: i64 = 10;

/// Which car the spine drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CarKind {
    Model,
    Gazebo,
}

impl CarKind {
    fn parse(name: &str) -> Result<Self> {
        match name {
            "model" => Ok(CarKind::Model),
            "gazebo" => Ok(CarKind::Gazebo),
            other => bail!("car:={other}: use model (the bench) or gazebo"),
        }
    }
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("spine_sim")?;

    let param = |name: &str, default: f64| -> Result<f64> {
        Ok(node
            .declare_parameter(name)
            .default(default)
            .read_only()?
            .get())
    };
    let kind = CarKind::parse(
        &node
            .declare_parameter::<Arc<str>>("car")
            .default("model".into())
            .read_only()?
            .get(),
    )?;
    let plant = PlantConfig {
        wheelbase: param("wheelbase", 1.57)? as f32,
        track: param("track", 0.77)? as f32,
        max_steering: param("max_steering", 0.5)? as f32,
        ..PlantConfig::default()
    };
    let start = Pose {
        x: param("start_x", 0.0)?,
        y: param("start_y", 0.0)?,
        yaw: param("start_yaw", 0.0)?,
        ..Pose::default()
    };
    let profile_name = node
        .declare_parameter::<Arc<str>>("speed_loop")
        .default("default".into())
        .read_only()?
        .get();
    let profile: SpeedLoopProfile = profile_name
        .parse()
        .map_err(|e| anyhow!("speed_loop:={profile_name}: {e}"))?;
    let spine = SpineConfig {
        speed_loop: profile.config(),
        ..SpineConfig::default()
    };
    log_info!(node.logger(), "The speed loop's profile: {}", profile.name());
    let mut wheels = BENCH_WHEELS;
    wheels.wheel_radius = param("wheel_radius", 0.15)? as f32;
    let bad_wheels = |e| anyhow!("bad wheel settings: {e:?}");

    let running = Arc::new(AtomicBool::new(true));
    let commands = Arc::clone(executor.commands());
    {
        let running = Arc::clone(&running);
        let commands = Arc::clone(&commands);
        ctrlc::set_handler(move || {
            running.store(false, Ordering::SeqCst);
            commands.halt_spinning();
        })?;
    }

    // The subscriptions live in `_keep`, as long as main.
    let (spine_loop, _keep) = match kind {
        CarKind::Model => {
            let bench = Bench::new(spine, plant, start, wheels).map_err(bad_wheels)?;
            log_info!(
                node.logger(),
                "The spine's firmware logic, driving a model car (wheelbase {} m). Arm it \
                 with /spine/arm, as on the real car.",
                plant.wheelbase
            );
            let bench = Arc::new(Mutex::new(bench));
            let keep = switches(&node, &bench)?;
            let truth = node.create_publisher::<Odometry>("/vehicle/bench/truth")?;
            let mut ticks: u64 = 0;
            let handle = spawn_loop(&node, bench, Pacing::Wall, &running, move |b, stamp| {
                ticks += 1;
                if ticks.is_multiple_of(2) {
                    truth.publish(odometry(&b.pose(), stamp))?;
                }
                Ok(())
            })?;
            (handle, keep)
        }
        CarKind::Gazebo => {
            let car = MeasuredCar::new(plant.ticks_per_metre);
            let bench = Bench::with_car(spine, car, wheels).map_err(bad_wheels)?;
            log_info!(
                node.logger(),
                "The spine's firmware logic, driving Gazebo's car through the actuator model, \
                 on Gazebo's clock. Arm it with /spine/arm, as on the real car."
            );
            let bench = Arc::new(Mutex::new(bench));
            let mut keep = switches(&node, &bench)?;
            keep.extend(measurements(&node, &bench)?);
            let outputs = node.create_publisher::<SpineOutputs>("/vehicle/gazebo/spine_outputs")?;
            let handle = spawn_loop(
                &node,
                bench,
                Pacing::NodeClock,
                &running,
                move |b, stamp| {
                    if let Some(out) = b.outputs() {
                        let mut msg = SpineOutputs::default();
                        msg.header.stamp = stamp;
                        msg.header.frame_id = "base_link".into();
                        msg.throttle = out.throttle;
                        msg.brake = out.brake;
                        msg.reverse = out.reverse;
                        msg.steering = out.steering_target;
                        outputs.publish(msg)?;
                    }
                    Ok(())
                },
            )?;
            (handle, keep)
        }
    };

    executor.spin(SpinOptions::default()).first_error()?;
    running.store(false, Ordering::SeqCst);
    match spine_loop.join() {
        Ok(result) => result,
        Err(_) => bail!("the spine loop panicked"),
    }
}

/// Subscriptions that must live as long as the node.
type Keep = Vec<Box<dyn std::any::Any>>;

/// The bus and the bench's switches: the same for every car.
fn switches<C: Car + Send + 'static>(node: &Node, bench: &Arc<Mutex<Bench<C>>>) -> Result<Keep> {
    let frames_in = {
        let bench = Arc::clone(bench);
        node.create_subscription::<Frame, _>("/can/tx", move |f: Frame| {
            if !f.is_extended && !f.is_rtr && !f.is_error && f.id <= 0x7FF {
                let len = (f.dlc as usize).min(8);
                bench.lock().unwrap().receive(f.id as u16, &f.data[..len]);
            }
        })?
    };
    let estop = {
        let bench = Arc::clone(bench);
        node.create_subscription::<Bool, _>("/vehicle/bench/hw_estop", move |m: Bool| {
            bench.lock().unwrap().set_hw_estop(m.data);
        })?
    };
    let faults = {
        let bench = Arc::clone(bench);
        node.create_subscription::<UInt16, _>("/vehicle/bench/driver_faults", move |m: UInt16| {
            bench.lock().unwrap().set_driver_faults(m.data);
        })?
    };
    Ok(vec![Box::new(frames_in), Box::new(estop), Box::new(faults)])
}

/// What Gazebo's car measured, from gazebo_adapter.
fn measurements(node: &Node, bench: &Arc<Mutex<Bench<MeasuredCar>>>) -> Result<Keep> {
    let wheels = {
        let bench = Arc::clone(bench);
        node.create_subscription::<WheelSpeeds, _>(
            "/vehicle/gazebo/wheels",
            move |w: WheelSpeeds| {
                let mut b = bench.lock().unwrap();
                let car = b.car_mut();
                car.measure_front_wheels([w.front_left, w.front_right]);
                // The rear axle's speed: what the motor's encoder sees.
                car.measure_speed((w.rear_left + w.rear_right) / 2.0);
            },
        )?
    };
    let steering = {
        let bench = Arc::clone(bench);
        node.create_subscription::<ActuatorState, _>(
            "/vehicle/gazebo/steering",
            move |s: ActuatorState| {
                bench.lock().unwrap().car_mut().measure_steering(s.measured);
            },
        )?
    };
    Ok(vec![Box::new(wheels), Box::new(steering)])
}

/// What paces the spine's ticks.
#[derive(Debug, Clone, Copy)]
enum Pacing {
    /// This computer's clock: a steady 100 Hz; a late tick doesn't catch up.
    Wall,
    /// The node's clock (Gazebo's, with use_sim_time): one tick per 10 ms of it, catching
    /// up as it runs; nothing while it stands still.
    NodeClock,
}

/// Run the spine in its own thread; `after` sees the bench after each tick.
fn spawn_loop<C, F>(
    node: &Node,
    bench: Arc<Mutex<Bench<C>>>,
    pacing: Pacing,
    running: &Arc<AtomicBool>,
    mut after: F,
) -> Result<thread::JoinHandle<Result<()>>>
where
    C: Car + Send + 'static,
    F: FnMut(&Bench<C>, Time) -> Result<()> + Send + 'static,
{
    let frames_out = node.create_publisher::<Frame>("/can/rx")?;
    let clock = node.get_clock();
    let running = Arc::clone(running);
    Ok(thread::spawn(move || -> Result<()> {
        let tick_ns = TICK_MS as i64 * 1_000_000;
        let period = Duration::from_millis(TICK_MS as u64);
        let mut next = Instant::now() + period;
        let mut last_ns: Option<i64> = None;
        let mut tick = |stamp: Time| -> Result<()> {
            let mut b = bench.lock().unwrap();
            for (id, data) in b.tick(TICK_MS) {
                frames_out.publish(frame(id, data, stamp.clone()))?;
            }
            after(&b, stamp)
        };
        while running.load(Ordering::SeqCst) {
            match pacing {
                Pacing::Wall => {
                    tick(clock.now().to_ros_msg().unwrap_or_default())?;
                    // Keep a steady 100 Hz; if a tick runs late, don't try to catch up.
                    let now = Instant::now();
                    if next > now {
                        thread::sleep(next - now);
                        next += period;
                    } else {
                        next = now + period;
                    }
                }
                Pacing::NodeClock => {
                    let now = clock.now();
                    if now.nsec > 0 {
                        let owed = last_ns.map_or(0, |t| (now.nsec - t) / tick_ns);
                        if last_ns.is_none() || !(0..=MAX_CATCH_UP).contains(&owed) {
                            // The first tick, a stall or a reset: start again from now.
                            last_ns = Some(now.nsec);
                            tick(now.to_ros_msg().unwrap_or_default())?;
                        } else if let Some(t) = last_ns.as_mut() {
                            for _ in 0..owed {
                                *t += tick_ns;
                                tick(stamp_of(*t))?;
                            }
                        }
                    }
                    thread::sleep(Duration::from_millis(1));
                }
            }
        }
        Ok(())
    }))
}

fn stamp_of(nsec: i64) -> Time {
    Time {
        sec: nsec.div_euclid(1_000_000_000) as i32,
        nanosec: nsec.rem_euclid(1_000_000_000) as u32,
    }
}

fn frame(id: u16, data: [u8; 8], stamp: Time) -> Frame {
    let mut f = Frame::default();
    f.header.stamp = stamp;
    f.id = id as u32;
    f.dlc = 8;
    f.data = data;
    f
}

fn odometry(p: &Pose, stamp: Time) -> Odometry {
    let mut o = Odometry::default();
    o.header.stamp = stamp;
    o.header.frame_id = "map".into();
    o.child_frame_id = "base_link".into();
    o.pose.pose.position.x = p.x;
    o.pose.pose.position.y = p.y;
    o.pose.pose.orientation.z = (p.yaw / 2.0).sin();
    o.pose.pose.orientation.w = (p.yaw / 2.0).cos();
    o.twist.twist.linear.x = p.speed as f64;
    o.twist.twist.angular.z = p.yaw_rate as f64;
    o
}
