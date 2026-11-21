//! `sim_cameras`: the simulator's cameras as their own program, so a slow picture can't
//! delay the IMU (docs/designs/2026-11-05_own-simulator.md).
//!
//! ```text
//! sim_cameras                         # the test world, a scripted drive, 10 s: the timing check
//! sim_cameras --save /tmp/pics        # and keep each camera's first picture
//! sim_cameras --from 18 --speed 0     # the car parked 18 m on (7 m from the stop sign)
//! sim_cameras --bench 300             # draw as fast as it can: the most pictures a second
//! sim_cameras --ros                   # publish the car's camera topics (feature `ros`)
//! sim_cameras --ros --pose truth      # draw where /vehicle/sim/truth says, stamped with its time
//! sim_cameras --world road.sdf        # an SDF world (the core's reader), not the test copy
//! sim_cameras --at 49,-71,0          # parked there (m, m, rad), in the map frame
//! sim_cameras --poses p.csv --save D  # one picture per camera at each x,y,yaw line, then stop
//!                                     # (tools/lane_data.py: the learned lane finder's data)
//! sim_cameras --image-topic front_left=/x/image_raw  # its picture elsewhere (faults)
//! sim_cameras --software              # a software renderer only (lavapipe), even beside a GPU
//! sim_cameras --graphics simple       # plain colours, flat sky (default in software; pretty on a GPU)
//! ```
//!
//! Without `--ros` nothing is published: it renders, times and prints.

#[cfg(feature = "ros")]
mod ros;

use std::{
    path::{Path, PathBuf},
    process::ExitCode,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use camera_core::{world, Cameras, Frame, Pose, Rig, SimWorld};
use sim_cameras::{Adapter, BevyRenderer, Graphics};

const CAMERAS: &str = include_str!("../config/cameras.yaml");
const VEHICLE: &str = include_str!("../../../bringup/aav_description/config/vehicle.yaml");
/// Gazebo's stop sign picture, so the detector sees the same sign.
const STOP_SIGN: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../aav_sim_models/models/stop_sign/materials/textures/stop_sign.png"
);
/// aav_sim_models's models, for `model://` in a world file.
const MODELS: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../aav_sim_models/models");
/// The scripted drive turns back after this far, m.
const LAP: f64 = 60.0;

#[derive(Clone, Copy, PartialEq, Debug)]
enum PoseSource {
    /// Along the road at a steady speed, on this computer's clock.
    Script,
    /// /vehicle/sim/truth, on the simulator's clock.
    Truth,
}

#[derive(Debug)]
struct Options {
    seconds: Option<f64>,
    cameras: Option<Vec<String>>,
    pose: PoseSource,
    ros: bool,
    save: Option<PathBuf>,
    speed: f64,
    from: f64,
    plain_sign: bool,
    bench: Option<usize>,
    world: Option<PathBuf>,
    models: Vec<PathBuf>,
    topics: Vec<(String, String)>,
    at: Option<[f64; 3]>,
    /// A file of `x,y,yaw` lines: draw each pose once, save, and stop.
    poses: Option<PathBuf>,
    adapter: Adapter,
    /// None: pretty on a GPU, simple in software.
    graphics: Option<Graphics>,
}

impl Options {
    fn parse(mut args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut o = Self {
            seconds: None,
            cameras: None,
            pose: PoseSource::Script,
            ros: false,
            save: None,
            speed: 1.0,
            from: 0.0,
            plain_sign: false,
            bench: None,
            world: None,
            models: Vec::new(),
            topics: Vec::new(),
            at: None,
            poses: None,
            adapter: Adapter::Any,
            graphics: None,
        };
        while let Some(arg) = args.next() {
            let mut value = |name: &str| args.next().ok_or(format!("{name} needs a value"));
            match arg.as_str() {
                "--seconds" => o.seconds = Some(number(&value("--seconds")?)?),
                "--cameras" => {
                    o.cameras = Some(value("--cameras")?.split(',').map(str::to_owned).collect())
                }
                "--pose" => {
                    o.pose = match value("--pose")?.as_str() {
                        "script" => PoseSource::Script,
                        "truth" => PoseSource::Truth,
                        other => return Err(format!("--pose {other}: script or truth")),
                    }
                }
                "--ros" => o.ros = true,
                "--save" => o.save = Some(PathBuf::from(value("--save")?)),
                "--speed" => o.speed = number(&value("--speed")?)?,
                "--from" => o.from = number(&value("--from")?)?,
                "--plain-sign" => o.plain_sign = true,
                "--bench" => {
                    o.bench = Some(
                        value("--bench")?
                            .parse()
                            .map_err(|e| format!("--bench: {e}"))?,
                    )
                }
                "--at" => {
                    let v = value("--at")?;
                    let n = v
                        .split(',')
                        .map(|s| number(s.trim()))
                        .collect::<Result<Vec<_>, _>>()?;
                    let [x, y, yaw] = n[..] else {
                        return Err(format!("--at {v}: x,y,yaw"));
                    };
                    o.at = Some([x, y, yaw]);
                }
                "--poses" => o.poses = Some(PathBuf::from(value("--poses")?)),
                "--world" => o.world = Some(PathBuf::from(value("--world")?)),
                "--models" => o.models.extend(
                    value("--models")?
                        .split(':')
                        .filter(|p| !p.is_empty())
                        .map(PathBuf::from),
                ),
                "--image-topic" => {
                    let v = value("--image-topic")?;
                    let (name, topic) = v
                        .split_once('=')
                        .ok_or(format!("--image-topic {v}: camera=/topic"))?;
                    o.topics.push((name.to_owned(), topic.to_owned()));
                }
                "--software" => o.adapter = Adapter::Software,
                "--graphics" => {
                    o.graphics = match value("--graphics")?.as_str() {
                        "auto" => None,
                        word => Some(word.parse().map_err(|e| format!("{e} (or auto)"))?),
                    }
                }
                "-h" | "--help" => return Err(USAGE.to_owned()),
                other => return Err(format!("unknown option {other}\n{USAGE}")),
            }
        }
        if o.poses.is_some() && (o.ros || o.save.is_none()) {
            return Err("--poses needs --save DIR, and no --ros".into());
        }
        if o.pose == PoseSource::Truth && !o.ros {
            return Err("--pose truth needs --ros".into());
        }
        Ok(o)
    }
}

const USAGE: &str = "sim_cameras [--ros] [--pose script|truth] [--seconds S] \
[--cameras front_left,front_right,rear,chase] [--save DIR] [--speed M/S] [--from X] [--plain-sign] [--bench N] [--at X,Y,YAW] [--poses FILE] [--world FILE.sdf] [--models DIR:DIR] [--image-topic camera=/topic] [--software] [--graphics auto|simple|pretty]";

fn number(s: &str) -> Result<f64, String> {
    s.parse().map_err(|e| format!("{s}: {e}"))
}

/// Each camera's numbers, for the timing check.
#[derive(Default)]
struct Tally {
    stamps: Vec<Duration>,
    /// From the moment a picture was due to the moment it was out.
    delays: Vec<Duration>,
}

fn quantile(v: &[Duration], q: f64) -> Duration {
    let mut v = v.to_vec();
    v.sort_unstable();
    v.get(((v.len().max(1) - 1) as f64 * q).round() as usize)
        .copied()
        .unwrap_or_default()
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}

fn main() -> ExitCode {
    let options = match Options::parse(std::env::args().skip(1)) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(2);
        }
    };
    match run(&options) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("sim_cameras: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(o: &Options) -> Result<(), String> {
    let mut rig = Rig::from_yaml(CAMERAS, VEHICLE).map_err(|e| e.to_string())?;
    if let Some(names) = &o.cameras {
        let names: Vec<&str> = names.iter().map(String::as_str).collect();
        rig = rig.only(&names);
        if rig.cameras.is_empty() {
            return Err(format!("no camera called {}", names.join(" or ")));
        }
    }
    for (name, topic) in &o.topics {
        let spec = rig
            .cameras
            .iter_mut()
            .find(|c| &c.name == name)
            .ok_or(format!("--image-topic: no camera called {name}"))?;
        spec.image_override = Some(topic.clone());
    }
    let world = match &o.world {
        Some(path) => sdf_world(path, &o.models)?,
        None => test_world(o.plain_sign),
    };
    let world = world.with_car(VEHICLE).map_err(|e| e.to_string())?;
    run_in(o, rig, &world, &model_dirs(&o.models))
}

/// Where `model://` may be: `models`, then aav_sim_models's models, then GZ_SIM_RESOURCE_PATH.
fn model_dirs(models: &[PathBuf]) -> Vec<PathBuf> {
    let mut dirs = models.to_vec();
    dirs.push(PathBuf::from(MODELS));
    if let Some(more) = std::env::var_os("GZ_SIM_RESOURCE_PATH") {
        dirs.extend(std::env::split_paths(&more));
    }
    dirs
}

/// The world with the simulator's placed models in it, each read from its SDF file.
#[cfg(feature = "ros")]
fn with_placed(base: &SimWorld, placed: &[ros::Placed], dirs: &[PathBuf]) -> SimWorld {
    use sim_core::world::{Surfaces, World};
    let mut world = base.clone();
    for (name, file, at) in placed {
        match World::model_shapes(file, dirs, Surfaces::Visual) {
            Ok(shapes) => world
                .shapes
                .extend(camera_core::core_world::placed(name, &shapes, at)),
            Err(e) => eprintln!("sim_cameras: {name}: {e}"),
        }
    }
    world
}

/// An SDF world, read as the core simulator reads it: `model://` from `models`, then
/// aav_sim_models's models, then GZ_SIM_RESOURCE_PATH.
fn sdf_world(path: &Path, models: &[PathBuf]) -> Result<SimWorld, String> {
    let dirs = model_dirs(models);
    let start = Instant::now();
    let core = sim_core::world::World::load(path, &dirs, sim_core::world::Surfaces::Visual)
        .map_err(|e| e.to_string())?;
    for w in &core.warnings {
        eprintln!("sim_cameras: {}: {w}", path.display());
    }
    eprintln!(
        "sim_cameras: {} read in {:.1} s: {} shapes",
        path.display(),
        start.elapsed().as_secs_f64(),
        core.shapes.len()
    );
    Ok(SimWorld::from_core(&core))
}

/// The road world's copy in camera-core, with Gazebo's stop sign picture unless `plain`.
fn test_world(plain: bool) -> SimWorld {
    let mut world = SimWorld::test_world();
    if !plain {
        let texture = PathBuf::from(STOP_SIGN);
        if texture.exists() {
            world.shapes.retain(|s| !s.name.starts_with("stop_sign"));
            world.shapes.extend(world::stop_sign(
                "stop_sign",
                [25.0, -3.0],
                std::f64::consts::PI,
                Some(texture),
            ));
        }
    }
    world
}

fn run_in(o: &Options, rig: Rig, world: &SimWorld, dirs: &[PathBuf]) -> Result<(), String> {
    let start = Instant::now();
    let renderer = BevyRenderer::with_graphics(&rig, world, o.adapter, o.graphics)
        .map_err(|e| e.to_string())?;
    eprintln!(
        "sim_cameras: drawing with {}, graphics {}",
        renderer.adapter(),
        renderer.graphics()
    );
    eprintln!(
        "sim_cameras: Bevy ready in {:.1} s; cameras: {}",
        start.elapsed().as_secs_f64(),
        rig.cameras
            .iter()
            .map(|c| format!("{} ({} Hz)", c.name, c.rate.get()))
            .collect::<Vec<_>>()
            .join(", ")
    );
    let mut cameras = Cameras::new(rig, renderer);
    if let Some(n) = o.bench {
        return bench(&mut cameras, n);
    }
    if let (Some(file), Some(dir)) = (&o.poses, &o.save) {
        return each_pose(&mut cameras, file, dir);
    }

    #[cfg(feature = "ros")]
    let (out, truth, entities) = if o.ros {
        let (out, (rx, placed)) = ros::RosOut::start(cameras.rig(), o.pose == PoseSource::Truth)?;
        (Some(out), Some(rx), Some(placed))
    } else {
        (None, None, None)
    };
    #[cfg(not(feature = "ros"))]
    if o.ros {
        return Err(
            "built without ROS: cargo run --features ros (tools/sim_cameras.sh --ros)".into(),
        );
    }
    // Only the ROS build hears of models placed while running.
    #[cfg(not(feature = "ros"))]
    let _ = dirs;

    let seconds = o.seconds.or((!o.ros).then_some(10.0));
    let n = cameras.rig().cameras.len();
    let mut tallies: Vec<Tally> = (0..n).map(|_| Tally::default()).collect();
    let mut saved = vec![false; n];
    let started = Instant::now();
    loop {
        let elapsed = started.elapsed();
        if seconds.is_some_and(|s| elapsed.as_secs_f64() >= s) {
            break;
        }
        // The car's pose, and the time to stamp it with.
        let (now, car) = match o.pose {
            PoseSource::Script => {
                let wall = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map_err(|e| e.to_string())?;
                let car = match o.at {
                    Some([x, y, yaw]) => Pose::planar(x, y, yaw),
                    None => {
                        Pose::planar((o.from + elapsed.as_secs_f64() * o.speed) % LAP, 0.0, 0.0)
                    }
                };
                (wall, car)
            }
            PoseSource::Truth => {
                #[cfg(feature = "ros")]
                {
                    let rx = truth.as_ref().ok_or("no truth subscription")?;
                    match rx.recv_timeout(Duration::from_millis(200)) {
                        Ok(stamped) => {
                            // The newest one: a slow picture skips poses, never queues them.
                            rx.try_iter().last().unwrap_or(stamped)
                        }
                        Err(_) => continue,
                    }
                }
                #[cfg(not(feature = "ros"))]
                unreachable!("--pose truth needs --ros")
            }
        };
        #[cfg(feature = "ros")]
        if let Some(list) = entities.as_ref().and_then(|rx| rx.try_iter().last()) {
            // The simulator placed or took away a model: draw the world as it is now.
            let names: Vec<&str> = list.iter().map(|p| p.0.as_str()).collect();
            eprintln!("sim_cameras: placed models: [{}]", names.join(", "));
            cameras
                .renderer_mut()
                .set_world(&with_placed(world, &list, dirs))
                .map_err(|e| e.to_string())?;
        }
        let due_at = Instant::now();
        let due = cameras.due(now);
        if due.is_empty() {
            if o.pose == PoseSource::Script {
                std::thread::sleep(Duration::from_millis(1));
            }
            #[cfg(feature = "ros")]
            free_until(out.as_ref(), cameras.next_due())?;
            continue;
        }
        let frames = cameras
            .renderer_mut()
            .render(&car, now, &due)
            .map_err(|e| e.to_string())?;
        for frame in frames {
            let i = frame.camera;
            if let (Some(dir), false) = (&o.save, saved[i]) {
                save(dir, &cameras.rig().cameras[i].name, &frame)?;
                saved[i] = true;
            }
            tallies[i].stamps.push(frame.stamp);
            #[cfg(feature = "ros")]
            if let Some(out) = &out {
                out.publish(frame)?;
            }
            tallies[i].delays.push(due_at.elapsed());
        }
        // Every picture for this pose is out: the simulator's lockstep may go on.
        #[cfg(feature = "ros")]
        free_until(out.as_ref(), cameras.next_due())?;
    }
    report(&cameras, &tallies, started.elapsed());
    Ok(())
}

use camera_core::CameraRenderer;

/// Lockstep: tell the simulator when the next picture is due (only with `--pose truth`).
#[cfg(feature = "ros")]
fn free_until(out: Option<&ros::RosOut>, next: Option<Duration>) -> Result<(), String> {
    match (out, next) {
        (Some(out), Some(next)) => out.free_until(next),
        _ => Ok(()),
    }
}

/// The poses in a file: one `x,y,yaw` (m, m, rad, the map frame) a line; `#` starts a note.
fn read_poses(text: &str) -> Result<Vec<[f64; 3]>, String> {
    text.lines()
        .map(|l| l.split('#').next().unwrap_or("").trim())
        .filter(|l| !l.is_empty())
        .map(|l| {
            let n = l
                .split(',')
                .map(|s| number(s.trim()))
                .collect::<Result<Vec<_>, _>>()?;
            match n[..] {
                [x, y, yaw] => Ok([x, y, yaw]),
                _ => Err(format!("{l}: x,y,yaw")),
            }
        })
        .collect()
}

/// Every camera at each pose in `file`, saved as `<camera>_<line>.png` (the line counted
/// from 0, five digits) in `dir`.
fn each_pose(cameras: &mut Cameras<BevyRenderer>, file: &Path, dir: &Path) -> Result<(), String> {
    let text = std::fs::read_to_string(file).map_err(|e| format!("{}: {e}", file.display()))?;
    let poses = read_poses(&text)?;
    let all: Vec<usize> = (0..cameras.rig().cameras.len()).collect();
    let start = Instant::now();
    for (k, [x, y, yaw]) in poses.iter().enumerate() {
        let frames = cameras
            .renderer_mut()
            .render(
                &Pose::planar(*x, *y, *yaw),
                Duration::from_millis(k as u64 * 100),
                &all,
            )
            .map_err(|e| e.to_string())?;
        for frame in frames {
            let name = format!("{}_{k:05}", cameras.rig().cameras[frame.camera].name);
            save(dir, &name, &frame)?;
        }
    }
    eprintln!(
        "sim_cameras: {} poses in {:.1} s",
        poses.len(),
        start.elapsed().as_secs_f64()
    );
    Ok(())
}

fn save(dir: &Path, name: &str, f: &Frame) -> Result<(), String> {
    std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    let path = dir.join(format!("{name}.png"));
    image::save_buffer(
        &path,
        &f.rgb,
        f.width,
        f.height,
        image::ExtendedColorType::Rgb8,
    )
    .map_err(|e| format!("{}: {e}", path.display()))?;
    eprintln!("sim_cameras: saved {}", path.display());
    Ok(())
}

/// The timing check: each camera's rate, and how long its pictures took to be out.
fn report(cameras: &Cameras<BevyRenderer>, tallies: &[Tally], run: Duration) {
    println!("camera        frames   rate Hz  wanted   delay ms: median   p99    worst  late");
    for (spec, t) in cameras.rig().cameras.iter().zip(tallies) {
        let period = spec.rate.period();
        let late = t.delays.iter().filter(|&&d| d > period).count();
        let span = match (t.stamps.first(), t.stamps.last()) {
            (Some(a), Some(b)) if t.stamps.len() > 1 => (*b - *a).as_secs_f64(),
            _ => 0.0,
        };
        let hz = if span > 0.0 {
            (t.stamps.len() - 1) as f64 / span
        } else {
            0.0
        };
        println!(
            "{:<12} {:>7} {:>9.2} {:>7.1} {:>16.1} {:>6.1} {:>8.1} {:>5}",
            spec.name,
            t.stamps.len(),
            hz,
            spec.rate.get(),
            ms(quantile(&t.delays, 0.5)),
            ms(quantile(&t.delays, 0.99)),
            ms(quantile(&t.delays, 1.0)),
            late
        );
    }
    println!(
        "run {:.1} s; late = out after the next picture was due",
        run.as_secs_f64()
    );
}

/// Draws every camera back to back, `n` times: the most a second this computer can do.
fn bench(cameras: &mut Cameras<BevyRenderer>, n: usize) -> Result<(), String> {
    let all: Vec<usize> = (0..cameras.rig().cameras.len()).collect();
    let mut times = Vec::with_capacity(n);
    let start = Instant::now();
    for k in 0..n {
        let car = Pose::planar((k as f64 * 0.1) % LAP, 0.0, 0.0);
        let t = Instant::now();
        cameras
            .renderer_mut()
            .render(&car, Duration::from_millis(k as u64 * 100), &all)
            .map_err(|e| e.to_string())?;
        times.push(t.elapsed());
    }
    let total = start.elapsed().as_secs_f64();
    println!(
        "bench: {n} rounds of {} cameras in {total:.2} s = {:.1} rounds/s ({:.0} pictures/s); \
         one round: median {:.1} ms, p99 {:.1} ms, worst {:.1} ms",
        all.len(),
        n as f64 / total,
        (n * all.len()) as f64 / total,
        ms(quantile(&times, 0.5)),
        ms(quantile(&times, 0.99)),
        ms(quantile(&times, 1.0)),
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::read_poses;

    #[test]
    fn poses_are_read_line_by_line() {
        let poses = read_poses("# x,y,yaw\n1,2,0.5\n\n -3 , 4.5, -1 # a note\n").unwrap();
        assert_eq!(poses, vec![[1.0, 2.0, 0.5], [-3.0, 4.5, -1.0]]);
        assert!(read_poses("1,2\n").is_err());
    }
}
