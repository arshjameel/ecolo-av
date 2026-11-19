//! The `simulator` node's decisions, with no ROS (S5, 18 Nov): its parameter checks, what
//! its services accept, its time stamps and its pace against the wall clock. `sim-node`
//! turns ROS's messages into these plain values and the answers back into ROS's codes,
//! so everything it decides is tested here, on any laptop.

use std::fmt;
use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::math::{Pose, Rot3, Vec3};
use crate::sim::Velocity;

/// The car's entity name, as Gazebo's spawn gave it (`-name aav`).
pub const CAR: &str = "aav";

/// A parameter out of its range.
#[derive(Debug, Clone, PartialEq)]
pub enum ParamError {
    /// `wheel_noise`: between 0 and 1 m/s.
    WheelNoise(f64),
    /// `wheel_slip`: between -0.5 and 0.5.
    WheelSlip(f64),
    /// `step`: above 0, at most 0.01 s.
    Step(f64),
    /// `real_time_factor`: 0 (as fast as it can) or more.
    RealTimeFactor(f64),
    /// A sensor's mount: three numbers each (which, how many).
    Mount(&'static str, usize),
}

impl fmt::Display for ParamError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParamError::WheelNoise(v) => write!(f, "wheel_noise:={v}: between 0 and 1 m/s"),
            ParamError::WheelSlip(v) => {
                write!(f, "wheel_slip:={v}: between -0.5 and 0.5 (a fraction)")
            }
            ParamError::Step(v) => write!(f, "step:={v}: between 0 and 0.01 s"),
            ParamError::RealTimeFactor(v) => {
                write!(f, "real_time_factor:={v}: 0 (as fast as it can) or more")
            }
            ParamError::Mount(what, n) => write!(f, "{what}: three numbers, not {n}"),
        }
    }
}

impl std::error::Error for ParamError {}

/// `wheel_noise` (m/s), checked.
///
/// # Errors
///
/// [`ParamError::WheelNoise`] outside 0 to 1 (or NaN).
pub fn wheel_noise(v: f64) -> Result<f64, ParamError> {
    if (0.0..=1.0).contains(&v) {
        Ok(v)
    } else {
        Err(ParamError::WheelNoise(v))
    }
}

/// `wheel_slip` (a fraction), checked.
///
/// # Errors
///
/// [`ParamError::WheelSlip`] outside -0.5 to 0.5 (or NaN).
pub fn wheel_slip(v: f64) -> Result<f64, ParamError> {
    if (-0.5..=0.5).contains(&v) {
        Ok(v)
    } else {
        Err(ParamError::WheelSlip(v))
    }
}

/// `step` (s), checked, as whole nanoseconds.
///
/// # Errors
///
/// [`ParamError::Step`] unless above 0 and at most 0.01 s.
pub fn step_ns(step: f64) -> Result<u64, ParamError> {
    if step > 0.0 && step <= 0.01 {
        Ok((step * 1e9).round() as u64)
    } else {
        Err(ParamError::Step(step))
    }
}

/// `real_time_factor`, checked.
///
/// # Errors
///
/// [`ParamError::RealTimeFactor`] below 0 (or NaN).
pub fn real_time_factor(v: f64) -> Result<f64, ParamError> {
    if v >= 0.0 {
        Ok(v)
    } else {
        Err(ParamError::RealTimeFactor(v))
    }
}

/// A sensor's mount from vehicle.yaml's xyz (m) and rpy (rad).
///
/// # Errors
///
/// [`ParamError::Mount`] if either isn't three numbers.
pub fn mount(xyz: &[f64], rpy: &[f64]) -> Result<Pose, ParamError> {
    let three = |v: &[f64], what| -> Result<[f64; 3], ParamError> {
        v.try_into().map_err(|_| ParamError::Mount(what, v.len()))
    };
    Ok(Pose::from_xyz_rpy(three(xyz, "xyz")?, three(rpy, "rpy")?))
}

/// ns on the simulated clock as ROS's (sec, nanosec).
#[must_use]
pub fn stamp(ns: u64) -> (i32, u32) {
    ((ns / 1_000_000_000) as i32, (ns % 1_000_000_000) as u32)
}

/// ROS's (sec, nanosec) as ns; a time before 0 is 0.
#[must_use]
pub fn nanoseconds(sec: i32, nanosec: u32) -> u64 {
    u64::try_from(sec).unwrap_or(0) * 1_000_000_000 + u64::from(nanosec)
}

/// The turn about z (rad) of a quaternion (x, y, z, w).
#[must_use]
pub fn yaw(q: [f64; 4]) -> f64 {
    let [x, y, z, w] = q;
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

/// A pose as a service gives it: the position (m) and the orientation (x, y, z, w).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Placement {
    /// m, in the map.
    pub position: [f64; 3],
    /// (x, y, z, w).
    pub orientation: [f64; 4],
}

/// A twist as a service gives it: linear (m/s) then angular (rad/s), x, y, z each.
pub type Twist6 = [f64; 6];

/// Why a service says no.
#[derive(Debug, Clone, PartialEq)]
pub enum Refusal {
    /// The pose isn't in the map (the world): its frame.
    Frame(String),
    /// The car is placed standing still.
    CarMoving,
    /// A model moves on the ground only: linear x, y and angular z.
    OffTheGround,
    /// The position isn't finite, or the quaternion isn't a unit one.
    BadPose,
    /// The pose or the twist isn't finite.
    NotFinite,
    /// No uri.
    NoUri,
    /// `model://<name>` in none of the model folders: the uri, the folders.
    NoModel(String, Vec<PathBuf>),
}

impl fmt::Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Refusal::Frame(frame) => {
                write!(f, "the pose must be in the map (the world), not {frame}")
            }
            Refusal::CarMoving => {
                f.write_str("the car is placed standing still: give a zero twist")
            }
            Refusal::OffTheGround => {
                f.write_str("a model moves on the ground: linear x, y and angular z only")
            }
            Refusal::BadPose => {
                f.write_str("the position must be finite and the quaternion a unit one")
            }
            Refusal::NotFinite => f.write_str("the pose and the twist must be finite"),
            Refusal::NoUri => f.write_str("no uri: give an SDF model file"),
            Refusal::NoModel(uri, models) => write!(f, "no {uri} in {models:?}"),
        }
    }
}

impl std::error::Error for Refusal {}

/// What a SetEntityState asks for, once checked.
#[derive(Debug, Clone, PartialEq)]
pub enum SetState {
    /// The car, put at (x, y, yaw) in the map, standing still.
    Car {
        /// m.
        x: f64,
        /// m.
        y: f64,
        /// rad.
        yaw: f64,
    },
    /// A placed model (its name), put at a pose, then moving at a velocity.
    Model {
        /// Its name.
        name: String,
        /// Where (x, y, z and yaw).
        pose: Pose,
        /// Still, or walking on.
        velocity: Velocity,
    },
}

/// A SetEntityState request checked: the car at a pose, standing still (Gazebo's
/// `set_pose`), or a placed model at a pose, then moving on the ground (TODO 170).
///
/// # Errors
///
/// A [`Refusal`]: the frame, a moving car, a model off the ground, a bad pose.
pub fn set_state(
    entity: &str,
    frame: &str,
    at: &Placement,
    twist: &Twist6,
) -> Result<SetState, Refusal> {
    let frame = frame.trim_start_matches('/');
    if !matches!(frame, "" | "world" | "map") {
        return Err(Refusal::Frame(frame.to_string()));
    }
    let [px, py, pz] = at.position;
    let yaw = yaw(at.orientation);
    if entity == CAR {
        if twist.iter().any(|v| *v != 0.0) {
            return Err(Refusal::CarMoving);
        }
        let norm = at.orientation.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !(px.is_finite() && py.is_finite() && (norm - 1.0).abs() < 0.01) {
            return Err(Refusal::BadPose);
        }
        return Ok(SetState::Car { x: px, y: py, yaw });
    }
    let [vx, vy, vz, wx, wy, wz] = *twist;
    if [vz, wx, wy].iter().any(|v| *v != 0.0) {
        return Err(Refusal::OffTheGround);
    }
    if ![px, py, pz, vx, vy, wz].iter().all(|v| v.is_finite()) {
        return Err(Refusal::NotFinite);
    }
    Ok(SetState::Model {
        name: entity.to_string(),
        pose: Pose::new(Vec3::new(px, py, pz), Rot3::from_yaw(yaw)),
        velocity: Velocity {
            x: vx,
            y: vy,
            yaw_rate: wz,
        },
    })
}

/// A SpawnEntity's uri as a model file: `model://<name>` found in the model folders (its
/// `model.sdf`), `file://<path>`, or a plain path.
///
/// # Errors
///
/// [`Refusal::NoUri`] for an empty uri, [`Refusal::NoModel`] for a `model://` in none of
/// the folders.
pub fn model_file(uri: &str, models: &[PathBuf]) -> Result<PathBuf, Refusal> {
    if uri.is_empty() {
        return Err(Refusal::NoUri);
    }
    match uri.strip_prefix("model://") {
        Some(name) => models
            .iter()
            .map(|m| m.join(name).join("model.sdf"))
            .find(|p| p.is_file())
            .ok_or_else(|| Refusal::NoModel(uri.to_string(), models.to_vec())),
        None => Ok(PathBuf::from(uri.strip_prefix("file://").unwrap_or(uri))),
    }
}

/// A spawned model's name: the one asked for, else its folder's (`.../post/model.sdf`:
/// post), else `model`.
#[must_use]
pub fn model_name(asked: &str, file: &Path) -> String {
    if !asked.is_empty() {
        return asked.to_string();
    }
    file.parent()
        .and_then(Path::file_name)
        .map_or_else(|| "model".into(), |n| n.to_string_lossy().into_owned())
}

/// The pose a SpawnEntity's initial_pose gives: x, y, z and its yaw only.
#[must_use]
pub fn spawn_pose(at: &Placement) -> Pose {
    let [x, y, z] = at.position;
    Pose::new(Vec3::new(x, y, z), Rot3::from_yaw(yaw(at.orientation)))
}

/// Keeps the simulated time with the wall's, at `real_time_factor`.
#[derive(Debug, Clone, PartialEq)]
pub struct Pace {
    rtf: f64,
    /// The wall time and the simulated time (ns) the pace counts from.
    origin: (Duration, u64),
}

/// What to do after a step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wait {
    /// Run on: as fast as it can, or behind but catching up.
    Go,
    /// Sleep this long: ahead of the wall.
    Sleep(Duration),
    /// Too far behind (over [`Pace::GIVE_UP`]): run on from here, slower than asked, as
    /// Gazebo does, rather than rush a burst of steps.
    Rebase,
}

impl Pace {
    /// Behind more than this, the pace starts again from now.
    pub const GIVE_UP: Duration = Duration::from_millis(100);

    /// A pace from `wall` (the wall clock's time now, from any fixed start) and the
    /// simulated `sim_ns` now; `rtf` 0: as fast as it can.
    #[must_use]
    pub fn new(rtf: f64, wall: Duration, sim_ns: u64) -> Self {
        Pace {
            rtf,
            origin: (wall, sim_ns),
        }
    }

    /// After a step: the simulated time is `sim_ns`, the wall's `wall`.
    pub fn after_step(&mut self, sim_ns: u64, wall: Duration) -> Wait {
        if self.rtf <= 0.0 {
            return Wait::Go;
        }
        let (wall0, sim0) = self.origin;
        let simulated = sim_ns.saturating_sub(sim0) as f64 * 1e-9;
        let due = wall0 + Duration::from_secs_f64(simulated / self.rtf);
        if due > wall {
            Wait::Sleep(due - wall)
        } else if wall - due > Self::GIVE_UP {
            self.origin = (wall, sim_ns);
            Wait::Rebase
        } else {
            Wait::Go
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LEVEL: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

    fn at(x: f64, y: f64, yaw: f64) -> Placement {
        Placement {
            position: [x, y, 0.0],
            orientation: [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()],
        }
    }

    #[test]
    fn parameters_out_of_range_are_refused_with_their_name() {
        assert_eq!(wheel_noise(0.05), Ok(0.05));
        assert_eq!(wheel_noise(1.5), Err(ParamError::WheelNoise(1.5)));
        assert!(wheel_noise(f64::NAN).is_err());
        assert_eq!(wheel_slip(-0.02), Ok(-0.02));
        assert!(wheel_slip(0.6).is_err());
        assert_eq!(step_ns(0.001), Ok(1_000_000));
        assert!(step_ns(0.0).is_err() && step_ns(0.02).is_err());
        assert_eq!(real_time_factor(0.0), Ok(0.0));
        assert!(real_time_factor(-1.0).is_err());
        let words = ParamError::WheelSlip(0.6).to_string();
        assert!(words.starts_with("wheel_slip:=0.6"), "{words}");
    }

    #[test]
    fn a_mount_is_three_numbers_and_three_angles() {
        let m = mount(&[1.0, 0.0, 1.4], &[0.0, 0.0, 0.0]).expect("a mount");
        assert_eq!(m.position, Vec3::new(1.0, 0.0, 1.4));
        assert_eq!(
            mount(&[1.0, 2.0], &[0.0; 3]),
            Err(ParamError::Mount("xyz", 2))
        );
        assert_eq!(mount(&[0.0; 3], &[]), Err(ParamError::Mount("rpy", 0)));
    }

    #[test]
    fn stamps_split_and_join_whole_seconds() {
        assert_eq!(stamp(0), (0, 0));
        assert_eq!(stamp(12_345_678_901), (12, 345_678_901));
        assert_eq!(nanoseconds(12, 345_678_901), 12_345_678_901);
        assert_eq!(nanoseconds(-1, 5), 5, "before 0 counts as 0");
    }

    #[test]
    fn a_quaternions_yaw() {
        for yaw_in in [0.0, 1.0, -2.5, 3.0] {
            assert!((yaw(at(0.0, 0.0, yaw_in).orientation) - yaw_in).abs() < 1e-12);
        }
    }

    #[test]
    fn the_car_is_placed_standing_still_in_the_map() {
        let still = [0.0; 6];
        assert_eq!(
            set_state(CAR, "map", &at(3.0, -4.0, 1.0), &still),
            Ok(SetState::Car {
                x: 3.0,
                y: -4.0,
                yaw: yaw(at(0.0, 0.0, 1.0).orientation)
            })
        );
        for frame in ["", "/world", "world"] {
            assert!(set_state(CAR, frame, &at(0.0, 0.0, 0.0), &still).is_ok());
        }
        assert_eq!(
            set_state(CAR, "odom", &at(0.0, 0.0, 0.0), &still),
            Err(Refusal::Frame("odom".into()))
        );
        let moving = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert_eq!(
            set_state(CAR, "map", &at(0.0, 0.0, 0.0), &moving),
            Err(Refusal::CarMoving)
        );
        let squashed = Placement {
            position: [0.0; 3],
            orientation: [0.0, 0.0, 0.0, 0.5],
        };
        assert_eq!(
            set_state(CAR, "map", &squashed, &still),
            Err(Refusal::BadPose)
        );
        let lost = Placement {
            position: [f64::NAN, 0.0, 0.0],
            orientation: LEVEL,
        };
        assert_eq!(set_state(CAR, "map", &lost, &still), Err(Refusal::BadPose));
    }

    #[test]
    fn a_model_walks_on_the_ground_only() {
        let walk = [0.8, -0.2, 0.0, 0.0, 0.0, 0.1];
        let Ok(SetState::Model {
            name,
            pose,
            velocity,
        }) = set_state("walker", "map", &at(5.0, 1.0, 0.0), &walk)
        else {
            panic!("a walker is placed");
        };
        assert_eq!(name, "walker");
        assert_eq!(pose.position, Vec3::new(5.0, 1.0, 0.0));
        assert_eq!(
            velocity,
            Velocity {
                x: 0.8,
                y: -0.2,
                yaw_rate: 0.1
            }
        );
        let flying = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        assert_eq!(
            set_state("walker", "map", &at(0.0, 0.0, 0.0), &flying),
            Err(Refusal::OffTheGround)
        );
        let wild = [f64::INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert_eq!(
            set_state("walker", "map", &at(0.0, 0.0, 0.0), &wild),
            Err(Refusal::NotFinite)
        );
    }

    #[test]
    fn a_spawned_models_file_and_name() {
        let here = std::env::temp_dir().join(format!("sim-core-node-{}", std::process::id()));
        let post = here.join("post");
        std::fs::create_dir_all(&post).expect("a folder");
        std::fs::write(post.join("model.sdf"), "<sdf/>").expect("a file");
        let models = vec![PathBuf::from("/nowhere"), here.clone()];
        let found = model_file("model://post", &models).expect("found");
        assert_eq!(found, post.join("model.sdf"));
        assert_eq!(model_name("", &found), "post");
        assert_eq!(model_name("box1", &found), "box1");
        assert_eq!(model_name("", Path::new("/")), "model");
        assert_eq!(
            model_file("file:///a/b.sdf", &models),
            Ok(PathBuf::from("/a/b.sdf"))
        );
        assert_eq!(model_file("b.sdf", &models), Ok(PathBuf::from("b.sdf")));
        assert_eq!(model_file("", &models), Err(Refusal::NoUri));
        assert!(matches!(
            model_file("model://kerb", &models),
            Err(Refusal::NoModel(..))
        ));
        std::fs::remove_dir_all(&here).ok();
    }

    #[test]
    fn the_pace_sleeps_ahead_runs_behind_and_gives_up_far_behind() {
        let ms = Duration::from_millis;
        let mut pace = Pace::new(1.0, ms(1000), 0);
        assert_eq!(pace.after_step(10_000_000, ms(1000)), Wait::Sleep(ms(10)));
        assert_eq!(
            pace.after_step(10_000_000, ms(1050)),
            Wait::Go,
            "a little behind"
        );
        assert_eq!(pace.after_step(10_000_000, ms(1200)), Wait::Rebase);
        // From there: 1200 ms on the wall is 10 ms simulated.
        assert_eq!(pace.after_step(20_000_000, ms(1200)), Wait::Sleep(ms(10)));
        // Half real time: 10 ms simulated takes 20 ms.
        let mut slow = Pace::new(0.5, ms(0), 0);
        assert_eq!(slow.after_step(10_000_000, ms(0)), Wait::Sleep(ms(20)));
        let mut flat_out = Pace::new(0.0, ms(0), 0);
        assert_eq!(flat_out.after_step(10_000_000_000, ms(0)), Wait::Go);
    }
}
