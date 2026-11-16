//! The car's cameras: each one's lens, rate, topics, frame and mount.
//!
//! `config/cameras.yaml` holds the lens, rate and topics; the mount of every real camera
//! comes from `vehicle.yaml` (the only place car measurements live), by the frame's name.
//! Only the simulator's own chase camera has its mount in `cameras.yaml`.

use std::{fmt, time::Duration};

use serde::Deserialize;

use crate::{
    geom::{Pose, Vec3},
    lens::{optical_in_link, Lens, LensError, Pixel},
};

/// A rate, in frames a second.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hz(f64);

impl Hz {
    pub fn new(hz: f64) -> Result<Self, RigError> {
        if hz.is_finite() && hz > 0.0 {
            Ok(Self(hz))
        } else {
            Err(RigError::BadRate(hz))
        }
    }

    #[must_use]
    pub fn get(self) -> f64 {
        self.0
    }

    /// The time between two frames.
    #[must_use]
    pub fn period(self) -> Duration {
        Duration::from_secs_f64(1.0 / self.0)
    }
}

/// One camera, ready to render.
#[derive(Clone, Debug, PartialEq)]
pub struct CameraSpec {
    /// Short name: front_left, front_right, rear, chase.
    pub name: String,
    /// `<topic>/image_raw` and `<topic>/camera_info`.
    pub topic: String,
    /// The header's frame_id, as Gazebo's `gz_frame_id`: the link, not an optical frame.
    pub frame_id: String,
    pub lens: Lens,
    pub rate: Hz,
    /// The camera's link in base_link (x ahead, z up).
    pub mount: Pose,
    /// Clip planes, metres.
    pub near: f64,
    pub far: f64,
    /// Publishes camera_info too (Gazebo's chase camera has none).
    pub camera_info: bool,
    /// Sees the car's own body (only the chase camera).
    pub sees_car: bool,
    /// The picture on this topic instead of `<topic>/image_raw` (camera_info stays): the
    /// faults relay, as Gazebo's bridge does it (`aav_gazebo` `faults.relayed`).
    pub image_override: Option<String>,
}

impl CameraSpec {
    #[must_use]
    pub fn image_topic(&self) -> String {
        self.image_override
            .clone()
            .unwrap_or_else(|| format!("{}/image_raw", self.topic))
    }

    #[must_use]
    pub fn info_topic(&self) -> String {
        format!("{}/camera_info", self.topic)
    }

    /// Where a point in the world (map) lands in this camera's picture, with the car at
    /// `car` (base_link in map). `None` behind the camera.
    #[must_use]
    pub fn project(&self, car: &Pose, world_point: Vec3) -> Option<Pixel> {
        let camera_in_map = car.then(&self.mount);
        let in_link = camera_in_map.inverse().apply(world_point);
        let optical = optical_in_link().inverse().apply(in_link);
        self.lens.project(optical)
    }
}

/// Every camera, in the config's order.
#[derive(Clone, Debug, PartialEq)]
pub struct Rig {
    pub cameras: Vec<CameraSpec>,
}

/// Why the rig can't be read.
#[derive(Debug)]
pub enum RigError {
    Yaml(String, serde_yaml_ng::Error),
    /// A camera's mount names a frame vehicle.yaml doesn't have.
    NoMount {
        camera: String,
        frame: String,
    },
    /// Neither `mount` nor `mount_xyz`, or both.
    MountChoice(String),
    /// Neither `focal_px` nor `horizontal_fov`, or both.
    LensChoice(String),
    Lens(String, LensError),
    BadRate(f64),
}

impl fmt::Display for RigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Yaml(which, e) => write!(f, "{which}: {e}"),
            Self::NoMount { camera, frame } => {
                write!(f, "camera {camera}: vehicle.yaml has no sensors.{frame}")
            }
            Self::MountChoice(c) => write!(f, "camera {c}: give mount or mount_xyz, not both"),
            Self::LensChoice(c) => {
                write!(f, "camera {c}: give focal_px or horizontal_fov, not both")
            }
            Self::Lens(c, e) => write!(f, "camera {c}: {e}"),
            Self::BadRate(hz) => write!(f, "rate {hz} Hz: must be above 0"),
        }
    }
}

impl std::error::Error for RigError {}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CameraEntry {
    name: String,
    topic: String,
    frame_id: String,
    /// A frame under vehicle.yaml's `sensors:`.
    #[serde(default)]
    mount: Option<String>,
    /// Or the mount itself (the chase camera only).
    #[serde(default)]
    mount_xyz: Option<[f64; 3]>,
    #[serde(default)]
    mount_rpy: Option<[f64; 3]>,
    width: u32,
    height: u32,
    #[serde(default)]
    focal_px: Option<f64>,
    #[serde(default)]
    horizontal_fov: Option<f64>,
    rate_hz: f64,
    near: f64,
    far: f64,
    #[serde(default = "yes")]
    camera_info: bool,
    #[serde(default)]
    sees_car: bool,
    #[serde(default = "yes")]
    enabled: bool,
}

fn yes() -> bool {
    true
}

#[derive(Deserialize)]
struct CamerasFile {
    cameras: Vec<CameraEntry>,
}

#[derive(Deserialize)]
struct Mount {
    xyz: [f64; 3],
    rpy: [f64; 3],
}

#[derive(Deserialize)]
struct VehicleFile {
    sensors: std::collections::BTreeMap<String, Mount>,
}

impl Rig {
    /// From `config/cameras.yaml`'s text and `vehicle.yaml`'s. Cameras with
    /// `enabled: false` are left out.
    pub fn from_yaml(cameras: &str, vehicle: &str) -> Result<Self, RigError> {
        let file: CamerasFile = serde_yaml_ng::from_str(cameras)
            .map_err(|e| RigError::Yaml("cameras.yaml".into(), e))?;
        let vehicle: VehicleFile = serde_yaml_ng::from_str(vehicle)
            .map_err(|e| RigError::Yaml("vehicle.yaml".into(), e))?;
        let cameras = file
            .cameras
            .into_iter()
            .filter(|c| c.enabled)
            .map(|c| {
                let mount = match (&c.mount, c.mount_xyz) {
                    (Some(frame), None) => {
                        let m = vehicle
                            .sensors
                            .get(frame)
                            .ok_or_else(|| RigError::NoMount {
                                camera: c.name.clone(),
                                frame: frame.clone(),
                            })?;
                        Pose::from_xyz_rpy(m.xyz, m.rpy)
                    }
                    (None, Some(xyz)) => Pose::from_xyz_rpy(xyz, c.mount_rpy.unwrap_or_default()),
                    _ => return Err(RigError::MountChoice(c.name.clone())),
                };
                let lens = match (c.focal_px, c.horizontal_fov) {
                    (Some(f), None) => Lens::from_focal(c.width, c.height, f),
                    (None, Some(fov)) => Lens::from_horizontal_fov(c.width, c.height, fov),
                    _ => return Err(RigError::LensChoice(c.name.clone())),
                }
                .map_err(|e| RigError::Lens(c.name.clone(), e))?;
                Ok(CameraSpec {
                    rate: Hz::new(c.rate_hz)?,
                    name: c.name,
                    topic: c.topic,
                    frame_id: c.frame_id,
                    lens,
                    mount,
                    near: c.near,
                    far: c.far,
                    camera_info: c.camera_info,
                    sees_car: c.sees_car,
                    image_override: None,
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self { cameras })
    }

    /// Only the named cameras, in the rig's order.
    #[must_use]
    pub fn only(mut self, names: &[&str]) -> Self {
        self.cameras.retain(|c| names.contains(&c.name.as_str()));
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VEHICLE: &str =
        include_str!("../../../../../bringup/aav_description/config/vehicle.yaml");
    const CAMERAS: &str = include_str!("../../../config/cameras.yaml");

    fn rig() -> Rig {
        Rig::from_yaml(CAMERAS, VEHICLE).unwrap()
    }

    #[test]
    fn has_gazebos_cameras() {
        let rig = rig();
        let names: Vec<_> = rig.cameras.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["front_left", "front_right", "rear", "chase"]);
        let front = &rig.cameras[0];
        assert_eq!(front.image_topic(), "/sensors/camera/front_left/image_raw");
        assert_eq!(front.info_topic(), "/sensors/camera/front_left/camera_info");
        assert_eq!(front.frame_id, "camera_left_link");
        assert_eq!((front.lens.width(), front.lens.height()), (640, 360));
        assert!((front.lens.fx() - 343.0).abs() < 1e-9);
        assert_eq!(front.rate.period(), Duration::from_millis(100));
        assert!(!rig.cameras[3].camera_info && rig.cameras[3].sees_car);
    }

    #[test]
    fn mounts_come_from_vehicle_yaml() {
        let rig = rig();
        let rear = &rig.cameras[2];
        assert_eq!(rear.mount.translation, Vec3::new(-0.20, 0.0, 0.90));
        // Turned round: its x looks back.
        assert!(rear.mount.rotation.axis(0).x < -0.99);
    }

    /// The projection test: a point straight ahead of the front left camera, at its own
    /// height, lands on the picture's centre; one 1 m to its left, 10 m on, 34.3 px left.
    #[test]
    fn projects_a_known_point() {
        let rig = rig();
        let cam = &rig.cameras[0];
        let car = Pose::planar(5.0, -3.0, 0.4);
        let m = car.then(&cam.mount);
        let ahead = m.apply(Vec3::new(10.0, 0.0, 0.0));
        let p = cam.project(&car, ahead).unwrap();
        assert!((p.u - 320.0).abs() < 1e-9 && (p.v - 180.0).abs() < 1e-9);
        let left_up = m.apply(Vec3::new(10.0, 1.0, 0.5));
        let p = cam.project(&car, left_up).unwrap();
        assert!((p.u - (320.0 - 34.3)).abs() < 1e-9);
        assert!((p.v - (180.0 - 17.15)).abs() < 1e-9);
        let behind = m.apply(Vec3::new(-3.0, 0.0, 0.0));
        assert!(cam.project(&car, behind).is_none());
    }

    #[test]
    fn missing_mount_is_an_error() {
        let bad = CAMERAS.replace("mount: camera_rear_link", "mount: nowhere_link");
        assert!(matches!(
            Rig::from_yaml(&bad, VEHICLE),
            Err(RigError::NoMount { .. })
        ));
    }
}
