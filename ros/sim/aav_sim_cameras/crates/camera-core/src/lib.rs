//! The simulator's cameras, with no engine and no ROS: lenses and camera_info, mounts,
//! the world they see, when each is due, and the [`CameraRenderer`] trait a renderer
//! (Bevy's, in `sim-cameras`) fills in.
//!
//! How the simulator's core uses it:
//!
//! ```ignore
//! let rig = Rig::from_yaml(CAMERAS_YAML, VEHICLE_YAML)?;
//! let world = SimWorld::test_world();               // or the core's world, converted
//! let renderer = sim_cameras::BevyRenderer::new(&rig, &world)?;
//! let mut cameras = Cameras::new(rig, renderer);
//! loop {
//!     let frames = cameras.tick(&car_in_map, sim_time)?;   // only the cameras due now
//!     // publish each frame on cameras.rig().cameras[frame.camera]'s topics
//! }
//! ```
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod core_world;
pub mod frame;
pub mod geom;
pub mod lens;
pub mod obj;
pub mod pretty;
pub mod rig;
pub mod schedule;
pub mod world;

use std::time::Duration;

pub use frame::Frame;
pub use geom::{Pose, Rotation, Vec3};
pub use lens::{Lens, Pixel};
pub use obj::TriMesh;
pub use pretty::{Detail, Graphics, Surface};
pub use rig::{CameraSpec, Hz, Rig};
pub use schedule::Timer;
pub use world::{Geometry, Look, Rgb, Shape, SimWorld, Sun};

/// Draws pictures of a world. Bevy's is `sim_cameras::BevyRenderer`; a test can use a fake.
pub trait CameraRenderer {
    /// What goes wrong when drawing.
    type Error;

    /// Replaces the world: the next pictures show this one.
    ///
    /// # Errors
    ///
    /// The renderer's error if it can't take the world.
    fn set_world(&mut self, world: &SimWorld) -> Result<(), Self::Error>;

    /// Draws the cameras `which` (indices into the rig it was made with), with the car's
    /// base_link at `car` in the map, and stamps them `stamp`. Blocks until every picture
    /// is back from the GPU. Returns them in `which`'s order.
    ///
    /// # Errors
    ///
    /// The renderer's error if it can't draw or read back a picture.
    fn render(
        &mut self,
        car: &Pose,
        stamp: Duration,
        which: &[usize],
    ) -> Result<Vec<Frame>, Self::Error>;
}

/// The rig, each camera's timer and a renderer: what the simulator's loop calls.
pub struct Cameras<R> {
    rig: Rig,
    timers: Vec<Timer>,
    renderer: R,
}

// Debug by hand: the renderer is the GPU's, left out.
impl<R> std::fmt::Debug for Cameras<R> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Cameras")
            .field("rig", &self.rig)
            .field("timers", &self.timers)
            .finish_non_exhaustive()
    }
}

impl<R: CameraRenderer> Cameras<R> {
    /// The rig's cameras, each with its own timer, drawn by `renderer`.
    pub fn new(rig: Rig, renderer: R) -> Self {
        let timers = rig.cameras.iter().map(|c| Timer::new(c.rate)).collect();
        Self {
            rig,
            timers,
            renderer,
        }
    }

    /// The cameras.
    pub fn rig(&self) -> &Rig {
        &self.rig
    }

    /// The renderer, to change its world.
    pub fn renderer_mut(&mut self) -> &mut R {
        &mut self.renderer
    }

    /// Which cameras are due at `now` (sim time). Moves their timers on.
    pub fn due(&mut self, now: Duration) -> Vec<usize> {
        self.timers
            .iter_mut()
            .enumerate()
            .filter_map(|(i, t)| t.due(now).then_some(i))
            .collect()
    }

    /// The sim time the next picture is due, of any camera: `None` before the first
    /// look. Lockstep (software rendering): the simulator holds its clock there until
    /// the cameras say a later one.
    pub fn next_due(&self) -> Option<Duration> {
        self.timers.iter().map(Timer::next).min().flatten()
    }

    /// Draws the cameras due at `now`, with the car at `car`. Often none.
    ///
    /// # Errors
    ///
    /// The renderer's error if it can't draw the cameras due.
    pub fn tick(&mut self, car: &Pose, now: Duration) -> Result<Vec<Frame>, R::Error> {
        let due = self.due(now);
        if due.is_empty() {
            return Ok(Vec::new());
        }
        self.renderer.render(car, now, &due)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Counts what it's asked to draw.
    struct Fake(Vec<usize>);

    impl CameraRenderer for Fake {
        type Error = std::convert::Infallible;
        fn set_world(&mut self, _: &SimWorld) -> Result<(), Self::Error> {
            Ok(())
        }
        fn render(
            &mut self,
            _: &Pose,
            stamp: Duration,
            which: &[usize],
        ) -> Result<Vec<Frame>, Self::Error> {
            self.0.extend(which);
            Ok(which
                .iter()
                .map(|&camera| Frame {
                    camera,
                    stamp,
                    width: 1,
                    height: 1,
                    rgb: vec![0; 3],
                })
                .collect())
        }
    }

    /// The rate test without a GPU: one simulated minute at the core's 50 Hz gives each
    /// 10 Hz camera exactly 600 pictures, stamped on the tenths.
    #[test]
    fn each_camera_at_its_rate() {
        let rig = Rig::from_yaml(
            include_str!("../../../config/cameras.yaml"),
            include_str!("../../../../../bringup/aav_description/config/vehicle.yaml"),
        )
        .unwrap();
        let n = rig.cameras.len();
        let mut cameras = Cameras::new(rig, Fake(Vec::new()));
        let mut stamps = Vec::new();
        for step in 0..3000u64 {
            let now = Duration::from_millis(step * 20);
            for f in cameras.tick(&Pose::IDENTITY, now).unwrap() {
                if f.camera == 0 {
                    stamps.push(f.stamp);
                }
            }
        }
        let drawn = &cameras.renderer_mut().0;
        for c in 0..n {
            assert_eq!(drawn.iter().filter(|&&d| d == c).count(), 600);
        }
        assert!(stamps.iter().all(|s| s.as_millis() % 100 == 0));
    }

    /// Lockstep: after each look, the cameras say when their next picture is due.
    #[test]
    fn says_its_next_picture() {
        let rig = Rig::from_yaml(
            include_str!("../../../config/cameras.yaml"),
            include_str!("../../../../../bringup/aav_description/config/vehicle.yaml"),
        )
        .unwrap();
        let mut cameras = Cameras::new(rig, Fake(Vec::new()));
        assert_eq!(cameras.next_due(), None);
        cameras.tick(&Pose::IDENTITY, Duration::ZERO).unwrap();
        assert_eq!(cameras.next_due(), Some(Duration::from_millis(100)));
        cameras
            .tick(&Pose::IDENTITY, Duration::from_millis(120))
            .unwrap();
        assert_eq!(cameras.next_due(), Some(Duration::from_millis(200)));
    }
}
