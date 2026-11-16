//! The ground under the car's four wheels: how high each stands, so the car rides up
//! slopes and tilts on them, and a kerb too high to roll up stops it.
//!
//! | Under a wheel | What happens |
//! |---|---|
//! | the same height, or a slope | the wheel follows it; the car's height, pitch and roll come from its four wheels |
//! | a step up of at most [`Ground::max_step`] | the wheel rolls up it |
//! | a higher step (a 0.3 m kerb) | the car stops, as at a wall |
//! | nothing (off the world's edge) | the wheel keeps its last height |
//!
//! The ground is whatever the rays hit (the world's visuals), found by casting straight
//! down from just above the wheel.

use crate::math::{Pose, Rot3, Vec3};
use crate::scene::{Ray, Scene};

/// How the wheels find the ground.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ground {
    /// m: the highest step a wheel rolls up. Higher stops the car.
    pub max_step: f64,
    pub wheelbase: f64,
    pub track: f64,
}

/// Why the car can't stand where it was asked to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Blocked {
    /// Which wheel: rear right, rear left, front right, front left.
    pub wheel: usize,
    /// m: how high the step in front of it is.
    pub step: f64,
}

/// How far above a wheel's last height the search starts: it sees a step this high.
const LOOK_UP: f64 = 0.6;
/// How far below it the search reaches.
const LOOK_DOWN: f64 = 2.0;

impl Ground {
    /// The wheels' contact points in base_link: rear right, rear left, front right,
    /// front left.
    #[must_use]
    pub fn wheels(&self) -> [Vec3; 4] {
        let (half, wb) = (self.track / 2.0, self.wheelbase);
        [
            Vec3::new(0.0, -half, 0.0),
            Vec3::new(0.0, half, 0.0),
            Vec3::new(wb, -half, 0.0),
            Vec3::new(wb, half, 0.0),
        ]
    }

    /// The four wheels' heights with the car at (x, y, yaw), each from its last height.
    pub fn settle(
        &self,
        scene: &Scene,
        x: f64,
        y: f64,
        yaw: f64,
        last: [f64; 4],
    ) -> Result<[f64; 4], Blocked> {
        let flat = Pose::new(Vec3::new(x, y, 0.0), Rot3::from_yaw(yaw));
        let mut out = last;
        for (i, wheel) in self.wheels().into_iter().enumerate() {
            let p = flat.apply(wheel);
            let top = last[i] + LOOK_UP;
            let ray = Ray {
                origin: Vec3::new(p.x, p.y, top),
                direction: Vec3::new(0.0, 0.0, -1.0),
            };
            if let Some(d) = scene.cast(&ray, LOOK_UP + LOOK_DOWN) {
                let h = top - d;
                if h - last[i] > self.max_step {
                    return Err(Blocked {
                        wheel: i,
                        step: h - last[i],
                    });
                }
                out[i] = h;
            }
        }
        Ok(out)
    }

    /// base_link on the ground under the rear axle, tilted to the four wheels.
    #[must_use]
    pub fn pose(&self, x: f64, y: f64, yaw: f64, h: [f64; 4]) -> Pose {
        let rear = (h[0] + h[1]) / 2.0;
        let front = (h[2] + h[3]) / 2.0;
        let left = (h[1] + h[3]) / 2.0;
        let right = (h[0] + h[2]) / 2.0;
        // REP-103: + pitch is the nose down; + roll is the left side up.
        let pitch = -((front - rear) / self.wheelbase).atan();
        let roll = ((left - right) / self.track).atan();
        Pose::new(Vec3::new(x, y, rear), Rot3::from_rpy(roll, pitch, yaw))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{Mesh, Triangle};

    fn ground() -> Ground {
        Ground {
            max_step: 0.1,
            wheelbase: 1.6,
            track: 1.2,
        }
    }

    /// Flat ground, and a 0.3 m kerb from x = 10 on.
    fn kerb() -> Scene {
        let mut t = Mesh::plane(100.0, 100.0).triangles;
        let block = Mesh::cuboid(Vec3::new(10.0, 100.0, 0.3))
            .transformed(&Pose::new(Vec3::new(15.0, 0.0, 0.15), Rot3::from_yaw(0.0)));
        t.extend(block.triangles);
        Scene::new(t)
    }

    /// A ramp rising 1 m in 10 m along x, from x = 0.
    fn ramp() -> Scene {
        let v = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
        Scene::new(vec![
            Triangle::new(v(0.0, -10.0, 0.0), v(10.0, -10.0, 1.0), v(10.0, 10.0, 1.0)),
            Triangle::new(v(0.0, -10.0, 0.0), v(10.0, 10.0, 1.0), v(0.0, 10.0, 0.0)),
        ])
    }

    #[test]
    fn on_flat_ground_the_car_sits_level() {
        let g = ground();
        let h = g.settle(&kerb(), 0.0, 0.0, 0.3, [0.0; 4]).unwrap();
        assert!(h.iter().all(|z| z.abs() < 1e-9), "{h:?}");
        let pose = g.pose(0.0, 0.0, 0.3, h);
        assert!(pose.position.z.abs() < 1e-9);
    }

    #[test]
    fn a_kerb_stops_the_front_wheels() {
        let g = ground();
        // The front axle 0.1 m past the kerb's face.
        let blocked = g.settle(&kerb(), 8.5, 0.0, 0.0, [0.0; 4]).unwrap_err();
        assert!(blocked.wheel >= 2);
        assert!((blocked.step - 0.3).abs() < 1e-9);
        // Already up on it, the car drives on.
        let up = g.settle(&kerb(), 12.0, 0.0, 0.0, [0.3; 4]).unwrap();
        assert!(up.iter().all(|z| (z - 0.3).abs() < 1e-9));
    }

    /// The wheels where the car first stands, found from a little up (as the simulator's
    /// start does).
    fn standing(scene: &Scene, x: f64, yaw: f64) -> [f64; 4] {
        let anywhere = Ground {
            max_step: f64::INFINITY,
            ..ground()
        };
        anywhere.settle(scene, x, 0.0, yaw, [0.4; 4]).unwrap()
    }

    #[test]
    fn on_a_ramp_the_nose_points_up() {
        let g = ground();
        let mut h = standing(&ramp(), 0.0, 0.0);
        // Up the ramp a little at a time, as the car drives.
        for k in 0..=50 {
            h = g.settle(&ramp(), 0.1 * k as f64, 0.0, 0.0, h).unwrap();
        }
        assert!(
            (h[0] - 0.5).abs() < 1e-6 && (h[2] - 0.66).abs() < 1e-6,
            "{h:?}"
        );
        let pose = g.pose(5.0, 0.0, 0.0, h);
        // Nose up: base_link's x leans up, by the ramp's slope.
        let forward = pose.rotation.apply(Vec3::new(1.0, 0.0, 0.0));
        assert!((forward.z / forward.x - 0.1).abs() < 1e-6, "{forward:?}");
        assert!((pose.position.z - 0.5).abs() < 1e-6);
    }

    #[test]
    fn across_a_ramp_the_high_side_rolls_up() {
        let g = ground();
        let mut h = [0.0; 4];
        let yaw = std::f64::consts::FRAC_PI_2; // facing +y: the left side is at -x
        for k in 0..=50 {
            h = g.settle(&ramp(), 0.1 * k as f64, 0.0, yaw, h).unwrap();
        }
        let pose = g.pose(5.0, 0.0, yaw, h);
        let left = pose.rotation.apply(Vec3::new(0.0, 1.0, 0.0));
        // The left wheels are 1.2 m lower down the ramp: the left side down.
        assert!(left.z < -0.09, "{left:?}");
    }
}
