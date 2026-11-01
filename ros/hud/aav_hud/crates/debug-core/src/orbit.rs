//! The 3D view's camera: it orbits a point on the ground (odom, z up), seen from a
//! heading, an elevation and a distance. The mouse turns, tilts, zooms and pans it.
//!
//! Three ways to follow the car (`C` switches):
//!
//! | Mode | The point | The heading |
//! |---|---|---|
//! | Chase | the car | behind the car, turning with it (plus the mouse's turn) |
//! | Follow | the car | fixed |
//! | Free | where it was panned to | fixed |

use std::f32::consts::{FRAC_PI_2, PI};

use map_core::geom::Pose;

/// How the camera follows the car.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Chase,
    Follow,
    Free,
}

impl Mode {
    /// The next one, for the `C` key.
    pub fn next(self) -> Self {
        match self {
            Self::Chase => Self::Follow,
            Self::Follow => Self::Free,
            Self::Free => Self::Chase,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Chase => "CHASE",
            Self::Follow => "FOLLOW",
            Self::Free => "FREE",
        }
    }
}

/// Radians of turn per pixel dragged.
const TURN_PER_PIXEL: f32 = 0.006;
/// The lowest and highest the camera may look from.
const MIN_ELEVATION: f32 = 0.08;
const MAX_ELEVATION: f32 = FRAC_PI_2 - 0.02;
const MIN_DISTANCE: f32 = 2.0;
const MAX_DISTANCE: f32 = 400.0;

/// The camera.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Orbit {
    pub mode: Mode,
    /// The point looked at, odom, metres.
    pub target: [f32; 3],
    /// Chase: radians added to "behind the car". Otherwise: the heading the camera
    /// looks from, radians from odom's x axis.
    pub turn: f32,
    /// Radians above the ground.
    pub elevation: f32,
    /// Metres from the target.
    pub distance: f32,
    /// The car's heading at the last update (for Chase).
    car_yaw: f32,
}

impl Default for Orbit {
    /// Behind the car, a little above: what the driver sees, plus the car.
    fn default() -> Self {
        Self {
            mode: Mode::Chase,
            target: [0.0, 0.0, 0.0],
            turn: 0.0,
            elevation: 0.55,
            distance: 28.0,
            car_yaw: 0.0,
        }
    }
}

impl Orbit {
    /// The heading the camera looks from (the camera sits that way from the target).
    pub fn heading(&self) -> f32 {
        match self.mode {
            Mode::Chase => self.car_yaw + PI + self.turn,
            Mode::Follow | Mode::Free => self.turn,
        }
    }

    /// Where the camera is, odom.
    pub fn position(&self) -> [f32; 3] {
        let (h, e) = (self.heading(), self.elevation);
        let [x, y, z] = self.target;
        [
            x + self.distance * e.cos() * h.cos(),
            y + self.distance * e.cos() * h.sin(),
            z + self.distance * e.sin(),
        ]
    }

    /// Follows the car: in Chase and Follow, the target moves with it.
    pub fn update(&mut self, car: Option<Pose>) {
        let Some(car) = car else { return };
        self.car_yaw = car.yaw as f32;
        if self.mode != Mode::Free {
            self.target = [car.position.x as f32, car.position.y as f32, 0.0];
        }
    }

    /// The next mode, keeping the camera where it is.
    pub fn switch_mode(&mut self) {
        let heading = self.heading();
        self.mode = self.mode.next();
        self.turn = match self.mode {
            Mode::Chase => heading - self.car_yaw - PI,
            Mode::Follow | Mode::Free => heading,
        };
    }

    /// A drag of (dx, dy) pixels: right turns the view round, down tilts it up.
    pub fn rotate(&mut self, dx: f32, dy: f32) {
        self.turn -= dx * TURN_PER_PIXEL;
        self.elevation = (self.elevation + dy * TURN_PER_PIXEL).clamp(MIN_ELEVATION, MAX_ELEVATION);
    }

    /// Closer (factor < 1) or farther.
    pub fn zoom(&mut self, factor: f32) {
        if factor.is_finite() && factor > 0.0 {
            self.distance = (self.distance * factor).clamp(MIN_DISTANCE, MAX_DISTANCE);
        }
    }

    /// A drag of (dx, dy) pixels that moves the ground with the mouse. Leaves Chase and
    /// Follow for Free.
    pub fn pan(&mut self, dx: f32, dy: f32) {
        if self.mode != Mode::Free {
            let heading = self.heading();
            self.mode = Mode::Free;
            self.turn = heading;
        }
        let scale = self.distance * 0.0018;
        let h = self.heading();
        // Forward on the screen is away from the camera: -(cos h, sin h). Right is
        // that turned a quarter clockwise: (-sin h, cos h).
        let (fx, fy) = (-h.cos(), -h.sin());
        let (rx, ry) = (-h.sin(), h.cos());
        self.target[0] += (-dx * rx + dy * fx) * scale;
        self.target[1] += (-dx * ry + dy * fy) * scale;
    }

    /// Back to the default view behind the car.
    pub fn reset(&mut self) {
        let car_yaw = self.car_yaw;
        *self = Self {
            car_yaw,
            ..Self::default()
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    #[test]
    fn chase_sits_behind_the_car() {
        let mut o = Orbit::default();
        o.update(Some(Pose::new(10.0, 0.0, f64::from(FRAC_PI_2))));
        let p = o.position();
        // The car faces north: the camera is south of it, and above.
        assert!(close(p[0], 10.0), "{p:?}");
        assert!(p[1] < -5.0 && p[2] > 2.0, "{p:?}");
        assert_eq!(o.target, [10.0, 0.0, 0.0]);
    }

    #[test]
    fn switching_modes_keeps_the_camera_still() {
        let mut o = Orbit::default();
        o.update(Some(Pose::new(3.0, 4.0, 1.0)));
        let before = o.position();
        for _ in 0..3 {
            o.switch_mode();
            let after = o.position();
            assert!(
                before.iter().zip(after).all(|(a, b)| close(*a, b)),
                "{:?}",
                o.mode
            );
        }
        assert_eq!(o.mode, Mode::Chase);
    }

    #[test]
    fn panning_frees_the_camera_and_moves_the_target() {
        let mut o = Orbit::default();
        o.update(Some(Pose::new(0.0, 0.0, 0.0)));
        o.pan(0.0, 100.0);
        assert_eq!(o.mode, Mode::Free);
        // Camera behind (west of) a car facing east: dragging down pulls the ground
        // toward us, so the target moves away, east.
        assert!(
            o.target[0] > 1.0 && close(o.target[1], 0.0),
            "{:?}",
            o.target
        );
        // Dragging right moves the ground right, so the target goes left (north).
        let y = o.target[1];
        o.pan(100.0, 0.0);
        assert!(o.target[1] > y + 1.0, "{:?}", o.target);
        o.update(Some(Pose::new(50.0, 0.0, 0.0)));
        assert!(o.target[0] < 10.0, "free: the target stays");
    }

    #[test]
    fn tilt_and_zoom_have_limits() {
        let mut o = Orbit::default();
        o.rotate(0.0, 1e6);
        assert!(o.elevation < FRAC_PI_2);
        o.rotate(0.0, -1e6);
        assert!(o.elevation > 0.0);
        o.zoom(1e-6);
        assert_eq!(o.distance, MIN_DISTANCE);
        o.zoom(f32::NAN);
        assert_eq!(o.distance, MIN_DISTANCE);
        o.reset();
        assert_eq!(o.mode, Mode::Chase);
    }
}
