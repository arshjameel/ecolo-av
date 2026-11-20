//! The geofence: an area the computer's drivers may not leave. The Rust twin of
//! `aav_safety`'s `geofence.py` (the parts the node uses; the launch file's
//! `motion_from` stays in Python).
//!
//! The fence is a [`Circle`] round the car's marked start spot or a site's [`Polygon`], in
//! the map frame. [`Geofence`] takes each global pose and GPS arrival, and says whether
//! twist_mux's geofence lock should be on: when the car's stopping distance, forward or
//! back, straight or at full lock, brings a corner within `margin` (plus the pose's
//! spread) of the edge; when the car is outside (latched); when the pose jumped
//! (latched); and while the pose can't be trusted.
//!
//! The stopping distance is RSS's, with ego's speed cap:
//!
//! ```text
//! d = v*t1 + a*t1^2/2 + v1*(rho - t1) + v1^2 / (2*b)      v1 = min(v + a*rho, max(v, cap))
//! ```
//!
//! Every sum is written in the Python's order, so the answers match it to the bit.
//! Design: docs/designs/2026-10-29_geofence.md. Units: m, s, rad, m/s, m/s^2.

use std::f64::consts::PI;
use std::fmt;

use crate::pyfmt::{cos, f1, fixed, hypot, max, min, repr, sin, sum};

/// m between the points checked along each stopping path.
pub const SAMPLE: f64 = 0.5;

/// A point (x, y), m.
pub type Point = (f64, f64);

/// The geofence's states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// No global pose yet, or waiting for it to settle.
    Starting,
    /// Clear of the edge.
    Ok,
    /// A stopping path comes too close to the edge.
    Near,
    /// The car went outside (latched).
    Outside,
    /// No pose, an unsure pose, or no GPS into the EKF.
    Lost,
    /// The pose jumped (latched).
    Jump,
    /// The backend can't brake to a stop: every program held.
    Coasting,
    /// Wheels off the ground: no fence.
    Stands,
}

impl State {
    /// The word on `/safety/geofence/state`.
    #[must_use]
    pub fn word(self) -> &'static str {
        match self {
            State::Starting => "STARTING",
            State::Ok => "OK",
            State::Near => "NEAR",
            State::Outside => "OUTSIDE",
            State::Lost => "LOST",
            State::Jump => "JUMP",
            State::Coasting => "COASTING",
            State::Stands => "STANDS",
        }
    }
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.word())
    }
}

/// A fence, a setting or a motion that can't be used. The text is the Python's.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigError(pub String);

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ConfigError {}

fn err<T>(text: impl Into<String>) -> Result<T, ConfigError> {
    Err(ConfigError(text.into()))
}

// --- the fence --------------------------------------------------------------------------

/// A round fence: `radius` m round (cx, cy).
#[derive(Debug, Clone, PartialEq)]
pub struct Circle {
    cx: f64,
    cy: f64,
    radius: f64,
}

impl Circle {
    /// # Errors
    /// Unless the centre is finite and the radius finite and above 0.
    pub fn new(cx: f64, cy: f64, radius: f64) -> Result<Self, ConfigError> {
        if !(cx.is_finite() && cy.is_finite() && radius.is_finite()) || radius <= 0.0 {
            return err("a circle needs a finite centre and a radius > 0");
        }
        Ok(Self { cx, cy, radius })
    }

    /// How far (x, y) is inside the fence (m); negative outside.
    #[must_use]
    pub fn depth(&self, x: f64, y: f64) -> f64 {
        self.radius - hypot(x - self.cx, y - self.cy)
    }

    /// `n` points round the circle, for drawing.
    #[must_use]
    pub fn outline(&self, n: usize) -> Vec<Point> {
        (0..n)
            .map(|i| {
                let a = 2.0 * PI * i as f64 / n as f64;
                (
                    self.cx + self.radius * cos(a),
                    self.cy + self.radius * sin(a),
                )
            })
            .collect()
    }
}

/// A fence with corners (x, y), in order round it, either way. Not self-crossing.
#[derive(Debug, Clone, PartialEq)]
pub struct Polygon {
    corners: Vec<Point>,
}

impl Polygon {
    /// # Errors
    /// Fewer than 3 finite corners, or an area under 1 m^2.
    pub fn new(corners: Vec<Point>) -> Result<Self, ConfigError> {
        if corners.len() < 3 || !corners.iter().all(|(x, y)| x.is_finite() && y.is_finite()) {
            return err("a polygon needs at least 3 finite corners");
        }
        let area = sum(corners
            .iter()
            .zip(corners.iter().cycle().skip(1))
            .map(|(&(x0, y0), &(x1, y1))| x0 * y1 - x1 * y0))
            / 2.0;
        if area.abs() < 1.0 {
            return err(format!(
                "the polygon is {} m^2: is it a line?",
                fixed(area.abs(), 2)
            ));
        }
        Ok(Self { corners })
    }

    /// One from [x0, y0, x1, y1, ...], as a ROS parameter holds it.
    ///
    /// # Errors
    /// An odd count, or as [`Polygon::new`].
    pub fn from_flat(values: &[f64]) -> Result<Self, ConfigError> {
        if !values.len().is_multiple_of(2) {
            return err("a polygon parameter needs pairs: x0, y0, x1, y1, ...");
        }
        Self::new(
            values
                .as_chunks::<2>()
                .0
                .iter()
                .map(|&[x, y]| (x, y))
                .collect(),
        )
    }

    fn edges(&self) -> impl Iterator<Item = (Point, Point)> + '_ {
        self.corners
            .iter()
            .copied()
            .zip(self.corners.iter().copied().cycle().skip(1))
    }

    /// Whether (x, y) is inside.
    #[must_use]
    pub fn contains(&self, x: f64, y: f64) -> bool {
        self.edges().fold(false, |inside, ((x0, y0), (x1, y1))| {
            if ((y0 > y) != (y1 > y)) && x < x0 + (y - y0) * (x1 - x0) / (y1 - y0) {
                !inside
            } else {
                inside
            }
        })
    }

    /// How far (x, y) is inside the fence (m); negative outside.
    #[must_use]
    pub fn depth(&self, x: f64, y: f64) -> f64 {
        let nearest = self
            .edges()
            .map(|(a, b)| to_segment(x, y, a, b))
            .reduce(min)
            .unwrap_or(f64::INFINITY);
        if self.contains(x, y) {
            nearest
        } else {
            -nearest
        }
    }

    /// The corners, for drawing.
    #[must_use]
    pub fn outline(&self) -> Vec<Point> {
        self.corners.clone()
    }
}

fn to_segment(x: f64, y: f64, (ax, ay): Point, (bx, by): Point) -> f64 {
    let (dx, dy) = (bx - ax, by - ay);
    let length2 = dx * dx + dy * dy;
    let t = if length2 == 0.0 {
        0.0
    } else {
        max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
    };
    hypot(x - (ax + t * dx), y - (ay + t * dy))
}

/// Either fence.
#[derive(Debug, Clone, PartialEq)]
pub enum Fence {
    /// Round the start spot.
    Circle(Circle),
    /// A site's.
    Polygon(Polygon),
}

impl Fence {
    /// How far (x, y) is inside the fence (m); negative outside.
    #[must_use]
    pub fn depth(&self, x: f64, y: f64) -> f64 {
        match self {
            Fence::Circle(c) => c.depth(x, y),
            Fence::Polygon(p) => p.depth(x, y),
        }
    }

    /// Its outline for drawing: a circle as 36 points.
    #[must_use]
    pub fn outline(&self) -> Vec<Point> {
        match self {
            Fence::Circle(c) => c.outline(36),
            Fence::Polygon(p) => p.outline(),
        }
    }
}

fn py_list(values: &[f64]) -> String {
    let items: Vec<String> = values.iter().map(|v| repr(*v)).collect();
    format!("[{}]", items.join(", "))
}

/// A site's polygon, if it was drawn for the datum the map frame uses now.
///
/// `corners`: map frame [x0, y0, ...]. `site_datum`: the [lat, lon] it was drawn for.
/// `datum`: navsat.yaml's [lat, lon, ...]. A site drawn on another datum is refused.
///
/// # Errors
/// No datum with the site, another datum, or a bad polygon.
pub fn site_fence(
    site: &str,
    corners: &[f64],
    site_datum: &[f64],
    datum: &[f64],
    tolerance: f64,
) -> Result<Polygon, ConfigError> {
    if site_datum.len() < 2 {
        return err(format!("site {site}: no datum [lat, lon] stored with it"));
    }
    if datum.len() < 2
        || site_datum[..2]
            .iter()
            .zip(&datum[..2])
            .any(|(a, b)| (a - b).abs() > tolerance)
    {
        return err(format!(
            "site {site} was drawn for the datum {}, but the map frame uses {}: redraw it, or \
             set the datum back",
            py_list(&site_datum[..2]),
            py_list(&datum[..datum.len().min(2)])
        ));
    }
    Polygon::from_flat(corners)
}

// --- how far the car goes before it stands still ----------------------------------------

/// What the car may do before it stops: from ego.yaml, vehicle.yaml and the backend.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Motion {
    /// s from the edge coming near to the brake biting.
    pub reaction_time: f64,
    /// m/s^2 counted on.
    pub braking: f64,
    /// m/s^2: ego's max_acceleration, while the lock reacts.
    pub max_accel: f64,
    /// m/s forward: ego's cap.
    pub max_speed: f64,
    /// m/s backward.
    pub max_reverse_speed: f64,
    /// 1/m at full lock.
    pub max_curvature: f64,
}

impl Motion {
    /// Check every number, as `geofence.py`'s `Motion` does.
    ///
    /// # Errors
    /// A number that's not finite, negative, or a braking not above 0.
    pub fn checked(self) -> Result<Self, ConfigError> {
        for (name, value) in [
            ("reaction_time", self.reaction_time),
            ("max_accel", self.max_accel),
            ("max_speed", self.max_speed),
            ("max_reverse_speed", self.max_reverse_speed),
            ("max_curvature", self.max_curvature),
        ] {
            if !(value.is_finite() && value >= 0.0) {
                return err(format!("{name} must be a finite number >= 0"));
            }
        }
        if !(self.braking.is_finite() && self.braking > 0.0) {
            return err("braking must be a finite number > 0");
        }
        Ok(self)
    }
}

/// How far the car goes before it stands still (m): RSS, with the speed cap.
#[must_use]
pub fn stopping_distance(speed: f64, rho: f64, accel: f64, brake: f64, cap: f64) -> f64 {
    let v = speed.abs();
    let top = max(v, cap);
    let t1 = if accel > 0.0 {
        min(rho, (top - v) / accel)
    } else {
        0.0
    };
    let v1 = v + accel * t1;
    v * t1 + 0.5 * accel * t1 * t1 + v1 * (rho - t1) + v1 * v1 / (2.0 * brake)
}

/// The pose after `s` m (negative: backward) on curvature `k` (1/m, + left).
#[must_use]
pub fn along_arc(x: f64, y: f64, yaw: f64, s: f64, k: f64) -> (f64, f64, f64) {
    if k.abs() < 1e-9 {
        return (x + s * cos(yaw), y + s * sin(yaw), yaw);
    }
    let turn = k * s;
    (
        x + (sin(yaw + turn) - sin(yaw)) / k,
        y - (cos(yaw + turn) - cos(yaw)) / k,
        yaw + turn,
    )
}

// --- the lock ---------------------------------------------------------------------------

/// The geofence's limits (config/geofence.yaml).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// m kept between the stopping point and the edge.
    pub margin: f64,
    /// m further in before NEAR lets go.
    pub hysteresis: f64,
    /// m: a pose spread (1 sigma) over this can't be trusted.
    pub max_std: f64,
    /// s without a global pose: lost.
    pub max_age: f64,
    /// s without a GPS position into the EKF: lost.
    pub gps_max_age: f64,
    /// s of good poses before the fence is used.
    pub settle_time: f64,
    /// m a pose may move beyond speed x time before it's a jump.
    pub jump_margin: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            margin: 1.0,
            hysteresis: 0.5,
            max_std: 3.0,
            max_age: 1.0,
            gps_max_age: 2.0,
            settle_time: 3.0,
            jump_margin: 2.0,
        }
    }
}

impl Settings {
    /// Check every number, as `geofence.py`'s `Settings` does.
    ///
    /// # Errors
    /// A number that's not finite, or out of its range.
    pub fn checked(self) -> Result<Self, ConfigError> {
        for (name, value) in [
            ("margin", self.margin),
            ("hysteresis", self.hysteresis),
            ("settle_time", self.settle_time),
            ("jump_margin", self.jump_margin),
        ] {
            if !(value.is_finite() && value >= 0.0) {
                return err(format!("{name} must be a finite number >= 0"));
            }
        }
        for (name, value) in [
            ("max_std", self.max_std),
            ("max_age", self.max_age),
            ("gps_max_age", self.gps_max_age),
        ] {
            if !(value.is_finite() && value > 0.0) {
                return err(format!("{name} must be a finite number > 0"));
            }
        }
        Ok(self)
    }
}

/// The global EKF's pose: map frame, base_link's position and heading, forward speed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// rad.
    pub yaw: f64,
    /// m/s along base_link's x, negative reversing.
    pub speed: f64,
    /// m: the position's spread, 1 sigma.
    pub std: f64,
}

impl Pose {
    /// Every number finite and the spread within `max_std`.
    #[must_use]
    pub fn good(&self, max_std: f64) -> bool {
        [self.x, self.y, self.yaw, self.speed, self.std]
            .iter()
            .all(|v| v.is_finite())
            && self.std <= max_std
    }
}

/// What the lock says: on or off, the state, and why.
#[derive(Debug, Clone, PartialEq)]
pub struct Verdict {
    /// The lock is on: every program held.
    pub locked: bool,
    /// The state.
    pub state: State,
    /// Why, for people.
    pub why: String,
}

/// How the geofence runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mode {
    /// A GPS position must keep reaching the EKF (always, but on stands).
    pub gps: bool,
    /// The backend can't brake to a stop: every program stays held.
    pub coasting: bool,
    /// Wheels off the ground: no fence, the lock stays open (said loudly).
    pub stands: bool,
}

impl Default for Mode {
    fn default() -> Self {
        Self {
            gps: true,
            coasting: false,
            stands: false,
        }
    }
}

/// Decide the geofence lock from each pose and each GPS arrival.
#[derive(Debug, Clone)]
pub struct Geofence {
    footprint: Vec<Point>,
    fence: Fence,
    motion: Motion,
    settings: Settings,
    mode: Mode,
    pose: Option<(Pose, f64)>,
    gps_time: Option<f64>,
    good_since: Option<f64>,
    latched: Option<(State, String)>,
    near_std: Option<f64>,
    state: State,
    why: String,
}

impl Geofence {
    /// `footprint`: the car's corners in base_link (vehicle.yaml's body).
    ///
    /// # Errors
    /// No corners, or a motion or settings out of range.
    pub fn new(
        footprint: Vec<Point>,
        fence: Fence,
        motion: Motion,
        settings: Settings,
        mode: Mode,
    ) -> Result<Self, ConfigError> {
        if footprint.is_empty() {
            return err("the footprint needs corners");
        }
        Ok(Self {
            footprint,
            fence,
            motion: motion.checked()?,
            settings: settings.checked()?,
            mode,
            pose: None,
            gps_time: None,
            good_since: None,
            latched: None,
            near_std: None,
            state: State::Starting,
            why: "no global pose yet".to_string(),
        })
    }

    /// The fence.
    #[must_use]
    pub fn fence(&self) -> &Fence {
        &self.fence
    }

    /// The last state and why.
    #[must_use]
    pub fn state(&self) -> (State, &str) {
        (self.state, &self.why)
    }

    /// Take a global pose that arrived at `now`.
    pub fn update(&mut self, now: f64, pose: Pose) {
        let s = self.settings;
        let last = self.pose.replace((pose, now));
        if !pose.good(s.max_std) {
            self.good_since = None;
            return;
        }
        let settled = self
            .good_since
            .is_some_and(|since| now - since >= s.settle_time);
        if self.good_since.is_none() {
            self.good_since = Some(now);
        }
        if let Some((last, last_time)) = last {
            if settled && last.good(s.max_std) && self.latched.is_none() {
                let step = hypot(pose.x - last.x, pose.y - last.y);
                let allowed =
                    max(last.speed.abs(), pose.speed.abs()) * (now - last_time) + s.jump_margin;
                if step > allowed {
                    self.latched = Some((
                        State::Jump,
                        format!(
                            "the global pose jumped {} m in {} s (at most {} m)",
                            f1(step),
                            f1(now - last_time),
                            f1(allowed)
                        ),
                    ));
                }
            }
        }
    }

    /// Take the arrival of a GPS position into the EKF.
    pub fn gps_arrived(&mut self, now: f64) {
        self.gps_time = Some(now);
    }

    /// The stopping distances (m) forward and backward, from |speed|.
    #[must_use]
    pub fn stopping(&self, speed: f64) -> (f64, f64) {
        let m = &self.motion;
        (
            stopping_distance(speed, m.reaction_time, m.max_accel, m.braking, m.max_speed),
            stopping_distance(
                speed,
                m.reaction_time,
                m.max_accel,
                m.braking,
                m.max_reverse_speed,
            ),
        )
    }

    /// The car's corners now, and along every path it could stop on: forward and
    /// backward, straight and at full lock either way, each its stopping distance long,
    /// checked every [`SAMPLE`] m.
    #[must_use]
    pub fn points(&self, pose: &Pose) -> Vec<Point> {
        let mut poses = vec![(pose.x, pose.y, pose.yaw)];
        let k = self.motion.max_curvature;
        let (forward, back) = self.stopping(pose.speed);
        for (d, sign) in [(forward, 1.0), (back, -1.0)] {
            // As Python's max(1, math.ceil(d / SAMPLE)).
            let n = (d / SAMPLE).ceil().max(1.0);
            let count = n as u64;
            for curvature in [-k, 0.0, k] {
                poses.extend((1..=count).map(|i| {
                    along_arc(pose.x, pose.y, pose.yaw, sign * d * i as f64 / n, curvature)
                }));
            }
        }
        poses
            .iter()
            .flat_map(|&(x, y, yaw)| {
                let (c, s) = (cos(yaw), sin(yaw));
                self.footprint
                    .iter()
                    .map(move |&(fx, fy)| (x + fx * c - fy * s, y + fx * s + fy * c))
            })
            .collect()
    }

    /// Whether the lock is on at `now`, the state and why.
    pub fn check(&mut self, now: f64) -> Verdict {
        let s = self.settings;
        if self.mode.stands {
            return self.say(
                false,
                State::Stands,
                "stands mode: NO FENCE. The wheels must be off the ground".to_string(),
            );
        }
        if self.mode.coasting {
            return self.say(
                true,
                State::Coasting,
                "the legacy speed loop's stop isn't measured: no program may drive until test \
                 4.3 measures it (vehicle.yaml open_loop_decel, pi_loop_decel)"
                    .to_string(),
            );
        }
        let Some((pose, pose_time)) = self.pose else {
            return self.say(true, State::Starting, "no global pose yet".to_string());
        };
        if now - pose_time > s.max_age {
            return self.say(
                true,
                State::Lost,
                format!("no global pose for {} s", f1(now - pose_time)),
            );
        }
        if !pose.good(s.max_std) {
            return self.say(
                true,
                State::Lost,
                format!(
                    "the global pose is unsure: {} m (over {} m) or not a number",
                    f1(pose.std),
                    f1(s.max_std)
                ),
            );
        }
        if self.mode.gps {
            let age = match self.gps_time {
                None => Some("none yet".to_string()),
                Some(t) if now - t > s.gps_max_age => Some(format!("none for {} s", f1(now - t))),
                Some(_) => None,
            };
            if let Some(age) = age {
                return self.say(
                    true,
                    State::Lost,
                    format!("no GPS position into the EKF ({age}): it drifts"),
                );
            }
        }
        // A good pose has set good_since.
        let since = self.good_since.unwrap_or(now);
        if now - since < s.settle_time {
            return self.say(
                true,
                State::Starting,
                "waiting for the global pose to settle".to_string(),
            );
        }
        let here = self.fence.depth(pose.x, pose.y);
        if here < 0.0 && !matches!(self.latched, Some((State::Outside, _))) {
            self.latched = Some((
                State::Outside,
                format!("the car went {} m outside the fence", f1(-here)),
            ));
        }
        if let Some((state, why)) = self.latched.clone() {
            let place = if here < 0.0 {
                format!(
                    "{} m outside: a person drives it back in, then clears it",
                    f1(-here)
                )
            } else {
                format!("{} m inside: a person clears it", f1(here))
            };
            return self.say(true, state, format!("{why}; now {place}"));
        }
        let std = match self.near_std {
            None => pose.std,
            Some(near) => max(pose.std, near),
        };
        let nearest = self
            .points(&pose)
            .iter()
            .map(|&(x, y)| self.fence.depth(x, y))
            .reduce(min)
            .unwrap_or(f64::INFINITY);
        let need = s.margin
            + std
            + if self.near_std.is_some() {
                s.hysteresis
            } else {
                0.0
            };
        if nearest < need {
            if self.near_std.is_none() {
                self.near_std = Some(pose.std);
            }
            let (forward, back) = self.stopping(pose.speed);
            return self.say(
                true,
                State::Near,
                format!(
                    "a stopping path ({} m ahead, {} m back) comes {} m from the edge (needs {} \
                     m) at {} m/s",
                    f1(forward),
                    f1(back),
                    f1(nearest),
                    f1(need),
                    f1(pose.speed)
                ),
            );
        }
        self.near_std = None;
        self.say(
            false,
            State::Ok,
            format!("{} m from the edge at {} m/s", f1(nearest), f1(pose.speed)),
        )
    }

    /// Forget OUTSIDE or JUMP: a person says the car is back in. Refused while not.
    /// Returns (success, message).
    pub fn clear(&mut self, now: f64) -> (bool, String) {
        let Some((state, _)) = &self.latched else {
            return (true, "nothing to clear".to_string());
        };
        let state = *state;
        let pose = match self.pose {
            Some((pose, t))
                if (now - t <= self.settings.max_age || (now - t).is_nan())
                    && pose.good(self.settings.max_std) =>
            {
                pose
            }
            _ => {
                return (
                    false,
                    "no good global pose: the fence cannot say where the car is".to_string(),
                )
            }
        };
        let here = self.fence.depth(pose.x, pose.y);
        if here < 0.0 {
            return (
                false,
                format!("the car is still {} m outside the fence", f1(-here)),
            );
        }
        self.latched = None;
        (
            true,
            format!("cleared {state}: the car is {} m inside", f1(here)),
        )
    }

    fn say(&mut self, locked: bool, state: State, why: String) -> Verdict {
        self.state = state;
        self.why.clone_from(&why);
        Verdict { locked, state, why }
    }
}

/// The car's body outline in base_link, front-left first (`vehicle.footprint_points`).
#[must_use]
pub fn footprint_points(length: f64, width: f64, rear_overhang: f64) -> Vec<Point> {
    let rear = -rear_overhang;
    let front = rear + length;
    let half = width / 2.0;
    vec![(front, half), (front, -half), (rear, -half), (rear, half)]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn motion() -> Motion {
        Motion {
            reaction_time: 0.5,
            braking: 1.0,
            max_accel: 1.0,
            max_speed: 2.0,
            max_reverse_speed: 1.0,
            max_curvature: 0.3,
        }
    }

    #[test]
    fn stopping_from_rest_and_at_the_cap() {
        // From rest: 0.5 s at 1 m/s^2 then braking from 0.5 m/s at 1 m/s^2.
        let d = stopping_distance(0.0, 0.5, 1.0, 1.0, 2.0);
        assert!((d - (0.125 + 0.125)).abs() < 1e-12);
        // At the cap it can't speed up.
        let d = stopping_distance(2.0, 0.5, 1.0, 1.0, 2.0);
        assert!((d - (1.0 + 2.0)).abs() < 1e-12);
    }

    #[test]
    fn circle_and_square_depth() {
        let c = Circle::new(0.0, 0.0, 10.0).expect("circle");
        assert!((c.depth(3.0, 4.0) - 5.0).abs() < 1e-12);
        let p = Polygon::from_flat(&[0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0]).expect("sq");
        assert!((p.depth(5.0, 1.0) - 1.0).abs() < 1e-12);
        assert!((p.depth(-2.0, 5.0) + 2.0).abs() < 1e-12);
        assert!(Polygon::from_flat(&[0.0, 0.0, 1.0, 0.0, 2.0, 0.0]).is_err());
    }

    #[test]
    fn outside_latches_until_cleared_inside() {
        let fence = Fence::Circle(Circle::new(0.0, 0.0, 30.0).expect("circle"));
        let mut g = Geofence::new(
            footprint_points(2.0, 1.0, 0.3),
            fence,
            motion(),
            Settings::default(),
            Mode::default(),
        )
        .expect("geofence");
        let at = |x| Pose {
            x,
            y: 0.0,
            yaw: 0.0,
            speed: 0.0,
            std: 0.1,
        };
        assert_eq!(g.check(0.0).state, State::Starting);
        for i in 0..40 {
            let t = f64::from(i) * 0.1;
            g.gps_arrived(t);
            g.update(t, at(0.0));
        }
        assert_eq!(g.check(4.0).state, State::Ok);
        g.update(4.1, at(0.5));
        g.gps_arrived(4.1);
        // A slow walk out, without jumps.
        let mut x = 0.5;
        let mut t = 4.1;
        while x < 31.0 {
            x += 0.15;
            t += 0.1;
            g.update(t, at(x));
            g.gps_arrived(t);
        }
        assert_eq!(g.check(t).state, State::Outside);
        assert!(!g.clear(t).0);
        g.update(t + 0.1, at(0.0));
        assert_eq!(g.check(t + 0.1).state, State::Outside);
        assert!(g.clear(t + 0.1).0);
        assert_eq!(g.clear(t + 0.1).1, "nothing to clear");
    }

    #[test]
    fn repr_of_a_datum() {
        assert_eq!(repr(45.386), "45.386");
        assert_eq!(repr(0.0), "0.0");
        assert_eq!(repr(-75.0), "-75.0");
    }
}
