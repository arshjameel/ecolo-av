//! How the follower steers while it keeps the lane: the 2025-26 PID on the offset, or a
//! challenger on the fitted lane centre (pure pursuit, Stanley), each with the car's lag.

use std::fmt;
use std::str::FromStr;

/// One lane line, y = a x² + b x + c, in base_link metres (lane_finder's fit).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line {
    pub a: f64,
    pub b: f64,
    pub c: f64,
}

impl Line {
    /// m left of base_link's x axis, `x` m ahead.
    pub fn y(&self, x: f64) -> f64 {
        (self.a * x + self.b) * x + self.c
    }

    /// dy/dx at `x`.
    pub fn slope(&self, x: f64) -> f64 {
        2.0 * self.a * x + self.b
    }

    /// The same line moved `by` m to its left (right when negative), square to itself.
    fn shifted(&self, by: f64, x: f64) -> f64 {
        self.y(x) + by * (1.0 + self.slope(x).powi(2)).sqrt()
    }
}

/// What the follower reads from one LaneGeometry message.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geometry {
    /// m from the lane's centre, measured ahead (lane_finder's `lookahead`); + = the car
    /// is left of it.
    pub offset: f64,
    /// The bend's radius, m (9999 = straight, or not seen).
    pub mean_radius: f64,
    /// m between the lines.
    pub lane_width: f64,
    pub left_detected: bool,
    pub right_detected: bool,
    pub both_detected: bool,
    /// Each line's fit; only read when its `*_detected` is set.
    pub left: Line,
    pub right: Line,
}

/// m: the lane's width when the message carries none (lane_finder's own default).
const DEFAULT_WIDTH: f64 = 3.5;

impl Geometry {
    /// Any line seen?
    pub fn lanes_ok(&self) -> bool {
        self.left_detected || self.right_detected
    }

    /// The lane centre's y, `x` m ahead, from the lines seen (None: none seen). One line
    /// alone: moved half a lane across, as lane_finder's `centre_y` does.
    pub fn centre(&self, x: f64) -> Option<f64> {
        let half = if self.lane_width > 0.0 {
            self.lane_width / 2.0
        } else {
            DEFAULT_WIDTH / 2.0
        };
        match (self.left_detected, self.right_detected) {
            (true, true) => Some((self.left.y(x) + self.right.y(x)) / 2.0),
            (true, false) => Some(self.left.shifted(-half, x)),
            (false, true) => Some(self.right.shifted(half, x)),
            (false, false) => None,
        }
    }

    /// The lane centre's slope dy/dx, `x` m ahead (a small step either side).
    fn centre_slope(&self, x: f64) -> Option<f64> {
        const H: f64 = 0.05;
        Some((self.centre(x + H)? - self.centre(x - H)?) / (2.0 * H))
    }
}

/// The 2025-26 PID's gains.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidGains {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl Default for PidGains {
    fn default() -> Self {
        PidGains {
            kp: 0.8,
            ki: 0.01,
            kd: 0.3,
        }
    }
}

/// The 2025-26 PID, its dt from the wall clock (control_interface.py's `PID`).
#[derive(Debug, Clone, PartialEq)]
pub struct Pid {
    gains: PidGains,
    integral: f64,
    prev_err: f64,
    prev_time: f64,
}

impl Pid {
    pub fn new(gains: PidGains, wall: f64) -> Self {
        Pid {
            gains,
            integral: 0.0,
            prev_err: 0.0,
            prev_time: wall,
        }
    }

    pub fn compute(&mut self, error: f64, wall: f64) -> f64 {
        let dt = (wall - self.prev_time).max(1e-4);
        self.integral += error * dt;
        let derivative = (error - self.prev_err) / dt;
        let g = self.gains;
        let out = g.kp * error + g.ki * self.integral + g.kd * derivative;
        self.prev_err = error;
        self.prev_time = wall;
        out
    }

    pub fn reset(&mut self, wall: f64) {
        self.integral = 0.0;
        self.prev_err = 0.0;
        self.prev_time = wall;
    }
}

/// Pure pursuit's settings: the look-ahead grows with speed, measured from where the car
/// will be once the command takes effect.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PursuitSettings {
    /// m: the look-ahead at a standstill.
    pub lookahead: f64,
    /// s: look-ahead added per m/s.
    pub lookahead_gain: f64,
    /// m: never nearer or further (the camera's fit covers 4-16 m ahead).
    pub lookahead_min: f64,
    pub lookahead_max: f64,
    /// s: from the picture to the wheels turning (the camera and the steering's lag).
    pub lag: f64,
}

impl Default for PursuitSettings {
    fn default() -> Self {
        PursuitSettings {
            lookahead: 3.0,
            lookahead_gain: 1.0,
            lookahead_min: 4.0,
            lookahead_max: 10.0,
            lag: 0.2,
        }
    }
}

/// Stanley's settings (Thrun et al. 2006): heading plus cross-track at the front axle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StanleySettings {
    /// 1/s: how hard the cross-track error turns the wheels.
    pub gain: f64,
    /// m/s: softens the cross-track term at low speed.
    pub soft_speed: f64,
    /// m: rear axle to front axle (vehicle.yaml).
    pub wheelbase: f64,
    /// rad: the wheels' full lock (vehicle.yaml's max_steering_angle).
    pub max_steer: f64,
    /// s: as pure pursuit's.
    pub lag: f64,
}

impl Default for StanleySettings {
    fn default() -> Self {
        StanleySettings {
            gain: 1.0,
            soft_speed: 1.0,
            wheelbase: 1.561,
            max_steer: 0.5,
            lag: 0.2,
        }
    }
}

/// A steering law's name, as the `steering` parameter and `lane_follower:=` give it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LawName {
    Pid,
    PurePursuit,
    Stanley,
}

impl LawName {
    pub const ALL: [LawName; 3] = [LawName::Pid, LawName::PurePursuit, LawName::Stanley];

    pub fn as_str(self) -> &'static str {
        match self {
            LawName::Pid => "pid",
            LawName::PurePursuit => "pure_pursuit",
            LawName::Stanley => "stanley",
        }
    }
}

impl fmt::Display for LawName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A steering law's name that isn't one.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownLaw(pub String);

impl fmt::Display for UnknownLaw {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "steering {:?}: pick pid, pure_pursuit or stanley",
            self.0
        )
    }
}

impl std::error::Error for UnknownLaw {}

impl FromStr for LawName {
    type Err = UnknownLaw;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let name = s.trim().to_lowercase();
        LawName::ALL
            .into_iter()
            .find(|l| l.as_str() == name)
            .ok_or_else(|| UnknownLaw(s.to_string()))
    }
}

/// How the follower turns while it keeps the lane.
#[derive(Debug, Clone, PartialEq)]
pub enum Law {
    /// The 2025-26 PID on the offset (error = -offset with `invert_offset_sign`).
    Pid { pid: Pid, invert_offset_sign: bool },
    /// Pure pursuit on the lane centre.
    PurePursuit(PursuitSettings),
    /// Stanley on the lane centre.
    Stanley(StanleySettings),
}

/// Where the car will be after `lag` s at `speed` m/s turning at `turn` rad/s:
/// (x, y, heading) in today's base_link.
fn ahead_by_lag(speed: f64, turn: f64, lag: f64) -> (f64, f64, f64) {
    let heading = turn * lag;
    if turn.abs() < 1e-9 {
        (speed * lag, 0.0, 0.0)
    } else {
        let r = speed / turn;
        (r * heading.sin(), r * (1.0 - heading.cos()), heading)
    }
}

/// A point in today's base_link, seen from a pose (x, y, heading) in it.
fn seen_from(pose: (f64, f64, f64), x: f64, y: f64) -> (f64, f64) {
    let (px, py, h) = pose;
    let (dx, dy) = (x - px, y - py);
    (h.cos() * dx + h.sin() * dy, -h.sin() * dx + h.cos() * dy)
}

impl Law {
    pub fn name(&self) -> LawName {
        match self {
            Law::Pid { .. } => LawName::Pid,
            Law::PurePursuit(_) => LawName::PurePursuit,
            Law::Stanley(_) => LawName::Stanley,
        }
    }

    /// The turn rate (rad/s, + = left) for this message at `speed` m/s; `last_turn` is the
    /// turn last commanded (the car is still answering it during the lag). Not clipped.
    pub fn steer(&mut self, g: &Geometry, speed: f64, last_turn: f64, wall: f64) -> f64 {
        match self {
            Law::Pid {
                pid,
                invert_offset_sign,
            } => {
                let error = if *invert_offset_sign {
                    -g.offset
                } else {
                    g.offset
                };
                pid.compute(error, wall)
            }
            Law::PurePursuit(s) => pursuit(s, g, speed, last_turn).unwrap_or(last_turn),
            Law::Stanley(s) => stanley(s, g, speed, last_turn).unwrap_or(last_turn),
        }
    }

    /// Forget the past (a new state): only the PID has one.
    pub fn reset(&mut self, wall: f64) {
        if let Law::Pid { pid, .. } = self {
            pid.reset(wall);
        }
    }
}

/// Pure pursuit: the arc from where the car will be to the lane centre a look-ahead on.
fn pursuit(s: &PursuitSettings, g: &Geometry, speed: f64, last_turn: f64) -> Option<f64> {
    let pose = ahead_by_lag(speed, last_turn, s.lag);
    let reach = (s.lookahead + s.lookahead_gain * speed).clamp(s.lookahead_min, s.lookahead_max);
    let x = pose.0 + reach;
    let (lx, ly) = seen_from(pose, x, g.centre(x)?);
    let d2 = lx * lx + ly * ly;
    (d2 > 1e-6).then(|| speed * 2.0 * ly / d2)
}

/// Stanley: the lane's heading plus the cross-track error at the front axle, from where
/// the car will be.
fn stanley(s: &StanleySettings, g: &Geometry, speed: f64, last_turn: f64) -> Option<f64> {
    let pose = ahead_by_lag(speed, last_turn, s.lag);
    let (h_sin, h_cos) = pose.2.sin_cos();
    let (fx, fy) = (pose.0 + s.wheelbase * h_cos, pose.1 + s.wheelbase * h_sin);
    // The centre's point nearest the front axle, taken square below it in today's x.
    let cy = g.centre(fx)?;
    let heading = g.centre_slope(fx)?.atan() - pose.2;
    let (_, across) = seen_from(pose, fx, cy);
    let (_, axle) = seen_from(pose, fx, fy);
    let error = across - axle;
    let wheel = (heading + (s.gain * error).atan2(s.soft_speed + speed.abs()))
        .clamp(-s.max_steer, s.max_steer);
    Some(speed * wheel.tan() / s.wheelbase)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lane(offset: f64, a: f64) -> Geometry {
        // The car `offset` m left of the centre of a 3.5 m lane that bends with y'' = 2a.
        Geometry {
            offset,
            mean_radius: 9999.0,
            lane_width: 3.5,
            left_detected: true,
            right_detected: true,
            both_detected: true,
            left: Line {
                a,
                b: 0.0,
                c: 1.75 - offset,
            },
            right: Line {
                a,
                b: 0.0,
                c: -1.75 - offset,
            },
        }
    }

    #[test]
    fn names_both_ways() {
        for l in LawName::ALL {
            assert_eq!(l.as_str().parse::<LawName>(), Ok(l));
        }
        assert_eq!(" Stanley ".parse::<LawName>(), Ok(LawName::Stanley));
        assert!("mpc".parse::<LawName>().is_err());
    }

    #[test]
    fn centre_from_one_line_is_half_a_lane_across() {
        let mut g = lane(0.3, 0.0);
        let both = g.centre(6.0).unwrap();
        g.right_detected = false;
        assert!((g.centre(6.0).unwrap() - both).abs() < 1e-12);
        g.left_detected = false;
        g.right_detected = true;
        assert!((g.centre(6.0).unwrap() - both).abs() < 1e-12);
        g.right_detected = false;
        assert_eq!(g.centre(6.0), None);
    }

    #[test]
    fn every_law_steers_back_to_the_centre() {
        for law in [
            Law::Pid {
                pid: Pid::new(PidGains::default(), 0.0),
                invert_offset_sign: true,
            },
            Law::PurePursuit(PursuitSettings::default()),
            Law::Stanley(StanleySettings::default()),
        ] {
            let mut left = law.clone();
            assert!(left.steer(&lane(0.5, 0.0), 2.0, 0.0, 0.1) < 0.0, "{law:?}");
            let mut right = law.clone();
            assert!(
                right.steer(&lane(-0.5, 0.0), 2.0, 0.0, 0.1) > 0.0,
                "{law:?}"
            );
        }
    }

    #[test]
    fn on_the_centre_of_a_bend_the_challengers_follow_it() {
        // A left bend of 50 m radius (y = x² / 100), the car on its centre line.
        let g = lane(0.0, 0.01);
        for mut law in [
            Law::PurePursuit(PursuitSettings {
                lag: 0.0,
                ..PursuitSettings::default()
            }),
            Law::Stanley(StanleySettings {
                lag: 0.0,
                ..StanleySettings::default()
            }),
        ] {
            let turn = law.steer(&g, 2.0, 0.0, 0.0);
            // 2 m/s on 50 m: 0.04 rad/s, the parabola's own bend, give or take.
            assert!((turn - 0.04).abs() < 0.015, "{law:?}: {turn}");
        }
    }

    #[test]
    fn pure_pursuit_on_a_straight_lane_is_its_formula() {
        let s = PursuitSettings {
            lag: 0.0,
            ..PursuitSettings::default()
        };
        // 0.4 m left of the centre, 2 m/s: look 5 m ahead, the centre 0.4 m right.
        let turn = Law::PurePursuit(s).steer(&lane(0.4, 0.0), 2.0, 0.0, 0.0);
        let want = 2.0 * 2.0 * -0.4 / (25.0 + 0.16);
        assert!((turn - want).abs() < 1e-12);
    }

    #[test]
    fn the_lag_counts_the_turn_already_under_way() {
        // Already turning right towards the centre: steer less than from scratch.
        let s = PursuitSettings::default();
        let fresh = Law::PurePursuit(s).steer(&lane(0.4, 0.0), 2.0, 0.0, 0.0);
        let turning = Law::PurePursuit(s).steer(&lane(0.4, 0.0), 2.0, -0.2, 0.0);
        assert!(turning > fresh, "{turning} {fresh}");
    }

    #[test]
    fn stanley_stays_within_full_lock() {
        let s = StanleySettings {
            gain: 100.0,
            ..StanleySettings::default()
        };
        let turn = Law::Stanley(s).steer(&lane(3.0, 0.0), 2.0, 0.0, 0.0);
        assert!((turn - 2.0 * (-0.5f64).tan() / 1.561).abs() < 1e-9);
    }
}
