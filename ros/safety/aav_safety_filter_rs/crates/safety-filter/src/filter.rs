//! The filter: an automatic driver's (v, w) in, the nearest safe (v, w) out.
//!
//! ```text
//!  driver (v, w) ─► 1 speed barrier ─► 2 keep the curve ─► 3 Lyapunov pull ─► 4 lane ∩ steering ─► (v, w)
//!                   v ≤ v_allow         w_d = w·v_out/v     w toward the line   clip to [lo, hi]
//! ```
//!
//! | Step | Keeps | Maths |
//! |---|---|---|
//! | 1 | room to stop before the road's end | CBF on `h_s = room − M − S(v)` |
//! | 2 | the driver's chosen curve | curvature `w / v` held |
//! | 3 | the car settling on the lane's line (soft) | CLF `V = ½·p²`, with a slack |
//! | 4 | the car inside its lane; the wheels' real reach | CBFs on `p = e_y + ℓ·sin e_ψ`; `|w| ≤ v·tan δ / L` |
//!
//! Every step is a minimum-change problem in one unknown, so each answer is a formula:
//! no solver, the same answer every time (docs/designs, the safety filter).

use std::fmt;

use ego_core::stop_law::StopCurve;

use crate::lane::Frame;

/// A velocity command: forward speed and turn rate.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Command {
    /// m/s, + forward.
    pub v: f64,
    /// rad/s, + left.
    pub w: f64,
}

/// The filter's settings. Units: m, s, rad.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// The stopping law: the same curve ego and the behaviour planner use.
    pub curve: StopCurve,
    /// m kept clear past the stopping distance.
    pub margin: f64,
    /// m: further than this from the line, the car has lost its road: the frame means
    /// nothing there (the nearest point jumps), so it may not go forward.
    pub max_offset: f64,
    /// 1/s: how fast the speed barrier may close (larger: later, harder trims).
    pub alpha_speed: f64,
    /// m: the look-ahead point's distance in front of `base_link`.
    pub look_ahead: f64,
    /// 1/s: how fast the lane barriers may close.
    pub alpha_lane: f64,
    /// m: half the car's width.
    pub half_width: f64,
    /// m kept inside each lane line.
    pub lane_margin: f64,
    /// 1/s: the decay the Lyapunov term asks for (`V̇ ≤ −λ·V`).
    pub clf_rate: f64,
    /// How much the Lyapunov term may trim (0: off; larger: a stronger pull to the line).
    pub clf_weight: f64,
    /// m: the wheelbase.
    pub wheelbase: f64,
    /// rad: the steering's reach, either way.
    pub max_steer: f64,
    /// m/s: the fastest forward.
    pub max_speed: f64,
    /// m/s: the fastest backwards (a positive number).
    pub max_reverse: f64,
}

/// Why settings were refused.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SettingsError {
    /// This setting must be finite and above 0.
    Positive(&'static str, f64),
    /// This setting must be finite and not negative.
    NotNegative(&'static str, f64),
}

impl fmt::Display for SettingsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SettingsError::Positive(name, v) => {
                write!(f, "{name} must be a finite number above 0 (got {v})")
            }
            SettingsError::NotNegative(name, v) => {
                write!(f, "{name} must be a finite number, not negative (got {v})")
            }
        }
    }
}

impl std::error::Error for SettingsError {}

impl Settings {
    /// The settings, checked.
    ///
    /// # Errors
    ///
    /// [`SettingsError`] for the first number out of its range.
    pub fn check(self) -> Result<Self, SettingsError> {
        let positive = [
            ("alpha_speed", self.alpha_speed),
            ("look_ahead", self.look_ahead),
            ("alpha_lane", self.alpha_lane),
            ("wheelbase", self.wheelbase),
            ("max_steer", self.max_steer),
            ("max_speed", self.max_speed),
            ("max_offset", self.max_offset),
        ];
        for (name, v) in positive {
            if !(v.is_finite() && v > 0.0) {
                return Err(SettingsError::Positive(name, v));
            }
        }
        let not_negative = [
            ("margin", self.margin),
            ("half_width", self.half_width),
            ("lane_margin", self.lane_margin),
            ("clf_rate", self.clf_rate),
            ("clf_weight", self.clf_weight),
            ("max_reverse", self.max_reverse),
        ];
        for (name, v) in not_negative {
            if !(v.is_finite() && v >= 0.0) {
                return Err(SettingsError::NotNegative(name, v));
            }
        }
        Ok(self)
    }

    /// 1/m: the tightest curve the wheels reach.
    #[must_use]
    pub fn max_curvature(&self) -> f64 {
        self.max_steer.tan() / self.wheelbase
    }
}

/// What the filter knows this tick, besides the command.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Inputs {
    /// The car in its lane, from a fresh road ahead and a fresh pose; `None`: stale or
    /// missing, so the car may not go forward.
    pub frame: Option<Frame>,
    /// m/s, the measured speed; `None`: stale or missing (the last output stands in).
    pub speed: Option<f64>,
    /// s since the last command (clamped to 0..0.2 s).
    pub dt: f64,
}

/// A rule that changed the command (or would have, had it been the binding one).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rule {
    /// No fresh road ahead or pose, or the car is too far off its line: no forward motion.
    NoRoad,
    /// The forward speed cap.
    MaxSpeed,
    /// The reverse speed cap.
    MaxReverse,
    /// The speed barrier: room to stop.
    SpeedBarrier,
    /// The left lane barrier.
    LaneLeft,
    /// The right lane barrier.
    LaneRight,
    /// The wheels' reach.
    Steering,
    /// The Lyapunov pull toward the lane's line.
    Lyapunov,
    /// The lane barriers and the wheels' reach disagree: steering is kept, the car slows.
    LaneInfeasible,
}

impl fmt::Display for Rule {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Rule::NoRoad => "no_road",
            Rule::MaxSpeed => "max_speed",
            Rule::MaxReverse => "max_reverse",
            Rule::SpeedBarrier => "speed_barrier",
            Rule::LaneLeft => "lane_left",
            Rule::LaneRight => "lane_right",
            Rule::Steering => "steering",
            Rule::Lyapunov => "lyapunov",
            Rule::LaneInfeasible => "lane_infeasible",
        })
    }
}

/// Everything the filter worked out this tick: logged on every command, so a drive can be
/// replayed and scored from its bag.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Report {
    /// The driver's command.
    pub input: Command,
    /// What goes on.
    pub output: Command,
    /// m/s: the speed the barrier judged (the faster of measured and last sent).
    pub speed_now: f64,
    /// m: the stopping law's prediction, `S(speed_now)`: how far the car needs to stop
    /// from here. Logged, so a recorded stop can be checked against it.
    pub stop_distance: f64,
    /// m: the speed barrier's value (room to spare after stopping); NaN: not checked.
    pub h_speed: f64,
    /// m/s: the most forward speed the barrier allows this tick.
    pub v_allow: f64,
    /// m: room before the left lane line, at the look-ahead point; NaN: not checked.
    pub h_left: f64,
    /// m: room before the right lane line; NaN: not checked.
    pub h_right: f64,
    /// rad/s: the lane and steering interval for w; NaN: not checked.
    pub w_low: f64,
    /// rad/s.
    pub w_high: f64,
    /// m²: the Lyapunov function `V = ½·p²`; NaN: not checked.
    pub lyapunov: f64,
    /// m²/s: `V̇` with the output, by the model; NaN: not checked.
    pub lyapunov_rate: f64,
    /// m²/s: how far `V̇ ≤ −λ·V` was relaxed (0: met).
    pub slack: f64,
    /// The rules that changed the command, in the order they acted.
    pub rules: Vec<Rule>,
}

impl Report {
    /// Did any rule change the command?
    #[must_use]
    pub fn trimmed(&self) -> bool {
        self.input != self.output
    }
}

/// The filter, with the one thing it remembers: the last command it sent.
#[derive(Debug, Clone, PartialEq)]
pub struct SafetyFilter {
    settings: Settings,
    last: Command,
}

/// The tick's length, kept sane: a long gap must not let the speed jump.
const MAX_DT: f64 = 0.2;
/// Below this forward speed (m/s) the lane and Lyapunov rules don't act: the car barely
/// moves, and ego holds the wheels where they are.
const CREEP: f64 = 0.02;

impl SafetyFilter {
    /// A filter that has sent nothing yet.
    #[must_use]
    pub fn new(settings: Settings) -> Self {
        SafetyFilter {
            settings,
            last: Command::default(),
        }
    }

    /// The settings in use.
    #[must_use]
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// The last command sent.
    #[must_use]
    pub fn last(&self) -> Command {
        self.last
    }

    /// One tick: the driver's command in, the filtered command and why out.
    pub fn step(&mut self, cmd: Command, inputs: &Inputs) -> Report {
        let report = filter(&self.settings, self.last, cmd, inputs);
        self.last = report.output;
        report
    }
}

/// The filter as a pure function of the last output, the command and the inputs.
#[must_use]
pub fn filter(s: &Settings, last: Command, cmd: Command, inputs: &Inputs) -> Report {
    let dt = if inputs.dt.is_finite() {
        inputs.dt.clamp(0.0, MAX_DT)
    } else {
        0.0
    };
    let mut r = Report {
        input: cmd,
        output: cmd,
        h_speed: f64::NAN,
        v_allow: f64::NAN,
        h_left: f64::NAN,
        h_right: f64::NAN,
        w_low: f64::NAN,
        w_high: f64::NAN,
        lyapunov: f64::NAN,
        lyapunov_rate: f64::NAN,
        ..Report::default()
    };
    // A command that isn't a number is no command: stop.
    if !(cmd.v.is_finite() && cmd.w.is_finite()) {
        r.output = Command::default();
        r.rules.push(Rule::NoRoad);
        return r;
    }
    let measured = inputs.speed.filter(|v| v.is_finite()).map(f64::abs);
    let speed_now = measured.unwrap_or(0.0).max(last.v.abs());
    r.speed_now = speed_now;
    r.stop_distance = s.curve.distance(speed_now);

    if cmd.v < 0.0 {
        // Backwards: the road ahead says nothing behind the car. Only the caps (ego's rule
        // too); reversing is slow on purpose, the radar faces forward.
        let v = cmd.v.max(-s.max_reverse);
        if v != cmd.v {
            r.rules.push(Rule::MaxReverse);
        }
        r.output = Command {
            v,
            w: keep_curve(cmd, v),
        };
        let reach = v.abs() * s.max_curvature();
        if r.output.w.abs() > reach {
            r.output.w = r.output.w.clamp(-reach, reach);
            r.rules.push(Rule::Steering);
        }
        return r;
    }

    // 1. The speed barrier.
    let mut v = cmd.v;
    if v > s.max_speed {
        v = s.max_speed;
        r.rules.push(Rule::MaxSpeed);
    }
    let frame = inputs.frame.filter(|f| f.offset.abs() <= s.max_offset);
    let Some(frame) = frame else {
        if v > 0.0 {
            r.rules.push(Rule::NoRoad);
        }
        r.output = Command {
            v: 0.0,
            w: if v > 0.0 { 0.0 } else { cmd.w },
        };
        r.v_allow = 0.0;
        return r;
    };
    let (h, allow) = speed_barrier(s, frame.room, speed_now, closing(&frame), dt);
    r.h_speed = h;
    r.v_allow = allow;
    if v > allow {
        v = allow;
        r.rules.push(Rule::SpeedBarrier);
    }

    // 2. The driver's curve, at the new speed.
    let wanted = keep_curve(cmd, v);
    if v < CREEP {
        r.output = Command { v, w: wanted };
        return r;
    }

    // 3 and 4. The lane, the Lyapunov pull and the wheels.
    let reach = v * s.max_curvature();
    let (mut low, mut high) = (-reach, reach);
    let lane = lane_bounds(s, &frame, v);
    if let Some(b) = lane {
        r.h_left = b.h_left;
        r.h_right = b.h_right;
    }
    let mut w = wanted;
    if let (Some(b), true) = (lane, s.clf_weight > 0.0) {
        let pulled = lyapunov_pull(s, &frame, v, w, b.c);
        if (pulled - w).abs() > 1e-12 {
            r.rules.push(Rule::Lyapunov);
        }
        w = pulled;
    }
    let mut infeasible = false;
    if let Some(b) = lane {
        if b.low > high || b.high < low {
            // The lane can't be kept with the wheels' reach: keep the reach (physics),
            // steer as near the lane as it allows, and slow down.
            infeasible = true;
            r.rules.push(Rule::LaneInfeasible);
            let toward = if b.low > high { high } else { low };
            low = toward;
            high = toward;
        } else {
            if b.high < high {
                high = b.high;
                if w > high {
                    r.rules.push(Rule::LaneLeft);
                }
            }
            if b.low > low {
                low = b.low;
                if w < low {
                    r.rules.push(Rule::LaneRight);
                }
            }
        }
    }
    if w > high || w < low {
        if !r.rules.contains(&Rule::LaneLeft) && !r.rules.contains(&Rule::LaneRight) {
            r.rules.push(Rule::Steering);
        }
        w = w.clamp(low, high);
    }
    r.w_low = low;
    r.w_high = high;
    if infeasible {
        v = v.min((speed_now - s.curve.a() * dt).max(0.0));
    }
    r.output = Command { v, w };
    if let Some(b) = lane {
        let p = b.p;
        r.lyapunov = 0.5 * p * p;
        r.lyapunov_rate = p * p_rate(&frame, v, w, b.c);
        let bound = -s.clf_rate * r.lyapunov;
        r.slack = (r.lyapunov_rate - bound).max(0.0);
    }
    r
}

/// The turn rate that keeps the command's curve at speed `v`.
fn keep_curve(cmd: Command, v: f64) -> f64 {
    if cmd.v.abs() > 1e-9 {
        cmd.w * v / cmd.v
    } else {
        cmd.w
    }
}

/// How fast the room shrinks per m/s of speed: `ṡ = v·cos e_ψ / (1 − κ·e_y)`.
///
/// On the line it is 1. Off the line on the inside of a bend the foot runs ahead of the
/// car (more than 1): the barrier must use it, or it trusts room that is going faster than
/// the car. Never below 1 (going slower toward the end is no reason to trust more room),
/// and capped at 5 where the formula blows up (near the bend's centre).
#[must_use]
pub fn closing(f: &Frame) -> f64 {
    let d = 1.0 - f.curvature * f.offset;
    let k = if d > 0.2 { f.twist.cos() / d } else { 5.0 };
    k.clamp(1.0, 5.0)
}

/// (h_s, the fastest forward speed allowed this tick); `closing` from [`closing`].
///
/// ```text
/// h_s = room − M − S(v)            S(v) = v·τ + v²/(2a)       (the stopping law)
/// ḣ_s = −k·v − S'(v)·v̇             S'(v) = τ + v/a            (fixed stops; k: closing)
/// CBF:  ḣ_s ≥ −α·h_s   ⇔   v̇ ≤ (α·h_s − k·v) / (τ + v/a)
/// ```
#[must_use]
pub fn speed_barrier(s: &Settings, room: f64, speed_now: f64, closing: f64, dt: f64) -> (f64, f64) {
    let c = &s.curve;
    let h = room - s.margin - c.distance(speed_now);
    if h.is_nan() {
        return (h, 0.0);
    }
    if h == f64::INFINITY {
        return (h, s.max_speed);
    }
    if h <= 0.0 {
        return (h, 0.0);
    }
    let slope = c.tau() + speed_now / c.a();
    let accel = (s.alpha_speed * h - closing * speed_now) / slope;
    (h, (speed_now + accel * dt).clamp(0.0, s.max_speed))
}

/// The lane's bounds on w at speed v.
#[derive(Debug, Clone, Copy)]
struct LaneBounds {
    /// The look-ahead point's offset: `p = e_y + ℓ·sin e_ψ` (m, + left).
    p: f64,
    /// `ℓ·cos e_ψ`: how much w moves p.
    c: f64,
    h_left: f64,
    h_right: f64,
    low: f64,
    high: f64,
}

/// How fast the look-ahead point moves sideways: `ṗ = v·sin e_ψ + ℓ·cos e_ψ·(w − v·κ)`.
fn p_rate(f: &Frame, v: f64, w: f64, c: f64) -> f64 {
    v * f.twist.sin() + c * (w - v * f.curvature)
}

/// The lane barriers as an interval for w; `None` without known edges.
///
/// ```text
/// h_L = room_L − p ≥ 0,   h_R = room_R + p ≥ 0          room = edge − half width − margin
/// CBF:  ṗ ≤ α·h_L  and  ṗ ≥ −α·h_R
///   ⇔   v·κ + (−α·h_R − v·sin e_ψ)/c  ≤  w  ≤  v·κ + (α·h_L − v·sin e_ψ)/c
/// ```
///
/// The look-ahead point makes the barrier "relative degree one": w moves p at once,
/// where the offset itself only moves once the heading has turned.
fn lane_bounds(s: &Settings, f: &Frame, v: f64) -> Option<LaneBounds> {
    if !(f.left.is_finite() && f.right.is_finite()) {
        return None;
    }
    let l = s.look_ahead;
    let p = f.offset + l * f.twist.sin();
    // Facing across or back along the lane, w barely moves p: hold a floor so the bound
    // stays finite (the car then mostly slows, via the infeasible branch).
    let c = (l * f.twist.cos()).max(0.1 * l);
    let room_left = f.left - s.half_width - s.lane_margin;
    let room_right = f.right - s.half_width - s.lane_margin;
    let h_left = room_left - p;
    let h_right = room_right + p;
    let drift = v * f.twist.sin();
    let base = v * f.curvature;
    Some(LaneBounds {
        p,
        c,
        h_left,
        h_right,
        low: base + (-s.alpha_lane * h_right - drift) / c,
        high: base + (s.alpha_lane * h_left - drift) / c,
    })
}

/// The Lyapunov term: the w nearest `w_d` that makes `V = ½·p²` fall at `λ`, with a slack.
///
/// ```text
/// minimise (w − w_d)² + q·δ²   subject to   V̇ = p·ṗ ≤ −λ·V + δ,  δ ≥ 0
/// V̇ is a·w − k with a = p·c:   the constraint is  a·w ≤ b + δ,  b = k − λ·V
/// met already: w = w_d.   Else (KKT, the constraint active):  w = (w_d + q·a·b) / (1 + q·a²)
/// ```
fn lyapunov_pull(s: &Settings, f: &Frame, v: f64, w_d: f64, c: f64) -> f64 {
    let p = f.offset + s.look_ahead * f.twist.sin();
    let a = p * c;
    if a.abs() < 1e-12 {
        return w_d;
    }
    let v_lyap = 0.5 * p * p;
    // V̇ = p·(v·sin e_ψ + c·(w − v·κ)) = a·w + p·(v·sin e_ψ − c·v·κ)
    let rest = p * (v * f.twist.sin() - c * v * f.curvature);
    let b = -s.clf_rate * v_lyap - rest;
    if a * w_d <= b {
        return w_d;
    }
    let q = s.clf_weight;
    (w_d + q * a * b) / (1.0 + q * a * a)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settings() -> Settings {
        Settings {
            curve: StopCurve::new(1.0, 2.0, 0.35).unwrap(),
            margin: 0.5,
            max_offset: 3.0,
            alpha_speed: 1.0,
            look_ahead: 1.5,
            alpha_lane: 1.0,
            half_width: 0.5,
            lane_margin: 0.1,
            clf_rate: 0.5,
            clf_weight: 0.0,
            wheelbase: 1.561,
            max_steer: 0.5,
            max_speed: 2.0,
            max_reverse: 0.5,
        }
        .check()
        .unwrap()
    }

    fn frame(room: f64, offset: f64, twist: f64, width: f64) -> Frame {
        Frame {
            along: 0.0,
            offset,
            twist,
            curvature: 0.0,
            room,
            left: width,
            right: width,
        }
    }

    fn inputs(f: Option<Frame>, speed: f64) -> Inputs {
        Inputs {
            frame: f,
            speed: Some(speed),
            dt: 0.05,
        }
    }

    #[test]
    fn plenty_of_room_changes_nothing() {
        let s = settings();
        let cmd = Command { v: 1.0, w: 0.05 };
        let r = filter(&s, cmd, cmd, &inputs(Some(frame(50.0, 0.0, 0.0, 1.5)), 1.0));
        assert_eq!(r.output, cmd);
        assert!(r.rules.is_empty(), "{:?}", r.rules);
        assert!(!r.trimmed());
    }

    #[test]
    fn no_road_means_no_forward_motion() {
        let s = settings();
        let r = filter(
            &s,
            Command::default(),
            Command { v: 1.0, w: 0.2 },
            &inputs(None, 0.0),
        );
        assert_eq!(r.output, Command::default());
        assert_eq!(r.rules, vec![Rule::NoRoad]);
    }

    #[test]
    fn the_end_of_the_road_stops_the_car() {
        let s = settings();
        let last = Command { v: 1.0, w: 0.0 };
        let r = filter(
            &s,
            last,
            last,
            &inputs(Some(frame(0.8, 0.0, 0.0, 1.5)), 1.0),
        );
        assert_eq!(r.output.v, 0.0);
        assert!(r.h_speed < 0.0);
        assert!(r.rules.contains(&Rule::SpeedBarrier));
    }

    #[test]
    fn the_barrier_trims_before_it_must() {
        // h positive but small: the speed may not rise, it falls.
        let s = settings();
        let last = Command { v: 1.0, w: 0.0 };
        let room = s.margin + s.curve.distance(1.0) + 0.3;
        let r = filter(
            &s,
            last,
            Command { v: 1.5, w: 0.0 },
            &inputs(Some(frame(room, 0.0, 0.0, 1.5)), 1.0),
        );
        assert!(r.output.v < 1.0, "{}", r.output.v);
        assert!(r.output.v > 0.9);
    }

    #[test]
    fn a_slower_command_passes() {
        let s = settings();
        let last = Command { v: 1.0, w: 0.0 };
        let room = s.margin + s.curve.distance(1.0) + 0.3;
        let cmd = Command { v: 0.2, w: 0.0 };
        let r = filter(
            &s,
            last,
            cmd,
            &inputs(Some(frame(room, 0.0, 0.0, 1.5)), 1.0),
        );
        assert_eq!(r.output, cmd);
    }

    #[test]
    fn the_curve_is_kept_when_the_speed_is_trimmed() {
        let s = settings();
        let cmd = Command { v: 3.0, w: 0.3 };
        let r = filter(
            &s,
            Command::default(),
            cmd,
            &inputs(Some(frame(50.0, 0.0, 0.0, 9.0)), 0.0),
        );
        let k_in = cmd.w / cmd.v;
        let k_out = r.output.w / r.output.v;
        assert!((k_in - k_out).abs() < 1e-12);
    }

    #[test]
    fn steering_toward_the_left_line_is_trimmed() {
        let s = settings();
        let last = Command { v: 1.0, w: 0.0 };
        // Near the left line, told to turn hard left.
        let f = frame(50.0, 0.8, 0.0, 1.5);
        let r = filter(&s, last, Command { v: 1.0, w: 0.6 }, &inputs(Some(f), 1.0));
        assert!(r.output.w < 0.6);
        assert!(r.rules.contains(&Rule::LaneLeft), "{:?}", r.rules);
        // Turning away is fine.
        let away = filter(&s, last, Command { v: 1.0, w: -0.2 }, &inputs(Some(f), 1.0));
        assert_eq!(away.output.w, -0.2);
    }

    #[test]
    fn the_wheels_reach_is_kept() {
        let s = settings();
        let last = Command { v: 1.0, w: 0.0 };
        let r = filter(
            &s,
            last,
            Command { v: 1.0, w: 5.0 },
            &inputs(Some(frame(50.0, 0.0, 0.0, 9.0)), 1.0),
        );
        assert!((r.output.w - s.max_curvature()).abs() < 1e-9);
        assert!(r.rules.contains(&Rule::Steering));
    }

    #[test]
    fn the_lyapunov_term_pulls_toward_the_line() {
        let mut s = settings();
        s.clf_weight = 5.0;
        let last = Command { v: 1.0, w: 0.0 };
        let f = frame(50.0, 0.4, 0.0, 1.5);
        let r = filter(&s, last, Command { v: 1.0, w: 0.0 }, &inputs(Some(f), 1.0));
        assert!(
            r.output.w < 0.0,
            "turns right, toward the line: {}",
            r.output.w
        );
        assert!(r.rules.contains(&Rule::Lyapunov));
        s.clf_weight = 0.0;
        let off = filter(&s, last, Command { v: 1.0, w: 0.0 }, &inputs(Some(f), 1.0));
        assert_eq!(off.output.w, 0.0);
    }

    #[test]
    fn backwards_is_only_capped() {
        let s = settings();
        let r = filter(
            &s,
            Command::default(),
            Command { v: -2.0, w: 0.0 },
            &inputs(None, 0.0),
        );
        assert_eq!(r.output.v, -0.5);
        assert_eq!(r.rules, vec![Rule::MaxReverse]);
    }

    #[test]
    fn too_far_off_the_line_is_no_road() {
        let s = settings();
        let r = filter(
            &s,
            Command::default(),
            Command { v: 1.0, w: 0.0 },
            &inputs(Some(frame(50.0, 3.5, 0.0, 1.5)), 0.0),
        );
        assert_eq!(r.output.v, 0.0);
        assert_eq!(r.rules, vec![Rule::NoRoad]);
    }

    #[test]
    fn nan_commands_stop() {
        let s = settings();
        let r = filter(
            &s,
            Command::default(),
            Command {
                v: f64::NAN,
                w: 0.0,
            },
            &inputs(None, 0.0),
        );
        assert_eq!(r.output, Command::default());
    }

    #[test]
    fn bad_settings_are_refused() {
        let mut s = settings();
        s.look_ahead = 0.0;
        assert_eq!(s.check(), Err(SettingsError::Positive("look_ahead", 0.0)));
        let mut s = settings();
        s.clf_weight = -1.0;
        assert_eq!(
            s.check(),
            Err(SettingsError::NotNegative("clf_weight", -1.0))
        );
    }
}
