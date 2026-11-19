//! control_interface's five states (aav_lane_geometry's `lane_follow.py`, ported): what to
//! command after each lane message and each 0.1 s tick.

use std::fmt;

use crate::pyfmt::fixed;
use crate::steering::{Geometry, Law};

/// twist_mux forgets the AI slot when /drivers/ai/cmd_vel has been quiet this long (s):
/// lane_speed.py's `AI_SLOT_TIMEOUT_S`.
pub const AI_SLOT_TIMEOUT_S: f64 = 0.5;

/// The follower's state, as /drivers/ai/lane_state names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    LaneKeep,
    LaneChanging,
    LaneCommitted,
    NoLaneStopping,
    Resuming,
}

impl State {
    pub fn as_str(self) -> &'static str {
        match self {
            State::LaneKeep => "LANE_KEEP",
            State::LaneChanging => "LANE_CHANGING",
            State::LaneCommitted => "LANE_COMMITTED",
            State::NoLaneStopping => "NO_LANE_STOPPING",
            State::Resuming => "RESUMING",
        }
    }
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// control_interface's parameters, with its defaults (README.md's table).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    pub base_speed: f64,
    pub max_angular_z: f64,
    pub min_speed: f64,
    pub min_curvature: f64,
    pub lane_change_rate_threshold: f64,
    pub lane_change_offset_threshold: f64,
    pub lane_change_speed: f64,
    pub lane_change_timeout: f64,
    pub stop_wait: f64,
    pub decel_rate: f64,
    pub accel_rate: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            base_speed: 2.0,
            max_angular_z: 0.5,
            min_speed: 0.3,
            min_curvature: 20.0,
            lane_change_rate_threshold: 0.05,
            lane_change_offset_threshold: 0.40,
            lane_change_speed: 1.0,
            lane_change_timeout: 8.0,
            stop_wait: 0.3,
            decel_rate: 0.5,
            accel_rate: 0.3,
        }
    }
}

/// One command for /drivers/ai/cmd_vel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    /// m/s.
    pub linear_x: f64,
    /// rad/s, + = left.
    pub angular_z: f64,
}

/// What to publish and log after one message or tick, in order.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Reply {
    pub commands: Vec<Command>,
    /// /drivers/ai/lane_status's line.
    pub status: Option<String>,
    /// /drivers/ai/lane_state (after a message only).
    pub state: Option<State>,
    /// (old, new), for the log.
    pub transitions: Vec<(State, State)>,
    /// s without a lane message, when the watchdog trips (for the log).
    pub quiet_for: Option<f64>,
}

/// The speed after slowing at `decel` m/s² for `dt` s (lane_speed.py's `slow_down`).
fn slow_down(speed: f64, decel: f64, dt: f64) -> f64 {
    if decel <= 0.0 {
        return 0.0;
    }
    (speed - decel * dt.max(0.0)).max(0.0)
}

/// The next speed while stopping (lane_speed.py's `stopping_speed`): 0 once the follower's
/// commands have been quiet for twist_mux's timeout, as ego is already stopping the car.
fn stopping_speed(speed: f64, decel: f64, dt: f64, quiet: f64) -> f64 {
    if quiet >= AI_SLOT_TIMEOUT_S {
        return 0.0;
    }
    slow_down(speed, decel, dt)
}

/// Python's `max(a, b)`: the first unless the second is greater (so NaN as Python has it).
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// numpy's clip: NaN stays NaN.
fn clip(v: f64, low: f64, high: f64) -> f64 {
    if v.is_nan() {
        v
    } else {
        v.max(low).min(high)
    }
}

/// The lane follower: the 2025-26 states around a steering [`Law`].
#[derive(Debug, Clone)]
pub struct Follower {
    s: Settings,
    law: Law,
    state: State,
    current_speed: f64,
    last_steer: f64,
    last_offset: f64,
    offset_rate: f64,
    state_entry: f64,
    last_msg: f64,
    tick_t: f64,
    sent_t: f64,
    lc_peak_offset: f64,
    last_t: f64,
    reply: Reply,
}

impl Follower {
    /// A follower starting now: `wall` and `now` as the node's two clocks read.
    pub fn new(settings: Settings, mut law: Law, wall: f64, now: f64) -> Self {
        law.reset(wall);
        Follower {
            s: settings,
            law,
            state: State::LaneKeep,
            current_speed: 0.0,
            last_steer: 0.0,
            last_offset: 0.0,
            offset_rate: 0.0,
            state_entry: wall,
            last_msg: now,
            tick_t: now,
            sent_t: now,
            lc_peak_offset: 0.0,
            last_t: wall,
            reply: Reply::default(),
        }
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn law(&self) -> &Law {
        &self.law
    }

    pub fn settings(&self) -> &Settings {
        &self.s
    }

    /// One lane message.
    pub fn on_geometry(&mut self, g: &Geometry, wall: f64, now: f64) -> Reply {
        self.reply = Reply::default();
        self.last_msg = now;
        let dt = (wall - self.last_t).max(1e-4);
        self.last_t = wall;
        self.offset_rate = (g.offset - self.last_offset) / dt;
        self.last_offset = g.offset;
        let lanes_ok = g.lanes_ok();
        match self.state {
            State::LaneKeep => self.lane_keep(g, lanes_ok, wall, now),
            State::LaneChanging => self.lane_changing(g, lanes_ok, wall, now),
            State::LaneCommitted => self.lane_committed(g, lanes_ok, wall, now),
            State::NoLaneStopping => self.no_lane_stopping(g, lanes_ok, wall, now),
            State::Resuming => self.resuming(g, lanes_ok, wall, now),
        }
        self.reply.state = Some(self.state);
        std::mem::take(&mut self.reply)
    }

    /// The watchdog, every 0.1 s on the node's clock: no message for `stop_wait` s slows
    /// the car at `decel_rate`, and the slow-down goes on here when messages stop.
    pub fn tick(&mut self, wall: f64, now: f64) -> Reply {
        self.reply = Reply::default();
        let dt = now - self.tick_t;
        self.tick_t = now;
        let elapsed = now - self.last_msg;
        if elapsed > self.s.stop_wait && self.state != State::NoLaneStopping {
            self.reply.quiet_for = Some(elapsed);
            self.transition(State::NoLaneStopping, wall);
        }
        if self.state == State::NoLaneStopping && self.current_speed > 0.0 {
            self.current_speed =
                stopping_speed(self.current_speed, self.s.decel_rate, dt, now - self.sent_t);
            let steer = if self.current_speed > 0.0 {
                self.last_steer
            } else {
                0.0
            };
            self.send(self.current_speed, steer, now);
        }
        std::mem::take(&mut self.reply)
    }

    fn lane_keep(&mut self, g: &Geometry, lanes_ok: bool, wall: f64, now: f64) {
        if !lanes_ok {
            self.transition(State::NoLaneStopping, wall);
            return;
        }
        let large = g.offset.abs() > self.s.lane_change_offset_threshold;
        let fast = self.offset_rate.abs() > self.s.lane_change_rate_threshold;
        if large && fast {
            self.lc_peak_offset = g.offset.abs();
            self.transition(State::LaneChanging, wall);
            return;
        }
        let (v, w) = self.lane_keep_command(g, wall);
        self.send(v, w, now);
        self.status(g, "LANE_KEEP");
    }

    fn lane_changing(&mut self, g: &Geometry, lanes_ok: bool, wall: f64, now: f64) {
        let elapsed = wall - self.state_entry;
        if elapsed > self.s.lane_change_timeout || !lanes_ok {
            self.transition(State::NoLaneStopping, wall);
            return;
        }
        let curr = g.offset.abs();
        self.lc_peak_offset = py_max(self.lc_peak_offset, curr);
        if curr < self.lc_peak_offset * 0.5 && curr < self.s.lane_change_offset_threshold * 0.5 {
            self.transition(State::LaneCommitted, wall);
            return;
        }
        self.send(self.s.lane_change_speed, self.last_steer, now);
        self.current_speed = self.s.lane_change_speed;
        let extra = format!(
            "LANE_CHANGING peak={}m",
            fixed(self.lc_peak_offset, 2, false)
        );
        self.status(g, &extra);
    }

    fn lane_committed(&mut self, g: &Geometry, lanes_ok: bool, wall: f64, now: f64) {
        self.law.reset(wall);
        self.transition(State::LaneKeep, wall);
        self.lane_keep(g, lanes_ok, wall, now);
    }

    fn no_lane_stopping(&mut self, g: &Geometry, lanes_ok: bool, wall: f64, now: f64) {
        if lanes_ok {
            self.transition(State::Resuming, wall);
            return;
        }
        let elapsed = fixed(wall - self.state_entry, 1, false);
        if self.current_speed > 0.0 {
            // tick() slows the car, whether messages come or not.
            self.status(g, &format!("NO_LANE decelerating t={elapsed}s"));
        } else {
            self.send(0.0, 0.0, now);
            self.status(g, &format!("STOPPED waiting t={elapsed}s"));
        }
    }

    fn resuming(&mut self, g: &Geometry, lanes_ok: bool, wall: f64, now: f64) {
        if !lanes_ok {
            self.transition(State::NoLaneStopping, wall);
            return;
        }
        self.current_speed = self
            .s
            .base_speed
            .min(self.current_speed + self.s.accel_rate * 0.1);
        // As in 2025-26: the lane-keeping command sets current_speed to its own speed, so
        // the command is that speed, not the ramp's (README.md, "Kept as it was").
        let (_, w) = self.lane_keep_command(g, wall);
        self.send(self.current_speed, w, now);
        let extra = format!(
            "RESUMING {}/{}",
            fixed(self.current_speed, 2, false),
            fixed(self.s.base_speed, 2, false)
        );
        self.status(g, &extra);
        if self.current_speed >= self.s.base_speed * 0.95 {
            self.transition(State::LaneKeep, wall);
        }
    }

    fn send(&mut self, linear_x: f64, angular_z: f64, now: f64) {
        self.reply.commands.push(Command {
            linear_x,
            angular_z,
        });
        self.sent_t = now;
    }

    /// The lane-keeping speed and turn (build_cmd_lane_keep).
    fn lane_keep_command(&mut self, g: &Geometry, wall: f64) -> (f64, f64) {
        let mut speed = self.s.base_speed;
        if !g.both_detected {
            speed *= 0.7;
        }
        if 0.0 < g.mean_radius && g.mean_radius < self.s.min_curvature {
            speed *= py_max(0.4, g.mean_radius / self.s.min_curvature);
        }
        let linear_x = py_max(self.s.min_speed, speed);
        self.current_speed = linear_x;
        let steer = self.law.steer(g, linear_x, self.last_steer, wall);
        let angular_z = clip(steer, -self.s.max_angular_z, self.s.max_angular_z);
        self.last_steer = angular_z;
        (linear_x, angular_z)
    }

    fn transition(&mut self, new: State, wall: f64) {
        if self.state != new {
            self.reply.transitions.push((self.state, new));
        }
        self.state = new;
        self.state_entry = wall;
        if matches!(new, State::LaneKeep | State::LaneCommitted) {
            self.law.reset(wall);
        }
    }

    fn status(&mut self, g: &Geometry, extra: &str) {
        self.reply.status = Some(format!(
            "[{}] offset={}m curv={}m speed={}m/s {}",
            self.state,
            fixed(g.offset, 3, true),
            fixed(g.mean_radius, 1, false),
            fixed(self.current_speed, 2, false),
            extra
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::steering::{Line, Pid, PidGains, PursuitSettings};

    fn seen(offset: f64) -> Geometry {
        Geometry {
            offset,
            mean_radius: 9999.0,
            lane_width: 3.5,
            left_detected: true,
            right_detected: true,
            both_detected: true,
            left: Line {
                a: 0.0,
                b: 0.0,
                c: 1.75 - offset,
            },
            right: Line {
                a: 0.0,
                b: 0.0,
                c: -1.75 - offset,
            },
        }
    }

    fn pid() -> Law {
        Law::Pid {
            pid: Pid::new(PidGains::default(), 0.0),
            invert_offset_sign: true,
        }
    }

    #[test]
    fn keeps_the_lane_and_says_so() {
        let mut f = Follower::new(Settings::default(), pid(), 0.0, 0.0);
        let r = f.on_geometry(&seen(0.2), 0.1, 0.1);
        assert_eq!(r.commands.len(), 1);
        assert_eq!(r.commands[0].linear_x, 2.0);
        assert!(r.commands[0].angular_z < 0.0);
        assert_eq!(r.state, Some(State::LaneKeep));
        assert!(r
            .status
            .unwrap()
            .starts_with("[LANE_KEEP] offset=+0.200m curv=9999.0m"));
    }

    #[test]
    fn lines_lost_slow_down_then_stop() {
        let mut f = Follower::new(Settings::default(), pid(), 0.0, 0.0);
        f.on_geometry(&seen(0.0), 0.1, 0.1);
        let mut none = seen(0.0);
        none.left_detected = false;
        none.right_detected = false;
        let r = f.on_geometry(&none, 0.2, 0.2);
        assert_eq!(
            r.transitions,
            vec![(State::LaneKeep, State::NoLaneStopping)]
        );
        let mut t = 0.2;
        let mut speeds = Vec::new();
        for _ in 0..60 {
            t += 0.1;
            f.on_geometry(&none, t, t);
            if let Some(c) = f.tick(t, t).commands.first() {
                speeds.push(c.linear_x);
            }
        }
        assert!(speeds.windows(2).all(|w| w[1] < w[0]), "{speeds:?}");
        assert_eq!(*speeds.last().unwrap(), 0.0);
    }

    #[test]
    fn messages_stopping_trip_the_watchdog() {
        let mut f = Follower::new(Settings::default(), pid(), 0.0, 0.0);
        f.on_geometry(&seen(0.0), 0.1, 0.1);
        assert!(f.tick(0.2, 0.2).quiet_for.is_none());
        let r = f.tick(0.5, 0.5);
        assert!(r.quiet_for.is_some());
        assert_eq!(f.state(), State::NoLaneStopping);
        assert_eq!(r.commands.len(), 1);
    }

    #[test]
    fn a_challenger_has_the_same_states() {
        let mut f = Follower::new(
            Settings::default(),
            Law::PurePursuit(PursuitSettings::default()),
            0.0,
            0.0,
        );
        let r = f.on_geometry(&seen(0.3), 0.1, 0.1);
        assert!(r.commands[0].angular_z < 0.0);
        let r = f.on_geometry(&seen(-0.3), 0.2, 0.2);
        assert!(r.commands[0].angular_z > 0.0);
    }
}
