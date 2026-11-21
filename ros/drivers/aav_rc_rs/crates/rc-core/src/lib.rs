//! The RC transmitter as a ROS driver (TODO 48): its sticks to a command. No ROS here.
//! The Rust twin of `aav_rc`'s `rc_drive.py`.
//!
//! ```text
//!   the ESP32 reads the receiver ─uplink─> legacy_uart_bridge ─/vehicle/rc─> RcDrive
//!        ─> (v, w) ─/drivers/rc/cmd_vel─> twist_mux (priority 95) ─> ego
//! ```
//!
//! | The mode switch | This driver |
//! |---|---|
//! | computer | nothing |
//! | RC via ROS | a command every tick, zeros too: nothing below it drives |
//! | RC direct (the ESP32 drives) | nothing |
//! | no signal | nothing |
//!
//! Leaving RC via ROS: one zero, then nothing. `/vehicle/rc` quiet while on RC via ROS:
//! zeros, a stop, until it comes back. The steering stick becomes a turn rate by the
//! bicycle model, as ego expects.
#![warn(missing_docs, clippy::must_use_candidate)]

/// Where the transmitter's mode switch is (`RcSticks.mode_switch`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Switch {
    /// No RC signal (0), or a number this driver doesn't know.
    NoSignal,
    /// The computer drives (1).
    Computer,
    /// A person drives with the sticks, through ROS (2): this driver.
    RcViaRos,
    /// A person drives with the sticks, straight from the ESP32 (3).
    RcDirect,
}

impl Switch {
    /// The switch for `RcSticks.mode_switch`; an unknown number is no signal.
    #[must_use]
    pub fn from_byte(n: u8) -> Switch {
        match n {
            1 => Switch::Computer,
            2 => Switch::RcViaRos,
            3 => Switch::RcDirect,
            _ => Switch::NoSignal,
        }
    }

    /// What the switch means, for the log.
    #[must_use]
    pub fn words(self) -> &'static str {
        match self {
            Switch::NoSignal => "no RC signal",
            Switch::Computer => "computer",
            Switch::RcViaRos => "RC via ROS: driving from the sticks",
            Switch::RcDirect => "RC direct (the ESP32 drives)",
        }
    }
}

/// The driver's limits (its parameters; vehicle.yaml's numbers through the launch file).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    /// m/s at full stick forward.
    pub max_speed: f64,
    /// m/s at full stick in reverse.
    pub max_reverse_speed: f64,
    /// rad at full stick.
    pub max_steering_angle: f64,
    /// m.
    pub wheelbase: f64,
    /// s without /vehicle/rc: the sticks are stale.
    pub stale: f64,
}

impl Default for Limits {
    fn default() -> Self {
        Limits {
            max_speed: 1.0,
            max_reverse_speed: 0.5,
            max_steering_angle: 0.5,
            wheelbase: 1.57,
            stale: 0.2,
        }
    }
}

/// One /vehicle/rc message.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sticks {
    /// The mode switch.
    pub switch: Switch,
    /// -1..1, + forward.
    pub throttle: f64,
    /// -1..1, + left.
    pub steering: f64,
    /// The left stick in its brake band.
    pub brake: bool,
}

/// A command: forward speed (m/s) and turn rate (rad/s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Twist {
    /// m/s, + forward.
    pub v: f64,
    /// rad/s, + left.
    pub w: f64,
}

/// `x` within -1..1; NaN is 0.
fn clamp1(x: f64) -> f64 {
    if x.is_nan() {
        return 0.0;
    }
    // Python's max(-1.0, min(1.0, x)).
    let low = if x < 1.0 { x } else { 1.0 };
    if low > -1.0 {
        low
    } else {
        -1.0
    }
}

/// The sticks' command: the bicycle model's turn rate.
#[must_use]
pub fn twist(s: &Sticks, lim: &Limits) -> Twist {
    let throttle = if s.brake { 0.0 } else { clamp1(s.throttle) };
    let v = throttle
        * if throttle >= 0.0 {
            lim.max_speed
        } else {
            lim.max_reverse_speed
        };
    let angle = clamp1(s.steering) * lim.max_steering_angle;
    Twist {
        v,
        w: v * angle.tan() / lim.wheelbase,
    }
}

/// The driver's state: the latest sticks, and whether it was driving.
#[derive(Debug, Clone)]
pub struct RcDrive {
    /// Its limits.
    pub limits: Limits,
    heard: Option<(Sticks, f64)>,
    driving: bool,
}

impl RcDrive {
    /// A driver that has heard nothing.
    #[must_use]
    pub fn new(limits: Limits) -> Self {
        RcDrive {
            limits,
            heard: None,
            driving: false,
        }
    }

    /// Notes one /vehicle/rc message, at `now` (s).
    pub fn heard(&mut self, sticks: Sticks, now: f64) {
        self.heard = Some((sticks, now));
    }

    /// True if the last sticks heard were on RC via ROS.
    #[must_use]
    pub fn driving(&self) -> bool {
        self.driving
    }

    /// When the last /vehicle/rc came (s), if one has.
    #[must_use]
    pub fn heard_at(&self) -> Option<f64> {
        self.heard.map(|(_, at)| at)
    }

    /// This tick's command, or None: publish nothing.
    pub fn command(&mut self, now: f64) -> Option<Twist> {
        let zero = Twist { v: 0.0, w: 0.0 };
        let fresh = self
            .heard
            .filter(|(_, at)| now - at <= self.limits.stale)
            .map(|(s, _)| s);
        let Some(sticks) = fresh else {
            // Gone quiet on RC via ROS: hold a stop. Never heard, or not driving: nothing.
            return self.driving.then_some(zero);
        };
        if sticks.switch == Switch::RcViaRos {
            self.driving = true;
            return Some(twist(&sticks, &self.limits));
        }
        if self.driving {
            self.driving = false;
            return Some(zero); // one zero as the switch leaves RC via ROS
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sticks(switch: Switch) -> Sticks {
        Sticks {
            switch,
            throttle: 0.5,
            steering: 0.0,
            brake: false,
        }
    }

    #[test]
    fn one_zero_as_the_switch_leaves_rc_via_ros() {
        let mut d = RcDrive::new(Limits::default());
        d.heard(sticks(Switch::RcViaRos), 0.0);
        assert_eq!(d.command(0.0).map(|t| t.v), Some(0.5));
        d.heard(sticks(Switch::Computer), 0.02);
        assert_eq!(d.command(0.02).map(|t| t.v), Some(0.0));
        assert_eq!(d.command(0.04), None);
    }

    #[test]
    fn quiet_on_rc_via_ros_holds_a_stop() {
        let mut d = RcDrive::new(Limits::default());
        d.heard(sticks(Switch::RcViaRos), 0.0);
        let _ = d.command(0.0);
        assert_eq!(d.command(1.0).map(|t| t.v), Some(0.0));
    }
}
