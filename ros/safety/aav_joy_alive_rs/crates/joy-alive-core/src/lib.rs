//! The joystick-alive check: is the joystick really there, and really moving? No ROS here.
//! The Rust twin of `aav_safety`'s `joy_alive.py`.
//!
//! joy_node resends its last state 20 times a second while the USB receiver is plugged
//! in, so a pad turned off, out of range or out of battery may look like a person holding
//! the trigger and the stick (TODO 74a; `docs/car-day.md`, A9b). The node `joy_alive`
//! asks [`JoyAlive`], for each Joy message, whether to pass it on or a released copy:
//!
//! | State | Why it stops |
//! |---|---|
//! | `WAITING` | at start-up, until the deadman has been seen released once |
//! | `SILENT` | no Joy message for `silent_after` s: the receiver unplugged, or joy_node dead |
//! | `FROZEN` | the deadman held, a motion axis off centre, every axis and button bit-identical for `frozen_after` s (0: off) |
//!
//! Each latches until the deadman is seen released. Times are seconds on a steady clock.
#![warn(missing_docs, clippy::must_use_candidate)]

use std::fmt;

/// Why the joystick is stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trip {
    /// Not yet seen the deadman released (start-up).
    Waiting,
    /// No message for `silent_after` s.
    Silent,
    /// The same values while driving for `frozen_after` s.
    Frozen,
}

impl Trip {
    /// The word on `/drivers/joy/alive`.
    #[must_use]
    pub fn word(self) -> &'static str {
        match self {
            Trip::Waiting => "WAITING",
            Trip::Silent => "SILENT",
            Trip::Frozen => "FROZEN",
        }
    }
}

/// The word for a joystick that may drive.
pub const OK: &str = "OK";

/// The check's settings (the node's parameters: `aav_teleop` `config/joy_alive.yaml`).
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// The deadman (teleop_twist_joy's `enable_button`).
    pub enable_button: usize,
    /// The axes that move the car (`axis_linear.x`).
    pub motion_axes: Vec<usize>,
    /// |value| above this: the stick asks for motion.
    pub motion_threshold: f64,
    /// s with no message: SILENT. Above 0.
    pub silent_after: f64,
    /// s of identical values while driving: FROZEN; 0 off.
    pub frozen_after: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            enable_button: 0,
            motion_axes: vec![1],
            motion_threshold: 0.05,
            silent_after: 0.5,
            frozen_after: 2.0,
        }
    }
}

/// A setting the check refuses.
#[derive(Debug, Clone, PartialEq)]
pub enum SettingsError {
    /// Not a finite number of 0 or more.
    NotANumber(&'static str),
    /// `silent_after` 0: silence must stop the car.
    SilentZero,
}

impl fmt::Display for SettingsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SettingsError::NotANumber(name) => write!(f, "{name} must be a number >= 0"),
            SettingsError::SilentZero => {
                write!(
                    f,
                    "silent_after must be more than 0: silence must stop the car"
                )
            }
        }
    }
}

impl std::error::Error for SettingsError {}

/// Watch one joystick's Joy messages; say whether each may pass, and why not.
#[derive(Debug, Clone)]
pub struct JoyAlive {
    settings: Settings,
    tripped: Option<Trip>,
    last_time: Option<f64>,
    same_since: f64,
    last_values: Option<(Vec<f32>, Vec<i32>)>,
    sizes: (usize, usize),
}

impl JoyAlive {
    /// A check that waits for the deadman to be seen released.
    ///
    /// # Errors
    ///
    /// A time or the threshold that is negative, NaN or infinite; `silent_after` 0.
    pub fn new(settings: Settings) -> Result<Self, SettingsError> {
        for (name, value) in [
            ("silent_after", settings.silent_after),
            ("frozen_after", settings.frozen_after),
            ("motion_threshold", settings.motion_threshold),
        ] {
            if value.is_nan() || value < 0.0 || value.is_infinite() {
                return Err(SettingsError::NotANumber(name));
            }
        }
        if settings.silent_after == 0.0 {
            return Err(SettingsError::SilentZero);
        }
        Ok(JoyAlive {
            settings,
            tripped: Some(Trip::Waiting),
            last_time: None,
            same_since: 0.0,
            last_values: None,
            sizes: (0, 0),
        })
    }

    /// The settings.
    #[must_use]
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// Why it stops now, or `None` while the joystick may drive.
    #[must_use]
    pub fn tripped(&self) -> Option<Trip> {
        self.tripped
    }

    /// The newest message's number of axes and of buttons: a released copy's sizes.
    #[must_use]
    pub fn sizes(&self) -> (usize, usize) {
        self.sizes
    }

    /// Whether the deadman is held in these buttons.
    #[must_use]
    pub fn held(&self, buttons: &[i32]) -> bool {
        buttons
            .get(self.settings.enable_button)
            .is_some_and(|&b| b != 0)
    }

    /// Whether these values ask the car to move: the deadman and a motion axis.
    #[must_use]
    pub fn driving(&self, axes: &[f32], buttons: &[i32]) -> bool {
        self.held(buttons)
            && self.settings.motion_axes.iter().any(|&i| {
                axes.get(i)
                    .is_some_and(|&a| f64::from(a).abs() > self.settings.motion_threshold)
            })
    }

    /// Take a Joy message at `now`; `true` to pass it on, `false` to send it released.
    pub fn on_joy(&mut self, now: f64, axes: &[f32], buttons: &[i32]) -> bool {
        let same = self
            .last_values
            .as_ref()
            .is_some_and(|(a, b)| a.as_slice() == axes && b.as_slice() == buttons);
        if !same {
            self.last_values = Some((axes.to_vec(), buttons.to_vec()));
            self.same_since = now;
        }
        self.last_time = Some(now);
        self.sizes = (axes.len(), buttons.len());
        if self.tripped.is_some() && !self.held(buttons) {
            self.tripped = None; // released: the next press drives
        }
        if self.tripped.is_none()
            && self.settings.frozen_after > 0.0
            && self.driving(axes, buttons)
            && now - self.same_since >= self.settings.frozen_after
        {
            self.tripped = Some(Trip::Frozen);
        }
        self.tripped.is_none()
    }

    /// Check for silence at `now`; `true` if it has just tripped.
    pub fn tick(&mut self, now: f64) -> bool {
        match self.last_time {
            Some(last)
                if self.tripped != Some(Trip::Silent)
                    && now - last >= self.settings.silent_after =>
            {
                self.tripped = Some(Trip::Silent);
                true
            }
            _ => false,
        }
    }

    /// One line for people: the state, a dash, and why.
    #[must_use]
    pub fn state(&self, now: f64) -> String {
        let s = &self.settings;
        match (self.tripped, self.last_time) {
            (Some(Trip::Waiting), None) => "WAITING - no joystick yet".to_string(),
            (Some(Trip::Waiting), Some(_)) => format!(
                "WAITING - release the deadman (button {}) once",
                s.enable_button
            ),
            (Some(Trip::Silent), None) => "SILENT - no joystick".to_string(),
            (Some(Trip::Silent), Some(last)) if now - last < s.silent_after => {
                "SILENT - the joystick is back with the deadman held: release it".to_string()
            }
            (Some(Trip::Silent), Some(last)) => format!(
                "SILENT - nothing from the joystick for {:.1} s (more than {} s): release the \
                 deadman to drive again",
                now - last,
                g(s.silent_after)
            ),
            (Some(Trip::Frozen), _) => format!(
                "FROZEN - the same values for over {} s while driving: a dead pad? Release \
                 the deadman to drive again",
                g(s.frozen_after)
            ),
            (None, _) => match &self.last_values {
                Some((axes, buttons)) if self.driving(axes, buttons) => {
                    format!("{OK} - driving")
                }
                _ => format!("{OK} - idle"),
            },
        }
    }
}

/// A number as Python's `format(x, 'g')`: 6 significant digits, no trailing zeros.
#[must_use]
pub fn g(x: f64) -> String {
    if x == 0.0 {
        return if x.is_sign_negative() { "-0" } else { "0" }.to_string();
    }
    if x.is_nan() {
        return "nan".to_string();
    }
    if x.is_infinite() {
        return if x > 0.0 { "inf" } else { "-inf" }.to_string();
    }
    let sci = format!("{x:.5e}");
    let (mantissa, exp) = sci.split_once('e').unwrap_or((&sci, "0"));
    let exp: i32 = exp.parse().unwrap_or(0);
    let trim = |s: &str| -> String {
        if s.contains('.') {
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            s.to_string()
        }
    };
    if (-4..6).contains(&exp) {
        let decimals = usize::try_from(5 - exp).unwrap_or(0);
        trim(&format!("{x:.decimals$}"))
    } else {
        let sign = if exp < 0 { '-' } else { '+' };
        format!("{}e{sign}{:02}", trim(mantissa), exp.abs())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RELEASED: [i32; 3] = [0, 0, 0];
    const HELD: [i32; 3] = [1, 0, 0];

    fn started(settings: Settings) -> JoyAlive {
        let mut check = JoyAlive::new(settings).expect("settings");
        assert!(check.on_joy(0.0, &[0.0, 0.0], &RELEASED));
        check
    }

    #[test]
    fn nothing_drives_before_the_deadman_is_seen_released() {
        let mut check = JoyAlive::new(Settings::default()).expect("settings");
        assert_eq!(check.state(0.0), "WAITING - no joystick yet");
        assert!(!check.on_joy(0.0, &[0.0, 0.5], &HELD));
        assert!(check.on_joy(0.1, &[0.0, 0.0], &RELEASED));
        assert!(check.on_joy(0.2, &[0.0, 0.5], &HELD));
        assert_eq!(check.state(0.2), "OK - driving");
    }

    #[test]
    fn frozen_values_while_driving_stop_and_latch() {
        let mut check = started(Settings::default());
        assert!(check.on_joy(1.0, &[0.0, 0.5], &HELD));
        assert!(check.on_joy(2.95, &[0.0, 0.5], &HELD));
        assert!(!check.on_joy(3.0, &[0.0, 0.5], &HELD));
        assert_eq!(check.tripped(), Some(Trip::Frozen));
        assert!(!check.on_joy(3.1, &[0.0, 0.6], &HELD));
        assert!(check.on_joy(3.2, &[0.0, 0.0], &RELEASED));
    }

    #[test]
    fn silence_trips_once_and_latches() {
        let mut check = started(Settings::default());
        assert!(!check.tick(0.4));
        assert!(check.tick(0.5));
        assert!(!check.tick(1.0));
        assert!(check.state(1.0).starts_with("SILENT - nothing"));
        assert!(!check.on_joy(2.0, &[0.0, 0.5], &HELD));
        assert!(check.state(2.0).contains("back with the deadman held"));
    }

    #[test]
    fn bad_settings_are_refused() {
        let bad = |s: Settings| JoyAlive::new(s).is_err();
        let d = Settings::default;
        assert!(bad(Settings {
            silent_after: 0.0,
            ..d()
        }));
        assert!(bad(Settings {
            silent_after: -1.0,
            ..d()
        }));
        assert!(bad(Settings {
            frozen_after: f64::NAN,
            ..d()
        }));
        assert!(bad(Settings {
            frozen_after: f64::INFINITY,
            ..d()
        }));
    }

    #[test]
    fn g_is_pythons() {
        assert_eq!(g(2.0), "2");
        assert_eq!(g(0.5), "0.5");
        assert_eq!(g(0.1234567), "0.123457");
        assert_eq!(g(1e-5), "1e-05");
        assert_eq!(g(1234567.0), "1.23457e+06");
        assert_eq!(g(100000.0), "100000");
    }
}
