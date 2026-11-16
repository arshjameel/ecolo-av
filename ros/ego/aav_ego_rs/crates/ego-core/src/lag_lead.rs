//! ego's lead: ask a little ahead, so the slow actuators answer about when the driver
//! wanted (`lag_lead.py`; the page: `docs/vehicle-model.md`, "ego's lead").
//!
//! ```text
//! u = f + T df/dt,   f = the request through a filter of time constant tf
//! ```
//!
//! | Lead | T |
//! |---|---|
//! | speeding up | throttle_dead_time + throttle_lag |
//! | slowing | brake_dead_time + brake_lag |
//! | steering | steer_dead_time + steer_lag |
//!
//! The lead runs before every limit, is capped, never changes direction, never softens a
//! slow-down, and never touches a stop. [`LagAwareWatch`] tells whose command ego obeys.

use std::collections::VecDeque;
use std::fmt;
use std::str::FromStr;

use car_model::ActuatorConfig;

use crate::clamp;

/// `lag_lead`: when the lead runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LeadMode {
    /// In a simulator only (use_sim_time): off on the car.
    #[default]
    Auto,
    /// The car too, once its lags are measured.
    On,
    /// Never.
    Off,
}

impl LeadMode {
    /// Its parameter value: `auto`, `on` or `off`.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            LeadMode::Auto => "auto",
            LeadMode::On => "on",
            LeadMode::Off => "off",
        }
    }
}

impl fmt::Display for LeadMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// A `lag_lead` value that isn't auto, on or off.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownMode(pub String);

impl fmt::Display for UnknownMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "lag_lead must be one of auto, on, off, not '{}'", self.0)
    }
}

impl std::error::Error for UnknownMode {}

impl FromStr for LeadMode {
    type Err = UnknownMode;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "auto" => Ok(LeadMode::Auto),
            "on" => Ok(LeadMode::On),
            "off" => Ok(LeadMode::Off),
            other => Err(UnknownMode(other.to_string())),
        }
    }
}

/// Whether the lead runs now: off never, auto in a simulator, on always; never twice.
#[must_use]
pub fn lead_wanted(mode: LeadMode, sim: bool, lag_aware_driving: bool) -> bool {
    if lag_aware_driving {
        return false;
    }
    mode == LeadMode::On || (mode == LeadMode::Auto && sim)
}

/// How late each actuator answers (s): its dead time plus its lag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeadTimes {
    /// The throttle's.
    pub speed_up: f64,
    /// The brake's.
    pub slow_down: f64,
    /// The steering's.
    pub steer: f64,
}

impl LeadTimes {
    /// From the car model's dead times and lags.
    #[must_use]
    pub fn from_config(cfg: &ActuatorConfig) -> Self {
        LeadTimes {
            speed_up: cfg.throttle_dead_time + cfg.throttle_lag,
            slow_down: cfg.brake_dead_time + cfg.brake_lag,
            steer: cfg.steer_dead_time + cfg.steer_lag,
        }
    }
}

/// The lead's own limits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeadLimits {
    /// s: the request's filter (the car then answers this fast).
    pub filter_time: f64,
    /// m/s: the most the lead adds to or takes from a speed.
    pub max_speed_lead: f64,
    /// rad: the most it adds to or takes from an angle.
    pub max_steer_lead: f64,
}

impl Default for LeadLimits {
    fn default() -> Self {
        LeadLimits {
            filter_time: 0.1,
            max_speed_lead: 0.3,
            max_steer_lead: 0.1,
        }
    }
}

/// The lead on a (speed, steering angle) request, one tick at a time.
#[derive(Debug, Clone, PartialEq)]
pub struct LagLead {
    /// How far ahead to lead each actuator.
    pub times: LeadTimes,
    /// The most the lead may add.
    pub limits: LeadLimits,
    /// The filtered (speed, angle); `None`: start from the next request.
    filtered: Option<(f64, f64)>,
}

impl LagLead {
    /// A lead with nothing filtered yet.
    #[must_use]
    pub fn new(times: LeadTimes, limits: LeadLimits) -> Self {
        LagLead {
            times,
            limits,
            filtered: None,
        }
    }

    /// The filters' state: `None` before the first request.
    #[must_use]
    pub fn filtered(&self) -> Option<(f64, f64)> {
        self.filtered
    }

    /// Set the filters' state (to start from a known one, as the vectors do).
    pub fn set_filtered(&mut self, filtered: Option<(f64, f64)>) {
        self.filtered = filtered;
    }

    /// Forget the filters: the next request starts them again, with no kick.
    pub fn reset(&mut self) {
        self.filtered = None;
    }

    fn filter(&self, old: f64, new: f64, dt: f64) -> f64 {
        let tf = self.limits.filter_time;
        if tf <= 0.0 {
            new
        } else {
            old + (new - old) * (1.0 - (-dt / tf).exp())
        }
    }

    /// The led (speed, angle) for the driver's request.
    pub fn step(&mut self, speed: f64, angle: f64, dt: f64) -> (f64, f64) {
        if dt <= 0.0 {
            self.reset();
            return (speed, angle);
        }
        let Some((old_speed, old_angle)) = self.filtered else {
            self.filtered = Some((speed, angle));
            return (speed, angle);
        };
        let lim = self.limits;
        let speeding_up = speed.abs() > old_speed.abs();
        let f_speed = self.filter(old_speed, speed, dt);
        let f_angle = self.filter(old_angle, angle, dt);
        let rate_speed = (f_speed - old_speed) / dt;
        let rate_angle = (f_angle - old_angle) / dt;
        self.filtered = Some((f_speed, f_angle));

        let t = if speeding_up {
            self.times.speed_up
        } else {
            self.times.slow_down
        };
        let mut led = f_speed + clamp(t * rate_speed, -lim.max_speed_lead, lim.max_speed_lead);
        // Never the other way: forward stays forward (or stopped), reverse stays reverse.
        if speed > 0.0 || (speed == 0.0 && f_speed >= 0.0) {
            led = 0.0_f64.max(led);
        } else {
            led = 0.0_f64.min(led);
        }
        // Never soften a slow-down.
        if !speeding_up {
            led = if speed >= 0.0 {
                led.min(speed)
            } else {
                led.max(speed)
            };
        }
        let led_angle = f_angle
            + clamp(
                self.times.steer * rate_angle,
                -lim.max_steer_lead,
                lim.max_steer_lead,
            );
        (led, led_angle)
    }
}

/// What identifies one TwistStamped: twist_mux passes it on unchanged.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommandKey {
    /// The stamp's seconds.
    pub sec: i32,
    /// The stamp's nanoseconds.
    pub nanosec: u32,
    /// twist.linear.x (m/s).
    pub vx: f64,
    /// twist.angular.z (rad/s).
    pub wz: f64,
}

/// Tells whether the command ego obeys came from one of the watched drivers, and whether
/// that driver said it is lag-aware (`/drivers/<d>/lag_aware`).
#[derive(Debug, Clone, PartialEq)]
pub struct LagAwareWatch {
    drivers: Vec<Watched>,
    keep: usize,
}

#[derive(Debug, Clone, PartialEq)]
struct Watched {
    name: String,
    aware: bool,
    recent: VecDeque<CommandKey>,
}

impl LagAwareWatch {
    /// Watch these drivers (a name given twice counts once), each one's last 50 commands.
    #[must_use]
    pub fn new<S: AsRef<str>>(drivers: &[S]) -> Self {
        LagAwareWatch::with_keep(drivers, 50)
    }

    /// As [`LagAwareWatch::new`], keeping each one's last `keep` commands.
    #[must_use]
    pub fn with_keep<S: AsRef<str>>(drivers: &[S], keep: usize) -> Self {
        let mut watched: Vec<Watched> = Vec::new();
        for d in drivers {
            if !watched.iter().any(|w| w.name == d.as_ref()) {
                watched.push(Watched {
                    name: d.as_ref().to_string(),
                    aware: false,
                    recent: VecDeque::with_capacity(keep),
                });
            }
        }
        LagAwareWatch {
            drivers: watched,
            keep,
        }
    }

    /// The watched drivers, in order.
    pub fn drivers(&self) -> impl Iterator<Item = &str> {
        self.drivers.iter().map(|w| w.name.as_str())
    }

    /// True if `driver` is watched.
    #[must_use]
    pub fn watches(&self, driver: &str) -> bool {
        self.drivers.iter().any(|w| w.name == driver)
    }

    /// Note whether `driver` says it is lag-aware; an unwatched one is ignored.
    pub fn set_aware(&mut self, driver: &str, aware: bool) {
        if let Some(w) = self.drivers.iter_mut().find(|w| w.name == driver) {
            w.aware = aware;
        }
    }

    /// A command this driver sent.
    pub fn sent(&mut self, driver: &str, key: CommandKey) {
        let keep = self.keep;
        if let Some(w) = self.drivers.iter_mut().find(|w| w.name == driver) {
            if keep == 0 {
                return;
            }
            if w.recent.len() == keep {
                w.recent.pop_front();
            }
            w.recent.push_back(key);
        }
    }

    /// The watched driver whose command this is, lag-aware or not.
    #[must_use]
    pub fn whose(&self, key: Option<&CommandKey>) -> Option<&str> {
        let key = key?;
        self.drivers
            .iter()
            .find(|w| w.recent.contains(key))
            .map(|w| w.name.as_str())
    }

    /// The lag-aware driver whose command this is.
    #[must_use]
    pub fn driving(&self, key: Option<&CommandKey>) -> Option<&str> {
        let key = key?;
        self.drivers
            .iter()
            .find(|w| w.aware && w.recent.contains(key))
            .map(|w| w.name.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(sec: i32, vx: f64) -> CommandKey {
        CommandKey {
            sec,
            nanosec: 0,
            vx,
            wz: 0.0,
        }
    }

    #[test]
    fn modes_parse_and_refuse() {
        assert_eq!("on".parse::<LeadMode>(), Ok(LeadMode::On));
        assert!("maybe".parse::<LeadMode>().is_err());
        assert!(!lead_wanted(LeadMode::Auto, false, false));
        assert!(lead_wanted(LeadMode::Auto, true, false));
        assert!(!lead_wanted(LeadMode::On, true, true));
    }

    #[test]
    fn the_watch_finds_whose_command_it_is() {
        let mut w = LagAwareWatch::with_keep(&["nav", "point", "nav"], 2);
        assert_eq!(w.drivers().count(), 2);
        w.sent("nav", key(1, 0.5));
        assert_eq!(w.whose(Some(&key(1, 0.5))), Some("nav"));
        assert_eq!(w.driving(Some(&key(1, 0.5))), None);
        w.set_aware("nav", true);
        assert_eq!(w.driving(Some(&key(1, 0.5))), Some("nav"));
        w.sent("nav", key(2, 0.5));
        w.sent("nav", key(3, 0.5));
        assert_eq!(w.whose(Some(&key(1, 0.5))), None, "only the last 2 kept");
        assert_eq!(w.whose(None), None);
    }

    #[test]
    fn the_lead_never_changes_direction_or_softens_a_stop() {
        let times = LeadTimes {
            speed_up: 0.5,
            slow_down: 0.5,
            steer: 0.5,
        };
        let mut lead = LagLead::new(times, LeadLimits::default());
        assert_eq!(lead.step(1.0, 0.0, 0.02), (1.0, 0.0));
        let (v, _) = lead.step(0.0, 0.0, 0.02);
        assert_eq!(v, 0.0);
    }
}
