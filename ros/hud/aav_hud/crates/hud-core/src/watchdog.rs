//! The sensor watchdog's verdict (`/safety/sensor_watchdog/state`, aav_safety), turned into
//! a banner. The HUD and the map window both show it, the same way.

use crate::health::Health;

/// A banner's worth of trouble from the sensor watchdog.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Alarm {
    /// The automatic drivers are stopped: a sensor they need failed.
    Stop(String),
    /// Something is wrong, but nothing was stopped for it.
    Warn(String),
}

/// What the banner says, from the verdict topic's health and its last text; None when
/// all is well.
///
/// A watchdog never heard is none of the screen's business (the turtle runs none). One
/// that went quiet is: then nothing watches the sensors, and twist_mux holds Nav2 for
/// its silence, so say so.
#[must_use]
pub fn alarm(health: Health, verdict: Option<&str>) -> Option<Alarm> {
    match health {
        Health::Never => None,
        Health::Stale(_) => Some(Alarm::Stop("SENSOR WATCHDOG SILENT".into())),
        Health::Ok(_) => match verdict {
            None | Some("OK") => None,
            Some(text) if text.starts_with("STOP") => Some(Alarm::Stop(text.into())),
            Some(text) => Some(Alarm::Warn(text.into())),
        },
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    #[test]
    fn only_trouble_makes_a_banner() {
        let ok = Health::Ok(Duration::ZERO);
        assert_eq!(alarm(Health::Never, None), None);
        assert_eq!(alarm(ok, Some("OK")), None);
        let stop = "STOP: odom DISAGREE (the wheels and the IMU disagree)";
        assert_eq!(alarm(ok, Some(stop)), Some(Alarm::Stop(stop.into())));
        assert!(matches!(
            alarm(ok, Some("WARN: radar NEVER")),
            Some(Alarm::Warn(_))
        ));
        assert_eq!(
            alarm(Health::Stale(Duration::from_secs(2)), Some("OK")),
            Some(Alarm::Stop("SENSOR WATCHDOG SILENT".into()))
        );
    }
}
