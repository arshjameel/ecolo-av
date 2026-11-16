//! The buzzer: what each beep sounds like, and when the car's state calls for one.

use crate::telemetry::{ErrorCode, Telemetry};

/// One tone: the buzzer on, then quiet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tone {
    /// How long the buzzer sounds (ms).
    pub on_ms: u16,
    /// How long it is quiet after (ms).
    pub off_ms: u16,
}

const fn tone(on_ms: u16, off_ms: u16) -> Tone {
    Tone { on_ms, off_ms }
}

const SHORT: Tone = tone(60, 80);
const LONG: Tone = tone(300, 80);

/// Each sound the sentinel makes. The patterns are the 2025-26 ones.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Beep {
    /// Three short: starting up.
    Boot,
    /// One long: ready, and listening on I2C.
    Ready,
    /// Two short: the first packet arrived.
    Connected,
    /// Long, short, short: no packet for 3 s; again every 10 s while it lasts.
    CommsLost,
    /// Short, short, short, long: packets again.
    CommsRestored,
    /// A tick: the page turned.
    Page,
    /// Two short: the page is pinned.
    Pinned,
    /// One short: unpinned.
    Unpinned,
    /// Two ticks: home.
    Home,
    /// Two long: a new error.
    Error,
    /// One very long: the e-stop.
    EStop,
    /// Short, long, short: the battery is low; again every 30 s.
    BatteryLow,
}

impl Beep {
    /// The beep's tones, in order.
    #[must_use]
    pub fn tones(self) -> &'static [Tone] {
        const BOOT: &[Tone] = &[SHORT, SHORT, SHORT];
        const READY: &[Tone] = &[tone(800, 0)];
        const TWO_SHORT: &[Tone] = &[SHORT, SHORT];
        const LOST: &[Tone] = &[LONG, SHORT, SHORT];
        const RESTORED: &[Tone] = &[SHORT, SHORT, SHORT, tone(600, 80)];
        const PAGE: &[Tone] = &[tone(30, 0)];
        const ONE_SHORT: &[Tone] = &[SHORT];
        const HOME: &[Tone] = &[tone(40, 40), tone(40, 0)];
        const ERROR: &[Tone] = &[LONG, tone(300, 160)];
        const ESTOP: &[Tone] = &[tone(1500, 0)];
        const BATTERY: &[Tone] = &[SHORT, LONG, SHORT];
        match self {
            Beep::Boot => BOOT,
            Beep::Ready => READY,
            Beep::Connected | Beep::Pinned => TWO_SHORT,
            Beep::CommsLost => LOST,
            Beep::CommsRestored => RESTORED,
            Beep::Page => PAGE,
            Beep::Unpinned => ONE_SHORT,
            Beep::Home => HOME,
            Beep::Error => ERROR,
            Beep::EStop => ESTOP,
            Beep::BatteryLow => BATTERY,
        }
    }
}

/// When no packet comes for this long, the link is lost.
pub const COMMS_TIMEOUT_MS: u32 = 3_000;
/// While it stays lost, the warning repeats this often.
pub const COMMS_REPEAT_MS: u32 = 10_000;
/// While the battery is low, its warning repeats this often.
pub const BATTERY_REPEAT_MS: u32 = 30_000;

/// Watches the packets, and calls for a beep when something changes.
///
/// The 2025-26 display beeped for an error on every packet, and held its loop still while
/// the buzzer sounded; here an error beeps once when it starts, and the board plays beeps
/// without stopping anything.
#[derive(Debug, Clone, Copy)]
pub struct Alerts {
    last_packet_ms: Option<u32>,
    lost: bool,
    lost_beep_ms: u32,
    error: ErrorCode,
    battery_beep_ms: Option<u32>,
}

impl Alerts {
    /// No packet seen yet, no alert sounding.
    #[must_use]
    pub const fn new() -> Self {
        Alerts {
            last_packet_ms: None,
            lost: false,
            lost_beep_ms: 0,
            error: ErrorCode::Ok,
            battery_beep_ms: None,
        }
    }

    /// A good packet arrived: the most important beep it calls for, if any.
    pub fn on_packet(&mut self, t: &Telemetry, now_ms: u32) -> Option<Beep> {
        let first = self.last_packet_ms.is_none();
        let restored = self.lost;
        self.last_packet_ms = Some(now_ms);
        self.lost = false;
        let new_error = t.error != self.error;
        self.error = t.error;

        let battery_due = t.error == ErrorCode::BatteryLow
            && self
                .battery_beep_ms
                .is_none_or(|at| now_ms.wrapping_sub(at) >= BATTERY_REPEAT_MS);
        if t.error != ErrorCode::BatteryLow {
            self.battery_beep_ms = None;
        }

        if new_error && t.error == ErrorCode::EStop {
            Some(Beep::EStop)
        } else if battery_due {
            self.battery_beep_ms = Some(now_ms);
            Some(Beep::BatteryLow)
        } else if new_error && t.error != ErrorCode::Ok {
            Some(Beep::Error)
        } else if restored {
            Some(Beep::CommsRestored)
        } else if first {
            Some(Beep::Connected)
        } else {
            None
        }
    }

    /// Call often: the link's watchdog.
    pub fn tick(&mut self, now_ms: u32) -> Option<Beep> {
        let since = now_ms.wrapping_sub(self.last_packet_ms?);
        if since <= COMMS_TIMEOUT_MS {
            return None;
        }
        if !self.lost || now_ms.wrapping_sub(self.lost_beep_ms) >= COMMS_REPEAT_MS {
            self.lost = true;
            self.lost_beep_ms = now_ms;
            return Some(Beep::CommsLost);
        }
        None
    }

    /// No packet for 3 s, after at least one.
    #[must_use]
    pub fn comms_lost(&self) -> bool {
        self.lost
    }
}

impl Default for Alerts {
    fn default() -> Self {
        Alerts::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::tests::sample;

    fn with(error: ErrorCode) -> Telemetry {
        Telemetry { error, ..sample() }
    }

    #[test]
    fn the_first_packet_says_hello() {
        let mut a = Alerts::new();
        assert_eq!(a.on_packet(&sample(), 100), Some(Beep::Connected));
        assert_eq!(a.on_packet(&sample(), 200), None);
    }

    #[test]
    fn nothing_is_lost_before_the_first_packet() {
        let mut a = Alerts::new();
        assert_eq!(a.tick(1_000_000), None);
        assert!(!a.comms_lost());
    }

    #[test]
    fn a_silent_link_warns_then_repeats() {
        let mut a = Alerts::new();
        a.on_packet(&sample(), 0);
        assert_eq!(a.tick(3_000), None);
        assert_eq!(a.tick(3_001), Some(Beep::CommsLost));
        assert!(a.comms_lost());
        assert_eq!(a.tick(13_000), None);
        assert_eq!(a.tick(13_001), Some(Beep::CommsLost));
        assert_eq!(a.on_packet(&sample(), 14_000), Some(Beep::CommsRestored));
        assert!(!a.comms_lost());
    }

    #[test]
    fn an_error_beeps_once() {
        let mut a = Alerts::new();
        a.on_packet(&sample(), 0);
        assert_eq!(a.on_packet(&with(ErrorCode::Motor), 100), Some(Beep::Error));
        assert_eq!(a.on_packet(&with(ErrorCode::Motor), 200), None);
        assert_eq!(a.on_packet(&with(ErrorCode::Ok), 300), None);
        assert_eq!(a.on_packet(&with(ErrorCode::Motor), 400), Some(Beep::Error));
    }

    #[test]
    fn the_e_stop_comes_first() {
        let mut a = Alerts::new();
        assert_eq!(a.on_packet(&with(ErrorCode::EStop), 0), Some(Beep::EStop));
        assert_eq!(a.on_packet(&with(ErrorCode::EStop), 100), None);
    }

    #[test]
    fn a_low_battery_repeats_every_30_s() {
        let mut a = Alerts::new();
        let low = with(ErrorCode::BatteryLow);
        assert_eq!(a.on_packet(&low, 0), Some(Beep::BatteryLow));
        assert_eq!(a.on_packet(&low, 29_999), None);
        assert_eq!(a.on_packet(&low, 30_000), Some(Beep::BatteryLow));
    }

    #[test]
    fn every_beep_has_a_sound() {
        use Beep::*;
        for b in [
            Boot,
            Ready,
            Connected,
            CommsLost,
            CommsRestored,
            Page,
            Pinned,
            Unpinned,
            Home,
            Error,
            EStop,
            BatteryLow,
        ] {
            assert!(b.tones().iter().all(|t| t.on_ms > 0), "{b:?}");
        }
    }
}
