//! The status display's logic: the Rust version of the 2025-26 `AAV_sentinel` Pico 2.
//!
//! The sentinel sits on the car's dashboard. It is an I2C device at 0x42: a computer on the
//! car (the 2025-26 Pi bridge in `AAV_cortex`, by its name; the sender isn't in any repo)
//! writes it a 46-byte telemetry packet, and it shows it.
//!
//! ```text
//!   I2C write ---> Telemetry ---> Sentinel ---> the LCD: 2 x 16 characters, 13 pages
//!                                   ^  |   ---> the 8x8 matrix: mode and direction
//!   knob, buttons ---> Input -------+  |   ---> 10 LEDs: computers and ROS nodes up
//!                                      +-----> the buzzer: Beep
//! ```
//!
//! | Module | Does |
//! |---|---|
//! | [`telemetry`] | the packet, byte for byte the 2025-26 `aav_telemetry_t` |
//! | [`pages`] | the 13 LCD pages |
//! | [`screen`] | a 16-character line that can't overflow |
//! | [`controls`] | the knob's clicks, and debounced buttons |
//! | [`alerts`] | the beeps, and the link's 3 s watchdog |
//! | [`matrix`] | the matrix's pictures |
//!
//! Plain `no_std` Rust with no hardware access: the board reads the pins, the clock and
//! the I2C writes, and drives the displays.
#![no_std]

pub mod alerts;
pub mod controls;
pub mod matrix;
pub mod pages;
pub mod screen;
pub mod telemetry;

pub use alerts::{Alerts, Beep, Tone};
pub use controls::{Button, Input, Knob, Turn};
pub use matrix::Pattern;
pub use pages::Page;
pub use screen::{Line, Screen};
pub use telemetry::{Telemetry, TelemetryError};

/// The sentinel's I2C address.
pub const I2C_ADDRESS: u8 = 0x42;

/// With no turn or press for this long, an unpinned page goes home.
pub const PAGE_TIMEOUT_MS: u32 = 10_000;

/// The whole display's state: feed it packets, inputs and the clock; read what to show.
#[derive(Debug, Clone, Copy)]
pub struct Sentinel {
    telemetry: Option<Telemetry>,
    page: Page,
    pinned: bool,
    last_input_ms: u32,
    alerts: Alerts,
}

impl Sentinel {
    #[must_use]
    pub const fn new(now_ms: u32) -> Self {
        Sentinel {
            telemetry: None,
            page: Page::HOME,
            pinned: false,
            last_input_ms: now_ms,
            alerts: Alerts::new(),
        }
    }

    /// One I2C write. A good packet replaces the last; a bad one is refused and changes
    /// nothing.
    pub fn on_write(&mut self, bytes: &[u8], now_ms: u32) -> Result<Option<Beep>, TelemetryError> {
        let t = Telemetry::from_write(bytes)?;
        self.telemetry = Some(t);
        Ok(self.alerts.on_packet(&t, now_ms))
    }

    /// A turn of the knob, or a press.
    pub fn on_input(&mut self, input: Input, now_ms: u32) -> Beep {
        self.last_input_ms = now_ms;
        match input {
            Input::Turn(turn) => {
                self.page = self.page.turn(match turn {
                    Turn::Next => 1,
                    Turn::Back => -1,
                });
                Beep::Page
            }
            Input::KnobPress => {
                self.pinned = !self.pinned;
                if self.pinned {
                    Beep::Pinned
                } else {
                    Beep::Unpinned
                }
            }
            Input::HomePress => {
                self.pinned = false;
                self.page = Page::HOME;
                Beep::Home
            }
        }
    }

    /// Call often (every few milliseconds): the page's timeout and the link's watchdog.
    pub fn tick(&mut self, now_ms: u32) -> Option<Beep> {
        if !self.pinned
            && self.page != Page::HOME
            && now_ms.wrapping_sub(self.last_input_ms) > PAGE_TIMEOUT_MS
        {
            self.page = Page::HOME;
        }
        self.alerts.tick(now_ms)
    }

    /// The LCD.
    #[must_use]
    pub fn screen(&self) -> Screen {
        self.page.render(self.telemetry.as_ref(), self.pinned)
    }

    /// The matrix.
    #[must_use]
    pub fn pattern(&self, now_ms: u32) -> Pattern {
        Pattern::now(self.telemetry.as_ref(), self.alerts.comms_lost(), now_ms)
    }

    /// The LED bar, LED 0 first: all off until the first packet.
    #[must_use]
    pub fn leds(&self) -> [bool; 10] {
        self.telemetry.map(|t| t.status.leds()).unwrap_or_default()
    }

    #[must_use]
    pub fn page(&self) -> Page {
        self.page
    }

    #[must_use]
    pub fn pinned(&self) -> bool {
        self.pinned
    }

    #[must_use]
    pub fn telemetry(&self) -> Option<&Telemetry> {
        self.telemetry.as_ref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::tests::sample;

    #[test]
    fn a_packet_fills_the_display() {
        let mut s = Sentinel::new(0);
        assert_eq!(s.leds(), [false; 10]);
        assert_eq!(s.pattern(0), Pattern::Smile);
        let beep = s.on_write(&sample().to_packet(), 10);
        assert_eq!(beep, Ok(Some(Beep::Connected)));
        assert_eq!(s.screen().top.as_str(), "MODE:AUTO       ");
        assert!(s.leds()[0]);
        assert_eq!(s.pattern(10), Pattern::Auto);
    }

    #[test]
    fn a_bad_packet_changes_nothing() {
        let mut s = Sentinel::new(0);
        s.on_write(&sample().to_packet(), 10).unwrap();
        let mut bad = sample().to_packet();
        bad[5] ^= 1;
        assert!(s.on_write(&bad, 20).is_err());
        assert_eq!(s.telemetry(), Some(&sample()));
    }

    #[test]
    fn the_knob_turns_pages_and_the_page_goes_home() {
        let mut s = Sentinel::new(0);
        assert_eq!(s.on_input(Input::Turn(Turn::Next), 100), Beep::Page);
        assert_eq!(s.page(), Page::Drive);
        s.tick(10_100);
        assert_eq!(s.page(), Page::Drive);
        s.tick(10_101);
        assert_eq!(s.page(), Page::HOME);
    }

    #[test]
    fn a_pinned_page_stays() {
        let mut s = Sentinel::new(0);
        s.on_input(Input::Turn(Turn::Back), 0);
        assert_eq!(s.on_input(Input::KnobPress, 0), Beep::Pinned);
        s.tick(60_000);
        assert_eq!(s.page(), Page::Error);
        assert_eq!(s.on_input(Input::HomePress, 60_000), Beep::Home);
        assert_eq!((s.page(), s.pinned()), (Page::HOME, false));
    }

    #[test]
    fn a_lost_link_shows_on_the_matrix() {
        let mut s = Sentinel::new(0);
        s.on_write(&sample().to_packet(), 0).unwrap();
        assert_eq!(s.tick(3_001), Some(Beep::CommsLost));
        assert_eq!(s.pattern(3_001), Pattern::Exclaim);
    }
}
