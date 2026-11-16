//! The knob (a KY-040 rotary encoder with a push switch) and the home button.
//!
//! The board reads the pins about every millisecond and feeds them here; out come whole
//! clicks and clean presses.

use aav_inputs::{Debouncer, Edge};

/// One click of the knob.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Turn {
    /// To the next page.
    Next,
    /// To the one before.
    Back,
}

/// What the person did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Input {
    Turn(Turn),
    /// The knob's own switch: pin or unpin the page.
    KnobPress,
    /// The separate button: home, and unpin.
    HomePress,
}

/// The knob's two outputs, CLK and DT, as a quadrature signal.
///
/// The 2025-26 code moved one page on every change of the pins it saw, so one click could
/// move one page or several, depending on how fast its loop ran. Here a click counts only
/// when the knob is back at rest (both pins high, with the pull-ups) after at least half a
/// cycle in one direction.
#[derive(Debug, Clone, Copy)]
pub struct Knob {
    last: u8,
    quarter_steps: i8,
}

/// Both pins high: where a KY-040 rests between clicks.
const REST: u8 = 0b11;

impl Knob {
    #[must_use]
    pub fn new(clk: bool, dt: bool) -> Self {
        Knob {
            last: pins(clk, dt),
            quarter_steps: 0,
        }
    }

    /// Feed the pins; returns a click when one ends.
    pub fn update(&mut self, clk: bool, dt: bool) -> Option<Turn> {
        let now = pins(clk, dt);
        if now == self.last {
            return None;
        }
        // The 2025-26 table: (before << 2) | after, for one quarter step each way.
        match (self.last << 2) | now {
            0b0001 | 0b0111 | 0b1110 | 0b1000 => self.quarter_steps += 1,
            0b0010 | 0b0100 | 0b1101 | 0b1011 => self.quarter_steps -= 1,
            _ => {} // both pins changed at once: a missed step, direction unknown
        }
        self.last = now;
        if now != REST {
            return None;
        }
        let steps = core::mem::take(&mut self.quarter_steps);
        match steps {
            s if s >= 2 => Some(Turn::Next),
            s if s <= -2 => Some(Turn::Back),
            _ => None, // a wobble, back where it started
        }
    }
}

fn pins(clk: bool, dt: bool) -> u8 {
    (u8::from(clk) << 1) | u8::from(dt)
}

/// A push button to ground, with a pull-up: high when released.
///
/// The 2025-26 code took the first falling edge, then ignored the pin for 200 ms; a
/// button held longer than that bounced on release, and counted a second press. A
/// [`Debouncer`] needs the level to hold for 20 ms each way, so a release never counts.
#[derive(Debug, Clone, Copy)]
pub struct Button(Debouncer);

impl Button {
    /// Steady for 20 ms before a change counts.
    pub const STABLE_MS: u32 = 20;

    #[must_use]
    pub const fn new() -> Self {
        Button(Debouncer::new(true, Self::STABLE_MS))
    }

    /// Feed the pin's level; true once per press.
    pub fn pressed(&mut self, level: bool, now_ms: u32) -> bool {
        self.0.update(level, now_ms) == Some(Edge::Falling)
    }
}

impl Default for Button {
    fn default() -> Self {
        Button::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn clicks(states: &[(bool, bool)]) -> std::vec::Vec<Turn> {
        let mut knob = Knob::new(true, true);
        states
            .iter()
            .filter_map(|&(c, d)| knob.update(c, d))
            .collect()
    }

    extern crate std;

    // From rest (1,1), one way round the cycle, and the other.
    const NEXT: [(bool, bool); 4] = [(true, false), (false, false), (false, true), (true, true)];
    const BACK: [(bool, bool); 4] = [(false, true), (false, false), (true, false), (true, true)];

    #[test]
    fn one_click_is_one_page() {
        assert_eq!(clicks(&NEXT), [Turn::Next]);
        assert_eq!(clicks(&BACK), [Turn::Back]);
    }

    #[test]
    fn clicks_add_up() {
        let mut states = std::vec::Vec::new();
        for _ in 0..3 {
            states.extend_from_slice(&NEXT);
        }
        states.extend_from_slice(&BACK);
        assert_eq!(
            clicks(&states),
            [Turn::Next, Turn::Next, Turn::Next, Turn::Back]
        );
    }

    #[test]
    fn a_wobble_is_not_a_click() {
        // A quarter step and back.
        assert!(clicks(&[(true, false), (true, true)]).is_empty());
        // Repeated readings change nothing.
        assert!(clicks(&[(true, true), (true, true)]).is_empty());
    }

    #[test]
    fn a_press_counts_once_however_long_it_is_held() {
        let mut b = Button::new();
        let mut presses = 0;
        // Bounces on press, held 500 ms, bounces on release.
        for ms in 0..1000u32 {
            let level = match ms {
                0..=99 => true,
                100..=105 => ms % 2 == 0,
                106..=605 => false,
                606..=611 => ms % 2 == 1,
                _ => true,
            };
            presses += usize::from(b.pressed(level, ms));
        }
        assert_eq!(presses, 1);
    }

    #[test]
    fn a_glitch_is_not_a_press() {
        let mut b = Button::new();
        assert!(!b.pressed(false, 0));
        assert!(!b.pressed(true, 5));
        assert!(!b.pressed(true, 100));
    }
}
