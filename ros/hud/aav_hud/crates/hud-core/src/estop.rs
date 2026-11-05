//! The HUD's e-stop button: one press sends `/safety/estop` true five times, 0.1 s apart.
//!
//! The same as aav_hud's button (docs/hud.md): repeated, so one lost message can't keep
//! the car going. It never sends false: clearing the e-stop stays a person's deliberate
//! act, from a laptop (`pixi run clear estop`). The type says so: the only message it
//! asks for is [`StopMessage`].

use std::time::{Duration, Instant};

/// Messages for one press: the first at once, then one every [`PERIOD`].
pub const REPEATS: u8 = 5;
/// Time between two messages of one press.
pub const PERIOD: Duration = Duration::from_millis(100);

/// "Publish `/safety/estop` true now." There is no message for false.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StopMessage;

/// The button's state: how many messages of the last press are still to send, and when.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EstopButton {
    left: u8,
    next_at: Option<Instant>,
    presses: u32,
}

impl EstopButton {
    /// Someone pressed it at `now`. Pressing again during a burst starts a new burst.
    pub fn press(&mut self, now: Instant) {
        self.left = REPEATS;
        self.next_at = Some(now);
        self.presses = self.presses.saturating_add(1);
    }

    /// Call often (every frame, or every 10 ms). Returns a message when one is due.
    ///
    /// At most one per call, and the next is due [`PERIOD`] after this one was due, so
    /// a slow caller still sends all five, never two at once.
    pub fn poll(&mut self, now: Instant) -> Option<StopMessage> {
        let due = self.next_at?;
        if self.left == 0 || now < due {
            return None;
        }
        self.left -= 1;
        // Late by more than a period: count from now, so they don't bunch up.
        let base = now.checked_sub(PERIOD).map_or(due, |t| due.max(t));
        self.next_at = (self.left > 0).then(|| base + PERIOD);
        Some(StopMessage)
    }

    /// Whether a burst is still being sent.
    pub fn is_sending(&self) -> bool {
        self.left > 0
    }

    /// How many times it was pressed since the HUD started.
    pub fn presses(&self) -> u32 {
        self.presses
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Polls every 10 ms for `span`, and returns when each message went out.
    fn run(button: &mut EstopButton, start: Instant, span: Duration) -> Vec<Duration> {
        let step = Duration::from_millis(10);
        let mut sent = Vec::new();
        let mut t = Duration::ZERO;
        while t <= span {
            if button.poll(start + t).is_some() {
                sent.push(t);
            }
            t += step;
        }
        sent
    }

    #[test]
    fn nothing_before_a_press() {
        let mut b = EstopButton::default();
        assert_eq!(run(&mut b, Instant::now(), Duration::from_secs(1)), []);
        assert!(!b.is_sending());
    }

    #[test]
    fn one_press_is_five_messages_a_tenth_apart() {
        let start = Instant::now();
        let mut b = EstopButton::default();
        b.press(start);
        assert!(b.is_sending());
        let sent = run(&mut b, start, Duration::from_secs(2));
        let ms: Vec<u128> = sent.iter().map(Duration::as_millis).collect();
        assert_eq!(ms, [0, 100, 200, 300, 400]);
        assert!(!b.is_sending());
        assert_eq!(b.presses(), 1);
    }

    #[test]
    fn a_slow_caller_still_gets_five_one_at_a_time() {
        let start = Instant::now();
        let mut b = EstopButton::default();
        b.press(start);
        // The screen froze for a second, then polls once a second.
        let late = start + Duration::from_secs(1);
        let got: Vec<_> = (0..10)
            .filter_map(|i| b.poll(late + Duration::from_secs(i)))
            .collect();
        assert_eq!(got.len(), usize::from(REPEATS));
    }

    #[test]
    fn pressing_again_restarts_the_burst() {
        let start = Instant::now();
        let mut b = EstopButton::default();
        b.press(start);
        run(&mut b, start, Duration::from_millis(150)); // two sent
        let again = start + Duration::from_millis(200);
        b.press(again);
        assert_eq!(run(&mut b, again, Duration::from_secs(1)).len(), 5);
        assert_eq!(b.presses(), 2);
    }
}
