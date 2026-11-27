//! The spine's state machine: five regions, each answering one question, all evaluated
//! once per tick in a fixed order (a hierarchical statechart with orthogonal regions).
//!
//! | # | Region | Question | States |
//! |---|---|---|---|
//! | 1 | [`Lifecycle`] | Is the spine ready to run at all? | Boot, SelfTest, Running |
//! | 2 | [`Safety`] | May anything move? | Ok, Estop, Faulted, Degraded, Overspeed, Latched |
//! | 3 | [`Authority`] | Whose commands count? | Idle, Rc, Drive, Test (Armed, Active) |
//! | 4 | [`Motion`] | Which way may the car move? | StandstillHold, Forward, Reverse |
//! | 5 | [`Confidence`] | How far do we trust the measurements? | Full, Degraded |
//!
//! Safety overrides everything: in `Estop`, `Faulted` or `Degraded`, the outputs are
//! throttle 0 and the stop brake, whoever drives. `Overspeed` cuts the throttle and brakes
//! until the car is back under the limit ([`Overspeed`]). `Latched` only keeps the computer
//! from driving until it re-arms (the RC and the test mode still work, as before this
//! module). Authority picks whose request counts; Motion constrains that request.
//! Steering angle, throttle and brake are outputs (numbers every tick), not states.
//!
//! Regions 1 to 3 decide. Regions 4 and 5 are observed: their rules are enforced by the
//! speed loop (direction changes only through a stop) and by `spine-board` (the direction
//! pin changes only with the DAC at 0), and by the configuration ([`SpeedFeedback`]).
//! They are reported here so a test, the log and later the CAN status can see them.
//! Speed is only ever an input: these regions decide what it means.
//!
//! [`SpeedFeedback`]: crate::spine::SpeedFeedback

use crate::protocol::Mode;

/// \[1\] Is the spine ready to run at all?
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Lifecycle {
    /// Before the first tick.
    #[default]
    Boot,
    /// Waiting for every device that stops the car (both Jrks, the DAC, the encoder) to
    /// answer without a fault. Nothing can be armed until it passes.
    SelfTest,
    /// Passed once. A fault after this is the Safety region's business, not a new test.
    Running,
}

impl Lifecycle {
    /// One tick. `devices_ok`: no driver fault that stops the car this tick.
    #[must_use]
    pub fn next(self, devices_ok: bool) -> Lifecycle {
        match self {
            Lifecycle::Boot => Lifecycle::SelfTest,
            Lifecycle::SelfTest if devices_ok => Lifecycle::Running,
            Lifecycle::SelfTest => Lifecycle::SelfTest,
            Lifecycle::Running => Lifecycle::Running,
        }
    }

    /// True once the self-test passed.
    #[must_use]
    pub fn is_running(self) -> bool {
        self == Lifecycle::Running
    }
}

/// \[2\] May anything move? Worked out each tick from what is happening now (an e-stop, a
/// driver fault) and what the spine remembers ([`Latch`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Safety {
    /// Nothing stops the car.
    Ok,
    /// An e-stop is active: the hardware loop is open, or ESTOP_CMD asks for a stop.
    Estop,
    /// A device that stops the car reports a fault.
    Faulted,
    /// An input the car relies on is gone: a lost RC, or a lost wheel speed. A hard stop.
    Degraded,
    /// Too fast: throttle off and braking until back under the limit ([`Overspeed`]).
    Overspeed,
    /// Safe, and waiting for a deliberate re-arm: disarm, then arm. After boot, an
    /// e-stop, a fault, a lost input, a command timeout, an overspeed that didn't clear, or
    /// leaving test mode.
    #[default]
    Latched,
}

impl Safety {
    /// The state for this tick. Highest first: e-stop, fault, lost input, overspeed, latch.
    #[must_use]
    pub fn of(
        estop: bool,
        driver_fault: bool,
        degraded: bool,
        overspeed: bool,
        latch: &Latch,
    ) -> Safety {
        if estop {
            Safety::Estop
        } else if driver_fault {
            Safety::Faulted
        } else if degraded {
            Safety::Degraded
        } else if overspeed {
            Safety::Overspeed
        } else if latch.latched {
            Safety::Latched
        } else {
            Safety::Ok
        }
    }
}

/// What the Safety region remembers between ticks: whether the car is latched, why, and
/// whether the first half of the re-arm (a disarm) has been seen since.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Latch {
    /// Stopped until a deliberate re-arm.
    pub latched: bool,
    /// The latch came from a command timeout (reported as `CMD_TIMEOUT`).
    pub timeout: bool,
    /// The latch came from a driver fault.
    pub fault: bool,
    /// The latch came from a lost wheel speed.
    pub speed_lost: bool,
    /// The latch came from an overspeed that didn't clear in time.
    pub overspeed: bool,
    /// A disarm (`armed = 0`, or a stop on the UART link) arrived since the latch.
    pub seen_disarm: bool,
}

impl Default for Latch {
    /// Boot latched: the computer must disarm, then arm.
    fn default() -> Self {
        Latch {
            latched: true,
            timeout: false,
            fault: false,
            speed_lost: false,
            overspeed: false,
            seen_disarm: false,
        }
    }
}

impl Latch {
    /// Latch: an e-stop, a lost RC, or leaving test mode.
    pub fn trip(&mut self) {
        self.latched = true;
        self.seen_disarm = false;
    }

    /// Latch on a driver fault.
    pub fn trip_fault(&mut self) {
        self.trip();
        self.fault = true;
    }

    /// Latch on a command timeout.
    pub fn trip_timeout(&mut self) {
        self.trip();
        self.timeout = true;
    }

    /// Latch on a lost wheel speed.
    pub fn trip_speed_lost(&mut self) {
        self.trip();
        self.speed_lost = true;
    }

    /// Latch on an overspeed that didn't clear in time.
    pub fn trip_overspeed(&mut self) {
        self.trip();
        self.overspeed = true;
    }

    /// Release on the second half of a re-arm, if nothing still stops the car.
    pub fn try_release(&mut self, stopping: bool, armed_request: bool) {
        if self.latched && !stopping && self.seen_disarm && armed_request {
            self.latched = false;
            self.timeout = false;
            self.fault = false;
            self.speed_lost = false;
            self.overspeed = false;
        }
    }
}

/// How long the car has to get back under the limit, once it trips (ms). Then a hard stop,
/// whether it is still speeding up or only holding its speed.
pub const OVERSPEED_WINDOW_MS: u32 = 2000;

/// The RC's shortest hold after a stop (ms): the car stays braked at least this long after
/// the right stick comes back from its stop, after any other stop, at power-up and when
/// the mode switch moves ([`RcHold`]). With no speed to tell a stop, it is also how long
/// the hard brake must be held before the car changes between forward and reverse.
/// Arsh, 21 Nov: "the timer starts after the right stick springs back to the middle".
pub const RC_BRAKE_MS: u32 = 1000;

/// How long the right stick must be held up to give the car back to the computer (ms).
pub const RC_GIVE_MS: u32 = 500;

/// How long the car must stay under [`STANDSTILL_SPEED`] before the RC changes between
/// forward and reverse (ms). Arsh, 21 Nov: "under 0.05 m/s for about 0.5s".
pub const RC_STOPPED_MS: u32 = 500;

/// How fast the RC's speed ramps up to its fixed speed (m/s^2): 0 to 1 m/s in 2 s
/// (Arsh and Ben, 21 Nov). To be checked on a car day.
pub const RC_ACCEL: f32 = 0.5;

/// The left stick (CH3, up and down), as a person means it: which way they want to go.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Pedal {
    /// Back: reverse, or slow down if the car still moves forward.
    Reverse,
    /// The middle (±20 %): throttle off, so the motor's own braking slows the car.
    #[default]
    Middle,
    /// Up: forward, or slow down if the car still moves backward.
    Forward,
}

/// The right stick, up and down (CH2): it springs back to the middle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RightStick {
    /// Anywhere short of the two ends: steering only.
    #[default]
    Centre,
    /// Pulled down past 88 %: a hard brake while it is held, and the RC takes the car.
    Stop,
    /// Pushed up past 88 % with the steering centred: give the car to the computer, once
    /// held for [`RC_GIVE_MS`].
    Give,
}

/// Inside the Authority region, in normal mode only: does the RC or the computer have the
/// car? (Arsh and Ben, 21 Nov.) Decided on the board, so it works with the computer dead.
///
/// ```text
/// Computer ──right stick down (Stop)─────────────────────► Rc
/// Rc ──right stick up (Give), steering centred, 0.5 s───► Computer
/// ```
///
/// Nothing else changes the owner: the left stick never takes the car from the computer.
/// After any stop, at power-up and when the mode switch moves, the RC has it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RcOwner {
    /// The computer has the car; it still needs arming to drive.
    Computer,
    /// The RC has the car.
    Rc {
        /// Since when the right stick has been held up to give it back (ms).
        give_since: Option<u32>,
    },
}

impl Default for RcOwner {
    fn default() -> Self {
        RcOwner::RC
    }
}

impl RcOwner {
    /// The RC, with nobody giving the car away.
    pub const RC: RcOwner = RcOwner::Rc { give_since: None };

    /// One tick, with the right stick where it is.
    #[must_use]
    pub fn next(self, now_ms: u32, right: RightStick) -> Self {
        match (self, right) {
            (_, RightStick::Stop) => RcOwner::RC,
            (RcOwner::Rc { give_since }, RightStick::Give) => {
                let since = give_since.unwrap_or(now_ms);
                if now_ms.wrapping_sub(since) >= RC_GIVE_MS {
                    RcOwner::Computer
                } else {
                    RcOwner::Rc {
                        give_since: Some(since),
                    }
                }
            }
            (RcOwner::Rc { .. }, RightStick::Centre) => RcOwner::RC,
            (RcOwner::Computer, _) => RcOwner::Computer,
        }
    }

    /// The RC has the car.
    #[must_use]
    pub fn is_rc(self) -> bool {
        matches!(self, RcOwner::Rc { .. })
    }
}

/// The RC held braked after a stop (Arsh, 21 Nov: "even if we let go of the brake, we
/// just assume it is a default zero speed until we move the left stick"). It lets go once
/// [`RC_BRAKE_MS`] have passed and the left stick has left the zone it was in (at any time
/// since), so a stick left pushed never drives the car by itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcHold {
    /// When the stop ended (ms): it restarts every tick while the stop lasts.
    pub since_ms: u32,
    /// Where the left stick was then; `None` with no RC heard yet.
    pub pedal: Option<Pedal>,
    /// The left stick has left that zone since.
    pub moved: bool,
}

impl RcHold {
    /// A hold from `now_ms`, the left stick at `pedal`.
    #[must_use]
    pub fn start(now_ms: u32, pedal: Option<Pedal>) -> Self {
        RcHold {
            since_ms: now_ms,
            pedal,
            moved: false,
        }
    }

    /// One tick with no stop, the left stick at `pedal` (`None`: no RC). The hold, or
    /// `None` once it lets go.
    #[must_use]
    pub fn next(self, now_ms: u32, pedal: Option<Pedal>) -> Option<Self> {
        match (self.pedal, pedal) {
            (_, None) => Some(self),
            // The first RC heard: the stick must leave where it is now.
            (None, Some(p)) => Some(RcHold {
                pedal: Some(p),
                ..self
            }),
            (Some(start), Some(p)) => {
                let moved = self.moved || p != start;
                if moved && now_ms.wrapping_sub(self.since_ms) >= RC_BRAKE_MS {
                    None
                } else {
                    Some(RcHold { moved, ..self })
                }
            }
        }
    }
}

/// Inside the Safety region: is the car bringing itself back under the speed limit? This
/// aims for the limit, not for a stop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Overspeed {
    /// Not over the trip speed.
    #[default]
    Clear,
    /// Past the trip speed at `since_ms`, and not yet back under the limit.
    Slowing {
        /// When it tripped (ms).
        since_ms: u32,
    },
}

impl Overspeed {
    /// One tick. `over_trip`: past the trip speed. `over_limit`: still over the limit.
    #[must_use]
    pub fn next(self, over_trip: bool, over_limit: bool, now_ms: u32) -> Overspeed {
        match self {
            Overspeed::Clear if over_trip => Overspeed::Slowing { since_ms: now_ms },
            Overspeed::Clear => Overspeed::Clear,
            Overspeed::Slowing { .. } if !over_limit => Overspeed::Clear,
            slowing => slowing,
        }
    }

    /// True while coming back under the limit.
    #[must_use]
    pub fn is_slowing(self) -> bool {
        matches!(self, Overspeed::Slowing { .. })
    }

    /// Slowing for the whole window, and still over the limit.
    #[must_use]
    pub fn expired(self, now_ms: u32) -> bool {
        match self {
            Overspeed::Clear => false,
            Overspeed::Slowing { since_ms } => now_ms.wrapping_sub(since_ms) >= OVERSPEED_WINDOW_MS,
        }
    }
}

/// \[3\] Whose commands count? Highest first: Rc, Test, Drive, Idle (Safety is above all).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Authority {
    /// Nobody drives: the actuators idle. On the wire: `Mode::Manual`.
    #[default]
    Idle,
    /// The RC transmitter, on RC direct.
    Rc,
    /// The computer, armed and sending fresh commands. On the wire: `Mode::Computer`.
    Drive,
    /// The hardware test mode (proposal 6.5): one actuator at a time, from a laptop.
    Test(TestStage),
}

impl Authority {
    /// What VEHICLE_STATUS reports.
    #[must_use]
    pub fn mode(self) -> Mode {
        match self {
            Authority::Idle => Mode::Manual,
            Authority::Rc => Mode::Rc,
            Authority::Drive => Mode::Computer,
            Authority::Test(_) => Mode::Test,
        }
    }

    /// True in the hardware test mode.
    #[must_use]
    pub fn in_test(self) -> bool {
        matches!(self, Authority::Test(_))
    }
}

/// Inside [`Authority::Test`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestStage {
    /// In test mode, holding still: throttle 0, brake on.
    Armed,
    /// One actuator moving to the target of the latest test command.
    Active,
}

/// One request to the hardware test mode. How these reach the spine (a new CAN frame, or
/// a bit in DRIVE_CMD) is still the team's choice: until then, only tests and a board
/// program's own code send them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TestRequest {
    /// The first half of entering test mode, like `armed = 0`; also leaves it.
    Exit,
    /// The second half: enter test mode (only after an `Exit`).
    Enter,
    /// Stop moving and hold still (back to `Armed`).
    Hold,
    /// Steering to this angle (rad, positive left), inside the steering limits.
    Steering(f32),
    /// Brake to this position (0..1).
    Brake(f32),
    /// Throttle to this value (0..1), forward only, capped at [`TEST_MAX_THROTTLE`].
    Throttle(f32),
}

impl TestRequest {
    /// The stage a request leaves the test mode in.
    #[must_use]
    pub fn stage(self) -> TestStage {
        match self {
            TestRequest::Steering(_) | TestRequest::Brake(_) | TestRequest::Throttle(_) => {
                TestStage::Active
            }
            TestRequest::Exit | TestRequest::Enter | TestRequest::Hold => TestStage::Armed,
        }
    }

    /// Whether the spine carries this request out at this [`Confidence`]. A throttle test
    /// needs a wheel speed: without one, the "faster than [`TEST_MAX_SPEED`]" exit can't
    /// fire. A refused request holds still, like [`TestRequest::Hold`].
    #[must_use]
    pub fn allowed(self, confidence: Confidence) -> bool {
        match self {
            TestRequest::Throttle(_) => confidence.measures_speed(),
            _ => true,
        }
    }
}

/// The most throttle a test command may ask for (0..1): the wheels are off the ground.
pub const TEST_MAX_THROTTLE: f32 = 0.15;

/// Faster than this (m/s, either way) and the spine leaves test mode.
pub const TEST_MAX_SPEED: f32 = 0.5;

/// Slower than this (m/s) counts as standing still.
pub const STANDSTILL_SPEED: f32 = 0.05;

/// \[4\] Which way may the car move? Direction changes only through `StandstillHold`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Motion {
    /// Stopped, with no throttle: the brake holds the car.
    #[default]
    StandstillHold,
    /// Moving, or about to, forward.
    Forward,
    /// Moving, or about to, backward.
    Reverse,
}

impl Motion {
    /// What this tick's outputs and the measured speed show.
    #[must_use]
    pub fn observe(measured_speed: f32, throttle: f32, reverse: bool) -> Motion {
        let still = measured_speed.abs() < STANDSTILL_SPEED;
        if still && throttle == 0.0 {
            Motion::StandstillHold
        } else if reverse {
            Motion::Reverse
        } else {
            Motion::Forward
        }
    }
}

/// \[5\] How far do we trust the measurements?
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Confidence {
    /// A wheel speed, from the board or sent over the UART link: the overspeed check works.
    #[default]
    Full,
    /// No wheel speed: open loop, throttle from the speed asked for, no overspeed check.
    /// A board with none whose link sends none (`SpeedFeedback::None`); or one whose link
    /// did send it and stopped, which the Safety region also treats as a hard stop.
    Degraded,
}

impl Confidence {
    /// The spine sees the car's speed: the overspeed check and the test mode's speed
    /// exit work.
    #[must_use]
    pub fn measures_speed(self) -> bool {
        matches!(self, Confidence::Full)
    }
}

/// All five regions, as the spine left them after the latest tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct State {
    /// \[1\] Booting, testing itself, or running.
    pub lifecycle: Lifecycle,
    /// \[2\] Stopped by a rule, or free to drive.
    pub safety: Safety,
    /// \[3\] Who drives.
    pub authority: Authority,
    /// \[4\] Which way the car may move.
    pub motion: Motion,
    /// \[5\] How far the measurements are trusted.
    pub confidence: Confidence,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_right_stick_down_takes_the_car_and_held_up_gives_it_back() {
        assert_eq!(RcOwner::Computer.next(0, RightStick::Stop), RcOwner::RC);
        // Nothing else takes it from the computer.
        for right in [RightStick::Centre, RightStick::Give] {
            assert_eq!(RcOwner::Computer.next(0, right), RcOwner::Computer);
        }
        // Up for 0.5 s gives it back; let go early and the count starts again.
        let o = RcOwner::RC.next(0, RightStick::Give);
        assert!(o.next(499, RightStick::Give).is_rc());
        assert_eq!(o.next(500, RightStick::Give), RcOwner::Computer);
        let o = o.next(300, RightStick::Centre).next(400, RightStick::Give);
        assert!(o.next(899, RightStick::Give).is_rc());
        assert_eq!(o.next(900, RightStick::Give), RcOwner::Computer);
        assert!(RcOwner::default().is_rc(), "the RC has the car at power-up");
    }

    #[test]
    fn the_hold_lets_go_after_a_second_once_the_left_stick_moves() {
        let h = RcHold::start(0, Some(Pedal::Forward));
        // The stick still where it was: held, however long.
        assert!(h.next(5000, Some(Pedal::Forward)).is_some());
        // Moved, but inside the second: still held.
        let moved = h.next(999, Some(Pedal::Middle)).unwrap();
        assert_eq!(h.next(1000, Some(Pedal::Middle)), None);
        assert_eq!(h.next(1000, Some(Pedal::Reverse)), None);
        // Moved inside the second and back: it still counts once the second is over.
        let back = moved.next(999, Some(Pedal::Forward)).unwrap();
        assert_eq!(back.next(1000, Some(Pedal::Forward)), None);
        // No RC: held.
        assert!(h.next(5000, None).is_some());
        // No RC heard at the start: the stick must leave where it is when it is first heard.
        let h = RcHold::start(0, None)
            .next(2000, Some(Pedal::Forward))
            .unwrap();
        assert!(h.next(3000, Some(Pedal::Forward)).is_some());
        assert_eq!(h.next(3000, Some(Pedal::Middle)), None);
    }

    #[test]
    fn lifecycle_boots_into_the_self_test_and_passes_only_without_faults() {
        let s = Lifecycle::Boot.next(true);
        assert_eq!(s, Lifecycle::SelfTest);
        assert_eq!(s.next(false), Lifecycle::SelfTest);
        assert_eq!(s.next(true), Lifecycle::Running);
        // Running never goes back to a self-test: Safety handles later faults.
        assert_eq!(Lifecycle::Running.next(false), Lifecycle::Running);
    }

    #[test]
    fn safety_ranks_estop_then_fault_then_latch() {
        let latched = Latch::default();
        let free = Latch {
            latched: false,
            ..Latch::default()
        };
        assert_eq!(Safety::of(true, true, true, true, &latched), Safety::Estop);
        assert_eq!(
            Safety::of(false, true, true, true, &latched),
            Safety::Faulted
        );
        assert_eq!(
            Safety::of(false, false, true, true, &latched),
            Safety::Degraded
        );
        assert_eq!(
            Safety::of(false, false, false, true, &latched),
            Safety::Overspeed
        );
        assert_eq!(
            Safety::of(false, false, false, false, &latched),
            Safety::Latched
        );
        assert_eq!(Safety::of(false, false, false, false, &free), Safety::Ok);
    }

    #[test]
    fn a_latch_releases_only_on_disarm_then_arm_with_nothing_stopping_the_car() {
        let mut l = Latch::default();
        l.try_release(false, true);
        assert!(l.latched, "no disarm seen yet");
        l.seen_disarm = true;
        l.try_release(true, true);
        assert!(l.latched, "something still stops the car");
        l.try_release(false, false);
        assert!(l.latched, "no arm yet");
        l.try_release(false, true);
        assert!(!l.latched);
    }

    #[test]
    fn a_trip_forgets_the_disarm_and_records_why() {
        let mut l = Latch {
            latched: false,
            seen_disarm: true,
            ..Latch::default()
        };
        l.trip_timeout();
        assert!(l.latched && l.timeout && !l.fault && !l.seen_disarm);
        l.seen_disarm = true;
        l.trip_fault();
        assert!(l.fault && !l.seen_disarm);
        l.seen_disarm = true;
        l.trip_speed_lost();
        assert!(l.speed_lost && !l.seen_disarm);
        l.seen_disarm = true;
        l.trip_overspeed();
        assert!(l.overspeed && !l.seen_disarm);
        l.seen_disarm = true;
        l.try_release(false, true);
        assert!(!l.timeout && !l.fault && !l.speed_lost && !l.overspeed);
    }

    #[test]
    fn overspeed_starts_past_the_trip_and_ends_only_under_the_limit() {
        let clear = Overspeed::Clear;
        assert_eq!(clear.next(false, true, 100), Overspeed::Clear);
        let slowing = clear.next(true, true, 100);
        assert_eq!(slowing, Overspeed::Slowing { since_ms: 100 });
        assert_eq!(
            slowing.next(false, true, 200),
            slowing,
            "still over the limit"
        );
        assert_eq!(
            slowing.next(true, true, 200),
            slowing,
            "the clock doesn't restart"
        );
        assert_eq!(slowing.next(false, false, 200), Overspeed::Clear);
    }

    #[test]
    fn overspeed_expires_after_its_window_even_across_a_clock_wrap() {
        let since_ms = u32::MAX - 500;
        let slowing = Overspeed::Slowing { since_ms };
        assert!(!slowing.expired(since_ms.wrapping_add(OVERSPEED_WINDOW_MS - 1)));
        assert!(slowing.expired(since_ms.wrapping_add(OVERSPEED_WINDOW_MS)));
        assert!(!Overspeed::Clear.expired(u32::MAX));
    }

    #[test]
    fn authority_maps_to_the_wire_mode() {
        assert_eq!(Authority::Idle.mode(), Mode::Manual);
        assert_eq!(Authority::Rc.mode(), Mode::Rc);
        assert_eq!(Authority::Drive.mode(), Mode::Computer);
        assert_eq!(Authority::Test(TestStage::Armed).mode(), Mode::Test);
        assert_eq!(Authority::Test(TestStage::Active).mode(), Mode::Test);
    }

    #[test]
    fn test_requests_that_move_an_actuator_are_active() {
        assert_eq!(TestRequest::Steering(0.1).stage(), TestStage::Active);
        assert_eq!(TestRequest::Brake(0.5).stage(), TestStage::Active);
        assert_eq!(TestRequest::Throttle(0.1).stage(), TestStage::Active);
        assert_eq!(TestRequest::Hold.stage(), TestStage::Armed);
        assert_eq!(TestRequest::Enter.stage(), TestStage::Armed);
    }

    #[test]
    fn a_throttle_test_needs_a_wheel_speed() {
        assert!(TestRequest::Throttle(0.1).allowed(Confidence::Full));
        assert!(!TestRequest::Throttle(0.1).allowed(Confidence::Degraded));
        for req in [
            TestRequest::Steering(0.1),
            TestRequest::Brake(0.5),
            TestRequest::Hold,
        ] {
            assert!(req.allowed(Confidence::Degraded), "{req:?}");
        }
    }

    #[test]
    fn motion_is_a_standstill_only_when_still_and_without_throttle() {
        assert_eq!(Motion::observe(0.0, 0.0, false), Motion::StandstillHold);
        assert_eq!(Motion::observe(0.0, 0.0, true), Motion::StandstillHold);
        assert_eq!(Motion::observe(0.0, 0.2, false), Motion::Forward);
        assert_eq!(Motion::observe(-0.3, 0.0, true), Motion::Reverse);
        assert_eq!(Motion::observe(1.0, 0.0, false), Motion::Forward);
    }
}
