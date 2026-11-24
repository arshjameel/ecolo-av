//! The RC on the bench: a receiver's iBUS channels in, decoded as the board decodes them,
//! and a gamepad standing in for the FS-i6X transmitter (`rc_pad`, 23 Nov).
//!
//! [`Receiver`] is the board's side, from the ibus crate unchanged: each set of channels
//! goes in as the bytes of a real iBUS frame, through the board's parser, checksum and
//! silence timeout ([`RC_TIMEOUT_MS`]), then [`RcMap::rc_input`] (the same thresholds and
//! failsafe as `esp32-umod`). [`Pad`] is the transmitter's side: a gamepad's sticks and
//! buttons as the FS-i6X's channels, in µs.

pub use ibus::{Channels, RcMap, CHANNELS};
use spine_core::RcInput;

/// Silence longer than this is a lost RC signal (ms): spine-board's `rc_timeout_ms`.
pub const RC_TIMEOUT_MS: u32 = 100;

/// The board's receiver: iBUS frames in, what the RC asks for out.
#[derive(Debug, Clone)]
pub struct Receiver {
    map: RcMap,
    wire: ibus::Receiver,
}

impl Receiver {
    /// A receiver read with `map` (today's car: [`RcMap::TODAY`]). Silent until the first
    /// frame, so the spine sees a lost RC until then, as on the car.
    #[must_use]
    pub fn new(map: RcMap) -> Self {
        Receiver {
            map,
            wire: ibus::Receiver::new(RC_TIMEOUT_MS),
        }
    }

    /// One frame's channels, arriving at `now_ms`: encoded as iBUS, fed byte by byte.
    pub fn receive(&mut self, channels: &Channels, now_ms: u32) {
        for byte in ibus::encode(channels) {
            self.wire.push(byte, now_ms);
        }
    }

    /// What the RC asks for at `now_ms`: None if silent past [`RC_TIMEOUT_MS`], or if the
    /// channels say so (the failsafe, a mode switch out of range).
    #[must_use]
    pub fn read(&self, now_ms: u32) -> Option<RcInput> {
        self.wire
            .channels(now_ms)
            .and_then(|c| self.map.rc_input(&c))
    }
}

/// A channel's value with its stick at rest, and the FS-i6X's ends (µs).
const CENTRE_US: f32 = 1500.0;
const HALF_TRAVEL_US: f32 = 500.0;
/// The FS-i6X's sticks reach -100 % one way and only +97 % the other (22 Nov): 1985 µs.
const STICK_DOWN_END_US: f32 = 1985.0;
/// The mode switch (CH6), with End points at 80 %: low 1100 µs (normal), high 1900
/// (manual, RC direct); the failsafe's value is 1000, under `lost_below`.
pub const MODE_LOW_US: u16 = 1100;
/// The mode switch high: manual (RC direct).
pub const MODE_HIGH_US: u16 = 1900;
/// The failsafe's CH6: the transmitter's radio lost.
pub const FAILSAFE_US: u16 = 1000;

/// Where a gamepad's sticks and buttons are, in sensor_msgs/Joy's axes and buttons.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PadLayout {
    /// The left stick up and down (up +1): the pedal, CH3.
    pub pedal_axis: usize,
    /// The right stick up and down (up +1): stop and give, CH2.
    pub stop_axis: usize,
    /// The right stick left and right (left +1): steering, CH4.
    pub steer_axis: usize,
    /// Toggles the mode switch, CH6.
    pub mode_button: usize,
    /// Held: the radio lost (CH6 at the failsafe's 1000 µs).
    pub lost_button: usize,
}

impl PadLayout {
    /// A DualShock 4 (CUH-ZCT2) through joy's game_controller_node, measured 23 Nov:
    /// the left stick's up and down is axis 1, the right stick's axis 3 and its left and
    /// right axis 2 (up and left +1); R1 is button 10, L1 button 9.
    pub const PS4: PadLayout = PadLayout {
        pedal_axis: 1,
        stop_axis: 3,
        steer_axis: 2,
        mode_button: 10,
        lost_button: 9,
    };
}

/// A gamepad as the FS-i6X: its sticks and buttons, each message, to the channels the
/// receiver would send.
#[derive(Debug, Clone)]
pub struct Pad {
    layout: PadLayout,
    map: RcMap,
    mode_high: bool,
    mode_was_pressed: bool,
}

impl Pad {
    /// A pad with `layout`, its channels where `map` reads them, the mode switch starting
    /// high (manual) or low (normal).
    #[must_use]
    pub fn new(layout: PadLayout, map: RcMap, mode_high: bool) -> Self {
        Pad {
            layout,
            map,
            mode_high,
            mode_was_pressed: false,
        }
    }

    /// Whether the mode switch is high (manual) now.
    #[must_use]
    pub fn mode_high(&self) -> bool {
        self.mode_high
    }

    /// The channels for one Joy message: `axes` -1..1, `buttons` 0 or 1. A press of the
    /// mode button flips the switch (on the press, not while held). A missing axis or
    /// button reads as at rest.
    pub fn channels(&mut self, axes: &[f32], buttons: &[i32]) -> Channels {
        let axis = |i: usize| axes.get(i).copied().unwrap_or(0.0);
        let pressed = |i: usize| buttons.get(i).is_some_and(|&b| b != 0);
        let mode_pressed = pressed(self.layout.mode_button);
        if mode_pressed && !self.mode_was_pressed {
            self.mode_high = !self.mode_high;
        }
        self.mode_was_pressed = mode_pressed;

        let mut ch = [CENTRE_US as u16; CHANNELS];
        ch[self.map.throttle] = stick_us(axis(self.layout.pedal_axis), STICK_DOWN_END_US);
        ch[self.map.stop_stick] = stick_us(axis(self.layout.stop_axis), STICK_DOWN_END_US);
        ch[self.map.steering] = stick_us(axis(self.layout.steer_axis), CENTRE_US + HALF_TRAVEL_US);
        ch[self.map.mode_switch] = if pressed(self.layout.lost_button) {
            FAILSAFE_US
        } else if self.mode_high {
            MODE_HIGH_US
        } else {
            MODE_LOW_US
        };
        Channels(ch)
    }
}

/// A stick, -1..1 with up (or left) +1, as the FS-i6X's µs: up 1000, down at most `low_end`.
fn stick_us(value: f32, low_end: f32) -> u16 {
    let v = if value.is_finite() {
        value.clamp(-1.0, 1.0)
    } else {
        0.0
    };
    (CENTRE_US - HALF_TRAVEL_US * v).round().min(low_end) as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use spine_core::{RcSwitch, RightStick};

    const AXES_AT_REST: [f32; 6] = [0.0; 6];
    const NO_BUTTONS: [i32; 21] = [0; 21];

    fn pad() -> Pad {
        Pad::new(PadLayout::PS4, RcMap::TODAY, true)
    }

    fn with_axis(i: usize, v: f32) -> [f32; 6] {
        let mut a = AXES_AT_REST;
        a[i] = v;
        a
    }

    fn with_button(i: usize) -> [i32; 21] {
        let mut b = NO_BUTTONS;
        b[i] = 1;
        b
    }

    #[test]
    fn at_rest_every_stick_is_centred_and_the_switch_high() {
        let ch = pad().channels(&AXES_AT_REST, &NO_BUTTONS).0;
        assert_eq!(ch[RcMap::TODAY.throttle], 1500);
        assert_eq!(ch[RcMap::TODAY.stop_stick], 1500);
        assert_eq!(ch[RcMap::TODAY.steering], 1500);
        assert_eq!(ch[RcMap::TODAY.mode_switch], MODE_HIGH_US);
    }

    #[test]
    fn up_and_left_are_1000_down_stops_at_97_percent() {
        let mut p = pad();
        let l = PadLayout::PS4;
        assert_eq!(
            p.channels(&with_axis(l.pedal_axis, 1.0), &NO_BUTTONS).0[2],
            1000
        );
        assert_eq!(
            p.channels(&with_axis(l.pedal_axis, -1.0), &NO_BUTTONS).0[2],
            1985
        );
        assert_eq!(
            p.channels(&with_axis(l.stop_axis, 1.0), &NO_BUTTONS).0[1],
            1000
        );
        assert_eq!(
            p.channels(&with_axis(l.stop_axis, -1.0), &NO_BUTTONS).0[1],
            1985
        );
        assert_eq!(
            p.channels(&with_axis(l.steer_axis, 1.0), &NO_BUTTONS).0[3],
            1000
        );
        assert_eq!(
            p.channels(&with_axis(l.steer_axis, -1.0), &NO_BUTTONS).0[3],
            2000
        );
        assert_eq!(
            p.channels(&with_axis(l.steer_axis, 0.5), &NO_BUTTONS).0[3],
            1250
        );
    }

    #[test]
    fn a_bad_axis_reads_as_at_rest_and_past_the_end_as_the_end() {
        let mut p = pad();
        let l = PadLayout::PS4;
        assert_eq!(
            p.channels(&with_axis(l.pedal_axis, f32::NAN), &NO_BUTTONS)
                .0[2],
            1500
        );
        assert_eq!(
            p.channels(&with_axis(l.pedal_axis, 3.0), &NO_BUTTONS).0[2],
            1000
        );
        assert_eq!(p.channels(&[], &[]).0[2], 1500);
    }

    #[test]
    fn r1_flips_the_mode_switch_once_per_press() {
        let mut p = pad();
        let r1 = with_button(PadLayout::PS4.mode_button);
        assert_eq!(p.channels(&AXES_AT_REST, &r1).0[5], MODE_LOW_US);
        assert_eq!(
            p.channels(&AXES_AT_REST, &r1).0[5],
            MODE_LOW_US,
            "held: no flip"
        );
        assert_eq!(p.channels(&AXES_AT_REST, &NO_BUTTONS).0[5], MODE_LOW_US);
        assert_eq!(p.channels(&AXES_AT_REST, &r1).0[5], MODE_HIGH_US);
    }

    #[test]
    fn l1_held_is_the_failsafe_and_the_switch_comes_back() {
        let mut p = pad();
        let l1 = with_button(PadLayout::PS4.lost_button);
        assert_eq!(p.channels(&AXES_AT_REST, &l1).0[5], FAILSAFE_US);
        assert_eq!(p.channels(&AXES_AT_REST, &NO_BUTTONS).0[5], MODE_HIGH_US);
    }

    #[test]
    fn the_receiver_decodes_the_pad_as_the_board_does() {
        let mut p = pad();
        let mut rx = Receiver::new(RcMap::TODAY);
        let l = PadLayout::PS4;
        let mut now = 0;
        let mut send = |rx: &mut Receiver, ch: Channels| {
            now += 10;
            rx.receive(&ch, now);
            rx.read(now).expect("an RC")
        };

        let rc = send(&mut rx, p.channels(&AXES_AT_REST, &NO_BUTTONS));
        assert_eq!(rc.switch, RcSwitch::Direct);
        assert!(rc.middle);
        assert_eq!(rc.right, RightStick::Centre);

        let rc = send(
            &mut rx,
            p.channels(&with_axis(l.pedal_axis, 1.0), &NO_BUTTONS),
        );
        assert!(rc.throttle > 0.99 && rc.full && !rc.middle, "{rc:?}");

        let rc = send(
            &mut rx,
            p.channels(&with_axis(l.stop_axis, -1.0), &NO_BUTTONS),
        );
        assert_eq!(rc.right, RightStick::Stop);
        let rc = send(
            &mut rx,
            p.channels(&with_axis(l.stop_axis, 1.0), &NO_BUTTONS),
        );
        assert_eq!(rc.right, RightStick::Give);

        // Give needs the steering within 15 % of centre: 14 % gives, 20 % doesn't.
        let mut up_and_aside = with_axis(l.stop_axis, 1.0);
        up_and_aside[l.steer_axis] = 0.14;
        let rc = send(&mut rx, p.channels(&up_and_aside, &NO_BUTTONS));
        assert_eq!(rc.right, RightStick::Give);
        up_and_aside[l.steer_axis] = 0.2;
        let rc = send(&mut rx, p.channels(&up_and_aside, &NO_BUTTONS));
        assert_ne!(rc.right, RightStick::Give);

        rx.receive(
            &p.channels(&AXES_AT_REST, &with_button(l.lost_button)),
            1000,
        );
        assert_eq!(rx.read(1000), None, "the failsafe: no RC");
    }

    #[test]
    fn silence_past_the_timeout_is_no_rc() {
        let mut rx = Receiver::new(RcMap::TODAY);
        assert_eq!(rx.read(0), None, "nothing heard yet");
        rx.receive(&pad().channels(&AXES_AT_REST, &NO_BUTTONS), 1000);
        assert!(rx.read(1100).is_some(), "100 ms: still fresh");
        assert_eq!(rx.read(1101), None, "101 ms of silence");
    }

    /// The whole chain on the model car: the pad's channels, as iBUS bytes, through the
    /// board's parser to spine-core, with the mode switch high (manual, RC direct).
    mod on_the_bench {
        use super::*;
        use crate::{Bench, PlantConfig, Pose, SpineConfig, BENCH_WHEELS};

        struct PadRig {
            bench: Bench,
            pad: Pad,
        }

        impl PadRig {
            fn new() -> Self {
                let spine = SpineConfig {
                    rc_fitted: true,
                    ..SpineConfig::default()
                };
                let mut bench =
                    Bench::new(spine, PlantConfig::default(), Pose::default(), BENCH_WHEELS)
                        .expect("the bench's wheel settings are valid");
                bench.fit_rc(RcMap::TODAY);
                PadRig {
                    bench,
                    pad: Pad::new(PadLayout::PS4, RcMap::TODAY, true),
                }
            }

            /// `seconds` of 10 ms ticks, the pad sending every 20 ms (`axes`, `buttons`),
            /// or nothing at all (`None`: the pad silent).
            fn run(&mut self, seconds: f32, pad: Option<(&[f32], &[i32])>) {
                for i in 0..(seconds * 100.0) as usize {
                    if let (Some((axes, buttons)), true) = (pad, i % 2 == 0) {
                        let ch = self.pad.channels(axes, buttons);
                        self.bench.rc_channels(&ch);
                    }
                    self.bench.tick(10);
                }
            }

            fn speed(&self) -> f32 {
                self.bench.pose().speed
            }
        }

        const PEDAL_UP: [f32; 6] = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0];

        /// Past the hold: a second with the left stick in its middle, then up.
        fn driving() -> PadRig {
            let mut rig = PadRig::new();
            rig.run(1.2, Some((&AXES_AT_REST, &NO_BUTTONS)));
            assert_eq!(rig.speed(), 0.0, "held for the first second");
            // RC_ACCEL's 0.5 m/s² ramp to 1 m/s, and the speed loop behind it.
            rig.run(4.0, Some((&PEDAL_UP, &NO_BUTTONS)));
            assert!(rig.speed() > 0.8, "the pedal up drives: {}", rig.speed());
            rig
        }

        #[test]
        fn the_left_stick_up_drives_once_the_hold_is_past() {
            let rig = driving();
            assert!(rig.speed() < 1.1, "the RC's 1 m/s: {}", rig.speed());
        }

        #[test]
        fn the_right_stick_down_stops() {
            let mut rig = driving();
            let mut stop = PEDAL_UP;
            stop[PadLayout::PS4.stop_axis] = -1.0;
            rig.run(3.0, Some((&stop, &NO_BUTTONS)));
            assert!(rig.speed() < 0.05, "stopped: {}", rig.speed());
        }

        #[test]
        fn l1_the_radio_lost_stops() {
            let mut rig = driving();
            rig.run(
                3.0,
                Some((&PEDAL_UP, &with_button(PadLayout::PS4.lost_button))),
            );
            assert!(rig.speed() < 0.05, "stopped: {}", rig.speed());
        }

        #[test]
        fn a_silent_pad_stops() {
            let mut rig = driving();
            rig.run(3.0, None);
            assert!(rig.speed() < 0.05, "stopped: {}", rig.speed());
        }

        #[test]
        fn after_a_stop_it_holds_until_the_left_stick_leaves_its_zone() {
            let mut rig = driving();
            rig.run(
                3.0,
                Some((&PEDAL_UP, &with_button(PadLayout::PS4.lost_button))),
            );
            // The radio back, the stick still up: held, since it never left its zone.
            rig.run(3.0, Some((&PEDAL_UP, &NO_BUTTONS)));
            assert!(rig.speed() < 0.05, "still held: {}", rig.speed());
            rig.run(0.5, Some((&AXES_AT_REST, &NO_BUTTONS)));
            rig.run(4.0, Some((&PEDAL_UP, &NO_BUTTONS)));
            assert!(rig.speed() > 0.8, "drives again: {}", rig.speed());
        }
    }
}
