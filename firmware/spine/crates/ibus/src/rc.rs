//! From channels to `spine_core::RcInput`: the mode switch, the two sticks and steering.
//!
//! The channel numbers are the 2025-26 firmware's (`main.cpp` `loop()`, `rc_handler.cpp`
//! `handleRCMode()`). It counted channels from 0, as its iBUS library does, so
//! "channel 5" there is CH6 on the transmitter.
//!
//! | Index | On the transmitter | Use |
//! |---|---|---|
//! | 5 | CH6 | mode switch: normal, RC via ROS, manual (RC direct) |
//! | 2 | CH3 | the left stick, up and down: forward, the middle, reverse |
//! | 3 | CH4 | the right stick, sideways: steering |
//! | 1 | CH2 | the right stick, up and down: the stop, and giving the car back |
//! | 4 | CH5 | (SwC) not read: its wiring is loose (Arsh, 21 Nov) |
//!
//! **The left stick (Arsh and Ben, 21 Nov).** The FS-i6X's left stick does not spring back
//! to the middle, so where it rests is a choice the driver made:
//!
//! ```text
//!   1000 ◄───────────────────── 1500 ─────────────────────► 2000   (µs, CH3)
//!   forward    │ full │  forward   middle (±20 %)   reverse  │ full │   reverse
//!              past 88 %                                       past 88 %
//! ```
//!
//! [`RcInput::middle`] is the middle zone: throttle off. [`RcInput::full`] is past 88 %:
//! with the car still going the other way, the hard brake. [`RcInput::throttle`] is still
//! rescaled from 0 at the middle's edge to ±1 at the stick's end, for the uplink; the
//! spine only reads its sign. Which end is forward follows the C++ (it counted the throttle
//! below 1500); [`RcMap::throttle_reversed`] swaps them. TODO(Q7): check on the
//! transmitter.
//!
//! **The right stick (CH2) springs back.** Pulled down past 88 % (towards 2000) is
//! [`RightStick::Stop`]; pushed up past 88 % with the steering (CH4) within 15 % of its
//! middle is [`RightStick::Give`]. The FS-i6X's CH2 reaches -100 % up and +97 % down
//! (Arsh, 21 Nov), so 88 % leaves room both ways.
//!
//! **The failsafe (Arsh, 21 Nov).** The transmitter's failsafe sets CH6 to 1000 µs, below
//! anything the switch sends: the FS-i6X can't go past -100%, so the switch's low position
//! is moved up to 1100 instead (End points CH6, low side 80%). A CH6 under
//! [`RcMap::lost_below`] (1050) means the radio link is gone, the same as a silent
//! receiver: [`RcMap::rc_input`] gives `None`, and `spine-core` makes a hard stop
//! (`RC_SIGNAL_LOST`). CH2's failsafe is its stop end. The car day check is in
//! `docs/car-day.md`, "The RC transmitter".
//!
//! **The mode switch's three positions (TODO 48).** [`RcMap::TODAY`] reads CH6 as the C++
//! did: two positions, normal or RC direct. [`RcMap::THREE_POSITION`] adds RC via ROS in
//! the middle, for a three-position switch on CH6 (the transmitter's aux-channel menu):
//!
//! | CH6 | `TODAY` | `THREE_POSITION` |
//! |---|---|---|
//! | under 1050 | the failsafe (1000): RC lost | the failsafe (1000): RC lost |
//! | 1100 (low) | normal (`RcSwitch::Computer`) | normal |
//! | 1500 (middle) | normal | **RC via ROS** (1301..1700) |
//! | 2000 (high) | manual (`RcSwitch::Direct`) | manual (above 1700) |
//!
//! A two-position switch on CH6 never reads the middle, so it works the same with both.

use crate::frame::Channels;
use spine_core::{RcInput, RcSwitch, RightStick};

/// Which channel does what, and where the switches change over.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcMap {
    /// Channel indexes, from 0 (index 0 is CH1).
    pub mode_switch: usize,
    /// The left stick's up and down channel: forward, the middle, reverse.
    pub throttle: usize,
    /// The steering stick's channel (the right stick, sideways).
    pub steering: usize,
    /// The right stick's up and down channel: the stop (down) and giving the car back (up).
    pub stop_stick: usize,
    /// A channel is usable only strictly between these (the C++: 100 and 2200).
    pub valid_above: u16,
    /// The top of the usable range (µs).
    pub valid_below: u16,
    /// The mode switch below this is the transmitter's failsafe: the radio link is gone
    /// (µs). The transmitter sets CH6 to 1000 when it loses the link, and the switch's low
    /// position sends 1100.
    pub lost_below: u16,
    /// RC direct while the mode switch reads above this.
    pub direct_above: u16,
    /// RC via ROS while it reads above this, up to `direct_above`; `None`: no such
    /// position (normal up to `direct_above`).
    pub via_ros_above: Option<u16>,
    /// A stick's middle, and how far it travels each way (µs).
    pub centre: u16,
    /// How far a stick travels from its middle to either end (µs).
    pub half_travel: u16,
    /// The middle zone: the left stick this close to its middle, as a share of its
    /// travel each way, is throttle off (0..1).
    pub brake_band: f32,
    /// A stick past this share of its travel is at its end (0..1): the left stick's hard
    /// brake, the right stick's stop and give.
    pub full: f32,
    /// The steering this close to its middle, as a share of its travel, is centred (0..1):
    /// the right stick can give the car back only then.
    pub centred: f32,
    /// False: the left stick below its middle (towards 1000) is forward, as the C++ read
    /// it. True: above its middle is forward.
    pub throttle_reversed: bool,
    /// The steering stick's gain to the right, over its gain to the left: 1.0 today, the
    /// same both ways. The C++ steered right harder (`legacy_link::STEER_RIGHT_PER_LEFT`,
    /// 1950 / 1550): the stick reached full right lock at 80 % of its travel. The board's
    /// `cpp-rc-steering` feature sets that.
    pub steer_right_gain: f32,
}

impl Default for RcMap {
    fn default() -> Self {
        Self::TODAY
    }
}

impl RcMap {
    /// Today's car.
    pub const TODAY: RcMap = RcMap {
        mode_switch: 5,
        throttle: 2,
        steering: 3,
        stop_stick: 1,
        valid_above: 100,
        valid_below: 2200,
        lost_below: 1050,
        direct_above: 1500,
        via_ros_above: None,
        centre: 1500,
        half_travel: 500,
        brake_band: 0.20,
        full: 0.88,
        centred: 0.15,
        throttle_reversed: false,
        steer_right_gain: 1.0,
    };

    /// Today's channels with the mode switch's middle as RC via ROS (TODO 48,
    /// proposed): the board's choice when it sends the sticks up (`uplink`).
    pub const THREE_POSITION: RcMap = RcMap {
        direct_above: 1700,
        via_ros_above: Some(1300),
        ..RcMap::TODAY
    };

    /// Where the mode switch is, for its channel's value.
    #[must_use]
    pub fn switch(&self, value: u16) -> RcSwitch {
        if value > self.direct_above {
            RcSwitch::Direct
        } else if self.via_ros_above.is_some_and(|above| value > above) {
            RcSwitch::ViaRos
        } else {
            RcSwitch::Computer
        }
    }

    /// What the RC asks for, or None if there is no usable RC: the mode switch out of
    /// range, or on the transmitter's failsafe (under `lost_below`).
    ///
    /// - Every position: the switch, and the sticks: throttle and steering, each -1 to
    ///   1, whether the left stick is in its middle or at its end, and the right stick. In
    ///   normal and manual the spine drives from them; via ROS the board sends them up.
    /// - A stick out of range: the right stick's stop and straight wheels. The C++ did a
    ///   safe stop here (`jrkSafeStop()`).
    #[must_use]
    pub fn rc_input(&self, ch: &Channels) -> Option<RcInput> {
        let usable = |index: usize| {
            ch.get(index)
                .filter(|&v| v > self.valid_above && v < self.valid_below)
        };
        let mode = usable(self.mode_switch)?;
        if mode < self.lost_below {
            return None;
        }
        let switch = self.switch(mode);
        let (Some(throttle), Some(steering), Some(stop_stick)) = (
            usable(self.throttle),
            usable(self.steering),
            usable(self.stop_stick),
        ) else {
            return Some(RcInput {
                switch,
                right: RightStick::Stop,
                ..RcInput::default()
            });
        };
        let (throttle, middle, full) = self.left_stick(throttle);
        Some(RcInput {
            switch,
            throttle,
            steering: self.steering(steering),
            middle,
            full,
            right: self.right_stick(stop_stick, steering),
        })
    }

    /// The steering stick, -1 (right) to 1 (left), with the right side's gain.
    fn steering(&self, value: u16) -> f32 {
        let s = self.below_centre(value);
        let s = if s < 0.0 {
            s * self.steer_right_gain
        } else {
            s
        };
        s.clamp(-1.0, 1.0)
    }

    /// How far a stick is below its middle: 1 at 1000, -1 at 2000.
    fn below_centre(&self, value: u16) -> f32 {
        (f32::from(self.centre) - f32::from(value)) / f32::from(self.half_travel)
    }

    /// The left stick: the throttle (-1..1, positive forward), whether it is in its
    /// middle, and whether it is at its end. Outside the middle the throttle runs from 0 at
    /// its edge to 1 at the end.
    fn left_stick(&self, value: u16) -> (f32, bool, bool) {
        let push = self.below_centre(value).clamp(-1.0, 1.0);
        let push = if self.throttle_reversed { -push } else { push };
        let band = self.brake_band.clamp(0.0, 0.99);
        let size = size(push);
        if size <= band {
            return (0.0, true, false);
        }
        let throttle = ((size - band) / (1.0 - band)).clamp(0.0, 1.0);
        (
            if push < 0.0 { -throttle } else { throttle },
            false,
            size >= self.full,
        )
    }

    /// The right stick, up and down: down (towards 2000) past `full` is the stop; up past
    /// it, with the steering within `centred` of its middle, gives the car back.
    fn right_stick(&self, value: u16, steering: u16) -> RightStick {
        let up = self.below_centre(value);
        if -up >= self.full {
            RightStick::Stop
        } else if up >= self.full && size(self.below_centre(steering)) <= self.centred {
            RightStick::Give
        } else {
            RightStick::Centre
        }
    }
}

/// How far from the middle, either way. No `f32::abs` in `no_std` on every toolchain.
fn size(push: f32) -> f32 {
    if push < 0.0 {
        -push
    } else {
        push
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::CHANNELS;

    /// Channels with the mode switch, the left stick and steering set; the rest at 1500.
    fn ch(mode_switch: u16, throttle: u16, steering: u16) -> Channels {
        let mut c = [1500; CHANNELS];
        c[5] = mode_switch;
        c[2] = throttle;
        c[3] = steering;
        Channels(c)
    }

    fn rc(c: Channels) -> Option<RcInput> {
        RcMap::default().rc_input(&c)
    }

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    #[test]
    fn the_right_gain_steers_right_harder_and_still_clamps() {
        let cpp = RcMap {
            steer_right_gain: 1950.0 / 1550.0,
            ..RcMap::TODAY
        };
        let steer = |map: &RcMap, us| map.rc_input(&ch(2000, 1500, us)).unwrap().steering;
        assert_eq!(steer(&RcMap::TODAY, 1750), -0.5);
        assert!((steer(&cpp, 1750) + 0.5 * 1950.0 / 1550.0).abs() < 1e-6);
        assert_eq!(steer(&cpp, 1250), 0.5); // left as before
        assert_eq!(steer(&cpp, 2000), -1.0); // full right lock, clamped
        assert_eq!(steer(&cpp, 1900), -1.0); // reached before the stick's end
    }

    const MANUAL: u16 = 2000;
    const NORMAL: u16 = 1100;

    #[test]
    fn switch_low_is_normal_and_high_is_manual() {
        let r = rc(ch(NORMAL, 1000, 1000)).unwrap();
        assert_eq!(r.switch, RcSwitch::Computer);
        // The sticks still read: in normal mode the RC can take over.
        assert_eq!((r.throttle, r.steering), (1.0, 1.0));
        // Exactly 1500 is still normal, as in the C++ (`> 1500`).
        assert_eq!(rc(ch(1500, 1000, 1500)).unwrap().switch, RcSwitch::Computer);
        assert_eq!(rc(ch(1501, 1000, 1500)).unwrap().switch, RcSwitch::Direct);
    }

    #[test]
    fn three_positions_put_rc_via_ros_in_the_middle() {
        let m = RcMap::THREE_POSITION;
        let at = |v| m.rc_input(&ch(v, 1250, 1500)).unwrap().switch;
        assert_eq!(at(1100), RcSwitch::Computer);
        assert_eq!(at(1300), RcSwitch::Computer);
        assert_eq!(at(1301), RcSwitch::ViaRos);
        assert_eq!(at(1500), RcSwitch::ViaRos);
        assert_eq!(at(1700), RcSwitch::ViaRos);
        assert_eq!(at(1701), RcSwitch::Direct);
        assert_eq!(at(2000), RcSwitch::Direct);
        // A two-position switch reads the same with either map.
        for v in [1100, 2000] {
            assert_eq!(
                at(v),
                RcMap::TODAY.rc_input(&ch(v, 1250, 1500)).unwrap().switch
            );
        }
    }

    #[test]
    fn the_failsafe_on_the_mode_switch_is_no_usable_rc() {
        // The transmitter's failsafe puts CH6 at 1000: under 1050, the link is gone. The
        // switch's low position sends 1100 (End points 80%).
        for map in [RcMap::TODAY, RcMap::THREE_POSITION] {
            assert_eq!(map.rc_input(&ch(1000, 1500, 1500)), None);
            assert_eq!(map.rc_input(&ch(1049, 1500, 1500)), None);
            assert!(map.rc_input(&ch(1050, 1500, 1500)).is_some());
            assert_eq!(
                map.rc_input(&ch(1100, 1500, 1500)).unwrap().switch,
                RcSwitch::Computer
            );
        }
    }

    #[test]
    fn the_middle_twenty_percent_is_throttle_off() {
        for us in [1400, 1450, 1500, 1550, 1600] {
            let r = rc(ch(MANUAL, us, 1500)).unwrap();
            assert_eq!((r.throttle, r.middle), (0.0, true), "{us}");
        }
        // Just past either edge: out of the band, barely moving.
        let fwd = rc(ch(MANUAL, 1399, 1500)).unwrap();
        assert!(!fwd.middle && fwd.throttle > 0.0 && fwd.throttle < 0.01);
        let rev = rc(ch(MANUAL, 1601, 1500)).unwrap();
        assert!(!rev.middle && rev.throttle < 0.0 && rev.throttle > -0.01);
    }

    #[test]
    fn outside_the_band_the_throttle_runs_from_zero_to_full() {
        // Below the middle is forward, as the C++ read CH3.
        assert_eq!(rc(ch(MANUAL, 1000, 1500)).unwrap().throttle, 1.0);
        assert_eq!(rc(ch(MANUAL, 2000, 1500)).unwrap().throttle, -1.0);
        // Halfway between the band's edge (1400) and the end (1000): half throttle.
        assert!(close(rc(ch(MANUAL, 1200, 1500)).unwrap().throttle, 0.5));
        assert!(close(rc(ch(MANUAL, 1800, 1500)).unwrap().throttle, -0.5));
        // Past the end still stops at full.
        assert_eq!(rc(ch(MANUAL, 900, 1500)).unwrap().throttle, 1.0);
    }

    #[test]
    fn a_reversed_left_stick_swaps_forward_and_reverse() {
        let m = RcMap {
            throttle_reversed: true,
            ..RcMap::TODAY
        };
        assert_eq!(m.rc_input(&ch(MANUAL, 1000, 1500)).unwrap().throttle, -1.0);
        assert_eq!(m.rc_input(&ch(MANUAL, 2000, 1500)).unwrap().throttle, 1.0);
        assert!(m.rc_input(&ch(MANUAL, 1500, 1500)).unwrap().middle);
    }

    #[test]
    fn steering_1000_is_plus_one_and_2000_minus_one() {
        // The C++: current_steering = map(steering, 1000, 2000, 100, -100) / 100.
        assert_eq!(rc(ch(MANUAL, 1500, 1000)).unwrap().steering, 1.0);
        assert_eq!(rc(ch(MANUAL, 1500, 1500)).unwrap().steering, 0.0);
        assert_eq!(rc(ch(MANUAL, 1500, 2000)).unwrap().steering, -1.0);
    }

    #[test]
    fn past_the_end_stops_at_full_lock_and_does_not_wrap() {
        // In the C++, steering above 2000 made map() negative; cast to uint16_t and
        // clamped, that was full lock the other way.
        assert_eq!(rc(ch(MANUAL, 1500, 2150)).unwrap().steering, -1.0);
        assert_eq!(rc(ch(MANUAL, 1500, 900)).unwrap().steering, 1.0);
    }

    #[test]
    fn the_gear_switch_is_not_read() {
        // SwC (CH5) anywhere, even out of range, changes nothing.
        for gear in [0, 1000, 1500, 2000, 3000] {
            let mut c = ch(MANUAL, 1200, 1500);
            c.0[4] = gear;
            let r = rc(c).unwrap();
            assert!(close(r.throttle, 0.5) && !r.middle, "{gear}");
        }
    }

    #[test]
    fn a_bad_stick_is_the_stop() {
        for bad in [0, 100, 2200, 3000] {
            let mut right = ch(MANUAL, 1000, 1500);
            right.0[1] = bad;
            for c in [ch(MANUAL, bad, 1500), ch(MANUAL, 1000, bad), right] {
                let r = rc(c).unwrap();
                assert_eq!(r.switch, RcSwitch::Direct, "{bad}");
                assert_eq!(
                    (r.throttle, r.steering, r.middle),
                    (0.0, 0.0, true),
                    "{bad}"
                );
                assert_eq!(r.right, RightStick::Stop, "{bad}");
            }
        }
    }

    /// Channels with the right stick's up and down (CH2) at `right`, steering at `steering`.
    fn right(right: u16, steering: u16) -> RightStick {
        let mut c = ch(NORMAL, 1500, steering);
        c.0[1] = right;
        rc(c).unwrap().right
    }

    #[test]
    fn the_right_stick_down_past_88_percent_is_the_stop() {
        // Down is towards 2000; the FS-i6X reaches +97 % (1985).
        assert_eq!(right(1939, 1500), RightStick::Centre);
        assert_eq!(right(1940, 1500), RightStick::Stop);
        assert_eq!(right(1985, 1500), RightStick::Stop);
        // Steered hard at the same time: still the stop.
        assert_eq!(right(1985, 2000), RightStick::Stop);
        assert_eq!(right(1500, 1500), RightStick::Centre);
    }

    #[test]
    fn the_right_stick_up_past_88_percent_gives_only_with_the_steering_centred() {
        assert_eq!(right(1061, 1500), RightStick::Centre);
        assert_eq!(right(1060, 1500), RightStick::Give);
        assert_eq!(right(1000, 1500), RightStick::Give);
        // Steering within 15 % (75 µs) of its middle.
        assert_eq!(right(1000, 1425), RightStick::Give);
        assert_eq!(right(1000, 1575), RightStick::Give);
        assert_eq!(right(1000, 1424), RightStick::Centre);
        assert_eq!(right(1000, 1576), RightStick::Centre);
    }

    #[test]
    fn the_left_stick_past_88_percent_is_its_end() {
        let at = |us| rc(ch(MANUAL, us, 1500)).unwrap();
        assert!(!at(1061).full && at(1060).full && at(1000).full);
        assert!(!at(1939).full && at(1940).full && at(1985).full);
        assert!(!at(1500).full);
        // Both ends still drive the same way as just inside them.
        assert!(at(1060).throttle > 0.0 && at(1940).throttle < 0.0);
    }

    #[test]
    fn a_bad_mode_switch_is_no_usable_rc() {
        for bad in [0, 100, 2200, 3000] {
            assert_eq!(rc(ch(bad, 1000, 1500)), None, "{bad}");
        }
    }
}
