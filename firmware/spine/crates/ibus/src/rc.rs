//! From channels to `spine_core::RcInput`: the mode switch, throttle and steering.
//!
//! The channel numbers are the 2025-26 firmware's (`main.cpp` `loop()`, `rc_handler.cpp`
//! `handleRCMode()`). It counted channels from 0, as its iBUS library does, so
//! "channel 5" there is CH6 on the transmitter.
//!
//! | Index | On the transmitter | Use |
//! |---|---|---|
//! | 5 | CH6 | mode switch: normal, RC via ROS, manual (RC direct) |
//! | 2 | CH3 | the left stick, up and down: brake band, forward, reverse |
//! | 3 | CH4 | steering stick |
//! | 4 | CH5 | (SwC) not read: its wiring is loose (Arsh, 21 Nov) |
//!
//! **The left stick (Arsh, 21 Nov).** The FS-i6X's left stick does not spring back to
//! the middle, so where it rests is a choice the driver made:
//!
//! ```text
//!   1000 ◄───────────────────── 1500 ─────────────────────► 2000   (µs, CH3)
//!   forward, full    forward    brake band (±30 %)   reverse    reverse, full
//!   throttle 1.0 ◄── 0.0 at the band's edge   0.0 at the band's edge ──► -1.0
//! ```
//!
//! Inside the band [`RcInput::brake`] is true and the throttle is 0. Outside it the
//! throttle is rescaled, so it starts from 0 at the band's edge and reaches ±1 at the
//! stick's end: no jump in speed. Which end is forward follows the C++ (it counted the
//! throttle below 1500); [`RcMap::throttle_reversed`] swaps them. TODO(Q7): check on
//! the transmitter.
//!
//! **The failsafe (Arsh, 21 Nov).** The transmitter's failsafe sets CH6 to 900 µs, below
//! anything the switch sends. A CH6 under [`RcMap::lost_below`] (950) means the radio link
//! is gone, the same as a silent receiver: [`RcMap::rc_input`] gives `None`, and
//! `spine-core` makes a hard stop (`RC_SIGNAL_LOST`). The car day check is in
//! `docs/car-day.md`, "The RC transmitter".
//!
//! **The mode switch's three positions (TODO 48).** [`RcMap::TODAY`] reads CH6 as the C++
//! did: two positions, normal or RC direct. [`RcMap::THREE_POSITION`] adds RC via ROS in
//! the middle, for a three-position switch on CH6 (the transmitter's aux-channel menu):
//!
//! | CH6 | `TODAY` | `THREE_POSITION` |
//! |---|---|---|
//! | under 950 | the failsafe: RC lost | the failsafe: RC lost |
//! | 1000 (low) | normal (`RcSwitch::Computer`) | normal |
//! | 1500 (middle) | normal | **RC via ROS** (1301..1700) |
//! | 2000 (high) | manual (`RcSwitch::Direct`) | manual (above 1700) |
//!
//! A two-position switch on CH6 never reads the middle, so it works the same with both.

use crate::frame::Channels;
use spine_core::{RcInput, RcSwitch};

/// Which channel does what, and where the switches change over.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcMap {
    /// Channel indexes, from 0 (index 0 is CH1).
    pub mode_switch: usize,
    /// The left stick's up and down channel: brake band, forward, reverse.
    pub throttle: usize,
    /// The steering stick's channel.
    pub steering: usize,
    /// A channel is usable only strictly between these (the C++: 100 and 2200).
    pub valid_above: u16,
    /// The top of the usable range (µs).
    pub valid_below: u16,
    /// The mode switch below this is the transmitter's failsafe: the radio link is gone
    /// (µs). The transmitter sets CH6 to 900 when it loses the link.
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
    /// The brake band: the left stick this close to its middle, as a share of its
    /// travel each way, is a brake (0..1).
    pub brake_band: f32,
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
        valid_above: 100,
        valid_below: 2200,
        lost_below: 950,
        direct_above: 1500,
        via_ros_above: None,
        centre: 1500,
        half_travel: 500,
        brake_band: 0.30,
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
    ///   1, and whether the left stick is in the brake band. In normal and manual the
    ///   spine drives from them; via ROS the board sends them up.
    /// - A stick out of range: the brake band and straight wheels. The C++ did a safe
    ///   stop here (`jrkSafeStop()`).
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
        let (Some(throttle), Some(steering)) = (usable(self.throttle), usable(self.steering))
        else {
            return Some(RcInput {
                switch,
                throttle: 0.0,
                steering: 0.0,
                brake: true,
            });
        };
        let (throttle, brake) = self.left_stick(throttle);
        Some(RcInput {
            switch,
            throttle,
            steering: self.steering(steering),
            brake,
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

    /// The left stick: the throttle (-1..1, positive forward) and whether it is in the
    /// brake band. Outside the band the throttle runs from 0 at its edge to 1 at the end.
    fn left_stick(&self, value: u16) -> (f32, bool) {
        let push = self.below_centre(value).clamp(-1.0, 1.0);
        let push = if self.throttle_reversed { -push } else { push };
        let band = self.brake_band.clamp(0.0, 0.99);
        // No `f32::abs` in `no_std` on every toolchain: by hand.
        let size = if push < 0.0 { -push } else { push };
        if size <= band {
            return (0.0, true);
        }
        let throttle = ((size - band) / (1.0 - band)).clamp(0.0, 1.0);
        (if push < 0.0 { -throttle } else { throttle }, false)
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
    const NORMAL: u16 = 1000;

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
        assert_eq!(at(1000), RcSwitch::Computer);
        assert_eq!(at(1300), RcSwitch::Computer);
        assert_eq!(at(1301), RcSwitch::ViaRos);
        assert_eq!(at(1500), RcSwitch::ViaRos);
        assert_eq!(at(1700), RcSwitch::ViaRos);
        assert_eq!(at(1701), RcSwitch::Direct);
        assert_eq!(at(2000), RcSwitch::Direct);
        // A two-position switch reads the same with either map.
        for v in [1000, 2000] {
            assert_eq!(
                at(v),
                RcMap::TODAY.rc_input(&ch(v, 1250, 1500)).unwrap().switch
            );
        }
    }

    #[test]
    fn the_failsafe_on_the_mode_switch_is_no_usable_rc() {
        // The transmitter's failsafe puts CH6 at 900: under 950, the link is gone.
        for map in [RcMap::TODAY, RcMap::THREE_POSITION] {
            assert_eq!(map.rc_input(&ch(900, 1500, 1500)), None);
            assert_eq!(map.rc_input(&ch(949, 1500, 1500)), None);
            assert!(map.rc_input(&ch(950, 1500, 1500)).is_some());
            assert!(map.rc_input(&ch(1000, 1500, 1500)).is_some());
        }
    }

    #[test]
    fn the_middle_thirty_percent_is_the_brake_band() {
        for us in [1350, 1400, 1500, 1600, 1650] {
            let r = rc(ch(MANUAL, us, 1500)).unwrap();
            assert_eq!((r.throttle, r.brake), (0.0, true), "{us}");
        }
        // Just past either edge: out of the band, barely moving.
        let fwd = rc(ch(MANUAL, 1349, 1500)).unwrap();
        assert!(!fwd.brake && fwd.throttle > 0.0 && fwd.throttle < 0.01);
        let rev = rc(ch(MANUAL, 1651, 1500)).unwrap();
        assert!(!rev.brake && rev.throttle < 0.0 && rev.throttle > -0.01);
    }

    #[test]
    fn outside_the_band_the_throttle_runs_from_zero_to_full() {
        // Below the middle is forward, as the C++ read CH3.
        assert_eq!(rc(ch(MANUAL, 1000, 1500)).unwrap().throttle, 1.0);
        assert_eq!(rc(ch(MANUAL, 2000, 1500)).unwrap().throttle, -1.0);
        // Halfway between the band's edge (1350) and the end (1000): half throttle.
        assert!(close(rc(ch(MANUAL, 1175, 1500)).unwrap().throttle, 0.5));
        assert!(close(rc(ch(MANUAL, 1825, 1500)).unwrap().throttle, -0.5));
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
        assert!(m.rc_input(&ch(MANUAL, 1500, 1500)).unwrap().brake);
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
            let mut c = ch(MANUAL, 1175, 1500);
            c.0[4] = gear;
            let r = rc(c).unwrap();
            assert!(close(r.throttle, 0.5) && !r.brake, "{gear}");
        }
    }

    #[test]
    fn a_bad_stick_is_the_brake_band() {
        for bad in [0, 100, 2200, 3000] {
            for c in [ch(MANUAL, bad, 1500), ch(MANUAL, 1000, bad)] {
                let r = rc(c).unwrap();
                assert_eq!(r.switch, RcSwitch::Direct, "{bad}");
                assert_eq!((r.throttle, r.steering, r.brake), (0.0, 0.0, true), "{bad}");
            }
        }
    }

    #[test]
    fn a_bad_mode_switch_is_no_usable_rc() {
        for bad in [0, 100, 2200, 3000] {
            assert_eq!(rc(ch(bad, 1000, 1500)), None, "{bad}");
        }
    }
}
