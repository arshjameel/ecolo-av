//! From channels to `spine_core::RcInput`: the mode switch, throttle and steering.
//!
//! The numbers are the 2025-26 firmware's (`main.cpp` `loop()`, `rc_handler.cpp`
//! `handleRCMode()`). It counted channels from 0, as its iBUS library does, so
//! "channel 5" there is CH6 on the transmitter.
//!
//! | Index | On the transmitter | Use |
//! |---|---|---|
//! | 5 | CH6 | mode switch: RC drives above 1500 (today: two positions) |
//! | 2 | CH3 | throttle stick |
//! | 3 | CH4 | steering stick |
//! | 4 | CH5 | gear switch: forward below 1200, reverse above 1800, brake between |
//!
//! TODO(Q7): check on the transmitter which switch is CH6 and which is CH5. The C++
//! prints "SwC: TOP=Brake, MID=Fwd, LOW=Rev" at start-up, but its code puts the brake
//! in the middle. The code's order is kept here.
//!
//! **The mode switch's three positions (TODO 48, proposed: the team and the hardware
//! lead decide).** [`RcMap::TODAY`] reads CH6 as the C++ did: two positions, computer
//! or RC direct. [`RcMap::THREE_POSITION`] adds RC via ROS in the middle, for a
//! three-position switch on CH6 (the transmitter's aux-channel menu):
//!
//! | CH6 | `TODAY` | `THREE_POSITION` |
//! |---|---|---|
//! | 1000 (low) | computer | computer |
//! | 1500 (middle) | computer | **RC via ROS** (1301..1700) |
//! | 2000 (high) | RC direct | RC direct (above 1700) |
//!
//! A two-position switch on CH6 never reads the middle, so it works the same with both.

use crate::frame::Channels;
use spine_core::{RcInput, RcSwitch};

/// Which channel does what, and where the switches change over.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcMap {
    /// Channel indexes, from 0 (index 0 is CH1).
    pub mode_switch: usize,
    pub throttle: usize,
    pub steering: usize,
    pub gear: usize,
    /// A channel is usable only strictly between these (the C++: 100 and 2200).
    pub valid_above: u16,
    pub valid_below: u16,
    /// RC direct while the mode switch reads above this.
    pub direct_above: u16,
    /// RC via ROS while it reads above this, up to `direct_above`; `None`: no such
    /// position (computer up to `direct_above`).
    pub via_ros_above: Option<u16>,
    /// The gear switch: forward below `forward_below`, reverse above
    /// `reverse_above`, brake in between.
    pub forward_below: u16,
    pub reverse_above: u16,
    /// A stick's middle, and how far it travels each way (µs).
    pub centre: u16,
    pub half_travel: u16,
    /// Throttle at or below this fraction counts as none (the C++: 5 %).
    pub throttle_deadband: f32,
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
        gear: 4,
        valid_above: 100,
        valid_below: 2200,
        direct_above: 1500,
        via_ros_above: None,
        forward_below: 1200,
        reverse_above: 1800,
        centre: 1500,
        half_travel: 500,
        throttle_deadband: 0.05,
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

    /// What the RC asks for, or None if the mode switch reads out of range.
    ///
    /// - Every position: the switch, and the sticks: throttle and steering, each -1 to
    ///   1, and the gear switch's brake. In RC direct the spine drives from them; via
    ///   ROS the board sends them up; on computer they do nothing.
    /// - A stick or the gear switch out of range: zero speed and straight wheels. The
    ///   C++ did a safe stop here (`jrkSafeStop()`).
    #[must_use]
    pub fn rc_input(&self, ch: &Channels) -> Option<RcInput> {
        let usable = |index: usize| {
            ch.get(index)
                .filter(|&v| v > self.valid_above && v < self.valid_below)
        };
        let switch = self.switch(usable(self.mode_switch)?);
        let (Some(throttle), Some(steering), Some(gear)) = (
            usable(self.throttle),
            usable(self.steering),
            usable(self.gear),
        ) else {
            return Some(RcInput {
                switch,
                throttle: 0.0,
                steering: 0.0,
                brake: false,
            });
        };
        let gear = self.gear(gear);
        Some(RcInput {
            switch,
            throttle: self.signed_throttle(throttle, gear),
            steering: self.steering(steering),
            brake: gear == Gear::Brake,
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

    /// Where the gear switch is.
    fn gear(&self, value: u16) -> Gear {
        if value < self.forward_below {
            Gear::Forward
        } else if value > self.reverse_above {
            Gear::Reverse
        } else {
            Gear::Brake
        }
    }

    /// How far a stick is below its middle: 1 at 1000, -1 at 2000.
    fn below_centre(&self, value: u16) -> f32 {
        (f32::from(self.centre) - f32::from(value)) / f32::from(self.half_travel)
    }

    /// The C++ counted the throttle stick below its middle as "go", with 1000 full
    /// (TODO(Q7): the transmitter may reverse CH3). The gear switch gives the sign.
    fn signed_throttle(&self, throttle: u16, gear: Gear) -> f32 {
        let push = self.below_centre(throttle).clamp(0.0, 1.0);
        let push = if push > self.throttle_deadband {
            push
        } else {
            0.0
        };
        match gear {
            Gear::Forward => push,
            Gear::Reverse => -push,
            // spine-core decides what the brake position does (RC_GEAR_BRAKE).
            Gear::Brake => 0.0,
        }
    }
}

/// The gear switch's three positions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Gear {
    Forward,
    Reverse,
    Brake,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::CHANNELS;

    /// Channels with the mode switch, throttle, steering and gear set; the rest at 1500.
    fn ch(mode_switch: u16, throttle: u16, steering: u16, gear: u16) -> Channels {
        let mut c = [1500; CHANNELS];
        c[5] = mode_switch;
        c[2] = throttle;
        c[3] = steering;
        c[4] = gear;
        Channels(c)
    }

    fn rc(c: Channels) -> Option<RcInput> {
        RcMap::default().rc_input(&c)
    }

    #[test]
    fn the_right_gain_steers_right_harder_and_still_clamps() {
        let cpp = RcMap {
            steer_right_gain: 1950.0 / 1550.0,
            ..RcMap::TODAY
        };
        let steer = |map: &RcMap, us| map.rc_input(&ch(2000, 1500, us, 1500)).unwrap().steering;
        assert_eq!(steer(&RcMap::TODAY, 1750), -0.5);
        assert!((steer(&cpp, 1750) + 0.5 * 1950.0 / 1550.0).abs() < 1e-6);
        assert_eq!(steer(&cpp, 1250), 0.5); // left as before
        assert_eq!(steer(&cpp, 2000), -1.0); // full right lock, clamped
        assert_eq!(steer(&cpp, 1900), -1.0); // reached before the stick's end
    }

    const ON: u16 = 2000;
    const OFF: u16 = 1000;
    const FWD: u16 = 1000;
    const BRAKE: u16 = 1500;
    const REV: u16 = 2000;

    #[test]
    fn switch_low_is_the_computer() {
        let r = rc(ch(OFF, 1000, 1000, FWD)).unwrap();
        assert_eq!(r.switch, RcSwitch::Computer);
        // The sticks still read: the spine ignores them in this position.
        assert_eq!((r.throttle, r.steering), (1.0, 1.0));
        // Exactly 1500 is still the computer, as in the C++ (`> 1500`).
        assert_eq!(
            rc(ch(1500, 1000, 1500, FWD)).unwrap().switch,
            RcSwitch::Computer
        );
        assert_eq!(
            rc(ch(1501, 1000, 1500, FWD)).unwrap().switch,
            RcSwitch::Direct
        );
    }

    #[test]
    fn three_positions_put_rc_via_ros_in_the_middle() {
        let m = RcMap::THREE_POSITION;
        let at = |v| m.rc_input(&ch(v, 1250, 1500, FWD)).unwrap().switch;
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
                RcMap::TODAY
                    .rc_input(&ch(v, 1250, 1500, FWD))
                    .unwrap()
                    .switch
            );
        }
        assert_eq!(
            m.rc_input(&ch(1500, 1250, 1500, FWD)).unwrap().throttle,
            0.5
        );
    }

    #[test]
    fn forward_and_reverse_take_the_throttle_below_centre() {
        let fwd = rc(ch(ON, 1000, 1500, FWD)).unwrap();
        assert_eq!(fwd.switch, RcSwitch::Direct);
        assert_eq!(fwd.throttle, 1.0);
        assert_eq!(rc(ch(ON, 1250, 1500, FWD)).unwrap().throttle, 0.5);
        assert_eq!(rc(ch(ON, 1250, 1500, REV)).unwrap().throttle, -0.5);
        // Above centre is no throttle.
        assert_eq!(rc(ch(ON, 1900, 1500, FWD)).unwrap().throttle, 0.0);
    }

    #[test]
    fn the_brake_position_asks_for_zero_speed_and_says_brake() {
        let r = rc(ch(ON, 1000, 1500, BRAKE)).unwrap();
        assert_eq!((r.throttle, r.brake), (0.0, true));
        // The edges count as brake too: forward needs < 1200, reverse > 1800.
        for edge in [1200, 1800] {
            let r = rc(ch(ON, 1000, 1500, edge)).unwrap();
            assert_eq!((r.throttle, r.brake), (0.0, true), "{edge}");
        }
        // Forward and reverse are not brake.
        assert!(!rc(ch(ON, 1000, 1500, FWD)).unwrap().brake);
        assert!(!rc(ch(ON, 1000, 1500, REV)).unwrap().brake);
    }

    #[test]
    fn the_throttle_deadband_is_the_cpps_5_percent() {
        // The C++ took `map(...) < -5`: 1474 and below is go, 1475 is not.
        assert_eq!(rc(ch(ON, 1475, 1500, FWD)).unwrap().throttle, 0.0);
        assert!(rc(ch(ON, 1474, 1500, FWD)).unwrap().throttle > 0.05);
    }

    #[test]
    fn steering_1000_is_plus_one_and_2000_minus_one() {
        // The C++: current_steering = map(steering, 1000, 2000, 100, -100) / 100.
        assert_eq!(rc(ch(ON, 2000, 1000, FWD)).unwrap().steering, 1.0);
        assert_eq!(rc(ch(ON, 2000, 1500, FWD)).unwrap().steering, 0.0);
        assert_eq!(rc(ch(ON, 2000, 2000, FWD)).unwrap().steering, -1.0);
    }

    #[test]
    fn past_the_end_stops_at_full_lock_and_does_not_wrap() {
        // In the C++, steering above 2000 made map() negative; cast to uint16_t and
        // clamped, that was full lock the other way.
        assert_eq!(rc(ch(ON, 2000, 2150, FWD)).unwrap().steering, -1.0);
        assert_eq!(rc(ch(ON, 2000, 900, FWD)).unwrap().steering, 1.0);
        assert_eq!(rc(ch(ON, 900, 1500, FWD)).unwrap().throttle, 1.0);
    }

    #[test]
    fn a_bad_stick_asks_for_a_stop() {
        for bad in [0, 100, 2200, 3000] {
            for c in [
                ch(ON, bad, 1500, FWD),
                ch(ON, 1000, bad, FWD),
                ch(ON, 1000, 1500, bad),
            ] {
                let r = rc(c).unwrap();
                assert_eq!(r.switch, RcSwitch::Direct, "{bad}");
                assert_eq!(
                    (r.throttle, r.steering, r.brake),
                    (0.0, 0.0, false),
                    "{bad}"
                );
            }
        }
    }

    #[test]
    fn a_bad_mode_switch_is_no_usable_rc() {
        for bad in [0, 100, 2200, 3000] {
            assert_eq!(rc(ch(bad, 1000, 1500, FWD)), None, "{bad}");
        }
    }
}
