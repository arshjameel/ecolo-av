//! The uplink (TODO 50, TODO 48): the ESP32's status, back to the Motor-Pi on TX1.
//!
//! Version 3 (1 Nov), what this firmware sends: 40 bytes.
//!
//! ```text
//!  byte  0     1        2     3         4     5     6       7      8..9
//!        0xA5  version  kind  sequence  echo  mode  switch  flags  faults
//!  byte  10..13         14..17         18..21        22..25     26..29       30..33
//!        steering (rad) brake (0..1)   throttle out  brake out  RC throttle  RC steering
//!  byte  34..35           36..37        38                39
//!        steering counts  brake counts  steering cal state CRC-8 of 1..38
//! ```
//!
//! Version 2 is the first 34 bytes, then its CRC-8 (of bytes 1..33) in byte 34: 35 bytes,
//! no counts and no calibration state. [`Uplink::decode`] reads both.
//!
//! The same sync byte and CRC-8 as the downlink ([`FrameV2`](crate::FrameV2)). Floats are
//! little-endian `f32`, counts little-endian `u16` (0 to 4095: the Jrk's scaled feedback,
//! in target units). A position the ESP32 can't measure (the Jrk has no analog feedback,
//! or its read failed) is NaN on the wire (or 0 counts), and its flag bit is clear.

use crate::frame::FrameError;
use crate::steer_cal::{CalState, COUNTS_MAX};
use crate::v2::{crc8, SYNC, VERSION};
use crate::v3::VERSION_3;

/// Bytes in a version 3 uplink frame: what this firmware sends.
pub const UPLINK_LEN: usize = 40;
/// Bytes in a version 2 uplink frame (before 1 Nov).
pub const UPLINK_V2_LEN: usize = 35;
/// Byte 2 of a status frame. The only kind so far; a new kind gets a new number.
pub const KIND_STATUS: u8 = 1;

/// Who drives the actuators (byte 5): the same numbers as `VehicleStatus.mode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveMode {
    /// Nobody: stopped, idle or latched (`MODE_MANUAL`).
    Stopped,
    /// The RC transmitter, straight from its sticks on the ESP32 (`MODE_RC`).
    RcDirect,
    /// The computer's frames: ROS drives, or a person through ROS (`MODE_COMPUTER`).
    Computer,
}

impl DriveMode {
    fn to_byte(self) -> u8 {
        match self {
            DriveMode::Stopped => 0,
            DriveMode::RcDirect => 1,
            DriveMode::Computer => 2,
        }
    }

    fn from_byte(byte: u8) -> Result<Self, FrameError> {
        match byte {
            0 => Ok(DriveMode::Stopped),
            1 => Ok(DriveMode::RcDirect),
            2 => Ok(DriveMode::Computer),
            _ => Err(FrameError::Field),
        }
    }
}

/// Where the transmitter's mode switch is (byte 6), as the ESP32 read it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeSwitch {
    /// No usable RC signal: the receiver is silent, or the switch reads out of range.
    NoSignal,
    /// The computer drives.
    Computer,
    /// A person drives with the RC, through ROS: the ESP32 sends the sticks up and
    /// takes the computer's frames.
    RcViaRos,
    /// A person drives with the RC, straight from the ESP32 (the fallback).
    RcDirect,
}

impl ModeSwitch {
    fn to_byte(self) -> u8 {
        match self {
            ModeSwitch::NoSignal => 0,
            ModeSwitch::Computer => 1,
            ModeSwitch::RcViaRos => 2,
            ModeSwitch::RcDirect => 3,
        }
    }

    fn from_byte(byte: u8) -> Result<Self, FrameError> {
        match byte {
            0 => Ok(ModeSwitch::NoSignal),
            1 => Ok(ModeSwitch::Computer),
            2 => Ok(ModeSwitch::RcViaRos),
            3 => Ok(ModeSwitch::RcDirect),
            _ => Err(FrameError::Field),
        }
    }
}

/// The RC sticks, as the ESP32 reads them (`ibus::RcMap`): what RC direct would drive.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Sticks {
    /// -1..1, positive forward: the throttle stick, signed by the gear switch.
    pub throttle: f32,
    /// -1..1, positive left.
    pub steering: f32,
    /// The gear switch is on brake.
    pub brake: bool,
}

/// One uplink status frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Uplink {
    /// +1 a frame, 255 then 0: a gap means frames were lost.
    pub sequence: u8,
    /// The last Orin counter the ESP32 took from a version 2 frame; none before the
    /// first, and never with version 1 frames.
    pub echo: Option<u8>,
    pub mode: DriveMode,
    pub switch: ModeSwitch,
    /// The spine accepts the computer's commands.
    pub armed: bool,
    /// Any stop: an e-stop, a timeout, a fault, latched.
    pub estop: bool,
    /// The spine's fault bits (`VehicleStatus.FAULT_*`).
    pub faults: u16,
    /// The steering Jrk's position (rad, positive left), if it has feedback.
    pub steering: Option<f32>,
    /// The brake Jrk's position (0 released .. 1 full), if it has feedback.
    pub brake_position: Option<f32>,
    /// What the ESP32 drives the throttle (0..1) and the brake (0..1) with.
    pub throttle_out: f32,
    pub brake_out: f32,
    /// The sticks; all 0 with [`ModeSwitch::NoSignal`].
    pub sticks: Sticks,
    /// The steering and brake Jrks' positions in counts (version 3; none in version 2).
    pub counts: Counts,
    /// What the steering calibration does (version 3; `Off` in version 2).
    pub calibration: CalState,
}

/// The Jrks' positions, raw: their scaled feedback, 0 to 4095 counts, in target units.
/// None when the Jrk has no analog feedback, or the read failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Counts {
    pub steering: Option<u16>,
    pub brake: Option<u16>,
}

/// Byte 7.
mod flag {
    pub const ARMED: u8 = 1 << 0;
    pub const ESTOP: u8 = 1 << 1;
    pub const ECHO: u8 = 1 << 2;
    pub const STEERING: u8 = 1 << 3;
    pub const BRAKE_POSITION: u8 = 1 << 4;
    pub const STICK_BRAKE: u8 = 1 << 5;
    /// Version 3 only: the counts are known.
    pub const STEERING_COUNTS: u8 = 1 << 6;
    pub const BRAKE_COUNTS: u8 = 1 << 7;
    pub const ALL_V2: u8 = ARMED | ESTOP | ECHO | STEERING | BRAKE_POSITION | STICK_BRAKE;
}

impl Uplink {
    /// The 40 bytes of version 3.
    pub fn encode(&self) -> [u8; UPLINK_LEN] {
        let bit = |on: bool, mask: u8| if on { mask } else { 0 };
        let mut b = [0; UPLINK_LEN];
        b[0] = SYNC;
        b[1] = VERSION_3;
        b[2] = KIND_STATUS;
        b[3] = self.sequence;
        b[4] = self.echo.unwrap_or(0);
        b[5] = self.mode.to_byte();
        b[6] = self.switch.to_byte();
        b[7] = bit(self.armed, flag::ARMED)
            | bit(self.estop, flag::ESTOP)
            | bit(self.echo.is_some(), flag::ECHO)
            | bit(self.steering.is_some(), flag::STEERING)
            | bit(self.brake_position.is_some(), flag::BRAKE_POSITION)
            | bit(self.sticks.brake, flag::STICK_BRAKE)
            | bit(self.counts.steering.is_some(), flag::STEERING_COUNTS)
            | bit(self.counts.brake.is_some(), flag::BRAKE_COUNTS);
        b[8..10].copy_from_slice(&self.faults.to_le_bytes());
        let values = [
            self.steering.unwrap_or(f32::NAN),
            self.brake_position.unwrap_or(f32::NAN),
            self.throttle_out,
            self.brake_out,
            self.sticks.throttle,
            self.sticks.steering,
        ];
        for (chunk, value) in b[10..34].chunks_exact_mut(4).zip(values) {
            chunk.copy_from_slice(&value.to_le_bytes());
        }
        b[34..36].copy_from_slice(&self.counts.steering.unwrap_or(0).to_le_bytes());
        b[36..38].copy_from_slice(&self.counts.brake.unwrap_or(0).to_le_bytes());
        b[38] = self.calibration.to_byte();
        b[39] = crc8(&b[1..39]);
        b
    }

    /// A frame from its bytes, version 2 (35) or 3 (40): the sync byte, the version (it
    /// says the length), the CRC, then the kind and each field.
    pub fn decode(b: &[u8]) -> Result<Uplink, FrameError> {
        if b.first() != Some(&SYNC) {
            return Err(FrameError::Sync);
        }
        let (len, flags_known) = match b.get(1) {
            Some(&VERSION) => (UPLINK_V2_LEN, flag::ALL_V2),
            Some(&VERSION_3) => (
                UPLINK_LEN,
                flag::ALL_V2 | flag::STEERING_COUNTS | flag::BRAKE_COUNTS,
            ),
            _ => return Err(FrameError::Version),
        };
        if b.len() != len {
            return Err(FrameError::Version);
        }
        if crc8(&b[1..len - 1]) != b[len - 1] {
            return Err(FrameError::Checksum);
        }
        if b[2] != KIND_STATUS {
            return Err(FrameError::Version);
        }
        let flags = b[7];
        if flags & !flags_known != 0 {
            return Err(FrameError::Flags);
        }
        let on = |mask: u8| flags & mask != 0;
        let value = |at: usize| {
            let mut four = [0; 4];
            four.copy_from_slice(&b[at..at + 4]);
            f32::from_le_bytes(four)
        };
        let (counts, calibration) = if len == UPLINK_LEN {
            let count = |at: usize, mask: u8| -> Result<Option<u16>, FrameError> {
                let c = u16::from_le_bytes([b[at], b[at + 1]]);
                match (on(mask), c) {
                    (true, c) if c <= COUNTS_MAX => Ok(Some(c)),
                    (false, 0) => Ok(None),
                    _ => Err(FrameError::Field),
                }
            };
            (
                Counts {
                    steering: count(34, flag::STEERING_COUNTS)?,
                    brake: count(36, flag::BRAKE_COUNTS)?,
                },
                CalState::from_byte(b[38]).ok_or(FrameError::Field)?,
            )
        } else {
            (Counts::default(), CalState::Off)
        };
        Ok(Uplink {
            sequence: b[3],
            echo: on(flag::ECHO).then_some(b[4]),
            mode: DriveMode::from_byte(b[5])?,
            switch: ModeSwitch::from_byte(b[6])?,
            armed: on(flag::ARMED),
            estop: on(flag::ESTOP),
            faults: u16::from_le_bytes([b[8], b[9]]),
            steering: on(flag::STEERING).then(|| value(10)),
            brake_position: on(flag::BRAKE_POSITION).then(|| value(14)),
            throttle_out: value(18),
            brake_out: value(22),
            sticks: Sticks {
                throttle: value(26),
                steering: value(30),
                brake: on(flag::STICK_BRAKE),
            },
            counts,
            calibration,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn status() -> Uplink {
        Uplink {
            sequence: 41,
            echo: Some(200),
            mode: DriveMode::Computer,
            switch: ModeSwitch::RcViaRos,
            armed: true,
            estop: false,
            faults: 0x0102,
            steering: Some(0.125),
            brake_position: None,
            throttle_out: 0.25,
            brake_out: 0.0,
            sticks: Sticks {
                throttle: 0.5,
                steering: -0.75,
                brake: false,
            },
            counts: Counts {
                steering: Some(2048),
                brake: None,
            },
            calibration: CalState::Limp,
        }
    }

    #[test]
    fn round_trip() {
        let u = status();
        let b = u.encode();
        assert_eq!((b[0], b[1], b[2], b[3], b[4]), (0xA5, 3, 1, 41, 200));
        assert_eq!((b[5], b[6], b[7]), (2, 2, 0b0100_1101));
        assert_eq!((b[34], b[35], b[36], b[37], b[38]), (0x00, 0x08, 0, 0, 1));
        assert_eq!((b[8], b[9]), (0x02, 0x01));
        assert!(f32::from_le_bytes([b[14], b[15], b[16], b[17]]).is_nan());
        assert_eq!(Uplink::decode(&b), Ok(u));
        let quiet = Uplink {
            echo: None,
            mode: DriveMode::Stopped,
            switch: ModeSwitch::NoSignal,
            steering: None,
            sticks: Sticks::default(),
            counts: Counts::default(),
            calibration: CalState::Off,
            ..u
        };
        assert_eq!(Uplink::decode(&quiet.encode()), Ok(quiet));
    }

    #[test]
    fn every_single_bit_error_is_caught() {
        let b = status().encode();
        for bit in 8..UPLINK_LEN * 8 {
            let mut bad = b;
            bad[bit / 8] ^= 0x80 >> (bit % 8);
            assert!(Uplink::decode(&bad).is_err(), "bit {bit}");
        }
    }

    #[test]
    fn unknown_kinds_modes_and_flags_are_turned_down() {
        let fix = |mut b: [u8; UPLINK_LEN]| {
            b[39] = crc8(&b[1..39]);
            b
        };
        let mut b = status().encode();
        b[2] = 2;
        assert_eq!(Uplink::decode(&fix(b)), Err(FrameError::Version));
        let mut b = status().encode();
        b[5] = 3;
        assert_eq!(Uplink::decode(&fix(b)), Err(FrameError::Field));
        let mut b = status().encode();
        b[6] = 4;
        assert_eq!(Uplink::decode(&fix(b)), Err(FrameError::Field));
        let mut b = status().encode();
        b[38] = 10;
        assert_eq!(Uplink::decode(&fix(b)), Err(FrameError::Field));
        // Counts past 4095, or counts with their flag clear.
        let mut b = status().encode();
        b[35] = 0x10;
        assert_eq!(Uplink::decode(&fix(b)), Err(FrameError::Field));
        let mut b = status().encode();
        b[36] = 1;
        assert_eq!(Uplink::decode(&fix(b)), Err(FrameError::Field));
        // A frame the wrong length for its version.
        assert_eq!(
            Uplink::decode(&status().encode()[..35]),
            Err(FrameError::Version)
        );
    }

    #[test]
    fn version_2_frames_still_read() {
        // A version 2 frame: the first 34 bytes, version 2, its CRC; bits 6 and 7 unknown.
        let mut b = [0u8; UPLINK_V2_LEN];
        b.copy_from_slice(&status().encode()[..35]);
        b[1] = 2;
        b[7] &= 0x3F;
        b[34] = crc8(&b[1..34]);
        let u = Uplink::decode(&b).unwrap();
        assert_eq!(
            (u.counts, u.calibration),
            (Counts::default(), CalState::Off)
        );
        assert_eq!(u.sequence, 41);
        b[7] |= 0x40;
        b[34] = crc8(&b[1..34]);
        assert_eq!(Uplink::decode(&b), Err(FrameError::Flags));
    }
}
