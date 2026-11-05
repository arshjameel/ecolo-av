//! The UART link from the Motor-Pi to the ESP32 control board: the 2025-26 frame
//! (version 1), and version 2 with the Orin's heartbeat (TODO 50).
//!
//! The ROS side is `ros/can_bridge/aav_legacy/aav_legacy/legacy_protocol.py`. The firmware
//! side was `firmware/legacy/aav_umod_esp32/src/modules/ros2_handler/ros2_handler.cpp`;
//! this crate does what it did, so the Rust board is a drop-in replacement.
//!
//! | Part | Here | The C++ |
//! |---|---|---|
//! | 13 bytes: three little-endian `f32`, then a sum checksum | [`Frame`] | `memcpy`, `calc += buf[j]` |
//! | resync: on a bad checksum, drop one byte and try again | [`Parser`] | `memmove(buf, buf + 1, 12)` |
//! | each value within -1..1, or the frame is turned down | [`Frame::check`] | `fabsf(...) <= 1.0f` |
//! | the Jrk and DAC targets for a frame | [`Frame::targets`] | the rest of `handleROS2Messages()` |
//!
//! ```text
//!  byte  0..3      4..7     8..11    12
//!       steering  speed    accel    sum(bytes 0..11) & 0xFF
//! ```
//!
//! Version 2 ([`FrameV2`], [`ParserV2`]) keeps the three values and adds a sync byte, a
//! version, the Orin's counter, the armed and e-stop flags, and a CRC-8:
//!
//! ```text
//!  byte  0     1   2..5      6..9   10..13  14       15     16
//!        0xA5  2   steering  speed  accel   counter  flags  CRC-8/SAE-J1850 of 1..15
//! ```
//!
//! The uplink ([`Uplink`], TODO 50 and 48) goes the other way, ESP32 to Motor-Pi on TX1
//! (GPIO17), 50 times a second: the mode, the transmitter's mode switch, the faults, the
//! last Orin counter taken (its echo), the Jrks' positions and the RC sticks. 35 bytes,
//! the same sync byte, version and CRC-8:
//!
//! ```text
//!  byte  0     1  2     3    4     5     6       7      8..9    10..33         34
//!        0xA5  2  kind  seq  echo  mode  switch  flags  faults  six f32 values CRC-8 of 1..33
//! ```
//!
//! Version 3 ([`FrameV3`], [`ParserV3`], 1 Nov) adds the steering actuator's calibration
//! to version 2: limp (its motor off, turned by hand) or a jog target in counts; its
//! rules are [`SteerCal`]'s. The version 3 uplink adds both Jrks' positions in counts and
//! the calibration's state. [`ParserV3`] and [`Uplink::decode`] read version 2 too.
//!
//! ```text
//!  byte  0     1   2..13            14       15     16        17..18      19
//!        0xA5  3   the three f32    counter  flags  cal mode  jog target  CRC-8 of 1..18
//! ```
//!
//! Version 1 goes one way only: the 2025-26 C++ sent nothing back.
#![no_std]

mod frame;
mod parser;
mod steer_cal;
mod targets;
mod uplink;
mod v2;
mod v3;

pub use frame::{Frame, FrameError, BAUD, FRAME_LEN};
pub use parser::{Parser, ParserV2, ParserV3};
pub use steer_cal::{
    Ask, CalRequest, CalState, Refusal, Situation, SteerCal, SteeringDrive, CAL_RATE,
    CAL_TIMEOUT_MS, COUNTS_MAX,
};
pub use targets::{
    Targets, Throttle, BRAKE_BELOW, BRAKE_FULL, BRAKE_RELEASED, DAC_CAP, SPEED_DEADBAND,
    STEER_CENTRE,
};
pub use uplink::{
    Counts, DriveMode, ModeSwitch, Sticks, Uplink, KIND_STATUS, UPLINK_LEN, UPLINK_V2_LEN,
};
pub use v2::{crc8, Flags, FrameV2, Heartbeat, FRAME_V2_LEN, SYNC, VERSION};
pub use v3::{FrameV3, FRAME_V3_LEN, VERSION_3};
