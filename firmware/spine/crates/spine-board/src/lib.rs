//! The glue between `spine-core` and the car's devices, for any board.
//!
//! `spine-core` decides; this crate connects it to the hardware without knowing which
//! chip runs it. A board crate (ESP32, Pico 2 or STM32, once the team picks: Q10 to
//! Q13) implements [`Board`], builds a [`SpineBoard`], and calls
//! [`SpineBoard::tick`] about 100 times a second.
//!
//! ```text
//!  Board: clock, e-stop pin,     RC receiver's        I2C bus: steering Jrk 0x05,
//!  wheel counter, CAN            serial port          brake Jrk 0x04, DAC 0x0C,
//!  (this crate's trait)          (embedded-io)        expander 0x20 (embedded-hal)
//!        |                            |                         |
//!        +----------------------------+-------------------------+
//!                                     |
//!                           SpineBoard::tick()
//!       read  -->  Inputs  -->  Spine::step  -->  Outputs  -->  write
//! ```
//!
//! | Module | Does |
//! |---|---|
//! | [`board`] | the [`Board`] trait: what differs from chip to chip |
//! | [`calibration`] | today's numbers: addresses, steering, brake, throttle, direction |
//! | [`link`] | where the commands come from: CAN, or the Motor-Pi's UART link, version 2 (the Orin's heartbeat) or 1 ([`CommandLink`]) |
//! | [`actuators`] | drives the devices from `Outputs` in a safe order, and reads them |
//! | [`wheel`] | the speed, from the wheel encoder's counter; two spoke sensors, counted |
//! | [`spine_board`] | the loop: [`SpineBoard`] |
//! | [`stand_in`] | stand-in devices on a bus, for a board with none wired (QEMU) |
//! | [`timing`] | the longest gap between ticks, and how many came late |
//! | [`scan`] | which I2C addresses answer, a few a tick, only while not armed |
//! | [`probe`] | which of the four devices answer at boot, and each Jrk's feedback mode: read only |
//! | [`health`] | the Pi link's verdict: OK, no frames, stale, the counter frozen, bad frames |
//! | [`cpp_log`] | the 2025-26 C++'s log lines in its words: boot, `STATUS:`, faults, Jrk errors, scan labels |
//!
//! A board crate, in outline:
//!
//! ```text
//! let bus = RefCell::new(i2c);  // one bus for four devices (embedded-hal-bus)
//! let cal = Calibration::TODAY;
//! let devices = Devices::new(RefCellDevice::new(&bus), RefCellDevice::new(&bus),
//!                            RefCellDevice::new(&bus), RefCellDevice::new(&bus), &cal);
//! let mut spine = SpineBoard::new(my_board, devices, ibus_uart, cal, SpineConfig::default());
//! spine.init();
//! loop {
//!     spine.tick();
//!     // wait for the next 10 ms
//! }
//! ```
//!
//! Plain `no_std` Rust, tested on a laptop with mocks.
#![no_std]
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod actuators;
pub mod board;
pub mod calibration;
pub mod cpp_log;
pub mod health;
pub mod link;
pub mod probe;
#[cfg(test)]
mod rig;
pub mod scan;
pub mod spine_board;
pub mod stand_in;
pub mod timing;
pub mod wheel;

pub use actuators::{Actuators, Devices, Positions, Readings};
pub use board::{Board, CanFrame, WheelCount};
pub use calibration::Calibration;
pub use health::{LinkHealth, LinkReport, LinkVerdict, BAD_WINDOW_MS};
/// A Jrk's error flags, as `Actuators::jrk_errors` gives them.
pub use jrk_g2::ErrorFlags;
pub use link::{
    CanLink, CommandLink, FrameVersion, LastFrame, LegacyLink, LinkStats, NoUplink, Report,
    UPLINK_PERIOD_MS,
};
pub use probe::{feedback_name, Device, DeviceCheck, Found};
pub use scan::{BusScan, ScanState};
pub use spine_board::SpineBoard;
pub use stand_in::{DeviceTargets, StandInBus};
pub use timing::TickTiming;
pub use wheel::{SpokeSensors, WheelSpeed};
