//! Hardware-independent logic for the AAV spine firmware.
//!
//! Plain `no_std` Rust with no allocator and no hardware access, so the same
//! code runs on the microcontroller and in `cargo test` on a laptop. The board
//! crate owns the drivers (CAN controller, I2C to the Jrk G2s and the DAC,
//! iBUS receiver, encoder inputs, e-stop input). Each tick it reads them into
//! [`Inputs`], calls [`Spine::step`], and writes [`Outputs`] back to them.
#![no_std]

pub mod encoder;
pub mod fsm;
pub mod limits;
pub mod protocol;
pub mod speed_loop;
pub mod spine;
pub mod watchdog;

pub use fsm::{
    Authority, Confidence, Lifecycle, Motion, Overspeed, Safety, State, TestRequest, TestStage,
    OVERSPEED_WINDOW_MS, STANDSTILL_SPEED, TEST_MAX_SPEED, TEST_MAX_THROTTLE,
};
pub use spine::{
    faults, DirectCmd, EstopSteering, Inputs, Outputs, RcGearBrake, RcInput, RcSwitch,
    SpeedFeedback, SpeedReport, Spine, SpineConfig, ESTOP_STEERING, IDLE_BRAKE, MAX_REVERSE_SPEED,
    MAX_SPEED, MAX_STEERING_ANGLE, RC_GEAR_BRAKE,
};
