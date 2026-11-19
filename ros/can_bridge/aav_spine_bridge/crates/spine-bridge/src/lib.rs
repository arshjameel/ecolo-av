//! spine_bridge's logic, with no ROS: what the Orin sends the spine over CAN, and what it
//! makes of the frames it hears.
//!
//! The frames are `firmware/can/aav.dbc`, packed by the firmware's own code
//! (`spine-core`'s `protocol`), so the bridge and the spine share one implementation.
//! The ROS node (`bridge-node`) only turns messages into calls here and [`Update`]s back
//! into messages. A port of aav_spine's `spine_bridge.py` and `wheel_speed.py`; its tests
//! carry their cases.
//!
//! - [`Bridge::drive`]: DRIVE_CMD at 50 Hz while ego's command is fresh, with the arming
//!   edge (armed=0 frames before armed=1);
//! - [`Bridge::arm`]: refused during an e-stop, and while the safety gate is shut
//!   ([`OutputGate`]: the output mux not OPEN for this bridge's [`Target`], or quiet);
//! - [`Bridge::set_estop`], [`Bridge::estop_frame`]: ESTOP_CMD, at once on a change and
//!   as a 10 Hz heartbeat;
//! - [`Bridge::receive`]: VEHICLE_FEEDBACK, VEHICLE_STATUS and the front wheel nodes'
//!   WHEEL_SPEED_L and WHEEL_SPEED_R, into [`Update`]s;
//! - [`Role`]: `Spine` does all of that. `Sensors` only listens to the wheel nodes and
//!   never sends, for a car driven another way (today's: the ESP32, through the Motor
//!   Pi's UART; docs/can-plan.md).
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

mod bridge;
mod gate;
mod ready;
pub mod wheels;

pub use bridge::{
    ArmError, Arming, BadFrame, Bridge, Command, Drive, Geometry, Motion, Role, Settings, Status,
    UnknownRole, Update,
};
pub use gate::{try_again, GateShut, OutputGate, Target, UnknownTarget, GATE_SHUT};
pub use ready::{NotReady, ReadyGate, NOT_READY};
pub use spine_core::protocol::{DriveCmd, EstopCmd, Mode, DRIVE_CMD_ID, ESTOP_CMD_ID};
pub use wheels::{Direction, Reading, Side};
