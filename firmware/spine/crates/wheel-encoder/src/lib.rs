//! The wheel encoder: two spoke sensors, counted, and the packet that reports them.
//!
//! Two boards count spokes the same way: the spine (through `spine-board`), and the
//! stand-alone wheel encoder Pico 2 (`boards/wheel-encoder-pico2`), which reports over
//! Ethernet to the Motor Pi, as the 2025-26 one did.
//!
//! ```text
//!  sensor A, sensor B --> SpokeSensors::update --> WheelCount (a running counter)
//!  (read every 1 ms)                                  |
//!                                Reporter::report  <--+  every 100 ms
//!                                      |
//!                                 SpokeDelta --> 4 bytes, big-endian --> UDP 10111
//! ```
//!
//! | Module | Does |
//! |---|---|
//! | [`spokes`] | [`SpokeSensors`]: one count a spoke, with its direction; [`WheelCount`] |
//! | [`packet`] | [`SpokeDelta`]: the UDP packet, byte for byte the 2025-26 one; [`Reporter`] |
//!
//! Plain `no_std` Rust with no hardware access: the board reads the pins and the clock.
#![no_std]
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod packet;
pub mod spokes;

pub use packet::{Direction, PacketError, Reporter, SpokeDelta};
pub use spokes::{SpokeSensors, WheelCount};
