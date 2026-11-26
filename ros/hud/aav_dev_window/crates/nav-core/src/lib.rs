//! The top-down navigation view, without the window: what to draw from which message,
//! the car's trail, and the panel's scale. No Bevy, no egui, no ROS.
//!
//! The developer window's map panel and `pixi run nav-view` (the same panel, alone in
//! its window) draw from these.

pub mod body;
pub mod marker;
pub mod scale;
pub mod trail;
pub mod zone;
