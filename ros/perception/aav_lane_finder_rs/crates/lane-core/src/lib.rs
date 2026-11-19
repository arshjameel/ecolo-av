//! The lane lines in a camera picture, with no ROS: aav_lane_finder's Python modules
//! (`birds_eye.py`, `lane_lines.py`) in Rust.
//!
//! For each picture:
//!
//! 1. [`BirdsEye::warp`]: the road ahead seen from above (a flat road), through one
//!    look-up table made once per lens, the same fixed-point bilinear sums as OpenCV's
//!    `remap`;
//! 2. [`paint_mask`]: paint is brighter than the road on both sides of it, and narrow;
//! 3. [`follow`]: sliding windows climb each line from where it starts near the car;
//! 4. [`fit_line`]: y = a x² + b x + c in base_link (m), and a confidence;
//! 5. [`lane_from`]: the car's offset from the lane's centre, the width, the radii.
//!
//! [`LaneFinder`] does all five, and remembers the lane's width. [`draw`] makes the debug
//! picture. Signs follow REP-103: x forward, y left; the offset is + when the car is left
//! of the lane's centre.

mod birds_eye;
mod draw;
mod error;
mod font;
mod lines;
mod picture;

pub use birds_eye::{project, rotation, BirdsEye, Distortion, Grid, Lens, Mount, NEAREST};
pub use draw::{draw, Drawing};
pub use error::{Error, Result};
pub use lines::{
    centre_y, fit_line, follow, lane_from, paint_mask, paint_of, radius, starts, Lane, LaneFinder,
    Line, Mask, Paint, Settings, View, Window, MAX_RADIUS, STRAIGHT,
};
pub use picture::{Bev, Encoding, Picture};
