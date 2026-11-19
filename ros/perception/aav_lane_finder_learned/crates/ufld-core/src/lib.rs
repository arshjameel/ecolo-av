//! The learned lane finder, with no ROS (TODO 262b): a small model of our own in the style
//! of Ultra-Fast-Lane-Detection v2 (Qin et al., TPAMI 2022: row anchors), trained on
//! Ravensim's pictures labelled from the lane map (`tools/lane_data.py`,
//! `tools/lane_train.py`), and turned into the same lane the classic finder gives.
//!
//! For each picture:
//!
//! 1. [`input`]: the rows below the horizon, scaled as Pillow scales them in training;
//! 2. the model ([`Model`], feature `ort`): for 4 lines (the car's lane's left and right
//!    and their neighbours) and each row anchor, a column, and whether the line is there;
//! 3. [`decode`]: each line's points in the picture (UFLD v2's local expectation), and
//!    [`paints`]: each line's paint, white or yellow, from the model's colour head;
//! 4. [`ground`]: each point onto the flat road, in base_link, through the lens and mount
//!    (the same as the classic finder's bird's-eye view, run backwards);
//! 5. [`Learned::lane`]: lane-core's `fit_line` and `lane_from`: the same polynomials, offset,
//!    width and steering hint as `LaneGeometry` carries from the classic finder.
//!
//! What the model was made for (its picture size, crop, rows, cells) is in a JSON file
//! beside the `.onnx` ([`Meta`]).

mod decode;
mod error;
mod lane;
mod meta;
#[cfg(feature = "ort")]
mod model;
mod preprocess;

pub use decode::{decode, paints, Anchor};
pub use error::{Error, Result};
pub use lane::{ground, Learned};
pub use meta::Meta;
#[cfg(feature = "ort")]
pub use model::{Answers, Model, Provider};
pub use preprocess::{input, resample, Taps};
