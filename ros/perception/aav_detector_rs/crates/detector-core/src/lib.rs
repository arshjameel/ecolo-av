//! RF-DETR from its ONNX file, in plain Rust (no ROS): a picture in, boxes out.
//!
//! The steps are aav_detector's `detr.py`, ported (the Python's answers are the tests):
//!
//! | Step | Module | What |
//! |---|---|---|
//! | in | [`Picture`] | a camera picture, RGB or BGR, 8 bits, any row stride |
//! | resize, normalise | [`preprocess`] | bilinear to the model's square (384 px for Nano), as OpenCV's `INTER_LINEAR`; ImageNet's mean and spread; (1, 3, H, W) |
//! | the model | `Detector` (feature `ort`) | ONNX Runtime through the `ort` crate: CPU, Core ML, CUDA or TensorRT |
//! | decode | [`decode`] | sigmoid scores, the best (query, class) pairs over a threshold, boxes in the picture's pixels; no NMS (DETR needs none) |
//!
//! Without the `ort` feature the crate is only the maths, and builds anywhere.

mod coco;
mod decode;
mod error;
#[cfg(feature = "ort")]
mod model;
mod picture;
mod preprocess;

pub use coco::ClassId;
pub use decode::{decode, Corners, DecodeOptions, Detection};
pub use error::Error;
#[cfg(feature = "ort")]
pub use model::{cache_dir, Detector, Provider};
pub use picture::{Picture, PixelOrder};
pub use preprocess::{preprocess, InputSize, IMAGENET_MEAN, IMAGENET_STD};

/// This crate's `Result`.
pub type Result<T> = std::result::Result<T, Error>;
