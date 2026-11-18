//! One camera's H.264 stream to pictures, with no ROS: `aav_camera_stream`'s logic modules
//! in Rust, for the `camera_stream` node in `stream-node`.
//!
//! ```text
//!  decoders  the decoder's command line (ffmpeg, gst-nvidia, gst-soft, gst-mac), and
//!            which one this computer runs (auto)
//!  pipe      the decoder as a child program, raw BGR pictures on its stdout
//!  reader    a thread that keeps the decoder running, restarting it with a growing pause
//!  latest    the newest frame only; the rates and delays the node reports; the stamp
//!  lens      calibration files, the lens scaled, OpenCV's undistortion
//!  timecode  the time code drawn into test pictures, to measure the delay
//!  preview   the small JPEG
//!  models    the detector choice: only `none` here (the models are Python's)
//! ```

pub mod decoders;
pub mod latest;
pub mod lens;
pub mod models;
pub mod pipe;
pub mod preview;
pub mod reader;
pub mod timecode;

/// What can go wrong; each message is the Python's.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// A setting this crate doesn't know, or a picture it can't take.
    Invalid(String),
    /// A calibration file that can't be read or used.
    Calibration(String),
    /// Starting or reading the decoder program.
    Io(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Invalid(m) | Error::Calibration(m) | Error::Io(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for Error {}

pub type Result<T> = std::result::Result<T, Error>;

/// Python's `round()`: halves to even.
pub(crate) fn round_even(x: f64) -> f64 {
    x.round_ties_even()
}
