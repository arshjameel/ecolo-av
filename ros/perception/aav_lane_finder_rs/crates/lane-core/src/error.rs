//! What can go wrong with a picture or a lens.

use std::fmt;

/// Why a picture or a lens can't be used. The node logs it and skips the picture.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// The picture's size is not the lens's.
    WrongSize {
        picture: (usize, usize),
        lens: (usize, usize),
    },
    /// Fewer bytes than the picture's rows need.
    TooFewBytes { needed: usize, given: usize },
    /// A pixel format the finder doesn't read.
    Encoding(String),
    /// A distortion model OpenCV's `projectPoints` doesn't know.
    DistortionModel(String),
    /// Not 4, 5, 8 or 12 distortion coefficients.
    DistortionLength(usize),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongSize { picture, lens } => write!(
                f,
                "the picture is {} x {}, the lens {} x {}",
                picture.0, picture.1, lens.0, lens.1
            ),
            Self::TooFewBytes { needed, given } => {
                write!(f, "the picture needs {needed} bytes, it has {given}")
            }
            Self::Encoding(e) => write!(
                f,
                "pictures in {e} can't be read (rgb8, bgr8, rgba8, bgra8 or mono8)"
            ),
            Self::DistortionModel(m) => write!(f, "the {m} distortion is not known"),
            Self::DistortionLength(n) => {
                write!(f, "{n} distortion coefficients: 4, 5, 8 or 12 are known")
            }
        }
    }
}

impl std::error::Error for Error {}

/// The crate's results.
pub type Result<T> = std::result::Result<T, Error>;
