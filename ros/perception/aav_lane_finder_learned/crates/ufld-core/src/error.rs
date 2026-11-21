//! What can go wrong: the model's JSON, the picture, the model's answers, ONNX Runtime.

use std::fmt;

/// An error from this crate.
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// The model's JSON is missing, unreadable or says something impossible.
    Meta(String),
    /// The picture: its bytes or its encoding.
    Picture(lane_core::Error),
    /// The model's inputs or outputs aren't the shape its JSON says.
    Shape(String),
    /// ONNX Runtime, or the session builder.
    #[cfg(feature = "ort")]
    Runtime(onnx_session::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Meta(what) => write!(f, "the model's JSON: {what}"),
            Self::Picture(e) => write!(f, "the picture: {e}"),
            Self::Shape(what) => write!(f, "the model's shape: {what}"),
            #[cfg(feature = "ort")]
            Self::Runtime(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Picture(e) => Some(e),
            #[cfg(feature = "ort")]
            Self::Runtime(e) => Some(e),
            _ => None,
        }
    }
}

impl From<lane_core::Error> for Error {
    fn from(e: lane_core::Error) -> Self {
        Self::Picture(e)
    }
}

#[cfg(feature = "ort")]
impl From<onnx_session::Error> for Error {
    fn from(e: onnx_session::Error) -> Self {
        Self::Runtime(e)
    }
}

#[cfg(feature = "ort")]
impl From<ort::Error> for Error {
    fn from(e: ort::Error) -> Self {
        Self::Runtime(e.into())
    }
}

/// This crate's `Result`.
pub type Result<T> = std::result::Result<T, Error>;
