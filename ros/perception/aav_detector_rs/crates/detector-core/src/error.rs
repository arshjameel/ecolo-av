//! What can go wrong: a picture of the wrong size, a model of the wrong shape, the runtime.

use std::fmt;

/// An error from this crate.
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// The picture's bytes don't match its width, height and row stride.
    Picture {
        /// Bytes needed.
        needed: usize,
        /// Bytes given.
        given: usize,
    },
    /// A model's input or outputs aren't RF-DETR's.
    Shape(String),
    /// The execution provider asked for isn't in this build, or failed to start.
    Provider(String),
    /// ONNX Runtime failed.
    #[cfg(feature = "ort")]
    Runtime(ort::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Picture { needed, given } => {
                write!(f, "picture: needs {needed} bytes, has {given}")
            }
            Self::Shape(what) => write!(f, "not an RF-DETR model: {what}"),
            Self::Provider(what) => write!(f, "execution provider {what}"),
            #[cfg(feature = "ort")]
            Self::Runtime(e) => write!(f, "ONNX Runtime: {e}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            #[cfg(feature = "ort")]
            Self::Runtime(e) => Some(e),
            _ => None,
        }
    }
}

#[cfg(feature = "ort")]
impl From<onnx_session::Error> for Error {
    fn from(e: onnx_session::Error) -> Self {
        match e {
            onnx_session::Error::Runtime(e) => Self::Runtime(e),
            other => Self::Provider(other.to_string()),
        }
    }
}

#[cfg(feature = "ort")]
impl From<ort::Error> for Error {
    fn from(e: ort::Error) -> Self {
        Self::Runtime(e)
    }
}
