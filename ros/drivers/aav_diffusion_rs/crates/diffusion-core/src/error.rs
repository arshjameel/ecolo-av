//! What can go wrong: a model of the wrong shape, the runtime, the pipe.

use std::fmt;

/// An error from this crate.
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// The model's inputs or outputs aren't the sampler's (crop, cond, noise -> paths).
    Shape(String),
    /// Opening the model: the provider, ONNX Runtime.
    Session(onnx_session::Error),
    /// ONNX Runtime failed to run it.
    Runtime(ort::Error),
    /// The pipe: closed, or a message of the wrong size.
    Pipe(std::io::Error),
    /// The settings line isn't the JSON expected.
    Settings(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Shape(what) => write!(f, "not the diffusion sampler: {what}"),
            Self::Session(e) => write!(f, "{e}"),
            Self::Runtime(e) => write!(f, "ONNX Runtime: {e}"),
            Self::Pipe(e) => write!(f, "the pipe: {e}"),
            Self::Settings(what) => write!(f, "the settings line: {what}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Session(e) => Some(e),
            Self::Runtime(e) => Some(e),
            Self::Pipe(e) => Some(e),
            Self::Shape(_) | Self::Settings(_) => None,
        }
    }
}

impl From<onnx_session::Error> for Error {
    fn from(e: onnx_session::Error) -> Self {
        Self::Session(e)
    }
}

impl From<ort::Error> for Error {
    fn from(e: ort::Error) -> Self {
        Self::Runtime(e)
    }
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Self::Pipe(e)
    }
}
