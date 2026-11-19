//! What can go wrong: a provider this build lacks, ONNX Runtime itself, a profile.

use std::fmt;

/// An error from this crate.
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// The execution provider asked for isn't in this build, or its name is unknown.
    Provider(String),
    /// ONNX Runtime failed.
    Runtime(ort::Error),
    /// ONNX Runtime's profile isn't the JSON trace expected.
    Profile(serde_json::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Provider(what) => write!(f, "execution provider {what}"),
            Self::Runtime(e) => write!(f, "ONNX Runtime: {e}"),
            Self::Profile(e) => write!(f, "ONNX Runtime's profile: {e}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Runtime(e) => Some(e),
            Self::Profile(e) => Some(e),
            Self::Provider(_) => None,
        }
    }
}

impl From<ort::Error> for Error {
    fn from(e: ort::Error) -> Self {
        Self::Runtime(e)
    }
}

impl From<serde_json::Error> for Error {
    fn from(e: serde_json::Error) -> Self {
        Self::Profile(e)
    }
}
