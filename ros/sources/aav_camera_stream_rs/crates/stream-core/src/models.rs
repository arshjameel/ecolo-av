//! The detector choice (`models.py`). The models themselves are `aav_detector`'s, in
//! PyTorch, Core ML or TensorRT: only the Python node loads them. The Rust node runs
//! `none`: the stream, undistortion and preview, every picture's (empty) detections.

use std::fmt;
use std::str::FromStr;

use crate::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Detector {
    RfDetr,
    Yolo,
    None,
}

impl Detector {
    pub const ALL: [Detector; 3] = [Detector::RfDetr, Detector::Yolo, Detector::None];

    pub fn name(self) -> &'static str {
        match self {
            Detector::RfDetr => "rfdetr",
            Detector::Yolo => "yolo",
            Detector::None => "none",
        }
    }
}

impl fmt::Display for Detector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

impl FromStr for Detector {
    type Err = Error;

    /// Spaces and capitals are forgiven, as the Python's `strip().lower()`.
    fn from_str(s: &str) -> Result<Self, Error> {
        let kind = s.trim().to_lowercase();
        Detector::ALL
            .into_iter()
            .find(|d| d.name() == kind)
            .ok_or_else(|| {
                let names: Vec<&str> = Detector::ALL.iter().map(|d| d.name()).collect();
                Error::Invalid(format!("detector:={s}: use one of {}", names.join(", ")))
            })
    }
}
