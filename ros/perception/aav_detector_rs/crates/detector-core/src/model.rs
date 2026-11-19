//! The ONNX model through ONNX Runtime (the `ort` crate, by `onnx-session`), on the
//! provider asked for.

use std::fmt;
use std::path::Path;

use onnx_session::{Coverage, Options};
use ort::session::Session;
use ort::value::TensorRef;

pub use onnx_session::{cache_dir, Provider};

use crate::{decode, preprocess, DecodeOptions, Detection, Error, InputSize, Picture, Result};

/// RF-DETR, exported to ONNX, ready to run: [`Detector::detect`] for each picture.
pub struct Detector {
    session: Session,
    input: String,
    boxes: String,
    logits: String,
    size: InputSize,
    provider: Provider,
    coverage: Option<Coverage>,
    notes: Vec<String>,
}

impl fmt::Debug for Detector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Detector")
            .field("input", &self.input)
            .field("size", &self.size)
            .field("provider", &self.provider)
            .field("coverage", &self.coverage)
            .finish_non_exhaustive()
    }
}

impl Detector {
    /// Load the ONNX model at `path` onto `provider`.
    pub fn open(path: impl AsRef<Path>, provider: Provider) -> Result<Self> {
        Self::open_with(path, provider, None)
    }

    /// As [`Detector::open`], writing ONNX Runtime's profile (a JSON trace: each node, its
    /// provider and time) to `profile`'s prefix; [`Detector::end_profiling`] finishes it.
    pub fn open_with(
        path: impl AsRef<Path>,
        provider: Provider,
        profile: Option<&Path>,
    ) -> Result<Self> {
        let options = Options {
            profile: profile.map(Path::to_path_buf),
            ..Options::default()
        };
        let opened = onnx_session::open(path, provider, &options)?;
        let session = opened.session;
        let input = session
            .inputs()
            .first()
            .ok_or_else(|| Error::Shape("no input".into()))?;
        let dims: Vec<i64> = input
            .dtype()
            .tensor_shape()
            .map(|s| s.iter().copied().collect())
            .unwrap_or_default();
        let size = match dims[..] {
            [1, 3, h, w] if h > 0 && w > 0 => InputSize {
                height: h as usize,
                width: w as usize,
            },
            _ => {
                return Err(Error::Shape(format!(
                    "expected an input of (1, 3, H, W), fixed; got {dims:?}"
                )))
            }
        };
        let names: Vec<String> = session
            .outputs()
            .iter()
            .map(|o| o.name().to_owned())
            .collect();
        if !(names.iter().any(|n| n == "dets") && names.iter().any(|n| n == "labels")) {
            return Err(Error::Shape(format!(
                "expected outputs dets and labels; got {names:?}"
            )));
        }
        Ok(Self {
            input: input.name().to_owned(),
            session,
            boxes: "dets".into(),
            logits: "labels".into(),
            size,
            provider,
            coverage: opened.coverage,
            notes: opened.notes,
        })
    }

    /// The model's input size (384 x 384 for RF-DETR Nano).
    pub fn input_size(&self) -> InputSize {
        self.size
    }

    /// The provider asked for. The CPU still runs any node it couldn't take:
    /// [`Detector::coverage`] says how many.
    pub fn provider(&self) -> Provider {
        self.provider
    }

    /// The provider's share of the graph, as ONNX Runtime logged it (Core ML: 802 of 808).
    pub fn coverage(&self) -> Option<Coverage> {
        self.coverage
    }

    /// ONNX Runtime's lines on which provider took how much of the graph.
    pub fn notes(&self) -> &[String] {
        &self.notes
    }

    /// Find things in `picture`: those scoring over `threshold`, best first.
    pub fn detect(&mut self, picture: &Picture<'_>, threshold: f32) -> Result<Vec<Detection>> {
        let input = preprocess(picture, self.size);
        let shape = [1, 3, self.size.height, self.size.width];
        let tensor = TensorRef::from_array_view((shape, input.as_slice()))?;
        let outputs = self
            .session
            .run(ort::inputs![self.input.as_str() => tensor])?;
        let (_, boxes) = outputs[self.boxes.as_str()].try_extract_tensor::<f32>()?;
        let (shape, logits) = outputs[self.logits.as_str()].try_extract_tensor::<f32>()?;
        let classes = shape.last().copied().unwrap_or(0).max(0) as usize;
        let options = DecodeOptions {
            threshold,
            ..DecodeOptions::default()
        };
        decode(
            boxes,
            logits,
            classes,
            picture.width(),
            picture.height(),
            options,
        )
    }

    /// Finish the profile started by [`Detector::open_with`]; return its file's name.
    pub fn end_profiling(&mut self) -> Result<String> {
        Ok(self.session.end_profiling()?)
    }
}
