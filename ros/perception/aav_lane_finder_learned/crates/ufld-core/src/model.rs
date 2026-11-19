//! The model on ONNX Runtime, through the detector's session builder (`onnx-session`).

use std::fmt;
use std::path::Path;

pub use onnx_session::Provider;
use onnx_session::{Coverage, Options};
use ort::session::Session;
use ort::value::TensorRef;

use crate::{Error, Meta, Result};

/// `models/lane-ufld.onnx` (or another), on one provider, with its JSON.
pub struct Model {
    session: Session,
    provider: Provider,
    coverage: Option<Coverage>,
    meta: Meta,
}

impl fmt::Debug for Model {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Model")
            .field("provider", &self.provider)
            .field("coverage", &self.coverage)
            .finish_non_exhaustive()
    }
}

impl Model {
    /// Load the model and the JSON beside it; check its input and outputs agree with it.
    pub fn open(path: impl AsRef<Path>, provider: Provider) -> Result<Self> {
        let path = path.as_ref();
        let meta = Meta::beside(path)?;
        let opened = onnx_session::open(path, provider, &Options::default())?;
        let session = opened.session;
        let input: Option<Vec<i64>> = session
            .inputs()
            .iter()
            .find(|i| i.name() == "image")
            .and_then(|i| {
                i.dtype()
                    .tensor_shape()
                    .map(|s| s.iter().copied().collect())
            });
        let want = vec![1, 3, meta.input_h as i64, meta.input_w as i64];
        if input.as_ref() != Some(&want) {
            return Err(Error::Shape(format!("input image {input:?}, not {want:?}")));
        }
        for name in ["loc", "exist"] {
            if !session.outputs().iter().any(|o| o.name() == name) {
                return Err(Error::Shape(format!("no output {name}")));
            }
        }
        Ok(Self {
            session,
            provider,
            coverage: opened.coverage,
            meta,
        })
    }

    pub fn meta(&self) -> &Meta {
        &self.meta
    }

    pub fn provider(&self) -> Provider {
        self.provider
    }

    /// The provider's share of the graph, if it said (Core ML does).
    pub fn coverage(&self) -> Option<Coverage> {
        self.coverage
    }

    /// The answers for one picture ([`crate::input`]'s numbers): (loc, exist), as
    /// [`crate::decode`] takes them.
    pub fn run(&mut self, input: &[f32]) -> Result<(Vec<f32>, Vec<f32>)> {
        let (h, w) = (self.meta.input_h, self.meta.input_w);
        if input.len() != 3 * h * w {
            return Err(Error::Shape(format!(
                "an input of {} numbers, not 3 x {h} x {w}",
                input.len()
            )));
        }
        let outputs = self.session.run(ort::inputs![
            "image" => TensorRef::from_array_view(([1, 3, h, w], input))?,
        ])?;
        let (_, loc) = outputs["loc"].try_extract_tensor::<f32>()?;
        let (_, exist) = outputs["exist"].try_extract_tensor::<f32>()?;
        Ok((loc.to_vec(), exist.to_vec()))
    }
}
