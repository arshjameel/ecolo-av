//! The model on ONNX Runtime, through the detector's session builder (`onnx-session`).

use std::fmt;
use std::path::Path;

pub use onnx_session::Provider;
use onnx_session::{Coverage, Options};
use ort::session::Session;
use ort::value::TensorRef;

use crate::{Error, Meta, Result};

/// The model's answers for one picture, as [`crate::decode`] and [`crate::paints`] take them.
#[derive(Debug, Clone, PartialEq)]
pub struct Answers {
    /// cells x rows x lanes logits: which column.
    pub loc: Vec<f32>,
    /// 2 x rows x lanes logits: is the line in that row.
    pub exist: Vec<f32>,
    /// 2 x lanes logits: white or yellow; None for a model exported without the colour head.
    pub colour: Option<Vec<f32>>,
}

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

    /// Whether the model has the colour head (`tools/lane_train.py` exports it as `colour`).
    pub fn has_colour(&self) -> bool {
        self.session.outputs().iter().any(|o| o.name() == "colour")
    }

    /// The answers for one picture ([`crate::input`]'s numbers).
    pub fn run(&mut self, input: &[f32]) -> Result<Answers> {
        let colour = self.has_colour();
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
        let colour = if colour {
            Some(outputs["colour"].try_extract_tensor::<f32>()?.1.to_vec())
        } else {
            None
        };
        Ok(Answers {
            loc: loc.to_vec(),
            exist: exist.to_vec(),
            colour,
        })
    }
}
