//! The sampler from its ONNX file: crop, cond and noise in; paths out.

use std::fmt;
use std::path::Path;

use onnx_session::{Coverage, Options, Provider};
use ort::session::Session;
use ort::value::TensorRef;

use crate::{Error, Result};

/// Cells a side of the costmap crop (`window.SIZE`).
pub const SIZE: usize = 128;
/// Points a path (`window.POINTS`).
pub const POINTS: usize = 64;
/// Numbers a point: x, y, cos and sin of the heading, the gear (`window.CHANNELS`).
pub const CHANNELS: usize = 5;
/// The cond: start x, y; goal x, y, cos, sin; reversing; cell size (`window.COND`).
pub const COND: usize = 8;
/// Numbers in one sample's path.
const PATH: usize = POINTS * CHANNELS;

/// The diffusion planner's sampler (`training/model.py`'s `Sampler`, exported): ready to
/// draw paths with [`Model::sample`].
pub struct Model {
    session: Session,
    provider: Provider,
    /// The samples the model takes at once; `None`: any number (exports before 19 Nov).
    batch: Option<usize>,
    coverage: Option<Coverage>,
}

impl fmt::Debug for Model {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Model")
            .field("provider", &self.provider)
            .field("batch", &self.batch)
            .field("coverage", &self.coverage)
            .finish_non_exhaustive()
    }
}

impl Model {
    /// Load the ONNX sampler at `path` onto `provider`; an error if its inputs and output
    /// aren't the sampler's.
    pub fn open(path: impl AsRef<Path>, provider: Provider) -> Result<Self> {
        let opened = onnx_session::open(path, provider, &Options::default())?;
        let session = opened.session;
        let dims = |name: &str| -> Result<Vec<i64>> {
            session
                .inputs()
                .iter()
                .find(|i| i.name() == name)
                .and_then(|i| {
                    i.dtype()
                        .tensor_shape()
                        .map(|s| s.iter().copied().collect())
                })
                .ok_or_else(|| Error::Shape(format!("no input {name}")))
        };
        let (s, c, p) = (SIZE as i64, COND as i64, POINTS as i64);
        let crop = dims("crop")?;
        if crop != [1, 1, s, s] {
            return Err(Error::Shape(format!("crop {crop:?}, not [1, 1, {s}, {s}]")));
        }
        let cond = dims("cond")?;
        if cond != [1, c] {
            return Err(Error::Shape(format!("cond {cond:?}, not [1, {c}]")));
        }
        let batch = match dims("noise")?[..] {
            [k, pts, ch] if pts == p && ch == CHANNELS as i64 => usize::try_from(k).ok(),
            ref other => {
                return Err(Error::Shape(format!(
                    "noise {other:?}, not [K, {p}, {CHANNELS}]"
                )))
            }
        };
        if !session.outputs().iter().any(|o| o.name() == "paths") {
            return Err(Error::Shape("no output paths".into()));
        }
        Ok(Self {
            session,
            provider,
            batch: batch.filter(|&k| k > 0),
            coverage: opened.coverage,
        })
    }

    /// The provider it runs on (the CPU takes any node this one can't).
    pub fn provider(&self) -> Provider {
        self.provider
    }

    /// The samples the model draws at once (16), if its export fixed them.
    pub fn batch(&self) -> Option<usize> {
        self.batch
    }

    /// The provider's share of the graph, as ONNX Runtime logged it.
    pub fn coverage(&self) -> Option<Coverage> {
        self.coverage
    }

    /// Draw paths: `crop` (SIZE x SIZE, row by row), `cond` (COND), `noise` (n x POINTS x
    /// CHANNELS, one sample after another). Returns n paths, the same layout as `noise`.
    pub fn sample(&mut self, crop: &[f32], cond: &[f32], noise: &[f32]) -> Result<Vec<f32>> {
        if crop.len() != SIZE * SIZE || cond.len() != COND || !noise.len().is_multiple_of(PATH) {
            return Err(Error::Shape(format!(
                "inputs of {}, {} and {} numbers",
                crop.len(),
                cond.len(),
                noise.len()
            )));
        }
        let session = &mut self.session;
        batches(noise, self.batch, |part| {
            let k = part.len() / PATH;
            let outputs = session.run(ort::inputs![
                "crop" => TensorRef::from_array_view(([1, 1, SIZE, SIZE], crop))?,
                "cond" => TensorRef::from_array_view(([1, COND], cond))?,
                "noise" => TensorRef::from_array_view(([k, POINTS, CHANNELS], part))?,
            ])?;
            let (_, paths) = outputs["paths"].try_extract_tensor::<f32>()?;
            Ok(paths.to_vec())
        })
    }
}

/// Run `run` over `noise` (n samples of POINTS x CHANNELS) `batch` samples at a time,
/// the last batch padded with zeros and the padding dropped; `None`: all at once.
/// `engines.batches` in the Python worker.
pub fn batches<F>(noise: &[f32], batch: Option<usize>, mut run: F) -> Result<Vec<f32>>
where
    F: FnMut(&[f32]) -> Result<Vec<f32>>,
{
    let n = noise.len() / PATH;
    let k = match batch {
        Some(k) if k != n => k,
        _ => return run(noise),
    };
    let mut out = Vec::with_capacity(noise.len());
    let mut padded = vec![0.0; k * PATH];
    for part in noise.chunks(k * PATH) {
        let paths = if part.len() == k * PATH {
            run(part)?
        } else {
            padded[..part.len()].copy_from_slice(part);
            padded[part.len()..].fill(0.0);
            run(&padded)?
        };
        let keep = part.len().min(paths.len());
        out.extend_from_slice(&paths[..keep]);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn doubled(part: &[f32]) -> Result<Vec<f32>> {
        Ok(part.iter().map(|v| v * 2.0).collect())
    }

    #[test]
    fn batches_cover_every_sample_once() {
        let noise: Vec<f32> = (0..5 * PATH).map(|i| i as f32).collect();
        let mut sizes = Vec::new();
        let out = batches(&noise, Some(2), |part| {
            sizes.push(part.len() / PATH);
            doubled(part)
        })
        .unwrap();
        assert_eq!(sizes, [2, 2, 2]);
        assert_eq!(out, noise.iter().map(|v| v * 2.0).collect::<Vec<_>>());
    }

    #[test]
    fn one_run_when_the_count_fits() {
        let noise = vec![1.0; 16 * PATH];
        let mut runs = 0;
        batches(&noise, Some(16), |p| {
            runs += 1;
            doubled(p)
        })
        .unwrap();
        batches(&noise, None, |p| {
            runs += 1;
            doubled(p)
        })
        .unwrap();
        assert_eq!(runs, 2);
    }
}
