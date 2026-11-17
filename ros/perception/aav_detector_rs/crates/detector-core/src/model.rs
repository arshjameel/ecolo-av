//! The ONNX model through ONNX Runtime (the `ort` crate), on the provider asked for.

use std::fmt;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock};

use ort::environment::Environment;
use ort::logging::LogLevel;
use ort::session::builder::AutoDevicePolicy;
use ort::session::Session;
use ort::value::TensorRef;

use crate::{decode, preprocess, DecodeOptions, Detection, Error, InputSize, Picture, Result};

/// Where ONNX Runtime runs the model. The CPU takes any part another provider can't.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Provider {
    /// The CPU alone: runs anywhere.
    Cpu,
    /// Core ML: a Mac's GPU and Neural Engine (feature `coreml`).
    CoreMl,
    /// CUDA: an NVIDIA GPU, the Orin's (feature `cuda`). Untested.
    Cuda,
    /// TensorRT: the Orin's GPU, half precision, its engine cached (feature `tensorrt`).
    /// Untested.
    TensorRt,
}

impl Provider {
    /// Every provider, for a parameter's help.
    pub const ALL: [Provider; 4] = [Self::Cpu, Self::CoreMl, Self::Cuda, Self::TensorRt];

    /// The name a parameter uses: `cpu`, `coreml`, `cuda`, `tensorrt`.
    pub fn name(self) -> &'static str {
        match self {
            Self::Cpu => "cpu",
            Self::CoreMl => "coreml",
            Self::Cuda => "cuda",
            Self::TensorRt => "tensorrt",
        }
    }

    /// The provider by its parameter name, or `None`.
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL
            .into_iter()
            .find(|p| p.name() == name.trim().to_ascii_lowercase())
    }

    /// The best this build has: Core ML on a Mac, TensorRT, then CUDA, else the CPU.
    pub fn best() -> Self {
        if cfg!(all(feature = "coreml", target_os = "macos")) {
            Self::CoreMl
        } else if cfg!(feature = "tensorrt") {
            Self::TensorRt
        } else if cfg!(feature = "cuda") {
            Self::Cuda
        } else {
            Self::Cpu
        }
    }
}

impl fmt::Display for Provider {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// ONNX Runtime's lines about which provider took which nodes, kept for [`Detector::notes`].
static NOTES: Mutex<Vec<String>> = Mutex::new(Vec::new());
static ENVIRONMENT: OnceLock<Environment> = OnceLock::new();

/// ONNX Runtime's one environment for this process. Its warnings go to stderr; its lines
/// on the providers' shares of the graph are kept as notes.
fn environment() -> Result<&'static Environment> {
    if let Some(env) = ENVIRONMENT.get() {
        return Ok(env);
    }
    let env = ort::init()
        .with_name("detector-core")
        .with_telemetry(false)
        .with_logger(Arc::new(
            |level, _category, _id, _location, message: &str| {
                let placement = message.contains("GetCapability")
                    || message.contains("Node placements")
                    || message.contains("number of nodes");
                if std::env::var_os("DETECTOR_COREML_PLAN").is_some() {
                    eprintln!("onnxruntime [{level:?}]: {message}");
                }
                if placement {
                    if let Ok(mut notes) = NOTES.lock() {
                        notes.push(message.trim().to_owned());
                    }
                } else if matches!(level, LogLevel::Warning | LogLevel::Error | LogLevel::Fatal) {
                    eprintln!("onnxruntime: {message}");
                }
            },
        ))
        .build()?;
    env.set_log_level(LogLevel::Verbose);
    Ok(ENVIRONMENT.get_or_init(|| env))
}

/// RF-DETR, exported to ONNX, ready to run: [`Detector::detect`] for each picture.
pub struct Detector {
    session: Session,
    input: String,
    boxes: String,
    logits: String,
    size: InputSize,
    provider: Provider,
}

impl fmt::Debug for Detector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Detector")
            .field("input", &self.input)
            .field("size", &self.size)
            .field("provider", &self.provider)
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
        let env = environment()?;
        let mut builder = Session::builder(env)?;
        // ort's own default picks a device itself, and on macOS always adds Core ML (in its
        // older, half-precision format): the CPU would not be the CPU. Only what we ask.
        builder = builder
            .with_auto_device(AutoDevicePolicy::PreferCPU)
            .map_err(ort::Error::from)?
            .with_execution_providers([dispatch(provider, &cache_dir(path.as_ref()))?])
            .map_err(ort::Error::from)?;
        if let Some(prefix) = profile {
            builder = builder.with_profiling(prefix).map_err(ort::Error::from)?;
        }
        let session = builder.commit_from_file(path)?;
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
        })
    }

    /// The model's input size (384 x 384 for RF-DETR Nano).
    pub fn input_size(&self) -> InputSize {
        self.size
    }

    /// The provider asked for. The CPU still runs any node it couldn't take:
    /// [`Detector::notes`] says how many.
    pub fn provider(&self) -> Provider {
        self.provider
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

    /// ONNX Runtime's lines, so far, on which provider took how much of each graph (Core
    /// ML's: "number of nodes supported by CoreML").
    pub fn notes() -> Vec<String> {
        NOTES.lock().map(|n| n.clone()).unwrap_or_default()
    }
}

/// Where a provider keeps what it compiled from the model, so the next start is quick:
/// beside the model, `models/rf-detr-nano.cache/` (outside git, as models/ is).
pub fn cache_dir(model: &Path) -> PathBuf {
    model.with_extension("cache")
}

/// The provider's registration, its compiled model kept in `cache`; an error if this build
/// lacks it.
#[allow(unused_variables)] // `cache`: only Core ML and TensorRT keep one
fn dispatch(provider: Provider, cache: &Path) -> Result<ort::ep::ExecutionProviderDispatch> {
    match provider {
        Provider::Cpu => Ok(ort::ep::CPU::default().build()),
        #[cfg(feature = "coreml")]
        Provider::CoreMl => {
            use ort::ep::coreml::{ComputeUnits, ModelFormat};
            use ort::ep::ArbitrarilyConfigurableExecutionProvider;
            // ML Program: Core ML's newer format, full precision, takes 802 of the 808
            // nodes (the older NeuralNetwork: 673, in 94 pieces, half precision).
            Ok(ort::ep::CoreML::default()
                .with_model_format(ModelFormat::MLProgram)
                .with_compute_units(ComputeUnits::All)
                .with_static_input_shapes(true)
                .with_profile_compute_plan(std::env::var_os("DETECTOR_COREML_PLAN").is_some())
                .with_arbitrary_config("ModelCacheDirectory", cache.join("coreml").display())
                .build()
                .error_on_failure())
        }
        #[cfg(feature = "cuda")]
        Provider::Cuda => Ok(ort::ep::CUDA::default().build().error_on_failure()),
        #[cfg(feature = "tensorrt")]
        Provider::TensorRt => Ok(ort::ep::TensorRT::default()
            .with_fp16(true)
            .with_engine_cache(true)
            .with_engine_cache_path(cache.join("tensorrt").display())
            .build()
            .error_on_failure()),
        #[allow(unreachable_patterns)]
        other => Err(Error::Provider(format!(
            "{other}: this build lacks it (cargo feature {other})"
        ))),
    }
}
