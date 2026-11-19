//! One ONNX Runtime session on the provider asked for, and only that one.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock};

use ort::environment::Environment;
use ort::logging::LogLevel;
use ort::session::builder::AutoDevicePolicy;
use ort::session::Session;

use crate::{Coverage, Error, Provider, Result};

/// Set (to anything) to see every line ONNX Runtime logs, and Core ML's compute plan.
pub const LOG_ENV: &str = "AAV_ONNX_LOG";

/// ONNX Runtime's lines while a session is built, kept for its [`Opened::notes`].
static NOTES: Mutex<Vec<String>> = Mutex::new(Vec::new());
/// One session built at a time, so each one's notes are its own.
static BUILDING: Mutex<()> = Mutex::new(());
static ENVIRONMENT: OnceLock<Environment> = OnceLock::new();

/// ONNX Runtime's one environment for this process. Its warnings go to stderr; its lines
/// on the providers' shares of the graph are kept as notes.
fn environment() -> Result<&'static Environment> {
    if let Some(env) = ENVIRONMENT.get() {
        return Ok(env);
    }
    let echo = std::env::var_os(LOG_ENV).is_some();
    let env = ort::init()
        .with_name("aav-onnx")
        .with_telemetry(false)
        .with_logger(Arc::new(
            move |level, _category, _id, _location, message: &str| {
                if echo {
                    eprintln!("onnxruntime [{level:?}]: {message}");
                }
                let placement = message.contains("GetCapability")
                    || message.contains("Node placements")
                    || message.contains("number of nodes")
                    || message.contains("TensorRT EP");
                if placement {
                    if let Ok(mut notes) = NOTES.lock() {
                        notes.push(message.trim().to_owned());
                    }
                } else if !echo
                    && matches!(level, LogLevel::Warning | LogLevel::Error | LogLevel::Fatal)
                {
                    eprintln!("onnxruntime: {message}");
                }
            },
        ))
        .build()?;
    env.set_log_level(LogLevel::Verbose);
    Ok(ENVIRONMENT.get_or_init(|| env))
}

/// How to open a model, beyond its provider.
#[derive(Debug, Clone, Default)]
pub struct Options {
    /// Write ONNX Runtime's profile (a JSON trace: each node, its provider and time) to
    /// this prefix; `Session::end_profiling` finishes it.
    pub profile: Option<PathBuf>,
    /// Where Core ML and TensorRT keep what they compile; `None`: [`cache_dir`].
    pub cache: Option<PathBuf>,
}

/// A session, with what ONNX Runtime said about where it put the graph.
pub struct Opened {
    /// The session, ready to run.
    pub session: Session,
    /// The provider asked for.
    pub provider: Provider,
    /// Its share of the graph, if it logged one (Core ML does; the CPU takes all).
    pub coverage: Option<Coverage>,
    /// ONNX Runtime's lines on the split, as it logged them.
    pub notes: Vec<String>,
}

impl std::fmt::Debug for Opened {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Opened")
            .field("provider", &self.provider)
            .field("coverage", &self.coverage)
            .finish_non_exhaustive()
    }
}

/// Where a provider keeps what it compiled from the model, so the next start is quick:
/// beside the model, `models/rf-detr-nano.cache/` (outside git, as `models/` is), with
/// `coreml/` and `tensorrt/` in it. TensorRT's engines hold to its version and the GPU:
/// a new JetPack builds them again.
pub fn cache_dir(model: &Path) -> PathBuf {
    model.with_extension("cache")
}

/// Load the ONNX model at `path` onto `provider` (and the CPU for what it can't take).
pub fn open(path: impl AsRef<Path>, provider: Provider, options: &Options) -> Result<Opened> {
    let path = path.as_ref();
    let env = environment()?;
    let cache = options.cache.clone().unwrap_or_else(|| cache_dir(path));
    let mut builder = Session::builder(env)?;
    // ort's own default picks a device itself, and on macOS always adds Core ML (in its
    // older, half-precision format): the CPU would not be the CPU. Only what we ask.
    builder = builder
        .with_auto_device(AutoDevicePolicy::PreferCPU)
        .map_err(ort::Error::from)?
        .with_execution_providers([dispatch(provider, &cache)?])
        .map_err(ort::Error::from)?;
    if let Some(prefix) = &options.profile {
        builder = builder.with_profiling(prefix).map_err(ort::Error::from)?;
    }
    let _one_at_a_time = BUILDING.lock().unwrap_or_else(|e| e.into_inner());
    take_notes();
    let session = builder.commit_from_file(path)?;
    let notes = take_notes();
    Ok(Opened {
        session,
        provider,
        coverage: Coverage::from_log(&notes),
        notes,
    })
}

fn take_notes() -> Vec<String> {
    NOTES
        .lock()
        .map(|mut n| std::mem::take(&mut *n))
        .unwrap_or_default()
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
            // ML Program: Core ML's newer format, full precision (the detector: 802 of
            // 808 nodes; the older NeuralNetwork: 673, in 94 pieces, half precision).
            // Static shapes: our models' are fixed, and Core ML's plan is better for it.
            Ok(ort::ep::CoreML::default()
                .with_model_format(ModelFormat::MLProgram)
                .with_compute_units(ComputeUnits::All)
                .with_static_input_shapes(true)
                .with_profile_compute_plan(std::env::var_os(LOG_ENV).is_some())
                .with_arbitrary_config("ModelCacheDirectory", cache.join("coreml").display())
                .build()
                .error_on_failure())
        }
        #[cfg(feature = "cuda")]
        Provider::Cuda => Ok(ort::ep::CUDA::default().build().error_on_failure()),
        // Half precision (as the Jetsons are usually run); the engine and the timing cache
        // kept, so only the first start builds them (minutes on the Orin). Untested.
        #[cfg(feature = "tensorrt")]
        Provider::TensorRt => {
            let dir = cache.join("tensorrt");
            Ok(ort::ep::TensorRT::default()
                .with_fp16(true)
                .with_engine_cache(true)
                .with_engine_cache_path(dir.display())
                .with_timing_cache(true)
                .with_timing_cache_path(dir.display())
                .build()
                .error_on_failure())
        }
        #[allow(unreachable_patterns)]
        other => Err(Error::Provider(format!(
            "{other}: this build lacks it (cargo feature {other})"
        ))),
    }
}
