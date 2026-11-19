//! Where ONNX Runtime runs a model, by the names our parameters use.

use std::fmt;

use crate::{Error, Result};

/// Where ONNX Runtime runs the model. The CPU takes any part another provider can't.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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

    /// ONNX Runtime's own name, as Python's `providers=[...]` takes it.
    pub fn ort_name(self) -> &'static str {
        match self {
            Self::Cpu => "CPUExecutionProvider",
            Self::CoreMl => "CoreMLExecutionProvider",
            Self::Cuda => "CUDAExecutionProvider",
            Self::TensorRt => "TensorrtExecutionProvider",
        }
    }

    /// The provider by its short name or ONNX Runtime's (any case), or `None`.
    pub fn from_name(name: &str) -> Option<Self> {
        let name = name.trim();
        Self::ALL.into_iter().find(|p| {
            p.name().eq_ignore_ascii_case(name) || p.ort_name().eq_ignore_ascii_case(name)
        })
    }

    /// The provider for a `device` or `provider` parameter: `auto` or empty (the best this
    /// build has), `cpu`, `coreml` (or `mps`, PyTorch's name for a Mac's GPU), `cuda` (or
    /// `cuda:N`: device N isn't picked yet, it runs on the first), `tensorrt`, or ONNX
    /// Runtime's names (`CoreMLExecutionProvider`...).
    pub fn from_device(device: &str) -> Result<Self> {
        let name = device.trim().to_ascii_lowercase();
        match name.as_str() {
            "" | "auto" => Ok(Self::best()),
            "mps" => Ok(Self::CoreMl),
            n if n.starts_with("cuda:") => Ok(Self::Cuda),
            n => Self::from_name(n).ok_or_else(|| {
                Error::Provider(format!(
                    "{device}: pick auto, cpu, coreml (mps), cuda or tensorrt"
                ))
            }),
        }
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

    /// Every provider this build has, the CPU first.
    pub fn built() -> Vec<Self> {
        let mut out = vec![Self::Cpu];
        if cfg!(all(feature = "coreml", target_os = "macos")) {
            out.push(Self::CoreMl);
        }
        if cfg!(feature = "cuda") {
            out.push(Self::Cuda);
        }
        if cfg!(feature = "tensorrt") {
            out.push(Self::TensorRt);
        }
        out
    }
}

impl fmt::Display for Provider {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_both_ways() {
        for p in Provider::ALL {
            assert_eq!(Provider::from_name(p.name()), Some(p));
            assert_eq!(Provider::from_name(p.ort_name()), Some(p));
        }
        assert_eq!(
            Provider::from_name(" coremlexecutionprovider "),
            Some(Provider::CoreMl)
        );
        assert_eq!(Provider::from_name("vulkan"), None);
    }

    #[test]
    fn devices() {
        assert_eq!(Provider::from_device("mps").ok(), Some(Provider::CoreMl));
        assert_eq!(Provider::from_device("cuda:1").ok(), Some(Provider::Cuda));
        assert_eq!(Provider::from_device("auto").ok(), Some(Provider::best()));
        assert_eq!(
            Provider::from_device("TensorrtExecutionProvider").ok(),
            Some(Provider::TensorRt)
        );
        let err = Provider::from_device("tpu").unwrap_err().to_string();
        assert!(err.contains("pick auto, cpu"), "{err}");
    }

    #[test]
    fn the_cpu_is_always_built() {
        assert_eq!(Provider::built()[0], Provider::Cpu);
        assert!(Provider::built().contains(&Provider::best()));
    }
}
