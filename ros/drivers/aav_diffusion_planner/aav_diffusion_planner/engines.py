"""
Which engine runs the model, and how much time each part of a plan may take. No ROS.

ONNX Runtime runs the one .onnx file on whichever engine ("execution provider") the
computer has. engines() turns the provider the node was given into ONNX Runtime's list,
best first, each with its settings, and always the CPU last: an engine that can't load
the model (no GPU, no TensorRT) hands it down the list instead of failing the plan.

  TensorrtExecutionProvider  the Orin (JetPack's TensorRT): FP16, the engine built once
                             and kept in a cache folder (the first start takes minutes)
  CUDAExecutionProvider      the Orin's GPU without TensorRT, or a desktop's NVIDIA card
  CoreMLExecutionProvider    a Mac's GPU and Neural Engine
  CPUExecutionProvider       anywhere: the fallback

BUDGET is the design's time budget for one plan (docs/designs/2026-11-02_diffusion-
planner.md, "Speed"); the node's hard stop is STOP_MS, after which the lattice's path goes.
"""

TENSORRT = 'TensorrtExecutionProvider'
CUDA = 'CUDAExecutionProvider'
COREML = 'CoreMLExecutionProvider'
CPU = 'CPUExecutionProvider'
# Each engine, and what comes after it if it can't run the model.
CHAINS = {
    TENSORRT: (TENSORRT, CUDA, CPU),
    CUDA: (CUDA, CPU),
    COREML: (COREML, CPU),
    CPU: (CPU,),
}

# ms for one plan, on the Orin (the design's budget). The model: 16 samples, 10 steps.
BUDGET = {'crop': 2.0, 'model': 50.0, 'guard': 10.0, 'total': 100.0}
STOP_MS = 150.0


def engines(provider, cache_dir=None, fp16=True):
    """
    Return ONNX Runtime's providers for `provider`: [(name, options)], best first, CPU last.

    cache_dir: where TensorRT keeps the engines it builds (None: build each start). fp16:
    TensorRT works in half precision (the usual choice on a Jetson). A provider not in
    CHAINS: ValueError, saying which there are.
    """
    if provider not in CHAINS:
        raise ValueError(f'provider {provider!r}: pick one of {", ".join(CHAINS)}')
    out = []
    for name in CHAINS[provider]:
        options = {}
        if name == TENSORRT:
            options['trt_fp16_enable'] = bool(fp16)
            if cache_dir:
                options['trt_engine_cache_enable'] = True
                options['trt_engine_cache_path'] = str(cache_dir)
                options['trt_timing_cache_enable'] = True
                options['trt_timing_cache_path'] = str(cache_dir)
        out.append((name, options))
    return out


def usable(chain, available):
    """Return the chain's entries this ONNX Runtime has (get_available_providers())."""
    return [(name, options) for name, options in chain if name in available]


def verdict(ms, budget):
    """Return 'OK' if a time (ms) is within its budget, else 'OVER'."""
    return 'OK' if ms <= budget else 'OVER'


def batches(run, noise, k):
    """
    Return run's paths for every sample in `noise` (n, POINTS, CHANNELS), k at a time.

    Since 19 Nov the exports have a fixed number of samples, k (Core ML can't build an
    open one; TensorRT builds one engine a shape). A node asking for another number gets
    them in batches of k, the last padded with zeros and the padding dropped. k None or
    equal to n: one run.
    """
    import numpy as np
    n = noise.shape[0]
    if not k or k == n:
        return run(noise)
    out = []
    for at in range(0, n, k):
        part = noise[at:at + k]
        short = k - part.shape[0]
        if short:
            part = np.concatenate([part, np.zeros((short,) + part.shape[1:], part.dtype)])
        out.append(run(part)[:k - short])
    return np.concatenate(out)
