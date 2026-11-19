# aav_diffusion_rs

## Overview

The diffusion planner's **model worker in Rust**: a drop-in challenger for
`aav_diffusion_planner`'s `worker.py` (Ben, 19 Nov, TODO 243). The model's ONNX file,
run by ONNX Runtime through the `ort` crate: Core ML on a Mac, the CPU anywhere,
TensorRT on the Orin (untested). Why, and the numbers: [docs/onnx.md](../../../docs/onnx.md).

```
diffusion_planner (Python node) ──pipe──► diffusion_worker (Rust) ──► ONNX Runtime ──► Core ML / CPU / TensorRT
   the window, the guard (G1-G8),  ◄──────  16 paths a plan
   the lattice's fallback: unchanged
```

Pick it with `diffusion_impl:=rust` (`sim.launch.py`, `navigation.launch.py` with
`planner:=diffusion`); `diffusion_provider:=` picks the hardware (`auto`: Core ML on a
Mac). The node, its guard and its fallback stay the Python's: only the model moved.

| Where | Runs on | Tested |
|---|---|---|
| a Mac | Core ML: all 2,178 nodes, in one piece, the GPU; 9.7 ms a plan | [[works]] the CPU's and PyTorch's paths (1e-6); Ravensim scoring deferred (Ben, 19 Nov) |
| any computer | the CPU | [[works]] 87 ms a plan |
| the Orin | TensorRT (`-DDIFFUSION_RS=ON -DDIFFUSION_FEATURES=tensorrt`) | [[untested]] never built there |

## Interface

| Crate | Is | ROS? |
|---|---|---|
| `crates/diffusion-core` | the model (`Model`), batches of samples, the pipe (`wire`, `wire.py`'s bytes); the binary `diffusion_worker`; the example `bench` | no |

It uses the detector's session builder, `onnx-session`
(`ros/perception/aav_detector_rs/crates/onnx-session`): the provider by name, its cache,
its share of the graph.

### Binary `diffusion_worker`

| In / out | What |
|---|---|
| stdin | one JSON line `{"model": ..., "provider": ...}`, then requests (`wire.py`) |
| stdout | a reply for each: 16 paths, or "the model failed" |
| stderr | the model, the provider, its share of the graph, the load time; failures |

The provider: `auto`, `cpu`, `coreml`, `cuda`, `tensorrt`, or ONNX Runtime's names
(`CoreMLExecutionProvider`...). One that can't load or run the model: the CPU from then on.

## Invariants

- The same pipe, the same paths as `worker.py`: the node can't tell them apart.
- Only the model moved: the window, the guard and the lattice's fallback are the node's.
- No ROS here; the maths has its own tests.
- The model's export has a fixed 16 samples (Core ML can't build an open number); another
  number runs in batches.

## Side Effects

- Loads `models/diffusion/planner.onnx`; Core ML and TensorRT keep what they compile in
  `models/diffusion/planner.cache/` (outside git).

## Verification

```sh
cargo test --release -p diffusion-core --features coreml       # + the model, if built
cargo run --release -p diffusion-core --features coreml --example bench -- models/diffusion/planner.onnx
pixi run measure onnx --models diffusion
pixi run measure controllers --planners diffusion_rs --reversing --tours lot
```
