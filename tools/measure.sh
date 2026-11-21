#!/usr/bin/env bash
# Measure how well something works: the path followers, the timing, the camera stream,
# an A-to-B run, or the bandwidth of a running system.
#
# usage: tools/measure.sh <what> [args]   (pixi run measure <what>)
#
#   controllers  drive MPPI with lag (and the trials) over the same routes in
#                Ravensim, headless, and compare them (about 15 min; docs/controllers.md)
#   timing       are messages on time? Ravensim parked, then driving (about 10 min; docs/mac.md)
#   stream       the camera stream end to end, without the car (pixi run get mediamtx first;
#                docs/camera-pipeline.md)
#   atob         score A-to-B runs from bags or CSVs (docs/a-to-b.md)
#                (Ravensim; the road world or the campus; docs/perception.md)
#   bandwidth    every topic of a running system: Hz, size, MB/s (docs/bandwidth.md)
#   diffusion    the diffusion planner's time a plan against its budget, on this computer
#                (the learn environment; on the Orin, JetPack's Python: docs/car-day.md)
#   onnx         every ONNX model in models/ on each provider here (the CPU, Core ML...):
#                the same answers, how much of the graph, ms; PASS/WARN/FAIL, --json
#                (the ml environment, no ROS: docs/onnx.md)
#
# All but bandwidth start or read a simulator on this computer only (tools/ros.sh
# --local), so they never reach a real car. bandwidth measures whatever runs on this ROS
# domain: the car too.
# Each script's own --help lists its options.
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -euo pipefail
cd "$(dirname "$0")/.."

usage() {
  echo "usage: pixi run measure <controllers|timing|stream|atob|bandwidth|diffusion|onnx> [args]" >&2
  echo "  controllers  MPPI with lag (and the trials) in Ravensim (about 15 min)" >&2
  echo "  timing       are messages on time? Ravensim parked, then driving (about 10 min)" >&2
  echo "  stream       the camera stream end to end, without the car" >&2
  echo "  atob         score A-to-B runs: pixi run measure atob <bag or CSV>..." >&2
  echo "  bandwidth    every topic of a running system: Hz, size, MB/s" >&2
  echo "  diffusion    the diffusion planner's time a plan, against its budget (no ROS)" >&2
  echo "  onnx         every ONNX model: Core ML against the CPU, coverage, ms (no ROS)" >&2
  echo "  add --help after one for its options" >&2
  exit 2
}

[[ $# -ge 1 ]] || usage
what=$1
shift
case "$what" in
  controllers) exec tools/ros.sh --local python3 tools/controller_compare.py "$@" ;;
  timing)      exec tools/ros.sh --local python3 tools/timing_test.py "$@" ;;
  stream)      exec tools/ros.sh --local python3 tools/stream_check.py "$@" ;;
  atob)        exec tools/ros.sh --local python3 tools/atob_score.py "$@" ;;
  bandwidth)   exec tools/ros.sh python3 tools/bandwidth.py "$@" ;;
  diffusion)
    if [[ ! -x .pixi/envs/learn/bin/python ]]; then
      echo "no learn environment: pixi install --locked -e learn, once" >&2
      exit 1
    fi
    exec .pixi/envs/learn/bin/python ros/drivers/aav_diffusion_planner/training/measure.py "$@" ;;
  onnx)
    if [[ ! -x .pixi/envs/ml/bin/python ]]; then
      echo "no ml environment: pixi install --locked -e ml, once" >&2
      exit 1
    fi
    exec .pixi/envs/ml/bin/python tools/onnx_measure.py "$@" ;;
  *) usage ;;
esac
