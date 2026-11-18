#!/usr/bin/env bash
# The diffusion planner's data and model (aav_diffusion_planner, a trial:
# docs/designs/2026-11-02_diffusion-planner.md). Everything lands in models/diffusion/,
# which git doesn't keep.
#
# usage: tools/diffusion.sh <what> [args]   (pixi run diffusion <what>)
#
#   data   the teachers' paths: map_server and planner_server (the lattice, Hybrid-A*) on
#          random maps, no simulator, on this computer only (about 10 min for 800 maps)
#   train  the dataset, the model (flow matching), its score by the guard, planner.onnx
#          (in the learn environment: pixi install --locked -e learn, once)
#   score  the guard's score of the trained model again, on the held-back maps
#   replay the plans a sim run recorded (diffusion_record:=), through a model, offline
#   export the trained weights to ONNX again: planner.onnx, and --fp16 planner_fp16.onnx
#
# Each one's --help lists its options. Portable: bash 3.2 (a Mac), no GNU-only flags.
set -euo pipefail
cd "$(dirname "$0")/.."
LEARN=.pixi/envs/learn/bin/python

usage() {
  echo "usage: pixi run diffusion <data|train|score|replay|export> [args]" >&2
  echo "  data   the lattice's and Hybrid-A*'s paths on random maps (no simulator)" >&2
  echo "  train  train the model on them, score it, export models/diffusion/planner.onnx" >&2
  echo "  score  score the trained model again with the guard" >&2
  echo "  replay replay a sim run's recorded plans through a model, offline" >&2
  echo "  export the weights to ONNX again (--fp16: planner_fp16.onnx too)" >&2
  exit 2
}

learn() {
  if [[ ! -x $LEARN ]]; then
    echo "no learn environment: run pixi install --locked -e learn, once" >&2
    exit 1
  fi
  local script=$1
  shift
  exec "$LEARN" "ros/drivers/aav_diffusion_planner/training/$script" "$@"
}

[[ $# -ge 1 ]] || usage
what=$1
shift
case "$what" in
  data)  exec tools/ros.sh --local python3 tools/diffusion_data.py "$@" ;;
  train) learn train.py train "$@" ;;
  score) learn train.py score "$@" ;;
  export) learn train.py export "$@" ;;
  replay) learn replay.py "$@" ;;
  *)     usage ;;
esac
