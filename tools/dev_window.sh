#!/usr/bin/env bash
# The developer window (ros/hud/aav_dev_window): pretend data, or live ROS.
#
#   pixi run dev-window                # pretend data: no ROS, no car, no build
#   pixi run dev-window --live         # beside demo or sim on THIS computer: it may send
#                                      # goals and set the e-stop, only while every sensor says SIM
#   pixi run dev-window --watch        # listens to the network it's on, such as the car's,
#                                      # plugged in. It has no publishers at all
#
# Any other arguments go to the program (--cubes N, --seconds S, --screenshot FILE...).
# --live and --watch need pixi run build first, for the message types.
#
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -euo pipefail
cd "$(dirname "$0")/.."

manifest=ros/hud/aav_dev_window/Cargo.toml
ndt_map=maps/ndt/test_lot.pcd

# cargo runs quiet (-q): on a fresh clone the first build (Bevy) shows nothing for minutes.
if [ ! -x ros/hud/aav_dev_window/target/release/dev_window ]; then
  echo "dev_window: the first time, cargo builds the window (Bevy): about 2-5 min, no output" >&2
fi

case "${1:-}" in
  --live)
    shift
    exec tools/ros.sh --local cargo run --release -q --manifest-path "$manifest" \
      --features ros -- --live --developer --ndt-map "$ndt_map" "$@"
    ;;
  --watch)
    shift
    exec tools/ros.sh cargo run --release -q --manifest-path "$manifest" \
      --features ros -- --live "$@"
    ;;
  *)
    exec cargo run --release -q --manifest-path "$manifest" -- "$@"
    ;;
esac
