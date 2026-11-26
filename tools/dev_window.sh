#!/usr/bin/env bash
# The developer window (ros/hud/aav_dev_window): pretend data, or live ROS.
#
#   pixi run dev-window                # pretend data: no ROS, no car, no build
#   pixi run dev-window --live         # beside demo or sim on THIS computer: it may send
#                                      # goals and set the e-stop, only while every sensor says SIM
#   pixi run dev-window --watch        # listens to the network it's on, such as the car's,
#                                      # plugged in. It has no publishers at all
#
#   pixi run nav-view [--live|--watch] # the same, as the navigation view: only the status
#                                      # strip and the map panel, in a small window of its own
#                                      # (this script with --map-only first)
#
# Any other arguments go to the program (--cubes N, --seconds S, --screenshot FILE, --map-span M,
# --title WORDS, --position X,Y...).
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

# The navigation view (pixi run nav-view): the same program, only its strip and map panel.
only=""
if [ "${1:-}" = "--map-only" ]; then
  only="--map-only"
  shift
fi

case "${1:-}" in
  --live)
    shift
    exec tools/ros.sh --local cargo run --release -q --manifest-path "$manifest" \
      --features ros -- $only --live --developer --ndt-map "$ndt_map" "$@"
    ;;
  --watch)
    shift
    exec tools/ros.sh cargo run --release -q --manifest-path "$manifest" \
      --features ros -- $only --live "$@"
    ;;
  *)
    exec cargo run --release -q --manifest-path "$manifest" -- $only "$@"
    ;;
esac
