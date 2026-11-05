#!/usr/bin/env bash
# One of our four raylib screens (docs/hud.md), live or with pretend data.
#
# usage: tools/screen.sh <hud|map|debug-hud|hw-panel> [--demo] [name:=value ...]
#        (pixi run hud, pixi run hud --demo, pixi run map fullscreen:=true ...)
#
#   live       the ROS node, through its launch file (needs pixi run build first)
#   --demo     pretend data: no ROS, no car, no build (cargo builds the window itself);
#              any other arguments go to the program (debug-hud --demo --bench-cubes 20000)
#
# The hardware panel drives Gazebo or the bench, so it runs localhost-only
# (tools/ros.sh --local): it never reaches a real car on the same network.
#
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -euo pipefail
cd "$(dirname "$0")/.."

usage() {
  echo "usage: pixi run <hud|map|debug-hud|hw-panel> [--demo] [name:=value ...]" >&2
  echo "  hud        the in-car screen" >&2
  echo "  map        the big screen's map window" >&2
  echo "  debug-hud  the debug screen: the lidar's cubes, the localizers, every sensor" >&2
  echo "  hw-panel   the hardware panel: every sensor's number, buttons (this computer only)" >&2
  echo "  --demo     pretend data: no ROS, no car" >&2
  exit 2
}

[[ $# -ge 1 ]] || usage
screen=$1
shift
case "$screen" in
  hud)       app=hud-app   launch=hud.launch.py       ros="tools/ros.sh" ;;
  map)       app=map-app   launch=map.launch.py       ros="tools/ros.sh" ;;
  debug-hud) app=debug-app launch=debug_hud.launch.py ros="tools/ros.sh" ;;
  hw-panel)  app=panel-app launch=hw_panel.launch.py  ros="tools/ros.sh --local" ;;
  *) usage ;;
esac

if [[ "${1:-}" == "--demo" ]]; then
  shift
  exec cargo run --release -q --locked --manifest-path ros/hud/aav_hud/Cargo.toml \
    -p "$app" --no-default-features -- --demo "$@"
fi
# shellcheck disable=SC2086  # ros is a command and maybe --local, on purpose
exec $ros ros2 launch aav_hud "$launch" "$@"
