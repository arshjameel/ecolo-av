#!/usr/bin/env bash
# Start the whole simulated car on this computer, in one go:
#   Gazebo (the car, in the walled test lot), the nav screen (RViz), the HUD, and the
#   street-map window.
#
# Everything runs localhost-only (tools/ros.sh --local), so it shares one ROS graph on
# this computer and never reaches the real car on the network.
#
#   pixi run demo                 # windowed
#   pixi run demo fullscreen      # HUD and map full-screen
#   pixi run debugdemo            # the same, plus the debug HUD (tools/demo_all.sh debug):
#                                 # the lidar's cubes, the localizers, every sensor; with
#                                 # AMCL, a trial tracker, and NDT (if maps/ndt/test_lot.pcd
#                                 # is there: pixi run ndt-map drive) on (docs/hud.md)
#   pixi run debugdemo tracker:=mpc ndt:=false   # any sim.launch.py switch, name:=value:
#                                 # yours come last, so they win (docs/switches.md)
#   pixi run pick                 # a menu of the switches, instead of remembering them
#   Ctrl-C                        # stops them all
#
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -u

here=$(cd "$(dirname "$0")/.." && pwd)
cd "$here"

full=false
debug=false
extra=""
for arg in "$@"; do
  case "$arg" in
    fullscreen) full=true ;;
    debug) debug=true ;;
    *:=*) extra="$extra $arg" ;;
    *) echo "usage: tools/demo_all.sh [debug] [fullscreen] [name:=value ...]" >&2; exit 2 ;;
  esac
done
# The debug HUD's run: AMCL beside the global EKF and a trial tracker, so every layer
# and the panel's tracker have something to show (docs/designs/2026-10-29_debug-hud.md).
# The test lot already has nav and the lidar on, as amcl:=true needs.
sim_args="world:=test_lot"
steps=4
if $debug; then
  sim_args="$sim_args amcl:=true tracker:=pure_pursuit reversing:=false"
  steps=5
  # NDT too, when its map of the test lot has been made (it isn't in git).
  ndt_map=maps/ndt/test_lot.pcd
  if [ -f "$ndt_map" ]; then
    sim_args="$sim_args ndt:=true ndt_map:=$ndt_map"
    echo "NDT: on, with $ndt_map (the purple car and the panel's NDT section)"
  else
    echo "NDT: off: no $ndt_map yet. Make it once with: pixi run ndt-map drive (about 6 min, headless)"
  fi
fi

# The switches you gave come last: ros2 launch keeps the last value of each.
sim_args="$sim_args$extra"
echo "switches: $sim_args   (pixi run pick: a menu; docs/switches.md: every switch)"

# Gazebo's transport finds its own parts by multicast, which a Mac on some networks (or
# none) refuses ("No route to host"); on this computer only, it needs none.
export GZ_IP="${GZ_IP:-127.0.0.1}"

pids=""
cleanup() {
  echo
  echo "stopping everything..."
  for p in $pids; do kill "$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap cleanup INT TERM EXIT

start() { "$@" & pids="$pids $!"; }

# The map window draws Ottawa's streets and searches places from two downloads, kept out
# of git. Without them it starts anyway, with no streets and no search: say so first.
for f in maps/streets/ottawa.pmtiles:get-street-map maps/places/ottawa.db:get-places; do
  [ -f "${f%%:*}" ] || echo "note: no ${f%%:*}, so the map window misses it: run pixi run ${f##*:} once"
done

echo "1/$steps  Gazebo (the car, in the test lot) — give it ~20s to spawn..."
# shellcheck disable=SC2086  # sim_args is several words on purpose
start tools/ros.sh --local ros2 launch aav_bringup sim.launch.py $sim_args
sleep 20

echo "2/$steps  nav screen (RViz: map, costmaps, path, car)..."
start tools/ros.sh --local rviz2 -d ros/bringup/aav_bringup/rviz/drive.rviz

echo "3/$steps  HUD (in-car screen)..."
start tools/ros.sh --local ros2 launch aav_hud hud.launch.py fullscreen:=$full

echo "4/$steps  street-map window..."
start tools/ros.sh --local ros2 launch aav_hud map.launch.py fullscreen:=$full

if $debug; then
  echo "5/$steps  debug HUD (the lidar's cubes, the localizers, every sensor)..."
  start tools/ros.sh --local ros2 launch aav_hud debug_hud.launch.py
fi

echo
echo "All $steps started, localhost-only (the real car is untouched)."
echo "Press Ctrl-C here to stop everything."
wait
