#!/usr/bin/env bash
# Ravensim and the developer window, in one go: the window Ben means by "see Ravensim".
#
#   pixi run ravensim                  # Ravensim (pixi run sim's world) + the developer window, live
#   pixi run ravensim world:=campus    # any sim.launch.py switch, name:=value, goes to the sim
#   Ctrl-C, or close the window        # stops both
#
# What it starts, both on this computer only (pixi runs it through tools/ros.sh --local):
#   1. the sim:      ros2 launch aav_bringup sim.launch.py rviz:=false <your switches>
#   2. the window:   tools/dev_window.sh --live (scene and console left; lidar, map, keys right)
#                    on the campus, with its RavenTiles and lanes when pixi run campus made them
#
# Both run in one process group of their own. Ctrl-C here, or either one ending, stops the
# whole group: SIGTERM first (ros2 launch shuts its nodes down), SIGKILL after 20 s.
# Changes no default: the sim gets rviz:=false, then your switches (the last value wins).
#
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -u
cd "$(dirname "$0")/.."

sim_args="rviz:=false"
world=""
for arg in "$@"; do
  case "$arg" in
    world:=*) world=${arg#world:=}; sim_args="$sim_args $arg" ;;
    *:=*) sim_args="$sim_args $arg" ;;
    *) echo "usage: pixi run ravensim [name:=value ...]   (sim.launch.py's switches)" >&2; exit 2 ;;
  esac
done

window_args="--live"
if [ "$world" = campus ]; then
  [ -d maps/campus/chunks ] && window_args="$window_args --chunks maps/campus/chunks"
  [ -f maps/campus/campus.osm ] && window_args="$window_args --lanes maps/campus/campus.osm"
  [ -d maps/campus/chunks ] || echo "note: no maps/campus/chunks: run pixi run campus once for the map's RavenTiles"
fi
echo "sim:    sim.launch.py $sim_args"
echo "window: tools/dev_window.sh $window_args"

# set -m: the subshell below gets a process group of its own; everything it starts joins
# it (job control is off inside). Ctrl-C reaches only this script, which stops the group.
set -m
(
  set +m
  # Through tools/ros.sh again: macOS drops DYLD_LIBRARY_PATH on the way into this script
  # (/usr/bin/env is a system program), and the nodes need it to find our messages.
  # shellcheck disable=SC2086  # several words on purpose
  tools/ros.sh --local ros2 launch aav_bringup sim.launch.py $sim_args &
  sim=$!
  # shellcheck disable=SC2086
  tools/dev_window.sh $window_args &
  window=$!
  # Bash 3.2 has no wait -n: look once a second. When one ends, end the other.
  while kill -0 "$sim" 2>/dev/null && kill -0 "$window" 2>/dev/null; do sleep 1; done
  kill -TERM "$sim" "$window" 2>/dev/null
  wait 2>/dev/null  # no "Terminated" line for the one we stopped
) &
group=$!
set +m

stop() {
  trap '' INT TERM
  if kill -0 -- "-$group" 2>/dev/null; then
    echo
    echo "stopping Ravensim and the window..."
    kill -TERM -- "-$group" 2>/dev/null
    n=0
    while kill -0 -- "-$group" 2>/dev/null && [ $n -lt 20 ]; do sleep 1; n=$((n + 1)); done
    kill -KILL -- "-$group" 2>/dev/null
  fi
}
trap 'stop; exit 130' INT TERM
trap stop EXIT
echo "Ctrl-C here, or close the window, to stop both."
wait "$group"
