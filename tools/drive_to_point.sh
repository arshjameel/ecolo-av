#!/usr/bin/env bash
# Drive to one point and stop there (docs/a-to-b.md; aav_drive_to_point's point_cli).
#
# usage: tools/drive_to_point.sh (--lat L --lon L | --x X --y Y | --ahead M [--left M]) [--speed V] [--sim]
#   --sim    Ravensim: this computer only (tools/ros.sh --local), on simulated time
#   neither  the real car: whatever shares this ROS domain, so NOT --local. It asks you to
#            type "drive" first; ego caps the speed at 1 m/s
#
# As tools/actuator_test.sh: only --sim keeps ROS on this computer, and the client itself
# checks /system/source and refuses a mismatch.
set -euo pipefail
cd "$(dirname "$0")/.."
local=()
for a in "$@"; do
  [[ "$a" == --sim ]] && local=(--local)
done
exec tools/ros.sh ${local[@]+"${local[@]}"} ros2 run aav_drive_to_point drive_to_point "$@"
