#!/usr/bin/env bash
# Drive to one point and stop there (docs/a-to-b.md; aav_drive_to_point_rs's drive_to_point).
#
# usage: tools/drive_to_point.sh (--lat L --lon L | --x X --y Y | --ahead M [--left M]) [--speed V] [--sim]
#   --sim     Ravensim: this computer only (tools/ros.sh --local), on simulated time
#   neither   the real car: whatever shares this ROS domain, so NOT --local. It asks you to
#             type "drive" first; ego caps the speed at 1 m/s
# The client is aav_drive_to_point_rs's (Rust). The Python one, and --python, went on 22 Nov.
#
# As tools/actuator_test.sh: only --sim keeps ROS on this computer, and the client itself
# checks /system/source and refuses a mismatch.
set -euo pipefail
cd "$(dirname "$0")/.."
local=()
package=aav_drive_to_point_rs
args=()
for a in "$@"; do
  case "$a" in
    --sim) local=(--local); args+=("$a") ;;
    --python) echo "--python: the Python client was removed on 22 Nov 2026 (the Rust one runs)" >&2
              exit 2 ;;
    *) args+=("$a") ;;
  esac
done
exec tools/ros.sh ${local[@]+"${local[@]}"} ros2 run "$package" drive_to_point ${args[@]+"${args[@]}"}
