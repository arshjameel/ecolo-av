#!/usr/bin/env bash
# The actuator tests (docs/tests.md stages 3-4; docs/actuator-tests.md).
#
# usage: tools/actuator_test.sh <test> [--sim|--bench] [--stands] [options]
#        (pixi run actuator-test; pixi run preflight is "preflight --stands")
#   --sim    Gazebo: this computer only (tools/ros.sh --local), on simulated time
#   --bench  bench mode: this computer only (tools/ros.sh --local)
#   neither  the real car: whatever shares this ROS domain, so NOT --local. The test asks
#            you to type its name first, and caps the speed at 1 m/s (0.3 on --stands)
#
# Only --sim and --bench keep ROS on this computer: a test meant for a stand-in can never
# reach a car on the same network, and a test meant for the car can't run against a local
# stand-in by mistake either (the test itself checks /system/source and refuses).
set -euo pipefail
cd "$(dirname "$0")/.."
local=()
for a in "$@"; do
  [[ "$a" == --sim || "$a" == --bench ]] && local=(--local)
done
exec tools/ros.sh ${local[@]+"${local[@]}"} ros2 run aav_actuator_tests actuator_test "$@"
