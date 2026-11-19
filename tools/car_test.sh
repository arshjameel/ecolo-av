#!/usr/bin/env bash
# Phase A on stands, one command, one report (docs/testing-plan.md; docs/actuator-tests.md).
#
# usage: tools/car_test.sh --stands [--sim] [--no-manual] [--only STEP...] [--out DIR]
#        (pixi run car-test --stands ...)
#   --sim    starts its own Ravensim and tests it: this computer only (tools/ros.sh --local),
#            on its own ROS domain (AAV_SIM_DOMAIN, or a free one)
#   no --sim the real car on stands: whatever shares this ROS domain, so NOT --local. It
#            asks you to type "car-test" first, and each test asks again before it moves
set -euo pipefail
cd "$(dirname "$0")/.."
mode=car
for a in "$@"; do
  if [[ "$a" == --sim ]]; then mode=sim; fi
done
case "$mode" in
  sim) exec tools/ros.sh --local ros2 run aav_actuator_tests car_test "$@" ;;
  car) exec tools/ros.sh ros2 run aav_actuator_tests car_test "$@" ;;
esac
