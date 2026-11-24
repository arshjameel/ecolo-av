#!/usr/bin/env bash
# colcon test for `pixi run test`: several packages at once, each pytest run on a ROS
# domain of its own; then, one at a time, the tests that need the whole computer or the
# wall clock; then the results.
#
# Until 24 Nov colcon tested one package at a time (17 min on Linux, 18 on ARM): side by
# side on one domain, two packages' tests heard each other's nodes (25 Oct). Now
# tools/pytest_plugins/aav_ros_domain.py gives each pytest run a domain no other process
# here holds (ROS's domain_coordinator), and the tests that chose their own take it
# (AAV_TEST_DOMAIN). Two kinds run after the rest, alone (24 Nov, on CI beside two other
# packages): aav_mppi_lag, whose closed-loop runs already use 4 threads each and passed its
# 600 s limit; and the tests marked `timing`, which run or time a tracker on the wall clock
# (aav_track's node test found its pretend car stopped).
#
# usage: tools/colcon_test.sh [colcon test options, e.g. --packages-select aav_ndt aav_track]
#        (AAV_COLCON_TEST_WORKERS=1 tests one package at a time)
set -euo pipefail
cd "$(dirname "$0")/.."

# One core left for the tests' own nodes; at most 3 (CI's runners have 4 cores).
cores=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)
workers=$((cores - 1))
((workers > 3)) && workers=3
((workers < 1)) && workers=1
workers=${AAV_COLCON_TEST_WORKERS:-$workers}

export PYTHONPATH="$PWD/tools/pytest_plugins${PYTHONPATH:+:$PYTHONPATH}"
# CMake packages' pytest reads PYTEST_ADDOPTS; colcon replaces it for Python packages, so
# they get the same through --pytest-args.
addopts=${PYTEST_ADDOPTS:-}

echo "colcon test: $workers packages at a time, each pytest run on its own ROS domain"
PYTEST_ADDOPTS="$addopts -p aav_ros_domain -m 'not timing'" \
  colcon test --parallel-workers "$workers" --event-handlers console_cohesion- "$@" \
  --packages-skip aav_mppi_lag --pytest-args -p aav_ros_domain -m 'not timing'

# selected PACKAGE: true with no selection, or when the selection names it.
selected() { (($# == 1)) || [[ " ${*:2} " == *" $1 "* ]]; }
if selected aav_mppi_lag "$@"; then
  echo "colcon test: aav_mppi_lag, alone"
  colcon test --packages-select aav_mppi_lag --event-handlers console_cohesion-
fi
if selected aav_track "$@"; then
  echo "colcon test: the timing tests, alone"
  # Their own results folder, so aav_track's results from above stay.
  PYTEST_ADDOPTS="$addopts -p aav_ros_domain -m timing" \
    colcon test --packages-select aav_track --test-result-base build/timing-tests \
    --event-handlers console_cohesion- --pytest-args -p aav_ros_domain -m timing
fi

colcon test-result --verbose
