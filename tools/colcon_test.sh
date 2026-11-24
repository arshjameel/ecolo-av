#!/usr/bin/env bash
# colcon test for `pixi run test`: several packages at once, each pytest run on a ROS
# domain of its own; then the wall-clock timing tests alone; then the results.
#
# Until 24 Nov colcon tested one package at a time (17 min on Linux, 18 on ARM): side by
# side on one domain, two packages' tests heard each other's nodes (24 Oct). Now
# tools/pytest_plugins/aav_ros_domain.py gives each pytest run a domain no other process
# here holds (ROS's domain_coordinator), and the tests that chose their own take it
# (AAV_TEST_DOMAIN). Tests marked `timing` time a tracker on the wall clock, which runs
# slow beside other packages: they run after the rest, alone.
#
# usage: tools/colcon_test.sh   (AAV_COLCON_TEST_WORKERS=1 tests one package at a time)
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
  colcon test --parallel-workers "$workers" --event-handlers console_cohesion- \
  --pytest-args -p aav_ros_domain -m 'not timing'

echo "colcon test: the timing tests, alone"
# Their own results folder, so aav_track's results from above stay.
PYTEST_ADDOPTS="$addopts -p aav_ros_domain -m timing" \
  colcon test --packages-select aav_track --test-result-base build/timing-tests \
  --event-handlers console_cohesion- --pytest-args -p aav_ros_domain -m timing

colcon test-result --verbose
