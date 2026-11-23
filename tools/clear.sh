#!/usr/bin/env bash
# A person clears a latch: the e-stop, or a safety node's latched stop.
#
# usage: tools/clear.sh <estop|gate|geofence|watchdog> [--sim]   (pixi run clear <...>)
#
#   estop     clear the e-stop (pixi run estop sets it)
#   gate      reopen the output mux after it latched STOP (docs/sources-and-targets.md)
#   geofence  the car is back inside the geofence: clear its latched OUTSIDE or JUMP
#   watchdog  all is right again: clear the sensor watchdog's latched faults
#
#   --sim     a simulator on this computer only (tools/ros.sh --local): the clear can
#             never reach a car on the same network. For the dry run (pixi run dry-run)
#
# The safety nodes refuse, with the reason, while the cause remains.
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -euo pipefail
cd "$(dirname "$0")/.."

usage() {
  echo "usage: pixi run clear <estop|gate|geofence|watchdog> [--sim]" >&2
  echo "  estop     clear the e-stop" >&2
  echo "  gate      reopen the output mux after it latched STOP" >&2
  echo "  geofence  clear the geofence's latched OUTSIDE or JUMP (car back inside)" >&2
  echo "  watchdog  clear the sensor watchdog's latched faults" >&2
  echo "  --sim     a simulator on this computer only: never a car on the network" >&2
  exit 2
}

[[ $# -eq 1 || ( $# -eq 2 && "$2" == "--sim" ) ]] || usage
ros="tools/ros.sh"
[[ "${2:-}" == "--sim" ]] && ros="tools/ros.sh --local"
# shellcheck disable=SC2086  # ros is a command and maybe --local, on purpose
case "$1" in
  estop)
    exec $ros ros2 topic pub -t 5 -r 10 /safety/estop std_msgs/msg/Bool "{data: false}" ;;
  gate)
    exec $ros ros2 service call /safety/output_mux/clear std_srvs/srv/Trigger ;;
  geofence)
    exec $ros ros2 service call /safety/geofence/clear std_srvs/srv/Trigger ;;
  watchdog)
    exec $ros ros2 service call /safety/sensor_watchdog/clear std_srvs/srv/Trigger ;;
  *) usage ;;
esac
