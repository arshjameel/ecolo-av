#!/usr/bin/env bash
# Make one of the sim's sensors or actuators fail on purpose, or mend it
# (docs/full-simulation.md step 6). Run it next to `pixi run sim`, on the same ROS domain.
#
# usage: pixi run sim-fault <sensor or actuator> <mode>
#   sensor: gps, imu, odom (the wheels), radar; camera, lidar (need `pixi run sim faults:=true`)
#   mode:   ok (mended), stop, freeze, nan (gps, imu, odom), jump, drift (gps only)
#   actuator: steer (ok, stuck), throttle (ok, stuck), brake (ok, weak, dead)
#   pixi run sim-fault gps stop      the GPS goes quiet
#   pixi run sim-fault brake weak    the brake gives 30 % of what is asked
#   pixi run sim-fault gps drift     the GPS slides 5 m north over a minute (no jump)
#   pixi run sim-fault gps ok        back to normal
#
# After a GPS jump, or wheels that disagree with the IMU (odom freeze while the car speeds
# up, brakes or turns), the sensor watchdog holds Nav2 until a person says all is right
# again: `pixi run sim-fault clear`.
set -euo pipefail
if [[ "${1:-}" == clear ]]; then
  exec ros2 service call /safety/sensor_watchdog/clear std_srvs/srv/Trigger
fi
if [[ $# -ne 2 ]]; then
  sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
  exit 2
fi
# Each sensor's own node takes its fault (aav_sim_models faults.py, NODES).
case "$1" in
  odom|steer|throttle|brake) node=sim_adapter ;;
  camera) node=sim_camera_relay ;;
  lidar) node=sim_lidar_relay ;;
  *) node="sim_$1" ;;
esac
exec ros2 param set "/$node" "fault.$1" "$2"
