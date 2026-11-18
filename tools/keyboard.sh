#!/usr/bin/env bash
# Drive from the keyboard: teleop_twist_keyboard into twist_mux as the "keyboard" driver,
# in the namespace /drivers/keyboard, so it publishes /drivers/keyboard/cmd_vel
# (priority 90, so the joystick still outranks it). Hold a key to keep moving; let go
# and the car stops within about a second. Keys: i forward, u/o forward left/right,
# , reverse, k stop, q/z faster/slower.
#
# usage: tools/keyboard.sh [--sim | --local]
#   --sim    simulation time (Ravensim, replay), and stay on this computer
#   --local  stay on this computer, on the wall clock (the bench, the turtle)
#   neither  drive whatever shares this ROS domain: on the car's network, the car
set -euo pipefail
sim=false
here=false
case "${1:-}" in
  --sim) sim=true; here=true ;;
  --local) here=true ;;
esac
if [[ "$here" == true ]]; then
  # The same rule as tools/ros.sh --local: pixi has already set SUBNET, so override it.
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${AAV_DISCOVERY:-LOCALHOST}"
  [[ "$ROS_AUTOMATIC_DISCOVERY_RANGE" == LOCALHOST ]] && unset ROS_STATIC_PEERS
fi
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p use_sim_time:=$sim -r __ns:=/drivers/keyboard
