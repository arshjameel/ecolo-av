#!/usr/bin/env bash
# Record a drive. Raw inputs (to re-run localization: aav_bringup replay.launch.py),
# commands (to replay the drive: tools/replay_commands.sh), and what the stack
# decided (to see why it did what it did). No camera images: they are huge.
# The actuators' own reports and the go-to driver's feedback (hidden: its action topic)
# let tools/atob_score.py score an A-to-B run from the bag; Gazebo's truth when in sim.
#
# usage: tools/record.sh [--lidar] [name]   -> bags/<date>_<name>/ (MCAP, Jazzy's default)
#   --lidar  also the lidar's cloud, to build the NDT map at home (pixi run ndt-map build):
#            about 9 MB/s, so 10 min is about 5 GB (docs/tests.md, 4.19)
set -euo pipefail
cd "$(dirname "$0")/.."
lidar=()
if [[ "${1:-}" == --lidar ]]; then
  lidar=(/sensors/lidar/points)
  shift
fi
name="${1:-drive}"
out="bags/$(date +%Y-%m-%d_%H%M%S)_${name}"
mkdir -p bags
# Through tools/ros.sh, which sources the workspace last: on a Mac, starting this script
# (/usr/bin/env bash) drops DYLD_LIBRARY_PATH, and then ros2 bag can't load our own
# messages (/vehicle/status).
exec tools/ros.sh ros2 bag record -o "$out" --include-hidden-topics --topics \
  /vehicle/odom /vehicle/status /vehicle/joint_states /vehicle/wheels \
  /vehicle/steering /vehicle/throttle /vehicle/brake /vehicle/gazebo/truth /system/source \
  /sensors/imu/data_raw /sensors/imu/data /sensors/imu/mag /sensors/gps/fix /sensors/gps/fix_velocity /vehicle/battery \
  /drivers/joy/joy /safety/estop /control/cmd_vel /drivers/joy/cmd_vel /drivers/keyboard/cmd_vel /drivers/scripted/cmd_vel /drivers/nav/cmd_vel \
  /drivers/point/cmd_vel /drivers/rc/cmd_vel /drivers/tests/cmd_vel /drive_to_point/_action/feedback /drive_to_point/_action/status \
  /ego/actuator_cmd /can/tx /can/rx \
  /localization/odom/local /localization/odom/global /localization/gps/odom /localization/gps/fix /localization/gps/heading \
  /tf /tf_static /robot_description /plan /goal_pose /clock ${lidar[@]+"${lidar[@]}"}
