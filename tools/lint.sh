#!/usr/bin/env bash
# Check our Python against ROS's own style rules (ament_flake8 and ament_pep257).
# The inherited packages are left out until someone brings them in line.
# Needs a ROS environment: `pixi run lint`, or source ROS first.
set -e
cd "$(dirname "$0")/.."
OURS=(ros/bringup/aav_bringup ros/bringup/aav_description ros/can_bridge ros/sim/aav_gazebo ros/sim/aav_turtlesim ros/sim/aav_carla ros/drivers ros/ego ros/localization
      ros/command_mux ros/io_mux ros/safety ros/sources/aav_sensors ros/sources/aav_camera_stream tools firmware/can firmware/spine/crates/legacy-link/gen_vectors.py
      ros/perception/aav_detector/aav_detector ros/perception/aav_detector/launch
      ros/perception/aav_detector/test ros/perception/aav_detector/setup.py ros/sim/aav_bench
      ros/perception/aav_lane_finder ros/perception/aav_lane_geometry/scripts/lane_speed.py
      ros/perception/aav_lane_geometry/test
      ros/hud/aav_hud/launch ros/perception/aav_radar_clusters/launch
      ros/perception/aav_radar_clusters/test ros/perception/aav_voxels)
ament_flake8 "${OURS[@]}"
ament_pep257 "${OURS[@]}"
