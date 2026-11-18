#!/usr/bin/env bash
# The simulator's cameras (ros/sim/aav_sim_cameras): Bevy draws the world from each camera
# on the car, offscreen, and the pictures go out on the car's camera topics (Gazebo's, until 16 Nov).
#
#   pixi run sim-cameras                      # the test world, 10 s: the timing check, no ROS
#   pixi run sim-cameras --save /tmp/pics     # and keep each camera's first picture
#   pixi run sim-cameras --bench 300          # as fast as it can: the most pictures a second
#   pixi run sim-cameras --ros                # publish the camera topics, a scripted drive
#   pixi run sim-cameras --ros --pose truth   # draw where /vehicle/sim/truth says (sim time)
#
# Other arguments go to the program (--seconds S, --cameras a,b, --from X, --speed M/S...).
# --ros needs pixi run build first, and stays on this computer (tools/ros.sh --local):
# it publishes sensor topics, so it must never meet a real car.
#
# Draws on the GPU: Metal on a Mac, Vulkan on Linux. With none, put tools/software_gl.sh in
# front (Mesa's lavapipe); tools/software_gl.sh --force tries that beside a GPU, a Mac too.
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -euo pipefail
cd "$(dirname "$0")/.."

manifest=ros/sim/aav_sim_cameras/Cargo.toml
# software_gl.sh's extra features (vulkan-on-mac), and on a Mac the Vulkan loader's folder:
# macOS drops DYLD_LIBRARY_PATH on the way through /usr/bin/env, so it comes under its own name.
extra=${AAV_SIM_CAMERAS_FEATURES:-}
if [[ -n "${AAV_VULKAN_LIB_DIR:-}" && "$(uname -s)" == Darwin ]]; then
  export DYLD_LIBRARY_PATH="$AAV_VULKAN_LIB_DIR${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
fi

case "${1:-}" in
  --ros) exec tools/ros.sh --local cargo run --release -q --manifest-path "$manifest" --features "ros${extra:+,$extra}" -- "$@" ;;
  *) exec cargo run --release -q --manifest-path "$manifest" ${extra:+--features "$extra"} -- "$@" ;;
esac
