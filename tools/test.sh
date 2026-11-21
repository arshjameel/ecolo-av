#!/usr/bin/env bash
# Run every test that doesn't need ROS: our packages' Python unit tests, the Rust spine
# library, the HUD's, the map window's, the debug HUD's and the hardware panel's logic
# (hud-core, map-core, debug-core, panel-core), NDT (ndt-core), AMCL (amcl-core), and the check that docs/interfaces.md matches
# docs/interfaces.yaml. Where the mcp environment is installed, also the MCP server over
# stdio (on a private, local domain).
# Needs Python 3 with pytest and PyYAML; Rust (cargo) for the spine. Works on any OS.
#
# usage: tools/test.sh
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONDONTWRITEBYTECODE=1
# Tests never reach the network: no car on the same WiFi hears them, and on GitHub's Macs
# a test's ROS program that used multicast never ended (Cyclone DDS, TODO 42).
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Each package's tests import the package, so pytest runs from inside it.
# (aav_sim_models's adapter test imports ROS messages, so only `colcon test` runs that one.)
for pkg in ros/ego/aav_ego ros/io_mux/aav_io_mux ros/can_bridge/aav_spine ros/can_bridge/aav_legacy \
           ros/sim/aav_sim_models ros/sim/aav_carla \
           ros/drivers/aav_scripted ros/drivers/aav_language ros/drivers/aav_navigation \
           ros/drivers/aav_actuator_tests ros/drivers/aav_drive_to_point ros/drivers/aav_routes ros/drivers/aav_rc \
           ros/drivers/aav_track ros/drivers/aav_diffusion_planner \
           ros/bringup/aav_description ros/bringup/aav_run_info ros/localization/aav_kiss_icp ros/localization/aav_lane_pose \
           ros/perception/aav_detector ros/sources/aav_camera_stream \
           ros/perception/aav_lane_finder ros/perception/aav_lidar \
           ros/perception/aav_voxels \
           ros/safety/aav_safety ros/safety/aav_ghost \
           ros/sim/aav_bench; do
  echo "== $pkg"
  (cd "$pkg" && python3 -m pytest -q -p no:cacheprovider)
done

echo "== tools: the MCP server's parts, the Pico 2's UF2 converter, CI's annotations"
python3 -m pytest -q -p no:cacheprovider tools/tests

echo "== firmware/spine"
if command -v cargo >/dev/null; then
  (cd firmware/spine && cargo test -q)
  echo "== ros/sources/aav_imu_yahboom: the IMU's serial frames"
  (cd ros/sources/aav_imu_yahboom && cargo test -q -p yahboom-protocol)
  echo "== ros/perception/aav_radar_clusters: DBSCAN, the boxes, the C++ node's answers"
  (cd ros/perception/aav_radar_clusters && cargo test -q --locked -p radar-clustering)
  echo "== ros/perception/aav_voxels_rs: the cubes in Rust, the Python's answers"
  (cd ros/perception/aav_voxels_rs && cargo test -q --locked -p voxels)
  echo "== ros/perception/aav_lidar_rs: the flat scan in Rust, the Python's ranges"
  (cd ros/perception/aav_lidar_rs && cargo test -q --locked -p lidar-scan)
  echo "== ros/perception/aav_things_rs: the things in Rust, the Python modules' answers"
  (cd ros/perception/aav_things_rs && cargo test -q --locked -p things-core)
  echo "== ros/perception/aav_lane_finder_rs: the lane finder in Rust, the Python's lanes on the same pictures"
  (cd ros/perception/aav_lane_finder_rs && cargo test -q --locked -p lane-core)
  echo "== ros/perception/aav_lane_finder_learned: the learned lane finder's maths (crop as Pillow, decode, ground, lane)"
  (cd ros/perception/aav_lane_finder_learned && cargo test -q --locked -p ufld-core)
  echo "== ros/sources/aav_camera_stream_rs: the camera stream in Rust, the Python modules' answers"
  (cd ros/sources/aav_camera_stream_rs && cargo test -q --locked -p stream-core)
  echo "== ros/perception/aav_detector_rs: the detector's maths in Rust (resize, decode, labels)"
  (cd ros/perception/aav_detector_rs && cargo test -q --locked -p detector-core)
  echo "== ros/ego/aav_ego_rs: ego and look_ahead in Rust, the Python modules' answers"
  (cd ros/ego/aav_ego_rs && cargo test -q --locked -p ego-core)
  echo "== ros/drivers/aav_behaviour_rs: the behaviour planner in Rust, the Python modules' answers"
  (cd ros/drivers/aav_behaviour_rs && cargo test -q --locked -p behaviour-core)
  echo "== ros/safety/aav_watchdog_rs: the sensor watchdog in Rust, the Python node's answers"
  (cd ros/safety/aav_watchdog_rs && cargo test -q --locked -p watchdog-core)
  echo "== ros/safety/aav_self_test_rs: the boot self-test's checks and verdict (TODO 255)"
  (cd ros/safety/aav_self_test_rs && cargo test -q --locked -p self-test-core)
  echo "== ros/io_mux/aav_io_mux_rs: the sensor mux and the output mux in Rust, the Python's answers"
  (cd ros/io_mux/aav_io_mux_rs && cargo test -q --locked -p io-mux-core)
  echo "== ros/localization/aav_localization_health_rs: localization_health in Rust, the Python's answers"
  (cd ros/localization/aav_localization_health_rs && cargo test -q --locked -p health-core)
  echo "== ros/localization/aav_kinematic_icp: lidar odometry for a car, synthetic scans"
  (cd ros/localization/aav_kinematic_icp && cargo test -q --locked -p kinematic-icp)
  echo "== ros/can_bridge/aav_spine_bridge: the Rust CAN bridge's logic, the shared CAN vectors"
  (cd ros/can_bridge/aav_spine_bridge && cargo test -q --locked -p spine-bridge)
  echo "== ros/can_bridge/aav_legacy_rs: the Rust UART bridge's logic, the Python modules' answers"
  (cd ros/can_bridge/aav_legacy_rs && cargo test -q --locked -p legacy-bridge)
  echo "== ros/drivers/aav_rc_rs: rc_driver's rules in Rust, rc_drive.py's answers"
  (cd ros/drivers/aav_rc_rs && cargo test -q --locked -p rc-core)
  echo "== ros/safety/aav_joy_alive_rs: joy_alive's rules in Rust, joy_alive.py's answers"
  (cd ros/safety/aav_joy_alive_rs && cargo test -q --locked -p joy-alive-core)
  echo "== ros/safety/aav_safety_rs: the geofence, stop hold and leash in Rust, the Python's answers"
  (cd ros/safety/aav_safety_rs && cargo test -q --locked -p safety-core)
  echo "== ros/localization/aav_gps_heading_rs: the GPS heading in Rust, course.py's answers"
  (cd ros/localization/aav_gps_heading_rs && cargo test -q --locked -p gps-heading-core)
  echo "== ros/command_mux/aav_mux_state: who is driving, by twist_mux's rule and files"
  (cd ros/command_mux/aav_mux_state && cargo test -q --locked -p mux-state-core)
  echo "== ros/perception/aav_stop_sign_rs: stop_sign_watcher's rules in Rust, stop_sign.py's answers"
  (cd ros/perception/aav_stop_sign_rs && cargo test -q --locked -p stop-sign-core)
  echo "== ros/drivers/aav_drive_to_point_rs: the point driver in Rust, the Python modules' answers"
  (cd ros/drivers/aav_drive_to_point_rs && cargo test -q --locked -p point-core)
  echo "== ros/bringup/aav_description/car-model: the car model's Rust twin, the shared vectors"
  (cd ros/bringup/aav_description/car-model && cargo test -q --locked)
  echo "== ros/bringup/aav_description/cloud-view: a PointCloud2's bytes, shared (TODO 176)"
  (cd ros/bringup/aav_description/cloud-view && cargo test -q --locked)
  echo "== ros/bringup/aav_description/map-chunks: the map's fixed 64 m chunks (TODO 175)"
  (cd ros/bringup/aav_description/map-chunks && cargo test -q --locked)
  echo "== ros/bringup/aav_description/map-edit: the map editor's edits, saves and layers (TODO 251)"
  (cd ros/bringup/aav_description/map-edit && cargo test -q --locked)
  echo "== ros/bringup/aav_description/lanelet-map: Lanelet2 maps in Rust (TODO 175)"
  (cd ros/bringup/aav_description/lanelet-map && cargo test -q --locked)
  echo "== ros/safety/aav_stop_zone: the stop zone's decision in Rust (TODO 182)"
  (cd ros/safety/aav_stop_zone && cargo test -q --locked -p stop-zone)
  echo "== ros/localization/aav_ndt: NDT's maths, the filters, LOST; the design's checks"
  (cd ros/localization/aav_ndt && cargo test -q --locked -p ndt-core)
  echo "== ros/localization/aav_pose_check: NDT leads, the GPS checks; who is wrong; the seed"
  (cd ros/localization/aav_pose_check && cargo test -q --locked -p pose-check-core)
  echo "== ros/localization/aav_amcl_rs: AMCL's filter: a near guess, a kidnap, KLD's bounds"
  (cd ros/localization/aav_amcl_rs && cargo test -q --locked -p amcl-core)
  echo "== ros/sim/aav_simulator: Ravensim's core: the car, the rays, the worlds, the camera lockstep"
  (cd ros/sim/aav_simulator && cargo test -q --locked -p sim-core --features shared)
  echo "== ros/sim/aav_spine_sim: bench mode's car: the spine's real logic on a model car"
  (cd ros/sim/aav_spine_sim && cargo test -q --locked -p bench-car)
  # The closed-loop runs (tests/closed_loop.rs) are the slowest Rust tests we have. Inside
  # `pixi run test` (AAV_COLCON_TEST_FOLLOWS=1), colcon test runs the whole crate again in
  # release, so they run once, there; alone, tools/test.sh runs them too (TODO 209).
  if [[ ${AAV_COLCON_TEST_FOLLOWS:-0} == 1 ]]; then
    echo "== ros/drivers/aav_mppi_lag: MPPI with the lag, its critics (closed loop: in colcon test)"
    (cd ros/drivers/aav_mppi_lag && cargo test -q --locked -p mppi-lag --features shared \
       --lib --test bag --test map_correction)
  else
    echo "== ros/drivers/aav_mppi_lag: MPPI with the lag, its critics, the closed-loop runs"
    (cd ros/drivers/aav_mppi_lag && cargo test -q --locked -p mppi-lag --features shared)
  fi
  echo "== ros/drivers/aav_mpc_lag: the lag-aware MPC in Rust, against the Python's vectors; closed loop"
  (cd ros/drivers/aav_mpc_lag && cargo test -q --locked -p mpc-lag)
  echo "== ros/drivers/aav_costmap: our own costmap's layers and hand-made scenes"
  (cd ros/drivers/aav_costmap && cargo test -q --locked -p costmap)
  echo "== ros/drivers/aav_planner: our own planner's curves and hand-made maps"
  (cd ros/drivers/aav_planner && cargo test -q --locked -p lattice-planner)
  echo "== ros/drivers/aav_navigator: our own navigator's goals against a pretend planner and follower"
  (cd ros/drivers/aav_navigator && cargo test -q --locked -p navigator)
  echo "== ros/drivers/aav_smoother: our own velocity smoother, for a car"
  (cd ros/drivers/aav_smoother && cargo test -q --locked -p car-smoother)
  # Only the logic: the window (raylib) needs a screen's libraries, and colcon builds it.
  echo "== ros/hud/aav_hud: the HUD's state, GPS, e-stop and formatting; the map's maths; the hardware panel's buttons"
  (cd ros/hud/aav_hud && cargo test -q --locked -p hud-core -p map-core -p debug-core -p panel-core)
  # Only the logic: the pictures need Bevy and a GPU (cargo test --release there).
  echo "== ros/sim/aav_sim_cameras: the simulator's cameras: lenses, camera_info, mounts, rates"
  (cd ros/sim/aav_sim_cameras && cargo test -q --locked -p camera-core)
  # The Rust API Guidelines' tools: rustfmt on every crate (the ROS nodes too: it needs no
  # build), clippy with warnings as errors on the crates above. The rclrs node crates need
  # ROS's generated messages, so clippy can't build them here.
  echo "== Rust style: cargo fmt --check everywhere, cargo clippy -D warnings on the logic"
  rust_style() {
    local dir=$1; shift
    (cd "$dir" && cargo fmt --all --check \
       && cargo clippy -q --locked --all-targets "$@" -- -D warnings) \
      || { echo "Rust style failed in $dir (cargo fmt --all; cargo clippy $*)"; return 1; }
  }
  rust_style firmware/spine
  rust_style ros/sources/aav_imu_yahboom -p yahboom-protocol
  rust_style ros/perception/aav_radar_clusters -p radar-clustering
  rust_style ros/ego/aav_ego_rs -p ego-core
  rust_style ros/perception/aav_lidar_rs -p lidar-scan
  rust_style ros/perception/aav_things_rs -p things-core
  rust_style ros/perception/aav_lane_finder_rs -p lane-core
  rust_style ros/perception/aav_lane_finder_learned -p ufld-core
  rust_style ros/sources/aav_camera_stream_rs -p stream-core
  rust_style ros/drivers/aav_behaviour_rs -p behaviour-core
  rust_style ros/safety/aav_watchdog_rs -p watchdog-core
  rust_style ros/safety/aav_self_test_rs -p self-test-core
  rust_style ros/io_mux/aav_io_mux_rs -p io-mux-core
  rust_style ros/can_bridge/aav_spine_bridge -p spine-bridge
  rust_style ros/can_bridge/aav_legacy_rs -p legacy-bridge
  rust_style ros/drivers/aav_rc_rs -p rc-core
  rust_style ros/safety/aav_joy_alive_rs -p joy-alive-core
  rust_style ros/safety/aav_safety_rs -p safety-core
  rust_style ros/localization/aav_gps_heading_rs -p gps-heading-core
  rust_style ros/command_mux/aav_mux_state -p mux-state-core
  rust_style ros/perception/aav_stop_sign_rs -p stop-sign-core
  rust_style ros/drivers/aav_drive_to_point_rs -p point-core
  rust_style ros/bringup/aav_description/car-model
  rust_style ros/bringup/aav_description/cloud-view
  rust_style ros/bringup/aav_description/map-chunks
  rust_style ros/bringup/aav_description/map-edit
  rust_style ros/bringup/aav_description/lanelet-map
  rust_style ros/safety/aav_stop_zone -p stop-zone
  rust_style ros/localization/aav_ndt -p ndt-core
  rust_style ros/localization/aav_pose_check -p pose-check-core
  rust_style ros/sim/aav_simulator -p sim-core --features shared
  rust_style ros/sim/aav_spine_sim -p bench-car
  rust_style ros/drivers/aav_mppi_lag -p mppi-lag --features shared
  rust_style ros/drivers/aav_mpc_lag -p mpc-lag
  rust_style ros/drivers/aav_costmap -p costmap
  rust_style ros/drivers/aav_planner -p lattice-planner
  rust_style ros/drivers/aav_navigator -p navigator
  rust_style ros/drivers/aav_smoother -p car-smoother
  rust_style ros/hud/aav_hud -p hud-core -p map-core -p debug-core -p panel-core
  rust_style ros/sim/aav_sim_cameras -p camera-core
  for board in firmware/spine/boards/*/; do
    (cd "$board" && cargo fmt --all --check) || { echo "cargo fmt --check failed in $board"; exit 1; }
  done
  echo "clean"
  # Every firmware crate must build without the standard library, for the Pico 2's chip.
  if [[ -d "$(rustc --print target-libdir --target thumbv8m.main-none-eabihf 2>/dev/null)" ]]; then
    echo "== firmware/spine, no_std, for the Pico 2 (thumbv8m.main-none-eabihf)"
    (cd firmware/spine && cargo build -q --target thumbv8m.main-none-eabihf -p spine-core \
       -p aav-inputs -p wheel-encoder -p sentinel -p wheel-core -p legacy-link -p jrk-g2 \
       -p ad5311 -p pca9555 -p mcp2515 -p ibus -p hd44780 -p spine-board -p adaptive-brake \
       && echo "builds")
    # The board programs: each a workspace of its own, built only for the Pico 2's chip.
    # Their UF2 files too, the ones to drag on: tools/elf2uf2.py checks the boot ROM's
    # IMAGE_DEF. pico2 is the spine; the other two replace the 2025-26 C++ Pico 2s.
    for board in pico2:spine-pico2 wheel-encoder-pico2:wheel-encoder-pico2 \
                 sentinel-pico2:sentinel-pico2; do
      dir=${board%%:*} bin=${board##*:}
      echo "== firmware/spine/boards/$dir: a board program, for the Pico 2"
      (cd "firmware/spine/boards/$dir" && cargo build -q --release --locked && echo "builds")
      python3 tools/elf2uf2.py \
        "firmware/spine/boards/$dir/target/thumbv8m.main-none-eabihf/release/$bin"
    done
  fi
  # Today's ESP32 control board: only with Espressif's Xtensa Rust (pixi run esp tools).
  echo "== firmware/spine/boards/esp32-umod: the spine on today's ESP32 control board"
  tools/esp32.sh check
  # The same firmware in Espressif's QEMU (pixi run esp qemu-tools): the heartbeat check.
  echo "== the ESP32 firmware in QEMU: it brakes when the Orin's heartbeat stops"
  tools/esp32.sh qemu-check
else
  echo "skipped: no cargo here (pixi run test has it; or install Rust with rustup)"
fi

echo "== the stand-ins stay on this computer"
# turtle, sim, bench and view must never meet a real car on the same network and domain.
python3 - <<'PY'
import re
import sys
tasks = dict(re.findall(r'^([\w-]+) = \{ cmd = "([^"]*)"', open('pixi.toml').read(), re.M))
bad = [t for t in ('turtle', 'sim', 'bench', 'bench-check', 'view', 'sim-check', 'sim-fault',
                   'demo', 'ravensim', 'ndt-map', 'carla-car', 'laptop-bench')
       if not tasks.get(t, '').startswith('tools/ros.sh --local ')]
# The families that start Ravensim or drive it: each choice's line in its script.
for script, choices in (('tools/screen.sh', ('hw-panel',)), ('tools/sim_cameras.sh', ('--ros',)),
                        ('tools/measure.sh', ('controllers', 'timing', 'stream', 'atob')),
                        ('tools/car_test.sh', ('sim',))):
    text = open(script).read()
    for choice in choices:
        line = re.search(rf'^ *{re.escape(choice)}\) .*$', text, re.M)
        if not line or 'tools/ros.sh --local' not in line.group(0):
            bad.append(f'{script} {choice}')
sys.exit(f'these pixi tasks need "tools/ros.sh --local": {bad}' if bad else None)
PY
if [[ -f install/setup.bash ]]; then
  got=$(env -u ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS=192.0.2.1 tools/ros.sh --local \
        bash -c 'echo "$ROS_AUTOMATIC_DISCOVERY_RANGE ${ROS_STATIC_PEERS:-no-peers}"')
  [[ "$got" == "LOCALHOST no-peers" ]] || { echo "tools/ros.sh --local gave: $got"; exit 1; }
  # As pixi runs it: its activation has already set SUBNET (15 Oct: --local wasn't local).
  got=$(ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET tools/ros.sh --local \
        bash -c 'echo "$ROS_AUTOMATIC_DISCOVERY_RANGE"')
  [[ "$got" == "LOCALHOST" ]] || { echo "tools/ros.sh --local kept SUBNET: $got"; exit 1; }
  got=$(AAV_DISCOVERY=SUBNET tools/ros.sh --local bash -c 'echo "$ROS_AUTOMATIC_DISCOVERY_RANGE"')
  [[ "$got" == "SUBNET" ]] || { echo "tools/ros.sh --local ignored AAV_DISCOVERY: $got"; exit 1; }
fi
# The keyboard with --sim or --local stays here too (a stand-in ros2 prints what it got).
stub=$(mktemp -d)
trap 'rm -rf "$stub"' EXIT
printf '#!/usr/bin/env bash\necho "${ROS_AUTOMATIC_DISCOVERY_RANGE:-unset} ${ROS_STATIC_PEERS:-no-peers}"\n' \
  > "$stub/ros2"
chmod +x "$stub/ros2"
for flag in --sim --local; do
  got=$(ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS=192.0.2.1 PATH="$stub:$PATH" \
        tools/keyboard.sh $flag)
  [[ "$got" == "LOCALHOST no-peers" ]] || { echo "tools/keyboard.sh $flag gave: $got"; exit 1; }
done
# The actuator tests and the preflight with --sim or --bench too; without them, the car's.
for flag in --sim --bench; do
  got=$(ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS=192.0.2.1 PATH="$stub:$PATH" \
        tools/actuator_test.sh preflight --stands $flag)
  [[ "$got" == "LOCALHOST no-peers" ]] || { echo "tools/actuator_test.sh $flag gave: $got"; exit 1; }
done
got=$(ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET PATH="$stub:$PATH" tools/actuator_test.sh preflight --stands)
[[ "$got" == SUBNET* ]] || { echo "tools/actuator_test.sh (the car) went local: $got"; exit 1; }
echo "they do"

echo "== the car kit's scripts (docs/car-install.md)"
for s in tools/car_kit.sh tools/car_install.sh tools/car_listen.sh tools/car_screen.sh \
         tools/car_screens_install.sh; do bash -n "$s"; done
# Away from a kit, the installer must stop at its checks, and say it changed nothing.
if out=$(bash tools/car_install.sh --dry-run --no-verify 2>&1); then
  echo "car_install.sh --dry-run passed with no kit: $out"; exit 1
fi
[[ "$out" == *"nothing was changed"* ]] || { echo "car_install.sh: $out"; exit 1; }
AAV_PREFIX=/nonexistent bash tools/car_listen.sh > /dev/null
echo "they parse, the installer refuses cleanly, and car_listen runs"

echo "== vendor/ stays out of the build"
# Without COLCON_IGNORE, pixi run build would try to build all of ROS 2's source.
[[ -f vendor/COLCON_IGNORE ]] || { echo "vendor/COLCON_IGNORE is missing"; exit 1; }
python3 -c "import glob, yaml; [yaml.safe_load(open(f))['repositories'] for f in glob.glob('vendor/*.repos')]"
echo "it does"

echo "== the MCP server, over stdio, as an AI app starts it"
if [[ -d .pixi/envs/mcp ]] && command -v pixi >/dev/null; then
  pixi run --frozen -e mcp python3 tools/tests/mcp_smoke.py
else
  echo "skipped: the mcp environment isn't installed (pixi install -e mcp)"
fi

echo "== docs/interfaces.md"
python3 tools/gen_interfaces_md.py --check && echo "in sync"

echo "== docs/codemap.md"
python3 tools/codemap.py --check && echo "in sync"
