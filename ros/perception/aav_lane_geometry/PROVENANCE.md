# Provenance

Copied on 2026-10-09 from github.com/AAV-2025-2026/Lane_Detection_ROS2 @ main (345285860e), src/lane_geometry.

Files, unchanged unless listed under Changes:

- `CMakeLists.txt`
- `msg/LaneGeometry.msg`
- `package.xml`
- `scripts/control_interface.py`
- `scripts/fake_lane_geometry_pub.py`

## Changes

- 2026-10-10: `CMakeLists.txt` `cmake_minimum_required` 3.8 -> 3.10;
  `scripts/control_interface.py` `.warn(` -> `.warning(` (deprecated in Jazzy).
- 2026-10-10: brought in from parked as `aav_lane_geometry` (the name in
  docs/conventions.md). The package, its CMake project and its message's package are
  renamed, and the scripts import `aav_lane_geometry.msg`.
  - `scripts/control_interface.py` publishes `TwistStamped` on `/cmd_vel/ai`
    (twist_mux's slot for a learned driver), through a new `send()`, instead of `Twist` on
    `/control_cmd`. The lane-keeping logic is unchanged.
  - `launch/control_test.launch.py` (since removed): the package name follows.
  - New: `launch/lane_follower.launch.py` (the driver, with fake:=true for a pretend lane).
  - `scripts/control_interface.py`, `scripts/fake_lane_geometry_pub.py`: a quiet Ctrl-C.
    They ignore the second Ctrl-C while cleaning up, catch `ExternalShutdownException`, and
    call `rclpy.try_shutdown()` instead of `rclpy.shutdown()`, which raised when Jazzy's own
    handler had already shut down.
- 2026-10-11: `launch/lane_follower.launch.py`: its docstring names the lane finder
  (`aav_lane_finder`, ours) as what publishes `/lane_geometry`, and `fake:=true` as the
  pretend lane without a camera. The code is unchanged.

## Topic names by area (2026-10-15)

The topics move to their areas (docs/naming.md, Decided by Ben on 15 Oct). Only names
change, never the logic:

- `scripts/control_interface.py`: `/lane_geometry` -> `/perception/lane/geometry`,
  `/cmd_vel/ai` -> `/drivers/ai/cmd_vel`, `/lane_geometry_status` ->
  `/drivers/ai/lane_status`, `/vehicle_state` -> `/drivers/ai/lane_state` (control_interface
  is the AI slot's driver, so what it publishes goes under `/drivers/ai/`).
- `scripts/fake_lane_geometry_pub.py`, `scripts/geometry_logger.py` (since removed),
  `launch/lane_follower.launch.py`: `/lane_geometry` -> `/perception/lane/geometry`.
- 25 Oct 2026: `launch/control_test.launch.py` starts both scripts with
  `prefix='python3'`, as `lane_follower.launch.py` does (on a Mac, `/usr/bin/env` drops
  the library path their messages need: `docs/mac.md`). `scripts/geometry_logger.py`
  calls `rclpy.try_shutdown()`, as the other two scripts do, so a second Ctrl-C under
  launch doesn't raise.
- 26 Oct 2026: removed `launch/control_test.launch.py` and `scripts/geometry_logger.py`
  (and its line in `CMakeLists.txt`): 2025-26 test aids that nothing started.
  `lane_follower.launch.py fake:=true` does what the test launch did. The naming and
  cleanup audit, `docs/designs/2026-10-26_naming-and-cleanup.md`.
- 26 Oct 2026: `scripts/control_interface.py`: no lane message for `stop_wait_s` now slows
  the car at `decel_rate`, as lost lines do, instead of one zero command after a fixed
  2 s (`docs/designs/2026-10-25_DRIFT_REPORT.md`, item 7). `stop_wait_s` was read but
  never used: it is now that wait, 0.3 s (was 2.0), below twist_mux's 0.5 s for the AI
  slot. The slow-down moved from the message callback to the 0.1 s watchdog timer, with
  the time between ticks, not a fixed 0.1 s a message. The watchdog's times are on the
  node's clock, not `time.time()`. New: `scripts/lane_speed.py` (the rule, no ROS),
  `test/test_lane_speed.py`; `CMakeLists.txt` installs the module and runs the test;
  `package.xml` adds the test dependencies.
- 19 Nov 2026: the Rust twin, `ros/drivers/aav_lane_follower_rs` (Ben: "We also need to port
  the lane following code"), the default from now on.
  - `scripts/control_interface.py`'s logic moved, unchanged, into a new
    `scripts/lane_follow.py` (no ROS): the `PID` and the five states, with the two clocks
    passed in (the wall clock for the PID and the states' times, the node's for the
    watchdog) and what to publish handed back. `control_interface.py` is now the ROS around
    it: the same node name, topics, QoS, parameters, log lines and publishing order. Its
    module docstring lists the topics. numpy is no longer imported (`np.clip` became
    `lane_follow.clip`, NaN kept).
  - `launch/lane_follower.launch.py`: `lane_follower_impl:=rust|python` and
    `lane_follower:=pid|pure_pursuit|stanley` (the Rust challengers' wheelbase, full lock
    and lag from `vehicle.yaml`).
  - `test/test_lane_speed.py` reads `stop_wait_s`'s default from `lane_follow.Settings`;
    new `test/test_lane_follow.py`; `CMakeLists.txt` installs `lane_follow.py` and runs the
    new test.
- 19 Nov 2026: `msg/LaneGeometry.msg` gains `left_paint`, `right_paint` and the
  `PAINT_UNKNOWN`, `PAINT_WHITE`, `PAINT_YELLOW` constants (TODO 264): each line's paint,
  so the costmap prices a yellow centre line apart from a white edge line. The old fields
  are unchanged.
- 19 Nov 2026: **the 2025-26 `control_interface` removed, by decision** (TODO 264; Ben, 19
  Nov: "drop there code all together. We use the perception system to identify the lines
  and then we bake it into the cost map"). Deleted: `scripts/control_interface.py`,
  `scripts/lane_follow.py`, `scripts/lane_speed.py`, `scripts/fake_lane_geometry_pub.py`
  (the 2025-26 fake lane, which only fed it), `launch/lane_follower.launch.py`, `test/`;
  and the Rust twin `ros/drivers/aav_lane_follower_rs`. They are in git tag
  `save/lane-followers`; their scores in `docs/history/lane-followers.md`.
  `CMakeLists.txt` and `package.xml` now build the message only (the 2025-26 maintainer
  line replaced by the team's).
