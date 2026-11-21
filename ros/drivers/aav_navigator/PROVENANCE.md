# Provenance

The `navigator` crate follows what Nav2's `bt_navigator`, our behaviour trees
(`aav_navigation/behavior_trees/*_car*.xml`, themselves Nav2's trees changed for a car)
and `waypoint_follower` do for us. It is not a behaviour-tree engine: the trees' logic is
rewritten as one plain state machine in Rust's own style (enums, no plugins, no ROS).

## Where it came from

| | |
|---|---|
| Repo | https://github.com/ros-planning/navigation2 |
| Packages | `nav2_bt_navigator`, `nav2_behavior_tree`, `nav2_waypoint_follower` |
| Commit | the Jazzy commit in `vendor/ros2-src.repos` (645abd95, as `aav_costmap`'s and `aav_planner`'s) |
| Licence | Apache-2.0 (Samsung Research America, Steve Macenski and the Nav2 authors) |
| Read 8 Nov 2026 | our four trees; `nav2_behavior_tree`'s `RecoveryNode`, `PipelineSequence`, `RateController`, `RoundRobin`, `RemovePassedGoals`, `WouldAPlannerRecoveryHelp`, `WouldAControllerRecoveryHelp`; `nav2_msgs`' action error codes; `waypoint_follower.cpp` (`stop_on_failure`, the missed waypoints) |
| Datum | `aav_hud`'s `map-core` `earth.rs` (ours): navsat_transform's UTM steps |

## What was taken

| Ours | Nav2's |
|---|---|
| plan every `replan_period` (1 s; through-poses 3 s) | `RateController hz="1.0"` / `"0.333"` |
| keep the path within `final_approach` of the goal, while it ends there and is under 20 s old | our trees' `OnTheFinalApproach` in `PlanOnlyWhenNeeded` (TODO 17). Not yet ported: the trees' `KeepAValidPath` (14 Nov, TODO 149), so ours still plans every second away from the goal |
| a failed plan: clear the global costmap, plan once more | the inner `RecoveryNode` with `WouldAPlannerRecoveryHelp` |
| a failed follow: clear the local costmap, follow once more | the inner `RecoveryNode` with `WouldAControllerRecoveryHelp` |
| then, 6 times at most: clear both, wait 5 s, back up 1 m (reversing), in turn | the outer `RecoveryNode` and `RoundRobin` |
| which error codes a recovery can fix | `WouldA*RecoveryHelp`'s lists |
| drop passed through-poses (1 m) | `RemovePassedGoals` |
| waypoints one by one, missed ones listed, `stop_on_failure` | `waypoint_follower` (`WaypointMode::StopAtEach`) |
| lat/lon to the map | `follow_gps_waypoints` calls `/fromLL`; we do its maths with the datum |

## What changed, and why

| Change | Why |
|---|---|
| A failed replan while the follower drives a good path doesn't stop the car; only 4 in a row do | Nav2's `PipelineSequence` halts FollowPath at the first failed replan: a stop for nothing |
| Through-poses also drops a pose once the car has passed it along the path | a car cutting a bend by more than 1 m never came back for it otherwise |
| Waypoints run as one through-poses goal by default (`pass_through`) | TODO 140: each waypoint a stop-and-face goal left the car half-turned at a sharp bend |
| The final approach also on a through-poses goal's last leg | the same circling near the goal (TODO 17) |
| A new goal on any action replaces the running one | Nav2 refuses a goal on a second navigator while one runs; one navigator is simpler for the screens and tools |
| "Start occupied" (205, 305) counts as a failure a recovery can fix | campus, 8 Nov: lidar marks beside a lot exit made the car's own spot occupied, and Nav2 gave the goal up at once; clearing the global costmap forgets the stale marks |
| `IsPathValid` on the final approach (14 Nov): once a second, the planner's service (planner_server's, or ours where none runs); a blocked kept path is planned again. No answer changes nothing | as the trees; until 14 Nov ours had none (the navigator reads no costmap) |
| Spin, DriveOnHeading, `GoalUpdated` within a recovery | not used by our trees, or covered by "a new goal replaces" |
| Time limits on every call (14 Nov): a plan unanswered in `plan_timeout` (10 s) fails as TIMEOUT (207/307); a follower silent for `follow_timeout` (4 s: no feedback, no result) as CONTROLLER_TIMED_OUT (107), which ends the goal; a clear unanswered in `recovery_timeout` (5 s) as failed | the gap list (14 Nov): a planner or follower that died mid-goal hung it for ever. Nav2 gets the same from its lifecycle bond (4 s) and the trees' server timeouts |
| A planner-recovery clear that fails ends the attempt with the plan's own error code, not 200 | Nav2's `RecoveryNode` leaves the plan's code on the blackboard |
| Error codes are enums (`PlanError`, `FollowError`, `Abort`), Nav2's numbers only at the edge | the Rust API Guidelines |
| 14 Nov: map -> odom or odom -> base_link over 1 s old is no car pose (`TRANSFORM_TIMEOUT`): no waypoint passed, no path kept, no progress | Nav2 looks the pose up through tf2 with `transform_tolerance`; ours kept the last pose for ever when `/tf` stopped |
| 14 Nov: with no `backup_action` (no behaviour server: all ours), BackUp is the follower's FollowPath on a straight line 1 m behind the car (`path::straight_back`) | all ours had no BackUp at all; `mppi_lag` now reverses and checks the costmap as it backs up, as behavior_server's BackUp checks the footprint |
| 14 Nov: a goal in `odom` or `base_link` is put on the map (through map -> odom, or the car's pose; the latest, under 1 s old); another frame is refused (INVALID_GOAL) | Nav2 transforms a goal from any frame tf2 knows; ours refused all but `map`. Ours uses the latest transforms, not those at the goal's stamp |
| 15 Nov: a to-pose path is kept up to `keep_path_age` (10 s) while it ends at the goal and IsPathValid says it is clear, then planned again every `replan_period`; through-poses still plan every period | the trees' KeepAValidPath (PathExpiringTimer 10 s, IsPathValid). Ours planned every second: each plan a little different, the car swayed twice as much (TODO 168) |
| 15 Nov (TODO 176): the Rust API Guidelines pass: `#[must_use]` on pure functions, doc comments, `Debug`, and the lints that keep them; no change in behaviour | Ben, 10 Oct: "Not just c code in the rust language" |
| 18 Nov (TODO 240): a FollowPath goal's feedback counts as word from the follower from the moment it is asked for, not only once the follower's answer ("accepted") has come; feedback before the answer shows it took the goal, and its result is then asked for by the goal's id (`call.rs`). A never-answered goal is logged | CI's nightly sim, 18 Nov: "no word from the follower in 4.0 s" while `mppi_lag` drove the car at 1 m/s; the silence began at the 10 s path renewal, when the follower moved its feedback to the new goal, whose feedback ours read only after the answer. |
