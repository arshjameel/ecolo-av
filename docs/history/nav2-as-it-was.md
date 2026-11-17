# Nav2, as it was

Nav2 drove our car in the simulator from 10 Oct to 16 Nov 2026. On 16 Nov Ben said
delete it "Now" (TODO 184), with the stop zone "shadow run, then swap". It went on
17 Nov. This page keeps what it was, what took each part's place, and its last scores.

## What ran

```text
  /goal_pose ─► bt_navigator ──► planner_server ──► controller_server ─► velocity_smoother
               (behaviour trees)  (Smac lattice,     (MPPI or RPP)        │
                                   Hybrid-A*...)                          ▼
                    global_costmap        local_costmap            collision_monitor
                    map_server            behavior_server          ─► /drivers/nav/cmd_vel
```

| Nav2 part | What it did | Ours in its place | Since |
|---|---|---|---|
| `bt_navigator`, `waypoint_follower` | the goal: plan, follow, replan, recover; waypoints | `aav_navigator` (Rust) | 14 Nov default |
| `planner_server` (SmacPlannerLattice, Hybrid, Smac2D, NavFn, Theta*) | the path | `aav_planner` (Rust: the lattice and Hybrid-A*) | 14 Nov default |
| `controller_server` (MPPI, Regulated Pure Pursuit) | follow the path | `aav_mppi_lag` (Rust: MPPI with the actuators' lag) | 14 Nov default |
| `local_costmap`, `global_costmap` | what is in the way | `aav_costmap` (Rust) | 14 Nov default |
| `velocity_smoother` | smooth the command | `aav_smoother` (`car_smoother`, under the same node name) | 17 Nov |
| `collision_monitor` | the stop zone: zero the command for points ahead | `aav_stop_zone` (Rust) | 17 Nov, after its shadow run |
| `map_server` | `/map`, latched | `aav_costmap`'s `map_server` | 17 Nov |
| `behavior_server` | back up, spin, wait | the navigator's own recoveries | 14 Nov |
| `lifecycle_manager_navigation` | start the servers in order | none needed: our nodes start themselves | 17 Nov |

## What stayed

| Piece | Why |
|---|---|
| `nav2_msgs` | the message types: our nodes serve Nav2's action names (`/navigate_to_pose`, `/follow_path`...), so no goal sender changed |
| `nav2_amcl` and its lifecycle manager | the AMCL trial (`amcl:=true`), a localizer scored beside the EKFs; no port yet (TODO 231) |
| `nav2.yaml` (a smaller one) | the blocks our ports read, in Nav2's layout: the costmaps, MPPI, the lattice, the smoother, the stop zone |
| Nav2's source in `vendor/` | to read; `tools/lattice_primitives.py` runs its lattice generator from there |

## The swap of the stop zone

The stop zone is a safety lock, so it ran in shadow first: ours on its own topics beside
Nav2's `collision_monitor`, driving nothing, on every obstacle check.

Three rounds (17 Nov, our simulator, `sim-check --obstacle --lidar-obstacle --kerb --moving
--behaviour-things --faults --with stop_zone_shadow:=true`). Each node logs its decision on
the command it decides on; for each of Nav2's stops, was ours stopped on the same command?

| Check | Runs | Nav2's stops | Ours stopped too | Ours later |
|---|---|---|---|---|
| obstacle (the radar) | 3 | 2 | 2 | never |
| lidar obstacle | 3 | 6 | 6 | never |
| kerb (lidar) | 3 | 4 | 4 | never |
| kerb (lidar and radar) | 3 | 4 | 4 | never |
| moving thing | 3 | 3 | 3 | never |
| parked box | 3 | 2 | 2 | never |
| faults (the radar silent) | 3 | 3 | 3 | never |
| **All** | 21 | **24** | **24** | **never** |

Ours also stopped alone, for up to 3.7 s: at start-up, before Nav2's had been switched on
(a silent sensor stops ours at once). Only extra caution.

!!! note "Two clocks"
    sim-check also timed each node's state message as it arrived (`AAV_SHADOW_LOG`). On a
    busy computer (load 10-34) ours' messages sometimes came later: up to 1.95 s with the
    car standing (0 cm), and 0.16 s (8 cm) once in a fault run. The nodes' own logs show ours
    decided on the same command each time, 13 µs sooner in that fault run: the delay was
    the message, not the decision.

So ours took the lock (TODO 182): `stop_zone:=` in every stack, on the bench and the car.

## The last scores

Nav2's last whole-stack scores, against ours, are in
[the league's scores, 16 Nov](../league-results-2026-11-16.md). In short:

| Level | Nav2 | Ours |
|---|---|---|
| costmap | went over the centre line | `ours_chunks`: the same drive, a third of the memory |
| navigator | a tie | ours stayed |
| steer | Nav2 MPPI failed or swayed; RPP swung | `mppi_lag_calm` |
| smoother | a tie (the champion until 17 Nov) | ours |

## How to see it again

`git log` before the 17 Nov merge has every launch file and test. `vendor/ros2-src/`
keeps Nav2's source (`pixi run get vendor`).
