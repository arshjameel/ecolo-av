# Lane followers, as they were (removed 19 Nov)

The lane followers steered the car by the camera's lines alone, on twist_mux's AI slot
(`/drivers/ai/cmd_vel`, priority 10). **Removed on 19 Nov** (TODO 264), by decision.
Ben, 19 Nov: "drop there code all together. We use the perception system to identify the
lines and then we bake it into the cost map. So curb, high cost, yllow line med cost,
lright side low cost, car in the way high cost, person in the way high cost, etc then have
the nav system handle it".

The code is in git tag **`save/lane-followers`** (`git checkout save/lane-followers`).

```
before:  lane_finder ─► /perception/lane/geometry ─► control_interface ─► /drivers/ai/cmd_vel
now:     lane_finder ─► /perception/lane/geometry ─► costmap (lanes:=camera) ─► planner ─► mppi_lag
```

## What went

| What | Where it was | Who |
|---|---|---|
| `control_interface`, the 2025-26 PID and its five states (keep, change, committed, stopping, resuming) | `ros/perception/aav_lane_geometry/scripts/` (`control_interface.py`, `lane_follow.py`, `lane_speed.py`) | the 2025-26 team; ported logic Ben |
| Its Rust port, the default since that morning | `ros/drivers/aav_lane_follower_rs` (`lane-follow-core`, `lane-follow-node`) | Ben |
| The challengers: pure pursuit (Coulter 1992) and Stanley (Thrun et al. 2006), each steering from where the car will be after its lag | `aav_lane_follower_rs` | Ben |
| The fake lane (`lane:=fake`) | `aav_lane_geometry/scripts/fake_lane_geometry_pub.py` | the 2025-26 team |
| The switches `lane_follower:=`, `lane_follower_impl:=`, `lane:=fake` | sim, bench and car launch files | |
| The checks `sim-check --lane`, `bench-check --lane`, `measure lane` | `tools/` | Ben |

**What stays:** the lane finder (both twins), the `LaneGeometry` message (now with each
line's paint), `aav_lane_pose`, `tools/lane_score.py`.

## Their scores (19 Nov, TODO 262)

`pixi run measure lane`, Ravensim on Ben's Mac (GPU), 19 Nov, **3 runs each**, round robin,
each on its own ROS domain. Mean over the runs (range in brackets). RMS is the distance
from the lane's centre by the simulator's truth; "settled" is the same from 15 s on, once
the start offset is gone; flips are the turn changing side (past 0.02 rad/s). The lane was
never lost for more than 0.3 s, and no car left the road.

**Road world, straight, from 0.3 m off, 60 s at 1 m/s**

| Follower | RMS cm | Settled RMS cm | Flips a minute |
|---|---|---|---|
| PID, Python | 6.7 (6.7-6.8) | 1.3 (1.1-1.6) | 165 |
| PID, Rust | 6.7 (6.6-7.0) | 1.3 (0.5-2.6) | 137 |
| pure pursuit | 8.4 (8.4-8.5) | 0.9 (0.6-1.2) | **0** |
| Stanley | 7.7 (7.7-7.7) | **0.3** (0.3-0.3) | **0** |

**Road world, from 0.3 m off, 45 s at 1.9 m/s** (`--speed 2`)

| Follower | RMS cm | Settled RMS cm | Flips a minute |
|---|---|---|---|
| PID, Python | 7.1 (6.9-7.2) | 2.1 (1.6-2.4) | 215 |
| PID, Rust | 7.0 (7.0-7.0) | 1.6 (1.5-1.7) | 208 |
| pure pursuit | 8.2 (8.2-8.2) | **0.4** (0.4-0.5) | **0** |
| Stanley | 7.7 (7.7-7.7) | **0.4** (0.3-0.5) | **0** |

**The campus, Library Road (bends, a dashed centre line), 90 s at 1.5 m/s**

| Follower | RMS cm | Max cm | Lost s | Flips a minute |
|---|---|---|---|---|
| PID, Python | 4.1 (4.0-4.2) | 16 (16-17) | 0 | 187 |
| PID, Rust | 3.9 (3.9-4.0) | 14 (12-15) | 0 | 188 |
| pure pursuit | **2.4** (2.1-2.7) | **8** (8-9) | 0.1 (0-0.3) | **1** |
| Stanley | 3.0 (2.9-3.1) | 10 (9-11) | 0 | 6 |

What it says:

- **The Rust PID is the Python PID**: the same scores, within the runs' spread.
- **The PID weaves**: its turn changes side about 3 times a second. It is the quickest
  back from 0.3 m off (the road's RMS), but never settles.
- **Pure pursuit** is the best on the campus's bends (40 % less error, half the worst
  case) and barely moves the wheel; **Stanley** holds a straight best. Both come back
  from 0.3 m off more slowly than the PID (a 4 m look-ahead at 1 m/s), so their road RMS
  is worse.

**The default stays `pid`**: the challengers don't win everywhere (the start offset), so
by the league's rule they stay challengers. Pure pursuit is the one to promote once it
comes back from an offset as fast (a shorter look-ahead near the car, TODO 262).

## Earlier lane drives (8 Nov, the 2025-26 PID)

| Run (Ben's Mac, quiet) | Ravensim | Gazebo |
|---|---|---|
| `sim-check --lane-only` (road) | back within 5 cm in 5.4 s, overshoot 3 cm; finder 1.0 cm from the truth; both lines in 299 of 299 | 5.1 s, overshoot 5 cm; finder 1.1 cm; 301 of 301 |
| `measure lane`, road, 60 s from 0.3 m off | 60 m at 0.99 m/s; error mean 3.0 cm, 95 % 19.5 cm (the start); lane never lost | 60 m at 0.99 m/s; mean 3.0 cm, 95 % 19.7 cm; never lost |
| `measure lane`, Library Road, 120 s | 171 m at 1.4 m/s; error mean 3.9-4.1 cm, 95 % 13-15 cm, max 25-29 cm; lost in 0-1 % of pictures | 60 m at 0.5 m/s; mean 197 cm: the car ended in the other lane; lost in 68 % of pictures |
| `bench-check --lane` (11 Oct, 6 runs) | back from 0.30 m to within 5 cm in 5.6-6.7 s, 2 cm past the centre at most; finder 0.8-1.2 cm out | |

## What replaces them

The lanes layer from the camera ([our costmap](../own-costmap.md#the-lanes-from-the-camera-19-nov-todo-264)):
the lines priced in the global costmap, the planner keeping right and crossing a line only
when it pays. Stage 2 scores it on the same roads against these numbers (PID 3.9 cm,
pure pursuit 2.4 cm RMS on Library Road).
