# Driving from A to B: speed up, drive, slow down, stop

Written 15 October 2026, checked 25 October. The goal: the car drives in a straight line from point
A to point B, speeding up, holding a speed, slowing down and stopping. This is the KEFC
request's "accelerate / hold / stop" demo, and the first step to GPS point-to-point.

> **The short version**
> - There are **three ways**. All three exist now: the third, the point driver (`aav_drive_to_point`), was built on 15 Oct for the main goal. Until 26 Oct it was called `aav_goto` (`pixi run goto`, action `/goto`).
> - **`pixi run drive-to-point --lat .. --lon ..`** drives to B and stops there, slowing at **half the measured braking**.
> - A straight drive in an empty lot needs **no costmap and no map**: only "where am I" and "where is B".
> - The hard part isn't the driving. It's **knowing where the car is and which way it faces** at the start.
> - Test it on the turtle, then the bench, then Gazebo, then the car on stands, then the car.
> - **`pixi run measure atob`** scores a recorded run: speeding up, holding, the stop on B ([Scoring a run](#scoring-a-run)).

## 1. The three ways

| | 1. Scripted drive | 2. Nav2 to a GPS point | 3. The point driver (`aav_drive_to_point`) |
|---|---|---|---|
| Idea | drive a speed profile: speed up, hold for a distance, stop | give Nav2 point B as latitude/longitude; it plans and steers | drive the straight line from A to B, holding the line, and stop at B |
| Needs GPS? | no | yes | yes |
| Steers to stay on the line? | **no**: wheels straight, it drifts | yes | yes |
| Stops exactly at B? | no: after a distance from the start | yes, within Nav2's goal tolerance | yes |
| Speed profile you choose? | **yes**: acceleration, speed, deceleration | roughly: Nav2's limits (1 m/s, 1 m/s²) | **yes** |
| Avoids obstacles? | no | **yes** (costmap, collision_monitor) | no: an open lot, a safety driver |
| Exists today? | ✅ `aav_scripted` | ✅ Nav2 + `navsat_transform` | ✅ `aav_drive_to_point` (15 Oct) |
| Tested | turtle, bench | Gazebo (goals of 8 m and 9.5 m), bench (40 m) | Gazebo: 10-60 m, any angle, 1 and 3 m/s ([section 4](#4-way-3-the-point-driver)) |
| Best for | the first drive on the car | point-to-point with obstacles | **the main goal**: GPS A to B with a smooth stop |

**Recommendation:** start with **1** on the car on stands, then on the ground. Then **3**
for the GPS A-to-B demo (tests.md 5.2). Use **2** where there are obstacles.

## 2. Way 1: the scripted drive (today)

The `scripted_driver` drives a speed profile. It knows how far it has gone from
`/localization/odom/local` (wheels + IMU), not from GPS.

```text
 speed
   ▲      hold
   │    ┌───────┐
   │   /         \
   │  / speed up  \ slow down
   └─/─────────────\────► distance
     A               stop (about B)
```

```bash
ros2 action send_goal /scripted_drive aav_interfaces/action/ScriptedDrive \
  "{target_speed: 1.0, acceleration: 0.5, deceleration: 0.5, hold_distance: 10.0, curvature: 0.0}" \
  --feedback
```

| Field | Means |
|---|---|
| `target_speed` | m/s to hold (negative = reverse) |
| `acceleration`, `deceleration` | m/s², how fast to speed up and slow down |
| `hold_distance` | metres at full speed; 0 = use `hold_time` (seconds) instead |
| `curvature` | 0 = straight. It sets a fixed steering angle; it doesn't correct |

- Any person driving (keyboard, joystick) overrides it at once. Cancel the goal and it slows down and stops.
- **Limits:** with wheels straight but no correction, a slightly-off steering centre makes it curve. Until the KTC-100 gives real steering feedback, expect drift.
- **Or in plain English:** `pixi run -e llm talk`, then type "drive 10 m forward". A local model plans the goal (the whole 10 m, ramps included), shows it, and drives only after you type yes. Stand-ins only for now: [docs/plain-english.md](plain-english.md).

## 3. Way 2: Nav2 to a GPS point (today)

Nav2 plans a path from where the car is to B and steers along it.

```text
 B (lat, lon) ─► /fromLL ─► B (x, y on the map) ─► Nav2 plans ─► Nav2 steers ─► stops at B
```

```bash
ros2 action send_goal /follow_gps_waypoints nav2_msgs/action/FollowGPSWaypoints \
  "{gps_poses: [{position: {latitude: 45.38612, longitude: -75.69588, altitude: 0.0},
                 orientation: {w: 1.0}}]}" --feedback
```

- `street_router` ([Street routes](routes.md)) and the language tools use this same action.
- On an **empty map** (the default, 200 m × 200 m, all free), Nav2's plan is close to a straight line; the planner only respects the car's turning circle.
- Nav2's speed: up to 1.0 m/s, speeding up at up to 1.0 m/s², slowing near the goal (`nav2.yaml`).
- The radar and the roof lidar still fill the local costmap, and collision_monitor still stops for something ahead.

## 4. Way 3: the point driver

`aav_drive_to_point` drives to one point B and stops there. B is a GPS point (lat/lon) or a map point.

```text
 pixi run drive-to-point ──► /drive_to_point action ──► point_driver ──► /drivers/point/cmd_vel ──► twist_mux (25) ──► ego
   lat/lon ──► /fromLL (navsat_transform) ──► B on the map
   /localization/odom/global (wheels + IMU + GPS) ──► where the car is, how fast
```

### How it drives

| Part | Does | File |
|---|---|---|
| Speed | speed up gently (0.5 m/s²), hold the cruise speed, then slow down **by the distance left**: v = √(2·a·d), with a = **half of `max_brake`** | `approach.py` |
| Steering | turn towards B first if it is more than 20° off the nose; then pure pursuit along the straight line to B | `line.py` |
| One drive | the two together, tick by tick (50 Hz) | `drive.py` |
| Rules | when it may start, when it must stop, how close is close (1 m with RTK, 2 m without) | `rules.py` |

```text
 speed
   ▲          hold (cruise)
   │      ┌─────────────┐
   │     /               \   v = √(2 a d): slowing at a = ½ max_brake
   │    / up 0.5 m/s²     \  stops exactly at B
   └───/───────────────────\──► distance left to B
       A                    B
```

Slowing by the distance left, not by the clock, corrects itself: if the car slows a little
late, the distance shrinks faster and the command drops to match.

### Why a line driver, and not Nav2 or the scripted drive

| Question | Line driver (point) | Nav2 | Scripted |
|---|---|---|---|
| Stops **at B**, wherever it started? | yes: speed from the distance left | yes, within its goal tolerance | no: after a distance |
| Slows at **half the measured braking**? | yes: one number from `vehicle.yaml` | no: its controller and smoother have their own limits | yes, but not ending at B |
| Path | a straight line: easy to predict and to explain | a planned path through a costmap | none: wheels straight |
| Parts that must work | the global EKF and the GPS | maps, costmaps, planner, controller, behaviour tree | the local EKF |
| Obstacles | ❌ none: open lot, safety driver | ✅ | ❌ |

So: **the line driver for the open-lot demo, Nav2 for anything with obstacles.**
`speed_profile.py` (the scripted drive's maths) doesn't fit: it slows at a fixed rate from
wherever it is, so the stop doesn't land on B. `approach.py` explains it at the top.

### Its safety

| It | How |
|---|---|
| Has its own twist_mux slot | priority **25**: below every person, test and script; above Nav2; held by the stop-sign and watchdog locks (30) |
| Stops at once | Ctrl-C (cancels the goal), the e-stop, the watchdog's lock **or its silence**, a stale position (0.5 s) or GPS fix (2 s), someone else driving, or the car not moving though asked |
| Refuses | a B over **200 m** away, no position, a B inside the car's tightest turn (it can't reverse) |
| Obeys ego | asks ego for its caps (1 m/s on the car) and plans with them |
| Asks first on the real car | `/system/source` REAL (or nobody announcing): a checklist, then you type `drive`, as in the [actuator tests](actuator-tests.md) |

### Run it

```bash
pixi run sim                                               # Gazebo, in one terminal
pixi run drive-to-point --sim --ahead 20                   # 20 m ahead of the car
pixi run drive-to-point --sim --ahead 30 --left 10         # ... and 10 m to its left
pixi run drive-to-point --sim --x 30 --y 5                 # a map point (x east, y north, metres)
pixi run drive-to-point --lat 45.38630 --lon -75.69580     # the real car: a GPS point; it asks first
pixi run sim-check --drive-to-point-only                   # the Gazebo proof below (8 minutes)
```

Or the action itself: `ros2 action send_goal /drive_to_point aav_interfaces/action/DriveToPoint "{x: 20.0, y: 0.0}" --feedback`.

### Proven in Gazebo (15 Oct)

`pixi run sim-check --drive-to-point-only`, Gazebo's truth as the judge:

| Check | Result |
|---|---|
| Targets 10, 30 and 60 m away, at any angle | every stop within 0.4 m of B (asked: 2 m) |
| At 1 and 3 m/s | no overshoot |
| Braking | 1.4-1.6 m/s², for a plan of 1.5 (half `max_brake`) |
| A cancel, the e-stop, a GPS fault | each one stopped it |

Untested on the bench and on the car: the car needs the brake test (4.3) for `max_brake` first.

### Scoring a run

`pixi run measure atob` turns one recorded run into numbers: how well the car sped up, held
its speed, and stopped on B. The same numbers for Gazebo and for the car.

```bash
pixi run measure atob bags/<run>                              # B from the point driver's feedback
pixi run measure atob bags/<run> --b-latlon 45.38630 -75.69580  # or give B (or --b-xy X Y)
pixi run measure atob bags/<run> --taped -0.12 0.05           # the car: the tape's stop error
pixi run measure atob run1.csv run2.csv --json                # CSV runs; JSON out
pixi run measure atob bags/<run> --save-csv run.csv           # the run as CSV, to share
```

The stop error is measured on the line from A to B:

```text
                         + left
   A ───────────────────── B ──► + past B
                         - right
          - short of B
```

| Number | Means | Unit |
|---|---|---|
| Asked to go, to 90 % of the set speed | the rise time | s |
| Holding: speed error, mean | measured − set, while holding; + too fast | m/s |
| Speed overshoot | how far the top speed went over the set speed | % |
| Hardest slowing | the strongest braking (speed smoothed over 0.5 s) | m/s² |
| Largest jerk | how suddenly the braking or speeding up changed | m/s³ |
| First slowing to stopped | from the last moment at 90 % of the set speed to standing still | s |
| Rolled after the command hit 0 | the brakes' share of the stop | m |
| The last metre took; crept in it | a slow creep up to B shows here (under 0.15 m/s) | s; m |
| Stop: past B, left of the line | where it stopped, by the EKF, the truth (Gazebo) or the tape (car) | m |
| Furthest past B | the worst moment, even if it rolled back | m |
| GPS part | real (tape or truth) − the EKF's view | m |

The speeds come from Gazebo's truth in the sim, from the wheels (`/vehicle/odom`) on the
car. `tools/atob_score.py`'s header says which topic gives each number.

### Why tape the stop on the car

The EKF's stop error says how well the **controller** did: the car stopped where it
*thought* B was. It can't see its own GPS error.

| Error | Seen by | Fix it with |
|---|---|---|
| Control: the car didn't stop where it meant to | the EKF's view | the speed loop, `brake_lead`, the deceleration |
| GPS: the car isn't where it thinks it is | the tape minus the EKF (the "GPS part") | RTK, a better B mark, the antenna offset |

So at the car: mark B on the ground, drive, and tape the stop from the mark (along the
line, then across it). `--taped ALONG ACROSS` gives the scorer the real answer, and it
splits the two errors. In Gazebo the truth does the tape's job. The B mark is only as
good as the GPS that placed it: average the fix for 30 s (section 5).

To record a run: `tools/record.sh` records the EKF, the wheels and `/control/cmd_vel`.
For B from the point driver's feedback, also record `/drive_to_point/_action/feedback` (a hidden topic:
`ros2 bag record --include-hidden-topics`), `/drivers/point/cmd_vel`, and in Gazebo
`/vehicle/gazebo/truth`.

## 5. "Where am I?" is the hard part

| Question | Answered by | The catch |
|---|---|---|
| Where is A, where is B? | GPS: drive there, record `/sensors/gps/fix` for 30 s, average it | needs a good fix; RTK makes it centimetres |
| Where is the car now? | `ekf_global`: wheels + IMU + GPS | GPS-only is ±1-3 m without RTK |
| **Which way does it face?** | the gyro tracks **turns**, but nothing knows the starting heading | ⚠️ the gap: see below |
| How far has it gone? | `ekf_local`: wheels + IMU | drifts slowly; fine over 10-50 m |

### The heading gap

The IMU's gyro measures **turning**, not **north**. We don't use the magnetometer (it
needs calibrating, and the car's motors upset it). The start heading comes from
`start_yaw`, typed in by hand; once the car moves, `gps_heading` (`aav_gps_heading`)
gives `ekf_global` the heading from the GPS receiver's velocity. The options:

| Option | How | Cost |
|---|---|---|
| Park on a marked spot, facing a known way | `start_yaw` from the marks | free; good enough for a demo |
| Drive a few metres first | ✅ built: `gps_heading`, from the GPS velocity (median 1.6-2.1° off in Gazebo) | a few metres of driving; needs `gps_driver:=ublox` on the car |
| The ZED-F9R's own dead reckoning | it fuses its IMU, wheel ticks and GPS, and outputs a heading | set it up on the receiver |
| Two GPS antennas | the line between them is the heading | a second receiver |

**Recommendation:** a marked start spot for the first demos, with `gps_heading` once the
car moves; then the ZED-F9R's heading if that isn't enough.

## 6. Do we need a costmap or a map?

| For | Needs |
|---|---|
| Way 1 (scripted) | nothing: no GPS, no map |
| Way 2 (Nav2) | the empty map (exists) + the datum at the test site + the sensors for obstacles |
| Way 3 (point driver) | the datum, A and B; no costmap. ⚠️ collision_monitor guards only Nav2's commands, not the point driver's |
| Later: a route with curbs and lanes | a map of the test site (`tools/make_map.py`), or a lane layer |

To make a map of the test lot: drive the edges, record GPS points, and draw them into a
map with `tools/make_map.py` (it already makes `empty` and `example_lot`).

## 7. Test it, step by step

| # | Where | Run | Proves |
|---|---|---|---|
| 1 | Turtle | `pixi run turtle`, then the scripted goal above | the speed profile and the action |
| 2 | Bench | `pixi run bench`, then the scripted goal; `pixi run bench-check` | the same through the real CAN frames and the spine logic |
| 3 | Gazebo | `pixi run sim`, then the GPS goal above; `pixi run sim-check --drive-to-point` | GPS, both EKFs, Nav2, stopping at B; the point driver's stop |
| 3b | Gazebo, the whole picture | `pixi run demo`: Gazebo, RViz, the HUD and the map window in one go. Give a goal with RViz's "2D Goal Pose", or the map window's search, then DRIVE HERE and CONFIRM. With a box in the lane and the screens: `pixi run sim obstacle:=true hud:=true map:=true` | Nav2 planning round things, the screens, a goal set by a person |
| 4 | Car on stands | `pixi run car orin` + `motor_pi`; the scripted goal at 0.3 m/s | the signs, the speed loop, the e-stops |
| 5 | Car, empty lot | scripted at 0.5 m/s, 10 m; then `pixi run drive-to-point --lat .. --lon ..` | the real thing, with a safety driver |

## 8. What to do first

| # | Task | Who |
|---|---|---|
| 1 | At the car: find out whether the GPS gives fixes (`ros2 topic hz /sensors/gps/fix`) | the team |
| 2 | Pick the test lot; measure its datum | the team |
| 3 | A "mark this spot" tool: average `/sensors/gps/fix` and print lat/lon | small, ours |
| 4 | The point driver (way 3) | ✅ built 15 Oct; on the car after the brake test (4.3) gives `max_brake` |
| 5 | A Gazebo world shaped like the test lot | ours |
