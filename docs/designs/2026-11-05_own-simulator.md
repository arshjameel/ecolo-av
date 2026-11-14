# Our own simulator: Rust and Bevy, and one window

* **Author:** Ben (the direction), written up 5 Nov 2026
* **Date:** 2026-11-05
* **Status:** the direction is Ben's. Everything marked "proposed" waits for an EGM review.
  The window runs on pretend data and on live Gazebo. **7 Nov: the core is built**
  (`aav_simulator`, `simulator:=own`; [our own simulator](../own-simulator.md)): Gazebo's worlds,
  the car, the lidar, radar, IMU, GPS and clock, without cameras. TODO 114, 115, 134.
  **Built (0.3, PR #27; checked 13 Nov):** the cameras too (`aav_sim_cameras`, TODO 135, 141), and `simulator:=own` is the default since 8 Nov. Left: a no-GPU camera path (TODO 151), the window's algorithm list and SWAP.

Ben, 5 Nov: "we want to replace gazebo with our own contorl too. We want to create all the
sensor data and stuff like gazebo does but in rust and with bevy."

## The idea in one line

A Rust simulator publishes the same topics as the real car. One Bevy window reads those topics,
so what you test on a desktop is what you see on the car.

## Who runs what (Ben, 5 Nov)

```text
 TESTING (Mac or carly)              ON THE CAR
 ┌────────────────────────┐          ┌─ ORIN: no graphics, no simulator ┐
 │ our simulator          │          │ sensors, perception, TensorRT    │
 │ the stack              │          │ planning, ego, safety            │
 │ the Bevy window        │          └───▲──────────────────┬───────────┘
 └────────────────────────┘      commands│       cubes, status, picture
                                     ┌───┴──────────────────▼───────────┐
 Dash Pi: HUD, street map (raylib)   │ MACBOOK, plugged in: Bevy, LLM   │
                                     └──────────────────────────────────┘
```

| Computer | Runs | Never runs |
|---|---|---|
| Orin | the real car's ROS, TensorRT | a screen, a simulator |
| Dash Pi | the HUD and street map, raylib | Bevy |
| MacBook | the Bevy window, the hardware panel later, the LLM | |
| Mac or carly | the simulator, for testing; only developers see it | |

## Everything is a drop-in replacement (Ben, 5 Nov)

"Gazebo produces sensor data, our code can produce the same thing and real sensors too. Same
with all the algorithms just swap out and test."

```text
 Gazebo ───────┐                     ┌── NDT, AMCL, EKF, ours
 our simulator ├─► the same topics ─►├── lattice, Hybrid-A*, diffusion, ours
 real sensors ─┘                     └── MPPI, Stanley, LQR, ours
```

| A drop-in must match | From |
|---|---|
| Topic name, type, frame, units | `docs/interfaces.yaml` |
| Rate, QoS, stamps | to add to the contract where it's missing (proposed) |
| `/system/source` says what it is | SIM or real; the real car only runs on real data (`aav_io_mux`) |

| The limit | Why |
|---|---|
| The safety locks are never swapped out | a new filter sits beside them, never instead |
| On the real car, a swap happens at launch, not live | SWAP stays SIM only |
| A simulator's extras, such as truth, are test-only | nothing that drives the car may read them |

**The missing test (proposed, TODO 118):** one conformance check that any source or algorithm
must pass before it counts as a drop-in. Then one scoring run for each, on the same routes.

## Goals and non-goals

| Goals | Non-goals |
|---|---|
| One window instead of Gazebo's, two RViz, the debug HUD | Changing the HUD or the street map |
| Every sensor Gazebo gives us, on the same topics | Lifelike pictures: Gazebo's aren't either |
| Sensor streams that repeat from a seed; measured speed | Proving the real car: the car decides that |
| Winter and faults built in | A general robot simulator: this one car, these worlds |

## Why our own

| Gain | How sure |
|---|---|
| Fewer windows and no copied picture | once cameras are ours; two programs stay (below) |
| Sensor streams that repeat exactly, from a seed | yes, for the simulator alone. Nav2, the EKFs and DDS still race, so **whole runs won't repeat**. Measure how much they vary |
| Faster than real time | unmeasured. Timeouts run on sim time, so speeding up shrinks the time to compute. Report the real-time factor with every score |
| Winter and faults built in | later. Any slip or weak-brake number is invented until the car measures it |
| CI without a GPU | yes, for everything except the cameras |

## What Gazebo does today, and what replaces it

Sizes are guesses for one person full time. Nothing here is built.

| Gazebo's job | Today | Ours (proposed) | Difficulty |
|---|---|---|---|
| Car movement | ackermann plugin, DART physics | a bicycle model with dead time; `actuator_model.py` stays at first | easy at 1 m/s |
| IMU, GPS, wheels | gz sensors, then `imu.py`, `gps.py`, `wheels.py` | raw values from `sim-core`; those Python nodes stay at first | easy |
| Faults | `faults.py`, `pixi run sim-fault`, one node per sensor | unchanged while those nodes stay | none yet |
| `/clock`, topics | `ros_gz_bridge` | `rclrs`, the same names | easy |
| Roof lidar, radar | `gpu_lidar` | ray casting on the CPU; `debug-app/src/demo.rs` has a small one | medium |
| Worlds | `test_lot.sdf`, `road.sdf` | read the same files | medium |
| Cameras | ogre2 | draw in Bevy, read the picture back | **hardest** |
| Tools and CI | 22 scripts and tests mention Gazebo | one `simulator:=` switch, tool by tool | medium, tedious |

**Done for this (5 Nov):** the nine Gazebo-named topics are now `/vehicle/sim/...` and
`/control/sim/...`, and the output mux's target is `sim`. `pixi run drop-in` measures each
stream's rate and frame against the contract's `drop_in` rules.

Roughly 2-3 weeks without cameras, and 5-6 with cameras, tools and the comparison.

## How it's built (proposed, after review)

```text
 sim-core (no ROS, no Bevy)   motion, IMU, GPS, rays, faults, worlds; tested with cargo
     │
 simulator (rclrs, no Bevy)   replaces gz sim and the bridge; runs in CI with no GPU
     │  the bridge's topics; aav_gazebo's nodes and their maths stay
     │
 camera renderer (Bevy)       only once cameras come: its own program, so a slow picture
     │                        can't delay the IMU
 dev_window (Bevy, egui)      ros/hud/aav_dev_window; the same on the desktop and the car
```

| Choice | Why |
|---|---|
| Replace `gz sim` and its bridge first; keep `gazebo_adapter` and the sensor nodes | the actuator model, faults, `sim-fault`, `/system/source` and most tools keep working. Those Python nodes move to Rust only after ours wins |
| Reuse `bench-car/src/plant.rs` (a bicycle model) and `debug-app/src/demo.rs`'s rays | they exist, so they move into `sim-core` rather than being written again |
| No Bevy in the simulator | CI needs no GPU, and the open laptop and Linux questions can't block it |

### Extra jobs the table needs

| Job | Why |
|---|---|
| Dead time in the car's motion | `actuator_model.py` leaves it out because Gazebo's pipeline adds about 20 ms. Ours would lose that delay |
| Spawning, removing and moving things | `sim_check.py` and `stream_check.py` use them |
| Hitting something | Gazebo stops a car at a wall. Ours needs a footprint check that fails the run (TODO 93) |
| `/joint_states`, `camera_info`, truth | on the bridge today |
| The spine and ESP32 loops | `sim-check --spine` uses them |

## The first version (after review)

**Smallest useful:** the empty world. No Bevy, no lidar, no world reader.

| In | Left out, for now |
|---|---|
| Motion with dead time, raw IMU, GPS, odometry, wheels | Lidar and radar: the next step, in the test lot |
| `/clock`, `/joint_states`, truth | Cameras: Gazebo keeps those runs |
| A `simulator:=` switch | The road world, winter, the car's sensor rates |

**It passes when** `sim-check --drive-to-point` and `--speed-steps`, which already use the empty
world, pass on it, and it lands within Gazebo's own measured spread. That's the case Gate 4
needs, and it tests the referee's method. Size: up to two weeks (a guess; a limit, too).

Ours can become the default for camera-free runs first. Gazebo keeps cameras and the detector.

## The referee: what it can and can't catch

**Gazebo is only partly independent.** Its motion goes through our `actuator_model.py`, and
`imu.py`, `gps.py` and `radar.py` are ours too. Comparing those with a Rust port compares our code
with itself.

| Channel | Who can judge it |
|---|---|
| Physics of the car and contacts; lidar geometry | Gazebo |
| Actuator lag, IMU and GPS noise, radar | **only the car's logs** |
| Everything, finally | the car |

| Test, ours against Gazebo | The mark |
|---|---|
| Same commands replayed, no feedback | each channel, because a closed loop hides model errors |
| Lidar at fixed poses | range differences |
| Whole routes: path, time, stop position, legs finished | **within Gazebo's spread between its own repeated runs.** Measure that first; compare distributions, not one run |
| Faults, obstacles | `sim-check --faults`, `--obstacle`, `--lidar-obstacle`, `--spine`: the same results |
| NDT | both on **one** map built in Gazebo: a map built in ours can confirm itself |

### What could still fool us

| False comfort | Guard |
|---|---|
| Sensors with no delay or jitter, and perfect clocks | add measured delays; sweep ranges, not one value |
| Mac speed standing in for the Orin's | report it; measure on the Orin when possible |
| Flat ground, guessed actuators | every result says "unvalidated" until the car's logs check it |
| "The detector can't tell the difference" | true of the message only. Detection rates must be tested on real pictures |

### What the drop-in check does and doesn't prove

`pixi run drop-in` checks each stream's rate on the run's clock, frame and publisher QoS. Passing
is **necessary, not enough**: it never looks at values. Stamp age, point fields and rules for more
topics come when a swap breaks on them. Scoring algorithms stays with `measure controllers`.

## The window's test (built 5 Nov)

`pixi run dev-window`: Ben's layout, in `ros/hud/aav_dev_window`, with pretend data.

| What was measured | Result |
|---|---|
| Computer | Ben's MacBook, Apple M4 Max, window 3200x2000 pixels |
| Clean release build | 1 min 32 s; after a change, about 3 s |
| 20,000 cubes, every layer on | **238.7 fps**, slowest 1 % of frames 9.9 ms |
| 60,000 cubes | 225.8 fps, 10.8 ms |
| 150,000 cubes | 179.3 fps, 21.3 ms |
| Reuse | the shared `debug-core`, `map-core` and `hud-core` built unchanged |

5,000 cubes also gave about 240 fps, so about 240 looks like a cap, and 20,000 cubes doesn't
reach the limit. The cubes panel is only about a quarter of the window's height, so a bigger
panel will cost more. raylib's earlier 358 fps (`docs/hud.md`) was under other conditions, so
it isn't a fair comparison.

### On live Gazebo (5 Nov, Ben's Mac, headless, test lot)

| What | Result |
|---|---|
| Cubes, picture, map, costmap, NDT's map, plan, console, source | all live |
| Speed, pool of 20,000 cubes | 219.7 fps, slowest 1 % of frames 8.2 ms; 147.6 fps and 9.9 ms in the final layout, with NDT's cells on |
| `--click-goal 14,0`, through every gate | Nav2 drove there: "Goal succeeded" |
| A second goal, (6.70, 31.11), in the same run | not scripted; matches a person's click and two presses in the window. Ben to confirm |

### The layout (Ben, 5 Nov)

```text
┌─ SIMULATED · GO / STOP and why · e-stop · source ──────── [E-STOP] ┐
├───────────────────────────────┬────────────────────────────────────┤
│ THE SCENE: the car driving    │ DEBUG: what the lidar sees, from   │
│ (Gazebo's chase camera)       │ the car; NDT's state over it       │
│                               ├────────────────────────────────────┤
│                               │ THE MAP: costmap, NDT's map, cubes,│
├───────────────────────────────┤ plan. Click to pick a goal         │
│ CONSOLE: /rosout              ├────────────────────────────────────┤
│                               │ KEYS | ALGORITHMS                  │
└───────────────────────────────┴────────────────────────────────────┘
```

| Note | Why |
|---|---|
| NDT's distance from the EKF is the newest pose of each | stamps aren't matched, so it reads high while driving: 0.34 m at about 1 m/s. For judging, use `measure controllers`' truth |
| Nav2's red outlines start off | with `screen_cubes:=true`, Nav2 isn't reading these cubes |

### What the first measurement found

Gazebo against the car's numbers in `docs/bandwidth.md`. The car's are unmeasured on this stack.

| Stream | Gazebo | The car | So |
|---|---|---|---|
| GPS | 4.9 Hz | 10 Hz | the stack is tested at half the car's rate |
| IMU | 49 Hz | 200 Hz | a quarter |
| Cameras | 9.9 Hz | 15 Hz | two-thirds |
| `/vehicle/wheels` | 986 Hz | 100 Hz | Gazebo publishes them every physics step (`docs/bandwidth.md:186`), so it has no ceiling rule |

Our simulator could match the car's rates. Gazebo's floors are what the rules hold today.

| Not tested | Why it matters |
|---|---|
| A slower team laptop | part of the pass mark: 30 fps or more. **Bevy isn't fully decided until then** |
| `--watch` on the car | no car yet; `car.launch.py` has no `screen_cubes` switch yet |
| The `drop_in` rules against real sensors | the frames and rates are from Gazebo |
| Linux and carly | Bevy needs a working GPU driver there |

## The order of work (proposed)

| Step | What | State |
|---|---|---|
| 1 | One RViz | edited; not launched yet |
| 2 | The window's test | done on Ben's Mac; a slower laptop next |
| 3 | The window on live Gazebo, reading cubes | done |
| 4 | Click to go and the console: done. Cancel a goal, the algorithm list | next. SWAP later: `pixi run pick` already restarts with chosen switches |
| 5 | Measure Gazebo's spread between repeated runs | **after the stopping curve is built in Gazebo** |
| 6 | The first version; the referee | **up to two weeks** |
| 7 | Lidar, radar, the test lot; compare each | |
| After Gate 4 | Cameras, the other tools, retiring Gazebo, winter | |
| Later | The window watch-only on the car; retire `debug_hud`; the hardware panel after car day | |

**How the car stops stays first** (Ben's pick, and what Gate 4 on 26 Feb needs). The simulator
doesn't directly help Gate 4, because Gazebo already works.

## The cubes need a switch (proposed)

The window draws `/perception/voxels/cubes`, about 0.1 MB/s (estimated) against the point cloud's
7-9 MB/s. But `voxels:=true` also makes Nav2 use the cubes, and that setup once drove into a post
(TODO 93). So `screen_cubes:=true` runs the cubes node for the screen while Nav2 stays on the
flat scan. It's in `sim.launch.py`, not yet `car.launch.py`.

The NDT match panel draws from the point cloud today, so it moves to the cubes or NDT's score.

## The MacBook as a driver

| Ben's plan | Today |
|---|---|
| The LLM runs on the MacBook | `aav_language` runs there; `--car` is marked "only with a safety driver" |
| It drives like any joystick | through `twist_mux`, never straight to ego, so the locks and a person still win |
| TensorRT on the Orin | `tools/build_engine.py` builds a model; the diffusion planner's move is still TODO 78 |
| Prediction | not built; it needs tracking, which is paused |

**To check first (TODO 116):** if the Mac sends Nav2 a goal and then drops off, the car may carry
on to that goal. If so, the Mac needs a heartbeat that stops the car when it goes quiet.

## Risks

| Risk | How bad | What we do |
|---|---|---|
| The simulator shares our wrong ideas | high | Gazebo as referee; the car decides |
| It takes time from Gate 4 | high | stopping first; a two-week limit; Gazebo keeps working |
| Camera read-back is slow | medium | one low-rate camera first; measure |
| Students' laptops can't run Bevy | medium | test early; raylib's `debug_hud` stays until it's proven. The simulator needs no Bevy |
| Bevy changes between versions | medium | pin a version; keep logic out of Bevy |
| 22 tools depend on Gazebo | medium | move them one at a time, behind a switch |

## The EGM reviews (5 Nov)

Design and zoom-out, two reviewers, reading only; they ran nothing. All of their major findings
are taken above. Neither reviewer questioned the direction.

## Questions for Ben

| # | Question | Proposed |
|---|---|---|
| O1 | Does carly have a screen and a GPU? It isn't in the repo | |
| O2 | Which laptop is the slow one for the test? | ask the team |
| O3 | Stopping first, then two weeks at most on the first version? | yes |
| O4 | Does the road world move too, or the test lot only at first? | the test lot first |

## Related

[One developer window](2026-11-05_one-window.md), [one car](2026-11-04_one-car.md),
[how the car stops](2026-11-05_how-the-car-stops.md), `ros/hud/aav_dev_window`, `docs/simulation.md`,
TODO 93, 114-118. [Its reviews](#the-egm-reviews-5-nov).
