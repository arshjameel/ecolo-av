# Gazebo, as it was

Gazebo Harmonic (gz-sim 8) played the car in `pixi run sim` from 10 Oct to 16 Nov 2026.
Ravensim, our own simulator in Rust, took over as the default on 8 Nov. On 16 Nov Ben said
remove Gazebo ("lets do it"; CARLA stays). This page keeps what it was, what took each
part's place, and its last scores.

## What ran

```text
 ego ─► output mux ─► gazebo_adapter ──/gazebo/cmd_vel──► ros_gz_bridge ──► gz sim
                      (actuator model)                                       │ AckermannSteering,
                                                                             │ IMU, NavSat, Sensors
 gazebo_imu, gazebo_gps, gazebo_radar ◄── /gazebo/imu, navsat, radar/points ◄┘ (ogre2 cameras,
 gazebo_camera (faults)               ◄── road bridge: the cameras              gpu_lidar)
```

| Gazebo part | What it did | In its place now | Since |
|---|---|---|---|
| `gz sim` (`ros_gz_sim`, `gazebo.launch.py`) | the world, the car, physics | Ravensim's `simulator` (`aav_simulator`, Rust) | 8 Nov default |
| `ros_gz_bridge` (`config/bridge*.yaml`) | Gazebo's topics into ROS | none needed: Ravensim publishes ROS itself | 8 Nov |
| ogre2 cameras (`aav.sensors.gazebo.xacro`) | the car's three cameras, the chase camera | `aav_sim_cameras` (Bevy, offscreen) | 8 Nov |
| `gpu_lidar` | the roof lidar, the radar stand-in | Ravensim's ray casting | 8 Nov |
| `aav.gazebo.xacro`'s plugins | AckermannSteering, IMU, NavSat, odometry, joint states | Ravensim's `TwistFollower`, IMU, GPS (the same numbers) | 8 Nov |
| `gazebo_adapter`, `gazebo_imu`, `gazebo_gps`, `gazebo_radar`, `gazebo_camera`, `gazebo_esp32` | the car's topics from Gazebo's | the same code, renamed `sim_adapter`, `sim_imu`, `sim_gps`, `sim_radar`, `sim_camera_relay`, `sim_esp32` (`aav_sim_models`) | 16 Nov |
| `/gazebo/cmd_vel`, `odom`, `imu`, `navsat`, `radar/points`, `camera/front_left/image_raw` | between the simulator and those nodes | `/vehicle/sim/...`, the same names after it | 16 Nov |
| `aav_gazebo` (the package) | the worlds, models, shared sim modules, nodes | `aav_sim_models` (renamed; Gazebo's bits deleted) | 16 Nov |
| `vehicle.gazebo_steering_limit` | the limit AckermannSteering needed to reach full lock | gone: Ravensim steers to `max_steering_angle` itself | 16 Nov |
| Gazebo's window, `gz_window.sh` | to watch the car | RViz, Lichtblick (`chase_camera`), the developer window | 16 Nov |
| Rendering with no GPU (CI, oirp-spss) | Mesa's llvmpipe drew Gazebo's cameras in lockstep with its clock | Ravensim's camera lockstep (`camera_lockstep:=auto` under `tools/software_gl.sh`) | 16 Nov |

## What stayed

| Piece | Why |
|---|---|
| The SDF worlds and models | Ravensim reads SDF 1.9, the same files. Gazebo's `<plugin>`s came out; `ravensim::sensors` marks a world that draws sensors |
| Gazebo's numbers | the IMU and GPS noise, the steering gain (100/s), the clock (1 kHz): Ravensim copies them, so the results below still compare |
| `vendor/gazebo/` | Gazebo's source at the releases we ran, to read (`pixi run get vendor`) |
| `spine_sim car:=gazebo`, `stack.launch.py target:=gazebo`, `simulator:=own` | old names, still taken for one release |

## Its last scores

### Head to head with Ravensim, cameras on (8 Nov, Ben's Mac, quiet)

| Check | Ravensim | Gazebo |
|---|---|---|
| `sim-check` x 3 | 3 of 3 pass; EKF vs truth 0.11-0.40 m | 2 of 3 (once the EKF never came up); 0.10-0.14 m |
| `--obstacle` (the box) | pass: 0.36 m before it | pass: 0.28 m |
| `--lidar-obstacle` (the post) | pass: 0.33 m | pass: 0.25 m |
| `--speed-steps`: overshoot | 15.9, 15.2, 10.1 % | the same to 0.1 % |
| `--lane-only` (road) | pass, 5.4 s | pass, 5.1 s |
| `measure lane`, a campus road | 171 m, mean error 4 cm | 60 m, ended in the other lane |
| `--perception` (RF-DETR, the stop-sign lock) | fail that day; 3 of 3 the next runs | pass: locked at 68 px |

### CI's nightly `sim` job (Linux, no GPU: llvmpipe)

| Night | Gazebo's `sim-check --obstacle` |
|---|---|
| 15 Nov | pass |
| 16 Nov (its last) | fail: the stack came up, Nav2's controller said "Failed to make progress", and the launch was stopped |

Most of the rows marked "works in Gazebo" in [Status](../status.md) were measured there,
10 Oct to 8 Nov.

## Why it went

| Reason | |
|---|---|
| "We run our own stack" (Ben, 14 Nov) | Nav2 and Gazebo were yardsticks; Ravensim passed every leg Gazebo passed |
| Its only job left | computers with no GPU. Ravensim's cameras now draw there too, in lockstep |
| Weight | `ros_gz` brought 25 packages (gz-sim, ogre-next, DART...) into every environment |

To bring it back for a comparison: check out the commit before its removal and run
`pixi run sim simulator:=gazebo`.
