# aav_sim_cameras

## Overview

**Our simulator's cameras.** Bevy draws the world from each camera on the car, offscreen,
and reads each picture back. The pictures go out on the car's camera topics, with Gazebo's old
picture size, lens, rate, encoding and frame, so nothing downstream can tell the two apart.
The plan: [our own simulator](../../../docs/designs/2026-11-05_own-simulator.md); how to
use it: [docs/own-simulator-cameras.md](../../../docs/own-simulator-cameras.md).

```text
 the core simulator ──/vehicle/sim/truth──► sim_cameras ──► /sensors/camera/<cam>/image_raw
   (or a scripted drive)                    (Bevy, no window) /sensors/camera/<cam>/camera_info
                                                              /vehicle/sim/chase_camera/image_raw
```

| Camera | Topic | Frame | Mount |
|---|---|---|---|
| front left | `/sensors/camera/front_left/` | `camera_left_link` | `vehicle.yaml` |
| front right | `/sensors/camera/front_right/` | `camera_right_link` | `vehicle.yaml` |
| rear | `/sensors/camera/rear/` | `camera_rear_link` | `vehicle.yaml` |
| chase | `/vehicle/sim/chase_camera/` (no camera_info) | `base_link` | `config/cameras.yaml` |

All four: 640 x 360, rgb8, 10 Hz. The three car cameras: focal 343 px, centre (320, 180),
no distortion. The chase camera: 70 degrees across.

## Subdirectories & Boundaries

| File | What it is |
|---|---|
| `crates/camera-core/` | no Bevy, no ROS: lenses and camera_info (`lens.rs`), what `graphics:=pretty` adds (`pretty.rs`: surfaces, their grain, the sky), mounts and topics (`rig.rs`), the shared world type (`world.rs`), the core's world as that type (`core_world.rs`), OBJ meshes (`obj.rs`), when each camera is due (`schedule.rs`), pictures (`frame.rs`), the `CameraRenderer` trait and `Cameras` (`lib.rs`) |
| `src/lib.rs` | `BevyRenderer`: the trait in Bevy, offscreen, stepped by hand |
| `src/scene.rs` | a `SimWorld` as Bevy's meshes, materials, sky and sun |
| `src/main.rs` | the program `sim_cameras`: the timing check, `--bench`, `--save`, `--ros` |
| `src/ros.rs` | the node `sim_cameras` (feature `ros`). Its docstring lists its topics |
| `config/cameras.yaml` | each camera's lens, rate, topic and frame |
| `tests/render.rs` | the pictures on this computer's GPU |

## Interface

```sh
pixi run sim-cameras                         # the test world, 10 s: the timing check
pixi run sim-cameras --save /tmp/pics        # and keep each camera's first picture
pixi run sim-cameras --from 18 --speed 0     # parked 7 m from the stop sign
pixi run sim-cameras --bench 300             # as fast as it can
pixi run sim-cameras --ros                   # publish, a scripted drive, wall-clock stamps
pixi run sim-cameras --ros --pose truth      # draw where /vehicle/sim/truth says, its stamps
pixi run sim-cameras --world maps/campus/campus.sdf --at 49,-71,0 --save /tmp/pics
                                             # an SDF world, parked there (x, y, yaw)
pixi run sim-cameras --world W --cameras front_left --poses p.csv --save DIR
                                             # one picture per camera at each x,y,yaw
                                             # line, then stop (tools/lane_data.py)
pixi run sim-cameras --graphics simple       # the plain look; pretty: grain, sky, haze
                                             # (auto, the default: pretty on a GPU)
```

`pixi run sim` (ours is the default) starts it with `--ros --pose truth --world <the world>`
(`aav_sim_models` `worlds.sim_cameras_command`). With `--ros` it also reads
`/vehicle/sim/entities` and draws each model placed while running. With `faults:=true` the
front left picture goes elsewhere: `--image-topic front_left=/vehicle/sim/camera/front_left/image_raw`
(its camera_info stays). A computer with no GPU draws in software: put
`tools/software_gl.sh` in front (Mesa's lavapipe); `--software` asks for it beside a GPU.
The log's first line says which adapter draws.

In Rust, for the core simulator (no ROS needed):

```rust
let rig = Rig::from_yaml(CAMERAS_YAML, VEHICLE_YAML)?;
let renderer = BevyRenderer::new(&rig, &world)?;   // world: camera_core::SimWorld
let mut cameras = Cameras::new(rig, renderer);
let frames = cameras.tick(&car_in_map, sim_time)?;  // only the cameras due now
```

## Invariants

- **Its own Cargo workspace, no `package.xml`**: colcon and the car's builds never compile
  Bevy. Runs on a Mac or a Linux desktop (with no GPU: in software), never on the Orin.
- The camera mounts come from `vehicle.yaml`: the only place car measurements live.
- Pictures are stamped with the car pose's time, and drawn on the whole tenths of sim
  time, as Gazebo's 10 Hz sensors were. With `--pose truth` they also publish
  `/vehicle/sim/cameras/free_until`, the next picture's sim time: with camera lockstep
  Ravensim's clock waits there, so software drawing keeps 10 Hz in sim time.
- The car cameras never see the car's body; only the chase camera does.
- Logic goes in `camera-core`, with its own tests.

## Side Effects

- Uses the GPU, or with none a software renderer (lavapipe, on the CPU). With `--ros`: publishes the camera topics above, through
  `tools/ros.sh --local`, so it never meets a real car.
- It doesn't announce on `/system/source`: the simulator that starts it does, as
  `sim_adapter` does for the cameras (`aav_sim_models/sources.py`).

## Verification

```sh
cd ros/sim/aav_sim_cameras && cargo test -q -p camera-core   # tools/test.sh runs this
pixi run cargo test --release                                # the pictures too: needs a GPU
pixi run sim-cameras --seconds 20                            # the timing check
```
