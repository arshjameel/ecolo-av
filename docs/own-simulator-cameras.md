# Ravensim's cameras

Ben, 7 Nov: replace Gazebo with our own Rust simulator, every sensor. This page is the
**cameras** part: `ros/sim/aav_sim_cameras`. The rest of the simulator (world, car, lidar,
IMU, GPS, wheels, clock) is [its own page](own-simulator.md); together they are
Ravensim, our own simulator (the default since 8 Nov, the only one since 16 Nov:
[Simulation](simulation.md)). With no GPU the clock waits for each picture
([camera lockstep](#camera-lockstep-16-nov)). They draw on the GPU;
with none, Mesa's software renderer draws them ([below](#with-no-gpu-15-nov)). The plan:
[our own simulator](designs/2026-11-05_own-simulator.md).

![The front left camera, 7 m from the stop sign](img/own-sim-cameras-front.png)

## In one line

Bevy draws the world from each camera on the car, with no window, and the pictures go out on
**Gazebo's camera topics**: same size, lens, rate, encoding and frame.

## The cameras

| Camera | Topics | Frame | Where it sits |
|---|---|---|---|
| front left | `/sensors/camera/front_left/image_raw`, `camera_info` | `camera_left_link` | `vehicle.yaml` |
| front right | `/sensors/camera/front_right/...` | `camera_right_link` | `vehicle.yaml` |
| rear | `/sensors/camera/rear/...` | `camera_rear_link` | `vehicle.yaml` |
| chase | `/vehicle/sim/chase_camera/image_raw` | `base_link` | 7 m behind, 3.5 m up |

| Setting | Value | Same as Gazebo? |
|---|---|---|
| Picture | 640 x 360, `rgb8` | yes |
| Lens | focal 343 px, centre (320, 180), no distortion | yes: K, P, R, D all match |
| Rate | 10 Hz, on whole tenths of sim time | yes |
| QoS | reliable, keep last 10 | yes |
| Stamp | the time of the car pose it was drawn from | yes |

![The chase camera: the car's body, the box, the stop sign](img/own-sim-cameras-chase.png)

## Run it

| Command | Does |
|---|---|
| `pixi run sim-cameras` | the test world for 10 s, then the timing table. No ROS |
| `pixi run sim-cameras --save /tmp/pics` | also keeps each camera's first picture |
| `pixi run sim-cameras --from 18 --speed 0` | the car parked 7 m from the stop sign |
| `pixi run sim-cameras --bench 300` | as fast as it can: the most pictures a second |
| `pixi run sim-cameras --ros` | publishes the topics, a scripted drive, wall-clock stamps |
| `pixi run sim-cameras --ros --pose truth` | draws where `/vehicle/sim/truth` says, with its stamps |
| `pixi run sim-cameras --world maps/campus/campus.sdf --at 49,-71,0` | a Gazebo world (the core's reader), parked at x, y, yaw |
| `pixi run sim world:=road` | the whole simulator: the launch starts the cameras on the world it runs |
| `pixi run sim-cameras --software` | a software renderer only, even beside a GPU |
| `tools/software_gl.sh pixi run sim-check` | the road world on a computer with no GPU: the cameras on Mesa's lavapipe |
| `tools/software_gl.sh --force pixi run sim-cameras --bench 100` | CI's way on a computer with a GPU, a Mac too |

The first line of the log says what draws: `sim_cameras: drawing with Apple M4 Max (Metal, GPU)`
or `sim_cameras: drawing with llvmpipe (LLVM 23.1.3, 128 bits) (Vulkan, software, on the CPU)`.

`--ros` needs `pixi run build` first, and stays on this computer (`tools/ros.sh --local`).

## How the core plugs it in

```text
 core simulator ─── /vehicle/sim/truth (50 Hz) ───► sim_cameras ───► the camera topics
                    or, in Rust: Cameras::tick(car pose, sim time)
```

Two ways, both ready:

| Way | When |
|---|---|
| **Its own program**, reading `/vehicle/sim/truth` | the default: a slow picture can't delay the IMU |
| **In Rust**, `camera_core::Cameras` with `sim_cameras::BevyRenderer` | if the core wants pictures in its own loop |

The shared types live in `camera-core`, which has no Bevy and no ROS:

| Type | What it is |
|---|---|
| `SimWorld` | the sky, the light, and a list of `Shape`s in the map frame; the car's body |
| `Shape` | a name, a `Geometry` (plane, box, cylinder, octagon), a pose, a `Look` (colour, picture) |
| `Rig`, `CameraSpec` | each camera: lens, rate, topic, frame, mount |
| `CameraRenderer` | the trait: `set_world(&SimWorld)` and `render(car, stamp, cameras)` |
| `Cameras` | the rig, a timer per camera, and a renderer: `tick(car, now)` draws the ones due |

### Joined to the core (8 Nov)

![The front left camera on Colonel By Drive, the campus world](img/own-sim-cameras-campus.png)

| Step | Where |
|---|---|
| The core reads the SDF world: every shape, its colour and picture, the sky and the sun | `sim-core` `world.rs` |
| Those shapes become a `SimWorld`: boxes, cylinders and planes one to one; OBJ meshes with their picture coordinates | `camera-core` `core_world.rs`, `obj.rs` |
| Bevy draws them; pictures repeat (the campus's tarmac and grass), with smaller copies far off | `src/scene.rs` |
| SDF's light is scaled by 0.6 to look like Gazebo's | `core_world::LIGHT` |
| `ravensim.launch.py` starts `sim_cameras --ros --pose truth --world <the world>`; `sim_adapter` announces the cameras | `aav_sim_models` `ravensim.launch.py` |
| Models placed while running (`obstacle:=true`, `spawn_entity`): the simulator lists them on `/vehicle/sim/entities`; the cameras read each SDF file and draw the world again | `sim-node`, `src/ros.rs`, `core_world::placed` |

## Tests

| Test | What it proves | Where |
|---|---|---|
| projection | a point 10 m ahead lands on (320, 180); 1 m left and 0.5 m up, 34.3 and 17.15 px off | `camera-core`, every `tools/test.sh` |
| camera_info | K, P, R, D as Gazebo's bridge sends them | `camera-core` |
| rate | one sim minute at 50 Hz gives each camera exactly 600 pictures, on the tenths | `camera-core` |
| picture colours | sky, road, grass, kerb, box, stop sign, post: each where the lens says, its colour | `tests/render.rs`, needs a GPU |
| picture edges | the stop sign's edges in the picture within 1 px of the lens's | `tests/render.rs` |

## Measured (7 Nov, Ben's M4 Max)

Other agents were using the same computer (load 10 to 40), so the slow tail is partly theirs.

| Run | Result |
|---|---|
| Bevy ready, shaders warmed | 0.2 to 0.5 s |
| Four cameras at 10 Hz, `--ros`, 25 s | 10.03 Hz each, 251 pictures, out 16 ms after due (median), 60 ms worst, none late |
| `--pose truth`, a 50 Hz truth on sim time | 10.00 Hz on sim time, every stamp on a tenth, 15 ms median, 26 ms worst |
| `ros2 topic hz` | 10.000 Hz, min 0.097 s, max 0.103 s |
| `--bench`, four cameras, a quiet minute | 57 rounds a second (227 pictures/s), 17.5 ms a round |
| No ROS, 30 s, heavy load | 9.92 Hz; median 19 ms; 25 of 298 late (worst 196 ms) |

So four cameras at the car's 15 Hz (60 pictures a second) should fit when the computer is quiet: not yet run at 15 Hz.

## With no GPU (15 Nov)

TODO 183, Gazebo's last job: GitHub's runners and oirp-spss have no GPU.

```text
 sim_cameras ─ Bevy ─ wgpu ─ Vulkan loader ─┬─ the GPU's driver        (a GPU: picked first)
                                            └─ Mesa's lavapipe, on the CPU (no GPU, or --software)
```

| Part | How |
|---|---|
| The adapter | wgpu picks a GPU first, a software one only when there is no GPU: the GPU path is unchanged |
| Asking for software | `--software`, or `WGPU_FORCE_FALLBACK_ADAPTER=1` (wgpu's fallback adapter) |
| Mesa's lavapipe | from conda-forge (`mesa-lavapipe`, MIT), through `pixi exec`: `tools/software_gl.sh`. Not in `pixi.lock`: it would replace a Linux laptop's own driver |
| On a Mac | Metal always. To try lavapipe anyway: `tools/software_gl.sh --force` builds the cameras with Vulkan too (feature `vulkan-on-mac`) |
| A fix on the way | lavapipe lists cooperative-matrix shapes but not the feature; Bevy asked for it and Vulkan refused the device. The cameras now turn that feature off (nothing uses it) |
| Building on Linux | the cameras take Bevy's renderer without its window parts (winit, X11, Wayland, gamepads), so pixi's environment is enough (gamepads wanted libudev's headers). The pictures are the same to the pixel |

### How fast (15 Nov, Ben's M4 Max, 16 cores, load 14)

The road world's test copy, all four cameras (640 x 360, 4x MSAA, shadows), `--bench`:

| Drawn by | One round of 4 pictures | Rounds a second |
|---|---|---|
| the GPU (Metal) | 13.6 ms | 74 |
| lavapipe, every core | 36.6 ms | 27 |
| lavapipe, 4 threads (`LP_NUM_THREADS=4`) | 54 ms (3 cameras: 39 ms) | 18 (3 cameras: 26) |
| lavapipe, 2 threads | 89 ms (3 cameras: 62 ms) | 11 (3 cameras: 16) |

| Check | Result |
|---|---|
| The timing check, 10 s, lavapipe | 4 cameras at 10.06 Hz, out 40 ms after due (median), worst 66 ms, none late |
| Its pictures against the GPU's | the same: no pixel off by more than 8 of 255 (worst 31, on edges) |
| The campus world (596,000 triangles), lavapipe | 1.3 s a round: **not usable**. Without shadows and MSAA still 0.29 s. Campus needs a GPU |

So the road world at 10 Hz x 3 cameras fits in software with 2 cores to spare for it.
GitHub's runners have 4 (x86_64), slower than the M4's: [[untested]] there.

## What it is not, yet

| Missing | Why it matters |
|---|---|
| Lavapipe on Linux (CI, oirp-spss) | tried on a Mac only (`software_gl.sh --force`). The picture tests (`tests/render.rs`) still don't run in CI |
| A side-by-side with Gazebo's pictures | the looks match by eye (`docs/img/sim-cameras.png`); not measured |
| The detector on these pictures | the stop sign carries Gazebo's own picture: see [own-simulator.md](own-simulator.md)'s results |
| The campus at 10 Hz on a busy computer | 4 cameras on the campus: 9.1 Hz, 105 ms median (no ROS, load 11) |
| Faults (freeze, stop) | `sim_camera_relay` does them for Gazebo; the simulator's fault switch should cover these |
| The JPEG topics | Gazebo has none either: `chase_jpeg` and `preview_jpeg` still make them |
| Lifelike pictures | a non-goal: Gazebo's aren't either |
