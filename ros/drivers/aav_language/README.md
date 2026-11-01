# aav_language

Plain-English driving (`docs/plain-english.md`). Two parts:

- the **drive server**: an MCP tool server over stdio, so a language model (or any AI
  app) can drive with high-level moves: drive, turn, stop, go to a GPS point;
- **talk**: a ReAct agent on a local model. You type what the car should do, it shows
  the plan, and nothing moves until you type "yes".

```
you -> talk (local model plans) -> drive server (MCP stdio) -> language_drive node
   -> /scripted_drive (aav_scripted) or /follow_gps_waypoints (Nav2) -> twist_mux -> ego
```

## Overview

Every rule is checked by the server, never by the model. The server sends goals to
the existing drivers, so twist_mux, ego and the safety locks stay in charge, and a
person on the joystick always wins.

```sh
pixi run -e llm talk                          # ask, plan, confirm, drive
pixi run -e llm talk --fake                   # a pretend car, no ROS
pixi run -e mcp drive-server                  # turtle, sim or bench (this computer only)
AAV_DISCOVERY=SUBNET pixi run -e mcp drive-server --car   # the real car: only with a safety driver
pixi run -e llm llm-eval                      # score local models on the test set
```

The `aav-drive` entry in `.mcp.json` starts the drive server for Claude Code.

Options:

| Command | Options |
|---|---|
| `drive-server` | `--car`; `--max-speed` (m/s, default 1.0; ego's `max_speed` caps it too); `--fake` (a pretend car, no ROS) |
| `talk` | a request (none = ask in a loop); `--model` (an MLX model); `--fake`; `--car`; `--yes`; `--max-speed`; `--json-actions` (JSON actions, not the model's own tool format); `-v` (print each Thought/Action) |

## Interface

MCP tools (`drive_server.py`):

| Tool | Does |
|---|---|
| `status()` | speed, pose, GPS, e-stop, armed, who drives |
| `drive(distance_m, speed_mps, direction, curvature, wait)` | the whole distance, ramps included |
| `turn(angle_deg, radius_m, direction, speed_mps, wait)` | an arc |
| `stop()`, `estop()` | cancel this server's drive (it slows to a stop; never refused); set the e-stop (never clears it) |
| `go_to_gps(lat, lon, wait)` | one Nav2 GPS waypoint, at most 200 m away |
| `wait(seconds)`, `check(tool, arguments)` | pause; would that call be refused? |

ROS node `language_drive` (`ros_backend.py`):

| Item | What |
|---|---|
| Subscribes | `/localization/odom/local` (`nav_msgs/Odometry`); `/sensors/gps/fix` (`sensor_msgs/NavSatFix`, best effort); `/safety/estop` (`std_msgs/Bool`); `/vehicle/status` (`aav_interfaces/VehicleStatus`); every twist_mux input and lock from `aav_command_mux`'s `twist_mux.yaml` (arrival times only, to tell who drives); `/diagnostics` (twist_mux's lock report); `/tf`, `/tf_static`; `/safety/geofence/fence` (`geometry_msgs/PolygonStamped`, latched) |
| Publishes | `/safety/estop` (`std_msgs/Bool`): only true, five times in 0.5 s |
| Calls | actions `/scripted_drive` (`aav_interfaces/action/ScriptedDrive`), `/follow_gps_waypoints` (`nav2_msgs/action/FollowGPSWaypoints`); `/ego/get_parameters` |

Default caps (`safety.Caps`), tightened to ego's own limits at start:

| Cap | Value |
|---|---|
| speed forward / reverse | 1.0 / 0.5 m/s |
| distance per command / reverse | 200 / 10 m |
| acceleration, deceleration | 0.5, 0.5 m/s² |
| `wait()`, turn | 120 s, 360° |
| turn radius | at least the car's own (`wheelbase / tan(max_steering_angle)`) |

Modules:

| Module | ROS? | Holds |
|---|---|---|
| `safety.py` | no | `Caps`, `effective_caps`, `check_drive`, `check_turn`, `Where`, `allowed_here`, `car_unreachable`, `mux_config`, `mux_choice`, `holding_locks`, `mux_locks_from_diagnostics`, `drive_end`, `gps_end`, `fence_refusal` (the geofence) |
| `drive_maths.py` | no | `plan_distance` -> `DriveGoal`, `plan_arc`, `signed_angle`, `min_turn_radius`, `haversine` |
| `drive_tools.py` | no | `DriveTools` (the tools, every rule checked here), `Backend` protocol, `CarState` |
| `react.py` | no | the ReAct loop: `ReactAgent` (what `talk` runs), `parse_reply` (Qwen, Llama, Gemma, Mistral formats, JSON fallback), `system_prompt`, `Plan`, `Step` |
| `fake_backend.py`, `eval_set.py` | no | a pretend car; the model test's requests and `score()` |
| `llm_mlx.py`, `llm_openai.py` | no | `MlxChat` (Apple silicon), `OpenAiChat` (llama-server, Ollama, trtllm-serve, mlx_lm.server on this computer) |
| `ros_backend.py` | yes | `RosBackend`: the node, on its own thread |
| `drive_server.py`, `agent.py`, `run_eval.py` | | the entry points: `drive_server`, `talk`, `llm-eval` |

## Invariants

- The rules live in `safety.py` and `drive_tools.py`, with no ROS imports; the model
  never decides them.
- High-level tools only, never raw velocities.
- Without `--car` it drives only a stand-in: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
  (no static peers) or a simulator's `/clock`, and none of the car's own nodes in sight
  (`legacy_uart_bridge`, `encoder_udp`, `socket_can_*`, or `spine_bridge` without `spine_sim`).
- Nothing moves while the e-stop is on, or while twist_mux's report is silent. A move is
  refused while a lock holds the driver that would carry it (`holding_locks`): the
  stop-sign and sensor locks (30) hold Nav2 (20), so `go_to_gps`, but not the scripted
  driver (50), so not `drive` or `turn`. It publishes only `true` on `/safety/estop`; a
  person clears it.
- A drive, turn or GPS goal that would end outside `/safety/geofence/fence` is refused
  before anything moves. It never calls `/safety/geofence/clear` (a test checks).
- `--car` refuses to start while ROS discovery stays on this computer (`tools/ros.sh
  --local`, LOCALHOST, no static peers): it could reach no car. Start it with
  `AAV_DISCOVERY=SUBNET` (`car_unreachable`).
- talk queues moves while planning (each one checked); nothing moves before "yes".
  `stop` and `estop` typed alone skip the model. Ctrl-C while moving calls `stop()`.
  `--yes` is for automated tests only, and never with `--car`.
- `drive()` covers the whole distance: if too short to reach the speed, the top speed
  drops so both ramps fit.
- When the app that started the server goes away (stdin closes), it cancels any drive.
- Models are meant to run on this computer: MLX in this process, or a server at
  `127.0.0.1:8080` by default. Nothing enforces it: `OpenAiChat` and `llm-eval --base-url`
  take any URL (and `OpenAiChat` an API key).
- Greedy decoding: the same request gives the same plan.
- Turn angles: + = left.

## Side Effects

- Sends action goals; may publish `/safety/estop` true.
- Logs every tool call, with the time, to `$ROS_LOG_DIR/aav_language/tool_calls.jsonl`
  (else `~/.ros/log/...`).
- talk starts the drive server as a child process. `llm_mlx` downloads models into the
  Hugging Face cache; `llm_openai` makes HTTP calls to a local server.

## Verification

```sh
pixi run build && colcon test --packages-select aav_language && colcon test-result --verbose
```

Quick, without ROS: `cd ros/drivers/aav_language && python3 -m pytest -q`
(`tools/test.sh` runs it). `pixi run test` runs everything.

| Test | Covers |
|---|---|
| `test_safety.py` | caps, where it may drive, who drives |
| `test_drive_maths.py` | distance and arc plans |
| `test_drive_tools.py` | the tools on the fake car |
| `test_react.py` | reply parsing and the plan |
| `test_llm_openai.py` | the OpenAI-compatible request and reply |
