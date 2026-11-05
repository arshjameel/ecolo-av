# Sources and targets: the sensor mux and the output mux

Simulated data must never drive the real car (plan.md S-4, T1.5, decision D3). Two
nodes at the edges of the stack make sure: the **sensor mux** checks where the data comes
from, and the **output mux** sends ego's commands to one target, the real car only while
everything is real. Package `aav_io_mux`, built 23 Oct.

!!! note "The short version"
    - **Source:** where the sensor data comes from: `real`, `sim`, `bench` or `replay`.
    - **Target:** who plays the car: `car`, `gazebo`, `bench`, `turtle` or `carla`.
    - Every backend reads only its own target's topic, `/control/<target>/actuator_cmd`.
      Nothing reads `/ego/actuator_cmd` directly any more.
    - The real car's topic carries commands only while **every** source is REAL. One SIM
      or BENCH anywhere on the network, and it goes silent, **latched**: the car brakes.
    - Reopen it, once the cause is gone: `pixi run clear gate` (the service
      `/safety/output_mux/clear`).
    - The HUD shows it (24 Oct): a red **STOP** with the reason, such as "GPS is SIM";
      amber **WAIT**; green **GO** ([HUD](hud.md#may-the-car-move-the-go-stop-panel)).

## The picture

```text
 sources                       checked by                    targets
 ───────                       ──────────                    ───────
 real drivers ─┐                                       ┌─► /control/car/... ──► spine_bridge
 Gazebo        ├─► /sensors/... ──► sensor_mux         │                        legacy_uart_bridge
 bench mocks   │   /vehicle/...      │ /system/sensor_mux/state
 a bag (replay)┘   /system/source ───┤ (ok? which source?)
                                     ▼                 ├─► /control/gazebo/.. ──► gazebo_adapter
 drivers ─► twist_mux ─► ego ─► output_mux ────────────┤   /control/bench/... ──► spine_bridge (bench)
                    /ego/actuator_cmd  (target:=car)   └─► /control/turtle/.. ──► turtlesim_adapter
```

## What each one checks

| Node | Reads | Says no when |
|---|---|---|
| `sensor_mux` (`source:=`) | `/system/source`; who publishes each sensor topic (the ROS graph) | any announcement isn't `source`; a sensor topic has **two** publishers; in a simulator, a sensor nobody announces (a real driver?) |
| `output_mux` (`target:=`) | ego's command; the sensor mux's verdict; `/system/source` | the sensor mux says no, checks for another source, or goes quiet for 3 s; any announcement isn't the target's source |

Both forget an announcement once the node that made it has gone. The real car's stock
drivers (u-blox, Velodyne, usb_cam) can't announce, so an unannounced sensor counts as
REAL. Every stand-in announces (Gazebo, the bench's mocks, `spine_bridge` off the car, a
replay), so it can't pass for real.

## Open, wait, stop

| State on `/safety/output_mux/state` | Means | Commands |
|---|---|---|
| `WAIT` | **doubt**: the sensor mux not heard yet, or quiet for 3 s; or it's unsure; or 2 s of all clear not over | none, then by itself again |
| `OPEN` | every source is the target's | ego's, unchanged |
| `STOP` | **evidence**: an announcement of another source, or the sensor mux's problem. Latched, with the reason | none, until `/safety/output_mux/clear` |

Evidence and doubt:

| Finding | Kind | Why |
|---|---|---|
| `/system/source` says SIM, BENCH or REPLAY (on the car) | evidence, at once | a stand-in says so itself |
| Two publishers on one sensor topic | doubt; evidence once it lasts 5 s | discovery can show a node before its name |
| In a simulator, a sensor nobody announces | doubt; evidence once it lasts 5 s | a node can publish before its announcement arrives |
| The sensor mux quiet for 3 s | doubt | a busy computer can delay it; it never latches |
| A node on `/system/source` whose announcement hasn't arrived | doubt; evidence once it lasts 5 s | the graph can show it before its message (up to 30 ms across processes, measured 24 Oct); the message may say SIM |
| An announcement whose node isn't in the graph yet | counts at once (for up to 10 s) | the message can come first; only once its node has been seen and gone is it forgotten |

## Arming waits for OPEN

Until the gate is `OPEN`, `spine_bridge` gets no commands, so an armed spine's 100 ms
watchdog would latch it again at once. So `spine_bridge` **refuses to arm** while the
gate is shut (Ben, 24 Oct), and says why.

| `/spine/arm` (true) when the output mux... | Answer |
|---|---|
| says `OPEN` for this bridge's target, within 1.5 s | `armed` |
| says `WAIT` or `STOP` | refused: `the safety gate is shut: <its line>; arm once it says OPEN` |
| hasn't been heard, or is quiet for 1.5 s (`gate_timeout`) | refused: no word, or quiet for so long |
| says `OPEN` for another target (`gazebo`, on the car) | refused: not this bridge's gate |
| says anything, during an e-stop | refused: the e-stop first |

| The bridge's `source:=` | Its target | Its gate |
|---|---|---|
| `real` | `car` | `OPEN - car: ...` |
| `bench` | `bench` | `OPEN - bench: ...` |
| `sim` (Gazebo, `spine:=true`) | `gazebo` | `OPEN - gazebo: ...` |

- Disarming always works.
- Both bridges, Rust and Python, say the same words (`spine-bridge`'s `gate.rs`,
  `aav_spine/output_gate.py`). Their clock for "quiet" is a steady one, not Gazebo's.
- `role:=sensors` has no `/spine/arm`, so nothing to refuse.
- `legacy_uart_bridge` (today's car, through the Motor Pi) has no arm step: the output
  mux's silence makes it brake after 0.3 s.
- The bench and Gazebo (`arm:=true`) start `arm_when_open` (`aav_spine`) after 12 s. It
  asks each second until the gate opens, then exits. It gives up after 120 s, or at once
  on an e-stop. `bench_check.py` and `sim_check.py` also wait for OPEN before they arm.

When it sends nothing, each backend's own timeout brakes, as when ego stops: `spine_bridge`
after 0.1 s (then the spine's 100 ms watchdog), `legacy_uart_bridge` after 0.3 s, Gazebo
after 0.2 s. The clear service refuses while anything is still wrong, and says what.

## One launch, `target:=`

```text
ros2 launch aav_bringup stack.launch.py target:=gazebo world:=road
```

| `target:=` | Starts | Source |
|---|---|---|
| `turtle` | `turtle.launch.py` | sim |
| `gazebo` | `sim.launch.py` | sim |
| `bench` | `bench.launch.py` | bench |
| `car` | `car.launch.py` (with `machine:=`) | real |
| `carla` | `carla.launch.py` | sim |

`source:=` may only say the target's own: `target:=car source:=sim` stops before anything
starts. The `pixi run` tasks start the same files directly, each with its two muxes. On
the car the muxes run next to ego (its `ego` part).

## Why check, not copy

The proposal's sensor mux picks one source per sensor and republishes it. We check
instead, and let each launch start only the chosen source:

| | Copy (republish) | Check (ours) |
|---|---|---|
| Cameras and point clouds | twice the bandwidth, and a hop of delay | nothing extra: the data isn't touched |
| A stray publisher on `/sensors/...` | still gets through, beside the copy | caught: two publishers |
| If the mux dies | the sensors stop (the watchdog stops Nav2) | the output mux stops the car within 3 s |
| Picks a source while running | yes | no: the launch picks it (`target:=`, `source:=`) |

The output mux itself is one more hop for ego's commands. Measured on the bench (23 Oct,
a Mac at load 30-45): the gaps between commands were the same before and after it
(median 19-21 ms); the long gaps at start-up were ego's own, before the hop.

## The heartbeat to the ESP32 (24 Oct)

TODO 50 (Ben, 24 Oct). With each command it passes on, the output mux also sends a
counter, +1 each time, on `/control/<target>/heartbeat`. So the counter moves **only
while the gate is OPEN**.

```text
 output_mux ── /control/car/actuator_cmd ──► legacy_uart_bridge ── UART, v2 frames ──► ESP32
  (OPEN)    └─ /control/car/heartbeat ─────►   (Motor Pi: copies     counter frozen
               counter 1, 2 ... 255, 0          the counter)          100 ms ─► brake
```

| The gate | The counter | The ESP32 |
|---|---|---|
| OPEN, ego at 50 Hz | moves | drives |
| WAIT or STOP | stops | brakes 100 ms later, latched |
| the Orin, its network or ego gone | stops (the Pi keeps sending, with the old counter) | brakes 100 ms later, latched |

The Pi only copies it, so a Pi cut off from the Orin can't keep the car going. Only the
real car's `legacy_uart_bridge` reads it; `spine_bridge`'s DRIVE_CMD has a counter of its
own. The frame: [the heartbeat](rust-firmware.md#the-heartbeat-version-2-of-the-serial-frame-24-oct).

## What it can't catch

- A stand-in that announces nothing and runs **instead of** the real driver (one
  publisher, unannounced): it looks real. Every stand-in of ours announces.
- Ghost obstacles: they only add an obstacle to Nav2's view, never a sensor, so they're
  allowed on the car ([ghost obstacles](ghost-obstacles.md)).
- The first 4 s after start: the sensor mux stays quiet 2 s while discovery settles, and
  the output mux waits 2 s more. Nothing drives in that time.
- A simulator started **later**, on the car's network: commands flow until its
  announcement arrives, then stop at once (latched). The guard test checks it takes
  under 1 s.

!!! warning "Two runs on one ROS domain see each other"
    On 24 Oct a guard test's REAL `spine_bridge`, on the same random domain as a Gazebo
    check (193), latched that check's output mux. The guard test, `sim_check.py` and
    `bench_check.py` now take a domain nobody on the computer uses
    (`aav_io_mux/domains.py`: a probe node sees no other node or topic for 1.5 s). The
    guard test also fails, naming them, if another run's nodes turn up later.

## Tests

| Test | Checks |
|---|---|
| `aav_io_mux/test/test_provenance.py` | the rules, no ROS: 34 cases, with the graph and the announcement in either order |
| `aav_io_mux/test/test_real_backend_guard.py` | the real nodes with `spine_bridge` (source real), Rust and Python: all REAL, its DRIVE_CMD frames flow; Gazebo announcing SIM, **none**, and `/spine/arm` refused with the STOP's reason; Gazebo gone and the stop cleared, `OPEN`, and it arms (armed frames follow); Gazebo starting late, the frames stop within 1 s; a node on `/system/source` not heard yet, `WAIT` and no frames, then `STOP` once it says SIM; and `spine_bridge` never listens to `/ego/actuator_cmd`. With `role:=sensors` (today's wiring: the Orin's CAN bus has only sensors), the mux still opens, not one frame reaches `/can/tx`, and it announces only the wheels |
| `spine-bridge`'s `tests/gate.rs`, `aav_spine/test/test_output_gate.py` | the arming rule, no ROS: not heard, WAIT, STOP, quiet, another target, OPEN; the same cases in both languages |
| `aav_bringup/test/test_stack_launch.py` | `target:=` picks the profile; a wrong `source:=` is refused; every profile runs the two muxes with its own target |
| `pixi run sim-check`, `pixi run bench-check` | the whole graph with both muxes: contract, drive, e-stops. Passed 23 Oct |
| the turtle (`pixi run turtle`) | OPEN; a scripted drive moved the turtle at 0.86 m/s |
| `aav_io_mux/test/test_heartbeat.py`, `aav_legacy/test/test_heartbeat_chain.py` | the counter: +1 a command, wraps in a byte; the muxes and `legacy_uart_bridge` on a pseudo-terminal: the frames carry the mux's counter, frozen when ego stops; a shut gate sends **no** heartbeat |
