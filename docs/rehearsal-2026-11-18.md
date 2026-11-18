# Rehearsal for 18 Nov: the Motor Pi alone, on a Mac

Ben's session, the night of 17 Nov. Tomorrow's path, run on a MacBook with the Rust
ESP32 firmware in QEMU instead of the board. Then a checklist for the morning.

!!! warning "QEMU is not the car"
    The firmware ran in Espressif's QEMU with stand-in I2C devices. No real UART, Jrk,
    DAC, joystick or RC receiver was touched. The list is in
    [What was not rehearsed](#what-was-not-rehearsed).

## What ran

```text
pretend keyboard, joystick ──► car.launch.py machine:=motor_pi_alone max_speed:=0.3
  (this Mac's ROS, domain 207,     twist_mux ► ego ► output mux ► legacy_uart_bridge
   LOCALHOST only)                                                    │ socket://127.0.0.1:47231
                                                                       ▼
iBUS frames (pretend RC) ──► QEMU UART2      QEMU UART1 ◄──► the Rust ESP32 firmware
                                              QEMU UART0 ──► its log (EVENT, STATUS, LINK)
```

| Part | As tomorrow? |
|---|---|
| The launch: `car.launch.py machine:=motor_pi_alone max_speed:=0.3` | yes, the same file and nodes; Rust by default |
| `legacy.yaml`: `frame_version: 2`; `uplink: true` (and once `false`) | yes, but `port: socket://127.0.0.1:47231` in place of `/dev/ttyAMA0` (put back after) |
| The firmware: `--features qemu,uplink`, the build `pixi run esp qemu-check` makes | the logic, yes; the devices are QEMU's stand-ins |
| QEMU's UART1 | `-serial tcp:127.0.0.1:47231,server=on,wait=off`: the bridge's `socket://` reaches it |
| The keyboard | `TwistStamped` on `/drivers/keyboard/cmd_vel` at 30 Hz, 0.5 m/s, as `teleop_twist_keyboard` sends |
| The joystick | `sensor_msgs/Joy` on `/drivers/joy/joy` at 20 Hz: through `joy_alive` and teleop, as `joy_node` sends |
| The RC | iBUS frames into QEMU's UART2; the firmware's uplink; `/vehicle/rc`; `rc_driver` |
| The Python fall-backs | `io_mux_impl:=python legacy_impl:=python rc_impl:=python`, the same checks |

Nothing opened `/dev/ttyAMA0` or any device: the bridge logged
`serial port socket://127.0.0.1:47231 open`. The driving script was a one-off, not kept.

## Results

Times are from the moment named to the firmware's `EVENT` line saying it (its own clock).

| # | Check | Rust (default) | Python fall-backs | Result |
|---|---|---|---|---|
| 1 | The stock `pixi run esp qemu-check` | brakes 110-115 ms after the heartbeat stops; uplink, RC modes | - | PASS |
| 2 | Start: the mux WAITs, then OPEN | counter frozen while WAIT (199 repeats); OPEN: 190 new counters in 4 s, 0 repeats | the same | PASS |
| 3 | Idle, nobody driving | armed, throttle 0, hold brake 0.21 (about 1050 counts) | the same | PASS |
| 4 | Keyboard forward (0.5 m/s asked) | ego 0.3 m/s, frame speed 0.10, firmware throttle 0.10 | the same | PASS |
| 5 | Keyboard forward-left | ROS steering +0.5 rad, ESP32 -0.5 (`steering_sign: -1`) | the same | PASS (which way is left: on stands, Q2) |
| 6 | Keyboard reverse | **0.5 m/s** (throttle 0.17): the cap was forward only | - | **FAIL, fixed**: now 0.3 m/s, throttle 0.10 |
| 7 | Let go of the key | throttle 0 after 0.40 s, then the hold brake | 0.39 s | PASS (car day 3.4 said 0.3 s: corrected) |
| 8 | E-stop while driving | full brake in 20 ms; stays braked while keys still ask; drives again after the clear | 7 ms | PASS |
| 9 | Joystick: push, ease off | 0.3 m/s, then 0.3 m/s | the same | PASS |
| 10 | Joystick: pull back | ego slows to 0, hold brake **80 ms**, then reverses | the same | PASS; open problem 1, **fixed** after: now holds 1.0 s |
| 11 | Joystick: hold back | reverse at 0.3 m/s (after the fix) | the same | PASS |
| 12 | Joystick goes silent while driving | `SILENT`; throttle 0 after 0.39 s | 0.38 s | PASS |
| 13 | Joystick frozen while driving | `FROZEN` at 2.06 s; throttle 0 at 2.29 s | the same | PASS |
| 14 | RC via ROS, a little stick | `/vehicle/rc` switch 2; `rc_driver` 0.4 m/s; ego 0.3; throttle 0.10 | **no RC at all** (uplink stale) | Rust PASS; Python **FAIL, fixed**, then PASS |
| 15 | RC switch to computer, receiver off | stands still, no fault | the same | PASS |
| 16 | The laptop stops sending while driving | throttle 0 after 0.40 s | 0.40 s | PASS |
| 17 | Kill the output mux (SIGKILL, by PID) while driving | full brake, fault 1, in **116 ms**; counter frozen, frames still coming | 103 ms | PASS against car day's 0.2 s |
| 18 | Kill the bridge (SIGKILL) while driving | full brake, fault 1, in 96 ms | 103 ms | PASS |
| 19 | Ctrl-C the bridge while driving | full brake in 114 ms (fault 1) | 8 ms | PASS, see open problem 2 |
| 20 | A simulator announces itself on the car | mux STOP, latched; full brake in 119 ms | - | PASS |
| 21 | `frame_version: 2`, `uplink: false` | drives, capped; bridge killed: brake in 118 ms | - | PASS |
| 22 | The hardware panel's topics | all 14 of its Motor Pi topics published (sensors: none, as expected) | - | PASS (window not drawn) |
| 23 | `car_setup.sh motor_pi` (dry run, Mac) | one STOP: the upgrade without `--after-esp32`, as designed | - | PASS |
| 24 | `car_setup.sh motor_pi services --machine motor_pi_alone` (dry run) | the plan, no errors | - | PASS |
| 25 | `car_install.sh --dry-run` (Mac) | refuses: Darwin, arm64, no kit, as designed | - | PASS |
| 26 | `pixi run esp walk --dry-run` | all 28 steps to "All done" | - | PASS |

**About "within 100 ms":** the firmware brakes 100 ms after the last *new* counter, then
waits for its next 10 ms tick. So a dead mux brakes in about 100-120 ms, never under 100.
Car day's gate is 0.2 s.

## Fixed tonight

| Fix | Why | Proof |
|---|---|---|
| `max_speed:=` caps reverse too (`ego.launch.py`) | on stands "the speed capped at 0.3 m/s" (car day A9), but held back the car reversed at 0.5 m/s | `test_ego_launch.py`; rehearsal 6 and 11 at 0.3 m/s |
| The Python bridge reads the uplink in chunks | on `socket://`, pyserial's `in_waiting` says 0 or 1: one byte each 10 ms, so the uplink went stale and RC via ROS never drove. A real UART was fine | `test_uplink_socket.py`: 5 of 76 frames before, all after |
| The hardware panel from a laptop: `pixi run tools/ros.sh ros2 launch aav_hud hw_panel.launch.py stands:=true` | `pixi run hw-panel` is localhost-only: it never sees the Motor Pi (`esp32-safety.md` said it would) | read in `tools/screen.sh` |
| Car day 3.4: "throttle off within 0.5 s" | measured 0.4 s, not 0.3 | rehearsal 7 |

## Open problems (for the team)

| # | Problem | Why it matters | Suggestion |
|---|---|---|---|
| 1 | **Fixed (17 Nov).** Pull the stick back while moving: ego reached 0 by its *model*, held the brake 80 ms, then reversed | the Motor Pi alone has no wheel speed, so the direction could flip while the rear wheels still turn (the DAC is 0 then) | ego's pedal now holds 0, the brake on, for `pedal_shift_hold` (1.0 s, `ego.yaml`) before the other way; with a fresh `/vehicle/odom`, also until it reads under `pedal_stop_speed` (0.05 m/s). Python and Rust, shared vectors. On stands: time the wheels' coast-down in 3.2b and set the hold above it. Not re-rehearsed in QEMU |
| 2 | The bridge's five "leave braked" frames on exit repeat the last counter | with version 2 the ESP32 ignores a repeated counter, so they do nothing; its own 100 ms rule brakes it | harmless; car day 3.7 "a last stop frame" really means "brakes within 0.2 s" |
| 3 | A dead mux or bridge is not restarted | the car stays braked (safe), but nothing drives until the whole launch restarts | Ctrl-C the launch, start it again |

## What was not rehearsed

| Not tried | Tomorrow's check |
|---|---|
| The real UART: `/dev/ttyAMA0`, the console off it (A7b), the wire's speed | A7b L1-L5; `LINK frames=` rising, `bad=0` |
| The Jrks, the DAC, the direction pin (QEMU has stand-ins) | 6c, 1-7 |
| The real joystick: `joy_node`, its button and axis numbers | A9b step 4 |
| The real RC receiver (iBUS), and its three-position switch (H10) | 6c 10-12; 6e only if H9 and H10 say yes |
| The network: the laptop on the Pi's domain 42 (here all on one Mac) | `pixi run doctor`; `ros2 topic hz /control/car/link_frame` from the laptop |
| The Pi's own CPU (Ubuntu 22.04) running all of it | `top` on the Pi while driving |
| The HUD and the panel drawing on screen | open them on the laptop |
| The red button, the Kar-Tech stop | 6c 13, A9 3.8 |

## The morning checklist

One page. **Never with `car orin` running too:** two muxes would both drive.

| # | Where | Run | Expect |
|---|---|---|---|
| 1 | Laptop | `pixi run car-day-build` (or the night before) | QEMU check PASS |
| 2 | Motor Pi | `systemctl status aav_serial_bridge` | inactive, disabled (A7b) |
| 3 | Laptop | `pixi run esp walk <port>` | the Rust flashed; first check SAFE TO GO ON |
| 4 | Motor Pi | `legacy.yaml`: `frame_version: 2` (`uplink: true` only if H9 and H10 say yes), then `~/aav-jazzy/aav.sh build` | |
| 5 | Motor Pi | `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=motor_pi_alone max_speed:=0.3` | `serial port /dev/ttyAMA0 open`; `OPEN - car: every source REAL` |
| 6 | Motor Pi, 2nd terminal | `~/aav-jazzy/aav.sh ros2 topic hz /control/car/link_frame` | about 10 Hz |
| 7 | ESP32 log (laptop USB) | `LINK` lines | `frames=` rising, `bad=0`, `counter=` moving |
| 8 | Laptop | `export ROS_DOMAIN_ID=42`, then `pixi run keyboard` | wheels turn at 0.3 m/s at most, both ways |
| 9 | Laptop, to watch | `export ROS_DOMAIN_ID=42`, then `pixi run tools/ros.sh ros2 launch aav_hud hw_panel.launch.py stands:=true` | the panel sees the car |
| 10 | Stands | [Car day 6c](car-day.md#session-6-the-esp32), then [A9](car-day.md#session-9-the-first-drive-on-stands) 3.1-3.9, then [A9b](car-day.md#a9b-the-joystick-on-stands) | each as written |

If a Rust node misbehaves, restart the launch with its fall-back:

| Node | Add to step 5 |
|---|---|
| The muxes (output and sensor) | `io_mux_impl:=python` |
| The ESP32's UART bridge | `legacy_impl:=python` |
| The RC driver | `rc_impl:=python` |
| The joystick's check (joy_alive; Rust since after the rehearsal) | `joy_alive_impl:=python` |

With `aav.sh car motor_pi_alone`, put them after `--`: `~/aav-jazzy/aav.sh car motor_pi_alone -- legacy_impl:=python`.
Each passed the same checks tonight (the bridge's after its fix: pull tonight's commits first).
