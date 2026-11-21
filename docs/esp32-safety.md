# The ESP32's safety rules

What the ESP32 control board does to keep the car still: how it boots, what lets it drive,
what counts as a lost link, and what each failure upstream does. Every line is checked
against the code; the proofs are in the last table.

!!! warning "Which firmware this is about"
    - **The car runs the 2025-26 C++ today** (`legacy.yaml`: `frame_version: 1`). It has
      none of the timeouts below. What it does instead: [section 8](#8-the-2025-26-c-and-open-questions).
    - Sections 2 to 7 are the **Rust firmware** (`firmware/spine/boards/esp32-umod`).
      [[untested]] on the car: **no record of a flash** (the board ran the C++ on 6 Nov). Proven in `cargo test` and in the
      emulator (QEMU) only.

## 1. The chain

```text
 joystick ─┐  (USB on a laptop, or on the Motor Pi)
 keyboard ─┤
 hw_panel ─┤  slot 45, only while GO is held
 RC via ROS┤  slot 95 (rc_driver, from the uplink)
 nav ...   ┘
     │
     ▼
 twist_mux ──► ego ──────► output mux ──────────────► legacy_uart_bridge ──UART──► ESP32 ──I2C──► steering Jrk
 picks one     50 Hz,      passes ego's command,        (on the Motor Pi)    50 Hz            ├──► brake Jrk
 (0.5 s per    always,     and +1 on the HEARTBEAT      copies the counter                    └──► DAC (throttle)
  slot)        stops on    counter, only while its      into each frame                            + direction pin
               0.3 s quiet gate is OPEN

 RC receiver ──iBUS──────────────────────────────────────────────────────────► ESP32 (RC direct: no Pi, no Orin)

 red mushroom button, Kar-Tech wireless stop ──► power, by hardware. Outside the firmware.
```

| Link | What it carries | Who makes it |
|---|---|---|
| `/control/car/actuator_cmd` | ego's speed and steering | the output mux |
| `/control/car/heartbeat` | the counter, +1 per command passed on | the output mux, only |
| the serial frame (version 2) | values, the counter, `armed` and `estop` flags, a CRC | the bridge: it **copies** the counter, never makes one |
| iBUS | the RC sticks and switches | the RC receiver |

**Rust by default** (Ben, 17 Nov: "Rust tomorrow"): the output mux (`aav_io_mux_rs`),
the bridge (`aav_legacy_rs`) and `rc_driver` (`aav_rc_rs`) run in Rust; each
`*_impl:=python` falls back. The bridge packs its frames with the firmware's own
`legacy-link`, so the Pi and the ESP32 share one copy of the bytes.

**The heartbeat is the output mux's counter.** Wherever the output mux runs (the Orin,
or the Motor Pi with `machine:=motor_pi_alone`), that machine is the heartbeat.

## 2. How it boots

| Output | At reset (`init`) | Nobody driving, after | Latched? |
|---|---|---|---|
| Throttle (DAC) | 0 | 0 | |
| Brake | full (3500) | full: this board sets `IDLE_BRAKE = 1.0` | |
| Direction | forward | forward | |
| Steering | centre (1950) | held where it is (centre, after boot) | |
| Mode | `Manual` (nobody) | `Manual` | **yes**: the computer must stop, then drive |
| RC mode switch | taken as Computer until the RC is heard | | |

| Step | What | Time |
|---|---|---|
| 1 | DAC 0, brake full, direction, steering centre: in that order | milliseconds |
| 2 | Each I2C device probed (read only) | about 1 ms |
| 3 | Self-test: no fault from the Jrks or the DAC. Nothing can arm before | the first tick |
| 4 | Running: waits for a stop frame (section 3) | |

A hang or a panic: the hardware watchdog (100 ms) resets the chip, back to step 1.

`spine-core`'s own `IDLE_BRAKE` is 0.0 (brake released); the ESP32 board overrides it
with 1.0 until the team decides (section 8).

## 3. What releases the brake

### Computer (joystick, keyboard, navigation through ROS)

```text
 boot ─► latched ─► a STOP frame with a new counter ─► unlatched ─► a DRIVE frame, new counter ─► drives
                    (ego says stand still: the hold                   (speed asked, no brake:
                     brake, or the armed flag clear)                   the brake lets go)
```

| # | Needed | How you get it |
|---|---|---|
| 1 | The self-test passed | every Jrk and the DAC answer |
| 2 | No e-stop | `/safety/estop` false (`pixi run clear estop`) |
| 3 | The counter moving | the output mux running, its gate OPEN (2 s of all clear) |
| 4 | A stop frame | ego standing still: start with the joystick's trigger **released** |
| 5 | A drive frame | trigger held, stick forward |
| 6 | RC switch on Computer or RC via ROS | |

The bridge sends stop frames by itself whenever ego is quiet, so steps 4 and 5 happen
by themselves once ROS runs. After any timeout: the same, from step 3.

### RC via ROS (CH6 middle)

The same as Computer, plus:

| Needed | Why |
|---|---|
| The RC signal stays | losing it is a hard stop |
| The uplink wire (GPIO17 to the Pi, H9) and `uplink: true` | the sticks go up on it |
| `rc_driver` running (with the bridge, `rc:=true`) | it turns the sticks into `/drivers/rc/cmd_vel` |
| The heartbeat | the command still goes through ego and the output mux |

### RC direct (CH6 high)

| Needed | Not needed |
|---|---|
| RC signal healthy | the Motor Pi, the Orin, any frame |
| No e-stop, no driver fault | arming |
| After any stop, and after power-up: signal back, throttle stick at zero, gear switch on brake (7 Nov) | |

| Stick and gear | Brake |
|---|---|
| Gear forward or reverse, throttle pushed | released |
| Throttle at zero | 0.33 for 1 s, then full |
| Gear on brake (middle) | full |

## 4. What counts as a lost connection

| What | Noticed after | The car | Latched? | To clear |
|---|---|---|---|---|
| No frames (Pi, bridge or wire gone) | 100 ms (QEMU: 111-120 ms) | throttle 0, brake full, fault 1 | yes | frames, a stop, a drive |
| Frames come, **counter frozen** (Orin or mux gone) | 100 ms (QEMU: 113-118 ms) | the same | yes | the same |
| Bad CRC | each frame ignored; fault bit 2 shown | all bad: as "no frames" | as no frames | fix the wire |
| Wrong version (13-byte frames into this build) | ignored | never drives | | `frame_version: 2` |
| A value outside -1..1 | that frame | throttle 0, the rest kept | no | |
| RC lost (silent 100 ms, or CH6 out of range), on RC via ROS or RC direct | 100 ms (QEMU: 117-119 ms) | brake full, fault 256 | yes | computer: stop, drive. RC direct: stick zero + gear on brake |
| RC lost, switch on Computer | | **nothing** | | |
| Wheel speed lost (300 ms) | only once a frame has sent one | brake full, fault 64 | yes | |
| Over 1.25 m/s (0.75 back) | at once | throttle 0, brake 0.5 until under 1.0 | no | by itself |
| Still over the limit 2 s later | 2 s | brake full, fault 512 | yes | |
| Driver fault (a Jrk, the DAC, the expander) | that tick | brake full, fault 8, 16 or 32 | yes | computer: stop, drive. RC direct: stick zero + gear on brake |
| E-stop flag (`/safety/estop`) | the next frame, whatever its counter | brake full | yes | a new counter with the flag clear, then stop, drive |

!!! note "The speed rules are off on the car"
    The wheel speed comes only in version 4 frames, and the Motor Pi doesn't send them
    yet. So no overspeed check and no lost-speed stop on the ESP32. The only limit is the
    open-loop throttle cap: `max_speed / full_throttle_speed` = 1.0 / 3.0, and 3.0 is a
    guess (Q4).

## 5. What each failure upstream does

| Fails | First to notice | Time | The car |
|---|---|---|---|
| The Orin, or its network (Orin + Motor Pi set-up) | the ESP32: counter frozen | 100 ms | full brake, latched |
| The output mux, or its gate not OPEN | the ESP32: no new counter | 100 ms | full brake; never arms |
| ego | the output mux gets nothing: no beat | 100 ms | full brake, latched |
| twist_mux | ego: no command for 0.3 s | 0.3 s | ego slows to 0; hold brake. **No fault**: the counter still moves |
| The joystick unplugged, or its node dead | `joy_alive` SILENT 0.5 s, twist_mux 0.5 s, ego 0.3 s | about 1 s | ego slows to 0 |
| The joystick's values frozen while driving | `joy_alive` FROZEN | 2 s | ego slows to 0 |
| Any driver's slot quiet | twist_mux | 0.5 s | the next driver, or ego slows to 0 |
| hw_panel closed, or its window frozen | it sends only while it draws | 0.5 s | as any quiet slot |
| The laptop's WiFi (running the keyboard or hw_panel) | twist_mux | 0.5 s | ego slows to 0 |
| The Motor Pi | the ESP32: no frames | 100 ms | full brake, latched |
| The bridge, Ctrl-C | it sends 5 stop frames as it ends | at once | braked |
| The bridge, crashed | the ESP32: no frames | 100 ms | full brake, latched |
| The serial wire | the ESP32: no frames | 100 ms | full brake, latched |
| The uplink wire (GPIO17) | the bridge warns after 0.2 s | | **nothing stops**; RC via ROS can't work |

```text
 a person or program goes quiet ──► ego slows the car (counter keeps moving, no fault)
 ego / mux / Orin / Pi / wire  ──► the ESP32 brakes hard in ~0.1 s and latches
```

## 6. Joystick and RC

### Does the joystick need the Orin?

**No, not if the Motor Pi runs everything** (`machine:=motor_pi_alone`). Then the Pi is
the heartbeat, and the Orin must **not** run `car orin` (two muxes, two egos, both would
drive).

| Set-up | Joystick plugged into | Heartbeat from | The Orin |
|---|---|---|---|
| `motor_pi_alone` (car day A9b) | the Motor Pi | the Motor Pi | off, or not running our stack |
| Orin + Motor Pi (`machine:=orin`, `car motor_pi`) | a laptop (`pixi run car laptop`) | the Orin | **needed**: it down = full brake in 100 ms |

The joystick going quiet is **not** a firmware stop: ego slows the car (section 5). The
ESP32 brakes on its own only when ego, the mux, the Pi or the wire stops.

### RC direct

Works with **no Pi and no Orin**. The ESP32 reads the receiver itself.

| Risk (Arsh, PR #23) | What the code does |
|---|---|
| The transmitter turned off while its switch is on Computer | nothing: a lost RC on Computer changes nothing. It **can't stop autonomy** |
| The Orin or Pi goes quiet during RC direct | RC keeps driving: its own input is fine. The computer latches and must re-arm later |
| Once the wheel speed is sent: a 0.3 s network gap | a hard stop and latch, even in RC direct |

## 7. The laptop as the heartbeat

| Question | True in the code today |
|---|---|
| Does hw_panel's `/hud/hw_panel/heartbeat` reach the ESP32? | **No.** The bridge reads only `/control/car/heartbeat`. The output mux reads screens only with `screens:=`, and refuses it for the car (the node won't start) |
| Can a laptop run the output mux for the real car? | **Yes, since 20 Nov**: `pixi run car laptop_orin` (below). Never tried on the car |
| What a laptop can do | drive (`pixi run keyboard`), watch (`pixi run tools/ros.sh ros2 launch aav_hud hw_panel.launch.py stands:=true`; not `pixi run hw-panel`, which stays on the laptop), e-stop |
| hw_panel's speed buttons on the real car | **off** (on stands too, until a driven-wheel speed exists, TODO 49b) |

### The command that works today, with no Orin

[[untested]] on the car. With the Rust firmware and `frame_version: 2`:

| Where | Run |
|---|---|
| the Motor Pi | `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=motor_pi_alone max_speed:=0.3` |
| the laptop (to drive) | `export ROS_DOMAIN_ID=42`, then `pixi run keyboard` |
| the laptop (to watch) | `export ROS_DOMAIN_ID=42`, then `pixi run tools/ros.sh ros2 launch aav_hud hw_panel.launch.py stands:=true` (`pixi run hw-panel` is localhost-only: it never sees the car) |

The heartbeat is made on the Motor Pi, beside the ESP32. The laptop can drop off: its
slot times out and ego stops the car.

### The laptop as the Orin (20 Nov)

Ben, 20 Nov: "make the joystick and the remote control drive the car and the hardware
screen ... provide the heartbeat". [[untested]] on the car.

```text
laptop: joystick ─► mux ─► ego ─► output mux ── WiFi ──► Motor Pi: bridge ─UART─► ESP32
        hw_panel ─► (45)          (the heartbeat)
RC transmitter ──────────────────────────────────────────────────── iBUS ─────► ESP32
```

| Where | Run |
|---|---|
| the Motor Pi | `~/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=motor_pi` (the bridge only) |
| the laptop, joystick plugged in | `export ROS_DOMAIN_ID=42`, then `pixi run tools/ros.sh ros2 launch aav_bringup car.launch.py machine:=laptop_orin max_speed:=0.3` |
| the laptop, the hardware screen | `export ROS_DOMAIN_ID=42`, then `pixi run tools/ros.sh ros2 launch aav_hud hw_panel.launch.py stands:=true` |

| If | The car |
|---|---|
| the laptop, the WiFi, the Pi or the wire stops for 100 ms | Rust firmware: full brake, latched. **A WiFi hiccup is a hard stop** (`motor_pi_alone` keeps the heartbeat off the WiFi) |
| the joystick goes quiet | ego slows the car (its slot times out) |
| the RC's CH6 goes to RC direct | the sticks drive, with no laptop and no Pi |
| the C++ firmware is still on | no heartbeat at all: stands only |

Never with `car orin` or `car motor_pi_alone` running too: two muxes and two egos would
both drive.

With the Orin reading the sensors and a recording: [drive by hand and record](drive-and-record.md).

## 8. The 2025-26 C++ and open questions

### What the C++ (on the car today) does differently

| | C++ (today) | Rust |
|---|---|---|
| Boot | WiFi first (up to 10 s), then DAC 0, steering centre, brake full | safe in milliseconds |
| First frame after boot | drives | latched until a stop frame |
| Frames stop | the last throttle stays on | full brake in 100 ms |
| Counter, CRC | none: a sum | heartbeat counter, CRC-8 |
| RC receiver silent | the last values stay | lost after 100 ms: full brake |
| A Jrk missing at boot | skipped; the car still drives | its fault stops the car |
| Jrk or DAC errors | printed | full brake |
| E-stop | the button cuts power | the same; the spare-contact input is off until wired (Q6) |
| Over the air | an HTTP firmware update page on its WiFi | none: the WiFi status build only reads |

With the C++, the Orin going down is still covered: the bridge runs on the Pi and sends
brake frames 0.3 s after ego goes quiet. The Pi, the bridge or the wire going down is not.
So the bridge locks every program out (`/safety/legacy_uart_bridge/lock`), and the
joystick is for stands only.

### Open questions

| # | Question | For | Recommendation |
|---|---|---|---|
| 1 | ✅ **Fixed 7 Nov:** RC direct drove again at once after an e-stop or a driver fault cleared, with the stick still pushed. Now any stop holds it (`rc_hold`) until the stick is at zero and the gear on brake. Test `rc_direct_waits_for_a_person_after_an_estop_or_a_driver_fault_clears` | Arsh to review | done |
| 2 | ✅ **Fixed 7 Nov, by the same change:** the board starts with its e-stop input on, so after power-up or a reset RC direct waits for the stick at zero and the gear on brake | Arsh to review | done |
| 3 | What does the receiver send when the transmitter is off (its failsafe, Q7)? If it keeps sending, the ESP32 never sees a lost RC | hardware lead | test bench check 11; set the failsafe to throttle zero and gear on brake |
| 4 | The transmitter off on Computer can't stop the car | Ben | the Kar-Tech stop is the stop; never count on the transmitter |
| 5 | No wheel speed to the ESP32, so no overspeed check | Arsh | send version 4 frames once a rear-wheel speed exists (TODO 49b); keep `max_speed:=0.3` until then |
| 6 | `IDLE_BRAKE`: 1.0 (held) or 0.0 (released) | Ben | keep 1.0 until the e-stop input is wired (Q6) |
| 7 | What the Kar-Tech and the red button cut, and what the brake does with no power (S1, S2) | hardware lead | press each on stands, write it down |
| 8 | Two output muxes at once (Orin and Pi, or a laptop) both drive, and their counters mix, so the heartbeat never freezes | Ben | warn when a second publisher appears on `/control/car/heartbeat` |
| 9 | A laptop as the heartbeat | Ben | no: use `motor_pi_alone` (wired, beside the ESP32) |

## Where it's proved

`spine.rs`, `fsm.rs`: `firmware/spine/crates/spine-core/src/`. `link.rs`, `health.rs`,
`actuators.rs`, `spine_board.rs`, `calibration.rs`: `firmware/spine/crates/spine-board/src/`.
`legacy_uart_bridge.py`, `legacy_protocol.py`: `ros/can_bridge/aav_legacy/aav_legacy/` (the Python
fall-back). `relay.rs`: `ros/can_bridge/aav_legacy_rs/crates/legacy-bridge/src/`; `main.rs`:
`ros/can_bridge/aav_legacy_rs/crates/legacy-node/src/` (the Rust bridge, the default). The
two give the same frames, byte for byte: `legacy-bridge`'s `tests/vectors.rs`.

| Statement | Code | Test |
|---|---|---|
| Boots latched | `fsm.rs:122-134`, `spine.rs:373` | `boots_latched_until_disarm_then_arm`; `armed_frames_after_a_reboot_do_not_drive` |
| Reset order: DAC 0, brake, direction, steering | `actuators.rs:128-133` | `init_cuts_the_throttle_then_brakes_then_sets_up` |
| `IDLE_BRAKE` 1.0 on the ESP32, 0.0 in spine-core | `boards/esp32-umod/src/main.rs:142`, `spine.rs:82` | `the_idle_brake_after_boot`; QEMU first check: "Brake PASS 1.00" (`esp32-bench.md`) |
| Self-test before arming | `fsm.rs:44-51`, `spine.rs:578-583` | `the_self_test_waits_for_the_devices_and_nothing_arms_before_it` |
| Release: a stop, then a command | `fsm.rs:168-177`, `spine.rs:498-505` | `direct_commands_need_a_stop_first`; `drive_frames_after_a_reboot_do_not_drive_until_a_stop_frame` |
| Ego standing still is a stop frame | `legacy_protocol.py:127-128` (hold brake), `legacy-link/src/targets.rs:63-67` (Cut) | `v2_after_a_heartbeat_loss_it_stays_braked_until_a_stop` |
| Only a changed counter is a command | `legacy-link/src/v2.rs:144`, `link.rs:460-463` | `v2_a_frozen_counter_brakes_within_the_timeout_though_frames_still_come` |
| 100 ms timeout, latched | `spine.rs:182`, `spine.rs:571-575` | `lost_commands_brake_and_latch`; QEMU 111-120 ms (`esp32-bench.md`) |
| Bad CRC ignored | `link.rs:430-434` | `v2_bad_crc_frames_are_ignored_so_the_heartbeat_stops` |
| 13-byte frames never drive this build | `link.rs:425-429` | `v2_ignores_version_1_frames_so_never_drives` |
| E-stop flag at once; released by a new counter | `link.rs:450-469` | `v2_the_estop_flag_works_whatever_the_counter` |
| RC lost: 100 ms; only on an RC position | `calibration.rs:147`, `spine.rs:528-531` | `a_silent_receiver_stops_rc`; `rc_lost_on_the_computer_position_changes_nothing`; QEMU 117-119 ms |
| RC direct's own latch | `spine.rs:556-563`, `spine.rs:657` | `rc_direct_stays_stopped_after_a_lost_signal_until_a_person_clears_it` |
| RC direct beats the computer; the computer still latches | `spine.rs:571-575`, `spine.rs:623-624` | `the_computer_stopping_while_rc_direct_drives_still_latches` |
| RC zero stick: 0.33 then full | `spine.rs:192-193`, `spine.rs:750-760` | `rc_at_zero_throttle_stops_lightly_for_a_second_then_holds` |
| Wheel speed rules off until a speed comes | `spine.rs:433-455` | `a_link_that_never_sends_a_speed_leaves_the_rules_off` |
| Overspeed: 1.25 m/s, 0.5 brake, 2 s | `spine.rs:189-190`, `fsm.rs:181`, `spine.rs:540-555` | `overspeed_cuts_the_throttle_and_brakes_until_under_the_limit_then_drives_on`; `an_overspeed_that_holds_steady_for_two_seconds_is_a_hard_stop_and_latches` |
| Driver fault stops; latches the computer | `spine.rs:532-535` | `driver_fault_stops_and_needs_rearm`; `driver_fault_stops_rc_too` |
| Open questions 1 and 2, fixed: any stop holds RC direct | `spine.rs` (`rc_hold` set on an e-stop and a driver fault too) | `rc_direct_waits_for_a_person_after_an_estop_or_a_driver_fault_clears`; the board's `rc_drives_without_the_computer`; QEMU's `direct` |
| The board's link verdict (OK, STALE, COUNTER FROZEN) | `health.rs:6-12` | `frames_with_a_frozen_counter_say_counter_frozen` |
| The counter: +1 per command, only while OPEN | `ros/io_mux/aav_io_mux/aav_io_mux/output_mux.py:109-112` | `test_heartbeat.py`: `test_it_counts_each_command_and_wraps_in_a_byte`, `test_it_never_moves_by_itself` |
| The bridge leaves 5 stop frames as it ends | `main.rs` (`leave_braked`); `legacy_uart_bridge.py:350-358` | |
| The joystick's SILENT 0.5 s, FROZEN 2 s | `ros/drivers/aav_teleop/config/joy_alive.yaml:10-15`, `ros/safety/aav_safety/aav_safety/joy_alive.py` | `test_joy_alive.py`: `test_frozen_values_while_driving_stop_after_frozen_after` |
| The bridge copies, never makes, the counter | `relay.rs` (`heartbeat`, `frame`); `ros/can_bridge/aav_legacy/aav_legacy/uart_relay.py:93`, `:164-185` | `the_counter_is_relayed_never_made_up`; `test_the_counter_moves_only_when_the_orin_sends_one`; `test_when_ego_goes_quiet_it_sends_stops_with_the_last_counter` |
| The bridge reads only `/control/car/heartbeat` | `main.rs` (`control/car/heartbeat`); `legacy_uart_bridge.py:150` | `test_heartbeat_chain.py` (both bridges; the Rust one with the Rust muxes) |
| The car refuses `screens:=` | `ros/io_mux/aav_io_mux/aav_io_mux/provenance.py:278-281` | `test_the_real_car_refuses_the_screens_rule` |
| ego sends at 50 Hz and stops on 0.3 s of quiet | `ros/ego/aav_ego/aav_ego/ego_node.py:112-130`, `ego.yaml:19` | |
| twist_mux: 0.5 s per slot, hw_panel at 45 | `ros/command_mux/aav_command_mux/config/twist_mux.yaml` | |
| hw_panel sends only while it draws | `ros/hud/aav_hud/crates/panel-app/src/ros.rs:1-13` | `sends_only_while_the_window_draws_and_the_estop_is_five_trues` |
| hw_panel speed buttons off on the real car | `ros/hud/aav_hud/crates/panel-core/src/gate.rs:1-8` | `the_preflight_refuses_the_real_car_without_a_driven_wheel_speed` |
| `motor_pi_alone`: mux, ego, joystick, bridge on the Pi | `ros/bringup/aav_bringup/config/machines/motor_pi_alone.yaml` | `test_every_machine_file_loads_and_starts` (launch only) |
| `laptop.yaml`: joystick and detector only | `ros/bringup/aav_bringup/config/machines/laptop.yaml` | |
| The car's file says version 1, uplink off | `ros/can_bridge/aav_legacy/config/legacy.yaml` | |
| C++: no timeout, holds the last values | `firmware/legacy/aav_umod_esp32/src/main.cpp:162-186`, `modules/ros2_handler/ros2_handler.cpp` | `tools/tests/test_legacy_cpp.py` (the frame only) |
| C++: OTA update page | `firmware/legacy/aav_umod_esp32/src/main.cpp:64` | |
| No e-stop input on the ESP32 yet | `boards/esp32-umod/src/main.rs:200-210` | |
