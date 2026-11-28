# Firmware: the spine and its CAN contract

ROS stops at the CAN bus. Below it, a microcontroller called the spine owns the pins, the
actuators and the watchdog. This page covers what goes where, the CAN contract, and
what's left to build.

## ros/ vs firmware/

| | `ros/` | `firmware/` |
|---|---|---|
| Runs on | Linux computers: the Orin, the Pis, laptops | microcontrollers: ESP32, Pico 2 |
| ROS? | yes | no |
| Hardware code | only for devices that plug into Linux by USB or serial: the IMU, GPS, radar | pins, I2C actuator drivers, the RC receiver, the e-stop, timing loops |
| Reaches ROS | directly | through a bridge node: CAN to `spine_bridge` (`aav_spine_bridge`), or today UART to `aav_legacy` |

ROS never sees a pin or an I2C address, only device names like `can0` and `/dev/aav/imu`.

## How a chip joins ROS 2

The chips never run ROS. A small bridge node on a Linux computer turns ROS topics into
plain wire messages, and back.

![How the chips join ROS 2: a bridge node on Linux, a plain wire, and firmware with no ROS](img/chips-and-ros.svg)

| Way to join ROS 2 | On the chip | Our pick |
|---|---|---|
| A bridge node on Linux | plain firmware that speaks CAN, UART or UDP | ✅ today's ESP32 and Pico 2, and the spine next |
| micro-ROS | a ROS library in C, usually on a small operating system like FreeRTOS, plus an "agent" program on a Linux computer | not for now (Ben, 10 Oct): it's a bridge either way, and it isn't Rust. We think again once we see the hardware (Ben, 11 Oct) |
| rclrs (ROS in Rust) on the chip | not possible: rclrs needs a full operating system, like Linux | rclrs fits the Linux side, if a bridge is ever written in Rust |

## One interface, many drivers

In ROS the interface is the message type, the topic and the frame. Any driver that
publishes the same thing drops in, and nothing downstream changes.

| Interface | Implementations |
|---|---|
| IMU: `sensor_msgs/Imu` on `/sensors/imu/data_raw`, frame `imu_link` | `aav_imu_yahboom`, `sim_imu` |
| GPS: `sensor_msgs/NavSatFix` on `/sensors/gps/fix`, frame `gps_link` | `ublox_gps`, `nmea_navsat_driver`, `ublox_dgnss`, `sim_gps` |
| Backend: `/control/<target>/actuator_cmd` in (ego's, through the output mux), `/vehicle/odom` out | `spine_bridge`, `legacy_uart_bridge`, `turtlesim_adapter`, `sim_adapter`, `carla_adapter` |

The firmware applies the same idea one level down. `spine-core` never touches hardware:
the board code fills an `Inputs` struct, calls `Spine::step`, and writes out the
`Outputs`. The logic is tested on a laptop, and changing a motor driver only changes board
code.

## The spine's job

- Take drive commands over CAN, and refuse them unless armed.
- Run the speed loop, and steer to the target angle.
- Read the wheel encoder. Report speed, steering, mode and faults.
- Let the RC override switch and the hardware e-stop beat the computer.
- **The red button always stops the car, whatever our software does** (Ben). We don't
  rewire it. The spine only listens, through a spare contact on the button, and tells
  ROS and the HUD it's pressed. If the spine crashes or loses power, the button still
  stops the car:

```
e-stop button ─────────────────► stops the car on its own (wired as it is today)
      │
      └─ a spare contact ──► spine (only listens) ──► ROS and the HUD: "e-stop pressed"
```
- Stop by itself when commands stop: a 100 ms watchdog (DBC v0.2.0, from the KEFC plan).

Who wins, highest first:

| # | Source | When |
|---|---|---|
| 1 | Any e-stop: the hardware button, or the `ESTOP_CMD` frame | always: throttle off, full brake |
| 1 | A driver fault: steering, brake, throttle or encoder | the same |
| 2 | The RC transmitter | while its override switch is on |
| 3 | The computer | while armed and sending fresh `DRIVE_CMD` frames |
| 4 | Nobody | the actuators idle |

The computer must arm (`/spine/arm`) before it drives: at power-up, and after an e-stop,
a timeout or a driver fault. Arming sends `armed = 0`, then `armed = 1`. A glitch or a
reboot never restarts the car by itself. `spine_bridge` refuses to arm while the safety
gate is shut: the output mux not `OPEN` ([sources and targets](sources-and-targets.md)).

## The CAN contract

`firmware/can/aav.dbc` is the source of truth: classic CAN at 500 kbit/s, 11-bit IDs,
little-endian. Every frame ends with a CRC-8 and a 4-bit counter, and a bad one is
rejected.

| ID | Frame | Direction | Rate | Carries |
|---|---|---|---|---|
| 0x010 | `ESTOP_CMD` | Orin → spine | 10 Hz, and at once on change | the e-stop request |
| 0x100 | `DRIVE_CMD` | Orin → spine | 50 Hz | target speed, steering angle, limits, armed |
| 0x180 | `VEHICLE_FEEDBACK` | spine → Orin | 50 Hz | measured speed and steering, wheel ticks |
| 0x181 | `VEHICLE_STATUS` | spine → Orin | 10 Hz | mode, armed, e-stop, throttle %, brake %, faults |
| 0x200 | `WHEEL_SPEED_L` | left wheel node → Orin | 100 Hz | left front wheel speed (m/s), pulse count |
| 0x201 | `WHEEL_SPEED_R` | right wheel node → Orin | 100 Hz | right front wheel speed (m/s), pulse count |

DBC v0.2.0 (15 Oct, [The KEFC CAN plan](can-plan.md)) added the wheel frames and cut the
timeout to 100 ms. The wheel frames' speed is signed, 1 mm/s a step (±32 m/s). One Hall
sensor can't tell direction, so `direction_known` = 0 and `spine_bridge` takes the sign
from the drive direction.

Both sides test against the same generated frames. To change a frame:
1. Edit the DBC.
2. Edit both codecs: `can_codec.py` and `protocol.rs`.
3. Run `python3 firmware/can/gen_test_vectors.py`, then `tools/test.sh`.

## firmware/spine: the Rust code

`crates/spine-core` is plain `no_std` Rust: no allocator, no dependencies, no hardware.
`cargo test` runs its 143 tests on any laptop.

| Module | Does |
|---|---|
| `spine.rs` | the priorities above: `Spine::step(&Inputs) -> Outputs`, about 100 times a second. Its `SpineConfig` holds the limits: 1.0 m/s (`MAX_SPEED`, since 6 Nov, T3.6), ±0.5 rad, 0.6 rad/s; 1.5 m/s² up, 2.5 m/s² down |
| `protocol.rs` | encodes and decodes the six frames, with the CRC and counter checks |
| `fsm.rs` | the five state regions the status lines report |
| `watchdog.rs` | the 100 ms command watchdog |
| `speed_loop.rs` | PI speed control into throttle and brake, never both at once. A stop brakes until the car is still, then holds |
| `limits.rs` | the helpers that apply them: `slew()` and `abs()` |
| `encoder.rs` | quadrature decoding, and speed from the time between counts |

Clean inputs for every board (debounced switches, an e-stop that trips at once, counting
with a lockout) are in a second crate, `aav-inputs`. The drivers for the spine's devices,
and the glue between them and `spine-core`, are written too (10 October):

| Crate | Drives | Tests |
|---|---|---|
| `jrk-g2` | the two Pololu Jrk G2s (steering, brake) over I2C | 15 |
| `ad5311` | the throttle DAC over I2C | 10 |
| `pca9555` | the I/O expander: the direction output | 6 |
| `ibus` | the FlySky RC receiver | 30 |
| `mcp2515` | the MCP2515 CAN controller over SPI: a Pico 2 has no CAN of its own | 34 |
| `legacy-link` | the Motor-Pi's 13-byte UART frames, and the targets the C++ set for them (15 Oct); version 2 with the Orin's heartbeat (24 Oct, TODO 50); versions 3 and 4 since | 121 |
| `spine-board` | all of them, from `spine-core`'s outputs, in a safe order; their errors become fault bits. Commands through CAN or the UART (`CommandLink`) | 119 |

On a laptop, [Bench mode](bench.md) runs `spine-core` on the CAN frames from `spine_bridge`,
driving a model car. The plan for all the firmware in Rust is
[Rust on the hardware](rust-firmware.md).

The board program for a Pico 2 is `boards/pico2` (11 October): Embassy, CAN through an
MCP2515, a 100 ms hardware watchdog, and the e-stop's spare contact on GP15. It builds
(60 KB of flash, 2.8 KB of RAM) and its UF2 file checks out, but it has never run on a
board: [[untested]]. The car's chip is still Q10-Q13. Flashing it:
[Rust on the hardware](rust-firmware.md).

!!! warning "Buy a 3.3 V CAN module"
  The common MCP2515 module, with a TJA1050, runs at 5 V and doesn't suit a Pico 2's
  3.3 V pins. Buy a 3.3 V one, for example an MCP2515 with an SN65HVD230.

Hardware facts from today's car:

| Part | Interface | Today's setting |
|---|---|---|
| Steering: Pololu Jrk G2 | I2C 0x05 | targets about 0–3500; the calibrations disagree (Q2) |
| Brake: Pololu Jrk G2 | I2C 0x04 | targets 400–3500 |
| Throttle: 10-bit DAC | I2C 0x0C | capped at 700 of 1023 |
| Direction | PCA9555 expander, I2C 0x20 | |
| I2C bus | SDA GPIO 21, SCL GPIO 22 | |
| RC receiver: FlySky FS-iA6B | iBUS on GPIO 32, 115200 baud | channel 5 override, 2 throttle, 3 steering, 4 gear (forward, brake, reverse). Counted from 0: on the transmitter they're CH6, CH3, CH4, CH5 (Q7) |

## The car's two other Pico 2s, in Rust (15 October)

The 2025-26 wheel encoder and status display were C++. Their Rust versions sit next to the
spine, in `firmware/spine/`. Both build and pass their tests; neither has been flashed:
[[untested]].

!!! note "The UDP wheel encoder is off (23 Oct)"
    Today's wheel speed comes from the hardware lead's Arduino on the Orin's CAN bus
    (0x200, 0x201), read by `spine_bridge role:=sensors`. The 2025-26 UDP encoder is off in
    `motor_pi.yaml` (`legacy_encoder: false`), so its Rust version waits too.

| Board | Does | Logic (tested on a laptop) | Build |
|---|---|---|---|
| `boards/wheel-encoder-pico2` | counts wheel spokes; every 100 ms sends the count to the Motor Pi, UDP port 10111, through a W5500 | `wheel-encoder` (12 tests) | `pixi run pico wheel build` |
| `boards/sentinel-pico2` | a status display: an I2C device at 0x42, a 16x2 LCD, an 8x8 matrix, 10 LEDs, a knob, a buzzer | `sentinel` (40), `hd44780` (5) | `pixi run pico sentinel build` |

The wheel encoder's packet is byte for byte the 2025-26 one: the Rust crate and
`aav_legacy` check the same examples (`firmware/udp/wheel_encoder_vectors.txt`). Pins,
settings and flashing: [Rust on the hardware](rust-firmware.md).

## Today's firmware: firmware/legacy/

Copied unchanged from the 2025-26 team, as the fallback until the spine exists.

!!! note "The ESP32's Rust replacement (15 Oct)"
    `firmware/spine/boards/esp32-umod` is `aav_umod_esp32` in Rust: the same board, pins
    and devices, and byte for byte the same UART frames (checked against the C++ itself).
    It builds, and runs in QEMU. The repo has no record of it flashed on the car: the
    last check of the board (6 Nov, [the audit](history/audit-2026-11-06-car.md)) found
    the C++. The C++ stays here, and on the car, until the Rust one is proven on the car: [Rust firmware](rust-firmware.md),
    [First flash on the bench](esp32-bench.md).

| Folder | What | Talks to ROS through |
|---|---|---|
| `aav_umod_esp32/` | the ESP32 actuator firmware | UART from the Motor Pi, 13-byte frames: `legacy_uart_bridge` |

The two Pico 2 programs that were here (the wheel encoder and the status display) were
replaced by their Rust versions on 15 October, above. Their C++ stays on GitHub:
`AAV-2025-2026/Wheel_Encoders` @ f824e9d805, and `AAV-2025-2026/AAV_cortex` @ 90de619e52
(`AAV_sentinel/`).

Build today's ESP32 firmware with `pixi run -e firmware legacy-esp32` (PlatformIO), and
add `-t upload` to flash it. PlatformIO's downloads, about 1.5 GB, go into
`vendor/platformio/`. Without `src/secrets.h`, the build uses the example's placeholder
WiFi details: fine for a test build, but that board can't join the car's WiFi.

!!! warning "Before driving on the legacy path"
  1. **No command timeout.** If frames stop, the last throttle stays on. So the bridge
     locks every program out (twist_mux's no-timeout lock, priority 60): only people and
     the actuator tests drive. `allow_programs_without_timeout: true` lifts it, on stands only.
  2. **The OTA update page has no password.** Anyone on the WiFi can flash the car
     (seen again on 6 Nov).
  3. **No feedback.** No speed, steering or faults come back.
  4. **The steering sign is unconfirmed.** Check it with the wheels off the ground.
  5. **The WiFi password was public** in the 2025-26 repositories. Change it.

!!! warning "Found in the 2025-26 ESP32 firmware, while porting its drivers"
  1. **Steering past full right turns full left.** Stick values from 2001 to 2199 pass its
     check, then wrap round to full lock the other way (`rc_handler.cpp`).
  2. **No RC timeout.** If the receiver goes quiet, the last throttle stays on.
  3. **A Jrk G2 missing at boot is skipped for good**, but the car still drives: throttle,
     with no brake.
  4. **Throttle and brake at the same time**, in the RC brake position.
  5. **The direction can flip with the throttle on.**
  6. **Stale commands.** Frames from ROS pile up during RC mode, and run when it ends.
  7. **Latched Jrk errors never stay latched**: reading them after every target clears them.
  8. **Weak iBUS sync**: frames get lost or misaligned.
  9. **All 16 expander pins are driven high**, whatever is wired to them.
  10. **Two throttle scales**: the top third of the RC stick does nothing.

  The Rust drivers fix each of these. Until the spine replaces this firmware, drive today's
  car with the RC transmitter's switch in reach.

## T3.6: the stops and its done test

Decided by Arsh after talking with the hardware lead, 6 Nov. **Ben and the team haven't
reviewed these yet.**

| What happens | The spine |
|---|---|
| E-stop, driver fault, the RC's right stick pulled down | hard stop |
| Lost input: commands, RC (in normal mode too), wheel speed | hard stop, latched until re-armed |
| Past 1.25 m/s (the limit is 1.0) | throttle off, brake 0.5, until under the limit |
| Still over the limit 2 s later, speeding up or steady | hard stop, latched |

The table in full: [`spine-core`](../firmware/spine/crates/spine-core/README.md).

### The RC transmitter (Arsh, 21 Nov)

The FS-i6X's left stick doesn't spring back to the middle, and its SwC switch (CH5) has
loose wiring. The ESP32 decides between the RC and the computer itself, so it still works
with the Motor Pi or the Orin dead. Since 22 Nov (Arsh and Ben,
[the design](designs/2026-11-22_rc-right-stick.md)) the right stick's up and down (CH2,
which springs back) hands the car over, and the left stick is one pedal:

```text
right stick (CH2):  up past 88 % ── give to the computer (steering centred, 0.5 s)
                    down past 88 % ── hard brake while held; the RC takes the car
left stick (CH3):   1000 ◄── forward ──┤ middle ±20 %: throttle off ├── reverse ──► 2000
                    past 88 % the other way, the car still moving: the hard brake too
```

| Mode switch (CH6) | Who drives |
|---|---|
| Low: **normal** | the computer once the right stick has given it the car; the RC takes it back with the right stick down. The RC must stay: losing it is a hard stop |
| Middle: RC via ROS | the computer, from the sticks sent up to ROS. The right stick down still brakes. The team's FS-i6X has a two-position switch, so it can't pick this |
| High: **manual** (RC direct) | the RC only; the computer's commands count for nothing. The right stick up does nothing |
| Under 1050: the transmitter's failsafe (1000) | RC lost: a hard stop, latched. The FS-i6X's failsafe can't go below -100% (1000), so the switch's low is moved to 1100 (End points CH6, low side 80%) |

| Rule | The spine |
|---|---|
| The left stick up or back | drive that way at a fixed speed (1.0 m/s, 0.5 back), ramped at 0.5 m/s² (`RC_ACCEL`). With no wheel speed (today's ESP32), the throttle starts where the wheels start to turn, 475 of 1023 forward and 525 back, and rises to 550 (554 back); 700 is the cap (Arsh, 27 Nov) |
| The left stick in its middle | throttle off, no brake: the car slows as the motor controller decides ([braking with the motor](#braking-with-the-motor-regen-not-used)) |
| The left stick the other way, the car still moving | throttle off; past 88 % the hard brake too. The new way once stopped: under 0.05 m/s for 0.5 s, or with no speed (today's ESP32), the hard brake held 1 s |
| The right stick down | the hard brake while held, the steering stick still steering. In normal mode the RC takes the car |
| Let go of it; after any other stop, at power-up, after the mode switch moves | braked at least 1 s (`RC_BRAKE_MS`), until the left stick leaves the zone it was in (`RcHold`) |
| Normal: the right stick up, steering within ±15 %, held 0.5 s (`RC_GIVE_MS`) | the computer has the car (it still needs arming) |
| Normal: the left stick alone | never takes the car from the computer |
| A stick out of range (CH2, CH3 or CH4) | the right stick's stop |
| The left stick sideways, SwC (CH5) | nothing |

Not known yet: whether the left stick's low end really is forward (`throttle_reversed` in
`ibus::RcMap`, Q7). The transmitter check: [Car day, the RC transmitter](car-day.md#the-rc-transmitter).

A person stopping any other way is ego's job, and an obstacle stop is navigation's (the stop zone) and ego's: not
built yet. The wheel speed reaches the firmware only once the Motor Pi sends frame
version 4: until then the two speed rules stay off.

**The proposed done test.** The proposal's needs the bench rig and the CAN link. The rig
(T3.7) isn't being built, and the ESP32 stays on the Motor Pi's UART:

> On the car, on stands: unplugging the UART, killing the Motor Pi and killing the Orin
> each cut the throttle and put the brake full on within **130 ms**, in 10 of 10 trials
> each. Only a deliberate re-arm releases it. Corrupted frames are tested in QEMU.

The 130 ms is the 100 ms timeout, up to two 10 ms ticks, and a margin over QEMU's measured
111-120 ms. It times the command, not the actuator's travel.

## Braking with the motor (regen): not used

Regenerative braking ("regen") slows a car with its motor, which then works as a generator
and charges the battery. **The firmware does none of it.** When the throttle is off (the
RC's middle zone, a stop), it sends the DAC 0, and whatever the motor does then is the motor
controller's own setting (the silver finned box), not our code.

**Why not, on this car** (Will, the electrical lead, 27 Nov): there is too little energy to
recover at the speeds it can reach, and its motor doesn't reach the speeds where it would
pay. The energy in the moving car is ½ × mass × speed²:

| Speed | Energy, empty car (181 kg, `vehicle.yaml`, a guess) | About as much as |
|---|---|---|
| 1 m/s (our limit) | 90 J | lifting a 9 kg bag one metre |
| 10 m/s (36 km/h, where it would start to pay, Will's estimate) | 9,000 J | 100 times more; out of this motor's reach |

**What a later team would need to try it:**

| Need | Why | Where to start |
|---|---|---|
| A motor controller that does regen, and its settings | it decides whether braking charges the battery | the controller's model and manual (not yet written down) |
| A battery and charge path that accept the current | charging too fast harms the cells | the battery's datasheet; the Victron monitor (`aav_battery_victron`) shows the current |
| A wheel speed on the board | regen fades near a stop, so the spine must know the speed to blend it with the mechanical brake | today's ESP32 has none; the wheel encoder reaches the Motor Pi only |
| A way to ask for it | the firmware has no regen command | in `spine-core`, the RC's middle zone and a stop send throttle 0 (`Spine::rc`, the open loop): a "regen" request would go there, and the board would send it to the controller |
| A car that goes faster | below about 10 m/s it recovers almost nothing | the motor and the gearing |

**What today's controller does at throttle 0** isn't measured. On stands (27 Nov), a rear
wheel spinning at the RC's speed stopped in about 0.5 s with the throttle off and **no**
brake, as fast as with the full brake: something slows it (the controller, or the drive
train's friction). On the ground, under the car's weight, the two will differ. Test there,
slowly, before changing the brake.

## Still to decide

For the team, from the drivers' port. Each is one constant at the top of `spine-core`'s
`spine.rs`, so the board program and the bench both follow a change:

| Constant | In spine-core | The 2025-26 firmware |
|---|---|---|
| `ESTOP_STEERING` | `Hold`: the steering stays where it is on any stop | centred it |
| `IDLE_BRAKE` | `0.0`: the brake lets go once the e-stop clears | kept it full on (1.0) |

⚠️ `IDLE_BRAKE` matters most: on a slope, a car with its brake off can roll. It is a team
choice:

| Board program | `IDLE_BRAKE` | Why |
|---|---|---|
| `spine-core`'s default, `boards/pico2`, the bench | `0.0` | the proposed setting |
| `boards/esp32-umod` (the car's board) | `1.0` (its own constant, `src/main.rs`) | keeps the C++'s full brake until the team decides: it has no e-stop input yet |

And the design doc's questions Q10–Q17:
- who owns the spine, and which microcontroller;
- the CAN transceiver and wiring;
- where the encoder and the e-stop connect;
- steering feedback;
- what the actuators do when nobody is driving.
