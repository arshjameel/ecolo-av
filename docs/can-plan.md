# The CAN bus: today's wiring, and the KEFC plan

Ben's KEFC request (October 2026, tab "Bens Mods") puts steering, braking and throttle on
a CAN bus. Our code had a CAN contract, `firmware/can/aav.dbc` (v0.1.0). This page
compares the two. Since 15 Oct, `aav.dbc` v0.2.0 follows the plan.

**Status: built in software, hardware not decided.** DBC v0.2.0, `spine_bridge` and the bench
use it (merged 15 Oct, so the software is ready). Whether the car gets CAN and the wheel
node boards is **the hardware lead's decision**, with the team (Ben, 15 Oct: "I am waiting
on the other people to make the final decisions. There is a hardware lead that isn't me").
No parts are ordered until then.

## Today's wiring (23 October)

**As Ben understands it; the hardware lead confirms.** Ben, 23 Oct: "the motor controllers
are on the esp32 with a pi as its network interface ... The can bus will be tied directory
to the orin not going thorugh the esp32. That will have the wheel encoders".

```text
                         network (ROS 2)
Orin ───────────────────────────────────────────── Motor Pi ── UART? ── ESP32
 │  ego, output mux                               legacy_uart_bridge     throttle, steering,
 │                                                                       brake (maybe the steering
 └─ CAN ──┬──────────────┐                                               angle, brake position)
          │              │
   wheel sensors     other sensors?
   (an Arduino:      (not known yet)
    0x200, 0x201)
```

Seen on the car, 6 Nov ([audit](history/audit-2026-11-06-car.md)): the Pi called CORTEX holds
the ESP32's UART (`/dev/ttyAMA0`), so it does the Motor Pi's job; the wire itself wasn't
checked. **The Orin's `can0` and `can1` were down**, so nothing read the wheel sensors
that evening. Whether `can0` should be up, and at what speed, is a question for the
hardware people.

| | Today's wiring | The KEFC plan (below) |
|---|---|---|
| Who drives the actuators | the ESP32, reached through the **Motor Pi** (probably serial) | the ESP32, on **CAN** |
| The car's backend | `legacy_uart_bridge` on the Motor Pi | `spine_bridge`, role spine, on the Orin |
| What the Orin's CAN bus carries | **sensors only**: the wheel frames, and a few others | drive commands, feedback, the wheel frames |
| The wheel sensors | an Arduino (the hardware lead's board, Arduino C++) | a Rust ESP32-CAN-X2 per wheel |
| The frames | the same: `aav.dbc` 0x200, 0x201 | the same |
| `car.launch.py` | `actuators:=uart` (the default) | `actuators:=can` |

### What runs where, today

| Machine | Runs | Topics |
|---|---|---|
| Orin | `socket_can_receiver` (no sender) and `spine_bridge role:=sensors` | `/can/rx` in; `/vehicle/odom`, `/vehicle/wheels` out |
| Motor Pi | `legacy_uart_bridge` | `/control/car/actuator_cmd` in (from the output mux); UART frames out |

`spine_bridge role:=sensors` **can't drive**: it has no `/can/tx` publisher, no
`/spine/arm` service and no `/safety/estop` subscription, and it ignores the spine's
frames (0x180, 0x181). On `/system/source` it names only `odometry` and `wheels`, never an
actuator. The output mux's guard test checks this, in Rust and in Python.

| `/vehicle/odom`, role sensors | How |
|---|---|
| Speed | the two front wheels' mean, while both are fresh. One stale: nothing (the sensor watchdog sees it stop) |
| Sign | the wheel's, if it measures it; else the last command's (one Hall sensor can't tell direction) |
| Yaw rate | not measured (covariance 1e3): the steering angle isn't on this bus. `ekf_local` takes it from the IMU |

!!! warning "Give every machine the same `actuators:=`"
    `actuators:=can` on the Orin and `uart` on the Motor Pi would start two backends. The
    wiring makes it moot (the ESP32 listens on one link), but don't mix them.

The 2025-26 UDP wheel encoder (`encoder_udp`, a Pico 2) is off in `motor_pi.yaml`: the
wheels are on CAN now, and two `/vehicle/odom` publishers would stop the output mux.

The Rust ESP32 firmware's 100 ms command timeout then sits on the UART link, not on CAN:
see [First flash on the bench](esp32-bench.md). Open questions for the hardware lead:
[Questions](audit-questions.md#8-the-cars-wiring-for-the-hardware-lead-23-oct).

## The KEFC plan in one picture

```text
Orin #1 ── CAN Pal (40-pin header) ──┬──────────────┬──────────────┐
  EKF, ego, spine_bridge             │              │              │
                              AAV control board  Wheel node L   Wheel node R
                              ESP32 MicroMod     ESP32-CAN-X2   ESP32-CAN-X2
                              2 Jrk G2, DAC      Hall sensor    Hall sensor
```

120 Ω at each end: on the CAN Pal, and on the last X2 board.

## What already matches

| Idea | KEFC PDF | Our DBC and code |
|---|---|---|
| ROS stops at the CAN bus | yes | yes: `spine_bridge` over `ros2_socketcan` |
| One command frame at 50 Hz | 0x100 | 0x100 `DRIVE_CMD`, 50 Hz |
| Status frame from the board | 0x180 | 0x180 `VEHICLE_FEEDBACK`, 0x181 `VEHICLE_STATUS` |
| A counter and a checksum, so a frozen Orin is caught | yes | yes: every frame, CRC-8 |
| Brake on silence | 100 ms | 100 ms (v0.2.0; it was 200 ms) |
| The speed loop on the board, not the Orin | yes | yes: `spine-core` |

## What is different

| # | Difference | KEFC PDF | Our DBC | Proposed |
|---|---|---|---|---|
| 1 | Which board is "the spine" | the existing ESP32 control board | a new Pico 2 | **the ESP32 control board.** It is already on the car, with its CAN plug |
| 2 | What 0x100 carries | steering, brake and throttle targets | target speed and steering angle | **ours.** The PDF's own loop table has the board running the speed loop, so it needs a speed, not a throttle |
| 3 | 0x010 | a heartbeat | the software e-stop | **ours.** Every frame's counter is already the heartbeat |
| 4 | Wheel speed | 0x200 left, 0x201 right, 100 Hz | one `wheel_ticks` in 0x180 | **add 0x200 and 0x201** from the PDF |
| 5 | Timeout | 100 ms | 200 ms | **100 ms**, from the PDF |
| 6 | Steering position | from the KTC-100 pot, via the Jrk | `measured_steering_angle` in 0x180 | the same: the board reads it from the Jrk |

## What changed in the code (15 Oct)

| Where | Change | Status |
|---|---|---|
| `aav.dbc` | v0.2.0: add `WHEEL_SPEED_L` 0x200 and `WHEEL_SPEED_R` 0x201, node `WHEEL_L`, `WHEEL_R`; the timeout to 100 ms; the spine node is the ESP32 board | done |
| `can_codec.py`, `protocol.rs`, test vectors | the two new frames, as `CLAUDE.md` says | done |
| `spine_bridge` | the two wheel speeds into `/vehicle/odom`, and `/vehicle/wheels` (the front two; the same topic Gazebo fills) | done |
| `spine-core` | stays: it has no chip in it. A new board crate for the ESP32 (`esp-hal`, Embassy, the TWAI CAN driver) beside the Pico 2 one | done: `boards/esp32-umod --features can`; its CAN pins are unknown |
| Wheel nodes | a small Rust program for the ESP32-CAN-X2 | not written: the hardware lead's Arduino sends the frames (23 Oct) |
| Gazebo, bench | the bench's spine simulator sends the two wheel frames too | done |

!!! note "Rust on an ESP32"
    The ESP32 and ESP32-S3 use Xtensa cores. Rust needs Espressif's toolchain
    (`espup`), not the stock one in Pixi. It is well supported: `esp-hal` has Embassy and a
    CAN (TWAI) driver.

## Still to confirm (from the PDF)

- The control board's CAN transceiver (U7) is fitted and works. The photos show the
  CANH/CANL plug ([The car in photos](car-photos.md)).
- The steering and brake actuators' wire counts: 5-6 wires means built-in feedback.
- The front brake rotors are steel, and have enough holes for the speed resolution.
- What the brake does when it loses power.
