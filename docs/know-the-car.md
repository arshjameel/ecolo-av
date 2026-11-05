# Know the car: the order, and when to stop

Written 1 November 2026. Ben, 1 Nov: "If we do enough tests we should be able to figure
out the driving characteristics of the car and get more accurate with every pass ... We
can map the area while there."

> **The short version**
> - One page: **the order**, and **when to stop**. How to run each test is on its own page (the links).
> - Car day's step A11: only after [A10](car-day.md#a10-the-ground-at-walking-pace) is signed. Test 1 is on stands, earlier (A6, session 9).
> - Each test fits one number. Run it again until **two runs in a row** are "good enough".
> - **Record with the lidar the whole time**: the same drives make the lot's map.

## The loop

```text
  run the test ──► measure ──► fit one number ──► vehicle.yaml ──► run again
                                                                      │
                          two runs in a row good enough? ── yes ──► next test
```

## The order

| # | Test | How to run it | Fits | Stop when (good enough) |
|---|---|---|---|---|
| 1 | Lock to lock, on stands | [3.0](tests.md#3b-steering-on-stands) by [Way A](car-day.md#a6-the-steering-actuators-ends-way-a), then 3.10 | `steering_limits`, `max_steering_angle` | equal limits each side; the hardware lead agrees |
| 2 | Fixed-steering circle, both ways | [4.5](tests.md#4b-steering-on-the-ground) | `max_steering_angle`, `steer_offset` | the radius within 5 %; left and right within 5 % |
| 3 | 10 m straight, by tape | [4.7, 4.20](tests.md#4e-record-the-lot-the-maps-and-the-scores) | `wheel_diameter` | within 1 % (10 cm in 10 m) |
| 4 | Speed step, 0 to 1 m/s | [4.1](tests.md#4a-speed-up-coast-brake) | `throttle_lag`, `max_accel`, `full_throttle_speed` | settles within 10 % of the speed asked |
| 5 | Brake from 1 m/s | [4.3](tests.md#4a-speed-up-coast-brake) | `max_brake`, `brake_lag` | the distance within 10 % of v² ÷ (2 × `max_brake`) |
| 6 | Coast-down from 2 m/s | [4.2](tests.md#4a-speed-up-coast-brake) | `drag` | two runs within 10 %. ❌ not on today's ESP32 ([why](actuator-tests.md#what-the-car-can-measure-today)) |
| 7 | Test 3 again | as 3 | `wheel_diameter` | a new `full_throttle_speed` can change it |

Why this order: steering first (every other test drives straight with it), then the
wheels (every speed reads them), then the speed, the brake and the drag.

How each fit works, and when it is refused: [Trusting the fits](actuator-tests.md#trusting-the-fits).
What a change does to the car's model: `pixi run python3 tools/vehicle_model.py --set <key>=<value>`.

## Stop the session when

| Stop | Then |
|---|---|
| Anything turns the wrong way, or faster than asked | the Kar-Tech stop; write it down; no more ground today |
| A fit is refused twice | write down why; go to the next test |
| The disk on the Orin has under 5 GB | copy the bags off (session 10) |
| A person says stop | stop. Ask why after |

## Map the lot at the same time

| ☐ | Do |
|---|---|
| ☐ | The lot's corner in `navsat.yaml`'s `datum`, the same every time ([4e](tests.md#4e-record-the-lot-the-maps-and-the-scores)) |
| ☐ | Before the first test: `~/aav-jazzy/aav.sh tools/record.sh --lidar learn` (5 GB per 10 min) |
| ☐ | Between tests: 2 laps and a figure 8 (4.19) |
| ☐ | Before leaving: the map (4.21). Blurred? Record again while the car is there |

The whole car's score each pass: [A to B](a-to-b.md), tests 5.1 and 5.2 in [Every test](tests.md#stage-5-the-whole-drive).

## The record of each pass

Print it. One line a pass.

| Pass | Test | Number | Before | Measured | After | Good enough? |
|---|---|---|---|---|---|---|
| 1 | 3 | `wheel_diameter` | 0.36 | | | ☐ |
| | | | | | | ☐ |
| | | | | | | ☐ |
| | | | | | | ☐ |
