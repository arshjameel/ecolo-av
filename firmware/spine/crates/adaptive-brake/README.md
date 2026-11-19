# adaptive-brake

## Overview

The adaptive brake (TODO 258): brake harder when the car slows less than the car model
says, **never above the brake's safe max**. The Rust port of `aav_description`'s
`adaptive_brake.py` (the sim's), for the car. How it works and why:
[How the car stops, section 4](../../../../docs/stopping.md#4-a-heavier-car-the-adaptive-brake).

```text
 ego's speed ─▶ speed loop ─▶ brake asked ─▶ AdaptiveBrake ─▶ brake sent ─▶ brake actuator
                    ▲                           ▲
              wheel speed ──────────────────────┘
 e-stop, stale command ─▶ full stop brake        (the locks: never through it)
```

## Where it runs, and why there

| Where | Why |
|---|---|
| **`legacy_uart_bridge`** (`aav_legacy_rs`, and the Python twin), after its `pi` speed loop: `adaptive_brake:=true` | It needs the speed loop's brake (0..1) and the wheel speed, and must sit below every lock. Today's car (`actuators:=uart`) runs its speed loop there, on the Motor Pi: the last place a brake level exists before the ESP32. Ego only gives a speed, so ego is too early; the ESP32's C++ is too late (not ours to change) |
| Not yet in `spine-core` (`actuators:=can`) | The CAN spine runs its speed loop in its firmware. This crate is `no_std` so the spine can take it later; `car.launch.py` refuses `adaptive_brake:=true` with `actuators:=can` until then |
| The sim (`sim_adapter`) | the Python copy; both check the same examples |

## Interface

| Item | Does |
|---|---|
| `AdaptiveBrake::new(CarModel, BrakeLimits, Tuning)` | a brake that has learnt nothing (effectiveness 1) |
| `update(throttle, brake, speed, dt) -> Result<BrakeOut, Error>` | one tick: the brake to send; `speed` `None` when no wheel speed is fresh |
| `reset()`, `reset_stop()` | forget everything (an e-stop); forget this stop, keep the effectiveness |
| `BrakeLimits::new(released, touch, full, safe_max, max_hold_time)` | `vehicle.yaml`'s `brake:`, checked |
| `CarModel`, `Tuning` | `vehicle.yaml`'s `actuators:` (the brake's part); the learning's numbers |
| `BrakeOut` | brake, asked, gain, effectiveness, capped, spent |

## Invariants

- Never weaker than asked; never above `safe_max` unless the speed loop itself asks for
  more (then exactly what it asked).
- The e-stop and the stale-command stops never pass through it.
- `no_std`, no allocator; `f64`, as the Python. The dead time's line holds 255 ticks.
- The Rust and the Python agree on `firmware/spine/adaptive_brake_vectors.txt`.

## Side Effects

None: numbers in, numbers out.

## Verification

```sh
cd firmware/spine && cargo test -p adaptive-brake
```

After changing either copy: `python3 firmware/spine/gen_adaptive_brake_vectors.py`, then
`tools/test.sh` (the Python's `test_adaptive_brake.py` checks the file is current).
