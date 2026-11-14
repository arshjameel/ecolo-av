# How a wanted deceleration reaches the brake

* **Author:** Ben
* **Date:** 2026-10-26
* **Status:** options only. The choice is still the team's and the hardware lead's. Checked 13 Nov: a closed loop in the bridge is written (`aav_legacy` `speed_control.py`, mode `pi`) but not selected, the car runs `open` (TODO 101); `aav.dbc`'s DRIVE_CMD still has no brake or signed-deceleration field (option 2 not taken).
* **Part of:** `concepts_brief.md`, decision 2 ("How a deceleration reaches the brake").

## In one line

Today **nothing** carries "slow down this hard" to the brake. The car coasts, and brakes
only when the speed asked for is almost zero. This note lays out four ways to fix that.

## 1. Where the wish gets lost today

```text
 aav_goto --speed ramp--> ego --AckermannDriveStamped--> output mux
                          speed: yes
                          acceleration: a rate cap, not a wish
                                 |
          +----------------------+------------------------+
          |                                               |
 TODAY'S CAR                                     THE RUST SPINE (not flashed)
 legacy_uart_bridge / to_legacy                  spine_bridge -> DRIVE_CMD (CAN)
   speed -> throttle, open loop                    speed -> target_speed
   brake only below 0.05 m/s, at 0.3               |acceleration| -> accel_limit
   acceleration: DROPPED                           spine-core speed loop:
          |                                          P brake past a 0.2 m/s band
 ESP32 (C++, or Rust legacy-uart)                    no wheel speed: no brake at all
   brake Jrk target = |channel| x 3500                while slowing
```

## 2. The facts, checked in the code

### Today's car: `legacy_uart_bridge`

| Fact | Where |
|---|---|
| The bridge reads only speed and steering. Ego's acceleration is never passed on | `legacy_uart_bridge.py:110-111` |
| Throttle is open loop: speed ÷ `full_throttle_speed` (3.0 m/s, a guess) | `legacy_protocol.py:74`, `:91` |
| Brake only when the speed asked for is below 0.05 m/s, at a fixed `hold_brake` 0.3 | `legacy_protocol.py:75-76`, `:89-90` |
| A stale command or an e-stop sends `stop_brake` 1.0, no throttle | `legacy_protocol.py:97-99`, `uart_relay.py:62-70` |
| Frame v1: 13 bytes (three floats, a sum checksum), no timeout | `legacy_protocol.py:8-20`, `:59` |
| Frame v2: 17 bytes, with the Orin's heartbeat and a CRC-8. The Rust ESP32 brakes 100 ms after the counter stops | `legacy_protocol.py:22-30`, `:60` |

### What the C++ ESP32 does with the third channel ("accel")

`firmware/legacy/aav_umod_esp32/src/modules/ros2_handler/ros2_handler.cpp`:

| Channel value | What happens | Line |
|---|---|---|
| any value outside −1..1 | frame rejected, throttle ramps to 0 | `:28`, `:72-75` |
| below −0.05 | brake Jrk target = clamp(\|a\| × 3500, 400, 3500); throttle cut **at once** | `:41-49` |
| −0.05 or above | brake released (400); throttle from the speed channel | `:50-52`, `:57-65` |

Three things follow from that:

- **The brake is a position, not a force.** 400 is released, 3500 is full
  (`hardware_configs.h:29-31`). The travel is unconfirmed (Q3, `calibration.rs:116-117`).
- **Channel −0.05 to −0.114 cuts the throttle but brakes nothing**: 0.114 × 3500 = 400.
  So today's `hold_brake` 0.3 is target 1050: about **21 % of the travel**.
- **After a brake frame the throttle starts again from zero**, rising 6 DAC codes per
  frame (`hardware_configs.h:47`, `dac.cpp:31-42`). Flicking between brake and throttle
  will lurch.

The Rust ESP32 board (`boards/esp32-umod`, `legacy-uart` build) copies this mapping
exactly (`legacy-link/src/targets.rs:62-68`, `spine-board/src/link.rs:149-158`). It runs
the channel as a `DirectCmd`: no speed loop (`spine.rs:532-567`). Nobody driving, it holds
the brake full on: `IDLE_BRAKE` 1.0 (`esp32-umod/src/main.rs:114-117`).

### The Rust spine on CAN (not flashed yet)

| Fact | Where |
|---|---|
| A stop (target ≤ 0) brakes max(0.3, 0.5 × (speed − 0.2)): **0.4 at 1 m/s** | `speed_loop.rs:202-214` |
| Slowing but not stopping: no brake until 0.2 m/s too fast, then 0.5 per m/s | `speed_loop.rs:232-240` |
| P only: no integral, no feed-forward on the brake | `speed_loop.rs:44-51`, `:62-65` |
| The `smooth` profile keeps the same brake law | `speed_loop.rs:78-87` (`..default()`) |
| No wheel speed (today's ESP32 board): open loop, **no brake while slowing**, the hold brake only below 0.05 m/s | `speed_loop.rs:255-264`, `spine.rs:105-108`, `esp32-umod/src/main.rs:396-402` |
| The setpoint slews 1.5 m/s² up, 2.5 m/s² down | `spine.rs:150-151`, `:501-505` |
| `accel_limit` only limits speeding up; slowing always uses 2.5 | `spine.rs:480-484`, `:504` |
| …though `aav.dbc` says it limits "acceleration or deceleration" | `aav.dbc:99` |
| `accel_limit` is unsigned, 8 bits; DRIVE_CMD has only bits 49-51 free | `aav.dbc:44-51` |
| Both bridges send \|acceleration\| as `accel_limit` | `spine_bridge.py:228`, `aav_spine_bridge/.../bridge.rs:353` |
| E-stop, timeout and driver faults return `stop_brake` 1.0 before any speed loop | `spine.rs:428-435` |
| Throttle and brake are never both on | `speed_loop.rs:142`, `spine-board/src/actuators.rs:161-165` |

### Ego

| Fact | Where |
|---|---|
| `max_acceleration` 1.0, `max_deceleration` 2.0, `max_jerk` 2.0 (speeding up only) | `ego.yaml:10-12`, `ackermann.py:40-44` |
| The published `acceleration` is the **rate cap used** (always ≥ 0; 2.0 while cruising or slowing) | `ackermann.py:95-101`, `:144`, `:173`; `ego_node.py:128` |
| Ego does keep the signed change of speed, but doesn't publish it | `ackermann.py:137`, `:170` |

### `aav_goto` and an A-to-B stop today

| Fact | Where |
|---|---|
| Slowing follows v = √(2 a (d − lead × v)); a = `max_brake` × 0.5 = 1.5 m/s² | `approach.py:74-77`, `goto.yaml:10` |
| `brake_lead` 0.8 s was tuned in Gazebo, whose model copies the **spine's** brake law, not today's car | `goto.yaml:12-16`, `aav_gazebo/actuator_model.py:145-165` |
| It asks for at least 0.1 m/s until 5 cm from B, then zero | `approach.py:87-88`, `:97`; `goto.yaml:17-18` |
| `max_brake` 3.0, `brake_lag` 0.2, `drag` 0.1 are all GUESS | `vehicle.yaml:55-57` |

So on today's car, an A-to-B stop:

1. goto ramps the speed down at about 1.5 m/s²;
2. the throttle eases off, but **no brake** comes on;
3. the car coasts. With the guessed drag (0.1 /s), coasting from 1 m/s loses only
   about 0.1 m/s²;
4. 5 cm from B (or past it) the command hits zero, and the brake goes to 21 % travel.

The real coasting is **untested**: the `coast` test (4.2) will tell.

## 3. The options

### Option 1: legacy, no firmware change

**What:** `to_legacy` turns a wanted deceleration into the brake channel, behind a
parameter whose default is today's behaviour.

| | |
|---|---|
| Where the wish comes from | **1a:** the drop in commanded speed, in `Relay` (low-pass over ~0.1 s). No contract change. **1b:** ego publishes its signed acceleration. That changes the field's meaning: `docs/interfaces.yaml` first, and both spine bridges' `abs()` |
| The law | brake fraction f = (a_wanted − a_coast) / `max_brake`; channel = −(0.114 + 0.886 f). Below a small threshold: the "coast" code (throttle cut, no brake) |
| Files | `legacy_protocol.py` (`LegacyMapping`, `to_legacy`), `uart_relay.py`, `legacy_uart_bridge.py` (pass `max_brake` from `vehicle.yaml`, as goto does), their tests |
| Contracts | none. The frame stays the 2025-26 firmware's, v1 and v2 |
| Gives the controller | a brake that follows goto's ramp, open loop, on today's car |

**Risks**

- Open loop on `max_brake`, a guess. goto's distance law corrects some of the error
  (`approach.py:17-21`), but only through the throttle.
- Brake ↔ throttle flicker (the throttle restarts from zero). Needs hysteresis: no
  throttle again in the same slowing phase.
- The brake's force against its position is unknown, and maybe not straight.

**Safety rules it keeps**

- Below 0.05 m/s, still `-hold_brake`, never less.
- `stop_command` (e-stop, stale) unchanged at 1.0; the v2 heartbeat unchanged.
- The red button stays hardware, outside all of this (`esp32-umod/src/main.rs:50-52`).

**Tests:** default mapping bit-identical to today's on a sweep; f ≥ hold at a stop; never
brake and throttle in one frame; e-stop wins at any wanted deceleration; the
hysteresis; `legacy-link/vectors.json` still passes (no frame change).

**Who decides:** the team (software). The hardware lead: whether the brake may move
while driving, the top level allowed, and Q3 (the travel).

### Option 2: spine, a signed target acceleration on CAN

**What:** DRIVE_CMD carries a signed target acceleration. The spine brakes by
feed-forward, brake = −a / `max_brake`, plus its P correction.

| | |
|---|---|
| Contract, sub-choice | **2a:** make `accel_limit` signed (8 bits × 0.05: −6.4..6.35 m/s²). Changes its meaning. **2b:** a new frame (0x101 is free) with a 16-bit signed target, its own counter and checksum |
| Files | `aav.dbc`, then CLAUDE.md's steps: `can_codec.py`, `protocol.rs`, `gen_test_vectors.py`, both test suites. Both bridges, `speed_loop.rs`, `spine.rs`, the Python copy in `aav_gazebo/actuator_model.py` and `gen_speed_loop_vectors.py`. A `MAX_BRAKE` copy in `spine.rs`, checked against `vehicle.yaml` like the others (`spine.rs:71-72`). Ego publishes the signed value (see 1b) |
| Gives the controller | an explicit brake request, which works **even with no wheel speed** (feed-forward needs none) |

**Risks:** the biggest change, on a board that isn't flashed. Fixes `accel_limit`'s
comment-vs-code mismatch, or makes it worse: pick one meaning. The stop branch must keep
max(hold, …).

**Tests:** codec vectors; spine-core: feed-forward + P never below today's stop brake,
e-stop and timeout still 1.0, never both pedals; Gazebo A-to-B scorer against today.

**Who decides:** the team (the CAN contract), the hardware lead (the flash, Gate 2,
brake levels).

### Option 3: spine, a better brake law, no contract change

**What:** change only `speed_loop.rs`. Three ways, which can combine:

| Way | Gives | Catch |
|---|---|---|
| 3a: an integral on the brake | removes the steady lag behind a ramp | needs wheel speed; windup |
| 3b: feed-forward from the **setpoint's own slope** (it already follows ego's ramp) | most of option 2, with no new field | needs a low-pass: ego sends at 50 Hz, the spine ticks at 100 Hz |
| 3c: no coast band while slowing | brakes sooner | more brake on small errors |

Note the `smooth` profile alone changes **nothing** about braking.

**Files:** `speed_loop.rs`, `actuator_model.py` and the speed-loop vectors (they must
match), tests. **Risks:** the same safety rules as option 2. **Who decides:** the team;
the hardware lead for brake levels.

### Option 4: leave it

**What:** tune goto's `brake_lead` and `brake_fraction` (or `deceleration`) to the
coasting car.

| Gives | Costs |
|---|---|
| no code; a baseline to score the others against | coasting is weak and changes with slope, load and wind; the only brake is still the last 5 cm; "never past B" is unlikely on a slope |

**Who decides:** the team.

## 4. Side by side

| | 1 legacy | 2 signed field | 3 better law | 4 leave it |
|---|---|---|---|---|
| Works on today's car | **yes** | no (needs the flash + CAN) | no (needs the flash) | yes |
| Contract change | none (1a) | `aav.dbc` | none | none |
| Needs wheel speed | no | no | 3a yes, 3b no | no |
| Brake follows the wish | open loop | feed-forward + P | 3b like 2 | no |
| Size | small | large | medium | none |

## 5. A possible order (for the team and the hardware lead to decide)

1. **First, no code:** the hardware lead confirms the brake travel (Q3) and the highest
   brake allowed while moving.
2. **Option 4 as the baseline:** run `coast` and the stop as they are today. Tag the runs
   "legacy, hold brake only" in `meta.yaml`.
3. **Option 1a,** off by default, tested offline. Switch it on only after the brake-level
   sweep below.
4. **On the spine, after Gate 2:** option 3b first (no contract change). Option 2 only if
   the logs show the slope isn't enough, or a planner must brake without changing speed.

## 6. What the actuator tests need to measure

Today's `brake` test (4.3) commands speed 0 (`procedures.py:230-236`). On the legacy
car that is ego's ramp, coasting, then **21 % travel**. It measures the hold brake, not
`max_brake`. Label it so.

| Test | Measures | For |
|---|---|---|
| `coast` (4.2) at 0.5 and 1.0 m/s | drag, a_coast | 1, 4 |
| **new:** brake-level sweep, fixed levels (the hardware lead picks the top) from 1 m/s | deceleration per level; the lowest level that bites; `brake_lag`; `max_brake` | 1, 2, 3 |
| **new:** ramp tracking at 0.5, 1.0, 1.5 m/s² | the logged brake level against the wish; the deceleration reached | 1, 2, 3 (the brief's "a logged brake level that follows it") |
| taped stop, A to B, 5 runs | distance from the mark; never past B | all |
| on stands: e-stop and a stale command during a slowing brake | 1.0 brake at once, every time | 1, 2, 3 |

Two gaps block these tests today:

- The tests can't ask for a fixed brake level: `cmd_vel` carries only speed. It needs a
  test-only way (to design).
- The bridges publish the brake **command** as NaN (`concepts_brief.md`, S1: "log the
  commands"). Without it, "follows the wish" can't be scored.
