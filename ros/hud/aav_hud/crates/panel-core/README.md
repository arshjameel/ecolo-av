# panel-core

## Overview

The hardware panel's logic, with no ROS and no raylib, so it is tested on any laptop.
The `hw_panel` program (`panel-app`) feeds it every sensor's topic, `/system/source`,
the e-stop and twist_mux's output, and the person's presses; it says what to draw and
what to send. Design: `docs/designs/2026-10-29_two-screens.md`.

```
presses (Speed +/-, arrows, CENTRE, STOP), GO held ──► buttons ──► a command, or nothing
/system/source, --stands, vehicle.yaml, the locks ──► gate (on: cap / off: why)
/safety/estop, /control/cmd_vel (a takeover), a source change ──► zero, GO pressed again
/vehicle/steering_actuator/raw, LIMP / HOLD / STEP / MARK / CHECK ──► calibration ──► limp or jog, 10 Hz
the three marks ──► steering_limits ──► the lines to copy;  CHECK ──► sweep (stops on resistance)
```

## Interface

| Module | Holds |
|---|---|
| `config` | `PanelConfig` from `config/hw_panel.yaml` (built in); `Topic`, `Number`, `NODE_NAME` = `hw_panel` |
| `vehicle` | `Vehicle` from vehicle.yaml (built in): `wheelbase`, `max_steering_angle`, `speed_wheels` (`SpeedWheels`), `wheel_diameter` |
| `rules` | `aav_actuator_tests/safety.py`, ported with its test vectors: `Kind`, `source_kind`, `preflight_refusal` (`Refusal`), `unseen` (`Sample`, `Unseen`), `taken_over` |
| `gate` | `gate(&Situation, Caps)` -> `Gate::On { cap, watch_unseen }` or `Gate::Off(Off)`: SIM and BENCH on (1.0 m/s); REAL on stands only past the preflight's refusal and with a wheel speed seen (0.3 m/s); REAL on the ground, unannounced, REPLAY off |
| `buttons` | `Buttons`: `press(Press)`, `set_go`, `observe_mux`, `zero(Zeroed)`, `command()` -> `Option<Command>`; `Command::from_steering` (ω = v tan δ / L) |
| `state` | `PanelState`, `Update`, `Row`: each sensor's number, OK / STALE / NEVER, rates for the big topics, `kind()`, `estop_on()`, `speed()` (`Speed`: measured, or `SpeedUnknown`: never, stale, NaN), `geofence_holds()`, `fresh_raw()`, `calibration_situation()`; `wheels_note` ("front, not driven") |
| `calibration` | the steering calibration page: `readiness(&Situation)` -> every `NotReady`; `allowance` -> `SoftwareAllows` (on a measured speed, or a person's word) or `NotAllowed`; `CalPage`: `press(CalPress)`, `tick()` -> `Option<Request>` (`Limp`, `Jog(counts)`, `Off`), the marks, `limits()`; `FirmwareState` (ActuatorRaw's byte); `result_file` (the lines a person copies) |
| `steering_limits` | `compute(&Marks)` -> `Limits` (equal limits, sign, gain, warnings) or `MeasurementError`; `firmware_steering`, `vehicle_yaml`, `report`: `aav_legacy`'s `steering_limits.py`, ported |
| `easter_egg` | the little robot (hud-core's `robot`): `watch(...)` -> `Watch` (still? an e-stop or a warning? the screen's fingerprint); `PassWatch::just_passed` (a check just PASSED: it drives across) |
| `graphs` | the GRAPHS page: `Graphs` (60 s of ego's and the mux's commands, the Pi's frames, the ESP32's `rx`, the uplink's counts, the IMU's yaw), `line()` -> `Point`s (gaps at lost lines and restarts), `Stats` (now, avg, low, high), `hops()` -> a `HopReport` per `Hop` (`Verdict`: OK, DIFFERENT, STALE, NO DATA, NOT TESTED), `counts()` (the status strip), `Freezer` (FREEZE, and at an e-stop) |
| `measure` | the MEASURE page (TODO 71): `Odometer` (`update(speed, now)`: \|speed\| x time by the trapezoid rule, gaps over 0.5 s or an unknown speed not counted but told; `distance`, `top`, `mean()`, `reset()`), `Entry` (a number typed in), `wheel_check` -> `WheelCheck` (not yet and why, or the error, the suggested diameter, PASS within 1 %), `report` (SAVE's lines) |
| `esp32` | the ESP32's JSON status lines (`firmware/status/`): `parse_line` -> `StatusLine` (`Heartbeat`, `Status` with `Rx`, `Event`, `Unknown`); tested on `golden.jsonl` |
| `ws` | just enough WebSocket (RFC 6455) to read them: `Url`, `request`, `check_answer`, `Decoder` -> `Message`, `client_frame` (a pong or a close only) |
| `sweep` | `Sweep::start`, `step()` -> the jog target, `outcome()`: `Passed`, or `Stopped` with the leg, why (`Stop`: resistance, no movement, not arrived, position lost, the firmware) and where |

It builds on [`hud-core`](../hud-core/README.md) (`health`, `origin`, `vehicle`, `gps`,
`state::Battery`); the app uses hud-core's `estop` and `heartbeat` too.

## Invariants

- No ROS and no raylib.
- The car moves only while GO is held; letting go sends 0.5 s of zeros, then nothing.
- Speed is never below 0 (no reverse) and never above the gate's cap.
- An e-stop, a takeover (twist_mux passing someone else's command for 0.3 s), a source
  change, the gate going off, or (on stands) 4 s of an unseen speed zero everything;
  GO must be let go and pressed again, and Speed + pressed again.
- Every rule errs towards "this is the real car": nothing announced is REAL, and so is a
  source number this crate doesn't know (where `safety.py` would say SIM).
- `rules`' tests are `test_safety.py`'s vectors; change both together.
- `steering_limits` and `steering_limits.py` test against one file,
  `ros/can_bridge/aav_legacy/test/steering_limits_vectors.json`; change all three together.
- The calibration page asks for nothing while it isn't allowed (an e-stop, the car moving,
  the speed unknown without a person's ON STANDS, STILL, the position unseen), and ends
  what runs when that happens. It says SOFTWARE ALLOWS, never "safe".
- A NaN speed is unknown, never stopped (`state::Speed`): today's ESP32 sends NaN all the
  time. A person's ON STANDS, STILL stands in for it on the calibration page only, and is
  forgotten on an e-stop, a measured movement or leaving the page. The speed buttons get no
  such word: on the real car's stands they stay off with no wheel speed seen.
- Jog targets stay inside `jog_min`..`jog_max`; the lock-to-lock check's stay inside the computed
  limits, and it won't start with the wheels outside them.
- The check stops, and holds where the wheels are, at the first sign of resistance.
- Nothing here writes vehicle.yaml or the firmware: `result_file` and `measure::report` are for a person to copy.
- The GRAPHS page only reads. A hop is OK only when its value changed in the window and
  matched the hop before: ego and the mux pair by stamp, the Pi's frames and the ESP32's
  `rx` by counter and check byte; the Pi must follow the mux (correlation 0.5 or more).
  A parked car is NOT TESTED, never OK. "Sampled N frames", never "every frame": counts
  prove the wire.
- `esp32` and `golden.jsonl` change together (`firmware/status/README.md`).
- The little robot is not still while the car moves, the buttons ask for speed, CENTRE
  runs or the check sweeps; an e-stop, a STALE sensor or a zero is a warning (`easter_egg`).

## Side Effects

- None.

## Verification

```sh
cd ros/hud/aav_hud && cargo test --locked -p panel-core
```

`tools/test.sh` runs it (with `hud-core`, `map-core` and `debug-core`).
