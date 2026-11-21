# The ESP32: every C++ behaviour, and the Rust's

The 2025-26 team's C++ (`firmware/legacy/aav_umod_esp32/src/`) drives the car today. The
Rust (`firmware/spine/boards/esp32-umod`) replaces it. Ben, 7 Nov: "we should have all the
features from c++ version at a minimum … We don't know why they made their choice so it's
best to assume it was for good reasons."

This page lists every behaviour of the C++, and where the Rust has it.

[[untested]] Nothing here has run on a board. The QEMU checks have (last section).

## How to read the tables

| Mark | Means |
|---|---|
| **Built** | the Rust does it, in the default build or behind the feature named |
| **Gap** | the Rust doesn't, and why |
| **Not needed** | the Rust doesn't, on purpose; the reason cites the code |

The C++ paths are under `src/` (`modules/` dropped). The Rust paths are under
`firmware/spine/`.

## The count

| Built | Gap | Not needed |
|---|---|---|
| 42 | 0 | 16 |

No gap is left. Three of the C++'s behaviours change how the car moves, so they are
**off by default**, behind features (next section).

## Three behaviours behind features

The default build keeps the team's choices. These features give the C++'s instead.

| Feature | The C++ | The default build |
|---|---|---|
| `cpp-stop-centre` | a safe stop centres the wheels (`jrkSafeStop`) | holds them where they are (`ESTOP_STEERING = Hold`) |
| `cpp-rc-steering` | RC right turns go 1950 counts from centre, left 1550 | the same on both sides, 1550 |
| `cpp-throttle-ramp` | the DAC moves 6 codes up or 12 down a **frame** (50 Hz: 300 up, 600 down a second); a brake demand or a stop cuts it at once | 600 up a second; down at once |
| `cpp-parity` | all three | |

`cpp-throttle-ramp` is for the UART link only (a `compile_error!` on CAN).

!!! note "A finding"
    The C++ ramp is called once a frame, not once a loop. So on the Motor-Pi's link it
    rose at **300 codes a second**, half the Rust default's 600.

## Boot

| What the C++ does | Where | Rust today | Mark |
|---|---|---|---|
| USB log at 115200 baud | `main.cpp:setup` | UART0 at 115200 (`main.rs`) | Built |
| waits 1 s before anything | `main.cpp:setup` (`delay(1000)`) | no wait: throttle 0 and brake on at once | Not needed: `SpineBoard::init` makes the car safe first; a slow Jrk is tried again every tick (`spine-board/src/actuators.rs`) |
| banner `=== ESP32: AAV Controller ===` | `main.cpp:setup` | the same, with `(Rust, spine-esp32-umod 0.1.0)` | Built (`boot_pin_lines`) |
| status LED on; red, green, blue off | `main.cpp:setup` | the same; then the status LED blinks (slowly while the computer drives) | Built (`main.rs`, `Leds`) |
| `Serial1 (ROS UART) @ GPIO16/17` line | `main.cpp:setup` | the same, with `115200 8N1` | Built (`boot_pin_lines`) |
| `iBUS Serial2 @ GPIO32/33` line | `main.cpp:setup` | `iBUS Serial2 @ GPIO32, 115200` (TX unused) | Built |
| `I2C -> SDA:21 SCL:22` line | `main.cpp:setup` | the same, with `100 kHz` | Built |
| I2C start retried for 3 s | `main.cpp:setup` | esp-hal's I2C can't fail to start | Not needed: `I2c::new` takes the pins; a missing device is a fault per tick |
| PCA9555: all outputs, all high; `PCA9555 ready` | `main.cpp:setup` | the same; the line | Built (`Actuators::init`, `cpp_log::boot_devices`) |
| DAC looked for 3 s; `DAC OK` or `WARNING: DAC not found — no throttle` | `main.cpp:setup`, `dac.cpp:detectDAC` | probed once; the same lines | Built (`DeviceCheck`, `cpp_log`). Not retried: the DAC is written every tick, and a missing one is the `THROTTLE_DAC` fault |
| DAC 0 at boot | `main.cpp:setup` | the same | Built (`Actuators::init`) |
| steering Jrk pinged, centred (1950); its line | `main.cpp:setup`, `jrk_g2.cpp:jrkPing` | the same; `JRK STEER (0x05) OK - centred (1950)` | Built |
| brake Jrk pinged, full on (3500); its line | `main.cpp:setup` | the same; `JRK BRAKE (0x04) OK - engaged (3500)` | Built |
| a missing Jrk is never written again | `jrk_g2.cpp:setSteer` (`if (!jrk_steer_ok) return`) | written every tick; its fault stops the car | Not needed: a fault the car can't see is worse (`spine-board` README, "Changed on purpose") |
| red LED on if the DAC is missing | `main.cpp:setup` | red on for any fault | Built (`THROTTLE_DAC` lights it) |
| `Ready — SwA: RC_EN …` line | `main.cpp:setup` | `Ready - CH6 mode: RC direct above 1500 …`, from the RC map | Built (`cpp_log::RcReady`) |

## The Motor-Pi's frames

| What the C++ does | Where | Rust today | Mark |
|---|---|---|---|
| 13-byte frames: three floats and a sum | `ros2_handler.cpp:handleROS2Messages` | the same, and versions 2 to 4 | Built (`legacy-link`) |
| a bad sum slides one byte and tries again | the same | the same | Built (`legacy-link`, the parser) |
| a value outside -1..1: frame rejected, throttle a step down | the same | the same: throttle 0, through the ramp if on | Built (`LegacyLink::take_v1`) |
| steering: 1950 counts a unit right, 1550 left | the same | the same | Built (`Frame::targets`) |
| brake below -0.05: `clamp(\|accel\| × 3500, 400, 3500)`, DAC cut | the same | the same | Built (`Frame::targets`) |
| no brake: brake released (400) | the same | the same | Built |
| speed below 0.03: no throttle | the same | the same | Built (`SPEED_DEADBAND`) |
| throttle `\|speed\| × 700`, capped at 700 | the same | the same | Built (`DAC_CAP`) |
| direction pin, then 1 ms, then the ramp | the same | direction only with the DAC at 0 | Built, safer (`spine-board` README, "Changed on purpose") |
| the DAC ramp, 6 up and 12 down a frame | `dac.cpp:writeDACRamped` | 600 up a second, down at once | Built behind `cpp-throttle-ramp` (`legacy_link::DacRamp`) |
| no command timeout: the last frame holds | `main.cpp:loop` | stop after 100 ms | Not needed: the 100 ms watchdog is a rule (`spine-core`) |
| `ROS: steer=… DAC=…`, `DBG: …` each frame | `ros2_handler.cpp` | none | Not needed: 50 lines a second would hold the tick (`TextLog`); `STATUS` says it 5 times a second |
| `ROS: BRAKE ENGAGED — DAC cut` each frame | the same | a `FAULT` or `EVENT` line on change | Not needed: the event line on a change says it once |
| `ROS: packet rejected — out of range` | the same | counted in `LINK` and `HEALTH` (`bad=`) | Built (as a count) |

## The RC

| What the C++ does | Where | Rust today | Mark |
|---|---|---|---|
| RC mode while CH6 reads 1500 to 2200 | `main.cpp:loop` | `RcMap::TODAY.direct_above` 1500 | Built (`ibus`) |
| channels usable only between 100 and 2200 | `rc_handler.cpp:handleRCMode` | `valid_above`, `valid_below` | Built |
| gear: forward below 1200, reverse above 1800, brake between | the same | not read: the left stick's whole travel says forward, brake band, reverse | Changed on purpose (21 Nov): SwC's wiring is loose |
| gear brake: brake full, DAC down | the same | the left stick's brake band (±30 %): brake full | Changed on purpose (21 Nov) |
| throttle only past -0.05 of the stick | the same | past the brake band, rescaled from 0 at its edge | Changed on purpose (21 Nov) |
| steering: 1950 counts right, 1550 left | the same | 1550 both sides | Built behind `cpp-rc-steering` (`steer_right_gain`) |
| RC throttle up to 1023, capped at 700 | the same, `writeDACRamped` | capped by `MAX_SPEED`, 1 m/s | Not needed: one forward limit for every driver (`spine` README, #23) |
| RC idle: brake released, coasting | the same | the stick in its band: the full brake | Not needed: the team's #23 rule, then the band (21 Nov), replaced it (`spine-core`) |
| lost RC in RC mode: safe stop, `RC: No valid signal - SAFE STOP` | the same | the `RC_LOST` fault; a `FAULT … set=RC_LOST` line | Built (`cpp_log::FaultChange`) |
| a safe stop centres the wheels | `jrk_g2.cpp:jrkSafeStop` | held | Built behind `cpp-stop-centre` |
| ROS frames not read in RC mode | `main.cpp:loop` | read and counted; RC wins | Not needed: the C++ then ran stale frames from the UART's buffer when RC let go |
| `RC FWD: …`, `RC BRAKE: …` each loop | `rc_handler.cpp` | none | Not needed: about 1000 lines a second |

## The devices

| What the C++ does | Where | Rust today | Mark |
|---|---|---|---|
| Jrk target clamped to 400..3500 | `jrk_g2.cpp:setSteer`, `setBrake` | the same | Built (`Calibration::TODAY`) |
| Jrk errors read after each target; `JRK STEER ERR: 0x…` | the same | read once a tick; a `JRK STEER ERR t=… flags=0x…` line on change; a fault | Built (`Actuators::jrk_errors`, `cpp_log::JrkErrorLine`) |
| `ERROR: DAC did not acknowledge!` | `dac.cpp:writeDAC` | the `THROTTLE_DAC` fault; its `FAULT` line | Built |
| DAC code: 10 bits, two bytes | `dac.cpp:writeDAC` | the same | Built (`ad5311`) |
| `DAC hw write: …` each write | `dac.cpp:writeDACRamped` | none | Not needed: one a frame or a loop; `STATUS` has the throttle |
| `JRK SAFE STOP` line | `jrk_g2.cpp:jrkSafeStop` | the `FAULT` and `EVENT` lines | Built |
| I2C at 100 kHz | Arduino's `Wire` default | `I2C_KHZ` 100 | Built |
| the unused `JRK_STEER_*` constants | `hardware_configs.h` | none | Not needed: nothing in the C++ reads them |

## The status line and the page

| What the C++ does | Where | Rust today | Mark |
|---|---|---|---|
| `STATUS:steer,speed,accel \| STR:OK BRK:OK` every 200 ms | `web_handler.cpp:publishStatus`, `main.cpp:loop` | the same line, every 200 ms, text log | Built (`cpp_log::CppStatus`) |
| …and after every frame and RC loop | `ros2_handler.cpp`, `rc_handler.cpp` | none | Not needed: a flood; 200 ms is enough to read |
| the status page on `/` and `/status` | `web_handler.cpp:handleStatus` | `/` and `/status`, with graphs (`wifi-status`) | Built |
| the log as a WebSocket on port 81 | `websoc_serial.h`, `main.cpp` | `/log` on port 80; the JSON lines on `/ws` | Built (another port) |
| early boot lines sent to the WebSocket | `main.cpp:setup` | none | Not needed: no client can be on yet; `CONFIG` and `DEVICES` repeat every 10 s |
| `/scan`: every address, named, as text; `Found: 0x05 <- JRK steer` lines | `web_handler.cpp:handleScan` | `/scan` as JSON with `names`; the same log lines; only while stopped | Built (`BusScan`, `cpp_log::ScanFound`) |
| `OTA -> http://…/update`, `Scan -> …` lines | `main.cpp:setup` | the same lines | Built |

## WiFi and the update

| What the C++ does | Where | Rust today | Mark |
|---|---|---|---|
| WiFi credentials from an untracked file | `main.cpp`, `secrets.h` | `src/wifi_secrets.rs` (untracked) | Built |
| joins for 10 s, blocking the boot | `main.cpp:setup` | joins in the background, again and again | Not needed: the tick never waits on the WiFi (board README) |
| green LED: WiFi joined; red: it failed | `main.cpp:setup` | green: the computer drives; red: a fault | Not needed: the car-day build has no WiFi; the LEDs say what drives |
| **firmware update** over WiFi, `/update` | `main.cpp` (`HTTPUpdateServer`) | `/update` on port 8080, behind `wifi-update` | Built (below) |

## The update over WiFi (`wifi-update`)

The C++'s `/update` takes any file from anyone, at any time, while the car drives. Ours
takes one only when all of these hold:

| Check | Where |
|---|---|
| the feature `wifi-update` is on: off by default, never in the car-day build | `Cargo.toml`, `tools/car_day_build.sh` (`tools/tests/test_esp32_settings.py`) |
| the password, in the `X-Update-Password` header, matches the untracked secrets file's; 12 characters at least; never the example's; 10 s lockout after a wrong one | `app-update/src/gate.rs` (`Guard`) |
| the car is parked: nobody driving, not armed, throttle 0, brake full on; checked before, at every 4 KB, and before the switch | `app-update/src/gate.rs` (`may_update`) |
| the file is an ESP32 app image: the magic, the chip, the segments, the checksum, the SHA-256, our app's name, it fits the slot | `app-update/src/image.rs` (`ImageCheck`) |
| written to the other OTA slot, then read back (CRC-32), then the slot's SHA-256 checked on flash | `boards/esp32-umod/src/update.rs` |
| the boot switches only then; the reboot waits 1 s, and only while parked | the same |

How to use it (bench only, wheels off the ground):

1. Flash `partitions-ota.csv` once over USB: the default table has no OTA slots, and
   the update says `409` without them.
2. Make the file: `espflash save-image --chip esp32 <the ELF> app.bin`.
3. `curl -H "X-Update-Password: …" --data-binary @app.bin http://<ip>:8080/update`.
4. The reply says the bytes, the version and the slot; the board restarts.

!!! warning "Limits"
    - The password crosses the WiFi in plain HTTP. Use the bench network only.
    - Each 4 KB erase holds the tick about 45 ms (esp-storage turns interrupts off).
      The car is parked, so nothing moves, but the `TICK` line shows it. A slow flash
      chip's erase (up to 400 ms) trips the 100 ms watchdog: the chip restarts into the
      old program, which is still the one to boot.
    - No browser form: `curl` only.

## Settings the CONFIG line states

Ben, 7 Nov: "It would be worth knowing its settings including steering limits and speed
limits and braking pressure limits, communication rates…". The `CONFIG` line (text, JSON,
and WiFi `/config`) states each one, from the constants the code runs with. Every key, and
where it comes from: the board README, "What the CONFIG line says".

## What was run

| Check | Result |
|---|---|
| `cargo test` (the crates) | passed |
| `tools/esp32.sh check` | fmt, clippy on 13 mixes, the default build and the `json-log,wifi-update` link: passed |
| `tools/esp32.sh qemu-check` | PASS |
| `tools/esp32.sh first-check --qemu` | SAFE TO GO ON |
| a real `wifi-update` image (874 KB) through `ImageCheck` | passed |

Untested: anything on the board; the update's flash writes and the switch; the WiFi; the
three `cpp-*` features in QEMU.
