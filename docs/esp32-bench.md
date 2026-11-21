# First flash on the bench: the Rust ESP32

The plan for the first time the Rust program (`firmware/spine/boards/esp32-umod`) goes onto
the car's ESP32 control board. Do it with Ben, **wheels off the ground**, with the C++
ready to go back on. What the program is: [Rust firmware](rust-firmware.md).
Its safety rules (boot, release, lost links, what each failure does): [ESP32 safety](esp32-safety.md).

[[untested]] Nothing here has been done on a board yet. The emulator has (next section).

## Walk me through it: `pixi run esp walk`

One command walks this whole page with you, one step at a time. Each step is a few words
on the screen, in bold colour, and the same words **spoken aloud** (the Mac's `say`).

| Part | What it does | You |
|---|---|---|
| Before you start | wheels off the ground, a person at the e-stop, the RC on | press **Enter** |
| Save the C++ | builds the C++, finds the board, copies its flash (4 MB), builds the Rust | watch; Enter to try again |
| Flash | says "wheels off the ground, e-stop in hand" | type **`flash`** |
| The first check | 30 s of the log; reads each row out ("Brake: pass. The brake is on.") and the board's settings | listen |
| Checks on stands | section 3, one question at a time ("Press the red e-stop now. Did the car stop by itself?") | **yes**, **no**, **skip** |
| Roll back | on a STOP or a no: puts the saved C++ back, listens for its first line | type **`flash`** |
| The record | `bench-logs/<date>-esp32-first-flash.md`: every step, every answer, the table | read it later |

`stop` (or `n`) stops at any step. After the flash, stopping asks: roll back?

| Command | Does |
|---|---|
| `pixi run esp walk` | the whole walk, on the first USB serial port |
| `pixi run esp walk /dev/cu.usbserial-0001` | on that port |
| `pixi run esp walk --dry-run` | a rehearsal: shows every step, **runs nothing** |
| `--quiet`, `--voice Samantha`, `--rate 160` | no speech; another voice; slower or faster |

[[untested]] Tested with a saved log and `--dry-run` only: not yet on a board.

!!! warning "Before you start"
    - The car on stands: **all four wheels off the ground**.
    - A second person at the **red e-stop button**. It cuts power by itself, whatever
      the program does.
    - The RC transmitter on, override switch **off**.
    - Know how to roll back (the last section). Try it once before the first flash.

## Before the board: the emulator

TODO 45 (Ben, 24 Oct): "if it doesn't get a heartbeat then it applys the breaks and stops
the acceleration". Proven first with no hardware at all: the same firmware, in
Espressif's ESP32 emulator (QEMU).

```text
 tools/esp32_qemu.py                         QEMU: an ESP32, running our firmware
   aav_legacy's frames, 50 Hz ──► UART1 RX ──► LegacyLink ──► spine-core ──► drivers
   reads the uplink frames ◄───── UART1 TX ◄── its status, 50 Hz      │        │
   fake iBUS, every 7 ms ───────► UART2 RX ──► the RC receiver ───────┘        │
   reads the log          ◄────── UART0    ◄── EVENT / STATUS / DEVICES lines  │
                                                       stand-in Jrks, DAC ◄────┘
```

| Command | Does |
|---|---|
| `pixi run esp qemu-tools` | downloads QEMU into `vendor/qemu` (once, 22 MB; macOS on Apple silicon) |
| `pixi run esp qemu-check` | builds with `--features qemu,uplink`, boots it, runs 6 cycles, then the RC's modes, prints three tables (`-- --no-rc` skips the RC). The frames come from the Rust bridge's relay (`aav_legacy_rs`, the car's default since 17 Nov); `-- --bridge python` uses the Python bridge's |

Each cycle: stand-still frames (the arming stop), drive frames (0.6 m/s), a **gap**, then
drive frames again with no stop first. Since 24 Oct the frames are version 2, with the
Orin's counter moving one a frame (TODO 50,
[the heartbeat](rust-firmware.md#the-heartbeat-version-2-of-the-serial-frame-24-oct)).
The gaps, in turn:

| Gap | The Motor Pi sends | Means |
|---|---|---|
| silence | nothing | the Pi or its cable is gone |
| frozen | drive frames at 50 Hz, the **same counter** | the Pi alive, the Orin gone |
| bad_crc | drive frames at 50 Hz, the counter moving, **every CRC wrong** | a bad cable: each frame ignored and counted |

### Results with the heartbeat, 24 Oct (Ben's Mac)

| Cycle | Gap | Drove (DAC peak) | Brake full + DAC 0 after the last new counter | Gap frames ignored | Fault bit 1 | Stays braked until a stop |
|---|---|---|---|---|---|---|
| 1 | silence | yes (140) | 119 ms | - | yes | yes |
| 2 | frozen | yes (140) | 113 ms | 30 of 30 (`repeats`) | yes | yes |
| 3 | bad_crc | yes (140) | 112 ms | 30 of 30 (`bad`) | yes | yes |
| 4 | silence | yes (140) | 113 ms | - | yes | yes |
| 5 | frozen | yes (140) | 118 ms | 30 of 30 | yes | yes |
| 6 | bad_crc | yes (140) | 111 ms | 30 of 30 | yes | yes |

A second run, inside `pixi run test`, passed too: 111 to 120 ms.

The check fails when it should: with the counter rule turned off on purpose, the frozen
cycle kept driving (no timeout, 0 repeats counted) and failed.

The firmware logs a `LINK` line with each `STATUS`: `frames=`, `bad=`, `skipped=`,
`repeats=`, `counter=`.

### Results: the uplink and the RC's modes, 24 Oct (Ben's Mac)

TODO 50's uplink and TODO 48's modes ([the uplink](rust-firmware.md#the-uplink-the-esp32s-status-back-to-the-pi-24-oct)).
The same run as the heartbeat's cycles (111-119 ms, all passed).

Each cycle's uplink frames:

| Cycle | Gap | Frames a second | Echo = a counter just sent | Computer, armed while driving | Said stopped + fault 1 after | Echo stays frozen in the gap |
|---|---|---|---|---|---|---|
| 1 | silence | 50 | yes | yes | 121 ms | - |
| 2 | frozen | 51 | yes | yes | 118 ms | yes |
| 3 | bad_crc | 51 | yes | yes | 115 ms | - |
| 4 | silence | 50 | yes | yes | 129 ms | - |
| 5 | frozen | 51 | yes | yes | 118 ms | yes |
| 6 | bad_crc | 50 | yes | yes | 128 ms | - |

No uplink frame lost over the run (the sequence numbers).

Then the RC, with fake iBUS frames on UART2 (CH6: 1500 = RC via ROS, 2000 = RC direct;
the throttle stick full, the gear forward):

| Step | Does | Seen | Result |
|---|---|---|---|
| 1 | RC via ROS; the frames say stand still | the uplink says RC via ROS, computer, RC throttle 1.0; **the DAC stays 0** | pass |
| 2 | RC via ROS; the frames drive | the DAC follows the frames (140) | pass |
| 3 | RC direct; the frames say stand still | mode RC direct; **the sticks drive** (DAC 346) | pass |
| 4 | RC direct, then the receiver silent | fault 256 after **119 ms**, brake full, DAC 0 | pass |
| 5 | RC via ROS, driving, then the receiver silent | fault 256 after **117 ms**, brake full, DAC 0 | pass |
| 6 | the receiver back on RC via ROS, drive frames, no stop | stays stopped and latched | pass |

The check fails when it should: built with the two-position map instead (`RcMap::TODAY`),
steps 1, 2, 5 and 6 failed.

### Results before the heartbeat, 24 Oct (version 1 frames, silence only)

| Cycle | Drove (DAC peak) | Timeout after the last frame | Full brake (3500) + DAC 0 | Fault bit 1 | Stays braked until a stop frame |
|---|---|---|---|---|---|
| 1 | yes (140) | 116 ms | 116 ms | yes | yes |
| 2 | yes (140) | 113 ms | 113 ms | yes | yes |
| 3 | yes (140) | 113 ms | 113 ms | yes | yes |

Over six runs: **111 to 120 ms**, every cycle passed. That is the 100 ms timeout, plus
up to two 10 ms ticks (the tick that reads the last frame, the tick that notices).
Within the 0.2 s of check 8 below. The check fails above 200 ms: a firmware built with a
400 ms timeout, on purpose, failed it (410-417 ms).

The times are the firmware's own clock. QEMU's clock follows the Mac's (no `-icount`):
over each run they agreed to 0.1 %.

### What it did and didn't exercise

| Part | In QEMU |
|---|---|
| spine-core, the 100 ms timeout, the latch, the re-arm | ✅ the real code |
| The UART link: version 2 frames, the CRC, resync, the counter rule (`legacy-link`) | ✅ the real code, real bytes on UART1 |
| The uplink: the status frame on UART1's TX, its echo, the mode | ✅ the real code; read by aav_legacy's own `UplinkParser` |
| The RC receiver (iBUS on UART2), the three modes, a lost RC | ✅ the real code; the frames are fake, made by the script |
| The Jrk, DAC and expander drivers, and their order | ✅ the real code; the bytes go to stand-ins |
| The hardware watchdog (TIMG1, 100 ms) | ✅ on and fed; it never fired |
| The Jrks, the DAC, the expander themselves | ❌ stand-ins (`spine_board::StandInBus`): QEMU can't add I2C devices |
| Embassy's timer and interrupts | ❌ a busy-wait tick instead: QEMU's interrupt matrix has no status registers |
| The Jrks' real feedback | ❌ the stand-ins say "analog" and report their target |
| The LEDs, CAN | ❌ unwired, not built |
| The real chip's timing, the Motor Pi, the cable | ❌ the bench, below |

### Found on the way

| What | Fixed |
|---|---|
| The firmware never turned the FPU on: it relied on the chip's reset state. QEMU starts with it off, and the first interrupt crashed (a double exception) | ✅ `enable_fpu()` in `main.rs`, on every build |
| The log had no times | ✅ `t=` (ms since boot) on every line, and an `EVENT` line at each change |

## What you need

| Thing | Why |
|---|---|
| A laptop with this repo, `pixi run esp tools` done | builds and flashes |
| A USB cable to the ESP32 MicroMod's USB port | flashing and the log |
| The Motor-Pi running `aav_legacy` (`pixi run car` profile, or `legacy.launch.py`), with **`frame_version: 2`** in `legacy.yaml` | sends version 2 frames, with the Orin's counter. The car's file says 1 while the C++ is on |
| The Orin running the output mux (the car profile), or a laptop as the Orin | makes the counter: no Orin, no heartbeat, the car stays braked |
| The RC transmitter | the override; for the three modes, a three-position switch on CH6 (H10) |
| For the uplink: the wire from the ESP32's TX1 (GPIO17) to the Pi's RX, confirmed by the hardware lead (H9) | the status back up; without it, skip "The uplink and the RC modes" |
| A multimeter | the DAC output, the direction pin |

## 1. Build, and save the C++ first

| Step | Command | Check |
|---|---|---|
| 1 | `pixi run -e firmware legacy-esp32` | the C++ still builds: it is the way back |
| 2 | `vendor/cargo-tools/bin/espflash read-flash 0 0x400000 backup-esp32.bin` | a copy of what is on the board now (4 MB). Keep it off git |
| 3 | `pixi run esp build` (version 2 frames, with the uplink; `-- --no-default-features --features legacy-uart,link-v1` for the 13-byte ones) | `Finished`; the program is `firmware/spine/boards/esp32-umod/target/xtensa-esp32-none-elf/release/spine-esp32-umod` |

## 2. Flash

| Step | Command | Check |
|---|---|---|
| 1 | `vendor/cargo-tools/bin/espflash list-ports` | the board's port (a Mac: `/dev/cu.usbserial-…`) |
| 2 | `pixi run esp flash` (add `--port <port>` if asked) | it flashes, resets, and shows the log |
| 3 | read the first lines | `spine-esp32-umod 0.1.0, link: Motor-Pi UART, version 2 frames (the Orin's heartbeat); the last reset: …`, then `CONFIG …` and `DEVICES 4/4 answered …` |
| 4 | leave the monitor: **Ctrl-C** | the port is free for the first check |

If it won't connect: hold the board's BOOT button while it starts flashing.

## 2a. The first check: the gate

Before any check below, let the board judge itself. Pi's bridge **off**, RC transmitter
on, nothing else.

| Step | Command | Check |
|---|---|---|
| 1 | `pixi run esp first-check` (or `pixi run esp first-check /dev/cu.usbserial-…`) | it reads the log for 30 s; it sends nothing |
| 2 | read the last line | `SAFE TO GO ON to the next step`: go to 3. `STOP: <check>, roll back`: section 4 |

Opening the port can restart the board (its reset wiring): that is fine, the check sees
the boot. `--save board.log` keeps what it read; `--pi on` once the bridge runs (then the
link must say OK); `--json` for a machine.

| Check | PASS when | If not |
|---|---|---|
| Build | the commit in the `CONFIG` line is this checkout's | flash again |
| Features | `legacy-uart` (`json-log`, `wifi-status` allowed), never `qemu` | build the right one |
| Last reset | a power-on | a watchdog (a tick hung), a panic, a brown-out: roll back |
| Devices | all four answered at boot: steering Jrk 0x05, brake Jrk 0x04, DAC 0x0C, expander 0x20; each Jrk's feedback mode | a wire, a power, an address |
| Brake | at `IDLE_BRAKE` (1.0: full on) | roll back |
| Throttle | 0 | **e-stop**, roll back |
| Steering | centred (1950) | roll back |
| Mode | stopped, not armed, latched | roll back |
| Faults | none, or only 1 (command timeout) and 256 (RC lost: the transmitter is off) | the fault names the device |
| Tick | the worst gap under 50 ms, half the watchdog | roll back |
| Restarts | none in the 30 s | roll back |
| Pi link | INFO only: says what the board sees. NO FRAMES is right with the bridge off | |

The Pi link, judged on the board by the spine's own 100 ms rule (`spine_board::LinkHealth`):

| The board says | Means |
|---|---|
| OK | frames with the Orin's counter moving |
| NO FRAMES | nothing yet: the bridge is off, or the cable |
| STALE | frames came, then stopped |
| COUNTER FROZEN | the Pi sends, the Orin's counter doesn't move: is the output mux running? |
| BAD FRAMES | bad frames or stray bytes in the last 3 s: the cable, or `frame_version` (2 for this build) |

The board also says it once a second on the cable (`HEALTH link=… age_ms=… counter_moving=…
devices=4/4 faults=…`), and with `wifi-status` at `/health` and in big words on its page.

The QEMU build, booted idle (`pixi run esp first-check --qemu`, 7 Nov, Ben's Mac):

```text
Check       Result  What it saw
----------  ------  -----------
Log         PASS    245 lines, text form
Build       PASS    216ca6b
Features    PASS    legacy-uart,qemu,uplink
Last reset  PASS    power-on
Devices     PASS    4/4: steering 0x05 answered, feedback analog; brake 0x04 answered, ...
Brake       PASS    1.00 (IDLE_BRAKE 1.0)
Throttle    PASS    0 in every line
Steering    PASS    centred
Mode        PASS    stopped, not armed, latched
Faults      PASS    0x0000
Tick        PASS    worst gap 10.1 ms (under 50); 0 late ticks in the window
Restarts    PASS    none in the window
Pi link     INFO    NO FRAMES: right while the Pi's bridge is off

SAFE TO GO ON to the next step
```

[[untested]] Not run on a board yet. On the board, the text log's own prints wait on the
UART, so the tick may come late now and then (`Late ticks`, INFO); the timing test in the
board's README measures it.

## 3. Checks, wheels off the ground

Only after 2a said SAFE TO GO ON. Do them in order. Stop at the first one that fails, and
roll back.

| # | Do | Expect |
|---|---|---|
| 1 | Power on, nothing else | brake pulled (3500), steering centred (1950), DAC 0 V. Log: `STATUS t=… mode=Manual … faults=0x0000`. Status LED blinks fast |
| 2 | Unplug a Jrk's I2C, then plug it back | the red LED and a fault bit (8 steering, 16 brake) while it is out |
| 3 | Start the Motor-Pi's bridge (`frame_version: 2`), with ego stale | the bridge sends stop frames: brake stays full. `LINK` shows `frames=` rising, `bad=0`, `counter=` still. `HEALTH link=ok` with the Orin's mux running, `link=counter_frozen` without |
| 4 | Drive slowly from ROS (keyboard or joystick, 0.3 m/s) | `mode=Computer`, green LED, brake lets go (400), the DAC rises smoothly (600 codes a second at most), the wheels turn **forward** |
| 5 | Steer left from ROS | the wheels turn **left**. If right: the steering sign (Q2), in `legacy.yaml` |
| 6 | Stop from ROS | throttle 0; the bridge's hold brake: target 1050, as the C++ gave it |
| 7 | Reverse slowly | the direction changes only with the DAC at 0; the wheels turn backwards |
| 8 | While driving, stop the bridge (Ctrl-C) | within 0.2 s: throttle 0, brake full, `faults` has 1 (timeout). The `EVENT` line's `t=` says when (QEMU: 111-120 ms) |
| 9 | Start the bridge again | brake stays on until ego is stale or stops (a stop frame), then it drives again |
| 9a | While driving, stop the Orin's output mux (or unplug the Orin's network), the bridge still running | within 0.2 s: brake full, throttle 0, `faults` has 1, though `LINK frames=` still rises; `repeats=` rises instead. The heartbeat (TODO 50) |
| 9b | Bring the Orin back | stays braked until ego stands still (a stop with a new counter), then drives |
| 9c | `/safety/estop` true while driving | `estop=true` at once, brake full |
| 10 | CH6 high (manual). **First the left stick in the middle** (its brake band: RC direct waits for this after power-up, any stop and any move of CH6), then pushed away a little | `mode=Rc`, blue LED, wheels turn; ROS commands ignored. Without the first step: nothing moves, as it should |
| 11 | RC: switch the transmitter off | within 0.1 s: brake full, `faults` has 256 (RC lost) |
| 12 | RC override off | back to the computer, or Manual |
| 13 | The red e-stop button | the car stops by itself, as today |
| 14 | 10 minutes of 1 to 13 (and 9a-9c), mixed | no unexpected resets in the log (`the last reset:` stays the power-on one) |

Compare with the C++ (the table "The C++ beside the Rust" in
[Rust firmware](rust-firmware.md)): the differences marked ⚠️ are expected.

## The uplink and the RC modes, on the bench

Only once the hardware lead says TX1 (GPIO17) reaches the Pi's RX (H9). Wheels still off
the ground; checks 1 to 14 passed first.

| Step | Do |
|---|---|
| 1 | `pixi run esp flash`: the uplink is on by default. The first log line ends `with the uplink on TX1 and the three-position mode switch` |
| 2 | `legacy.yaml`: `frame_version: 2`, **`uplink: true`**; restart the bridge |

| # | Do | Expect |
|---|---|---|
| U1 | Bridge running, nothing else | the bridge logs `uplink: frames from the ESP32`; `ros2 topic hz /vehicle/rc` about 50; `/vehicle/status` 10. The ESP32's `LINK` line: `uplinks=` rising, `uplink_errors=0` |
| U2 | Unplug TX1's wire | within 0.2 s the bridge warns `no frame from the ESP32`; the car still brakes and drives as before (the uplink is only news) |
| U3 | Drive slowly from ROS | `/vehicle/status` mode 2 (computer), armed; the echo follows the Orin's counter (`/control/car/heartbeat`) |
| U4 | Steer by hand from ROS, left then right | `/vehicle/steering` measured follows, + left. NaN: that Jrk has no feedback set up (D6) |
| U5 | Brake from ROS (stop) | `/vehicle/brake` measured rises to about the command (0.3 hold, 1.0 full) |
| U6 | Stop the Orin's output mux | `/vehicle/status` faults has 1 within 0.2 s, mode 0 |
| R1 | Transmitter on, CH6 **low**, the left stick in the middle | `/vehicle/rc` mode_switch 1 (normal); the computer may drive. The left stick pushed would take over (normal mode's arbiter, `docs/firmware.md`) |
| R2 | CH6 **middle** (RC via ROS), sticks centred | mode_switch 2; `rc_driver` logs `RC via ROS`; `/drivers/rc/cmd_vel` zeros at 50 Hz; twist_mux passes it (`/control/cmd_vel`) |
| R3 | The left stick pushed away a little | the wheels turn, **through ego**: its speed and steering-rate limits; `/vehicle/status` mode 2 |
| R4 | Steer with the stick while rolling | the wheels steer (standing still they don't: the bicycle model) |
| R5 | Transmitter off while on RC via ROS | within 0.1 s: brake full, faults 256; stays braked when it comes back, until the stick is centred (a stop), then drives |
| R6 | CH6 **high** (RC direct); the left stick in the middle first | `/vehicle/status` mode 1; the ESP32 drives from the sticks, as check 10; ROS can't |
| R7 | Transmitter off while on RC direct | as check 11: brake full, faults 256 |
| R8 | Stop the bridge while on RC via ROS | the heartbeat stops: brake full within 0.2 s (fault 1). RC direct still works: the fallback |

## 4. Roll back to the C++

Any time something is wrong:

| Step | Command |
|---|---|
| 1 | press the e-stop; unplug the Motor-Pi's bridge |
| 2 | `pixi run -e firmware legacy-esp32 -t upload` (PlatformIO builds and flashes the C++; `src/secrets.h` for its WiFi). Set `frame_version: 1` back in `legacy.yaml`: the C++ reads only the 13-byte frame |
| 3 | or the backup: `vendor/cargo-tools/bin/espflash write-bin 0 backup-esp32.bin` |
| 4 | check the C++ is back: its log says `=== ESP32: AAV Controller ===` |

## After the bench

| When | Then |
|---|---|
| All 14 checks pass | the same on the ground, at walking speed, with Ben |
| Proven on the car | Ben decides: the C++ in `firmware/legacy/aav_umod_esp32` is removed, and `docs/firmware.md` updated |
