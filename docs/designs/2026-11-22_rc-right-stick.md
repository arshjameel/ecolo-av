# The RC: the right stick takes and gives, the left stick is one pedal

Arsh and Ben, 21 Nov. Built in the ESP32's Rust (`ibus`, `spine-core`), 22 Nov.
Replaces the brake band and the 1 s race of 21 Nov ([firmware](../firmware.md#the-rc-transmitter-arsh-21-nov)).
Untested on the car.

## Why

The first version used the left stick's middle as both "brake" and "hands off, let the
computer drive", with a 1 s window where whoever asked first got the car. That made
handing over a matter of timing. The FS-i6X's right stick springs back to the middle, and
its up and down (CH2) was unused, so it now does the handing over on purpose. That frees
the left stick to work like one pedal (Ben's idea): the middle lets the motor slow the
car, instead of a full brake.

## The controls

```text
right stick (CH2, springs back)      left stick (CH3, stays where it is put)
   up past 88 %: give the car             up: forward
   to the computer (steering              middle ±20 %: throttle off, the motor slows the car
   centred, held 0.5 s)                   back: reverse
   down past 88 %: hard brake,            past 88 % the other way while still moving:
   the RC takes the car                   the hard brake too
```

| Input | Does |
|---|---|
| Right stick past +88 % (down) | the hard brake while it is held, and the RC takes the car from the computer. On the first frame, whatever the steering |
| Right stick past -88 % (up), steering (CH4) within ±15 %, held 0.5 s | the computer gets the car (normal mode only; ignored in manual) |
| Let go of the right stick | braked at least 1 s, and until the left stick leaves the zone it was in |
| Left stick up or back | the car drives that way at a fixed speed: 1.0 m/s forward, 0.5 m/s back, ramped at 0.5 m/s² |
| Left stick in the middle | throttle off, no brake: the car slows as the motor controller decides (no regen of our own: `docs/firmware.md`, "Braking with the motor") |
| Left stick the other way while the car moves | throttle off; past ±88 % the hard brake too. Then the new way once stopped |

"Stopped" is under 0.05 m/s for 0.5 s, when the spine has a speed. Today's ESP32 has none
(the Motor Pi doesn't send frame version 4 yet), so there the left stick must be held past
88 % for 1 s (the hard brake) before the direction changes.

## The values

All to check on a car day.

| What | Value | Where |
|---|---|---|
| Stick ends, both sticks | 88 % | `RcMap::full` |
| Steering centred, to give | ±15 % | `RcMap::centred` |
| Left stick middle | ±20 % (±30 % until 26 Nov) | `RcMap::brake_band` |
| Held up, to give | 0.5 s | `RC_GIVE_MS` |
| Braked after a stop | at least 1 s | `RC_BRAKE_MS` |
| Stopped | under 0.05 m/s for 0.5 s | `STANDSTILL_SPEED`, `RC_STOPPED_MS` |
| Forward, reverse | 1.0, 0.5 m/s | `max_speed`, `max_reverse_speed` |
| Speeding up | 0.5 m/s² | `RC_ACCEL` |

The FS-i6X's sticks reach -100 % up and +97 % down (Arsh, 21 Nov), with every subtrim 0,
so 88 % leaves 9 % to spare.

## The transmitter

The failsafe saves where a stick or switch is while CANCEL is **held** (the FS-i6X manual,
8.3). On top of the 21 Nov settings (CH6 -100 %, CH3 0 %, CH4 0 %, End points CH6 low 80 %):

| Channel | Failsafe | Why |
|---|---|---|
| CH2 | the right stick held **fully down** (about +97 %) | a dropped link holds the stop. Never up: that is the give |

## Choices made while building it (for Arsh and Ben to check)

| # | Choice | Why |
|---|---|---|
| 1 | At power-up, after any stop and when the mode switch moves, the RC has the car in normal mode: the computer drives only after a give | a glitch or a reboot never hands the car to the computer by itself |
| 2 | With no speed, only the hard brake held 1 s changes direction; the stick short of 88 % never reverses | the spine can't tell the car stopped otherwise |
| 3 | Any stick out of range (CH2, CH3 or CH4) is the right stick's stop | as before: a bad stick brakes |
| 4 | The right stick's stop also brakes in RC via ROS, while it is held | one stop that works in every position |
| 5 | The steering stick still steers during the stop and the hold | it is the same stick: a person may swerve while braking |
| 6 | The middle has no brake, even with the car stopped | Arsh, 21 Nov: throttle off only. On a slope, hold the right stick down |
| 7 | Giving doesn't need the car stopped. If the computer isn't armed, nobody drives (the idle brake) | the person decides when |
| 8 | The left stick never takes the car from the computer | only the right stick does |
| 9 | A left stick moved and put back inside the 1 s still counts as moved | the person moved it on purpose |

## How it was tested

`cargo test` in `firmware/spine`, `cargo clippy`, `cargo fmt`; the QEMU check's offline
tests. Not run: `pixi run esp qemu-check` (the QEMU here is a Mac build), and nothing on the car.

## Trying it without the car (23 Nov)

A PS4 gamepad plays the FS-i6X, and these rules run unchanged in `spine_sim`
(`aav_spine_sim`'s `rc_pad`): the sticks become the receiver's channels, which go through
the ibus crate's own parser and `RcMap::TODAY`, as on the ESP32.

| Command | What runs |
|---|---|
| `pixi run rc-pad` | the gamepad, `rc_pad`, `spine_sim`'s model car, rviz: nothing else, so it needs only `aav_spine_sim` built. The mode switch starts high (manual); `mode_start:=low` for normal |
| `pixi run sim spine:=true rc_pad:=true [rc_pad_mode:=low]` | the same in Ravensim, with the whole stack: in normal, give hands the car to the computer |

| The gamepad | The FS-i6X |
|---|---|
| left stick up and down | the left stick: the pedal (CH3) |
| right stick up and down; left and right | the right stick: stop and give (CH2); steering (CH4) |
| R1, each press | the mode switch (CH6), high and low |
| L1, held | the radio lost (CH6 at 1000, the failsafe) |

The gamepad's left stick springs back, so its middle is where it rests; the FS-i6X's stays
where it's left. When measured, the gamepad's right stick drifted up to 14 % sideways when
pushed straight up, just inside give's 15 % window: a crooked push up may not give, and
the FS-i6X may do the same.

Not tested this way: the radio, the real sticks' travel, the ESP32's timing, the car.
