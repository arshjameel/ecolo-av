# aav_teleop

Human drivers: the joystick, its e-stop button, and the keyboard. Stock ROS packages,
set up by our config files, plus `aav_safety`'s `joy_alive` between the joystick and
teleop_twist_joy: no code of its own. A person is a driver like any
other: they publish a velocity command into twist_mux and never talk to the wheels.

```
joystick -> joy_node -> /drivers/joy/joy -> joy_alive -> /drivers/joy/joy_checked -> teleop_twist_joy -> /drivers/joy/cmd_vel -> twist_mux
                                         -> joy_teleop (one button) -> /safety/estop true
keyboard -> teleop_twist_keyboard -> /drivers/keyboard/cmd_vel -> twist_mux
```

## Overview

It replaces the 2025-26 team's `joy_teleop_ackerman`, which drove the wheels directly.
The bringup profiles (`aav_bringup`: turtle, sim, bench, car, carla) start
`teleop.launch.py`. Run it on the computer the joystick's USB receiver is plugged into
(any computer on the same `ROS_DOMAIN_ID`).

## Interface

| Item | What |
|---|---|
| Launch | `ros2 launch aav_teleop teleop.launch.py [estop_button:=true] [joy_device:=true] [use_sim_time:=false]`; `joy_device:=false` leaves joy_node out, for a script that plays the pad (bench `teleop:=stand_in`) |
| Keyboard | `pixi run keyboard` (`tools/keyboard.sh`: teleop_twist_keyboard, `stamped:=true`, namespace `/drivers/keyboard`) |

Nodes (all in the namespace `/drivers/joy`, so relative topics land there):

| Node (package) | Reads | Writes |
|---|---|---|
| `joy_node` (joy) | the USB joystick | `/drivers/joy/joy` (`sensor_msgs/Joy`), 20 Hz while held |
| `joy_alive` (aav_safety) | `/drivers/joy/joy` | `/drivers/joy/joy_checked` (each message, or a released copy while stopped); `/drivers/joy/alive` (`std_msgs/String`, 10 Hz: `OK`, `WAITING`, `SILENT`, `FROZEN`, and why) |
| `teleop_twist_joy_node` (teleop_twist_joy) | `/drivers/joy/joy_checked` | `/drivers/joy/cmd_vel` (`geometry_msgs/TwistStamped`, `base_link`) |
| `joy_teleop` (joy_teleop), if `estop_button` | `/drivers/joy/joy` | `/safety/estop` (`std_msgs/Bool`) true, once per press |

Config files (each names its node `/**/<node>`, so it matches in any namespace):

| File | Sets |
|---|---|
| `config/joy.yaml` | device 0, deadzone 0.05, autorepeat 20 Hz |
| `config/teleop_twist_joy.yaml` | deadman button 0 (trigger); axis 1 = speed, 1.0 m/s at full stick; axis 0 = turn rate, 0.35 rad/s at full stick; `inverted_reverse` |
| `config/estop_button.yaml` | button 2 publishes `/safety/estop` true |
| `config/joy_alive.yaml` | deadman button 0, motion axis 1 (as teleop_twist_joy), `silent_after` 0.5 s, `frozen_after` 2.0 s (0 off) |

twist_mux priorities: joystick 100, keyboard 90 (`aav_command_mux`).

## Invariants

- The joystick sends only while the deadman (button 0) is held, then one zero on release.
- **The joystick-alive check** (TODO 74a): joy_node repeats its last state while the
  receiver is plugged in, so a pad turned off could look like a held trigger. `joy_alive`
  stops the joystick when nothing comes for 0.5 s (`SILENT`) or, while the deadman is held
  and the throttle off centre, every value stays bit-identical for 2 s (`FROZEN`). It
  latches until the deadman is released; at start-up nothing drives until it has been
  released once (`WAITING`). A stick pinned at its edge trips `FROZEN` too: release, press.
  The rules: `aav_safety` `joy_alive.py`. `tools/tests/test_joy_alive_config.py` checks
  that its deadman and axis match `teleop_twist_joy.yaml`.
- No stock option does this (joy 3.3.0, teleop_twist_joy 2.6.5, read 2 Nov):
  | Option | Why not |
  |---|---|
  | joy `autorepeat_rate: 0` | joy then sends only changes: a dead pad goes quiet, but so does a still hand, and teleop_twist_joy publishes only on a Joy message, so twist_mux's 0.5 s timeout would stop a person holding the stick still |
  | joy `sticky_buttons` | only makes buttons toggle |
  | teleop_twist_joy | no timeout of its own: it publishes on each Joy message |
  | joy on unplug | closes the device (SDL's DEVICEREMOVED) and stops publishing: `SILENT` catches it in 0.5 s |
- The stick sets a turn **rate**, not a steering angle; ego converts it with the bicycle
  model. Full stick at 1.0 m/s is about full lock (`0.35 = 1.0 x tan(0.5) / 1.57`). The
  car can't steer standing still.
- Stick back = reverse while the trigger is held; ego caps it at `max_reverse_speed`.
- The e-stop button only ever sets true. There is no release button: joy_teleop 2.0.0
  reads an empty Joy message as "all pressed", and a spurious release is unsafe. Clear
  it from a laptop: `ros2 topic pub --once /safety/estop std_msgs/msg/Bool "{data: false}"`.
- Commands are `TwistStamped` (`publish_stamped_twist: true`), never Ackermann.
- TODO: check the button and axis numbers on the team's joystick
  (`ros2 run joy joy_enumerate_devices`, then `ros2 topic echo /drivers/joy/joy`).

## Side Effects

- Reads a USB joystick (`joy_node`).
- Publishes `/drivers/joy/cmd_vel` and `/safety/estop`; the keyboard publishes
  `/drivers/keyboard/cmd_vel`.

## Verification

No tests of its own (only config and a launch file). `joy_alive`'s logic is tested in
`aav_safety`; `pixi run bench-check` plays a live, a frozen and a silent pad through this
launch file (bench `teleop:=stand_in`) and checks that the car stops.

```sh
pixi run build && ros2 launch aav_teleop teleop.launch.py
ros2 topic echo /drivers/joy/cmd_vel     # hold the trigger, move the stick
```

`pixi run check <turtle|sim|car>` checks the running graph against `docs/interfaces.yaml`.
`pixi run test` runs every package's tests.
