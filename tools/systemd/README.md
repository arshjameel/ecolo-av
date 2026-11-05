# tools/systemd

What starts our stack when a car computer boots. The runbook: `docs/car-setup.md`.

## Overview

`tools/car_setup.sh <role> services` copies these into the car user's `~/.config`, turns
on linger (so user units start at boot with nobody logged in), and enables the unit.

```
boot ─> systemd --user (linger) ─> aav-car.service
          reads ~/.config/aav/car.env ─> ~/aav-jazzy/aav.sh ros2 launch aav_bringup
                                         car.launch.py machine:=$AAV_MACHINE $AAV_CAR_ARGS
```

The dash's two screens have their own units: [screens/](../screens/README.md).

## Interface

| File | Installed to | Is |
|---|---|---|
| `aav-car.service` | `~/.config/systemd/user/` | systemd user unit: `car.launch.py` for this computer |
| `car.env` | `~/.config/aav/car.env` | its settings; `@MACHINE@` filled in by `car_setup.sh` |

`car.env` keys:

| Key | Means | Default |
|---|---|---|
| `AAV_MACHINE` | the machine file, `ros/bringup/aav_bringup/config/machines/<name>.yaml` | the role's: `orin` or `motor_pi`; `motor_pi_alone` for the first moves (`--machine`) |
| `AAV_CAR_ARGS` | `car.launch.py`'s arguments, space separated | `max_speed:=0.3` |

Which role gets which unit:

| Role | `aav-car` | Other |
|---|---|---|
| `orin` | `machine:=orin` | chrony serves the car's time |
| `motor_pi` | `machine:=motor_pi` (or `motor_pi_alone`) | refused while the 2025-26 `aav_serial_bridge` is enabled |
| `localization` | none yet: no machine file (the IMU and GPS: `orin.yaml`'s TODO) | chrony, udev |
| `dash` | none: the screens' units | chrony |
| `camera_pi` | none: no ROS | MediaMTX enabled, chrony |
| `lab_desktop` | none | none |

## Invariants

- User units: they run as the car's user, need no sudo once installed. Linger needs
  sudo once (`loginctl enable-linger`).
- `KillSignal=SIGINT`: a stop is a Ctrl-C, so the legacy bridge sends its last stop frame.
- `Restart=on-failure` after 5 s, no start limit; a clean `systemctl --user stop` holds.
- `car.env` is systemd's `EnvironmentFile`: `KEY=value`, no `$VARIABLES`, no `~`.
- Never two drive paths: `car_setup.sh` won't enable it on the Motor Pi while theirs is
  enabled, and `motor_pi_alone` never runs beside `car orin`.

## Side Effects

Once enabled: our stack for this computer starts at every boot and restarts after a
crash; its log is in the journal (`journalctl --user -u aav-car -f`). On the Motor Pi
that opens the ESP32's serial port.

## Verification

```sh
python3 -m pytest -q tools/tests/test_car_setup.py   # also in tools/test.sh
bash tools/car_setup.sh orin services                 # the plan; writes nothing
```

On the car: `systemctl --user status aav-car`, then `bash car_setup.sh <role> check`.
