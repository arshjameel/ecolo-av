# aav_localization_health

## Overview

Warn-only checks that localization's inputs and outputs are believable (Betsi, 1 Nov).
It never stops the car and never changes what the EKFs see: the sensor watchdog
(`aav_safety`) does the stopping. These are the checks the watchdog doesn't do.

| Check | WARN when |
|---|---|
| `gyro_bias` | parked, the gyro's mean turn rate stands out (0.2 deg/s = 12 deg a minute without GPS) |
| `imu_mounting` | parked, gravity says the IMU is upside down, tilted, or in the wrong units |
| `gps_quality` | no fix, an unknown covariance, or a sigma too big or too small to believe |
| `timestamps` | an input's stamp is too old, or from the future (a computer's clock is off) |
| `frames` | an input's `frame_id` isn't the one the EKFs expect |
| `tf` | `base_link -> imu_link / gps_link` missing, or `map -> base_link` old |
| `wheels` | the two front wheels disagree with the turn the gyro measures |
| `motion` | an EKF says a speed, turn or acceleration the car can't do; `ekf_local` jumps |

`localization.launch.py` starts it (`health:=true` by default; `health:=false` leaves it out).

## Interface

### Node `localization_health`

| In / out | Topic | Type |
|---|---|---|
| in | `/sensors/imu/data`, `/vehicle/odom`, `/vehicle/wheels`, `/sensors/gps/fix` | Imu, Odometry, WheelSpeeds, NavSatFix |
| in | `/localization/odom/local`, `/localization/odom/global`, `/tf`, `/tf_static` | Odometry, TF |
| out | `/localization/health` | `std_msgs/String`: OK, or WARN and every check that isn't OK |
| out | `/diagnostics` | `diagnostic_msgs/DiagnosticArray`: one status per check |

Parameters: `config/localization_health.yaml` (every threshold, with its reason); the
wheelbase and track come from `vehicle.yaml`.

### Modules

| Module | No ROS? | Holds |
|---|---|---|
| `health.py` | yes | `Settings`, `Health` (fed message by message; `report()`), `summary()` |
| `localization_health.py` | no | the node |

## Invariants

- Warn only: nothing reads `/localization/health` to drive.
- A required input (IMU, wheel odometry, `ekf_local`) gone quiet makes its checks STALE;
  an optional one that never came (no GPS on the bench) says "not on this profile".
- `health.py` has no ROS imports, and its own tests.

## Side Effects

- Publishes `/localization/health` and `/diagnostics`; logs each change of state once.

## Verification

```sh
cd ros/localization/aav_localization_health && python3 -m pytest -q   # the checks, no ROS
```

`tools/test.sh` runs it; `pixi run test` runs everything. Thresholds were tuned in Gazebo
only: on the car, read `/diagnostics` after a parked spell and a drive.
