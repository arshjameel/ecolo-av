# How the car knows how fast it's going

Ben's question (29 Oct): once localization works, can we speed up and slow down, read
the sensors for acceleration and braking, **and** judge the speed from localization?

**Yes.** The car has several ways to tell its speed. Each one fails in its own way, so we
use them together. This page shows each one, then a braking example with numbers.

**You'll learn:**
- the four speed sources: wheels, IMU, GPS, lidar;
- how each one goes wrong;
- what the EKF does with them today;
- one stop from 1 m/s, as each sensor sees it;
- where we use the speed: measuring the car, the speed loop, the preflight on stands.

The filter behind all this is in [Learn: localization](learn-localization.md). The loop
that uses the speed is in [Learn: control](learn-control.md).

## 1. Three words: speed, acceleration, braking

| Word | Means | Unit | Example |
|---|---|---|---|
| **Speed** | how far you go each second | m/s | 1 m/s = a brisk walk |
| **Acceleration** | how fast the speed changes | m/s² | +1 m/s²: after 1 s you go 1 m/s faster |
| **Braking** (deceleration) | acceleration backwards | m/s² | −1 m/s²: from 1 m/s to 0 in 1 s |

Two rules link them:

```text
  speed  = position changing      (so: position ÷ time ──► speed)
  accel  = speed changing         (so: add up accel over time ──► speed)
```

Each sensor gives **one** of the three. To get the others you divide (noisy) or add up
(drifts). That's the whole story of this page.

## 2. The wheels: counting pulses

A sensor on each front wheel clicks once per hole (or spoke) as the wheel turns.

```text
   ●  ●  ●            one click per hole
 ●   wheel  ●   ──►   time between clicks ──► speed
   ●  ●  ●
```

On our car: the hardware lead's Arduino reads both front wheels and sends
`WHEEL_SPEED_L` and `WHEEL_SPEED_R` on CAN (0x200, 0x201, 100 Hz, `firmware/can/aav.dbc`).
`spine_bridge` turns them into `/vehicle/odom`.

**In numbers** (`vehicle.yaml`; both are guesses until someone measures):

| Setting | Value |
|---|---|
| `wheel_diameter` | 0.30 m, so one turn = 0.94 m |
| `encoder_counts_per_rev` | 8, so one click every 0.12 m |

| Speed | Clicks a second | Time between clicks |
|---|---|---|
| 1.0 m/s | 8.5 | 0.12 s |
| 0.2 m/s | 1.7 | 0.59 s |

So at walking pace it's good. Near a stop, the clicks come too slowly to tell you much.

| How it goes wrong | What you see |
|---|---|
| **Slip**: the wheel spins on gravel or wet paint | the wheels say faster than the car |
| **Lock**: hard braking stops the wheel; the car slides | the wheels say **0** while the car still moves |
| **Wrong size**: the diameter is 1 % off | every speed 1 % off, forever |
| **No direction**: one Hall sensor can't tell forward from back | the sign comes from the drive command (`direction_known` = 0) |
| **On stands**: the wheels spin, the car doesn't move | the wheels say 1 m/s, the car is still (section 9) |

Still the best everyday speed: direct, fast, no drift.

## 3. The IMU: feeling the push

The IMU's **accelerometer** feels acceleration, like you feel pushed into your seat.
Ours is a Yahboom on USB, 200 Hz ([Learn: sensors](learn-sensors.md)).

| Good for | Why |
|---|---|
| **"How hard are we braking right now?"** | it measures that directly, 200 times a second |
| Spotting a wheel that lies | a locked wheel says 0; the IMU still feels the car slowing |

| How it goes wrong | In numbers |
|---|---|
| **Adding up drifts**: a small bias grows | a bias of 0.05 m/s², added up for 10 s = **0.5 m/s** wrong |
| **Gravity on a slope** looks like a push | a 1° slope = 9.81 × sin 1° = **0.17 m/s²** |
| **Bumps and vibration** | noisy: average it over a few tenths of a second |

So: trust the IMU for **changes** in speed over a second or two, never for the speed itself.

## 4. GPS: two ways to a speed

### From positions (noisy)

Speed = how far the fix moved ÷ the time. But each fix is off by 1-3 m on its own
([Learn: GPS](learn-gps.md)).

```text
 true path:   ●────────●          1 m in 1 s
 GPS says:      ✕          ✕      each fix off by up to 1 m
                └──────────┘      looks like 0 to 3 m/s
```

At 1 fix a second (our receiver, from the factory), that speed is useless second by
second. Over 10 s it gets good: the watchdog fits a straight line through 10 s of fixes
(`wheel_check.py`).

### From Doppler (clean)

The receiver also measures its own velocity from the Doppler shift of the satellites'
signals: good to about **0.05 m/s**. Our driver (`ublox_gps`) publishes it on
`/sensors/gps/fix_velocity` (`docs/interfaces.yaml`).

| Today, that velocity is used for | Not used for |
|---|---|
| the **heading** (`gps_heading`, into `ekf_global`) | the speed: no EKF reads it as a speed |

!!! warning "Never seen from our receiver"
    `ublox_gps` has never run with the car's ZED-F9R. Gazebo makes a stand-in velocity.

## 5. Lidar localization: planned

Planned (research pick 2, [research picks](designs/2026-10-28_research-picks.md)).
NDT matches each lidar scan against a saved map of the lot, to a few centimetres
([Learn: localization §8](learn-localization.md#8-next-matching-the-lidar-to-a-map-ndt-and-icp)).

| | GPS | Lidar on a map |
|---|---|---|
| Position error | 1-3 m | a few cm |
| Speed from 2 positions 0.1 s apart | useless | 0.03 m ÷ 0.1 s ≈ 0.3 m/s at worst, better averaged |
| Fails when | under trees, beside buildings | a bare lot with nothing to match; no map yet |

It doesn't slip and doesn't drift. It's a separate check on the wheels.

From the lidar's points to that map: [Learn: lidar](learn-lidar.md).

## 6. The EKF: what it reads today

From `ros/localization/aav_localization/config/` (more in
[Learn: localization §7](learn-localization.md#7-our-two-ekfs-their-real-inputs)):

| Input | Gives | `ekf_local` | `ekf_global` |
|---|---|---|---|
| `/vehicle/odom` (wheels) | **forward speed**, turn rate | speed, turn | speed, turn |
| `/sensors/imu/data` | turn rate | turn | turn |
| `/localization/gps/odom` | position | - | **x, y** |
| `/localization/gps/heading` | heading | - | **yaw** |
| IMU acceleration | - | **not fused** | **not fused** |
| GPS Doppler as a speed | - | not fused | not fused |
| NDT pose | - | - | planned |

**So the EKF's speed today is the wheels' speed.** The GPS only pulls the position back
slowly. If the wheels lie, the EKF's speed lies too.

The **watchdog** (`aav_safety`, `wheel_check.py`) is what catches a lying wheel today:

| Check | Compares | Trips when |
|---|---|---|
| speed | the wheels' change in speed vs the IMU's push, over 1.5 s | off by 0.3 m/s for 1 s |
| gps | the wheels' speed vs a line through 10 s of GPS | off by 0.15 m/s |

Then it says DISAGREE and stops Nav2 (never a person or a test script).

## 7. Worked example: stopping from 1 m/s

The car drives at 1.0 m/s and brakes gently at 1.0 m/s². It stops after **1 s**,
in **0.5 m** (distance = speed² ÷ (2 × braking) = 1 ÷ 2).

| Time | Truth | Wheels | IMU | GPS (positions) | GPS Doppler | Lidar + NDT (planned) |
|---|---|---|---|---|---|---|
| 0.0 s | 1.0 m/s | 1.0 | −1.0 m/s² | "somewhere 0-3 m/s" | 1.00 ± 0.05 | 1.0 ± 0.1 |
| 0.5 s | 0.5 m/s | 0.5 (a click every 0.24 s) | −1.0 m/s² | no new fix yet | no new fix yet | 0.5 ± 0.1 |
| 1.0 s | 0 | 0.1-0.2: still waiting for a click | 0 | one fix: no use | 0.00 ± 0.05 | 0.0 ± 0.1 |
| 1.5 s | 0 | 0, after the stale timeout | 0 | | | 0.0 |

What to take from it:

| Question | Best answer from |
|---|---|
| How hard did we brake? | the **IMU**: −1.0 m/s², at once |
| How fast at each moment? | the **wheels**, until the last few tenths |
| When did we really stop? | the IMU (the push ends) and later the lidar |
| Where did we stop? | lidar + NDT (planned); today the EKF, ± GPS's metre |

### The same stop, wheels locked

Now a hard stop on gravel: the front wheels lock at 0.4 s, while the car still does 0.6 m/s.

| Sensor | Says |
|---|---|
| Wheels | **0**, at 0.4 s. Wrong: the car slides on |
| IMU | still −1.x m/s² until the car really stops |
| EKF today | believes the wheels: "stopped" 0.6 s too early, about 0.2 m short |
| Watchdog | a gap for about 0.6 s, under its 1 s: it says nothing |

That's why the control plan wants **the IMU's acceleration to help while braking**.

## 8. Where we use the speed

| Use | Reads | Page |
|---|---|---|
| **Measuring the car**: `max_accel`, `max_brake`, `drag`, lags into `vehicle.yaml` | wheels for speed; the IMU for the g figures and the brake fit | [Actuator tests](actuator-tests.md), [Measure the car](measure-the-car.md) |
| **Scoring a run**: rise, overshoot, hold, stop error | Gazebo's truth, else the wheels, else `ekf_global`; the stop from the EKF or a tape | `tools/atob_score.py`, [A to B](a-to-b.md) |
| **The speed loop's feedback**: feed-forward + PI | wheel speed; the IMU to help while braking | [the control plan](designs/2026-10-27_control-plan.md), [Learn: control](learn-control.md) |
| **Catching a lying sensor** | wheels vs IMU vs GPS | `aav_safety` `wheel_check.py` |

`vehicle.yaml`'s `max_accel` (1.5 m/s²) and `max_brake` (3.0 m/s²) are **guesses** until
tests 4.1 and 4.3 run in a lot.

## 9. On stands: disagreeing on purpose

The preflight on stands (`pixi run preflight`, 29 Oct; [Actuator tests](actuator-tests.md#the-preflight-on-stands))
lifts the car, so the wheels spin in the air. On stands, `actuator-test` caps the speed
at 0.3 m/s.

!!! warning "Today nothing measures the spinning wheels"
    The motor drives the **rear** wheels; the sensors are on the **front** ones. On
    stands the front wheels stand still, so `/vehicle/odom` says 0 (TODO 49b). The table
    below needs a rear-wheel speed; until then the preflight only checks the sensors
    (`--no-drive`).

```text
   wheels:  0.3 m/s  ──►  "we're moving"
   IMU:     0         ──►  "nothing pushed me"
   GPS:     still     ──►  "same place"
```

| Who | What happens |
|---|---|
| The wheels | read the motor's speed: exactly what we want to test |
| `ekf_local` | adds up the wheel speed: it thinks the car drove away. Ignore it on stands |
| The watchdog | expect **DISAGREE**, latched: 0.3 m/s of wheels against a still GPS is over its 0.15 m/s limit. It holds only Nav2 and friends, not `actuator-test`. Clear it after: `pixi run watchdog-clear` |
| The preflight's check | wheels follow the command; the IMU's speed change stays under 0.4 x the wheels', the GPS's Doppler under 0.1 m/s: that's a **pass**. While it runs, an IMU that says the car moves stops it |

Off the stands, the same disagreement means a fault. On the stands, it means the wheels
work and nothing else is fooled into moving.

## 10. Today vs planned

| Piece | Today | Planned |
|---|---|---|
| Wheel speed on CAN → `/vehicle/odom` | ✅ built; bench-tested; never seen the Arduino's frames | the rotor's holes counted, the diameter measured |
| IMU acceleration | read by the watchdog and actuator tests; **not** in the EKF | helps the speed loop while braking |
| GPS positions | in `ekf_global` (x, y); the watchdog's 10 s check | |
| GPS Doppler velocity | the heading only | maybe a speed input too: an idea, not planned |
| Lidar + NDT | ❌ | research pick 2: a map with KISS-ICP, then NDT into `ekf_global` |
| Speed loop on wheel speed | in `spine-core` and Gazebo; today's ESP32 is open loop | the control plan, steps 1 and 3 |
| Preflight on stands | `actuator-test --stands` (steering only) | TODO 29 Oct: speed up, hold, brake, every sensor live |

## Where this lives in the code

| What | Where |
|---|---|
| Wheel frames | `firmware/can/aav.dbc` (0x200, 0x201) |
| Wheel frames → `/vehicle/odom` | `ros/can_bridge/aav_spine` `spine_bridge.py`, `aav_spine_bridge` (Rust) |
| Wheel size, the guesses | `ros/bringup/aav_description/config/vehicle.yaml` |
| The EKFs | `ros/localization/aav_localization/config/ekf_local.yaml`, `ekf_global.yaml` |
| GPS driver settings | `ros/sources/aav_sensors/config/ublox_gps.yaml` |
| Wheels vs IMU vs GPS | `ros/safety/aav_safety/aav_safety/wheel_check.py` |
| The speed loop | `firmware/spine/crates/spine-core/src/speed_loop.rs` |
| Scoring a run | `tools/atob_score.py` |
