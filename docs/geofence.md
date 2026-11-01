# The geofence

An area the computer's drivers may not leave. It works **without Nav2**: it stops the
point driver, a script, a replay and the language server the same way it stops Nav2.

People are never held. The safety driver on the joystick can always drive.

## In one picture

```
            the fence (a 30 m circle round the start spot, or the lot's polygon)
     ┌───────────────────────────────────────────────────────────┐
     │                                               margin + σ   │
     │     start ●                                          ┆     │
     │                 car ▶▶▶▶ stopping path ▶▶▶▶ ✋ NEAR  ┆     │
     │                                                      ┆     │
     └───────────────────────────────────────────────────────────┘
```

The lock closes when any **stopping path** of the car brings a corner within the
**margin** (1 m) plus the pose's **spread** (σ) of the edge.

## Who it stops

| Driver | Priority | Held by the geofence (51)? |
|---|---|---|
| joystick, RC, keyboard | 100, 95, 90 | **never** |
| actuator tests | 60 | no: they **stop themselves** on its state |
| scripted, replay, point, Nav2, AI | 50 … 10 | **yes** |

## The stopping path

Each tick it works out how far the car could go before it stands still (RSS):

| Part | Value today | From |
|---|---|---|
| reaction ρ | 0.9 s on the spine | the geofence's tick 0.1 + ego's `cmd_timeout` 0.3 + the backend's timeout 0.2 + `brake_lag` 0.2 + pose lag 0.1 |
| braking b | 1.0 m/s² | half of the softer of ego's 2.0 and `max_brake` 3.0 |
| speeding up | 1.0 m/s² for ρ | ego's `max_acceleration`, up to its `max_speed` |

```
from 1 m/s (at the cap): 1.0 × 0.9 + 1.0² / (2 × 1.0) = 1.4 m
```

It checks that path **forward and back**, **straight and at full lock** either way.
No car number is copied: the launch reads `ego.yaml`, `vehicle.yaml` and the backend's
config.

So a 30 m circle leaves about **26 m** usable round the start: 30 m, less the stopping
path, the margin and σ.

## The states

| State | Lock | Means | Lets go |
|---|---|---|---|
| `OK` | open | inside, with room to stop | |
| `STARTING` | **on** | under 3 s of good poses | by itself |
| `NEAR` | **on** | a stopping path reaches the margin | when 0.5 m further in |
| `LOST` | **on** | no pose for 1 s; σ over 3 m; NaN; **no GPS into the EKF for 2 s** | by itself, once good |
| `OUTSIDE` | **on** | the car itself is outside | a person drives in, then `pixi run geofence-clear` |
| `JUMP` | **on** | the pose jumped further than speed × time + 2 m | a person: `pixi run geofence-clear` |
| `COASTING` | **on** | the backend can't brake to a stop (below) | when test 4.3 measures it |
| `STANDS` | open | stands mode: **no fence** (logged every 5 s) | |

At a standstill near the edge, NEAR holds: a person drives the car back in.

## On the car

| Setting | Default | Where |
|---|---|---|
| the fence | a 30 m circle round `start_x`, `start_y` | `car.launch.py geofence_radius:=` |
| a site's polygon instead | none yet | `geofence_site:=`, or the machine file's `geofence: site:` |
| stands mode | off | `car.launch.py stands:=true`; needed with `gps:=false` |

**Mark the start spot.** The circle's centre is the start pose you give car.launch.py.

**Today's car coasts.** `legacy_uart_bridge` in `speed_loop.mode: open` brakes only
below `stop_speed`, so from 1 m/s the car rolls about 10 m. The geofence keeps every
program held (`COASTING`) until test 4.3 measures the stop and someone writes
`actuators.open_loop_decel` into `vehicle.yaml`. (The no-timeout lock holds them on
today's firmware anyway.)

### A site's polygon (a TODO for the team)

1. Survey the datum (the 26 Oct review's W3) and set it in `navsat.yaml`.
2. Drive or walk the lot's corners; read each one's map x, y.
3. Add `sites.<name>: {datum: [lat, lon], corners: [x0, y0, ...]}` to
   `ros/safety/aav_safety/config/geofence.yaml`.
4. Put `site: <name>` in `orin.yaml`'s `geofence:`.

A site drawn on another datum is refused at start-up.

## Other parts that know the fence

| Part | Does |
|---|---|
| the point driver | refuses a B outside the fence |
| the drive server (`aav_language`) | refuses a drive, turn or GPS goal that ends outside; never clears it |
| the actuator tests | stop on NEAR, OUTSIDE, JUMP or LOST on the ground; refuse STANDS there |
| RViz, Lichtblick | draw `/safety/geofence/fence` |

## What we measured (bench, 29 Oct)

`pixi run bench-check --geofence`, on the spine's model car, a 12 m fence:

| Check | Result |
|---|---|
| a scripted drive at 1 m/s | the lock closed at 1.08 m/s; the car stopped **1.06 m** on (the geofence counts on 1.56 m) |
| where it stopped | its furthest corner 9.3 m from the start: inside the 12 m fence, in NEAR |
| a person backs it in (keyboard) | not held: 3.4 m back, then OK |

Not yet run: Gazebo with a fence you can reach, and the real car.

## Try it

```sh
pixi run bench-check --geofence     # a scripted drive at a 12 m fence must stop inside it
pixi run sim world:=test_lot geofence_radius:=15   # watch /safety/geofence/state
pixi run geofence-clear             # after OUTSIDE or JUMP, once the car is back in
```

Design: `docs/designs/2026-10-29_geofence.md`.
