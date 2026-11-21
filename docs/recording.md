# Recording a mapping drive

One hand-driven drive that records **everything** the later phases need: the NDT map and
SLAM, AMCL and a learned localizer, labelling, the transfer functions. Phase C of the
[testing plan](testing-plan.md) (TODO 249).

```sh
pixi run record-drive --check                  # only the checks
pixi run record-drive --name lot --note "the lot, clockwise, dry"
pixi run record-drive --summary bags/<folder>  # the summary again
pixi run record-drive --sim                    # the kit's own test, in Ravensim
pixi run record-drive --sim --loop             # the whole route, back home (for SLAM)
pixi run record-drive --sim --loop --sim-box 22 -6.5   # ... with a parked car (a box)
                                               # for this drive only: map-area's test data
```

`tools/record.sh --mapping` is the same command. The short `tools/record.sh` (cameras as
JPEG, what the stack decided) stays for everyday drives.

!!! warning "Not yet run on the car"
    Built and tested in Ravensim (19 Nov). The car's clocks, its disk and its real rates
    are untested.

## What it does

```
  checks ──▶ asks "record anyway?" ──▶ ros2 bag record ──▶ Ctrl-C ──▶ summary
 disk, rates,     (only on warnings)      MCAP + zstd                 per topic, and
 clocks, frames                           2 GB / 5 min files          "good for ..."
```

## What it records

| Group | Topics | Rate on the car |
|---|---|---|
| Lidar | `/sensors/lidar/points` and its raw `packets` | 10 Hz |
| IMU | `/sensors/imu/data_raw`, `data`, `mag` | 200 Hz |
| GPS | `/sensors/gps/*` (fix, velocity), the u-blox driver's own (`/ublox_gps/*`) | 1 Hz |
| Cameras | all three as JPEG (`image_raw/compressed`) and `camera_info` | 15 Hz |
| Radar | `/sensors/radar/points` | 10 Hz |
| The car | `/vehicle/wheels`, `odom`, `steering`, `throttle`, `brake`, `status`, `/can/rx` | 50-100 Hz |
| Commands | `/ego/actuator_cmd`, `/control/cmd_vel`, each driver's, **who is driving** (`/control/command_mux/state`) | 10-50 Hz |
| Safety | the watchdog's state and lock, the e-stop, the output mux, the geofence | 2-10 Hz |
| Position | the EKFs (`/localization/odom/local`, `global`), NDT | 30 Hz |
| Frames | `/tf`, `/tf_static`, `/robot_description` | |

The full list, with sizes: `STREAMS` in `tools/recording.py`.

A camera with pictures but no JPEG topic gets a JPEG copy for the drive
(image_transport's `republish`). `--raw-cameras` keeps raw pictures: huge.

## How big

| Group | GB an hour (before zstd) |
|---|---|
| Cameras (3 × JPEG, ~300 kB) | 49 |
| Lidar (points 33 + packets 3) | 37 |
| Everything else | about 1.5 |
| **Total** | **about 87** |

zstd hardly shrinks JPEGs. In Ravensim it shrank the lidar to a quarter; a real lidar's
noise compresses less. So plan for **up to 87 GB an hour**: a fast SSD on the Orin. The
camera size is a guess (docs/bandwidth.md): the check prints the measured rate too.

## The checks before

| Check | Refuses | Warns |
|---|---|---|
| Disk | under 10 min left | under 1 hour left |
| Topics | none at all (wrong domain?) | one missing, or under 80 % of its rate |
| Stamps | | a sensor's stamps from the future, or over 0.5 s old (a clock off) |
| Clocks | | `chronyc tracking` over 10 ms, here and on each car computer by ssh |
| Frames | | `/tf_static` lacks a sensor's frame |

Warnings ask "Record anyway?". `--yes` skips the question; `--force` records past a
refusal. On a Mac there is no `chronyc`: the line says "not checked".

The ssh to the other computers uses keys only (`BatchMode`): it never asks for a password.
A computer it can't reach gets a WARN line, not a stop. `--no-ssh` skips them.

## The folder

`bags/<date>_<name>/`:

| File | What |
|---|---|
| `bag/` | the recording: MCAP, zstd, a new file every 2 GB or 5 min |
| `drive.yaml` | when, the git commit, the computer, your note, every check, the clocks; after: the verdicts |
| `vehicle.yaml`, `machines/` | the car's measurements and machine files, as they were |
| `summary.md` | per topic: count, rate, gaps, size; good for which job |
| `record.log` | the recorder's own output |

## The summary after

Per topic: how many, the rate, gaps over 3 periods, the size, and flags (dropped,
gaps, none). Then one line per job:

| Job | Needs |
|---|---|
| NDT map | the lidar, a pose (`/localization/odom/global`), `lidar_link` in `/tf_static` |
| SLAM | the lidar, the raw IMU, the wheels or odometry |
| Labelling | the front camera and its lens, the lidar, the frames |
| Transfer functions | ego's command, steering, throttle, brake, wheels, the raw IMU |

Each says **good**, **weak** (there, with gaps) or **no** (missing).

## Then the map

```sh
pixi run ndt-map --out maps/ndt/<place> build bags/<folder>/bag --poses /localization/odom/global
```

`--sim` does all of it: Ravensim's test lot, two legs, the summary, then this map.

## Ravensim's run, 19 Nov

`pixi run record-drive --sim` on an M4 Max: the test lot, two legs, both reached.

| | |
|---|---|
| Checks | all OK but `/vehicle/battery` (Ravensim has none); the three cameras republished as JPEG |
| Recorded | 102 s, 1.0 GB of messages, **245 MB on disk** (zstd: 24 %) |
| Lost | none (the recorder's own count); no gaps on any topic |
| Good for | NDT map **good**, SLAM **good**, labelling **good**, transfer functions **good** |

Some rows of its summary:

| Topic | Count | Rate (Hz) | Size |
|---|---|---|---|
| `/sensors/lidar/points` | 1013 | 10.0 | 934 MB |
| `/sensors/camera/front_left/image_raw/compressed` | 1015 | 10.0 | 17 MB |
| `/sensors/imu/data_raw` | 5068 | 49.9 | 1.6 MB |
| `/vehicle/wheels` | 10135 | 99.8 | 0.5 MB |
| `/ego/actuator_cmd` | 5076 | 50.0 | 0.2 MB |
| `/control/command_mux/state` | 1016 | 10.0 | 0.2 MB |

**The NDT map from it** (1012 clouds, 8 s), against the same clouds placed by Ravensim's
truth:

| Poses | Points | Off the truth's map: median | 95 % |
|---|---|---|---|
| Ravensim's truth | 77,000 | - | - |
| `/localization/odom/local` (wheels + IMU) | 132,000 | 16 cm | 60 cm |
| `/localization/odom/global` (with the GPS) | 463,000 | 33 cm | 1.2 m |

The recording is usable, but **a map from the EKF's poses is blurred**: each wall is drawn
several times (6 times the points). That is why phase D makes the map with SLAM
(KISS-SLAM), not from the EKF: [Maps from drives](maps-from-drives.md) (`pixi run
map-drive`: 2.3 cm off).

The first run found two faults, both fixed: 60 ms of jitter at 50 Hz counted as a gap (now
a gap must also be over 0.1 s), and Ravensim's cameras, still being built, came up after the
check (no JPEG copies, so labelling read "no").
