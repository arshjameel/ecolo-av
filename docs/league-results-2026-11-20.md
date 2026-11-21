# The league, scored: 20 Nov (stage 2)

Ben, 20 Nov: "Please work on phase 2". Stage 2 scores what stage 1 built, in Ravensim,
reversing on, **3 runs each**, on Ben's Mac (M4 Max), one job at a time, overnight.
Scores are tracked, not gates ([the league](league.md#how-a-challenger-wins)).

**How each was run.** `measure controllers` starts `stack:=best` (the champions) and changes
one thing. So each challenger is compared with **the option it would replace**, run the
same way. The raw results: `log/stage2/<job>/results.md` (not in git).

## What changed

| Level | Was | Now | Why |
|---|---|---|---|
| steer (`stack:=ours`) | `mppi_lag_calm` | **`mppi_lag_1k`** | no column worse on either tour, the worst off-path smaller, half the work a tick |

Nothing else changed. `stack:=car` is untouched: `mppi_lag_1k` still needs timing on the
Orin (TODO 152).

## Each challenger against what it would replace

Lot: 4 legs, 12 a job. University box: 1 leg, 3 a job. Every job reached every leg.
Sway in m/s²; off the path in m, mean / worst.

| Challenger | Baseline | Lot sway | Lot off the path | Box sway | Box off the path | Verdict |
|---|---|---:|---:|---:|---:|---|
| `steer:=mppi_lag_1k` (152) | `mppi_lag_calm` | 0.025 / 0.025 | 0.14 / 0.64 vs 0.15 / 0.91 | 0.095 / 0.100 | 0.12 / 0.60 vs 0.14 / 0.68 | ✅ **wins**: now ours' |
| `steer:=mppi_lag_smooth` (212) | `mppi_lag_calm` | **0.018** / 0.025 | 0.19 / 0.82 vs 0.15 / 0.91 | 0.095 / 0.100 | **0.22** / 0.87 vs 0.14 / 0.68 | 🟡 the calmest, but further off the path |
| `path:=lane_jerk` (214) | `path:=lane` | 0.029 / 0.032 | 0.19 / 1.01 vs 0.19 / 0.96 | **0.108** / 0.148 | **0.22** / 0.92 vs 0.11 / 0.75 | 🟡 calmer, further off the path; the tours have no pass |
| `kerbs:=patchwork` (215) | `kerbs:=on` | 0.031 / 0.030 | 0.17 / 0.64 vs 0.20 / 0.98 | 0.191 / 0.232 | 0.14 / 0.60 vs 0.23 / 0.73 | 🟡 better on most, but more left-right changes on the lot (12.5 vs 10.1); real kerbs are its test |
| `behaviour:=on_per_kind` (217) | `behaviour:=on` | 0.031 / 0.035 | 0.19 / 0.99 vs 0.21 / 1.57 | **0.241** / 0.204 | 0.18 / 0.70 vs 0.11 / 0.69 | 🟡 better on the lot, worse on the box; nobody walks on these tours |
| `pose:=kinematic_ekf` (216) | `pose:=ekf` | 0.034 / 0.032 | 0.19 / 1.28 vs 0.18 / 0.90 | 0.216 / 0.218 | 0.21 / 0.89 vs 0.14 / 0.86 | ❌ no gain in the sim (no wheel slip); kept for the car |

MPPI's tick: `mppi_lag_1k` 1.8 ms mean (9.0 worst), the 2,000-sample ones 3.5 ms; none missed.

## The planners (behind plain `mppi_lag`)

| Planner | Lot sway | Lot off the path (m) | Box sway | Plan time, mean (ms) | Plans from the model |
|---|---:|---:|---:|---:|---:|
| `own` (the lattice) | 0.032 | 0.18 | 0.218 | 8-20 | — |
| `diffusion_rs` (Rust, Core ML) | **0.022** | 0.18 | 0.211 | 50-72 | **100 %** |
| `diffusion_coreml` (Python, Core ML) | 0.030 | 0.18 | 0.226 | 117 | **20-50 %** |
| `own_hybrid` (Hybrid-A*, 178) | 0.026 | 0.20 | **0.044** | 1-14 | — |
| `own_smooth` (177) | 0.028 | 0.19 | 0.247 | 15-18 | — |

- **Diffusion from Rust works**: every plan was the model's; the calmest on the lot. Its first fair score.
- **The Python worker on Core ML barely works**: the lattice drew most of its plans.
- **Hybrid-A\*** reached 15 of 15 legs (7 of 8 before), by far the calmest on the box; one run changed gear 6 times.
- **`own_smooth`** cut inside the turning radius once (0.50 m): not kept to the car.

## Lanes: University Drive (`tools/lane_score.py`)

| Path | Off the lane's centre (m) | In the lane | Over the centre line | Crossings a leg |
|---|---:|---:|---:|---:|
| **`path:=lane`** (the lane map, ours') | **0.24** | **100 %** | **0 %** | **0** |
| the lattice + `lanes:=camera` (264) | 1.35 | 59 % | 41 % | 2.4 |
| the lattice, no lanes | 1.41 | 48 % | 52 % | 4.7 |

The lane map wins where we have one. The camera's lanes help the lattice keep right, but
only a little: they are for roads with no lane map. Seen live (20 Nov): the learned finder
misses lines on some stretches.

## MPC against MPPI (21 Nov, TODO 213)

Ben, 20 Nov: "fix mpc to have reverse". Once `mpc_lag_rs` reversed, all three followers
raced with reversing on, 3 runs each: the lot, and University Drive with `path:=lane`.
Raw: `log/stage2/mpc_lot/`, `log/stage2/mpc_univ/`.

| | `mppi_lag` (ours) | `mpc_lag` (Python) | `mpc_lag_rs` (Rust) |
|---|---:|---:|---:|
| Lot: legs | 12/12 | 12/12 | 12/12 |
| Lot: sway (m/s²) | **0.032** | 0.052 | 0.055 |
| Lot: off the path, mean / worst (m) | 0.21 / 0.99 | 0.18 / 1.07 | **0.17 / 0.88** |
| University: legs | 6/9 * | 8/9 | **9/9** |
| University: sway (m/s²) | **0.132** | 0.224 | 0.251 |
| University: off the lane's centre (m) | 0.23 | 0.22 | 0.24 |
| University: in the lane, over the centre line | 100 %, 0 % | 100 %, 0 % | 100 %, 0 % |
| The tick, mean (ms) | 3.4 | 6.8 | **1.6** |

\* One run lost all 3 legs at the start: the sensor watchdog stopped the car (the wheels and
the IMU disagreed), not the follower.

- **`mpc_lag_rs` reaches every leg and reverses** (6.6 m backwards on the lot's turn back,
  as MPPI), the closest to the path and the cheapest.
- **But it sways about twice as much as MPPI.** So MPPI stays ours'.
- **So: the tube** (Ben, 21 Nov: "It doesn't need to be super aggressive sticking to the
  line"): no cost within w m of the path, so it stops chasing the centre.

### The tube, tuned (21 Nov)

`mpc_lag_rs` alone, 3 runs each, the same tours. Raw: `log/stage2/tube*/`.

| Tube w | University sway | Lot sway | Legs (lot, University) | Off the lane's centre (m) | University worst off the path (m) |
|---|---:|---:|---|---:|---:|
| none | 0.251 | 0.055 | 12/12, 9/9 | 0.24 | 0.95 |
| 0.1 m | 0.197 | 0.058 | 12/12, 9/9 | 0.21 | 0.90 |
| 0.2 m | 0.205 | 0.054 | 12/12, 9/9 | 0.28 | 1.33 |
| **0.3 m** | **0.160** | **0.054** | **12/12, 9/9** | 0.24 | 1.39 |
| 0.4 m | 0.185 | — | —, 9/9 | 0.26 | 1.57 |
| *`mppi_lag`* | *0.132* | *0.032* | *12/12, 6/9* | *0.23* | *0.81* |

- **0.3 m is the default now** (`vehicle.MPC_LAG_TUBE`; `mpc_lag_tube:=0` turns it off):
  University Drive's sway down 36 %, every leg, as near the lane's centre.
- **The lot is unchanged**: its sway is the tight turns, not wobble about the line.
- **Still behind MPPI on sway** (0.160 against 0.132), ahead on legs and the tick. Next:
  more cost on the steering's change (`r_rate`), and the costmap in its cost.

## Position and the checks

| Job | Result |
|---|---|
| NDT leading (`--ndt-leads`), campus | every leg; 92 % of clouds gave a pose, heading off 0.1° at worst, 1.7 ms a match |
| AMCL (`--amcl`), campus | **not run**: AMCL runs only on the test lot, the only world with a 2D map |
| `sim-check --blocked-street` (174) | ✅ the car backs out |
| `sim-check --moving` (189) | ❌ twice, then fixed: the walker's feet were kept as a kerb for ever. Since the fix in the voxels' step memory: ✅ the car waits, sets off and reaches the goal |

## What next

| # | Do |
|---|---|
| 1 | Diffusion: the Rust worker the default; score it on the campus |
| 2 | `mppi_lag_1k` timed on the Orin, then into `stack:=car` |
| 3 | A 2D map for the campus (`map-drive` makes one), so AMCL can be scored there |
| 4 | The camera's lanes where they miss lines; then `path:=lane` with the camera's lanes |
