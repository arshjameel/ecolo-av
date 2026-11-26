# One car model

How the car answers a command: one model, in Python and Rust, for ego, the simulators
and the drivers. Ben, 7 Nov: "we really want the ego to have a mathematical model for
steering, braking and acceleration ... have the algorithms know about lag".

> **Every number is a GUESS** until car tests 3.10-3.12 and 4.1-4.5
> ([actuator tests](actuator-tests.md)). The dead times are 0 until measured.

## Where it lives

| What | Where |
|---|---|
| Python | `ros/bringup/aav_description/aav_description/car_model.py` |
| Rust twin | `ros/bringup/aav_description/car-model/` (crate `car-model`) |
| Shared test vectors | `car-model/vectors.json`, written by `car-model/gen_vectors.py` |
| Numbers | `vehicle.yaml`: `actuators:`, `wheelbase`, `max_steering_angle` |
| Old import path | `aav_sim_models.actuator_model` still works: the same objects |

Why `aav_description`: it already holds `vehicle.yaml` and `vehicle.py`, and ego and
the simulator both depend on it. The model is part of "what the car is".

## The picture

```
 command ──▶ dead time ──▶ lag ──▶ speed ──────────────┐
 (pedals or                                            ├──▶ bicycle ──▶ x, y, yaw
  a set speed)                                         │
 steering ─▶ dead time ─▶ deadband ─▶ motor rate ─▶ linkage lag ─▶ + offset ─┘
```

A set speed goes through the spine's speed loop first (Python: the port; Rust:
`spine-core`'s own).

## One step of dt

| # | Part | Maths |
|---|---|---|
| 1 | Dead time | each input waits `floor(dead_time / dt + 0.5)` steps |
| 2 | Throttle, brake | `level' = (u - level) / lag`, stepped exactly: `level += (u - level)(1 - e^(-dt/lag))` |
| 3 | Speed | `v += max_accel · throttle · dt` (0 below the deadband), then brake and drag take `(max_brake · brake + drag · abs(v)) · dt` off, never past 0 |
| 4 | Steering | setpoint moves only past `steer_deadband`; motor at most `steer_rate · dt`; linkage lags `steer_lag`; plus `steer_offset`; within `±max_steering` |
| 5 | Bicycle | `yaw += v tan(steer) / wheelbase · dt`; `x += v cos(yaw_mid) dt`, `y += v sin(yaw_mid) dt` |

`v` and `steer` in step 5 are the step's new ones; `yaw_mid` is halfway between the old
and new yaw.

## The API

| Python (`aav_description.car_model`) | Rust (`car_model`) |
|---|---|
| `ActuatorConfig.from_vehicle_yaml(path)` | `ActuatorConfig::from_vehicle_yaml(path)?` |
| `Command(steer, throttle, brake, reverse)` | `Command::pedals(throttle, brake, steer)` |
| `Command(steer, speed=v)` | `Command::speed(v, steer)` |
| `step(cfg, state, command, dt, loop)` | `cfg.step(&state, command, dt, Some(&mut loop))?` |
| `predict(cfg, state, commands, dt, horizon, loop)` | `cfg.predict(&state, &commands, dt, horizon, Some(&loop))?` |
| `CarState` | `CarState` |

`predict` gives a state every `dt` for `horizon` s, the first one `dt` ahead. The last
command is held. The speed loop is copied, never advanced.

## ego's lead

The actuators answer late. ego can ask a little ahead, so the car answers about when the
driver wanted (`aav_ego` `lag_lead.py`).

```
driver's request ──▶ filter (lag_lead_filter) ──▶ + T × its rate ──▶ ego's limits ──▶ car
```

| Lead | T |
|---|---|
| speeding up | `throttle_dead_time + throttle_lag` |
| slowing | `brake_dead_time + brake_lag` |
| steering | `steer_dead_time + steer_lag` |

The car then answers as if its lag were `lag_lead_filter` (0.1 s), not T. A dead time
can't be undone (it would need the future): it counts as more lag. The steering motor's
rate limit isn't a lag: nothing undoes it.

### When it runs

| `lag_lead` | Simulator (`use_sim_time`) | Real car |
|---|---|---|
| `auto` (default) | **on** | off |
| `on` | on | on: only after tests 3.10-3.12 and 4.1-4.5 |
| `off` | off | off |

Never while a lag-aware driver drives, whatever the setting. Change it live:
`ros2 param set /ego lag_lead off`.

### Still a safety lock

| Rule | How |
|---|---|
| Every limit still applies | the lead runs **before** `max_speed`, the steering limit and rate, the slews, the jerk, the body |
| Small | at most `lag_lead_max_speed` (0.3 m/s) and `lag_lead_max_steer` (0.1 rad) |
| No change of direction | forward stays forward, reverse stays reverse |
| A slow-down is never softened | while slowing, never above the request |
| A stop is a stop | no fresh command: no lead |

On the car model with today's guesses, the lead follows a speed ramp about 6% closer: the
spine's speed loop is most of the lag, and the lead can't change that loop.

## Lag-aware drivers: the contract

A driver that leads for the lag itself (an MPC or MPPI that uses this model) tells ego,
so the lag isn't led for twice.

| Step | The driver does |
|---|---|
| 1 | publish `true` on `/drivers/<slot>/lag_aware` (`std_msgs/Bool`, latched: reliable, transient local, depth 1) when its lag-aware mode starts; `false` when it ends |
| 2 | stamp every `TwistStamped` with now |
| 3 | that's all: ego does the rest |

How ego knows who is driving: twist_mux only publishes `/diagnostics`, not the winner. But
it passes the winner's message on **unchanged**. So ego keeps each lag-aware driver's last
50 commands and looks for the one on `/control/cmd_vel` (same stamp and twist).

| Slot | Who will publish it |
|---|---|
| `nav` | `aav_track`'s lag-aware trackers, the Rust MPPI (both drive through navigation's slot) |
| `scripted`, `point`, `ai` | nobody yet |

The slots ego watches are its `lag_aware_drivers` parameter.

## ego's trace

Where the car is going, like a tail but in front (Ben, 7 Nov). `/ego/trace`,
`nav_msgs/Path`, 10 Hz (`aav_ego` `trace_ahead.py`).

| | |
|---|---|
| Horizon | 2 s, a pose every 0.1 s (21 poses) |
| Frame | `base_link`, at the header's stamp |
| Starts from | ego's twin of the car: the model, stepped with what ego sent, its speed the last `/vehicle/odom` |
| Runs | a copy of ego (lead, ramps, limits) with the request held, into the car model |

Why `base_link`: ego needs no localization (stands, the turtle). A driver turns the trace
into `odom` with TF at the stamp, then matches it against its plan.

## Checks

| Test | Checks |
|---|---|
| `aav_description/test/test_car_model.py` | dead times, the circle, braking, predict; the vectors are up to date |
| `car-model/tests/vectors.rs` | the Rust gives the Python's states (1e-9; 1e-3 with a speed loop, which is f32 in Rust) |
| `aav_sim_models/test/test_actuator_model.py` | the simulated car is unchanged |
| `aav_ego/test/test_lag_lead.py` | the lead: when it runs, its limits, ego's limits after it |
| `aav_ego/test/test_trace_ahead.py` | the trace and the twin |
| `aav_ego_rs/test/test_ego_rs_node.py` | the node (Rust): a command drives, both nodes answer |

After changing either side:

```bash
python3 ros/bringup/aav_description/car-model/gen_vectors.py
tools/test.sh
```
