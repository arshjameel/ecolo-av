# Provenance

Ported for `ecolo-av` on 2026-11-16 (TODO 185): a Rust rewrite of our own Python node and
modules (Ben's code): `ros/safety/aav_safety`'s `sensor_watchdog.py`, `watchdog.py` and
`wheel_check.py` at commit `678d2cd4`. No outside code: nothing new for
`docs/acknowledgements.md` (CPython's `hypot` and `%g` are ego-core's, reused; its `sum()`
is the same Neumaier sum as `aav_behaviour_rs`'s; zlib's CRC-32 is the standard polynomial,
written out). The Python stays, as the default.

| Part | From the Python |
|---|---|
| `crates/watchdog-core/src/watch.rs` | `watchdog.py`: `Rule`, `Watch`, `Watchdog`, the eleven states and every check (stale, NaN, frozen, jump, disagree, stuck, overspeed, lost...). Kinds and states are enums; a bad rule is a `RuleError` |
| `crates/watchdog-core/src/wheels.rs` | `wheel_check.py`: `CrossCheck`, `WheelCheck`, `trapezoid`, `fitted_speed` |
| `crates/watchdog-core/src/feed.rs` | `sensor_watchdog.py`'s logic without ROS: what each message gives the watch, the tick (states, lock, state line, what is logged), the `/diagnostics` statuses |
| `crates/watchdog-core/src/lib.rs` | `py_sum`, `py_max`, `py_min` (as `aav_behaviour_rs`'s); `crc32` and `picture_digest`: `zlib.crc32` of every 997th byte, as the node digests a picture |
| `crates/watchdog-node` | `sensor_watchdog.py`: the same node name, topics, QoS, parameters and service; `params.rs` is `aav_behaviour_rs`'s |
| `gen_vectors.py` | new: the Python node's answers, tick by tick, for `tests/vectors.rs` |

## Where it differs

None on purpose. The steady clock (for a stalled node) is Rust's `Instant`; the Python's is
`time.monotonic()`.

**20 Nov:** the Python node(s) in `aav_safety` were removed (Ben, 16 Nov: "lets remove python
were possible"; tag `save/python-safety-nodes`). The maths modules stay there, so
`gen_vectors.py` still runs.
