# Provenance

Ported for `ecolo-av` on 2026-11-17 (Ben: "Rust tomorrow"): a Rust rewrite of our own
Python nodes and modules (Ben's code): `ros/io_mux/aav_io_mux`'s `sensor_mux.py`,
`output_mux.py`, `provenance.py` and `heartbeat.py` at commit `3cd39dc8`. No outside code:
nothing new for `docs/acknowledgements.md` (Python's `%g` is ego-core's, reused).

**22 Nov:** the Python nodes (`sensor_mux.py`, `output_mux.py`) were removed
(Ben: "lets start removing the python fall back after being tested enough times"; git tag `save/python-fallbacks-1`).
`provenance.py`, `heartbeat.py` and `muxes.py` stay in `aav_io_mux`, so `gen_vectors.py`
still runs.

| Part | From the Python |
|---|---|
| `crates/io-mux-core/src/provenance.rs` | `provenance.py`: `Kind` (the numbers as an enum, `Unknown(n)` never REAL), `Target`, `ChoiceError` (the `ValueError`s' words), `SENSORS`, `Announcers`, `foreign`, `check_sensors`, `Hold`, `Report`, `Gate` (its `clear` returns a `Result`). `choose` stays Python only: the launch file's |
| `crates/io-mux-core/src/muxes.rs` | `muxes.py`: the two nodes' work without ROS (new on 17 Nov, taken out of `sensor_mux.py` and `output_mux.py`, which now call it) |
| `crates/io-mux-core/src/lib.rs` | `heartbeat.py`: `Counter` |
| `crates/io-mux-node` | `sensor_mux.py`, `output_mux.py`: the same node names, topics, QoS, parameters and service; its `Declarer` is `aav_watchdog_rs`'s `params.rs`, the parts these nodes use |
| `gen_vectors.py` | new: the Python's answers, step by step, for `tests/vectors.rs` |

## Where it differs

- The steady clock is Rust's `Instant` (offset by 1000 s); the Python's is
  `time.monotonic()`. Only differences between times count.
- The parameters are read only (`ros2 param set` is refused; the Python accepted it and
  ignored it).
- Where the ROS graph can't be read, the Python node would raise and die; the Rust one
  skips that judgement (no verdict; the gate's report ages, so it waits) and refuses a
  clear.

## Changes since

None yet.
