# Provenance

Ported for `ecolo-av` on 2026-11-18: a Rust rewrite of our own Python nodes and modules
(Ben's code): `ros/drivers/aav_drive_to_point`'s `point_driver.py`, `point_cli.py`,
`approach.py`, `line.py`, `drive.py`, `rules.py` and `run.py`, at commit `09aa2c28` (with
`run.py`, the drive's ticks moved out of `point_driver.py` the same day). No outside code:
nothing new for `docs/acknowledgements.md`.

**22 Nov:** the Python node and client (`point_driver.py`, `point_cli.py`) were removed
(Ben: "lets start removing the python fall back after being tested enough times"; git tag `save/python-fallbacks-1`).
The maths modules stay in `aav_drive_to_point`, so `gen_vectors.py` still runs.

| Part | From the Python |
|---|---|
| `crates/point-core` `approach`, `line`, `drive`, `rules`, `run` | the modules of the same names; the reasons are enums with the Python's words |
| `crates/point-core` `cli` | `point_cli.py`'s argument rules, `--ahead` sum and feedback line |
| `crates/point-node` `point_driver` | `point_driver.py`: the same node name, action, topics, QoS and parameters |
| `crates/point-node` `drive_to_point` | `point_cli.py` |
| `gen_vectors.py` | new: the Python modules' answers, for `tests/vectors.rs` |

It reuses our own crates: `ego-core` (the stopping law, the car model, Python's `hypot`
and `g` format) and `behaviour-core` (the geofence's depth).

## Where it differs

| What | Rust |
|---|---|
| The drive's timer | runs all the time at `rate_hz` and ticks only during a drive (the Python made one per drive) |
| A plan that can't be made from odd numbers | refused with `error: ...` and the reason (the Python raised, then aborted with the exception's text) |
| Ctrl-C at the client's "type drive" prompt | ends the client at once (nothing moved), as before, but without the "Cancelled" line |
