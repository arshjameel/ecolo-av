# Provenance

Ported for `ecolo-av` on 2026-11-19: a Rust rewrite of our own Python node and module
(Ben's code): `ros/localization/aav_gps_heading`'s `gps_heading.py` and `course.py`, at
commit `ceab7c55`. The Python stayed as the fall-back until 20 Nov, then went (tag `save/python-gps-heading`);
`vectors.json` is now a golden fixture.

| Part | From |
|---|---|
| `crates/gps-heading-core` | `course.py`; `hypot` is CPython 3.12's `vector_norm`, as in `aav_safety_rs` (`docs/acknowledgements.md`) |
| `crates/gps-heading-node` | `gps_heading.py`: the same node name, topics, QoS and parameters |
| `gen_vectors.py`, `test/test_gps_heading_rs_node.py` | new |

## Where it differs

With `min_speed:=0`, a GPS speed of exactly 0 gives no heading (the Python divides by
zero and stops).
