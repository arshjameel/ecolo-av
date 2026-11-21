# Provenance

Ported for `ecolo-av` on 2026-11-19: a Rust rewrite of our own Python nodes and modules
(Ben's code): `ros/safety/aav_safety`'s `geofence.py`, `geofence_node.py`, `hold.py`,
`stop_hold.py`, `leash.py` and `goal_leash_node.py`, at commit `ceab7c55`. The Python
stays, as the fall-back (`*_impl:=python`).

| Part | From |
|---|---|
| `crates/safety-core/src/geofence.rs` | `geofence.py`: the fences, `site_fence`, `Motion`, `Settings`, `Pose`, `Geofence`, `stopping_distance`, `along_arc`; the states as the enum `State` (the launch file's `motion_from` and its helpers stay Python) |
| `crates/safety-core/src/hold.rs`, `leash.rs` | `hold.py`, `leash.py` |
| `crates/safety-core/src/pyfmt.rs` | new: Python's `:.1f`, `repr`, `:g`, `min`, `max`, Python 3.12's `sum()` (Neumaier), and CPython 3.12's `math.hypot` (`vector_norm` in `Modules/mathmodule.c`, PSF licence: its algorithm, rewritten in Rust) |
| `crates/safety-node` | the three nodes: the same names, topics, QoS and parameters |
| `gen_vectors.py`, `test/test_safety_rs_nodes.py` | new |

## Where it differs

A datum's `repr` in an error line never uses the exponent form; the geofence reads
`vehicle.yaml`'s body itself (`vehicle_yaml`).

**20 Nov:** the Python node(s) in `aav_safety` were removed (Ben, 16 Nov: "lets remove python
were possible"; tag `save/python-safety-nodes`). The maths modules stay there, so
`gen_vectors.py` still runs.
