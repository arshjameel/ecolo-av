# Provenance

Ported for `ecolo-av` on 2026-11-17: a Rust rewrite of our own Python node and module
(Ben's code): `ros/safety/aav_safety`'s `joy_alive_node.py` and `joy_alive.py`, at commit
`8e5ca0eb`. No outside code: nothing new for `docs/acknowledgements.md`. The Python stays,
as the fall-back until 20 Nov.

| Part | From the Python |
|---|---|
| `crates/joy-alive-core` | `joy_alive.py`: `JoyAlive`, its states as the enum `Trip`, the settings' checks as `SettingsError`; `g` matches Python's `:g` for the state lines |
| `crates/joy-alive-node` | `joy_alive_node.py`: the same node name, topics, QoS and parameters |
| `gen_vectors.py` | new: `joy_alive.py`'s answers, for `tests/vectors.rs` |
| `aav_teleop`'s `test/test_joy_chain.py` | new: the joystick's chain, both languages |

## Where it differs

A negative `motion_axes` entry is refused at start-up (the Python read it from the end).

**20 Nov:** the Python node(s) in `aav_safety` were removed (Ben, 16 Nov: "lets remove python
were possible"; tag `save/python-safety-nodes`). The maths modules stay there, so
`gen_vectors.py` still runs.
