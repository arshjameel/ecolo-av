# Provenance

Ported for `ecolo-av` on 2026-11-17: a Rust rewrite of our own Python node and module
(Ben's code): `ros/drivers/aav_rc`'s `rc_driver.py` and `rc_drive.py`, at commit
`3cd39dc8`. No outside code: nothing new for `docs/acknowledgements.md`. The Python stays,
as the fall-back (`rc_impl:=python`).

| Part | From the Python |
|---|---|
| `crates/rc-core` | `rc_drive.py`: `Limits`, `Sticks`, `twist`, `RcDrive`; the switch is an enum |
| `crates/rc-node` | `rc_driver.py`: the same node name, topic, QoS and parameters |
| `gen_vectors.py` | new: `rc_drive.py`'s answers, for `tests/vectors.rs` |

## Where it differs

A mode switch number above 3 is logged as "no RC signal" (the Python logged the number).
It drives the same: nothing.
