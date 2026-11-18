# aav_rc_rs

## Overview

`rc_driver` in Rust (rclrs): the **default** since 17 Nov, with the Rust UART bridge, in
place of `aav_rc`'s Python one. The same node name, topic, QoS and parameters.
`rc_impl:=python` falls back.

```
legacy_uart_bridge ─/vehicle/rc─> rc_driver ─/drivers/rc/cmd_vel─> twist_mux (95)
                                  only while the mode switch is on RC via ROS
```

## Interface

| Crate | Is | ROS? |
|---|---|---|
| `crates/rc-core` | the rules: `Switch` (an enum), `Limits`, `Sticks`, `twist`, `RcDrive` | no: `cargo test` |
| `crates/rc-node` | the node, binary `rc_driver` | yes (rclrs 0.8) |

The node, its parameters and rules: `aav_rc`'s README. `rc.launch.py` and
`legacy.launch.py` start it (`rc_impl:=rust`, the default).

## Invariants

- **The same commands as the Python, exactly**: `tests/vectors.rs` reads `vectors.json`,
  which `gen_vectors.py` writes from `rc_drive.py` (every switch, NaN sticks, silences).
  Change one, change both, then `pixi run python ros/drivers/aav_rc_rs/gen_vectors.py`.
- One zero as the switch leaves RC via ROS; zeros while `/vehicle/rc` is quiet on it.
- `rc-core` has no ROS; `cargo fmt` and `cargo clippy -D warnings` clean.

## Side Effects

- Publishes `/drivers/rc/cmd_vel` at 50 Hz while driving.

## Verification

```sh
cd ros/drivers/aav_rc_rs && cargo test -p rc-core   # tools/test.sh runs it
```

The process: `aav_legacy`'s `test_uplink_chain.py` runs it beside the Rust bridge.
