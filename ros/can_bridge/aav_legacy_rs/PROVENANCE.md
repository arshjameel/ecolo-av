# Provenance

Ported for `ecolo-av` on 2026-11-17: a Rust rewrite of our own Python node and modules
(Ben's code): `ros/can_bridge/aav_legacy`'s `legacy_uart_bridge.py`, `legacy_protocol.py`
(the mapping and the uplink parser), `uart_relay.py`, `speed_control.py`,
`uplink_reader.py` and `link_report.py`, at commit `3cd39dc8`. No outside code: nothing new
for `docs/acknowledgements.md` (the smooth stop's idea is Autoware's, already credited
there for `speed_control.py`). The frames are `firmware/spine/crates/legacy-link`'s
(ours), reused, not copied. The Python stays, as the fall-back (`legacy_impl:=python`).

| Part | From the Python |
|---|---|
| `crates/legacy-bridge/src/mapping.rs` | `LegacyMapping`, `to_legacy`, `from_pedals`, `stop_command`, `clamp1` |
| `crates/legacy-bridge/src/relay.rs` | `uart_relay.py`: `Relay`, `programs_locked`; versions, places and refusals are enums |
| `crates/legacy-bridge/src/speed.rs` | `speed_control.py`: `SpeedControl`, `SpeedControlConfig`, `Pedals` |
| `crates/legacy-bridge/src/uplink.rs` | `UplinkParser`, `uplink_reader.py`'s reader and fields, `CAL_STATES`' words |
| `crates/legacy-bridge/src/link.rs` | `link_report.py`: `LinkReport`, `frame_fields` |
| `crates/legacy-node` | `legacy_uart_bridge.py`: the same node name, topics, QoS and parameters; `params.rs` is `aav_watchdog_rs`'s, with whole numbers |
| `gen_vectors.py` | new: the Python modules' answers, for `tests/vectors.rs` |

Changed in `legacy-link` for this: `DriveMode::to_byte` and `ModeSwitch::to_byte` are public.

## Where it differs

| What | Python | Rust |
|---|---|---|
| a calibration request with an unknown mode | its callback raised: the node stopped | logged, dropped |
| port URLs | any pyserial URL | a device, or `socket://host:port` |
| a write that can't finish | blocks (pyserial, no write timeout) | 50 ms, then counted as failed and the port reopened |
| a pseudo-terminal on macOS | pyserial sets its speed | serialport can't: opened with its speed left alone |
