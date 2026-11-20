# Provenance

Ported for `ecolo-av` on 2026-11-16 (TODO 185): a Rust rewrite of our own Python node and
module (Betsi's, 1 Nov): `ros/localization/aav_localization_health`'s
`localization_health.py` and `health.py` at commit `3cb75bbe`. No outside code: nothing
new for `docs/acknowledgements.md` (CPython's `hypot` and `%g` are `ego-core`'s, its
`sum()`, `max()` and `min()` are `watchdog-core`'s, all reused; tf2's `Time()` lookup is
written out from what tf2 does, checked against tf2's own Buffer). The Python stayed until 20 Nov, then went (tag
`save/python-localization-health`); `config/localization_health.yaml` moved here, and
`vectors.json` is now a golden fixture.

| Part | From the Python |
|---|---|
| `crates/health-core/src/health.rs` | `health.py`: `Settings`, `Status`, `Health` (every check), `front_wheel_speeds`, `mean_and_sigma`, `summary`. Checks, inputs and levels are enums (`Check`, `Input`, `Ekf`, `Level`) |
| `crates/health-core/src/tree.rs` | new: the part of tf2's buffer the node uses (is a frame there; `Time()`'s stamp, as `getLatestCommonTime`, and whether every moving link has data then) |
| `crates/health-core/src/lib.rs` | `format_g` (`format(v, '.4g')`, as `/diagnostics` writes numbers); `degrees`, `radians` (CPython's constants) |
| `crates/health-core/tests/checks.rs` | `test/test_health.py`, every case |
| `crates/health-node` | `localization_health.py`: the same node name, topics, QoS and parameters |
| `gen_vectors.py` | new: the Python's answers on made-up drives, and tf2's on random trees |

## Changed in the Python, for this port

| What | Why |
|---|---|
| `mean_and_sigma`: `(x - mean) * (x - mean)`, not `(x - mean) ** 2` (16 Nov) | `**` is libm's `pow()`, and macOS's rounds a bit differently from Linux's (1,556 of 20,000 lists differed on a Mac): the shared answers would hold on one machine only. A product rounds the same everywhere |

## Where it differs

None on purpose. `tilt_deg` goes through libm's `acos` on each side: the vectors allow
it 4 ulps, for a Linux machine checking a file a Mac wrote.
