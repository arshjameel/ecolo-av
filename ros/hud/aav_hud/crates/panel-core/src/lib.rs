//! The hardware panel's logic, with no ROS and no raylib, so it is tested on any laptop.
//!
//! The hardware panel (`hw_panel`, in panel-app) shows every sensor's number straight
//! from its own node, and has very simple buttons for the actuators: Speed +/−, the
//! steering arrows, CENTRE, STOP, hold-to-drive GO, and the E-STOP
//! (docs/designs/2026-10-29_two-screens.md). The buttons go through twist_mux's
//! `buttons` slot (45), ego and the spine, never around them.
//!
//! | Module | What it holds |
//! |---|---|
//! | [`config`] | the topic names, steps and caps, from `config/hw_panel.yaml` |
//! | [`vehicle`] | the wheelbase, the steering limit, which wheels the speed comes from (vehicle.yaml) |
//! | [`rules`] | `aav_actuator_tests/safety.py`'s rules, ported with its test vectors: the source, the stands refusal, the unseen speed, a takeover |
//! | [`gate`] | where the speed buttons may drive: SIM, BENCH, stands (later), never the car on the ground |
//! | [`buttons`] | what a press asks for, and what goes to twist_mux each frame |
//! | [`state`] | every sensor's and actuator's number, OK / STALE / NEVER, the command path |
//! | [`calibration`] | the steering calibration page: allowed or why not, limp, hold, steps, the marks, what to send |
//! | [`steering_limits`] | equal limits from the marks: `aav_legacy`'s `steering_limits.py`, on shared vectors |
//! | [`sweep`] | the lock-to-lock check: a state machine that stops and holds on resistance |
//! | [`graphs`] | the GRAPHS page: 60 s of what the Pi wrote and the ESP32 got, now / avg / low / high, each hop's verdict, FREEZE |
//! | [`measure`] | the MEASURE page: speed and distance since RESET, a tape run typed in, the wheel diameter's check |
//! | [`esp32`] | the ESP32's JSON status lines (`firmware/status/`), read only |
//! | [`ws`] | just enough of a WebSocket client to read them: the address, the opening, the frames |
//! | [`easter_egg`] | when the little robot may show: still, no warning, nothing changed; a check just PASSED |
//!
//! The E-STOP burst and the heartbeat that keeps the buttons to a live window are
//! hud-core's (`estop`, `heartbeat`).

pub mod buttons;
pub mod calibration;
pub mod config;
pub mod easter_egg;
pub mod esp32;
pub mod gate;
pub mod graphs;
pub mod measure;
pub mod rules;
pub mod state;
pub mod steering_limits;
pub mod sweep;
pub mod vehicle;
pub mod ws;
