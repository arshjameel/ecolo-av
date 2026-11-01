//! Where the panel's news comes from, and where its commands go: the ROS node, or the
//! demo. The screen sees only this trait, so it runs the same with either.

use std::time::Instant;

use panel_core::{buttons::Command, state::Update};

/// A source of [`Update`]s, and a sink for the buttons.
pub trait DataSource {
    /// A short label for the corner: "ROS" or "DEMO".
    fn label(&self) -> &'static str;

    /// Everything that arrived since the last call. Never blocks.
    fn drain(&mut self, now: Instant) -> Vec<Update>;

    /// The E-STOP button was pressed: `/safety/estop` true, five times. Never false.
    fn press_estop(&mut self);

    /// A frame was drawn, and this is what the buttons say to send now (None: nothing).
    /// The command goes out only while frames keep being drawn (hud-core's heartbeat).
    fn frame_drawn(&mut self, command: Option<Command>);

    /// Whether the source has stopped (Ctrl-C, or the ROS node failed): the window closes.
    fn has_stopped(&self) -> bool {
        false
    }
}
