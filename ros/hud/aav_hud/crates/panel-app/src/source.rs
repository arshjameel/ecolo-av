//! Where the panel's news comes from, and where its commands go: the ROS node, or the
//! demo. The screen sees only this trait, so it runs the same with either.

use std::time::Instant;

use panel_core::{buttons::Command, calibration::Request, state::Update};

use crate::esp32::Esp32Reader;

/// A source of [`Update`]s, and a sink for the buttons.
pub trait DataSource {
    /// A short label for the corner: "ROS" or "DEMO".
    fn label(&self) -> &'static str;

    /// The pretend car (`--demo`): the LADDER tab shows pretend data too, and runs nothing.
    fn is_pretend(&self) -> bool {
        false
    }

    /// Everything that arrived since the last call. Never blocks.
    fn drain(&mut self, now: Instant) -> Vec<Update>;

    /// The E-STOP button was pressed: `/safety/estop` true, five times. Never false.
    fn press_estop(&mut self);

    /// A frame was drawn, and this is what the buttons and the calibration page say to
    /// send now (None: nothing). They go out only while frames keep being drawn
    /// (hud-core's heartbeat).
    fn frame_drawn(&mut self, command: Option<Command>, calibration: Option<Request>);

    /// Whether the source has stopped (Ctrl-C, or the ROS node failed): the window closes.
    fn has_stopped(&self) -> bool {
        false
    }
}

/// A source with the ESP32's status lines beside it (`--esp32`): their news joins the
/// other's; the buttons and the e-stop go to the other only.
pub struct WithEsp32 {
    pub inner: Box<dyn DataSource>,
    pub esp32: Esp32Reader,
}

impl DataSource for WithEsp32 {
    fn label(&self) -> &'static str {
        self.inner.label()
    }

    fn is_pretend(&self) -> bool {
        self.inner.is_pretend()
    }

    fn drain(&mut self, now: Instant) -> Vec<Update> {
        let mut news = self.inner.drain(now);
        news.extend(self.esp32.drain());
        news
    }

    fn press_estop(&mut self) {
        self.inner.press_estop();
    }

    fn frame_drawn(&mut self, command: Option<Command>, calibration: Option<Request>) {
        self.inner.frame_drawn(command, calibration);
    }

    fn has_stopped(&self) -> bool {
        self.inner.has_stopped()
    }
}
