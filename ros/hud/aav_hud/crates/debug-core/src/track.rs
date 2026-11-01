//! `/drivers/nav/track/status` (diagnostic_msgs/DiagnosticStatus, aav_track) in plain
//! words: which trial tracker drives, and how well it is doing.

/// What the panel shows of the tracker.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TrackStatus {
    /// pure_pursuit, stanley, lqr or mpc.
    pub tracker: String,
    /// Metres, + left of the path.
    pub cross_track: Option<f64>,
    /// This tick's work, milliseconds.
    pub loop_ms: Option<f64>,
    /// Ticks missed since the path came.
    pub missed_ticks: Option<u64>,
    /// Radians, + left.
    pub steer: Option<f64>,
    /// Metres left along the path.
    pub left: Option<f64>,
}

impl TrackStatus {
    /// From the message's `message` and its key-value pairs. Unknown or unreadable
    /// values are left out; the tracker's name falls back to `message`.
    pub fn from_values<'a>(
        message: &str,
        values: impl IntoIterator<Item = (&'a str, &'a str)>,
    ) -> Self {
        let mut s = Self {
            tracker: message.to_owned(),
            ..Self::default()
        };
        let number = |v: &str| v.trim().parse::<f64>().ok().filter(|x| x.is_finite());
        for (key, value) in values {
            match key {
                "tracker" if !value.is_empty() => s.tracker = value.to_owned(),
                "cross_track_m" => s.cross_track = number(value),
                "loop_ms" => s.loop_ms = number(value),
                "missed_ticks" => s.missed_ticks = value.trim().parse().ok(),
                "steer_rad" => s.steer = number(value),
                "left_m" => s.left = number(value),
                _ => {}
            }
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_aav_tracks_values() {
        let s = TrackStatus::from_values(
            "lqr",
            [
                ("tracker", "lqr"),
                ("loop_ms", "0.412"),
                ("cross_track_m", "-0.051"),
                ("missed_ticks", "2"),
                ("steer_rad", "0.1200"),
                ("left_m", "12.50"),
                ("k_ey", "0.9"),
            ],
        );
        assert_eq!(s.tracker, "lqr");
        assert_eq!(s.cross_track, Some(-0.051));
        assert_eq!(s.missed_ticks, Some(2));
        assert_eq!(s.left, Some(12.5));
    }

    #[test]
    fn bad_values_are_left_out() {
        let s = TrackStatus::from_values("stanley", [("loop_ms", "nan"), ("steer_rad", "x")]);
        assert_eq!(s.tracker, "stanley");
        assert_eq!((s.loop_ms, s.steer), (None, None));
    }
}
