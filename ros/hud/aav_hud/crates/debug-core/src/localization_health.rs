//! Betsi's localization health checks (aav_localization_health) on the debug screen.
//!
//! `/localization/health` is a std_msgs/String, once a second: `OK`, or `WARN` and every
//! check that isn't OK, as `WARN <check> <level>: <why>; <check> <level>: <why>`
//! (its `health.py`, `summary()`). It only warns: nothing drives from it.

/// What `/localization/health` said.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Verdict {
    /// Every check is OK.
    Ok,
    /// These checks aren't, each as `<check> <level>: <why>`.
    Warn(Vec<String>),
    /// Something else: shown as it came.
    Other(String),
}

impl Verdict {
    /// Reads the message's text.
    pub fn parse(text: &str) -> Self {
        let text = text.trim();
        if text == "OK" {
            return Self::Ok;
        }
        match text.strip_prefix("WARN") {
            Some(rest) if rest.is_empty() || rest.starts_with(char::is_whitespace) => Self::Warn(
                rest.split(';')
                    .map(str::trim)
                    .filter(|c| !c.is_empty())
                    .map(str::to_owned)
                    .collect(),
            ),
            _ => Self::Other(text.to_owned()),
        }
    }

    /// The panel's big word.
    pub fn word(&self) -> &'static str {
        match self {
            Self::Ok => "OK",
            Self::Warn(_) => "WARN",
            Self::Other(_) => "?",
        }
    }

    /// The checks that aren't OK, by name only: "gyro_bias, tf".
    pub fn names(&self) -> String {
        match self {
            Self::Ok => String::new(),
            Self::Warn(checks) => checks
                .iter()
                .map(|c| c.split_whitespace().next().unwrap_or(c))
                .collect::<Vec<_>>()
                .join(", "),
            Self::Other(text) => text.clone(),
        }
    }

    /// One line for the panel: "OK", or "WARN: gyro_bias, tf".
    pub fn line(&self) -> String {
        match self {
            Self::Ok => "OK".to_owned(),
            Self::Warn(_) => format!("WARN: {}", self.names()),
            Self::Other(text) => text.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ok_and_warn_read() {
        assert_eq!(Verdict::parse("OK\n"), Verdict::Ok);
        let v = Verdict::parse(
            "WARN gyro_bias WARN: parked, mean +0.310 deg/s; gps_quality STALE: no fix for 3.0 s",
        );
        assert_eq!(
            v,
            Verdict::Warn(vec![
                "gyro_bias WARN: parked, mean +0.310 deg/s".into(),
                "gps_quality STALE: no fix for 3.0 s".into(),
            ])
        );
        assert_eq!(v.word(), "WARN");
        assert_eq!(v.line(), "WARN: gyro_bias, gps_quality");
    }

    #[test]
    fn anything_else_is_shown_as_it_came() {
        assert_eq!(
            Verdict::parse("WARNING x"),
            Verdict::Other("WARNING x".into())
        );
        assert_eq!(Verdict::parse("WARN"), Verdict::Warn(Vec::new()));
        assert_eq!(Verdict::parse("hello").line(), "hello");
    }
}
