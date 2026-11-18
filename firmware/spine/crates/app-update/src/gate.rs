//! May an update start, or go on? The car must be parked, and the password right.

/// The brake (0..1) the car must have on: full, as `IDLE_BRAKE` 1.0 keeps it on the
/// ESP32 board while nobody drives.
pub const BRAKED_AT_LEAST: f32 = 0.99;

/// The shortest update password taken.
pub const MIN_PASSWORD_LEN: usize = 12;

/// After a wrong password, every try is refused for this long (ms).
pub const LOCKOUT_MS: u32 = 10_000;

/// The car as the latest tick left it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarState {
    /// Nobody drives: the spine's mode is manual (not RC, the computer or a test).
    pub stopped: bool,
    /// The car is armed: it may drive.
    pub armed: bool,
    /// 0..1, to the DAC.
    pub throttle: f32,
    /// 0..1, to the brake actuator.
    pub brake: f32,
}

/// Why not now.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NotNow {
    /// Someone drives: the RC, the computer, or a test.
    Driving,
    /// The car is armed.
    Armed,
    /// The throttle isn't 0.
    Throttle,
    /// The brake isn't full on.
    NotBraked,
}

impl NotNow {
    /// For the reply and the log.
    #[must_use]
    pub const fn reason(self) -> &'static str {
        match self {
            NotNow::Driving => "someone is driving (RC, computer or test)",
            NotNow::Armed => "the car is armed",
            NotNow::Throttle => "the throttle is on",
            NotNow::NotBraked => "the brake is not full on",
        }
    }
}

/// The car is parked: stopped, not armed, no throttle, the brake full on.
///
/// # Errors
///
/// The first reason it isn't, in the order above.
pub fn may_update(s: &CarState) -> Result<(), NotNow> {
    if !s.stopped {
        Err(NotNow::Driving)
    } else if s.armed {
        Err(NotNow::Armed)
    } else if s.throttle != 0.0 {
        Err(NotNow::Throttle)
    } else if s.brake.is_nan() || s.brake < BRAKED_AT_LEAST {
        Err(NotNow::NotBraked)
    } else {
        Ok(())
    }
}

/// Why a password is refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PasswordError {
    /// The build's own password is too short, or the example's: updates are off.
    NotSetUp,
    /// Not the password.
    Wrong,
    /// A wrong one came less than [`LOCKOUT_MS`] ago.
    LockedOut {
        /// How long until a try is taken again (ms).
        ms_left: u32,
    },
}

impl PasswordError {
    /// For the reply and the log.
    #[must_use]
    pub const fn reason(self) -> &'static str {
        match self {
            PasswordError::NotSetUp => {
                "updates are off: this build's update password is missing, too short or the example's"
            }
            PasswordError::Wrong => "wrong password",
            PasswordError::LockedOut { .. } => "a wrong password came less than 10 s ago",
        }
    }
}

/// Checks the update password, and locks out for [`LOCKOUT_MS`] after a wrong one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Guard<'a> {
    expected: &'a str,
    example: &'a str,
    wrong_at: Option<u32>,
}

impl<'a> Guard<'a> {
    /// `expected`: the build's password (from the untracked secrets file). `example`: the
    /// example file's, which is never taken.
    #[must_use]
    pub const fn new(expected: &'a str, example: &'a str) -> Self {
        Guard {
            expected,
            example,
            wrong_at: None,
        }
    }

    /// Is `given` the password, at `now_ms`? Compared in constant time.
    ///
    /// # Errors
    ///
    /// [`PasswordError`]: updates aren't set up, the password is wrong, or a
    /// wrong one came too lately.
    pub fn check(&mut self, given: &[u8], now_ms: u32) -> Result<(), PasswordError> {
        if self.expected.len() < MIN_PASSWORD_LEN || self.expected == self.example {
            return Err(PasswordError::NotSetUp);
        }
        if let Some(at) = self.wrong_at {
            let since = now_ms.wrapping_sub(at);
            if since < LOCKOUT_MS {
                return Err(PasswordError::LockedOut {
                    ms_left: LOCKOUT_MS - since,
                });
            }
        }
        if same(given, self.expected.as_bytes()) {
            self.wrong_at = None;
            Ok(())
        } else {
            self.wrong_at = Some(now_ms);
            Err(PasswordError::Wrong)
        }
    }
}

/// The same bytes, in a time that doesn't depend on where they differ.
fn same(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b).fold(0u8, |d, (x, y)| d | (x ^ y)) == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    const PARKED: CarState = CarState {
        stopped: true,
        armed: false,
        throttle: 0.0,
        brake: 1.0,
    };

    #[test]
    fn only_a_parked_car_updates() {
        assert_eq!(may_update(&PARKED), Ok(()));
        let not = |s: CarState| may_update(&s).unwrap_err();
        assert_eq!(
            not(CarState {
                stopped: false,
                ..PARKED
            }),
            NotNow::Driving
        );
        assert_eq!(
            not(CarState {
                armed: true,
                ..PARKED
            }),
            NotNow::Armed
        );
        assert_eq!(
            not(CarState {
                throttle: 0.01,
                ..PARKED
            }),
            NotNow::Throttle
        );
        assert_eq!(
            not(CarState {
                brake: 0.5,
                ..PARKED
            }),
            NotNow::NotBraked
        );
        assert_eq!(
            not(CarState {
                brake: f32::NAN,
                ..PARKED
            }),
            NotNow::NotBraked
        );
    }

    #[test]
    fn the_password_must_be_set_up_right_and_wrong_ones_lock_out() {
        let example = "example-update-password";
        assert_eq!(
            Guard::new("short", example).check(b"short", 0),
            Err(PasswordError::NotSetUp)
        );
        assert_eq!(
            Guard::new(example, example).check(example.as_bytes(), 0),
            Err(PasswordError::NotSetUp)
        );
        let mut g = Guard::new("a-long-enough-one", example);
        assert_eq!(g.check(b"a-long-enough-one", 0), Ok(()));
        assert_eq!(
            g.check(b"a-long-enough-onf", 100),
            Err(PasswordError::Wrong)
        );
        // Even the right one waits out the lockout.
        assert_eq!(
            g.check(b"a-long-enough-one", 5_100),
            Err(PasswordError::LockedOut { ms_left: 5_000 })
        );
        assert_eq!(g.check(b"a-long-enough-one", 10_100), Ok(()));
        assert_eq!(g.check(b"", 10_200), Err(PasswordError::Wrong));
        // Across the clock's wrap.
        let mut w = Guard::new("a-long-enough-one", example);
        assert_eq!(w.check(b"x", u32::MAX - 10), Err(PasswordError::Wrong));
        assert!(matches!(
            w.check(b"a-long-enough-one", 100),
            Err(PasswordError::LockedOut { .. })
        ));
    }
}
