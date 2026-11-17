//! Lockstep with the cameras: the simulated time waits for a slow picture.
//!
//! With a GPU the cameras keep up, and the world runs in real time. With a software
//! renderer (Mesa's llvmpipe, a computer with no GPU, as CI's runners) one round of
//! pictures takes longer than its 0.1 s, and a free-running world would leave the cameras
//! skipping poses: 1.6 Hz on CI. Gazebo draws its sensors in lockstep; this is ours.
//!
//! ```text
//! cameras ──"free until t" (the next picture's sim time)──> CameraGate
//! simulator: at each pose it publishes for the cameras (the truth), stamped `now`:
//!     now < t            go on
//!     now >= t           wait (the clock holds) until the cameras say a later t
//!     no word for `timeout` (wall time): go on, and run free until they speak again
//! ```
//!
//! Before the cameras first speak (Bevy still starting), the world runs free.

use std::sync::{Condvar, Mutex, PoisonError};
use std::time::{Duration, Instant};

/// How a wait ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Waited {
    /// No wait: the cameras are free past this time, or haven't spoken yet.
    Free,
    /// The cameras answered after this long (wall time).
    Answered(Duration),
    /// No answer within the timeout: the gate runs free until they speak again.
    TimedOut,
}

/// What the cameras last said, in sim-time nanoseconds; `None`: run free.
#[derive(Debug, Default)]
pub struct CameraGate {
    free_until: Mutex<Option<u64>>,
    spoke: Condvar,
    timeout: Duration,
}

impl CameraGate {
    /// A gate that waits at most `timeout` (wall time) for each answer.
    #[must_use]
    pub fn new(timeout: Duration) -> Self {
        Self {
            free_until: Mutex::new(None),
            spoke: Condvar::new(),
            timeout,
        }
    }

    /// The cameras' word: nothing is due before `ns` (sim time). A smaller value than the
    /// last (the simulation restarted) is taken as it is.
    pub fn free_until(&self, ns: u64) {
        *self
            .free_until
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = Some(ns);
        self.spoke.notify_all();
    }

    /// Called as the simulator publishes a pose stamped `now_ns`: returns once the
    /// cameras are free past it, or they don't answer within the timeout.
    pub fn wait_past(&self, now_ns: u64) -> Waited {
        let start = Instant::now();
        let mut free = self
            .free_until
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if free.is_none_or(|t| now_ns < t) {
            return Waited::Free;
        }
        loop {
            let left = self.timeout.saturating_sub(start.elapsed());
            if left.is_zero() {
                *free = None;
                return Waited::TimedOut;
            }
            free = self
                .spoke
                .wait_timeout(free, left)
                .unwrap_or_else(PoisonError::into_inner)
                .0;
            match *free {
                None => return Waited::Free,
                Some(t) if now_ns < t => return Waited::Answered(start.elapsed()),
                Some(_) => {}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    const MS: u64 = 1_000_000;

    #[test]
    fn runs_free_before_the_cameras_speak() {
        let gate = CameraGate::new(Duration::from_millis(10));
        assert_eq!(gate.wait_past(500 * MS), Waited::Free);
    }

    #[test]
    fn goes_on_before_the_next_picture() {
        let gate = CameraGate::new(Duration::from_millis(10));
        gate.free_until(100 * MS);
        assert_eq!(gate.wait_past(80 * MS), Waited::Free);
    }

    #[test]
    fn waits_at_the_picture_until_the_cameras_answer() {
        let gate = Arc::new(CameraGate::new(Duration::from_secs(5)));
        gate.free_until(100 * MS);
        let cameras = Arc::clone(&gate);
        let answer = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(30));
            cameras.free_until(200 * MS);
        });
        match gate.wait_past(100 * MS) {
            Waited::Answered(d) => assert!(d >= Duration::from_millis(20), "{d:?}"),
            other => panic!("{other:?}"),
        }
        answer.join().unwrap();
        assert_eq!(gate.wait_past(120 * MS), Waited::Free);
    }

    #[test]
    fn an_answer_too_early_keeps_it_waiting() {
        let gate = Arc::new(CameraGate::new(Duration::from_millis(100)));
        gate.free_until(100 * MS);
        let cameras = Arc::clone(&gate);
        std::thread::spawn(move || cameras.free_until(100 * MS));
        assert_eq!(gate.wait_past(100 * MS), Waited::TimedOut);
    }

    #[test]
    fn silent_cameras_time_out_then_run_free() {
        let gate = CameraGate::new(Duration::from_millis(20));
        gate.free_until(100 * MS);
        assert_eq!(gate.wait_past(100 * MS), Waited::TimedOut);
        assert_eq!(gate.wait_past(120 * MS), Waited::Free);
        gate.free_until(200 * MS);
        assert_eq!(gate.wait_past(140 * MS), Waited::Free);
    }
}
