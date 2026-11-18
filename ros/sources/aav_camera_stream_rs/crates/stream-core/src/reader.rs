//! A thread that keeps a decoder running and reads it as fast as frames come (`reader.py`).
//!
//! Each frame goes into a [`Latest`] slot with its arrival time (wall clock, ns). When the
//! stream ends (the Pi restarts, the network drops), the decoder stops and starts again
//! after a pause that grows from `retry` to `retry_max` s, so a missing camera costs little.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::latest::{next_pause, Latest};
use crate::pipe::{PipeSource, Stopper};

/// A picture and when it arrived (wall clock, ns since 1970).
pub type Arrival = (Vec<u8>, i64);

/// The wall clock in ns since 1970, as Python's `time.time_ns()`.
pub fn wall_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| i64::try_from(d.as_nanos()).unwrap_or(i64::MAX))
}

/// What the reader thread and the node share.
#[derive(Default)]
struct Shared {
    connected: AtomicBool,
    frames: AtomicU64,
    starts: AtomicU64,
    stopping: Mutex<bool>,
    wake: Condvar,
    current: Mutex<Option<Stopper>>,
}

impl Shared {
    fn stopping(&self) -> bool {
        *self.stopping.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Wait `pause` s, or less if stopped meanwhile.
    fn wait(&self, pause: f64) {
        let stopping = self.stopping.lock().unwrap_or_else(PoisonError::into_inner);
        drop(
            self.wake
                .wait_timeout_while(stopping, Duration::from_secs_f64(pause), |s| !*s)
                .unwrap_or_else(PoisonError::into_inner),
        );
    }
}

/// Keeps a decoder running on its own thread.
pub struct Reader {
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
}

/// The pauses between decoder starts (s).
#[derive(Debug, Clone, Copy)]
pub struct Retry {
    pub first: f64,
    pub max: f64,
}

impl Default for Retry {
    fn default() -> Self {
        Retry {
            first: 1.0,
            max: 5.0,
        }
    }
}

impl Reader {
    /// Start reading: `make_source` makes a new decoder for each start; `log` says what
    /// happens.
    pub fn start(
        make_source: impl Fn() -> PipeSource + Send + 'static,
        latest: Arc<Latest<Arrival>>,
        retry: Retry,
        log: impl Fn(&str) + Send + 'static,
    ) -> std::io::Result<Self> {
        let shared = Arc::new(Shared::default());
        let me = Arc::clone(&shared);
        let thread = std::thread::Builder::new()
            .name("stream-reader".into())
            .spawn(move || run(&me, make_source, &latest, retry, log))?;
        Ok(Reader {
            shared,
            thread: Some(thread),
        })
    }

    pub fn connected(&self) -> bool {
        self.shared.connected.load(Ordering::Relaxed)
    }

    /// Frames read, all starts together.
    pub fn frames(&self) -> u64 {
        self.shared.frames.load(Ordering::Relaxed)
    }

    /// Decoder starts.
    pub fn starts(&self) -> u64 {
        self.shared.starts.load(Ordering::Relaxed)
    }

    /// Stop the decoder and the thread (waits for it).
    pub fn stop(&mut self) {
        *self
            .shared
            .stopping
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = true;
        self.shared.wake.notify_all();
        let current = self
            .shared
            .current
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone();
        if let Some(stopper) = current {
            stopper.stop(); // unblocks a read waiting on the pipe
        }
        if let Some(thread) = self.thread.take() {
            thread.join().ok();
        }
    }
}

impl Drop for Reader {
    fn drop(&mut self) {
        self.stop();
    }
}

fn run(
    shared: &Shared,
    make_source: impl Fn() -> PipeSource,
    latest: &Latest<Arrival>,
    retry: Retry,
    log: impl Fn(&str),
) {
    let mut pause = retry.first;
    while !shared.stopping() {
        let mut source = make_source();
        *shared
            .current
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = Some(source.stopper());
        shared.starts.fetch_add(1, Ordering::Relaxed);
        let mut got = 0u64;
        let outcome = source.start().and_then(|()| {
            while !shared.stopping() {
                let Some(frame) = source.read()? else {
                    break;
                };
                if !shared.connected.swap(true, Ordering::Relaxed) {
                    log("stream: pictures coming");
                }
                got += 1;
                shared.frames.fetch_add(1, Ordering::Relaxed);
                latest.put((frame, wall_ns()));
            }
            Ok(())
        });
        if let Err(e) = outcome {
            log(&format!("stream: {e}"));
        }
        source.stop();
        *shared
            .current
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = None;
        if shared.stopping() {
            break;
        }
        if shared.connected.swap(false, Ordering::Relaxed) {
            log("stream: ended; trying again");
        }
        pause = next_pause(pause, got > 0, retry.first, retry.max);
        shared.wait(pause);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn frames_arrive_and_a_dead_decoder_is_started_again() {
        let latest = Arc::new(Latest::default());
        let env: crate::decoders::Env = std::env::vars().collect();
        let make = move || {
            let argv = ["sh", "-c", "printf 'abcdef'"].map(String::from).to_vec();
            PipeSource::new(argv, 2, 1, env.clone())
        };
        let retry = Retry {
            first: 0.05,
            max: 0.2,
        };
        let mut reader = Reader::start(make, Arc::clone(&latest), retry, |_| {}).expect("a thread");
        let start = Instant::now();
        while reader.starts() < 3 && start.elapsed() < Duration::from_secs(5) {
            std::thread::sleep(Duration::from_millis(10));
        }
        reader.stop();
        assert!(reader.starts() >= 3);
        assert!(reader.frames() >= 3);
        assert_eq!(
            latest.take(Duration::ZERO).map(|(f, _)| f),
            Some(b"abcdef".to_vec())
        );
    }
}
