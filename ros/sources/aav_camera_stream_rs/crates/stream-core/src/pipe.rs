//! The decoder as a child program writing raw BGR pictures to a pipe (`PipeSource`).
//!
//! It runs in its own process group, so stopping it reaches its children too (gst's,
//! ffmpeg's): SIGTERM to the group, then SIGKILL after 2 s.

use std::io::{ErrorKind, Read};
use std::os::unix::process::CommandExt;
use std::process::{Child, ChildStdout, Command, Stdio};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use crate::decoders::Env;
use crate::{Error, Result};

/// Stops a running decoder from any thread.
#[derive(Debug, Clone, Default)]
pub struct Stopper {
    child: Arc<Mutex<Option<Child>>>,
}

impl Stopper {
    /// Stop the decoder's process group: SIGTERM, then SIGKILL. Unblocks a read waiting
    /// on its pipe. Safe to call twice, or from two threads at once.
    pub fn stop(&self) {
        let child = self
            .child
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take();
        let Some(mut child) = child else {
            return;
        };
        let Ok(group) = libc::pid_t::try_from(child.id()) else {
            return;
        };
        for signal in [libc::SIGTERM, libc::SIGKILL] {
            if matches!(child.try_wait(), Ok(Some(_))) {
                break;
            }
            // SAFETY: killpg only sends a signal; the group is the child's own (its pid,
            // from process_group(0)), not reaped yet since we hold the Child.
            let sent = unsafe { libc::killpg(group, signal) };
            if sent != 0 {
                // Gone already. macOS says EPERM, not ESRCH, for a group whose last
                // member has exited but not been reaped.
                break;
            }
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(2) {
                if matches!(child.try_wait(), Ok(Some(_))) {
                    break;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        }
        child.wait().ok();
    }
}

/// A decoder program's pipe: [`PipeSource::read`] returns the next picture.
#[derive(Debug)]
pub struct PipeSource {
    argv: Vec<String>,
    size: usize,
    env: Env,
    stdout: Option<ChildStdout>,
    stopper: Stopper,
}

impl PipeSource {
    /// A decoder whose pictures are `width` x `height` BGR.
    pub fn new(argv: Vec<String>, width: u32, height: u32, env: Env) -> Self {
        PipeSource {
            argv,
            size: width as usize * height as usize * 3,
            env,
            stdout: None,
            stopper: Stopper::default(),
        }
    }

    /// Start the program, in its own process group, stdin from /dev/null.
    pub fn start(&mut self) -> Result<()> {
        let (program, args) = self
            .argv
            .split_first()
            .ok_or_else(|| Error::Io("the decoder has no command".to_string()))?;
        let mut child = Command::new(program)
            .args(args)
            .env_clear()
            .envs(&self.env)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .process_group(0)
            .spawn()
            .map_err(|e| Error::Io(format!("{program}: {e}")))?;
        self.stdout = child.stdout.take();
        *self
            .stopper
            .child
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = Some(child);
        Ok(())
    }

    /// The handle that stops this decoder from another thread.
    pub fn stopper(&self) -> Stopper {
        self.stopper.clone()
    }

    /// The next picture (height x width x 3 bytes), or None at the end of the stream.
    pub fn read(&mut self) -> Result<Option<Vec<u8>>> {
        let Some(stdout) = self.stdout.as_mut() else {
            return Ok(None);
        };
        let mut frame = vec![0u8; self.size];
        match stdout.read_exact(&mut frame) {
            Ok(()) => Ok(Some(frame)),
            Err(e) if e.kind() == ErrorKind::UnexpectedEof => Ok(None),
            Err(e) => Err(Error::Io(e.to_string())),
        }
    }

    pub fn stop(&mut self) {
        self.stopper.stop();
        self.stdout = None;
    }
}

impl Drop for PipeSource {
    fn drop(&mut self) {
        self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn env() -> Env {
        std::env::vars().collect()
    }

    #[test]
    fn whole_pictures_come_then_the_end() {
        // 2 x 1 pictures: 6 bytes each; 13 bytes is two and a half.
        let argv = ["sh", "-c", "printf 'abcdefghijklm'"]
            .map(String::from)
            .to_vec();
        let mut source = PipeSource::new(argv, 2, 1, env());
        source.start().expect("sh runs");
        assert_eq!(source.read().ok().flatten(), Some(b"abcdef".to_vec()));
        assert_eq!(source.read().ok().flatten(), Some(b"ghijkl".to_vec()));
        assert_eq!(source.read().ok().flatten(), None);
    }

    #[test]
    fn stop_ends_a_program_that_never_writes() {
        let argv = ["sh", "-c", "sleep 30"].map(String::from).to_vec();
        let mut source = PipeSource::new(argv, 2, 1, env());
        source.start().expect("sh runs");
        let stopper = source.stopper();
        let t = std::thread::spawn(move || source.read().ok().flatten());
        let start = Instant::now();
        std::thread::sleep(Duration::from_millis(100));
        stopper.stop();
        assert_eq!(t.join().ok().flatten(), None);
        assert!(start.elapsed() < Duration::from_secs(5));
    }

    #[test]
    fn a_missing_program_is_an_error() {
        let argv = vec!["/no/such/decoder".to_string()];
        assert!(PipeSource::new(argv, 2, 1, env()).start().is_err());
    }
}
