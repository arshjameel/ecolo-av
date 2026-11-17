//! Clouds waiting for their transform: tf2's `MessageFilter`, in short, with no ROS.
//!
//! A cloud may come a little before /tf has the car's pose at its stamp. Keep the cloud,
//! oldest first, and use it once the transform is there. Never block a callback to wait:
//! in a one-thread node, /tf can't arrive while it waits.
//!
//! | The cloud's stamp | Found | Not found |
//! |---|---|---|
//! | at or before the newest transform | [`Step::AtStamp`] | [`Step::Drop`] (too old) |
//! | at most `tolerance` past it | | [`Step::Newest`]: the newest pose stands in |
//! | later | | [`Step::Wait`]; [`Step::Drop`] once older than `max_wait` |

use std::collections::VecDeque;

/// What to do with one waiting cloud.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// The transform at its stamp is there.
    AtStamp,
    /// A hair newer than /tf: the newest transform stands in.
    Newest,
    /// Too new: ask again later.
    Wait,
    /// Never coming: too old for the buffer, or waited too long.
    Drop,
}

/// How long a cloud may wait.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WaitShape {
    /// s: a stamp this much newer than the newest transform uses it.
    pub tolerance: f64,
    /// s on the node's clock: a cloud this old is dropped.
    pub max_wait: f64,
    /// Clouds kept waiting (3 s of the 10 Hz cubes); more: the oldest goes.
    pub size: usize,
}

impl Default for WaitShape {
    fn default() -> Self {
        Self {
            tolerance: 0.1,
            max_wait: 1.5,
            size: 30,
        }
    }
}

/// What the TF buffer says about one cloud's stamp.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Look {
    /// The transform at the stamp is there.
    pub found: bool,
    /// s: the newest transform's time, `None` if none yet.
    pub newest: Option<f64>,
}

/// The step for one cloud stamped `stamp` (s), the node's clock at `now` (s).
#[must_use]
pub fn decide(stamp: f64, look: Look, now: f64, shape: &WaitShape) -> Step {
    if look.found {
        return Step::AtStamp;
    }
    match look.newest {
        // Older than the buffer, or no path between.
        Some(newest) if stamp <= newest => Step::Drop,
        Some(newest) if stamp - newest <= shape.tolerance => Step::Newest,
        _ if now - stamp > shape.max_wait => Step::Drop,
        _ => Step::Wait,
    }
}

/// One stream's clouds, oldest first, each used once its transform is there.
#[derive(Clone, Debug)]
pub struct Waiting<T> {
    shape: WaitShape,
    queue: VecDeque<(f64, T)>,
    /// Dropped since the node last said so.
    pub dropped: u64,
}

impl<T> Waiting<T> {
    /// Nothing waiting.
    #[must_use]
    pub fn new(shape: WaitShape) -> Self {
        Self {
            shape,
            queue: VecDeque::new(),
            dropped: 0,
        }
    }

    /// How many clouds wait.
    #[must_use]
    pub fn len(&self) -> usize {
        self.queue.len()
    }

    /// Whether none waits.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    /// The waiting clouds' stamps, oldest first.
    pub fn stamps(&self) -> impl Iterator<Item = f64> + '_ {
        self.queue.iter().map(|(s, _)| *s)
    }

    /// Keep a cloud; past `size`, the oldest goes.
    pub fn add(&mut self, stamp: f64, item: T) {
        self.queue.push_back((stamp, item));
        while self.queue.len() > self.shape.size {
            self.queue.pop_front();
            self.dropped += 1;
        }
    }

    /// The clouds ready now, oldest first, each with [`Step::AtStamp`] or
    /// [`Step::Newest`]. `look(stamp, item)` asks the TF buffer. Stops at the first cloud
    /// still waiting: those after it are newer.
    pub fn take(&mut self, mut look: impl FnMut(f64, &T) -> Look, now: f64) -> Vec<(T, Step)> {
        let mut ready = Vec::new();
        while let Some((stamp, item)) = self.queue.front() {
            let step = decide(*stamp, look(*stamp, item), now, &self.shape);
            if step == Step::Wait {
                break;
            }
            let Some((_, item)) = self.queue.pop_front() else {
                break;
            };
            if step == Step::Drop {
                self.dropped += 1;
            } else {
                ready.push((item, step));
            }
        }
        ready
    }
}

impl<T> Default for Waiting<T> {
    fn default() -> Self {
        Self::new(WaitShape::default())
    }
}
