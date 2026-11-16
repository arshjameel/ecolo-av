//! Lines out a slow serial port, a few bytes a tick, so writing never blocks the tick.

use crate::{Line, TooLong, MAX_LINE};
use core::fmt::{self, Display, Write};

/// `fmt::Write` into a slot; an error once it is full.
struct SlotWriter<'b> {
    buf: &'b mut [u8],
    len: usize,
}

impl Write for SlotWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let end = self.len.checked_add(s.len()).ok_or(fmt::Error)?;
        let dest = self.buf.get_mut(self.len..end).ok_or(fmt::Error)?;
        dest.copy_from_slice(s.as_bytes());
        self.len = end;
        Ok(())
    }
}

/// A queue of lines for a serial log, handed out a chunk at a time.
///
/// The ESP32's log (the USB cable, 115200 baud) takes about 11 bytes a millisecond, and its
/// print waits, with interrupts off, while the UART's 128-byte FIFO is full. A 560-byte
/// status line printed at once would hold the tick for about 50 ms. A chunk of at most
/// the bytes the FIFO drains in one tick never waits.
///
/// `SLOTS` lines wait at most, each with its newline. A line that finds the queue full is
/// dropped whole and counted; a reader sees the gap in `seq`.
#[derive(Debug)]
pub struct Trickle<const SLOTS: usize = 4, const N: usize = { MAX_LINE + 1 }> {
    slots: [Slot<N>; SLOTS],
    /// The slot being sent, and how many slots hold a line.
    head: usize,
    queued: usize,
    /// Bytes of the head's line already handed out.
    sent: usize,
    dropped: u32,
}

#[derive(Debug, Clone, Copy)]
struct Slot<const N: usize> {
    buf: [u8; N],
    len: usize,
}

/// The queue was full, or the line too long: it was dropped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Dropped;

impl From<TooLong> for Dropped {
    fn from(_: TooLong) -> Self {
        Dropped
    }
}

impl<const SLOTS: usize, const N: usize> Trickle<SLOTS, N> {
    #[must_use]
    pub const fn new() -> Self {
        Trickle {
            slots: [Slot {
                buf: [0; N],
                len: 0,
            }; SLOTS],
            head: 0,
            queued: 0,
            sent: 0,
            dropped: 0,
        }
    }

    /// Queue `line`, with a newline.
    pub fn push(&mut self, line: &Line<'_>) -> Result<(), Dropped> {
        self.push_text(line)
    }

    /// Queue any one line of text (a text log line, say), with a newline. It must hold no
    /// newline of its own; one too long for a slot is dropped whole, as with [`push`].
    ///
    /// [`push`]: Trickle::push
    pub fn push_text(&mut self, text: &impl Display) -> Result<(), Dropped> {
        if self.queued == SLOTS || N == 0 {
            self.dropped = self.dropped.saturating_add(1);
            return Err(Dropped);
        }
        let slot = &mut self.slots[(self.head + self.queued) % SLOTS];
        let mut w = SlotWriter {
            buf: &mut slot.buf[..N - 1],
            len: 0,
        };
        if write!(w, "{text}").is_err() {
            self.dropped = self.dropped.saturating_add(1);
            return Err(Dropped);
        }
        let len = w.len;
        slot.buf[len] = b'\n';
        slot.len = len + 1;
        self.queued += 1;
        Ok(())
    }

    /// The next at most `max` bytes to send; `None` when nothing waits. A chunk never
    /// spans two lines.
    pub fn next_chunk(&mut self, max: usize) -> Option<&[u8]> {
        if self.queued == 0 || max == 0 {
            return None;
        }
        let head = self.head;
        let len = self.slots[head].len;
        let start = self.sent;
        let end = len.min(start.saturating_add(max));
        if end == len {
            self.head = (self.head + 1) % SLOTS;
            self.queued -= 1;
            self.sent = 0;
        } else {
            self.sent = end;
        }
        Some(&self.slots[head].buf[start..end])
    }

    /// How many lines wait, the one being sent included.
    #[must_use]
    pub fn len(&self) -> usize {
        self.queued
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.queued == 0
    }

    /// Lines dropped since boot.
    #[must_use]
    pub fn dropped(&self) -> u32 {
        self.dropped
    }
}

impl<const SLOTS: usize, const N: usize> Default for Trickle<SLOTS, N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::{Body, Event, Value};
    use std::string::{String, ToString};
    use std::vec::Vec;

    fn event(seq: u32) -> Line<'static> {
        Line {
            seq,
            up_ms: 0,
            body: Body::Event(Event {
                what: "mode",
                from: Value::Text("rc"),
                to: Value::Text("computer"),
            }),
        }
    }

    fn drain<const S: usize, const N: usize>(t: &mut Trickle<S, N>, max: usize) -> Vec<String> {
        let mut chunks = Vec::new();
        while let Some(c) = t.next_chunk(max) {
            assert!(c.len() <= max);
            chunks.push(String::from_utf8(c.to_vec()).unwrap());
        }
        chunks
    }

    #[test]
    fn lines_come_out_whole_in_order_in_small_chunks() {
        let mut t: Trickle = Trickle::new();
        t.push(&event(1)).unwrap();
        t.push(&event(2)).unwrap();
        assert_eq!(t.len(), 2);
        let chunks = drain(&mut t, 16);
        let text: String = chunks.concat();
        assert_eq!(
            text,
            std::format!("{}\n{}\n", event(1), event(2)),
            "{chunks:?}"
        );
        // No chunk holds the end of one line and the start of the next.
        assert!(chunks.iter().all(|c| !c[..c.len() - 1].contains('\n')));
        assert!(t.is_empty());
    }

    #[test]
    fn a_full_queue_drops_the_new_line_whole_and_counts_it() {
        let mut t: Trickle<2> = Trickle::new();
        t.push(&event(1)).unwrap();
        t.push(&event(2)).unwrap();
        assert_eq!(t.push(&event(3)), Err(Dropped));
        assert_eq!(t.dropped(), 1);
        // Half way through the first: still full.
        t.next_chunk(10);
        assert_eq!(t.push(&event(4)), Err(Dropped));
        drain(&mut t, 1000);
        t.push(&event(5)).unwrap();
        assert_eq!(drain(&mut t, 1000), [event(5).to_string() + "\n"]);
    }

    #[test]
    fn a_line_too_long_for_a_slot_is_dropped() {
        let mut t: Trickle<2, 20> = Trickle::new();
        assert_eq!(t.push(&event(1)), Err(Dropped));
        assert!(t.is_empty());
        assert_eq!(t.dropped(), 1);
    }

    #[test]
    fn a_text_line_goes_out_whole_like_a_status_line() {
        let mut t: Trickle<2, 64> = Trickle::new();
        t.push_text(&format_args!("CONFIG t={} a={}", 5, 1.5))
            .unwrap();
        assert_eq!(drain(&mut t, 7).concat(), "CONFIG t=5 a=1.5\n");
        let long = "x".repeat(64);
        assert_eq!(t.push_text(&long), Err(Dropped));
        assert_eq!((t.dropped(), t.is_empty()), (1, true));
    }

    #[test]
    fn it_wraps_around_its_slots() {
        let mut t: Trickle<3> = Trickle::new();
        let mut out = String::new();
        for seq in 0..10 {
            t.push(&event(seq)).unwrap();
            out.extend(drain(&mut t, 50));
        }
        let want: String = (0..10).map(|s| event(s).to_string() + "\n").collect();
        assert_eq!(out, want);
    }
}
