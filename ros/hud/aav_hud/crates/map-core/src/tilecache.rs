//! The street map's decoded tiles: which to draw this frame, which to ask the loader
//! for, and which to forget.
//!
//! The loader (a thread in map-app) takes tile ids from a queue and sends back decoded
//! tiles. Each frame, [`TileCache::frame`] gets the tiles the view wants and:
//!
//! - draws the ones it has; for one it hasn't yet, its nearest ancestor it has (a
//!   blurrier tile for a moment instead of a hole);
//! - queues the missing ones, the middle of the view first, and takes out of the queue
//!   what the view no longer wants (zooming fast leaves no backlog);
//! - forgets the least recently drawn tiles past [`TileCache::capacity`].

use std::{
    collections::{HashMap, HashSet, VecDeque},
    sync::Arc,
};

use crate::{streets::TileShapes, tiles::TileId};

/// How many zooms up to look for a stand-in tile.
const ANCESTORS: usize = 4;

#[derive(Debug, Clone)]
enum Slot {
    /// Decoded: the shapes, and the frame it was last drawn.
    Ready(Arc<TileShapes>, u64),
    /// Not in the file (outside the extract), or it couldn't be decoded.
    Missing(u64),
}

/// The decoded tiles, and the ones asked for.
#[derive(Debug, Clone)]
pub struct TileCache {
    slots: HashMap<TileId, Slot>,
    /// Queued or being decoded.
    asked: HashSet<TileId>,
    frame: u64,
    /// Tiles kept at most (a zoom-15 tile of Ottawa holds up to ~100k points).
    pub capacity: usize,
}

impl Default for TileCache {
    fn default() -> Self {
        Self {
            slots: HashMap::new(),
            asked: HashSet::new(),
            frame: 0,
            capacity: 160,
        }
    }
}

impl TileCache {
    /// A tile from the loader: its shapes, or None when it isn't in the file or can't be
    /// decoded.
    pub fn insert(&mut self, id: TileId, shapes: Option<TileShapes>) {
        self.asked.remove(&id);
        let slot = match shapes {
            Some(s) => Slot::Ready(Arc::new(s), self.frame),
            None => Slot::Missing(self.frame),
        };
        self.slots.insert(id, slot);
    }

    /// One frame: `wanted` (middle first) in, the tiles to draw out. `queue` is the
    /// loader's: missing tiles go in, unwanted ones come out.
    pub fn frame(
        &mut self,
        wanted: &[TileId],
        queue: &mut VecDeque<TileId>,
    ) -> Vec<Arc<TileShapes>> {
        self.frame += 1;
        let now = self.frame;
        let wanted_set: HashSet<TileId> = wanted.iter().copied().collect();
        // What the view no longer wants leaves the queue (not the one being decoded).
        queue.retain(|id| {
            let keep = wanted_set.contains(id);
            if !keep {
                self.asked.remove(id);
            }
            keep
        });
        let mut draw: Vec<TileId> = Vec::new();
        for &id in wanted {
            match self.slots.get_mut(&id) {
                Some(Slot::Ready(_, used)) => {
                    *used = now;
                    draw.push(id);
                }
                Some(Slot::Missing(used)) => *used = now,
                None => {
                    if self.asked.insert(id) {
                        queue.push_back(id);
                    }
                    let stand_in = std::iter::successors(id.parent(), |p| p.parent())
                        .take(ANCESTORS)
                        .find(|p| matches!(self.slots.get(p), Some(Slot::Ready(..))));
                    if let Some(p) = stand_in {
                        if let Some(Slot::Ready(_, used)) = self.slots.get_mut(&p) {
                            *used = now;
                        }
                        draw.push(p);
                    }
                }
            }
        }
        // A stand-in for several tiles is drawn once; stand-ins first, under the
        // sharper tiles.
        draw.sort_by_key(|id| id.z);
        draw.dedup();
        self.forget_old(&wanted_set);
        draw.iter()
            .filter_map(|id| match self.slots.get(id) {
                Some(Slot::Ready(s, _)) => Some(Arc::clone(s)),
                _ => None,
            })
            .collect()
    }

    /// How many tiles are kept.
    #[must_use]
    pub fn len(&self) -> usize {
        self.slots.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }

    fn forget_old(&mut self, wanted: &HashSet<TileId>) {
        let extra = self.slots.len().saturating_sub(self.capacity);
        if extra == 0 {
            return;
        }
        let mut by_age: Vec<(u64, TileId)> = self
            .slots
            .iter()
            .filter(|(id, _)| !wanted.contains(id))
            .map(|(&id, slot)| match slot {
                Slot::Ready(_, used) | Slot::Missing(used) => (*used, id),
            })
            .collect();
        by_age.sort();
        for (_, id) in by_age.into_iter().take(extra) {
            self.slots.remove(&id);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(z: u8, x: u32, y: u32) -> TileId {
        TileId::new(z, x, y)
    }

    #[test]
    fn missing_tiles_are_asked_for_once_and_drawn_when_they_come() {
        let mut c = TileCache::default();
        let mut q = VecDeque::new();
        let want = [t(15, 10, 10), t(15, 11, 10)];
        assert!(c.frame(&want, &mut q).is_empty());
        assert_eq!(q, want);
        // Still waiting: not asked twice.
        c.frame(&want, &mut q);
        assert_eq!(q.len(), 2);
        // The loader takes both; one comes back, one isn't in the file.
        q.clear();
        c.insert(want[0], Some(TileShapes::default()));
        c.insert(want[1], None);
        assert_eq!(c.frame(&want, &mut q).len(), 1);
        assert!(q.is_empty());
    }

    #[test]
    fn a_parent_stands_in_until_the_tile_comes() {
        let mut c = TileCache::default();
        let mut q = VecDeque::new();
        c.insert(t(14, 5, 5), Some(TileShapes::default()));
        // Four children of 14/5/5: the parent is drawn once for all four.
        let kids = [t(15, 10, 10), t(15, 11, 10), t(15, 10, 11), t(15, 11, 11)];
        assert_eq!(c.frame(&kids, &mut q).len(), 1);
        assert_eq!(q.len(), 4);
    }

    #[test]
    fn the_queue_drops_what_the_view_left() {
        let mut c = TileCache::default();
        let mut q = VecDeque::new();
        c.frame(&[t(15, 1, 1), t(15, 2, 2)], &mut q);
        c.frame(&[t(13, 0, 0)], &mut q);
        assert_eq!(q, [t(13, 0, 0)]);
        // Wanted again later: asked for again.
        c.frame(&[t(15, 1, 1)], &mut q);
        assert_eq!(q, [t(15, 1, 1)]);
    }

    #[test]
    fn old_tiles_are_forgotten_first() {
        let mut c = TileCache {
            capacity: 2,
            ..TileCache::default()
        };
        let mut q = VecDeque::new();
        for x in 0..3 {
            c.insert(t(15, x, 0), Some(TileShapes::default()));
            c.frame(&[t(15, x, 0)], &mut q);
        }
        assert_eq!(c.len(), 2);
        // The first one went.
        q.clear();
        c.frame(&[t(15, 0, 0)], &mut q);
        assert_eq!(q, [t(15, 0, 0)]);
    }
}
