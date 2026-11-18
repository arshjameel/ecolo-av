//! Acceptance filters: which frames the chip keeps (data sheet, section 4.5).
//!
//! RXB0 has one mask and two filters (RXF0, RXF1); RXB1 has one mask and four (RXF2 to
//! RXF5). With exact IDs, both masks compare all 11 ID bits, and nothing else: their extended
//! bits stay 0, or they would also compare a standard frame's first two data bytes (4.5.1).
//!
//! Every one of the six filters gets one of the wanted IDs. A filter left at its reset value
//! would let in ID 0x000, the highest-priority ID on any bus.

use crate::regs::{BUKT, RXM_ANY};
use embedded_can::StandardId;

/// The most IDs the six filters can hold.
pub const MAX_IDS: usize = 6;

/// Which frames reach the receive buffers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Filters(Kind);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// The filters off: every frame, standard or extended.
    All,
    /// Standard frames with one of these IDs, one a filter.
    Exact([StandardId; MAX_IDS]),
}

/// Exact filters need 1 to 6 IDs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterError {
    /// No ID given.
    NoIds,
    /// More IDs than the chip has filters.
    TooManyIds,
}

/// The register values for a set of filters: filters 0 to 2, filters 3 to 5, both masks,
/// RXB0CTRL and RXB1CTRL.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct FilterRegisters {
    pub rxf0_2: [u8; 12],
    pub rxf3_5: [u8; 12],
    pub masks: [u8; 8],
    pub rxb0ctrl: u8,
    pub rxb1ctrl: u8,
}

impl Filters {
    /// Every frame: for a bus monitor, or to look for traffic while debugging.
    #[must_use]
    pub const fn all() -> Filters {
        Filters(Kind::All)
    }

    /// Only standard (11-bit) frames with one of these IDs.
    ///
    /// RXB0 takes the first two IDs; RXB1 takes the rest, and the first ones again in its
    /// spare filters. Rollover moves a frame for RXB0 into RXB1 when RXB0 is full, so with
    /// two IDs, as the spine has, both buffers serve both.
    ///
    /// # Errors
    ///
    /// [`FilterError::NoIds`] for none, [`FilterError::TooManyIds`] for more than six.
    pub fn standard_ids(ids: &[StandardId]) -> Result<Filters, FilterError> {
        if ids.is_empty() {
            return Err(FilterError::NoIds);
        }
        if ids.len() > MAX_IDS {
            return Err(FilterError::TooManyIds);
        }
        let mut slots = [ids[0]; MAX_IDS];
        for (k, slot) in slots.iter_mut().enumerate() {
            *slot = ids[k % ids.len()];
        }
        Ok(Filters(Kind::Exact(slots)))
    }

    /// The ID each of the six filters holds; None with the filters off.
    #[must_use]
    pub fn ids(&self) -> Option<[StandardId; MAX_IDS]> {
        match self.0 {
            Kind::All => None,
            Kind::Exact(ids) => Some(ids),
        }
    }

    pub(crate) fn registers(&self) -> FilterRegisters {
        match self.0 {
            Kind::All => FilterRegisters {
                rxf0_2: [0; 12],
                rxf3_5: [0; 12],
                masks: [0; 8],
                rxb0ctrl: RXM_ANY | BUKT,
                rxb1ctrl: RXM_ANY,
            },
            Kind::Exact(ids) => {
                let (rxf0_2, rxf3_5) = ids.split_at(3);
                FilterRegisters {
                    rxf0_2: pack(rxf0_2),
                    rxf3_5: pack(rxf3_5),
                    // Both masks compare all 11 ID bits.
                    masks: pack(&[StandardId::MAX; 2]),
                    // RXM = 00: the filters decide, by each filter's EXIDE bit (standard here).
                    rxb0ctrl: BUKT,
                    rxb1ctrl: 0,
                }
            }
        }
    }
}

/// A filter's or mask's four bytes for a standard ID: SIDH, SIDL (EXIDE = 0), EID8, EID0.
fn standard(id: StandardId) -> [u8; 4] {
    let raw = id.as_raw();
    [(raw >> 3) as u8, ((raw & 0x07) as u8) << 5, 0, 0]
}

/// Consecutive filters' or masks' registers, in address order: four bytes each.
fn pack<const N: usize>(ids: &[StandardId]) -> [u8; N] {
    let mut bytes = [0; N];
    for (chunk, &id) in bytes.as_chunks_mut::<4>().0.iter_mut().zip(ids) {
        *chunk = standard(id);
    }
    bytes
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    fn ids(raw: &[u16]) -> std::vec::Vec<StandardId> {
        raw.iter().map(|&r| StandardId::new(r).unwrap()).collect()
    }

    #[test]
    fn the_spines_two_ids() {
        // ESTOP_CMD 0x010 and DRIVE_CMD 0x100 (firmware/can/aav.dbc).
        let f = Filters::standard_ids(&ids(&[0x010, 0x100])).unwrap();
        let r = f.registers();
        let estop = [0x02, 0x00, 0, 0];
        let drive = [0x20, 0x00, 0, 0];
        assert_eq!(r.rxf0_2, [estop, drive, estop].concat().as_slice());
        assert_eq!(r.rxf3_5, [drive, estop, drive].concat().as_slice());
        assert_eq!(r.masks, [0xFF, 0xE0, 0, 0, 0xFF, 0xE0, 0, 0]);
        assert_eq!((r.rxb0ctrl, r.rxb1ctrl), (0x04, 0x00));
    }

    #[test]
    fn every_filter_holds_a_wanted_id() {
        for n in 1..=MAX_IDS {
            let wanted = ids(&[0x101, 0x102, 0x103, 0x104, 0x105, 0x106][..n]);
            let slots = Filters::standard_ids(&wanted).unwrap().ids().unwrap();
            assert!(slots.iter().all(|id| wanted.contains(id)), "{n}");
            // Each ID is in at least one filter, and RXB0's two are the first ones.
            assert!(wanted.iter().all(|id| slots.contains(id)), "{n}");
            assert_eq!(slots[0], wanted[0]);
        }
    }

    #[test]
    fn one_to_six_ids() {
        assert_eq!(Filters::standard_ids(&[]), Err(FilterError::NoIds));
        let seven = ids(&[1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(Filters::standard_ids(&seven), Err(FilterError::TooManyIds));
    }

    #[test]
    fn all_turns_the_filters_off() {
        let r = Filters::all().registers();
        assert_eq!((r.rxb0ctrl, r.rxb1ctrl), (0x64, 0x60));
        assert_eq!(Filters::all().ids(), None);
    }
}
