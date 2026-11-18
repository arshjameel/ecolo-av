//! The bit timing: how the MCP2515 divides each bit into time quanta, from the module's
//! crystal and the bus's bitrate.
//!
//! The choice copies Linux's `can_calc_bittiming()` (drivers/net/can/dev/calc_bittiming.c),
//! with the MCP2515's limits from Linux's `mcp251x` driver. So the spine samples each bit
//! where a SocketCAN node on the same bus does: 87.5 % of the bit up to 500 kbit/s, as CiA
//! recommends. Only exact bitrates are accepted: both ends of a bus must agree.
//!
//! | Rule | Where it comes from |
//! |---|---|
//! | TQ = 2 x (BRP + 1) / Fosc, BRP 0 to 63 | data sheet, Register 5-1 |
//! | PropSeg 1-8 TQ, PS1 1-8, PS2 2-8, SJW 1-4 | data sheet, sections 5.2.2, 5.2.3 and 5.2.6 |
//! | PropSeg + PS1 >= PS2, and PS2 > SJW | data sheet, section 5.5 |
//! | the sample point, and SJW = max(1, min(PS1, PS2 / 2)) | Linux's `can_calc_bittiming()` and `can_sjw_set_default()` |

/// The oscillator on the MCP2515 module: read it off the crystal's can ("8.000" or "16.000"
/// on most modules).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Crystal(u32);

impl Crystal {
    /// An 8 MHz crystal.
    pub const MHZ_8: Crystal = Crystal(8_000_000);
    /// A 16 MHz crystal.
    pub const MHZ_16: Crystal = Crystal(16_000_000);

    /// A crystal of `hz`.
    #[must_use]
    pub const fn from_hz(hz: u32) -> Crystal {
        Crystal(hz)
    }

    /// Its frequency (Hz).
    #[must_use]
    pub const fn hz(self) -> u32 {
        self.0
    }
}

/// The bus's bitrate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Bitrate(u32);

impl Bitrate {
    /// 125 kbit/s.
    pub const KBPS_125: Bitrate = Bitrate(125_000);
    /// 250 kbit/s.
    pub const KBPS_250: Bitrate = Bitrate(250_000);
    /// The car's bus (`firmware/can/aav.dbc`).
    pub const KBPS_500: Bitrate = Bitrate(500_000);
    /// 1 Mbit/s.
    pub const MBPS_1: Bitrate = Bitrate(1_000_000);

    /// A bitrate of `bps` bits a second.
    #[must_use]
    pub const fn from_bps(bps: u32) -> Bitrate {
        Bitrate(bps)
    }

    /// Bits a second.
    #[must_use]
    pub const fn bps(self) -> u32 {
        self.0
    }
}

/// No setting of the MCP2515 gives exactly this bitrate from this crystal. An 8 MHz crystal,
/// for example, can't make 1 Mbit/s: a bit needs at least 6 time quanta of 250 ns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimingError;

/// One bit, in time quanta (TQ): the sync segment (always 1 TQ), the propagation segment
/// and phase segment 1, then the sample point, then phase segment 2.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BitTiming {
    /// The prescaler: one TQ is `brp` periods of Fosc / 2 (1 to 64).
    brp: u8,
    prop_seg: u8,
    phase_seg1: u8,
    phase_seg2: u8,
    sjw: u8,
}

// The MCP2515's limits, as Linux's mcp251x driver gives them.
const TSEG1: (u8, u8) = (3, 16); // PropSeg + PS1
const TSEG2: (u8, u8) = (2, 8); // PS2
const BRP_MAX: u32 = 64;

impl BitTiming {
    /// The timing for `bitrate` from `crystal`.
    ///
    /// # Errors
    ///
    /// [`TimingError`] if no setting gives exactly that bitrate.
    pub fn new(crystal: Crystal, bitrate: Bitrate) -> Result<BitTiming, TimingError> {
        let clock = crystal.hz() / 2; // the TQ clock before the prescaler
        let target = sample_point_target(bitrate);
        let min_tq = 1 + TSEG1.0 + TSEG2.0;
        let max_tq = 1 + TSEG1.1 + TSEG2.1;
        // Most quanta first, as Linux does: on a tie, the finer resolution wins.
        let (tq, brp, tseg1, tseg2, _) = (min_tq..=max_tq)
            .rev()
            .filter_map(|tq| {
                let per_bit = u32::from(tq).checked_mul(bitrate.bps())?;
                let brp =
                    (per_bit != 0 && clock.is_multiple_of(per_bit)).then(|| clock / per_bit)?;
                let (tseg1, tseg2, error) = split(tq, target)?;
                (1..=BRP_MAX)
                    .contains(&brp)
                    .then_some((tq, brp, tseg1, tseg2, error))
            })
            .fold(None, |best: Option<(u8, u32, u8, u8, u16)>, c| match best {
                Some(b) if b.4 <= c.4 => Some(b),
                _ => Some(c),
            })
            .ok_or(TimingError)?;
        debug_assert_eq!(1 + tseg1 + tseg2, tq);
        let prop_seg = tseg1 / 2;
        let phase_seg1 = tseg1 - prop_seg;
        Ok(BitTiming {
            brp: brp as u8, // 1 to 64, checked above
            prop_seg,
            phase_seg1,
            phase_seg2: tseg2,
            sjw: phase_seg1.min(tseg2 / 2).max(1),
        })
    }

    /// CNF3, CNF2 and CNF1, in address order (0x28 to 0x2A). PS2 is set in CNF3 (BTLMODE),
    /// each bit is sampled once (SAM = 0), and the CLKOUT pin's SOF signal is off.
    #[must_use]
    pub fn registers(&self) -> [u8; 3] {
        let cnf1 = ((self.sjw - 1) << 6) | (self.brp - 1);
        let cnf2 = 0x80 | ((self.phase_seg1 - 1) << 3) | (self.prop_seg - 1);
        let cnf3 = self.phase_seg2 - 1;
        [cnf3, cnf2, cnf1]
    }

    /// Time quanta in one bit.
    #[must_use]
    pub fn quanta(&self) -> u8 {
        1 + self.prop_seg + self.phase_seg1 + self.phase_seg2
    }

    /// Where in the bit it is read, in tenths of a percent (875 is 87.5 %).
    #[must_use]
    pub fn sample_point_permille(&self) -> u16 {
        let before = u16::from(1 + self.prop_seg + self.phase_seg1);
        1000 * before / u16::from(self.quanta())
    }

    /// The prescaler (1 to 64), and the segments in TQ: propagation, phase 1, phase 2, and
    /// the synchronisation jump width.
    #[must_use]
    pub fn segments(&self) -> (u8, u8, u8, u8, u8) {
        (
            self.brp,
            self.prop_seg,
            self.phase_seg1,
            self.phase_seg2,
            self.sjw,
        )
    }

    /// The bitrate these settings give from `crystal`, in bit/s.
    #[must_use]
    pub fn bitrate(&self, crystal: Crystal) -> u32 {
        crystal.hz() / (2 * u32::from(self.brp) * u32::from(self.quanta()))
    }
}

/// CiA's sample points for classic CAN, as Linux's `can_calc_sample_point_nrz()`.
fn sample_point_target(bitrate: Bitrate) -> u16 {
    match bitrate.bps() {
        b if b > 800_000 => 750,
        b if b > 500_000 => 800,
        _ => 875,
    }
}

/// Split one bit of `tq` quanta into TSEG1 (PropSeg + PS1) and TSEG2 (PS2) for a sample point
/// at or before `target`, as Linux's `can_update_sample_point()`. Returns the two and how far
/// the sample point misses the target, in tenths of a percent.
fn split(tq: u8, target: u16) -> Option<(u8, u8, u16)> {
    let tq16 = u16::from(tq);
    let tseg = tq - 1; // TSEG1 + TSEG2
                       // Quanta after the sample point, rounded up; then one fewer, as Linux tries both.
    let after = tq16 - target * tq16 / 1000;
    [after, after.saturating_sub(1)]
        .into_iter()
        .filter_map(|tseg2| {
            let tseg2 = u8::try_from(tseg2).ok()?.clamp(TSEG2.0, TSEG2.1);
            let tseg1 = tseg.checked_sub(tseg2)?;
            let (tseg1, tseg2) = if tseg1 > TSEG1.1 {
                (TSEG1.1, tseg - TSEG1.1)
            } else {
                (tseg1, tseg2)
            };
            if tseg1 < TSEG1.0 || !(TSEG2.0..=TSEG2.1).contains(&tseg2) {
                return None;
            }
            let sample = 1000 * (tq16 - u16::from(tseg2)) / tq16;
            // Only sample points at or before the target count, as in Linux.
            let error = target.checked_sub(sample)?;
            Some((tseg1, tseg2, error))
        })
        .min_by_key(|&(_, _, error)| error)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn timing(crystal: Crystal, bitrate: Bitrate) -> BitTiming {
        BitTiming::new(crystal, bitrate).expect("a standard bitrate")
    }

    #[test]
    fn the_cars_bus_on_a_16_mhz_module() {
        let t = timing(Crystal::MHZ_16, Bitrate::KBPS_500);
        // What Linux sets for an MCP2515 at 16 MHz: tq 125 ns, prop-seg 6, phase-seg1 7,
        // phase-seg2 2, sjw 1, sample point 87.5 %.
        assert_eq!(t.segments(), (1, 6, 7, 2, 1));
        assert_eq!((t.quanta(), t.sample_point_permille()), (16, 875));
        assert_eq!(t.registers(), [0x01, 0xB5, 0x00]);
    }

    #[test]
    fn the_cars_bus_on_an_8_mhz_module() {
        let t = timing(Crystal::MHZ_8, Bitrate::KBPS_500);
        // 8 quanta of 250 ns: PS2 can't go below 2, so the sample point is 75 %. Linux sets
        // the same: prop-seg 2, phase-seg1 3, phase-seg2 2, sjw 1.
        assert_eq!(t.segments(), (1, 2, 3, 2, 1));
        assert_eq!((t.quanta(), t.sample_point_permille()), (8, 750));
        assert_eq!(t.registers(), [0x01, 0x91, 0x00]);
    }

    #[test]
    fn other_bitrates() {
        // (crystal, bitrate, CNF3 CNF2 CNF1, sample point)
        let cases = [
            (Crystal::MHZ_16, Bitrate::KBPS_250, [0x01, 0xB5, 0x01], 875),
            (Crystal::MHZ_16, Bitrate::KBPS_125, [0x01, 0xB5, 0x03], 875),
            (Crystal::MHZ_16, Bitrate::MBPS_1, [0x01, 0x91, 0x00], 750),
            (Crystal::MHZ_8, Bitrate::KBPS_250, [0x01, 0xB5, 0x00], 875),
            (Crystal::MHZ_8, Bitrate::KBPS_125, [0x01, 0xB5, 0x01], 875),
            // 20 quanta: 85 % is as close to 87.5 % as a whole quantum allows.
            (
                Crystal::from_hz(20_000_000),
                Bitrate::KBPS_500,
                [0x02, 0xBF, 0x00],
                850,
            ),
        ];
        for (crystal, bitrate, cnf, sample) in cases {
            let t = timing(crystal, bitrate);
            assert_eq!(t.registers(), cnf, "{crystal:?} {bitrate:?}");
            assert_eq!(t.sample_point_permille(), sample, "{crystal:?} {bitrate:?}");
        }
    }

    #[test]
    fn an_8_mhz_crystal_cant_make_1_mbit() {
        assert_eq!(
            BitTiming::new(Crystal::MHZ_8, Bitrate::MBPS_1),
            Err(TimingError)
        );
        assert_eq!(
            BitTiming::new(Crystal::MHZ_16, Bitrate::from_bps(0)),
            Err(TimingError)
        );
        // 16 MHz / 2 / 300 kbit/s is not a whole number of quanta.
        assert_eq!(
            BitTiming::new(Crystal::MHZ_16, Bitrate::from_bps(300_000)),
            Err(TimingError)
        );
    }

    #[test]
    fn every_timing_keeps_the_data_sheets_rules() {
        let crystals = (4..=25).map(|mhz| Crystal::from_hz(mhz * 1_000_000));
        let bitrates = [
            10_000, 20_000, 50_000, 100_000, 125_000, 250_000, 500_000, 1_000_000,
        ];
        let mut found = 0;
        for crystal in crystals {
            for &bps in &bitrates {
                let Ok(t) = BitTiming::new(crystal, Bitrate::from_bps(bps)) else {
                    continue;
                };
                found += 1;
                let (brp, prop, ps1, ps2, sjw) = t.segments();
                assert_eq!(t.bitrate(crystal), bps, "{crystal:?} {bps}");
                assert!((1..=64).contains(&brp));
                assert!((1..=8).contains(&prop) && (1..=8).contains(&ps1));
                assert!((2..=8).contains(&ps2) && (1..=4).contains(&sjw));
                assert!(prop + ps1 >= ps2 && ps2 > sjw, "{t:?}");
                assert!((6..=25).contains(&t.quanta()), "{t:?}");
            }
        }
        assert!(found > 100, "{found}");
    }
}
