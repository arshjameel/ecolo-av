//! A classic CAN frame, and its layout in the MCP2515's transmit and receive buffers:
//! SIDH, SIDL, EID8, EID0, DLC, then up to 8 data bytes (data sheet, Registers 3-3 to 3-8
//! and 4-4 to 4-9).

use embedded_can::{ExtendedId, Id, StandardId};

/// SIDL's bits: SRR (a standard remote frame, received only) and IDE / EXIDE (an extended ID).
const SIDL_SRR: u8 = 0x10;
const SIDL_IDE: u8 = 0x08;
/// DLC's RTR bit: a remote frame (sent, or an extended frame received).
const DLC_RTR: u8 = 0x40;

/// A classic CAN frame: an 11-bit or 29-bit ID, and 0 to 8 data bytes (none in a remote frame).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    id: Id,
    remote: bool,
    dlc: u8,
    data: [u8; 8],
}

impl Frame {
    /// A data frame; None with more than 8 bytes.
    pub fn new(id: impl Into<Id>, data: &[u8]) -> Option<Frame> {
        let dlc = u8::try_from(data.len()).ok().filter(|&n| n <= 8)?;
        let mut frame = Frame {
            id: id.into(),
            remote: false,
            dlc,
            data: [0; 8],
        };
        frame.data[..data.len()].copy_from_slice(data);
        Some(frame)
    }

    /// A remote frame, asking for `dlc` bytes; None above 8.
    pub fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Frame> {
        let dlc = u8::try_from(dlc).ok().filter(|&n| n <= 8)?;
        Some(Frame {
            id: id.into(),
            remote: true,
            dlc,
            data: [0; 8],
        })
    }

    /// A data frame with an 11-bit ID given as a number, as the spine's frames are; None if
    /// the ID needs more than 11 bits, or there are more than 8 bytes.
    #[must_use]
    pub fn new_standard(id: u16, data: &[u8]) -> Option<Frame> {
        Frame::new(StandardId::new(id)?, data)
    }

    /// The ID and the bytes of a standard (11-bit) data frame; None for an extended or a
    /// remote frame.
    #[must_use]
    pub fn standard_data(&self) -> Option<(u16, &[u8])> {
        match self.id {
            Id::Standard(id) if !self.remote => Some((id.as_raw(), self.data())),
            _ => None,
        }
    }

    /// The frame's ID, standard or extended.
    #[must_use]
    pub fn id(&self) -> Id {
        self.id
    }

    /// The data bytes; empty for a remote frame.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        if self.remote {
            &[]
        } else {
            &self.data[..usize::from(self.dlc)]
        }
    }

    /// 0 to 8.
    #[must_use]
    pub fn dlc(&self) -> usize {
        usize::from(self.dlc)
    }

    /// A remote frame: a request, with no data.
    #[must_use]
    pub fn is_remote(&self) -> bool {
        self.remote
    }

    /// The bytes for LOAD TX BUFFER, and how many of them to send.
    pub(crate) fn to_registers(self) -> ([u8; 13], usize) {
        let mut r = [0; 13];
        match self.id {
            Id::Standard(id) => {
                let raw = id.as_raw();
                r[0] = (raw >> 3) as u8;
                r[1] = ((raw & 0x07) as u8) << 5;
            }
            Id::Extended(id) => {
                let raw = id.as_raw();
                r[0] = (raw >> 21) as u8;
                r[1] = (((raw >> 18) & 0x07) as u8) << 5 | SIDL_IDE | ((raw >> 16) & 0x03) as u8;
                r[2] = (raw >> 8) as u8;
                r[3] = raw as u8;
            }
        }
        r[4] = self.dlc | if self.remote { DLC_RTR } else { 0 };
        let data = self.data();
        r[5..5 + data.len()].copy_from_slice(data);
        (r, 5 + data.len())
    }

    /// A frame from READ RX BUFFER's 13 bytes. A DLC above 8 means 8 bytes, as in CAN 2.0.
    pub(crate) fn from_registers(r: &[u8; 13]) -> Frame {
        let sid = u16::from(r[0]) << 3 | u16::from(r[1] >> 5);
        let extended = r[1] & SIDL_IDE != 0;
        let (id, remote) = if extended {
            let raw = u32::from(sid) << 18
                | u32::from(r[1] & 0x03) << 16
                | u32::from(r[2]) << 8
                | u32::from(r[3]);
            // 11 + 18 bits always fit in 29.
            let id = ExtendedId::new(raw).unwrap_or(ExtendedId::MAX);
            (Id::Extended(id), r[4] & DLC_RTR != 0)
        } else {
            let id = StandardId::new(sid).unwrap_or(StandardId::MAX); // 11 bits always fit
            (Id::Standard(id), r[1] & SIDL_SRR != 0)
        };
        let dlc = (r[4] & 0x0F).min(8);
        let mut data = [0; 8];
        if !remote {
            data.copy_from_slice(&r[5..13]);
            data[usize::from(dlc)..].fill(0);
        }
        Frame {
            id,
            remote,
            dlc,
            data,
        }
    }
}

impl embedded_can::Frame for Frame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        Frame::new(id, data)
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        Frame::new_remote(id, dlc)
    }

    fn is_extended(&self) -> bool {
        matches!(self.id, Id::Extended(_))
    }

    fn is_remote_frame(&self) -> bool {
        self.remote
    }

    fn id(&self) -> Id {
        self.id
    }

    fn dlc(&self) -> usize {
        Frame::dlc(self)
    }

    fn data(&self) -> &[u8] {
        Frame::data(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn std_id(raw: u16) -> StandardId {
        StandardId::new(raw).unwrap()
    }

    #[test]
    fn a_standard_frame_in_the_buffer() {
        // DRIVE_CMD's ID, 0x100: SIDH 0x20, SIDL 0x00. 0x181: SIDH 0x30, SIDL 0x20.
        let f = Frame::new(std_id(0x100), &[1, 2, 3]).unwrap();
        let (r, n) = f.to_registers();
        assert_eq!(&r[..n], &[0x20, 0x00, 0, 0, 3, 1, 2, 3]);
        let (r, _) = Frame::new(std_id(0x181), &[]).unwrap().to_registers();
        assert_eq!(&r[..5], &[0x30, 0x20, 0, 0, 0]);
        let (r, _) = Frame::new(std_id(0x7FF), &[]).unwrap().to_registers();
        assert_eq!(&r[..2], &[0xFF, 0xE0]);
    }

    #[test]
    fn frames_survive_the_buffer() {
        let id = ExtendedId::new(0x1ABC_DEF5).unwrap();
        for frame in [
            Frame::new(std_id(0x010), &[0x01, 0xA5]).unwrap(),
            Frame::new(std_id(0x7FF), &[9; 8]).unwrap(),
            Frame::new(id, &[1, 2, 3, 4, 5]).unwrap(),
            Frame::new(ExtendedId::MAX, &[]).unwrap(),
            Frame::new_remote(id, 4).unwrap(),
        ] {
            let (r, n) = frame.to_registers();
            let mut rx = [0xEE; 13]; // the unsent bytes are whatever the buffer held
            rx[..n].copy_from_slice(&r[..n]);
            assert_eq!(Frame::from_registers(&rx), frame, "{frame:?}");
        }
    }

    #[test]
    fn an_extended_id_in_the_buffer() {
        let id = ExtendedId::new(0x1ABC_DEF5).unwrap();
        let (r, n) = Frame::new(id, &[]).unwrap().to_registers();
        // SID = 0x1ABCDEF5 >> 18 = 0x6AF; EID17..16 = 0b00; EID15..0 = 0xDEF5.
        assert_eq!(&r[..n], &[0xD5, 0xE8, 0xDE, 0xF5, 0]);
    }

    #[test]
    fn remote_frames_as_received() {
        // A standard remote frame sets SRR in SIDL; an extended one sets RTR in DLC.
        let mut r = [0; 13];
        r[0] = 0x20;
        r[1] = SIDL_SRR;
        r[4] = 2;
        let f = Frame::from_registers(&r);
        assert!(f.is_remote());
        assert_eq!((f.dlc(), f.data()), (2, &[][..]));
        let mut r = [0; 13];
        r[1] = SIDL_IDE;
        r[4] = DLC_RTR | 1;
        assert!(Frame::from_registers(&r).is_remote());
    }

    #[test]
    fn a_dlc_above_8_is_8_bytes() {
        let mut r = [7; 13];
        r[0] = 0x20;
        r[1] = 0;
        r[4] = 15;
        let f = Frame::from_registers(&r);
        assert_eq!((f.dlc(), f.data()), (8, &[7; 8][..]));
    }

    #[test]
    fn standard_frames_by_number() {
        let f = Frame::new_standard(0x181, &[1, 2]).unwrap();
        assert_eq!(f.standard_data(), Some((0x181, &[1, 2][..])));
        assert!(Frame::new_standard(0x800, &[]).is_none()); // needs 12 bits
        assert!(Frame::new_standard(0x100, &[0; 9]).is_none());
        // Extended and remote frames are not the spine's.
        let extended = Frame::new(ExtendedId::new(0x100).unwrap(), &[1]).unwrap();
        assert_eq!(extended.standard_data(), None);
        assert_eq!(
            Frame::new_remote(std_id(0x100), 1).unwrap().standard_data(),
            None
        );
    }

    #[test]
    fn at_most_8_bytes() {
        assert!(Frame::new(std_id(1), &[0; 9]).is_none());
        assert!(Frame::new_remote(std_id(1), 9).is_none());
        let f = <Frame as embedded_can::Frame>::new(std_id(5), &[1]).unwrap();
        assert!(embedded_can::Frame::is_standard(&f));
        assert!(embedded_can::Frame::is_data_frame(&f));
    }
}
