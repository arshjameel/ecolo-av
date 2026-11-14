//! Is an upload an ESP32 app image for this program? Checked as the bytes come.
//!
//! The image, as `espflash save-image --chip esp32` (or esptool's `elf2image`) writes it,
//! and as the ESP-IDF bootloader reads it from an app slot:
//!
//! ```text
//!  24-byte header: 0xE9, segment count, SPI mode and speed, entry, ..., chip ID (bytes
//!                  12..13, 0 = ESP32), ..., hash appended (byte 23)
//!  each segment:   load address (4), length (4), its bytes; the first segment starts
//!                  with the app descriptor: 0xABCD5432, ..., version (32), project (32)
//!  then:           zeros to the end of a 16-byte block, whose last byte is the XOR of
//!                  every segment byte, seeded 0xEF
//!  then:           the SHA-256 of all of it (32 bytes), when byte 23 says so
//! ```
//!
//! [`ImageCheck`] refuses anything else, and an image without the SHA-256: the board
//! checks that hash on flash after the write (`esp-bootloader-esp-idf`'s
//! `PartitionEntry::sha256`).

/// The header's first byte.
pub const IMAGE_MAGIC: u8 = 0xE9;
/// The header's length.
pub const HEADER_LEN: usize = 24;
/// The chip ID in an ESP32's image (bytes 12..13).
pub const CHIP_ESP32: u16 = 0x0000;
/// The most segments the bootloader takes.
pub const MAX_SEGMENTS: u8 = 16;
/// The app descriptor's first word, at the start of the first segment.
pub const APP_DESC_MAGIC: u32 = 0xABCD_5432;
/// The appended SHA-256.
pub const HASH_LEN: usize = 32;

const CHECKSUM_SEED: u8 = 0xEF;
/// The app descriptor's bytes this check reads: magic, secure version, reserved, the
/// version (32) and the project's name (32).
const DESC_LEN: usize = 80;
const DESC_VERSION: core::ops::Range<usize> = 16..48;
const DESC_PROJECT: core::ops::Range<usize> = 48..80;

/// What the image must be.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Expect<'a> {
    /// The app descriptor's project name: the program's own (`CARGO_PKG_NAME`).
    pub project: &'a str,
    /// The OTA slot's size: the image must fit.
    pub slot_len: u32,
}

/// Why an upload is not taken. The first one found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageError {
    /// The first byte isn't 0xE9: not an app image (a merged flash image, an ELF, ...).
    NotAnImage,
    /// For another chip.
    WrongChip { chip: u16 },
    /// No segments, or more than the bootloader takes.
    Segments { count: u8 },
    /// A segment's length isn't a multiple of 4, or runs past the slot.
    BadSegment,
    /// The first segment doesn't start with the app descriptor.
    NoAppDescriptor,
    /// Another program's image.
    WrongProject,
    /// No SHA-256 appended (byte 23): the write can't be checked.
    NoHash,
    /// The XOR checksum doesn't match.
    Checksum { want: u8, got: u8 },
    /// Bigger than the slot.
    TooBig { slot_len: u32 },
    /// Bytes after the image's end.
    TrailingBytes,
    /// It ended early.
    Truncated,
}

/// What a whole, checked image is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImageInfo {
    /// Its length in bytes, hash included.
    pub len: u32,
    /// The CRC-32 of all of it, to compare with a read of the slot.
    pub crc32: u32,
    /// Its app descriptor's version (`CARGO_PKG_VERSION`), zero-padded.
    pub version: [u8; 32],
}

impl ImageInfo {
    /// The version, up to its first zero byte.
    pub fn version_str(&self) -> &str {
        let end = self.version.iter().position(|&b| b == 0).unwrap_or(32);
        core::str::from_utf8(&self.version[..end]).unwrap_or("?")
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Part {
    Header,
    SegmentHeader,
    SegmentData {
        left: u32,
    },
    /// Zeros, then the checksum byte, which is at `checksum_at`.
    Padding {
        checksum_at: u32,
    },
    Hash {
        left: u8,
    },
    Done,
}

/// The image, checked a byte at a time as it comes: [`push`](ImageCheck::push) each chunk,
/// write it once [`ready_to_write`](ImageCheck::ready_to_write), then
/// [`finish`](ImageCheck::finish).
#[derive(Debug, Clone)]
pub struct ImageCheck<'a> {
    expect: Expect<'a>,
    part: Part,
    pos: u32,
    /// The header, then each segment's header, as it fills.
    buf: [u8; HEADER_LEN],
    filled: usize,
    segments: u8,
    segment: u8,
    xor: u8,
    desc: [u8; DESC_LEN],
    desc_len: usize,
    desc_checked: bool,
    crc: Crc32,
}

impl<'a> ImageCheck<'a> {
    pub fn new(expect: Expect<'a>) -> Self {
        ImageCheck {
            expect,
            part: Part::Header,
            pos: 0,
            buf: [0; HEADER_LEN],
            filled: 0,
            segments: 0,
            segment: 0,
            xor: CHECKSUM_SEED,
            desc: [0; DESC_LEN],
            desc_len: 0,
            desc_checked: false,
            crc: Crc32::new(),
        }
    }

    /// The next bytes. The first error stops the check: every later push gives it again.
    pub fn push(&mut self, bytes: &[u8]) -> Result<(), ImageError> {
        for &b in bytes {
            self.byte(b)?;
        }
        Ok(())
    }

    /// The header and the app descriptor have been read and are right: the bytes so far
    /// may be written.
    pub fn ready_to_write(&self) -> bool {
        self.desc_checked
    }

    /// The bytes taken so far.
    pub fn len(&self) -> u32 {
        self.pos
    }

    pub fn is_empty(&self) -> bool {
        self.pos == 0
    }

    /// The upload ended: the image is whole and right, or why not.
    pub fn finish(&self) -> Result<ImageInfo, ImageError> {
        if self.part != Part::Done {
            return Err(ImageError::Truncated);
        }
        let mut version = [0; 32];
        version.copy_from_slice(&self.desc[DESC_VERSION]);
        Ok(ImageInfo {
            len: self.pos,
            crc32: self.crc.value(),
            version,
        })
    }

    fn byte(&mut self, b: u8) -> Result<(), ImageError> {
        if self.pos >= self.expect.slot_len {
            return Err(ImageError::TooBig {
                slot_len: self.expect.slot_len,
            });
        }
        let at = self.pos;
        self.pos += 1;
        self.crc.push(&[b]);
        match self.part {
            Part::Header => {
                self.buf[self.filled] = b;
                self.filled += 1;
                if self.filled == 1 && b != IMAGE_MAGIC {
                    return Err(ImageError::NotAnImage);
                }
                if self.filled == HEADER_LEN {
                    self.header()?;
                }
            }
            Part::SegmentHeader => {
                self.buf[self.filled] = b;
                self.filled += 1;
                if self.filled == 8 {
                    let len =
                        u32::from_le_bytes([self.buf[4], self.buf[5], self.buf[6], self.buf[7]]);
                    if !len.is_multiple_of(4) || len >= self.expect.slot_len {
                        return Err(ImageError::BadSegment);
                    }
                    if self.segment == 0 && (len as usize) < DESC_LEN {
                        return Err(ImageError::NoAppDescriptor);
                    }
                    self.filled = 0;
                    self.part = if len == 0 {
                        self.next_segment()
                    } else {
                        Part::SegmentData { left: len }
                    };
                }
            }
            Part::SegmentData { left } => {
                self.xor ^= b;
                if self.segment == 0 && self.desc_len < DESC_LEN {
                    self.desc[self.desc_len] = b;
                    self.desc_len += 1;
                    if self.desc_len == DESC_LEN {
                        self.descriptor()?;
                    }
                }
                self.part = if left == 1 {
                    self.next_segment()
                } else {
                    Part::SegmentData { left: left - 1 }
                };
            }
            Part::Padding { checksum_at } => {
                if at == checksum_at {
                    if b != self.xor {
                        return Err(ImageError::Checksum {
                            want: self.xor,
                            got: b,
                        });
                    }
                    self.part = Part::Hash {
                        left: HASH_LEN as u8,
                    };
                }
            }
            Part::Hash { left } => {
                self.part = if left == 1 {
                    Part::Done
                } else {
                    Part::Hash { left: left - 1 }
                };
            }
            Part::Done => return Err(ImageError::TrailingBytes),
        }
        Ok(())
    }

    fn header(&mut self) -> Result<(), ImageError> {
        let h = &self.buf;
        let chip = u16::from_le_bytes([h[12], h[13]]);
        if chip != CHIP_ESP32 {
            return Err(ImageError::WrongChip { chip });
        }
        let count = h[1];
        if count == 0 || count > MAX_SEGMENTS {
            return Err(ImageError::Segments { count });
        }
        if h[23] != 1 {
            return Err(ImageError::NoHash);
        }
        self.segments = count;
        self.filled = 0;
        self.part = Part::SegmentHeader;
        Ok(())
    }

    fn descriptor(&mut self) -> Result<(), ImageError> {
        let d = &self.desc;
        if u32::from_le_bytes([d[0], d[1], d[2], d[3]]) != APP_DESC_MAGIC {
            return Err(ImageError::NoAppDescriptor);
        }
        let name = &d[DESC_PROJECT];
        let end = name.iter().position(|&b| b == 0).unwrap_or(name.len());
        if &name[..end] != self.expect.project.as_bytes() {
            return Err(ImageError::WrongProject);
        }
        self.desc_checked = true;
        Ok(())
    }

    /// After a segment's last byte: the next segment, or the padding.
    fn next_segment(&mut self) -> Part {
        self.segment += 1;
        if self.segment < self.segments {
            Part::SegmentHeader
        } else {
            // The checksum is the last byte of the 16-byte block after the segments.
            let end = (self.pos + 1).div_ceil(16) * 16;
            Part::Padding {
                checksum_at: end - 1,
            }
        }
    }
}

/// CRC-32 (IEEE 802.3, as zlib's `crc32`), a chunk at a time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Crc32 {
    state: u32,
}

impl Crc32 {
    pub const fn new() -> Self {
        Crc32 { state: !0 }
    }

    pub fn push(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.state ^= u32::from(b);
            for _ in 0..8 {
                let mask = (self.state & 1).wrapping_neg();
                self.state = (self.state >> 1) ^ (0xEDB8_8320 & mask);
            }
        }
    }

    pub const fn value(&self) -> u32 {
        !self.state
    }
}

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    const PROJECT: &str = "spine-esp32-umod";

    /// A small image as espflash makes one: two segments, the first with the descriptor.
    fn image(chip: u16, project: &str, hash: bool) -> Vec<u8> {
        let mut desc = std::vec![0u8; 256];
        desc[..4].copy_from_slice(&APP_DESC_MAGIC.to_le_bytes());
        desc[16..21].copy_from_slice(b"0.1.0");
        desc[48..48 + project.len()].copy_from_slice(project.as_bytes());
        let segments: [Vec<u8>; 2] = [desc, (0u8..64).collect()];
        let mut out = std::vec![0u8; HEADER_LEN];
        out[0] = IMAGE_MAGIC;
        out[1] = 2;
        out[12..14].copy_from_slice(&chip.to_le_bytes());
        out[23] = u8::from(hash);
        let mut xor = CHECKSUM_SEED;
        for (i, seg) in segments.iter().enumerate() {
            out.extend_from_slice(&(0x3f40_0020u32 + i as u32).to_le_bytes());
            out.extend_from_slice(&(seg.len() as u32).to_le_bytes());
            out.extend_from_slice(seg);
            xor = seg.iter().fold(xor, |x, &b| x ^ b);
        }
        while !(out.len() + 1).is_multiple_of(16) {
            out.push(0);
        }
        out.push(xor);
        if hash {
            out.extend_from_slice(&[0xAB; HASH_LEN]);
        }
        out
    }

    fn expect() -> Expect<'static> {
        Expect {
            project: PROJECT,
            slot_len: 0x1000,
        }
    }

    fn check(bytes: &[u8], chunk: usize) -> Result<ImageInfo, ImageError> {
        let mut c = ImageCheck::new(expect());
        for part in bytes.chunks(chunk) {
            c.push(part)?;
        }
        c.finish()
    }

    #[test]
    fn a_good_image_passes_in_any_chunks_with_its_length_crc_and_version() {
        let img = image(CHIP_ESP32, PROJECT, true);
        for chunk in [1, 7, 64, 4096] {
            let info = check(&img, chunk).unwrap();
            assert_eq!(info.len as usize, img.len());
            let mut crc = Crc32::new();
            crc.push(&img);
            assert_eq!(info.crc32, crc.value());
            assert_eq!(info.version_str(), "0.1.0");
        }
        assert_eq!(img.len() % 16, 0);
    }

    #[test]
    fn it_is_ready_to_write_only_after_the_descriptor() {
        let img = image(CHIP_ESP32, PROJECT, true);
        let mut c = ImageCheck::new(expect());
        c.push(&img[..HEADER_LEN + 8 + 79]).unwrap();
        assert!(!c.ready_to_write());
        c.push(&img[HEADER_LEN + 8 + 79..HEADER_LEN + 8 + 80])
            .unwrap();
        assert!(c.ready_to_write());
    }

    #[test]
    fn everything_else_is_refused() {
        let good = image(CHIP_ESP32, PROJECT, true);
        assert_eq!(
            check(&[0x7F, b'E', b'L', b'F'], 64),
            Err(ImageError::NotAnImage)
        );
        assert_eq!(
            check(&image(0x0009, PROJECT, true), 64),
            Err(ImageError::WrongChip { chip: 9 })
        );
        assert_eq!(
            check(&image(CHIP_ESP32, "blinky", true), 64),
            Err(ImageError::WrongProject)
        );
        assert_eq!(
            check(&image(CHIP_ESP32, PROJECT, false), 64),
            Err(ImageError::NoHash)
        );
        // One bit flipped in a segment: the checksum.
        let mut bad = good.clone();
        bad[HEADER_LEN + 8 + 100] ^= 1;
        assert!(matches!(check(&bad, 64), Err(ImageError::Checksum { .. })));
        // Cut short, and one byte too many.
        assert_eq!(
            check(&good[..good.len() - 1], 64),
            Err(ImageError::Truncated)
        );
        let mut long = good.clone();
        long.push(0);
        assert_eq!(check(&long, 64), Err(ImageError::TrailingBytes));
        // No segments; a length not a multiple of 4.
        let mut none = good.clone();
        none[1] = 0;
        assert_eq!(check(&none, 64), Err(ImageError::Segments { count: 0 }));
        let mut odd = good.clone();
        odd[HEADER_LEN + 4] = 3;
        assert_eq!(check(&odd, 64), Err(ImageError::BadSegment));
        // The descriptor's magic.
        let mut nodesc = good.clone();
        nodesc[HEADER_LEN + 8] = 0;
        assert_eq!(check(&nodesc, 64), Err(ImageError::NoAppDescriptor));
        // Bigger than the slot.
        let mut c = ImageCheck::new(Expect {
            project: PROJECT,
            slot_len: 300,
        });
        assert_eq!(c.push(&good), Err(ImageError::TooBig { slot_len: 300 }));
    }

    #[test]
    fn crc32_is_zlibs() {
        let mut c = Crc32::new();
        c.push(b"123456789");
        assert_eq!(c.value(), 0xCBF4_3926);
        let mut halves = Crc32::new();
        halves.push(b"1234");
        halves.push(b"56789");
        assert_eq!(halves, c);
        assert_eq!(Crc32::new().value(), 0);
    }
}

/// A real image, when `AAV_ESP32_APP_IMAGE` names one (`espflash save-image --chip esp32
/// <elf> <file>`; not `--merge`): what the board would be sent.
#[cfg(test)]
mod real_image {
    extern crate std;

    use super::*;

    #[test]
    fn a_real_espflash_image_passes() {
        let Some(path) = std::env::var_os("AAV_ESP32_APP_IMAGE") else {
            return; // nothing to read: the made-up images above still ran
        };
        let bytes = std::fs::read(path).expect("the image");
        let mut c = ImageCheck::new(Expect {
            project: "spine-esp32-umod",
            slot_len: 0x1E_0000,
        });
        c.push(&bytes).expect("a valid image");
        let info = c.finish().expect("whole");
        assert_eq!(info.len as usize, bytes.len());
        assert_eq!(info.version_str(), "0.1.0");
    }
}
