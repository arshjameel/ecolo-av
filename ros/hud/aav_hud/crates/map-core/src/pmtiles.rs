//! Reads a PMTiles file (version 3): many map tiles in one file, found through
//! directories of tile numbers. The spec: <https://github.com/protomaps/PMTiles/blob/main/spec/v3/spec.md>
//!
//! Why our own reader: the maintained Rust crate (`pmtiles`, MIT/Apache) reads only
//! through tokio's async I/O, and `pmtiles2` brings brotli, zstd and deku for a format
//! that needs a varint and gzip here. This file is the part we need: the header, the
//! directories, and gzip (flate2).
//!
//! The whole file is read into memory: the Ottawa extract is about 40 MB.

use std::io::Read as _;

use flate2::read::GzDecoder;
use thiserror::Error;

use crate::tiles::TileId;

const MAGIC: &[u8; 7] = b"PMTiles";
const HEADER_LEN: usize = 127;
/// Deeper than this, a directory chain is broken (the spec needs at most 4 levels).
const MAX_DEPTH: usize = 4;

/// Why a PMTiles file can't be read.
#[derive(Debug, Error)]
pub enum PmtilesError {
    #[error("not a PMTiles version 3 file")]
    NotPmtiles,
    #[error("the file is cut short (wanted bytes {0} to {1})")]
    Truncated(u64, u64),
    #[error("{0} compression isn't supported here (only gzip or none)")]
    Compression(Compression),
    #[error("the tiles are {0:?}, not vector tiles (MVT)")]
    NotVector(TileType),
    #[error("a directory is broken")]
    BadDirectory,
    #[error("gzip: {0}")]
    Gzip(#[from] std::io::Error),
}

/// How the directories or the tiles are compressed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Compression {
    Unknown,
    None,
    Gzip,
    Brotli,
    Zstd,
}

impl From<u8> for Compression {
    fn from(code: u8) -> Self {
        match code {
            1 => Self::None,
            2 => Self::Gzip,
            3 => Self::Brotli,
            4 => Self::Zstd,
            _ => Self::Unknown,
        }
    }
}

impl std::fmt::Display for Compression {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

/// What the tiles are.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TileType {
    Unknown,
    Mvt,
    Png,
    Jpeg,
    Webp,
    Avif,
}

impl From<u8> for TileType {
    fn from(code: u8) -> Self {
        match code {
            1 => Self::Mvt,
            2 => Self::Png,
            3 => Self::Jpeg,
            4 => Self::Webp,
            5 => Self::Avif,
            _ => Self::Unknown,
        }
    }
}

/// The parts of the header used here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Header {
    root: (u64, u64),
    leaves: u64,
    data: u64,
    internal: Compression,
    pub tile_compression: Compression,
    pub tile_type: TileType,
    pub min_zoom: u8,
    pub max_zoom: u8,
}

/// One directory entry: a run of tiles, or (run 0) a deeper directory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Entry {
    tile_id: u64,
    offset: u64,
    length: u64,
    run_length: u64,
}

/// A PMTiles file of vector tiles, in memory.
#[derive(Debug, Clone)]
pub struct Archive {
    bytes: Vec<u8>,
    pub header: Header,
    root: Vec<Entry>,
}

impl Archive {
    /// Reads the header and the root directory. Only vector tiles (MVT), compressed with
    /// gzip or not at all.
    pub fn new(bytes: Vec<u8>) -> Result<Self, PmtilesError> {
        let header = parse_header(&bytes)?;
        if header.tile_type != TileType::Mvt {
            return Err(PmtilesError::NotVector(header.tile_type));
        }
        for c in [header.internal, header.tile_compression] {
            if !matches!(c, Compression::Gzip | Compression::None) {
                return Err(PmtilesError::Compression(c));
            }
        }
        let mut archive = Self {
            bytes,
            header,
            root: Vec::new(),
        };
        let (offset, length) = header.root;
        archive.root = archive.directory(offset, length)?;
        Ok(archive)
    }

    /// One tile's bytes, unzipped (a Mapbox Vector Tile); None when the file has no such
    /// tile.
    pub fn tile(&self, id: TileId) -> Result<Option<Vec<u8>>, PmtilesError> {
        let wanted = tile_number(id);
        let mut dir = std::borrow::Cow::Borrowed(&self.root);
        for _ in 0..MAX_DEPTH {
            let Some(e) = find(&dir, wanted) else {
                return Ok(None);
            };
            if e.run_length > 0 {
                let raw = self.slice(self.header.data + e.offset, e.length)?;
                return decompress(raw, self.header.tile_compression).map(Some);
            }
            dir = std::borrow::Cow::Owned(self.directory(self.header.leaves + e.offset, e.length)?);
        }
        Err(PmtilesError::BadDirectory)
    }

    fn slice(&self, offset: u64, length: u64) -> Result<&[u8], PmtilesError> {
        let end = offset.saturating_add(length);
        usize::try_from(offset)
            .ok()
            .zip(usize::try_from(end).ok())
            .and_then(|(a, b)| self.bytes.get(a..b))
            .ok_or(PmtilesError::Truncated(offset, end))
    }

    fn directory(&self, offset: u64, length: u64) -> Result<Vec<Entry>, PmtilesError> {
        let raw = decompress(self.slice(offset, length)?, self.header.internal)?;
        parse_directory(&raw).ok_or(PmtilesError::BadDirectory)
    }
}

fn parse_header(b: &[u8]) -> Result<Header, PmtilesError> {
    if b.len() < HEADER_LEN || &b[..7] != MAGIC || b[7] != 3 {
        return Err(PmtilesError::NotPmtiles);
    }
    let u64_at = |at: usize| {
        let mut word = [0u8; 8];
        word.copy_from_slice(&b[at..at + 8]);
        u64::from_le_bytes(word)
    };
    Ok(Header {
        root: (u64_at(8), u64_at(16)),
        leaves: u64_at(40),
        data: u64_at(56),
        internal: Compression::from(b[97]),
        tile_compression: Compression::from(b[98]),
        tile_type: TileType::from(b[99]),
        min_zoom: b[100],
        max_zoom: b[101],
    })
}

fn decompress(raw: &[u8], how: Compression) -> Result<Vec<u8>, PmtilesError> {
    match how {
        Compression::None => Ok(raw.to_vec()),
        Compression::Gzip => {
            let mut out = Vec::with_capacity(raw.len() * 3);
            GzDecoder::new(raw).read_to_end(&mut out)?;
            Ok(out)
        }
        other => Err(PmtilesError::Compression(other)),
    }
}

/// Unsigned LEB128 varints, as protobuf.
struct Varints<'a> {
    bytes: &'a [u8],
}

impl Varints<'_> {
    fn next(&mut self) -> Option<u64> {
        let mut value = 0u64;
        for (i, &byte) in self.bytes.iter().enumerate().take(10) {
            value |= u64::from(byte & 0x7f) << (7 * i);
            if byte & 0x80 == 0 {
                self.bytes = &self.bytes[i + 1..];
                return Some(value);
            }
        }
        None
    }
}

/// A directory: the count, then each column (tile ids as differences, run lengths,
/// lengths, offsets; an offset of 0 means "right after the one before").
fn parse_directory(raw: &[u8]) -> Option<Vec<Entry>> {
    let mut v = Varints { bytes: raw };
    let n = usize::try_from(v.next()?).ok()?;
    // Each entry takes at least 4 bytes: a corrupt count can't ask for a huge Vec.
    if n > raw.len() {
        return None;
    }
    let mut entries = vec![
        Entry {
            tile_id: 0,
            offset: 0,
            length: 0,
            run_length: 0,
        };
        n
    ];
    let mut id = 0u64;
    for e in &mut entries {
        id = id.checked_add(v.next()?)?;
        e.tile_id = id;
    }
    for e in &mut entries {
        e.run_length = v.next()?;
    }
    for e in &mut entries {
        e.length = v.next()?;
    }
    let mut previous: Option<Entry> = None;
    for e in &mut entries {
        let raw_offset = v.next()?;
        e.offset = match (raw_offset, previous) {
            (0, Some(p)) => p.offset + p.length,
            (o, _) => o.checked_sub(1)?,
        };
        previous = Some(*e);
    }
    Some(entries)
}

/// The entry that holds tile `wanted`: the last one starting at or before it, if its run
/// reaches it (or it points to a deeper directory).
fn find(entries: &[Entry], wanted: u64) -> Option<Entry> {
    let i = entries
        .partition_point(|e| e.tile_id <= wanted)
        .checked_sub(1)?;
    let e = entries[i];
    (e.run_length == 0 || wanted < e.tile_id + e.run_length).then_some(e)
}

/// A tile's number in PMTiles: all the tiles of the zooms before it, then its place
/// along a Hilbert curve at its own zoom.
pub fn tile_number(id: TileId) -> u64 {
    let before: u64 = (0..id.z).map(|z| 1u64 << (2 * u32::from(z))).sum();
    let n = 1u64 << id.z;
    let (mut x, mut y) = (u64::from(id.x), u64::from(id.y));
    let mut d = 0u64;
    let mut s = n / 2;
    while s > 0 {
        let rx = u64::from(x & s > 0);
        let ry = u64::from(y & s > 0);
        d += s * s * ((3 * rx) ^ ry);
        // Turn the quadrant so the curve carries on.
        if ry == 0 {
            if rx == 1 {
                x = n - 1 - x;
                y = n - 1 - y;
            }
            std::mem::swap(&mut x, &mut y);
        }
        s /= 2;
    }
    before + d
}

#[cfg(test)]
mod tests {
    use super::*;
    use flate2::{write::GzEncoder, Compression as Level};
    use std::io::Write as _;

    #[test]
    fn tile_numbers_match_the_spec() {
        // The spec's own examples.
        assert_eq!(tile_number(TileId::new(0, 0, 0)), 0);
        assert_eq!(tile_number(TileId::new(1, 0, 0)), 1);
        assert_eq!(tile_number(TileId::new(1, 0, 1)), 2);
        assert_eq!(tile_number(TileId::new(1, 1, 1)), 3);
        assert_eq!(tile_number(TileId::new(1, 1, 0)), 4);
        assert_eq!(tile_number(TileId::new(2, 0, 0)), 5);
        assert_eq!(tile_number(TileId::new(12, 3423, 1763)), 19_078_479);
    }

    fn varint(mut v: u64, out: &mut Vec<u8>) {
        loop {
            let byte = (v & 0x7f) as u8;
            v >>= 7;
            if v == 0 {
                out.push(byte);
                return;
            }
            out.push(byte | 0x80);
        }
    }

    fn gzip(b: &[u8]) -> Vec<u8> {
        let mut e = GzEncoder::new(Vec::new(), Level::default());
        e.write_all(b).expect("in memory");
        e.finish().expect("in memory")
    }

    /// (tile id, run length, offset, length) -> a gzipped directory.
    fn directory(entries: &[(u64, u64, u64, u64)]) -> Vec<u8> {
        let mut out = Vec::new();
        varint(entries.len() as u64, &mut out);
        let mut last = 0;
        for &(id, ..) in entries {
            varint(id - last, &mut out);
            last = id;
        }
        for &(_, run, ..) in entries {
            varint(run, &mut out);
        }
        for &(.., len) in entries {
            varint(len, &mut out);
        }
        for &(_, _, off, _) in entries {
            varint(off + 1, &mut out);
        }
        gzip(&out)
    }

    /// A small archive: tile 0/0/0, tiles 1/0/0 to 1/0/1 sharing one blob (a run of 2),
    /// and zoom 2's tiles behind a leaf directory.
    fn archive() -> Vec<u8> {
        let blobs = [gzip(b"world"), gzip(b"half"), gzip(b"zoom two")];
        let mut data = Vec::new();
        let mut at = Vec::new();
        for b in &blobs {
            at.push((data.len() as u64, b.len() as u64));
            data.extend_from_slice(b);
        }
        let leaf = directory(&[(5, 1, at[2].0, at[2].1)]);
        // Root: 0 -> blob 0; 1 and 2 -> blob 1; 5.. -> the leaf.
        let root = directory(&[
            (0, 1, at[0].0, at[0].1),
            (1, 2, at[1].0, at[1].1),
            (5, 0, 0, leaf.len() as u64),
        ]);
        let root_at = HEADER_LEN as u64;
        let leaves_at = root_at + root.len() as u64;
        let data_at = leaves_at + leaf.len() as u64;
        let mut h = vec![0u8; HEADER_LEN];
        h[..7].copy_from_slice(MAGIC);
        h[7] = 3;
        let mut put = |at: usize, v: u64| h[at..at + 8].copy_from_slice(&v.to_le_bytes());
        put(8, root_at);
        put(16, root.len() as u64);
        put(40, leaves_at);
        put(48, leaf.len() as u64);
        put(56, data_at);
        put(64, data.len() as u64);
        h[97] = 2; // gzip directories
        h[98] = 2; // gzip tiles
        h[99] = 1; // MVT
        h[101] = 2;
        [h, root, leaf, data].concat()
    }

    #[test]
    fn tiles_come_out_of_runs_and_leaf_directories() {
        let a = Archive::new(archive()).expect("a valid archive");
        assert_eq!(a.header.max_zoom, 2);
        let get = |z, x, y| a.tile(TileId::new(z, x, y)).expect("readable");
        assert_eq!(get(0, 0, 0).as_deref(), Some(&b"world"[..]));
        assert_eq!(get(1, 0, 0).as_deref(), Some(&b"half"[..]));
        assert_eq!(get(1, 0, 1).as_deref(), Some(&b"half"[..]));
        assert_eq!(get(1, 1, 1), None);
        assert_eq!(get(2, 0, 0).as_deref(), Some(&b"zoom two"[..]));
        assert_eq!(get(2, 3, 3), None);
    }

    #[test]
    fn bad_files_are_refused() {
        assert!(matches!(
            Archive::new(b"not a map".to_vec()),
            Err(PmtilesError::NotPmtiles)
        ));
        let mut png = archive();
        png[99] = 2;
        assert!(matches!(
            Archive::new(png),
            Err(PmtilesError::NotVector(TileType::Png))
        ));
        let mut zstd = archive();
        zstd[98] = 4;
        assert!(matches!(
            Archive::new(zstd),
            Err(PmtilesError::Compression(Compression::Zstd))
        ));
        let mut short = archive();
        short.truncate(HEADER_LEN + 3);
        assert!(matches!(
            Archive::new(short),
            Err(PmtilesError::Truncated(..))
        ));
    }

    /// The real Ottawa extract, when `pixi run get street-map` has downloaded it.
    #[test]
    fn the_ottawa_extract_has_carleton() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../../../maps/streets/ottawa.pmtiles"
        );
        let Ok(bytes) = std::fs::read(path) else {
            eprintln!("skipped: no {path} (pixi run get street-map)");
            return;
        };
        let a = Archive::new(bytes).expect("the extract reads");
        assert_eq!(a.header.max_zoom, 15);
        let tile = a.tile(TileId::new(15, 9493, 11737)).expect("readable");
        assert!(tile.is_some_and(|t| t.len() > 1000));
        // Far from Ottawa: not in the extract.
        assert_eq!(a.tile(TileId::new(15, 0, 0)).expect("readable"), None);
    }
}
