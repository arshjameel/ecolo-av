//! Maps on disk: a map server's `.yaml` and `.pgm` in, one `.pgm` per chunk out (C4).
//!
//! ```text
//!  campus.yaml + campus.pgm ─▶ read_map ─▶ WholeMap ─▶ cut ─▶ write_chunks
//!                                                              │
//!   <out>/chunks.yaml            the grid: origin, size        ▼
//!   <out>/fine/<i_j>.pgm         0.25 m cells     <out>/coarse/<i_j>.pgm   1 m cells
//! ```
//!
//! A chunk's `.pgm` keeps the map server's trinary colours (free 254, taken 0, unknown
//! 128; `mode: trinary`, `negate: 0`), top row first as every `.pgm` is, so any image
//! viewer shows it.

use std::fmt;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::{ChunkKey, Detail, Grid, OccupancyChunk, Point, WholeMap, UNKNOWN};

/// Why a map or a chunk couldn't be read or written.
#[derive(Debug)]
pub enum FileError {
    /// The file couldn't be read or written.
    Io(PathBuf, io::Error),
    /// The `.yaml` isn't what it should be.
    Yaml(PathBuf, String),
    /// Not a binary `.pgm` (P5, 8 bits) of the expected size.
    Pgm(PathBuf, String),
    /// `chunks.yaml`'s grid is bad.
    Grid(crate::GridError),
}

impl fmt::Display for FileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FileError::Io(path, e) => write!(f, "{}: {e}", path.display()),
            FileError::Yaml(path, e) => write!(f, "{}: {e}", path.display()),
            FileError::Pgm(path, e) => write!(f, "{}: {e}", path.display()),
            FileError::Grid(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for FileError {}

/// A map server's `.yaml`: the parts we use.
#[derive(Debug, Clone, Deserialize)]
pub struct MapYaml {
    /// The `.pgm`, relative to the `.yaml`.
    pub image: PathBuf,
    /// One cell's edge (m).
    pub resolution: f64,
    /// The lower-left corner (x, y m, yaw rad).
    pub origin: [f64; 3],
    /// map_server's: 1 swaps black and white (0 if left out).
    #[serde(default)]
    pub negate: i32,
    /// Darker than this (0..1) is taken.
    pub occupied_thresh: f64,
    /// Lighter than this is free.
    pub free_thresh: f64,
}

/// A whole map as read from disk: ROS's cells, bottom row first.
#[derive(Debug, Clone)]
pub struct LoadedMap {
    /// Its `.yaml`.
    pub yaml: MapYaml,
    /// Cells across.
    pub width: usize,
    /// Cells up.
    pub height: usize,
    /// ROS's cells (-1 unknown, 0..100), bottom row first, x fastest.
    pub data: Vec<i8>,
}

impl LoadedMap {
    /// The map as a [`WholeMap`], to cut.
    #[must_use]
    pub fn whole(&self) -> WholeMap<'_> {
        WholeMap {
            origin: Point::new(self.yaml.origin[0], self.yaml.origin[1]),
            resolution: self.yaml.resolution,
            width: self.width,
            height: self.height,
            data: &self.data,
        }
    }
}

/// Read a map server's `.yaml` and its `.pgm`, as map_server does in trinary mode.
///
/// # Errors
///
/// [`FileError`]: the file can't be read or written, or isn't what it should be.
pub fn read_map(yaml_path: &Path) -> Result<LoadedMap, FileError> {
    let text =
        fs::read_to_string(yaml_path).map_err(|e| FileError::Io(yaml_path.to_path_buf(), e))?;
    let yaml: MapYaml = serde_yaml_ng::from_str(&text)
        .map_err(|e| FileError::Yaml(yaml_path.to_path_buf(), e.to_string()))?;
    let image = yaml_path
        .parent()
        .unwrap_or(Path::new("."))
        .join(&yaml.image);
    let (width, height, pixels) = read_pgm(&image)?;
    let data = flip_rows(&pixels, width)
        .map(|v| occupancy_of(v, &yaml))
        .collect();
    Ok(LoadedMap {
        yaml,
        width,
        height,
        data,
    })
}

/// map_server's trinary rule: how dark a pixel is, against the two thresholds.
fn occupancy_of(pixel: u8, yaml: &MapYaml) -> i8 {
    let shade = f64::from(pixel) / 255.0;
    let p = if yaml.negate == 0 { 1.0 - shade } else { shade };
    if p > yaml.occupied_thresh {
        100
    } else if p < yaml.free_thresh {
        0
    } else {
        UNKNOWN
    }
}

/// A cell as a trinary pixel: free 254, taken 0, unknown 128; costs between, greyed.
///
/// Unknown is mid-grey (p = 0.498), unknown under any map YAML's thresholds; ROS 1's grey
/// 205 (p = 0.196) reads back as free under the usual `free_thresh: 0.25` (TODO 263).
fn pixel_of(cell: i8) -> u8 {
    match cell {
        UNKNOWN => 128,
        c => (254 - i32::from(c.clamp(0, 100)) * 254 / 100) as u8,
    }
}

/// The rows of a `width`-wide image, last row first (a `.pgm` is top first; ROS bottom).
fn flip_rows<T: Copy>(pixels: &[T], width: usize) -> impl Iterator<Item = T> + '_ {
    pixels.chunks(width.max(1)).rev().flatten().copied()
}

/// Read a binary 8-bit `.pgm` (P5): its width, height and pixels, top row first.
fn read_pgm(path: &Path) -> Result<(usize, usize, Vec<u8>), FileError> {
    let bytes = fs::read(path).map_err(|e| FileError::Io(path.to_path_buf(), e))?;
    let bad = |why: &str| FileError::Pgm(path.to_path_buf(), why.to_string());
    // The header: P5, width, height, maxval, each after white space or # comments.
    let mut fields = Vec::new();
    let mut at = 0;
    while fields.len() < 4 {
        while at < bytes.len() && (bytes[at].is_ascii_whitespace() || bytes[at] == b'#') {
            if bytes[at] == b'#' {
                while at < bytes.len() && bytes[at] != b'\n' {
                    at += 1;
                }
            } else {
                at += 1;
            }
        }
        let start = at;
        while at < bytes.len() && !bytes[at].is_ascii_whitespace() {
            at += 1;
        }
        if start == at {
            return Err(bad("the header ends early"));
        }
        fields.push(String::from_utf8_lossy(&bytes[start..at]).into_owned());
    }
    at += 1; // the one white-space byte before the pixels
    if fields[0] != "P5" {
        return Err(bad("not a binary pgm (P5)"));
    }
    let number = |s: &str| {
        s.parse::<usize>()
            .map_err(|_| bad("a size is not a number"))
    };
    let (width, height, maxval) = (
        number(&fields[1])?,
        number(&fields[2])?,
        number(&fields[3])?,
    );
    if maxval != 255 {
        return Err(bad("only 8-bit pgm files (maxval 255)"));
    }
    let pixels = bytes
        .get(at..at + width * height)
        .ok_or_else(|| bad("fewer pixels than its size"))?;
    Ok((width, height, pixels.to_vec()))
}

fn write_pgm(path: &Path, width: usize, pixels: &[u8]) -> Result<(), FileError> {
    let height = pixels.len() / width.max(1);
    let mut bytes = format!("P5\n{width} {height}\n255\n").into_bytes();
    bytes.extend_from_slice(pixels);
    fs::write(path, bytes).map_err(|e| FileError::Io(path.to_path_buf(), e))
}

/// The folder for one detail's chunks.
#[must_use]
pub fn detail_dir(out: &Path, detail: Detail) -> PathBuf {
    out.join(match detail {
        Detail::Fine => "fine",
        Detail::Coarse => "coarse",
    })
}

/// Cut `map` at both details and write every chunk, with `chunks.yaml` for the grid.
/// Returns how many chunks were written at each detail (fine, coarse).
///
/// # Errors
///
/// [`FileError::Io`] if a folder or file can't be written.
pub fn write_chunks(
    grid: &Grid,
    map: &WholeMap<'_>,
    out: &Path,
) -> Result<(usize, usize), FileError> {
    let mut counts = [0; 2];
    for (n, detail) in [Detail::Fine, Detail::Coarse].into_iter().enumerate() {
        let dir = detail_dir(out, detail);
        fs::create_dir_all(&dir).map_err(|e| FileError::Io(dir.clone(), e))?;
        for chunk in crate::cut(grid, map, detail).values() {
            write_chunk(out, chunk)?;
            counts[n] += 1;
        }
    }
    let origin = grid.origin();
    let index = out.join("chunks.yaml");
    let text = format!(
        "# The map in fixed chunks (map-chunks; docs/designs/2026-11-15_three-levels.md).\n\
         # One .pgm per chunk, named i_j: fine/ 0.25 m cells, coarse/ 1 m; trinary colours.\n\
         origin: [{}, {}]\nsize: {}\nfine: {}\ncoarse: {}\n",
        origin.x,
        origin.y,
        grid.size(),
        counts[0],
        counts[1]
    );
    fs::write(&index, text).map_err(|e| FileError::Io(index, e))?;
    Ok((counts[0], counts[1]))
}

/// Write one chunk as `<out>/<detail>/<i_j>.pgm`.
///
/// # Errors
///
/// [`FileError::Io`] if it can't be written.
pub fn write_chunk(out: &Path, chunk: &OccupancyChunk) -> Result<PathBuf, FileError> {
    let path = detail_dir(out, chunk.detail).join(format!("{}.pgm", chunk.key));
    let pixels: Vec<u8> = flip_rows(chunk.cells(), chunk.side())
        .map(pixel_of)
        .collect();
    write_pgm(&path, chunk.side(), &pixels)?;
    Ok(path)
}

/// Read one chunk back; `None` if it has no file (nothing known there).
///
/// # Errors
///
/// [`FileError`]: the file can't be read or written, or isn't what it should be.
pub fn read_chunk(
    out: &Path,
    grid: &Grid,
    key: ChunkKey,
    detail: Detail,
) -> Result<Option<OccupancyChunk>, FileError> {
    let path = detail_dir(out, detail).join(format!("{key}.pgm"));
    if !path.exists() {
        return Ok(None);
    }
    let (width, height, pixels) = read_pgm(&path)?;
    let side = grid.cells_per_side(detail);
    if (width, height) != (side, side) {
        return Err(FileError::Pgm(
            path,
            format!("{width} x {height}, not {side} x {side}"),
        ));
    }
    let trinary = MapYaml {
        image: PathBuf::new(),
        resolution: detail.cell(),
        origin: [0.0; 3],
        negate: 0,
        occupied_thresh: 0.65,
        free_thresh: 0.1, // 128 and old chunks' 205 (p = 0.196) unknown, 254 free
    };
    let cells = flip_rows(&pixels, width)
        .map(|v| occupancy_of(v, &trinary))
        .collect();
    Ok(OccupancyChunk::from_cells(grid, key, detail, cells))
}

/// Read the grid a folder of chunks was cut on.
///
/// # Errors
///
/// [`FileError`]: no `chunks.yaml`, a bad one, or a bad grid in it.
pub fn read_grid(out: &Path) -> Result<Grid, FileError> {
    #[derive(Deserialize)]
    struct Index {
        origin: [f64; 2],
        size: f64,
    }
    let path = out.join("chunks.yaml");
    let text = fs::read_to_string(&path).map_err(|e| FileError::Io(path.clone(), e))?;
    let index: Index =
        serde_yaml_ng::from_str(&text).map_err(|e| FileError::Yaml(path, e.to_string()))?;
    Grid::new(Point::new(index.origin[0], index.origin[1]), index.size).map_err(FileError::Grid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("map-chunks-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// A 160 x 160 m map at 0.25 m from (-80, -80): free in the middle, a wall, unknown round.
    fn write_test_map(dir: &Path) -> PathBuf {
        let side = 640;
        // Mid-grey is unknown under any thresholds (205 reads free below free_thresh 0.196).
        let mut pixels = vec![128u8; side * side]; // top row first
        for row in 200..440 {
            for col in 200..440 {
                pixels[row * side + col] = 254;
            }
        }
        for row in 200..440 {
            pixels[row * side + 300] = 0; // a wall along y, at x = -80 + 300.5 * 0.25
        }
        write_pgm(&dir.join("lot.pgm"), side, &pixels).unwrap();
        let yaml = dir.join("lot.yaml");
        fs::write(
            &yaml,
            "image: lot.pgm\nmode: trinary\nresolution: 0.25\norigin: [-80.0, -80.0, 0.0]\n\
             negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        )
        .unwrap();
        yaml
    }

    #[test]
    fn a_map_reads_as_map_server_reads_it() {
        let dir = scratch("read");
        let map = read_map(&write_test_map(&dir)).unwrap();
        assert_eq!((map.width, map.height), (640, 640));
        let at = |x: f64, y: f64| {
            let col = ((x + 80.0) / 0.25) as usize;
            let row = ((y + 80.0) / 0.25) as usize;
            map.data[row * 640 + col]
        };
        assert_eq!(at(-70.0, -70.0), UNKNOWN);
        assert_eq!(at(0.0, 0.0), 0);
        // The pgm's row 300 from the top is ROS's row 339 from the bottom: same wall either way.
        assert_eq!(at(-80.0 + 300.5 * 0.25, 0.0), 100);
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn chunks_written_read_back_the_same() {
        let dir = scratch("round");
        let map = read_map(&write_test_map(&dir)).unwrap();
        let grid = Grid::new(Point::new(-64.0, -64.0), 64.0).unwrap();
        let out = dir.join("chunks");
        let (fine, coarse) = write_chunks(&grid, &map.whole(), &out).unwrap();
        // The known square runs from -30 to 30: the four chunks round (0, 0).
        assert_eq!((fine, coarse), (4, 4));
        assert_eq!(read_grid(&out).unwrap(), grid);
        for detail in [Detail::Fine, Detail::Coarse] {
            let cut = crate::cut(&grid, &map.whole(), detail);
            for (key, chunk) in &cut {
                assert_eq!(
                    read_chunk(&out, &grid, *key, detail).unwrap().as_ref(),
                    Some(chunk)
                );
            }
            assert!(read_chunk(&out, &grid, ChunkKey::new(5, 5), detail)
                .unwrap()
                .is_none());
        }
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn a_broken_file_says_why() {
        let dir = scratch("broken");
        fs::write(dir.join("bad.pgm"), b"P2\n2 2\n255\n0 0 0 0\n").unwrap();
        let err = read_pgm(&dir.join("bad.pgm")).unwrap_err();
        assert!(err.to_string().contains("P5"), "{err}");
        assert!(matches!(
            read_map(&dir.join("none.yaml")),
            Err(FileError::Io(..))
        ));
        let _ = fs::remove_dir_all(dir);
    }
}
