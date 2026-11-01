//! The map file: PCL's PCD format (v0.7), which PCL, CloudCompare, Foxglove and Autoware
//! read. Only x, y and z are kept; other fields are skipped. `DATA binary` and `DATA
//! ascii`; not `binary_compressed`. `tools/ndt_map.py` writes binary float32 x y z.

use std::{
    error::Error,
    fmt,
    io::{self, BufRead, Read, Write},
};

use crate::Point;

/// Why a PCD file couldn't be read.
#[derive(Debug)]
pub enum PcdError {
    Io(io::Error),
    /// A header line is missing or wrong.
    Header(String),
    /// A DATA kind this reader doesn't know (binary_compressed).
    Unsupported(String),
    /// The data ends before `POINTS` points.
    Short {
        expected: usize,
        got: usize,
    },
}

impl fmt::Display for PcdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "reading the PCD: {e}"),
            Self::Header(what) => write!(f, "the PCD's header: {what}"),
            Self::Unsupported(kind) => write!(f, "PCD DATA {kind} isn't supported"),
            Self::Short { expected, got } => {
                write!(f, "the PCD says {expected} points but holds {got}")
            }
        }
    }
}

impl Error for PcdError {}

impl From<io::Error> for PcdError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

/// One field: its name, the bytes of one value, its type letter (F, I, U) and how many.
#[derive(Clone, Debug, PartialEq)]
struct Column {
    name: String,
    size: usize,
    kind: char,
    count: usize,
}

/// Reads a PCD file's points (x, y, z).
pub fn read(input: impl Read) -> Result<Vec<Point>, PcdError> {
    let mut input = io::BufReader::new(input);
    let mut header: Vec<(String, Vec<String>)> = Vec::new();
    let data = loop {
        let mut line = String::new();
        if input.read_line(&mut line)? == 0 {
            return Err(PcdError::Header("no DATA line".into()));
        }
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut words = line.split_whitespace().map(str::to_owned);
        let key = words.next().unwrap_or_default().to_ascii_uppercase();
        let values: Vec<String> = words.collect();
        if key == "DATA" {
            break values.first().cloned().unwrap_or_default();
        }
        header.push((key, values));
    };
    let get = |key: &str| {
        header
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.clone())
            .ok_or_else(|| PcdError::Header(format!("no {key} line")))
    };
    let numbers = |key: &str| -> Result<Vec<usize>, PcdError> {
        get(key)?
            .iter()
            .map(|v| {
                v.parse()
                    .map_err(|_| PcdError::Header(format!("{key} {v}: not a count")))
            })
            .collect()
    };
    let names = get("FIELDS")?;
    let sizes = numbers("SIZE")?;
    let kinds = get("TYPE")?;
    let counts = numbers("COUNT").unwrap_or_else(|_| vec![1; names.len()]);
    if [sizes.len(), kinds.len(), counts.len()]
        .iter()
        .any(|&n| n != names.len())
    {
        return Err(PcdError::Header(
            "FIELDS, SIZE, TYPE, COUNT differ in length".into(),
        ));
    }
    let columns: Vec<Column> = names
        .into_iter()
        .zip(sizes)
        .zip(kinds)
        .zip(counts)
        .map(|(((name, size), kind), count)| Column {
            name,
            size,
            kind: kind.chars().next().unwrap_or('?'),
            count,
        })
        .collect();
    let points = match numbers("POINTS") {
        Ok(n) => n.first().copied().unwrap_or(0),
        Err(_) => {
            let w = numbers("WIDTH")?.first().copied().unwrap_or(0);
            let h = numbers("HEIGHT")?.first().copied().unwrap_or(1);
            w * h
        }
    };
    let xyz = ["x", "y", "z"].map(|n| columns.iter().position(|c| c.name == n));
    let [Some(x), Some(y), Some(z)] = xyz else {
        return Err(PcdError::Header("x, y and z are needed".into()));
    };
    match data.as_str() {
        "binary" => read_binary(input, &columns, [x, y, z], points),
        "ascii" => read_ascii(input, &columns, [x, y, z], points),
        other => Err(PcdError::Unsupported(other.into())),
    }
}

fn read_binary(
    mut input: impl Read,
    columns: &[Column],
    xyz: [usize; 3],
    points: usize,
) -> Result<Vec<Point>, PcdError> {
    let offsets: Vec<usize> = columns
        .iter()
        .scan(0, |at, c| {
            let start = *at;
            *at += c.size * c.count;
            Some(start)
        })
        .collect();
    let step: usize = columns.iter().map(|c| c.size * c.count).sum();
    for &i in &xyz {
        let c = &columns[i];
        if c.kind != 'F' || !(c.size == 4 || c.size == 8) {
            return Err(PcdError::Header(format!("{} must be F 4 or F 8", c.name)));
        }
    }
    let mut bytes = Vec::with_capacity(points * step);
    input.read_to_end(&mut bytes)?;
    let got = bytes.len() / step.max(1);
    if got < points {
        return Err(PcdError::Short {
            expected: points,
            got,
        });
    }
    let value = |point: &[u8], i: usize| -> f64 {
        let o = offsets[i];
        match columns[i].size {
            4 => f64::from(f32::from_le_bytes([0, 1, 2, 3].map(|k| point[o + k]))),
            _ => f64::from_le_bytes([0, 1, 2, 3, 4, 5, 6, 7].map(|k| point[o + k])),
        }
    };
    Ok(bytes
        .chunks_exact(step)
        .take(points)
        .map(|p| {
            let [x, y, z] = xyz.map(|i| value(p, i));
            Point::new(x, y, z)
        })
        .collect())
}

fn read_ascii(
    input: impl BufRead,
    columns: &[Column],
    xyz: [usize; 3],
    points: usize,
) -> Result<Vec<Point>, PcdError> {
    // Where each field's first value is in a line of words.
    let starts: Vec<usize> = columns
        .iter()
        .scan(0, |at, c| {
            let start = *at;
            *at += c.count;
            Some(start)
        })
        .collect();
    let mut out = Vec::with_capacity(points);
    for line in input.lines().take(points) {
        let line = line?;
        let words: Vec<&str> = line.split_whitespace().collect();
        let value = |i: usize| -> Result<f64, PcdError> {
            words
                .get(starts[i])
                .and_then(|w| w.parse().ok())
                .ok_or_else(|| PcdError::Header(format!("a point line: {line}")))
        };
        out.push(Point::new(value(xyz[0])?, value(xyz[1])?, value(xyz[2])?));
    }
    if out.len() < points {
        return Err(PcdError::Short {
            expected: points,
            got: out.len(),
        });
    }
    Ok(out)
}

/// Writes points as a binary PCD of float32 x, y, z (as `tools/ndt_map.py` does).
pub fn write(mut output: impl Write, points: &[Point]) -> io::Result<()> {
    let n = points.len();
    write!(
        output,
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n\
         SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\n\
         VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )?;
    let bytes: Vec<u8> = points
        .iter()
        .flat_map(|p| [p.x, p.y, p.z])
        .flat_map(|v| (v as f32).to_le_bytes())
        .collect();
    output.write_all(&bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn binary_round_trip() {
        let points = vec![Point::new(1.0, -2.5, 0.75), Point::new(40.0, 3.0, 1.5)];
        let mut file = Vec::new();
        write(&mut file, &points).expect("written");
        assert_eq!(read(file.as_slice()).expect("read"), points);
        // Cut short: says so.
        let short = &file[..file.len() - 4];
        assert!(matches!(read(short), Err(PcdError::Short { .. })));
    }

    #[test]
    fn ascii_with_other_fields() {
        let file = "VERSION 0.7\nFIELDS intensity x y z\nSIZE 4 4 4 4\nTYPE F F F F\n\
                    COUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n\
                    7 1 2 3\n8 4 5 6\n";
        let points = read(file.as_bytes()).expect("read");
        assert_eq!(
            points,
            vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)]
        );
    }

    #[test]
    fn compressed_is_refused() {
        let file = "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 0\nDATA binary_compressed\n";
        assert!(matches!(
            read(file.as_bytes()),
            Err(PcdError::Unsupported(_))
        ));
    }
}
