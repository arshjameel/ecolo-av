//! The pipe between the planner node and its model worker: the same bytes as
//! `aav_diffusion_planner/wire.py`.
//!
//! One JSON line of settings (`{"model": ..., "provider": ...}`), then for each plan:
//!
//! | | Header (little-endian) | Then (float32) |
//! |---|---|---|
//! | request | seq (u32), k samples (u32), 0.0 (f32) | crop (SIZE x SIZE), cond (COND), noise (k x POINTS x CHANNELS) |
//! | reply | seq, k (0: the model failed), the model's ms | paths (k x POINTS x CHANNELS) |

use std::io::{BufRead, Read, Write};

use crate::{Error, Result, CHANNELS, COND, POINTS, SIZE};

/// The worker's settings: the model's file and the provider's name (ONNX Runtime's, as
/// the Python worker takes it, or ours: `coreml`, `auto`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Settings {
    /// The `.onnx` file.
    pub model: String,
    /// The provider, as given.
    pub provider: String,
}

/// Read the settings line; `Ok(None)` if the pipe closed first.
pub fn read_settings(input: &mut impl BufRead) -> Result<Option<Settings>> {
    let mut line = String::new();
    if input.read_line(&mut line)? == 0 {
        return Ok(None);
    }
    let v: serde_json::Value =
        serde_json::from_str(&line).map_err(|e| Error::Settings(e.to_string()))?;
    let model = v["model"]
        .as_str()
        .ok_or_else(|| Error::Settings("no model".into()))?
        .to_owned();
    let provider = v["provider"].as_str().unwrap_or("cpu").to_owned();
    Ok(Some(Settings { model, provider }))
}

/// One plan's request.
#[derive(Debug, Clone, PartialEq)]
pub struct Request {
    /// Its number: a reply too late for its plan is known by it.
    pub seq: u32,
    /// The costmap crop, SIZE x SIZE.
    pub crop: Vec<f32>,
    /// The cond, COND numbers.
    pub cond: Vec<f32>,
    /// k samples of POINTS x CHANNELS.
    pub noise: Vec<f32>,
}

fn floats(input: &mut impl Read, n: usize) -> std::io::Result<Vec<f32>> {
    let mut bytes = vec![0u8; 4 * n];
    input.read_exact(&mut bytes)?;
    Ok(bytes
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect())
}

fn header(input: &mut impl Read) -> std::io::Result<(u32, u32, f32)> {
    let mut b = [0u8; 12];
    input.read_exact(&mut b)?;
    Ok((
        u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
        u32::from_le_bytes([b[4], b[5], b[6], b[7]]),
        f32::from_le_bytes([b[8], b[9], b[10], b[11]]),
    ))
}

/// Read one request; `Ok(None)` if the pipe closed before it began.
pub fn read_request(input: &mut impl Read) -> Result<Option<Request>> {
    let (seq, k, _) = match header(input) {
        Ok(h) => h,
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e.into()),
    };
    Ok(Some(Request {
        seq,
        crop: floats(input, SIZE * SIZE)?,
        cond: floats(input, COND)?,
        noise: floats(input, k as usize * POINTS * CHANNELS)?,
    }))
}

/// Write a request (the node's side; for tests and the bench).
pub fn write_request(out: &mut impl Write, request: &Request) -> Result<()> {
    let k = (request.noise.len() / (POINTS * CHANNELS)) as u32;
    write_header(out, request.seq, k, 0.0)?;
    for v in request
        .crop
        .iter()
        .chain(&request.cond)
        .chain(&request.noise)
    {
        out.write_all(&v.to_le_bytes())?;
    }
    Ok(())
}

fn write_header(out: &mut impl Write, seq: u32, k: u32, ms: f32) -> std::io::Result<()> {
    out.write_all(&seq.to_le_bytes())?;
    out.write_all(&k.to_le_bytes())?;
    out.write_all(&ms.to_le_bytes())
}

/// Write a reply: the paths, or `None` (k = 0) when the model failed; then flush.
pub fn write_reply(out: &mut impl Write, seq: u32, paths: Option<&[f32]>, ms: f32) -> Result<()> {
    let paths = paths.unwrap_or(&[]);
    write_header(out, seq, (paths.len() / (POINTS * CHANNELS)) as u32, ms)?;
    for v in paths {
        out.write_all(&v.to_le_bytes())?;
    }
    out.flush()?;
    Ok(())
}

/// Read a reply: (seq, paths or `None` if the model failed, ms).
pub fn read_reply(input: &mut impl Read) -> Result<(u32, Option<Vec<f32>>, f32)> {
    let (seq, k, ms) = header(input)?;
    if k == 0 {
        return Ok((seq, None, ms));
    }
    Ok((
        seq,
        Some(floats(input, k as usize * POINTS * CHANNELS)?),
        ms,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_request_and_a_reply_go_round() {
        let request = Request {
            seq: 7,
            crop: (0..SIZE * SIZE).map(|i| i as f32 / 100.0).collect(),
            cond: vec![0.5; COND],
            noise: vec![-1.25; 3 * POINTS * CHANNELS],
        };
        let mut bytes = Vec::new();
        write_request(&mut bytes, &request).unwrap();
        // wire.py's size: a 12-byte header, then the floats.
        assert_eq!(
            bytes.len(),
            12 + 4 * (SIZE * SIZE + COND + 3 * POINTS * CHANNELS)
        );
        let back = read_request(&mut bytes.as_slice()).unwrap().unwrap();
        assert_eq!(back, request);

        let mut reply = Vec::new();
        write_reply(&mut reply, 7, Some(&request.noise), 12.5).unwrap();
        let (seq, paths, ms) = read_reply(&mut reply.as_slice()).unwrap();
        assert_eq!((seq, ms), (7, 12.5));
        assert_eq!(paths.unwrap(), request.noise);

        let mut failed = Vec::new();
        write_reply(&mut failed, 8, None, 1.0).unwrap();
        assert_eq!(failed.len(), 12);
        assert_eq!(read_reply(&mut failed.as_slice()).unwrap().1, None);
    }

    #[test]
    fn a_closed_pipe_is_the_end_not_an_error() {
        assert!(read_request(&mut [].as_slice()).unwrap().is_none());
        assert!(read_settings(&mut [].as_slice()).unwrap().is_none());
    }

    #[test]
    fn settings_as_the_node_sends_them() {
        let line = br#"{"model": "/m/planner.onnx", "provider": "CoreMLExecutionProvider"}
"#;
        let s = read_settings(&mut line.as_slice()).unwrap().unwrap();
        assert_eq!(s.model, "/m/planner.onnx");
        assert_eq!(s.provider, "CoreMLExecutionProvider");
        assert!(read_settings(&mut b"{}\n".as_slice()).is_err());
    }
}
