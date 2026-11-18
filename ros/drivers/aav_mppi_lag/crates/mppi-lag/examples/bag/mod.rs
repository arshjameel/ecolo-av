//! A recording's plans (TODO 202): every `nav_msgs/Path` on a topic of an MCAP file (what
//! `tools/record.sh` writes), as poses, so `follow_plan` can drive the sim's or the car's
//! own plans again offline. Shared by the example and `tests/bag.rs`
//! (which writes the other way, to check it).
//!
//! The messages are ROS 2's CDR (little or big endian, by the 4-byte header): a Header
//! (stamp, frame id), then a sequence of PoseStamped (Header, position, orientation).

use mppi_lag::Pose;

/// One plan from the recording: when it was logged (s from the first message) and its poses.
pub struct Plan {
    pub time: f64,
    pub frame: String,
    pub poses: Vec<Pose>,
}

/// Every `nav_msgs/Path` on `topic` in the MCAP bytes `mcap`, in log order.
pub fn plans(mcap: &[u8], topic: &str) -> Result<Vec<Plan>, String> {
    let mut first = None;
    let mut found = Vec::new();
    for message in mcap::MessageStream::new(mcap).map_err(|e| e.to_string())? {
        let message = message.map_err(|e| e.to_string())?;
        let start = *first.get_or_insert(message.log_time);
        if message.channel.topic != topic {
            continue;
        }
        let (frame, poses) = decode_path(&message.data)?;
        found.push(Plan {
            time: message.log_time.saturating_sub(start) as f64 * 1e-9,
            frame,
            poses,
        });
    }
    Ok(found)
}

/// A CDR reader: each field aligned to its own size, counted from after the 4-byte header.
struct Cdr<'a> {
    data: &'a [u8],
    at: usize,
    little: bool,
}

impl<'a> Cdr<'a> {
    fn new(bytes: &'a [u8]) -> Result<Self, String> {
        match bytes {
            [0, kind, _, _, data @ ..] if *kind <= 1 => Ok(Self {
                data,
                at: 0,
                little: *kind == 1,
            }),
            _ => Err("not a CDR message".into()),
        }
    }

    fn take<const N: usize>(&mut self) -> Result<[u8; N], String> {
        self.at = self.at.next_multiple_of(N);
        let bytes = self
            .data
            .get(self.at..self.at + N)
            .ok_or("the message ends too soon")?;
        self.at += N;
        let mut out: [u8; N] = bytes.try_into().map_err(|_| "never")?;
        if !self.little {
            out.reverse();
        }
        Ok(out)
    }

    fn u32(&mut self) -> Result<u32, String> {
        self.take::<4>().map(u32::from_le_bytes)
    }

    fn f64(&mut self) -> Result<f64, String> {
        self.take::<8>().map(f64::from_le_bytes)
    }

    fn string(&mut self) -> Result<String, String> {
        let n = self.u32()? as usize;
        let bytes = self
            .data
            .get(self.at..self.at + n)
            .ok_or("a string runs past the end")?;
        self.at += n;
        Ok(String::from_utf8_lossy(bytes.strip_suffix(&[0]).unwrap_or(bytes)).into_owned())
    }

    /// std_msgs/Header: the stamp (skipped) and the frame id.
    fn header(&mut self) -> Result<String, String> {
        self.u32()?;
        self.u32()?;
        self.string()
    }
}

/// A `nav_msgs/Path` in CDR: its frame and its poses (yaw from the quaternion).
pub fn decode_path(bytes: &[u8]) -> Result<(String, Vec<Pose>), String> {
    let mut cdr = Cdr::new(bytes)?;
    let frame = cdr.header()?;
    let n = cdr.u32()? as usize;
    let poses = (0..n)
        .map(|_| {
            cdr.header()?;
            let (x, y, _z) = (cdr.f64()?, cdr.f64()?, cdr.f64()?);
            let (qx, qy, qz, qw) = (cdr.f64()?, cdr.f64()?, cdr.f64()?, cdr.f64()?);
            let yaw = (2.0 * (qw * qz + qx * qy)).atan2(1.0 - 2.0 * (qy * qy + qz * qz));
            Ok(Pose::new(x, y, yaw))
        })
        .collect::<Result<_, String>>()?;
    Ok((frame, poses))
}
