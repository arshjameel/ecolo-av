//! Just enough of a WebSocket client (RFC 6455) to read the ESP32's status lines, with no
//! network in it: the address, the opening request and its answer, and the frames. The
//! app's `esp32` thread owns the socket.
//!
//! ```text
//! ws://192.168.4.1:81/ ──► Url ──► request() ──► the board ──► check_answer()
//! bytes ──► Decoder::push ──► Decoder::message() ──► Message::Text (lines) / Ping / Close
//! ```
//!
//! Read only: the only frames this client sends are the protocol's own, a pong for a
//! ping and a close ([`client_frame`]), never a command.
//! Not done: TLS (`wss://`), and checking `Sec-WebSocket-Accept` (it needs SHA-1; the
//! answer's 101 and `Upgrade: websocket` are checked).

use std::fmt;

use thiserror::Error;

/// Why a WebSocket can't be used.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum WsError {
    /// Not a ws:// address.
    #[error("not a ws:// address: {0:?} (such as ws://192.168.4.1:81/)")]
    Address(String),
    /// wss://: TLS isn't done here.
    #[error("wss:// needs TLS, which this panel doesn't do: use ws://")]
    Tls,
    /// The board didn't switch to a WebSocket: its answer.
    #[error("the board didn't switch to a WebSocket: {0}")]
    Refused(String),
    /// A frame bigger than [`MAX_MESSAGE`] (bytes).
    #[error("a frame of {0} bytes: more than this panel takes")]
    TooBig(u64),
    /// An opcode RFC 6455 doesn't define.
    #[error("a frame with opcode {0:#x}, which RFC 6455 doesn't define")]
    Opcode(u8),
}

/// A `ws://` address.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Url {
    /// The host name or address.
    pub host: String,
    /// The port.
    pub port: u16,
    /// Starts with `/`.
    pub path: String,
}

impl Url {
    /// A `ws://` address from text.
    ///
    /// # Errors
    ///
    /// [`WsError`] if it isn't a ws:// address, or is wss://.
    pub fn parse(s: &str) -> Result<Self, WsError> {
        let bad = || WsError::Address(s.to_owned());
        if s.starts_with("wss://") {
            return Err(WsError::Tls);
        }
        let rest = s.strip_prefix("ws://").ok_or_else(bad)?;
        let (authority, path) = match rest.find('/') {
            Some(i) => (&rest[..i], &rest[i..]),
            None => (rest, "/"),
        };
        let (host, port) = match authority.rsplit_once(':') {
            Some((h, p)) => (h, p.parse().map_err(|_| bad())?),
            None => (authority, 80),
        };
        if host.is_empty() || host.contains(char::is_whitespace) {
            return Err(bad());
        }
        Ok(Self {
            host: host.to_owned(),
            port,
            path: path.to_owned(),
        })
    }
}

impl fmt::Display for Url {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "ws://{}:{}{}", self.host, self.port, self.path)
    }
}

/// The opening request. `nonce`: 16 bytes, different each time.
#[must_use]
pub fn request(url: &Url, nonce: [u8; 16]) -> String {
    format!(
        "GET {} HTTP/1.1\r\nHost: {}:{}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n\
         Sec-WebSocket-Key: {}\r\nSec-WebSocket-Version: 13\r\n\r\n",
        url.path,
        url.host,
        url.port,
        base64(&nonce)
    )
}

/// Checks the answer's head (up to the blank line): 101, and `Upgrade: websocket`.
///
/// # Errors
///
/// [`WsError::Refused`] if the answer isn't 101 with `Upgrade: websocket`.
pub fn check_answer(head: &str) -> Result<(), WsError> {
    let first = head.lines().next().unwrap_or_default();
    let switched = first
        .split_whitespace()
        .nth(1)
        .is_some_and(|code| code == "101");
    let upgrade = head.lines().any(|l| {
        l.split_once(':').is_some_and(|(k, v)| {
            k.trim().eq_ignore_ascii_case("upgrade") && v.trim().eq_ignore_ascii_case("websocket")
        })
    });
    if switched && upgrade {
        Ok(())
    } else {
        Err(WsError::Refused(first.to_owned()))
    }
}

/// Standard base64, with padding.
#[must_use]
pub fn base64(bytes: &[u8]) -> String {
    const TABLE: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b = [
            chunk[0],
            chunk.get(1).copied().unwrap_or(0),
            chunk.get(2).copied().unwrap_or(0),
        ];
        let n = u32::from(b[0]) << 16 | u32::from(b[1]) << 8 | u32::from(b[2]);
        let sextets = [n >> 18, n >> 12, n >> 6, n].map(|v| char::from(TABLE[(v & 63) as usize]));
        let keep = chunk.len() + 1;
        for (i, c) in sextets.into_iter().enumerate() {
            out.push(if i < keep { c } else { '=' });
        }
    }
    out
}

/// A frame's kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Opcode {
    /// More of a message.
    Continuation,
    /// A text message.
    Text,
    /// A binary message.
    Binary,
    /// Close.
    Close,
    /// Ping.
    Ping,
    /// Pong.
    Pong,
}

impl Opcode {
    fn from_bits(b: u8) -> Result<Self, WsError> {
        Ok(match b {
            0x0 => Self::Continuation,
            0x1 => Self::Text,
            0x2 => Self::Binary,
            0x8 => Self::Close,
            0x9 => Self::Ping,
            0xA => Self::Pong,
            other => return Err(WsError::Opcode(other)),
        })
    }

    fn bits(self) -> u8 {
        match self {
            Self::Continuation => 0x0,
            Self::Text => 0x1,
            Self::Binary => 0x2,
            Self::Close => 0x8,
            Self::Ping => 0x9,
            Self::Pong => 0xA,
        }
    }
}

/// A whole message, its fragments joined.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Message {
    /// Lossy UTF-8: a broken byte becomes U+FFFD, and that line won't parse.
    Text(String),
    /// Binary bytes.
    Binary(Vec<u8>),
    /// A ping, with its bytes.
    Ping(Vec<u8>),
    /// A pong.
    Pong,
    /// The board closes.
    Close,
}

/// The largest message this panel takes: a status line is about 500 bytes.
pub const MAX_MESSAGE: u64 = 1 << 20;

/// Turns the bytes read into messages.
#[derive(Debug, Default)]
pub struct Decoder {
    buf: Vec<u8>,
    /// A text or binary message in pieces: its kind and what came so far.
    partial: Option<(Opcode, Vec<u8>)>,
}

impl Decoder {
    /// Adds bytes read from the socket.
    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// The next whole message, if its bytes are all here.
    pub fn message(&mut self) -> Option<Result<Message, WsError>> {
        loop {
            let (fin, opcode, payload) = match self.frame()? {
                Ok(f) => f,
                Err(e) => return Some(Err(e)),
            };
            let message = match opcode {
                Opcode::Ping => Message::Ping(payload),
                Opcode::Pong => Message::Pong,
                Opcode::Close => Message::Close,
                Opcode::Text | Opcode::Binary if fin => whole(opcode, payload),
                Opcode::Text | Opcode::Binary => {
                    self.partial = Some((opcode, payload));
                    continue;
                }
                Opcode::Continuation => {
                    let Some((kind, mut so_far)) = self.partial.take() else {
                        continue; // a piece of nothing: dropped
                    };
                    so_far.extend_from_slice(&payload);
                    if so_far.len() as u64 > MAX_MESSAGE {
                        return Some(Err(WsError::TooBig(so_far.len() as u64)));
                    }
                    if !fin {
                        self.partial = Some((kind, so_far));
                        continue;
                    }
                    whole(kind, so_far)
                }
            };
            return Some(Ok(message));
        }
    }

    /// One frame off the front of the buffer: (fin, opcode, unmasked payload).
    fn frame(&mut self) -> Option<Result<Frame, WsError>> {
        let b = &self.buf;
        if b.len() < 2 {
            return None;
        }
        let fin = b[0] & 0x80 != 0;
        let opcode = match Opcode::from_bits(b[0] & 0x0F) {
            Ok(o) => o,
            Err(e) => return Some(Err(e)),
        };
        let masked = b[1] & 0x80 != 0;
        let (len, mut at) = match b[1] & 0x7F {
            126 => {
                let bytes: [u8; 2] = b.get(2..4)?.try_into().ok()?;
                (u64::from(u16::from_be_bytes(bytes)), 4)
            }
            127 => {
                let bytes: [u8; 8] = b.get(2..10)?.try_into().ok()?;
                (u64::from_be_bytes(bytes), 10)
            }
            n => (u64::from(n), 2),
        };
        if len > MAX_MESSAGE {
            return Some(Err(WsError::TooBig(len)));
        }
        let mask: Option<[u8; 4]> = if masked {
            let m = b.get(at..at + 4)?.try_into().ok()?;
            at += 4;
            Some(m)
        } else {
            None
        };
        // At most MAX_MESSAGE, so it fits a usize.
        let end = at + len as usize;
        let mut payload = b.get(at..end)?.to_vec();
        if let Some(m) = mask {
            payload
                .iter_mut()
                .zip(m.iter().cycle())
                .for_each(|(byte, k)| *byte ^= k);
        }
        self.buf.drain(..end);
        Some(Ok((fin, opcode, payload)))
    }
}

/// One frame: whether it is a message's last, its kind, its payload (unmasked).
type Frame = (bool, Opcode, Vec<u8>);

fn whole(kind: Opcode, payload: Vec<u8>) -> Message {
    match kind {
        Opcode::Text => Message::Text(String::from_utf8_lossy(&payload).into_owned()),
        _ => Message::Binary(payload),
    }
}

/// A frame from this client: always masked, as RFC 6455 asks. Only for the protocol's
/// own replies (a pong, a close).
#[must_use]
pub fn client_frame(opcode: Opcode, payload: &[u8], mask: [u8; 4]) -> Vec<u8> {
    let mut out = vec![0x80 | opcode.bits()];
    match payload.len() {
        n @ 0..=125 => out.push(0x80 | n as u8),
        n @ 126..=0xFFFF => {
            out.push(0x80 | 126);
            out.extend_from_slice(&(n as u16).to_be_bytes());
        }
        n => {
            out.push(0x80 | 127);
            out.extend_from_slice(&(n as u64).to_be_bytes());
        }
    }
    out.extend_from_slice(&mask);
    out.extend(payload.iter().zip(mask.iter().cycle()).map(|(b, k)| b ^ k));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame as the board sends it: unmasked.
    fn server_frame(fin: bool, opcode: Opcode, payload: &[u8]) -> Vec<u8> {
        let mut out = vec![if fin { 0x80 } else { 0 } | opcode.bits()];
        match payload.len() {
            n @ 0..=125 => out.push(n as u8),
            n @ 126..=0xFFFF => {
                out.push(126);
                out.extend_from_slice(&(n as u16).to_be_bytes());
            }
            n => {
                out.push(127);
                out.extend_from_slice(&(n as u64).to_be_bytes());
            }
        }
        out.extend_from_slice(payload);
        out
    }

    #[test]
    fn addresses() {
        let u = Url::parse("ws://192.168.4.1:81/").expect("valid");
        assert_eq!(
            (u.host.as_str(), u.port, u.path.as_str()),
            ("192.168.4.1", 81, "/")
        );
        let u = Url::parse("ws://aav-esp32.local").expect("valid");
        assert_eq!((u.port, u.path.as_str()), (80, "/"));
        assert_eq!(u.to_string(), "ws://aav-esp32.local:80/");
        let u = Url::parse("ws://h:81/status").expect("valid");
        assert_eq!(u.path, "/status");
        assert_eq!(Url::parse("wss://h/"), Err(WsError::Tls));
        assert!(Url::parse("http://h/").is_err());
        assert!(Url::parse("ws://h:port/").is_err());
        assert!(Url::parse("ws://:81/").is_err());
    }

    #[test]
    fn the_opening_and_its_answer() {
        let url = Url::parse("ws://192.168.4.1:81/").expect("valid");
        // RFC 6455's own example nonce.
        let nonce = *b"the sample nonce";
        let req = request(&url, nonce);
        assert!(req.starts_with("GET / HTTP/1.1\r\n"));
        assert!(req.contains("Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"));
        assert!(req.ends_with("\r\n\r\n"));
        let ok =
            "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n";
        assert_eq!(check_answer(ok), Ok(()));
        assert!(check_answer("HTTP/1.1 404 Not Found\r\n").is_err());
        assert!(check_answer("HTTP/1.1 101 x\r\nUpgrade: h2c\r\n").is_err());
    }

    #[test]
    fn base64_as_the_rfc() {
        assert_eq!(base64(b""), "");
        assert_eq!(base64(b"f"), "Zg==");
        assert_eq!(base64(b"fo"), "Zm8=");
        assert_eq!(base64(b"foo"), "Zm9v");
        assert_eq!(base64(b"foobar"), "Zm9vYmFy");
    }

    #[test]
    fn frames_arrive_in_any_pieces() {
        let line = r#"{"t":"hb","schema":1,"seq":1,"up_ms":1}"#;
        let mut bytes = server_frame(true, Opcode::Text, line.as_bytes());
        bytes.extend(server_frame(true, Opcode::Ping, b"hi"));
        let mut d = Decoder::default();
        // One byte at a time: nothing until a frame is whole.
        let mut got = Vec::new();
        for b in &bytes {
            d.push(&[*b]);
            while let Some(m) = d.message() {
                got.push(m.expect("valid"));
            }
        }
        assert_eq!(
            got,
            vec![Message::Text(line.into()), Message::Ping(b"hi".to_vec())]
        );
    }

    #[test]
    fn long_and_fragmented_messages_join() {
        let long = "x".repeat(300);
        let mut d = Decoder::default();
        d.push(&server_frame(true, Opcode::Text, long.as_bytes()));
        assert_eq!(d.message(), Some(Ok(Message::Text(long))));
        d.push(&server_frame(false, Opcode::Text, b"ab"));
        d.push(&server_frame(true, Opcode::Ping, b""));
        d.push(&server_frame(true, Opcode::Continuation, b"cd"));
        // A ping may come between the pieces.
        assert_eq!(d.message(), Some(Ok(Message::Ping(vec![]))));
        assert_eq!(d.message(), Some(Ok(Message::Text("abcd".into()))));
        assert_eq!(d.message(), None);
        d.push(&server_frame(true, Opcode::Close, b""));
        assert_eq!(d.message(), Some(Ok(Message::Close)));
    }

    #[test]
    fn bad_frames_are_errors() {
        let mut d = Decoder::default();
        d.push(&[0x83, 0x00]);
        assert_eq!(d.message(), Some(Err(WsError::Opcode(3))));
        let mut d = Decoder::default();
        let mut huge = vec![0x81, 127];
        huge.extend_from_slice(&(MAX_MESSAGE + 1).to_be_bytes());
        d.push(&huge);
        assert_eq!(d.message(), Some(Err(WsError::TooBig(MAX_MESSAGE + 1))));
    }

    #[test]
    fn our_frames_are_masked_and_a_decoder_reads_them_back() {
        let f = client_frame(Opcode::Pong, b"hi", [1, 2, 3, 4]);
        assert_eq!(f[0], 0x8A);
        assert_eq!(f[1], 0x80 | 2);
        let mut d = Decoder::default();
        d.push(&client_frame(Opcode::Text, b"hello", [9, 8, 7, 6]));
        assert_eq!(d.message(), Some(Ok(Message::Text("hello".into()))));
    }
}
