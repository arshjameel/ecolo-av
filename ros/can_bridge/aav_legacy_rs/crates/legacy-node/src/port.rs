//! The ESP32's port: a serial device (`/dev/ttyAMA0`, a pseudo-terminal in the tests), or
//! `socket://host:port`, as pyserial's URL, for Ravensim's `sim_esp32` (TCP).

use std::io::{self, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

/// How long a write may wait: a frame is 13 to 20 bytes, about 2 ms at 115200 baud.
const WRITE_TIMEOUT: Duration = Duration::from_millis(50);
/// How long connecting to `socket://` may take.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(1);

/// True if setting the speed failed with ENOTTY ("Not a typewriter"): a pseudo-terminal
/// on macOS.
fn is_not_a_tty(e: &serialport::Error) -> bool {
    e.description.contains("Not a typewriter") || e.description.contains("Inappropriate ioctl")
}

/// An open port.
pub enum Port {
    /// A serial device.
    Serial(Box<dyn serialport::SerialPort>),
    /// A TCP connection (`socket://host:port`).
    Tcp(TcpStream),
}

impl Port {
    /// Opens `name` at `baud` (ignored for `socket://`).
    pub fn open(name: &str, baud: u32) -> io::Result<Port> {
        if let Some(address) = name.strip_prefix("socket://") {
            let target = address
                .to_socket_addrs()?
                .next()
                .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "no address"))?;
            let stream = TcpStream::connect_timeout(&target, CONNECT_TIMEOUT)?;
            stream.set_nodelay(true)?;
            stream.set_write_timeout(Some(WRITE_TIMEOUT))?;
            return Ok(Port::Tcp(stream));
        }
        if name.contains("://") {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "only a device or socket://host:port",
            ));
        }
        let open = |baud| serialport::new(name, baud).timeout(WRITE_TIMEOUT).open();
        match open(baud) {
            Ok(serial) => Ok(Port::Serial(serial)),
            // macOS refuses to set a speed on a pseudo-terminal (the tests' port): open it
            // raw, its speed left alone (serialport skips that call for 0), as
            // aav_imu_yahboom does. A real serial port never gets here.
            Err(e) if is_not_a_tty(&e) => Ok(Port::Serial(open(0).map_err(io::Error::from)?)),
            Err(e) => Err(io::Error::from(e)),
        }
    }

    /// Writes once: how many bytes the kernel took (a short write is the caller's to see).
    pub fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        match self {
            Port::Serial(s) => s.write(bytes),
            Port::Tcp(t) => t.write(bytes),
        }
    }

    /// Reads what is waiting, without waiting. A closed connection is an error.
    pub fn read_waiting(&mut self) -> io::Result<Vec<u8>> {
        let mut buf = vec![0u8; 4096];
        match self {
            Port::Serial(s) => {
                let waiting = usize::try_from(s.bytes_to_read().map_err(io::Error::from)?)
                    .unwrap_or(buf.len());
                if waiting == 0 {
                    return Ok(Vec::new());
                }
                buf.truncate(waiting.min(buf.len()));
                let n = s.read(&mut buf)?;
                buf.truncate(n);
                Ok(buf)
            }
            Port::Tcp(t) => {
                t.set_nonblocking(true)?;
                let got = t.read(&mut buf);
                t.set_nonblocking(false)?;
                match got {
                    Ok(0) => Err(io::Error::new(
                        io::ErrorKind::UnexpectedEof,
                        "the other end closed the connection",
                    )),
                    Ok(n) => {
                        buf.truncate(n);
                        Ok(buf)
                    }
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(Vec::new()),
                    Err(e) => Err(e),
                }
            }
        }
    }
}
