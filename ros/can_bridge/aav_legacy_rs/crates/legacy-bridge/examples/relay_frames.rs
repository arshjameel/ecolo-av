//! The Rust bridge's frames, one per line, for tools/esp32_qemu.py's `--bridge rust`: the
//! QEMU check drives the emulated Rust ESP32 with what this bridge's `Relay` writes.
//!
//! stdin, one line each:  `<speed m/s> <steering rad> <counter 0..255>`
//! stdout, one line each: the frame's bytes in hex (frame_version 2; `--version 1|2|3`)
//!
//! Each line is ego's command, fresh, with the Orin's counter as it last came: the relay
//! then writes the frame legacy_uart_bridge would. A bad line ends it with an error.

use std::io::{self, BufRead, Write};

use legacy_bridge::{FrameVersion, Relay, RelaySettings};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let version = match args.iter().position(|a| a == "--version") {
        Some(i) => FrameVersion::try_from(args.get(i + 1).ok_or("--version N")?.parse::<i64>()?)?,
        None => FrameVersion::V2,
    };
    let mut relay = Relay::new(
        RelaySettings {
            version,
            ..RelaySettings::default()
        },
        None,
    );
    let stdout = io::stdout();
    let mut out = stdout.lock();
    for line in io::stdin().lock().lines() {
        let line = line?;
        let words: Vec<&str> = line.split_whitespace().collect();
        let [speed, steering, counter] = words[..] else {
            return Err(format!("want <speed> <steering> <counter>, got {line:?}").into());
        };
        relay.command(speed.parse()?, steering.parse()?, 0.0);
        relay.heartbeat(counter.parse()?);
        let hex: String = relay
            .frame(0.0)
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        writeln!(out, "{hex}")?;
        out.flush()?;
    }
    Ok(())
}
