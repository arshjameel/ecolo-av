//! The telemetry packet the sentinel receives as an I2C device at 0x42.
//!
//! Byte for byte the 2025-26 `aav_telemetry_t` (`AAV_cortex`, `aav_telemetry.hpp`): a
//! packed C struct, little-endian, 46 bytes, ending in an XOR checksum.
//!
//! | Bytes | Field | Type |
//! |---|---|---|
//! | 0 | magic, 0xAA | u8 |
//! | 1 | version, 1 | u8 |
//! | 2 | drive mode: 0 RC, 1 autonomous | u8 |
//! | 3 | drive state: 0 stop, 1 forward, 2 reverse, 3 brake | u8 |
//! | 4-15 | steering, speed, accel: -1 to 1 | 3 x f32 |
//! | 16, 17-18 | steering Jrk: ok, error flags | u8, u16 |
//! | 19, 20-21 | brake Jrk: ok, error flags | u8, u16 |
//! | 22, 23-24 | throttle DAC: ok, value 0-1023 | u8, u16 |
//! | 25-32 | iBUS: enable, throttle, steering, mode channels | 4 x u16 |
//! | 33 | RC signal: 1 ok | u8 |
//! | 34-35 | computers (bits 0-4) and ROS nodes (bits 5-9) up | u16 |
//! | 36-43 | latitude, longitude | 2 x f32 |
//! | 44 | error code | u8 |
//! | 45 | XOR of bytes 0 to 44 | u8 |

/// Bytes in a packet.
pub const LEN: usize = 46;
/// The first byte of every packet.
pub const MAGIC: u8 = 0xAA;
/// The packet layout this crate reads.
pub const VERSION: u8 = 0x01;

/// Why a write was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TelemetryError {
    /// No 0xAA in the write.
    NoMagic,
    /// From the 0xAA on, only this many bytes: a packet is 46.
    Short(usize),
    /// A packet layout this crate doesn't know.
    Version(u8),
    /// The checksum didn't match.
    Checksum { got: u8, expected: u8 },
}

/// Who drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveMode {
    Rc,
    Autonomous,
}

/// What the car is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveState {
    Stop,
    Forward,
    Reverse,
    Brake,
}

/// A group of a Jrk G2's "errors halting" flags (its `getErrorFlagsHalting()`), as the
/// display shows them. Bits from Pololu's Jrk G2 user's guide, 4.10, as in the `jrk-g2`
/// crate. (The 2025-26 page named five of its six bits wrongly: its "input" was bit 2, the
/// motor driver; its "over-current" bit 9, a serial buffer.)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JrkFlag {
    /// Bit 0: waiting for a command after a reset or an error.
    AwaitingCommand,
    /// Bit 1: no motor power.
    NoPower,
    /// Bit 2: the motor driver reported an error.
    MotorDriver,
    /// Bit 5: the feedback sensor is disconnected.
    FeedbackDisconnect,
    /// Bits 6 and 13: soft or hard over-current.
    OverCurrent,
    /// Bits 7 to 12: the command link (serial, I2C).
    Serial,
}

impl JrkFlag {
    /// All six, in the display's order.
    pub const ALL: [JrkFlag; 6] = [
        JrkFlag::AwaitingCommand,
        JrkFlag::NoPower,
        JrkFlag::MotorDriver,
        JrkFlag::FeedbackDisconnect,
        JrkFlag::OverCurrent,
        JrkFlag::Serial,
    ];

    /// The flag's bits.
    #[must_use]
    pub fn mask(self) -> u16 {
        match self {
            JrkFlag::AwaitingCommand => 1 << 0,
            JrkFlag::NoPower => 1 << 1,
            JrkFlag::MotorDriver => 1 << 2,
            JrkFlag::FeedbackDisconnect => 1 << 5,
            JrkFlag::OverCurrent => (1 << 6) | (1 << 13),
            JrkFlag::Serial => 0b11_1111 << 7,
        }
    }

    /// Three letters for the LCD.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            JrkFlag::AwaitingCommand => "AWT",
            JrkFlag::NoPower => "PWR",
            JrkFlag::MotorDriver => "DRV",
            JrkFlag::FeedbackDisconnect => "FBK",
            JrkFlag::OverCurrent => "OVC",
            JrkFlag::Serial => "SER",
        }
    }
}

/// A Jrk G2's error flags, as it reported them.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct JrkErrors(pub u16);

impl JrkErrors {
    #[must_use]
    pub fn has(self, flag: JrkFlag) -> bool {
        self.0 & flag.mask() != 0
    }
}

/// A Jrk G2 (steering or brake), as the actuator board saw it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Jrk {
    pub ok: bool,
    pub errors: JrkErrors,
}

/// The throttle DAC.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Dac {
    pub ok: bool,
    /// 0 to 1023.
    pub value: u16,
}

/// The RC receiver's raw channels, in microseconds (about 1000 to 2000).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Ibus {
    pub enable: u16,
    pub throttle: u16,
    pub steering: u16,
    pub mode: u16,
}

impl Ibus {
    /// The 2025-26 check: every channel above 100.
    #[must_use]
    pub fn channels_ok(&self) -> bool {
        [self.enable, self.throttle, self.steering, self.mode]
            .iter()
            .all(|&c| c > 100)
    }
}

/// A computer or a ROS node, each one LED on the bar.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unit {
    JetsonOrin,
    MotorPi,
    RadarPi,
    UiPi,
    CameraPi,
    Imu,
    Radar,
    Lidar,
    Camera,
    Gps,
}

impl Unit {
    pub const COMPUTERS: [Unit; 5] = [
        Unit::JetsonOrin,
        Unit::MotorPi,
        Unit::RadarPi,
        Unit::UiPi,
        Unit::CameraPi,
    ];
    pub const NODES: [Unit; 5] = [Unit::Imu, Unit::Radar, Unit::Lidar, Unit::Camera, Unit::Gps];

    /// Its bit in the status word, and its LED on the bar: 0 to 9.
    #[must_use]
    pub fn index(self) -> u8 {
        self as u8
    }

    /// Three letters for the LCD.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Unit::JetsonOrin => "JET",
            Unit::MotorPi => "MPI",
            Unit::RadarPi => "RPI",
            Unit::UiPi => "UPI",
            Unit::CameraPi => "CPI",
            Unit::Imu => "IMU",
            Unit::Radar => "RDR",
            Unit::Lidar => "LDR",
            Unit::Camera => "CAM",
            Unit::Gps => "GPS",
        }
    }
}

/// Which computers and ROS nodes are up.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Status(pub u16);

impl Status {
    #[must_use]
    pub fn is_up(self, unit: Unit) -> bool {
        self.0 & (1 << unit.index()) != 0
    }

    /// The ten LEDs of the bar, LED 0 first.
    #[must_use]
    pub fn leds(self) -> [bool; 10] {
        core::array::from_fn(|i| self.0 & (1 << i) != 0)
    }
}

/// The car's error, one at a time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCode {
    Ok,
    Imu,
    Barometer,
    GpsNoFix,
    BatteryLow,
    Motor,
    CommsLost,
    EStop,
    Unknown(u8),
}

impl ErrorCode {
    #[must_use]
    pub fn from_byte(b: u8) -> Self {
        match b {
            0x00 => ErrorCode::Ok,
            0x01 => ErrorCode::Imu,
            0x02 => ErrorCode::Barometer,
            0x03 => ErrorCode::GpsNoFix,
            0x04 => ErrorCode::BatteryLow,
            0x05 => ErrorCode::Motor,
            0x06 => ErrorCode::CommsLost,
            0xFF => ErrorCode::EStop,
            other => ErrorCode::Unknown(other),
        }
    }

    #[must_use]
    pub fn to_byte(self) -> u8 {
        match self {
            ErrorCode::Ok => 0x00,
            ErrorCode::Imu => 0x01,
            ErrorCode::Barometer => 0x02,
            ErrorCode::GpsNoFix => 0x03,
            ErrorCode::BatteryLow => 0x04,
            ErrorCode::Motor => 0x05,
            ErrorCode::CommsLost => 0x06,
            ErrorCode::EStop => 0xFF,
            ErrorCode::Unknown(b) => b,
        }
    }

    /// For the LCD: at most 16 characters.
    #[must_use]
    pub fn text(self) -> &'static str {
        match self {
            ErrorCode::Ok => "OK",
            ErrorCode::Imu => "IMU FAIL",
            ErrorCode::Barometer => "BARO FAIL",
            ErrorCode::GpsNoFix => "GPS NO FIX",
            ErrorCode::BatteryLow => "BATTERY LOW",
            ErrorCode::Motor => "MOTOR FAIL",
            ErrorCode::CommsLost => "COMMS LOST",
            ErrorCode::EStop => "!!! E-STOP !!!",
            ErrorCode::Unknown(_) => "UNKNOWN",
        }
    }
}

/// One packet, read.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Telemetry {
    pub drive_mode: DriveMode,
    pub drive_state: DriveState,
    /// -1 to 1, as the actuator board takes them.
    pub steering: f32,
    pub speed: f32,
    pub accel: f32,
    pub steering_jrk: Jrk,
    pub brake_jrk: Jrk,
    pub dac: Dac,
    pub ibus: Ibus,
    pub rc_active: bool,
    pub status: Status,
    /// Degrees.
    pub latitude: f32,
    pub longitude: f32,
    pub error: ErrorCode,
}

impl Telemetry {
    /// Read one I2C write. As the 2025-26 display did, bytes before the first 0xAA are
    /// skipped; after it must come a whole packet. Bytes after the packet are ignored.
    pub fn from_write(write: &[u8]) -> Result<Self, TelemetryError> {
        let start = write
            .iter()
            .position(|&b| b == MAGIC)
            .ok_or(TelemetryError::NoMagic)?;
        let rest = &write[start..];
        let packet: &[u8; LEN] = rest
            .get(..LEN)
            .and_then(|p| p.try_into().ok())
            .ok_or(TelemetryError::Short(rest.len()))?;
        Self::from_packet(packet)
    }

    /// Read exactly one packet.
    pub fn from_packet(p: &[u8; LEN]) -> Result<Self, TelemetryError> {
        if p[1] != VERSION {
            return Err(TelemetryError::Version(p[1]));
        }
        let expected = checksum(&p[..LEN - 1]);
        if p[LEN - 1] != expected {
            return Err(TelemetryError::Checksum {
                got: p[LEN - 1],
                expected,
            });
        }
        let u16_at = |i: usize| u16::from_le_bytes([p[i], p[i + 1]]);
        let f32_at = |i: usize| f32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        let jrk_at = |i: usize| Jrk {
            ok: p[i] != 0,
            errors: JrkErrors(u16_at(i + 1)),
        };
        Ok(Telemetry {
            // As before: anything but 1 is RC, and an unknown state shows as a stop.
            drive_mode: if p[2] == 1 {
                DriveMode::Autonomous
            } else {
                DriveMode::Rc
            },
            drive_state: match p[3] {
                1 => DriveState::Forward,
                2 => DriveState::Reverse,
                3 => DriveState::Brake,
                _ => DriveState::Stop,
            },
            steering: f32_at(4),
            speed: f32_at(8),
            accel: f32_at(12),
            steering_jrk: jrk_at(16),
            brake_jrk: jrk_at(19),
            dac: Dac {
                ok: p[22] != 0,
                value: u16_at(23),
            },
            ibus: Ibus {
                enable: u16_at(25),
                throttle: u16_at(27),
                steering: u16_at(29),
                mode: u16_at(31),
            },
            rc_active: p[33] != 0,
            status: Status(u16_at(34)),
            latitude: f32_at(36),
            longitude: f32_at(40),
            error: ErrorCode::from_byte(p[44]),
        })
    }

    /// The packet, as a sender builds it: for tests, and for whoever writes the sender.
    #[must_use]
    pub fn to_packet(&self) -> [u8; LEN] {
        let mut p = [0; LEN];
        p[0] = MAGIC;
        p[1] = VERSION;
        p[2] = match self.drive_mode {
            DriveMode::Rc => 0,
            DriveMode::Autonomous => 1,
        };
        p[3] = match self.drive_state {
            DriveState::Stop => 0,
            DriveState::Forward => 1,
            DriveState::Reverse => 2,
            DriveState::Brake => 3,
        };
        let mut put = |i: usize, bytes: &[u8]| p[i..i + bytes.len()].copy_from_slice(bytes);
        put(4, &self.steering.to_le_bytes());
        put(8, &self.speed.to_le_bytes());
        put(12, &self.accel.to_le_bytes());
        for (i, jrk) in [(16, self.steering_jrk), (19, self.brake_jrk)] {
            put(i, &[u8::from(jrk.ok)]);
            put(i + 1, &jrk.errors.0.to_le_bytes());
        }
        put(22, &[u8::from(self.dac.ok)]);
        put(23, &self.dac.value.to_le_bytes());
        let ibus = self.ibus;
        for (i, ch) in [ibus.enable, ibus.throttle, ibus.steering, ibus.mode]
            .iter()
            .enumerate()
        {
            put(25 + 2 * i, &ch.to_le_bytes());
        }
        put(33, &[u8::from(self.rc_active)]);
        put(34, &self.status.0.to_le_bytes());
        put(36, &self.latitude.to_le_bytes());
        put(40, &self.longitude.to_le_bytes());
        put(44, &[self.error.to_byte()]);
        p[LEN - 1] = checksum(&p[..LEN - 1]);
        p
    }
}

/// The 2025-26 checksum: every byte XORed together.
#[must_use]
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |acc, b| acc ^ b)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A packet with something in every field.
    pub(crate) fn sample() -> Telemetry {
        Telemetry {
            drive_mode: DriveMode::Autonomous,
            drive_state: DriveState::Forward,
            steering: -0.5,
            speed: 0.25,
            accel: -1.0,
            steering_jrk: Jrk {
                ok: true,
                errors: JrkErrors(0),
            },
            brake_jrk: Jrk {
                ok: false,
                errors: JrkErrors(0x0011),
            },
            dac: Dac {
                ok: true,
                value: 700,
            },
            ibus: Ibus {
                enable: 1000,
                throttle: 1500,
                steering: 1999,
                mode: 2000,
            },
            rc_active: true,
            status: Status(0b10_1101_0111),
            latitude: 45.3842,
            longitude: -75.6981,
            error: ErrorCode::Ok,
        }
    }

    /// The same packet, laid out by hand from the C struct, to check `to_packet`.
    fn by_hand() -> [u8; LEN] {
        let mut p = [0u8; LEN];
        p[..4].copy_from_slice(&[0xAA, 0x01, 1, 1]);
        p[4..8].copy_from_slice(&(-0.5f32).to_le_bytes());
        p[8..12].copy_from_slice(&0.25f32.to_le_bytes());
        p[12..16].copy_from_slice(&(-1.0f32).to_le_bytes());
        p[16..22].copy_from_slice(&[1, 0, 0, 0, 0x11, 0]);
        p[22..25].copy_from_slice(&[1, 0xBC, 0x02]); // 700
        p[25..33].copy_from_slice(&[0xE8, 0x03, 0xDC, 0x05, 0xCF, 0x07, 0xD0, 0x07]);
        p[33] = 1;
        p[34..36].copy_from_slice(&[0b1101_0111, 0b10]);
        p[36..40].copy_from_slice(&45.3842f32.to_le_bytes());
        p[40..44].copy_from_slice(&(-75.6981f32).to_le_bytes());
        p[44] = 0;
        p[45] = p[..45].iter().fold(0, |a, b| a ^ b);
        p
    }

    #[test]
    fn the_layout_is_the_c_structs() {
        assert_eq!(sample().to_packet(), by_hand());
        assert_eq!(Telemetry::from_packet(&by_hand()), Ok(sample()));
    }

    #[test]
    fn leading_bytes_are_skipped_and_trailing_ones_ignored() {
        let mut write = [0u8; 3 + LEN + 2];
        write[3..3 + LEN].copy_from_slice(&by_hand());
        assert_eq!(Telemetry::from_write(&write), Ok(sample()));
    }

    #[test]
    fn bad_writes_are_refused() {
        let good = by_hand();
        assert_eq!(
            Telemetry::from_write(&[1, 2, 3]),
            Err(TelemetryError::NoMagic)
        );
        assert_eq!(
            Telemetry::from_write(&good[..20]),
            Err(TelemetryError::Short(20))
        );
        let mut bad = good;
        bad[10] ^= 0x40;
        assert!(matches!(
            Telemetry::from_write(&bad),
            Err(TelemetryError::Checksum { .. })
        ));
        let mut v2 = good;
        v2[1] = 2;
        v2[45] = checksum(&v2[..45]);
        assert_eq!(Telemetry::from_write(&v2), Err(TelemetryError::Version(2)));
    }

    #[test]
    fn unknown_modes_read_as_before() {
        let mut p = by_hand();
        p[2] = 7; // not 1: RC
        p[3] = 9; // unknown: stop
        p[44] = 0x42;
        p[45] = checksum(&p[..45]);
        let t = Telemetry::from_packet(&p).unwrap();
        assert_eq!(t.drive_mode, DriveMode::Rc);
        assert_eq!(t.drive_state, DriveState::Stop);
        assert_eq!(t.error, ErrorCode::Unknown(0x42));
    }

    #[test]
    fn status_bits_are_the_led_bar() {
        let s = sample().status;
        assert!(s.is_up(Unit::JetsonOrin));
        assert!(!s.is_up(Unit::UiPi));
        assert!(s.is_up(Unit::Gps));
        assert!(!s.is_up(Unit::Camera));
        assert_eq!(
            s.leds(),
            [true, true, true, false, true, false, true, true, false, true]
        );
    }

    #[test]
    fn jrk_flags_use_pololus_bits() {
        let e = JrkErrors(0x0011); // bits 0 and 4
        assert!(e.has(JrkFlag::AwaitingCommand));
        assert!(!e.has(JrkFlag::FeedbackDisconnect)); // bit 4 is the input's
        assert!(JrkErrors(1 << 13).has(JrkFlag::OverCurrent));
        assert!(JrkErrors(1 << 9).has(JrkFlag::Serial));
        assert!(!JrkErrors(1 << 9).has(JrkFlag::OverCurrent));
    }

    #[test]
    fn error_codes_round_trip() {
        for b in 0..=255u8 {
            assert_eq!(ErrorCode::from_byte(b).to_byte(), b);
        }
    }

    #[test]
    fn ibus_channels_need_a_signal() {
        assert!(sample().ibus.channels_ok());
        let mut quiet = sample().ibus;
        quiet.mode = 0;
        assert!(!quiet.channels_ok());
    }
}
