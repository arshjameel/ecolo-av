//! The LCD's 13 pages: what each shows, from the latest packet.
//!
//! The knob turns through them; page 0, the overview, is home. A `*` in the top line's
//! last column means the page is pinned: it stays, instead of going home after 10 s.

use crate::lcd_line;
use crate::screen::{Line, Screen};
use crate::telemetry::{DriveMode, DriveState, Jrk, JrkFlag, Status, Telemetry, Unit};

/// One page of the LCD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Page {
    /// Who drives, and what the car does.
    Overview,
    /// Steering, speed and acceleration, as commanded.
    Drive,
    /// The steering Jrk: OK or not, its flags in hex.
    SteeringJrk,
    /// The brake Jrk: OK or not, its flags in hex.
    BrakeJrk,
    /// The steering Jrk's error flags, one by one.
    SteeringFlags,
    /// The brake Jrk's error flags, one by one.
    BrakeFlags,
    /// The throttle DAC.
    Throttle,
    /// The RC receiver's four channels.
    Ibus,
    /// The RC signal.
    Rc,
    /// Which computers are up.
    Computers,
    /// Which ROS nodes are up.
    Nodes,
    /// Latitude and longitude.
    Gps,
    /// The car's error code.
    Error,
}

impl Page {
    /// In the knob's order.
    pub const ALL: [Page; 13] = [
        Page::Overview,
        Page::Drive,
        Page::SteeringJrk,
        Page::BrakeJrk,
        Page::SteeringFlags,
        Page::BrakeFlags,
        Page::Throttle,
        Page::Ibus,
        Page::Rc,
        Page::Computers,
        Page::Nodes,
        Page::Gps,
        Page::Error,
    ];

    /// Home.
    pub const HOME: Page = Page::Overview;

    /// The page `steps` on (back, if negative), round the end.
    #[must_use]
    pub fn turn(self, steps: i8) -> Page {
        let n = Page::ALL.len() as i32;
        let here = self as i32;
        Page::ALL[(here + i32::from(steps)).rem_euclid(n) as usize]
    }

    /// What it shows, from the latest packet (or a waiting message before the first).
    #[must_use]
    pub fn render(self, telemetry: Option<&Telemetry>, pinned: bool) -> Screen {
        let Some(t) = telemetry else {
            return Screen {
                top: lcd_line!("  AAV SENTINEL"),
                bottom: lcd_line!(" Waiting for Pi"),
            };
        };
        let (top, bottom) = match self {
            Page::Overview => (
                lcd_line!("MODE:{}", mode(t.drive_mode)),
                lcd_line!("STATE:{}", state(t.drive_state)),
            ),
            Page::Drive => (
                lcd_line!("STR:{:+.2}", t.steering),
                lcd_line!("SPD:{:+.2} A:{:+.1}", t.speed, t.accel),
            ),
            Page::SteeringJrk => jrk_page("STEER", t.steering_jrk),
            Page::BrakeJrk => jrk_page("BRAKE", t.brake_jrk),
            Page::SteeringFlags => flags_page("STR", t.steering_jrk),
            Page::BrakeFlags => flags_page("BRK", t.brake_jrk),
            Page::Throttle => (
                lcd_line!("DAC:{}", ok(t.dac.ok)),
                lcd_line!("VAL:{:>5}/1023", t.dac.value),
            ),
            Page::Ibus => (
                lcd_line!("EN:{:<4} TH:{:<4}", t.ibus.enable, t.ibus.throttle),
                lcd_line!("ST:{:<4} MD:{:<4}", t.ibus.steering, t.ibus.mode),
            ),
            Page::Rc => (
                lcd_line!("RC:{}", if t.rc_active { "ACTIVE" } else { "LOST" }),
                lcd_line!(
                    "CHANNELS:{}",
                    if t.ibus.channels_ok() { "OK" } else { "ERR" }
                ),
            ),
            Page::Computers => units(t.status, &Unit::COMPUTERS),
            Page::Nodes => units(t.status, &Unit::NODES),
            Page::Gps => (
                lcd_line!("LAT:{:.4}", t.latitude),
                lcd_line!("LON:{:.4}", t.longitude),
            ),
            Page::Error => (
                lcd_line!("ERROR 0x{:02X}", t.error.to_byte()),
                lcd_line!("{}", t.error.text()),
            ),
        };
        Screen {
            top: top.with_last(if pinned { b'*' } else { b' ' }),
            bottom,
        }
    }
}

fn mode(m: DriveMode) -> &'static str {
    match m {
        DriveMode::Rc => "RC",
        DriveMode::Autonomous => "AUTO",
    }
}

fn state(s: DriveState) -> &'static str {
    match s {
        DriveState::Stop => "STOP",
        DriveState::Forward => "FWD",
        DriveState::Reverse => "REV",
        DriveState::Brake => "BRAKE",
    }
}

fn ok(good: bool) -> &'static str {
    if good {
        "OK"
    } else {
        "FAIL"
    }
}

fn jrk_page(name: &str, jrk: Jrk) -> (Line, Line) {
    (
        lcd_line!("{} JRK:{}", name, ok(jrk.ok)),
        lcd_line!("ERR:0x{:04X}", jrk.errors.0),
    )
}

/// The six flags, three a line: a flag's label when set, `---` when clear.
fn flags_page(name: &str, jrk: Jrk) -> (Line, Line) {
    let [a, b, c, d, e, f] = JrkFlag::ALL.map(|flag| {
        if jrk.errors.has(flag) {
            flag.label()
        } else {
            "---"
        }
    });
    (
        lcd_line!("{} {} {} {}", name, a, b, c),
        lcd_line!("    {} {} {}", d, e, f),
    )
}

/// Five units, three on the top line and two below: `+` up, `-` down.
fn units(status: Status, units: &[Unit; 5]) -> (Line, Line) {
    let [a, b, c, d, e] = units.map(|u| (u.label(), if status.is_up(u) { '+' } else { '-' }));
    (
        lcd_line!("{}{} {}{} {}{}", a.0, a.1, b.0, b.1, c.0, c.1),
        lcd_line!("{}{} {}{}", d.0, d.1, e.0, e.1),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::tests::sample;
    use crate::telemetry::{ErrorCode, JrkErrors};

    fn show(page: Page, t: &Telemetry) -> [&'static str; 2] {
        let s = page.render(Some(t), false);
        // Leak is fine in a test: it gives &'static strs to compare.
        let leak = |l: Line| -> &'static str { std::boxed::Box::leak(l.as_str().into()) };
        [leak(s.top), leak(s.bottom)]
    }

    extern crate std;

    #[test]
    fn the_knob_wraps_round() {
        assert_eq!(Page::Overview.turn(1), Page::Drive);
        assert_eq!(Page::Overview.turn(-1), Page::Error);
        assert_eq!(Page::Error.turn(1), Page::Overview);
        assert_eq!(Page::Overview.turn(26), Page::Overview);
    }

    #[test]
    fn before_the_first_packet() {
        let s = Page::Gps.render(None, true);
        assert_eq!(s.top.as_str(), "  AAV SENTINEL  ");
        assert_eq!(s.bottom.as_str(), " Waiting for Pi ");
    }

    #[test]
    fn every_page_fits_and_shows_the_pin() {
        let t = sample();
        for page in Page::ALL {
            let s = page.render(Some(&t), true);
            assert_eq!(s.top.bytes()[15], b'*', "{page:?}");
            assert!(s.top.as_str().is_ascii() && s.bottom.as_str().is_ascii());
        }
    }

    #[test]
    fn the_pages() {
        let t = sample();
        assert_eq!(
            show(Page::Overview, &t),
            ["MODE:AUTO       ", "STATE:FWD       "]
        );
        assert_eq!(
            show(Page::Drive, &t),
            ["STR:-0.50       ", "SPD:+0.25 A:-1.0"]
        );
        assert_eq!(
            show(Page::SteeringJrk, &t),
            ["STEER JRK:OK    ", "ERR:0x0000      "]
        );
        assert_eq!(
            show(Page::BrakeJrk, &t),
            ["BRAKE JRK:FAIL  ", "ERR:0x0011      "]
        );
        assert_eq!(
            show(Page::Throttle, &t),
            ["DAC:OK          ", "VAL:  700/1023  "]
        );
        assert_eq!(
            show(Page::Ibus, &t),
            ["EN:1000 TH:1500 ", "ST:1999 MD:2000 "]
        );
        assert_eq!(show(Page::Rc, &t), ["RC:ACTIVE       ", "CHANNELS:OK     "]);
        assert_eq!(
            show(Page::Gps, &t),
            ["LAT:45.3842     ", "LON:-75.6981    "]
        );
        assert_eq!(
            show(Page::Error, &t),
            ["ERROR 0x00      ", "OK              "]
        );
    }

    #[test]
    fn flags_show_by_name() {
        let mut t = sample();
        t.steering_jrk.errors = JrkErrors((1 << 0) | (1 << 5) | (1 << 9));
        assert_eq!(
            show(Page::SteeringFlags, &t),
            ["STR AWT --- --- ", "    FBK --- SER "]
        );
    }

    #[test]
    fn units_up_and_down() {
        let t = sample(); // 0b10_1101_0111
        assert_eq!(
            show(Page::Computers, &t),
            ["JET+ MPI+ RPI+  ", "UPI- CPI+       "]
        );
        assert_eq!(
            show(Page::Nodes, &t),
            ["IMU- RDR+ LDR+  ", "CAM- GPS+       "]
        );
    }

    #[test]
    fn the_e_stop_shouts() {
        let mut t = sample();
        t.error = ErrorCode::EStop;
        assert_eq!(
            show(Page::Error, &t),
            ["ERROR 0xFF      ", "!!! E-STOP !!!  "]
        );
    }
}
