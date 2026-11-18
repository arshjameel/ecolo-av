//! How a thing looks on a screen: its words and its colour.
//!
//! TODO 192 step 3. The markers (RViz, Foxglove) and the Rust screens (`hud_core::things`)
//! use the same colours:
//!
//! | Kind | Colour | Why |
//! |---|---|---|
//! | person | yellow | the one to watch: high-visibility vest |
//! | car, truck, bus, motorcycle | blue | another vehicle |
//! | stop sign | red | the sign's own red |
//! | any other name (bicycle...) | green | named, not one of the above |
//! | unknown | orange | no camera has named it |
//! | coasting (unseen now) | grey | moved on its speed, not seen in the last cloud |

use crate::UNKNOWN;

/// Red, green, blue, each 0..1.
pub type Rgb = [f64; 3];

/// Another vehicle's names.
pub const VEHICLES: [&str; 4] = ["car", "truck", "bus", "motorcycle"];
/// A person.
pub const PERSON: Rgb = [1.0, 0.85, 0.1];
/// A vehicle.
pub const VEHICLE: Rgb = [0.25, 0.55, 1.0];
/// A stop sign.
pub const STOP_SIGN: Rgb = [0.9, 0.1, 0.1];
/// Any other name.
pub const OTHER: Rgb = [0.3, 0.85, 0.4];
/// No name.
pub const UNNAMED: Rgb = [1.0, 0.6, 0.0];
/// Unseen now.
pub const COASTING: Rgb = [0.6, 0.6, 0.6];

/// A thing's colour by its kind; grey while coasting.
#[must_use]
pub fn colour(kind: &str, coasting: bool) -> Rgb {
    if coasting {
        return COASTING;
    }
    let name = named(kind).to_lowercase();
    match name.as_str() {
        "person" => PERSON,
        "stop sign" => STOP_SIGN,
        n if VEHICLES.contains(&n) => VEHICLE,
        UNKNOWN => UNNAMED,
        _ => OTHER,
    }
}

/// A thing's words: its name and id, then its speed: `person 3 1.2 m/s`. An unknown
/// thing shows only its id (`3 0.0 m/s`); ` R`: the radar gave its speed.
#[must_use]
pub fn caption(id: u32, kind: &str, vx: f64, vy: f64, radar: bool) -> String {
    let name = named(kind);
    let who = if name.to_lowercase() == UNKNOWN {
        id.to_string()
    } else {
        format!("{name} {id}")
    };
    let r = if radar { " R" } else { "" };
    format!("{who} {:.1} m/s{r}", ego_core::hypot(vx, vy))
}

/// The kind without its spaces round it; empty is unknown.
fn named(kind: &str) -> &str {
    let k = kind.trim();
    if k.is_empty() {
        UNKNOWN
    } else {
        k
    }
}
