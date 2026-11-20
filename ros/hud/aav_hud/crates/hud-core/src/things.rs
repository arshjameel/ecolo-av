//! The things near the car (`/world/things`, aav_things_rs), named by the camera: what a
//! screen shows of each. TODO 192 step 3 (Ben, 15 Nov: "object deteciton and display
//! like a stop sign").
//!
//! | Kind | Colour | Words |
//! |---|---|---|
//! | person | yellow | `person 3` |
//! | car, truck, bus, motorcycle | blue | `car 7` |
//! | stop sign | red | `stop sign 2` |
//! | any other name | green | `bicycle 5` |
//! | unknown | orange | `4` (no name: the id only) |
//! | coasting (unseen now) | grey | the same words |
//!
//! The colours live in `theme` (`thing_colour`), the same as the RViz markers (aav_things_rs
//! `looks.py`). They stay the same in every theme, as the localizers' do: a person is
//! yellow on every screen. The things are in odom; [`Thing::in_base`] moves one into
//! base_link with the car's pose.

use crate::theme::{self, Rgba};

/// What a thing is, from the camera's label.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    /// A person.
    Person,
    /// A car, truck, bus or motorcycle.
    Vehicle,
    /// A stop sign.
    StopSign,
    /// Named, but none of the above (a bicycle...).
    Other,
    /// No camera has named it.
    Unknown,
}

/// The car's pose in odom: x, y (m), yaw (rad).
pub type CarPose = (f64, f64, f64);

/// The labels that count as a vehicle (COCO's names).
const VEHICLES: [&str; 4] = ["car", "truck", "bus", "motorcycle"];

/// `Thing.UNKNOWN` in the message.
pub const UNKNOWN: &str = "unknown";

impl Kind {
    /// The kind of a detector label (`person`, `stop sign`...); empty is unknown.
    #[must_use]
    pub fn of(label: &str) -> Self {
        let label = label.trim().to_lowercase();
        match label.as_str() {
            "" | UNKNOWN => Self::Unknown,
            "person" => Self::Person,
            "stop sign" => Self::StopSign,
            l if VEHICLES.contains(&l) => Self::Vehicle,
            _ => Self::Other,
        }
    }

    /// Its colour on every screen (the markers' too: `looks.py`).
    #[must_use]
    pub fn colour(self) -> Rgba {
        theme::thing_colour(self)
    }
}

/// One thing (aav_interfaces/Thing): odom, metres and m/s.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Thing {
    /// Its tracking number.
    pub id: u32,
    /// The camera's label as it came (`person`, `stop sign`); `unknown` or empty: none.
    pub name: String,
    /// m, along odom x.
    pub x: f64,
    /// m, along odom y.
    pub y: f64,
    /// m/s, along odom x.
    pub vx: f64,
    /// m/s, along odom y.
    pub vy: f64,
    /// Along odom x, along odom y, above the road.
    pub length: f64,
    /// m, along odom y.
    pub width: f64,
    /// m, above the road.
    pub height: f64,
    /// Not seen in the last cloud: moved on its speed.
    pub coasting: bool,
}

impl Thing {
    /// What it is, from its name.
    #[must_use]
    pub fn kind(&self) -> Kind {
        Kind::of(&self.name)
    }

    /// Whether a camera named it.
    #[must_use]
    pub fn is_named(&self) -> bool {
        self.kind() != Kind::Unknown
    }

    /// Its words: `person 3`; an unnamed thing, its id only.
    #[must_use]
    pub fn label(&self) -> String {
        if self.is_named() {
            format!("{} {}", self.name.trim(), self.id)
        } else {
            self.id.to_string()
        }
    }

    /// Its colour: by kind, grey while coasting.
    #[must_use]
    pub fn colour(&self) -> Rgba {
        if self.coasting {
            theme::THING_COASTING
        } else {
            self.kind().colour()
        }
    }

    /// Its middle in base_link, for the car at (x, y, yaw) in odom.
    #[must_use]
    pub fn in_base(&self, (cx, cy, yaw): CarPose) -> (f64, f64) {
        let (dx, dy) = (self.x - cx, self.y - cy);
        let (sin, cos) = yaw.sin_cos();
        (cos * dx + sin * dy, -sin * dx + cos * dy)
    }

    /// Its box's four corners in base_link (the box is upright and square to odom).
    #[must_use]
    pub fn corners_in_base(&self, car: CarPose) -> [(f64, f64); 4] {
        let (hl, hw) = (self.length / 2.0, self.width / 2.0);
        [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)].map(|(dx, dy)| {
            Thing {
                x: self.x + dx,
                y: self.y + dy,
                ..Thing::default()
            }
            .in_base(car)
        })
    }
}

/// The named things ahead of the car and within `range` m, nearest first, with their
/// distance (m, from the rear axle): for a short list of words, such as `PERSON 4.2 m`.
/// "Ahead" is in front of the rear axle, within 45 degrees either side.
#[must_use]
pub fn named_ahead(things: &[Thing], car: CarPose, range: f64) -> Vec<(&Thing, f64)> {
    let mut ahead: Vec<_> = things
        .iter()
        .filter(|t| t.is_named())
        .filter_map(|t| {
            let (x, y) = t.in_base(car);
            (x > 0.0 && y.abs() <= x && x.hypot(y) <= range).then_some((t, x.hypot(y)))
        })
        .collect();
    ahead.sort_by(|a, b| a.1.total_cmp(&b.1));
    ahead
}

#[cfg(test)]
mod tests {
    use super::*;

    fn thing(id: u32, name: &str, x: f64, y: f64) -> Thing {
        Thing {
            id,
            name: name.into(),
            x,
            y,
            length: 0.5,
            width: 0.5,
            height: 1.7,
            ..Thing::default()
        }
    }

    #[test]
    fn labels_name_their_kind() {
        assert_eq!(Kind::of("person"), Kind::Person);
        assert_eq!(Kind::of("Stop Sign"), Kind::StopSign);
        assert_eq!(Kind::of("truck"), Kind::Vehicle);
        assert_eq!(Kind::of("bicycle"), Kind::Other);
        assert_eq!(Kind::of("unknown"), Kind::Unknown);
        assert_eq!(Kind::of(""), Kind::Unknown);
    }

    #[test]
    fn a_named_thing_says_its_name_and_an_unnamed_one_its_id() {
        assert_eq!(thing(3, "person", 0.0, 0.0).label(), "person 3");
        assert_eq!(thing(2, "stop sign", 0.0, 0.0).label(), "stop sign 2");
        assert_eq!(thing(4, "unknown", 0.0, 0.0).label(), "4");
        assert_eq!(thing(4, "", 0.0, 0.0).label(), "4");
    }

    #[test]
    fn each_kind_has_its_own_colour_and_coasting_is_grey() {
        let kinds = [
            Kind::Person,
            Kind::Vehicle,
            Kind::StopSign,
            Kind::Other,
            Kind::Unknown,
        ];
        let mut colours: Vec<_> = kinds.iter().map(|k| k.colour()).collect();
        colours.push(theme::THING_COASTING);
        colours.sort_by_key(|c| (c.r, c.g, c.b));
        colours.dedup();
        assert_eq!(colours.len(), kinds.len() + 1);
        let mut p = thing(1, "person", 0.0, 0.0);
        assert_eq!(p.colour(), Kind::Person.colour());
        p.coasting = true;
        assert_eq!(p.colour(), theme::THING_COASTING);
    }

    #[test]
    fn a_thing_moves_into_base_link() {
        // The car at (10, 5) facing +y (left): a thing at (10, 8) is 3 m ahead.
        let car = (10.0, 5.0, std::f64::consts::FRAC_PI_2);
        let (x, y) = thing(1, "person", 10.0, 8.0).in_base(car);
        assert!((x - 3.0).abs() < 1e-9 && y.abs() < 1e-9, "{x} {y}");
        let corners = thing(1, "person", 10.0, 8.0).corners_in_base(car);
        assert!(corners
            .iter()
            .all(|(cx, _)| (cx - 3.0).abs() <= 0.25 + 1e-9));
    }

    #[test]
    fn only_named_things_ahead_are_listed_nearest_first() {
        let car = (0.0, 0.0, 0.0);
        let things = [
            thing(1, "stop sign", 12.0, 2.0),
            thing(2, "person", 4.0, -1.0),
            thing(3, "unknown", 2.0, 0.0), // not named
            thing(4, "person", -3.0, 0.0), // behind
            thing(5, "car", 2.0, 6.0),     // beside, not ahead
            thing(6, "car", 40.0, 0.0),    // too far
        ];
        let ids: Vec<_> = named_ahead(&things, car, 30.0)
            .iter()
            .map(|(t, _)| t.id)
            .collect();
        assert_eq!(ids, vec![2, 1]);
    }
}
