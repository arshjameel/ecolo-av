//! The map editor's tools, with no Bevy and no egui: tested.
//!
//! | Tool | A left drag on the map | Keys |
//! |---|---|---|
//! | Pan (P) | moves the view | |
//! | Box (B) | draws a box: then Delete, or Label | Delete / Backspace, Esc |
//! | Align (A) | moves our map over the street map | arrows: 5 cm (shift 50 cm); Q / E: turn 0.1° (shift 1°) |
//!
//! Every tool: the wheel or a pinch zooms, a right or middle drag pans, Cmd/Ctrl+Z undoes.

use map_edit::{Align, Point, Region};

/// What a left drag on the map does.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Tool {
    /// Moves the view.
    #[default]
    Pan,
    /// Draws a box.
    Box,
    /// Moves our map.
    Align,
}

impl Tool {
    /// Every tool, in the order the panel lists them.
    pub const ALL: [Self; 3] = [Self::Pan, Self::Box, Self::Align];

    /// The words on its button.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Pan => "Pan (P)",
            Self::Box => "Box (B)",
            Self::Align => "Align (A)",
        }
    }
}

/// The smallest box side a drag makes (m): a click is not a box.
pub const MIN_BOX: f64 = 0.05;
/// An arrow key's shift (m); with shift held, ten times.
pub const NUDGE_M: f64 = 0.05;
/// Q or E's turn (degrees); with shift held, ten times.
pub const NUDGE_DEG: f64 = 0.1;

/// The box a drag from `from` to `to` draws (map points), turned by `yaw` about its middle.
/// `None` if it is too small to be one.
#[must_use]
pub fn drag_box(from: Point, to: Point, yaw: f64) -> Option<Region> {
    let mut region = Region::from_corners(from, to);
    region.yaw_rad = yaw;
    (region.size_m.iter().all(|&s| s >= MIN_BOX) && region.is_valid()).then_some(region)
}

/// The alignment after a drag of (`dx`, `dy`) screen pixels at `scale` pixels a metre
/// (screen y is down, the map's y is up).
#[must_use]
pub fn dragged(align: Align, dx: f64, dy: f64, scale: f64) -> Align {
    if scale <= 0.0 || !scale.is_finite() {
        return align;
    }
    align.moved(dx / scale, -dy / scale)
}

/// A key that moves the map in the align tool.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Nudge {
    /// Arrow left: -x.
    Left,
    /// Arrow right: +x.
    Right,
    /// Arrow up: +y.
    Up,
    /// Arrow down: -y.
    Down,
    /// Q: counter-clockwise.
    TurnLeft,
    /// E: clockwise.
    TurnRight,
}

/// The alignment after a key; turns are about `pivot` (the view's middle).
#[must_use]
pub fn nudged(align: Align, key: Nudge, big: bool, pivot: Point) -> Align {
    let k = if big { 10.0 } else { 1.0 };
    let (m, turn) = (NUDGE_M * k, (NUDGE_DEG * k).to_radians());
    match key {
        Nudge::Left => align.moved(-m, 0.0),
        Nudge::Right => align.moved(m, 0.0),
        Nudge::Up => align.moved(0.0, m),
        Nudge::Down => align.moved(0.0, -m),
        Nudge::TurnLeft => align.turned_about(pivot, turn),
        Nudge::TurnRight => align.turned_about(pivot, -turn),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_click_is_not_a_box() {
        let p = Point::new(3.0, 4.0);
        assert!(drag_box(p, p, 0.0).is_none());
        let b = drag_box(p, Point::new(5.0, 1.0), 0.2).expect("a box");
        assert_eq!(b.centre_m, [4.0, 2.5]);
        assert_eq!(b.yaw_rad, 0.2);
    }

    #[test]
    fn a_drag_moves_the_map_with_the_pointer() {
        // 10 px a metre: 20 px right and 10 px down is 2 m east, 1 m south.
        let a = dragged(Align::IDENTITY, 20.0, 10.0, 10.0);
        assert_eq!(a, Align::new(2.0, -1.0, 0.0));
        assert_eq!(dragged(a, 5.0, 5.0, 0.0), a);
    }

    #[test]
    fn keys_nudge_and_turn_in_place() {
        let pivot = Point::new(40.0, 20.0);
        assert_eq!(
            nudged(Align::IDENTITY, Nudge::Up, true, pivot),
            Align::new(0.0, 0.5, 0.0)
        );
        let turned = nudged(Align::IDENTITY, Nudge::TurnRight, false, pivot);
        assert!((turned.yaw_rad + 0.1f64.to_radians()).abs() < 1e-12);
        let p = turned.apply(pivot);
        assert!((p.x - pivot.x).abs() < 1e-9 && (p.y - pivot.y).abs() < 1e-9);
    }
}
