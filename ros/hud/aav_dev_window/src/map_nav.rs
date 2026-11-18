//! How the map panel moves: following the car, the whole map, or where a person put it.
//! No Bevy, no egui, tested.
//!
//! Ben, 17 Nov: "The map can't be scrolled". The panel kept fitting itself to the chunks
//! round the car, and only a right-drag (hard on a Mac's trackpad) or the wheel broke it
//! free. Now map-core's [`View`] (the map window's own) keeps the mode:
//!
//! | Does | Mouse or key |
//! |---|---|
//! | pan (the view becomes free) | drag, left or right button |
//! | zoom about the pointer | the wheel, or a pinch |
//! | pick a goal | click, without dragging |
//! | follow the car / the whole map | F; or the panel's buttons |
//!
//! Until a person moves it, following the car sizes the view to the chunks loaded (the
//! three levels) or, on a big map, to 160 m; a small map (the test lot) is shown whole.

use map_core::{
    geom::Point,
    grid::Bounds,
    view::{Mode, PixelsPerMetre, ScreenPoint, View, Viewport},
};

/// A map wider than this (m) starts following the car, not fitted whole.
pub const BIG_MAP: f64 = 300.0;
/// How many metres across a big map's view starts, round the car.
pub const BIG_MAP_VIEW: f64 = 160.0;
/// The wheel's zoom: this power of e per point of scroll.
const WHEEL_ZOOM: f64 = 0.002;

/// What a person did to the map this frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Gesture {
    /// A drag by (dx, dy) screen points.
    Drag { dx: f64, dy: f64 },
    /// The wheel: points of scroll, up positive (closer), at the pointer.
    Wheel { by: f64, at: Option<ScreenPoint> },
    /// A pinch: the zoom factor (2 = twice as close), at the pointer.
    Pinch {
        factor: f64,
        at: Option<ScreenPoint>,
    },
    /// F: following the car <-> the whole map; from a free view, back to the car.
    ToggleFollow,
    /// The button: follow the car.
    FollowCar,
    /// The button: the whole map.
    WholeMap,
}

/// The map panel's view, and whether a person has chosen one yet.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MapNav {
    pub view: View,
    touched: bool,
}

impl MapNav {
    /// Each frame, before drawing. `whole`: the whole map's box (the static map, or the
    /// chunks); `near`: the chunks loaded round the car, if any.
    pub fn frame(
        &mut self,
        car: Point,
        whole: Option<Bounds>,
        near: Option<Bounds>,
        vp: &Viewport,
    ) {
        if !self.touched {
            self.start(whole, near, vp);
        }
        self.view.update(Some(car), whole, vp);
    }

    /// The view before a person moves it. Again each frame: the window's size changes
    /// in its first frames.
    fn start(&mut self, whole: Option<Bounds>, near: Option<Bounds>, vp: &Viewport) {
        let side = |b: Bounds| {
            let (w, h) = b.size();
            w.max(h)
        };
        let span = match (near, whole) {
            (Some(near), _) => Some(side(near)),
            (None, Some(whole)) if side(whole) > BIG_MAP => Some(BIG_MAP_VIEW),
            _ => None,
        };
        match span {
            Some(span) => {
                self.view.mode = Mode::FollowCar;
                self.view
                    .set_scale(PixelsPerMetre(vp.width.min(vp.height) / span.max(1.0)));
            }
            None => self.view.mode = Mode::WholeMap,
        }
    }

    /// The whole map from the start (`--map-whole`), as the button does.
    pub fn show_whole(&mut self) {
        self.view.mode = Mode::WholeMap;
        self.touched = true;
    }

    /// A person's gesture.
    pub fn apply(&mut self, gesture: Gesture, vp: &Viewport) {
        match gesture {
            Gesture::Drag { dx, dy } => {
                if dx == 0.0 && dy == 0.0 {
                    return;
                }
                self.view.pan(dx, dy);
            }
            Gesture::Wheel { by, at } => {
                if by == 0.0 {
                    return;
                }
                self.view.zoom((by * WHEEL_ZOOM).exp(), at, vp);
            }
            Gesture::Pinch { factor, at } => {
                if factor == 1.0 {
                    return;
                }
                self.view.zoom(factor, at, vp);
            }
            Gesture::ToggleFollow => self.view.toggle_follow(),
            Gesture::FollowCar => self.view.mode = Mode::FollowCar,
            Gesture::WholeMap => self.show_whole(),
        }
        self.touched = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vp() -> Viewport {
        Viewport {
            x: 0.0,
            y: 0.0,
            width: 800.0,
            height: 400.0,
        }
    }

    fn square(x: f64, y: f64, side: f64) -> Bounds {
        Bounds {
            min: Point::new(x, y),
            max: Point::new(x + side, y + side),
        }
    }

    /// The campus: 2 km across, the car in it, 3 x 3 chunks of 64 m round it.
    fn campus(nav: &mut MapNav, car: Point) {
        let near = square(car.x - 96.0, car.y - 96.0, 192.0);
        nav.frame(
            car,
            Some(square(-1000.0, -1000.0, 2000.0)),
            Some(near),
            &vp(),
        );
    }

    #[test]
    fn it_starts_on_the_car_sized_to_the_chunks() {
        let mut nav = MapNav::default();
        let car = Point::new(50.0, -70.0);
        campus(&mut nav, car);
        assert_eq!(nav.view.mode, Mode::FollowCar);
        assert_eq!(nav.view.centre, car);
        assert!((nav.view.scale.0 - 400.0 / 192.0).abs() < 1e-9);
    }

    #[test]
    fn a_small_map_is_shown_whole() {
        let mut nav = MapNav::default();
        nav.frame(
            Point::new(5.0, 5.0),
            Some(square(0.0, 0.0, 40.0)),
            None,
            &vp(),
        );
        assert_eq!(nav.view.mode, Mode::WholeMap);
        assert_eq!(nav.view.centre, Point::new(20.0, 20.0));
    }

    #[test]
    fn a_drag_pans_and_the_car_no_longer_pulls_it_back() {
        let mut nav = MapNav::default();
        campus(&mut nav, Point::new(0.0, 0.0));
        let k = nav.view.scale.0;
        nav.apply(
            Gesture::Drag {
                dx: -100.0,
                dy: 50.0,
            },
            &vp(),
        );
        assert_eq!(nav.view.mode, Mode::Free);
        let moved = nav.view.centre;
        assert!((moved.x - 100.0 / k).abs() < 1e-9 && (moved.y - 50.0 / k).abs() < 1e-9);
        // The car drives on; the view stays where the person put it.
        campus(&mut nav, Point::new(30.0, 0.0));
        assert_eq!(nav.view.centre, moved);
        assert!((nav.view.scale.0 - k).abs() < 1e-9);
    }

    #[test]
    fn far_round_the_campus_by_dragging() {
        let mut nav = MapNav::default();
        campus(&mut nav, Point::new(0.0, 0.0));
        for _ in 0..100 {
            nav.apply(Gesture::Drag { dx: -40.0, dy: 0.0 }, &vp());
            campus(&mut nav, Point::new(0.0, 0.0));
        }
        assert!(nav.view.centre.x > 1500.0, "{:?}", nav.view.centre);
    }

    #[test]
    fn the_wheel_zooms_about_the_pointer() {
        let mut nav = MapNav::default();
        campus(&mut nav, Point::new(0.0, 0.0));
        nav.apply(Gesture::Drag { dx: 1.0, dy: 0.0 }, &vp());
        let at = ScreenPoint::new(700.0, 100.0);
        let under = nav.view.to_map(at, &vp());
        let k = nav.view.scale.0;
        nav.apply(
            Gesture::Wheel {
                by: 200.0,
                at: Some(at),
            },
            &vp(),
        );
        assert!(nav.view.scale.0 > k);
        let still = nav.view.to_map(at, &vp());
        assert!((still.x - under.x).abs() < 1e-6 && (still.y - under.y).abs() < 1e-6);
        nav.apply(
            Gesture::Pinch {
                factor: 0.5,
                at: Some(at),
            },
            &vp(),
        );
        assert!(nav.view.scale.0 < k);
    }

    #[test]
    fn f_and_the_buttons_go_back_to_the_car() {
        let mut nav = MapNav::default();
        campus(&mut nav, Point::new(0.0, 0.0));
        nav.apply(
            Gesture::Drag {
                dx: -300.0,
                dy: 0.0,
            },
            &vp(),
        );
        nav.apply(Gesture::ToggleFollow, &vp());
        let car = Point::new(12.0, 3.0);
        campus(&mut nav, car);
        assert_eq!((nav.view.mode, nav.view.centre), (Mode::FollowCar, car));
        nav.apply(Gesture::WholeMap, &vp());
        campus(&mut nav, car);
        assert_eq!(nav.view.centre, Point::new(0.0, 0.0), "the campus, whole");
        nav.apply(Gesture::FollowCar, &vp());
        campus(&mut nav, car);
        assert_eq!(nav.view.centre, car);
    }

    #[test]
    fn a_still_gesture_changes_nothing() {
        let mut nav = MapNav::default();
        campus(&mut nav, Point::new(0.0, 0.0));
        nav.apply(Gesture::Drag { dx: 0.0, dy: 0.0 }, &vp());
        nav.apply(Gesture::Wheel { by: 0.0, at: None }, &vp());
        assert_eq!(nav.view.mode, Mode::FollowCar);
        assert!(!nav.touched);
    }
}
