//! A car's side view as lines, faint in the `initial_d` theme's background: a mid-80s
//! hatchback coupe (a long low bonnet, a boxy cabin, the hatch sloping to a square
//! tail), the kind of car Initial D's hero drives.
//!
//! Our own drawing: the proportions of the type, made up from round numbers, not traced
//! from any picture. Units: the car is 1 long; x from the tail (0) to the nose (1); y up
//! from the ground (0). The screen flips y and scales it: a [`Placement`].
//!
//! All four screens draw it with one function, `hud_app::paint::faint_car`.

use std::f32::consts::PI;

/// A point, (x, y): x towards the nose, y up.
pub type Point = (f32, f32);

/// A wheel: its centre and radius.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wheel {
    pub centre: Point,
    pub radius: f32,
}

/// The drawing: lines (each a run of points), and the two wheels as circles.
#[derive(Debug, Clone, PartialEq)]
pub struct Silhouette {
    /// The body's outline (closed: its last point is its first), then the windows, the
    /// door's cut and the crease along the side.
    pub lines: Vec<Vec<Point>>,
    pub wheels: [Wheel; 2],
}

/// The car's height, from the ground to the roof, as a part of its length.
pub const HEIGHT: f32 = 0.345;

/// Where the car goes on a screen, in pixels (y down): where its tail meets the ground,
/// and how long it is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Placement {
    pub left: f32,
    pub ground: f32,
    pub length: f32,
}

impl Placement {
    /// As long as it fits in the box (`x`, `y`, `width`, `height`), standing on the
    /// box's bottom, at its right end.
    pub fn bottom_right(x: f32, y: f32, width: f32, height: f32) -> Self {
        let length = width.min(height / HEIGHT).max(0.0);
        Self {
            left: x + width - length,
            ground: y + height,
            length,
        }
    }

    /// A point of the drawing on the screen.
    pub fn at(self, (x, y): Point) -> Point {
        (self.left + x * self.length, self.ground - y * self.length)
    }
}

/// The wheels' size, and where they sit.
const WHEEL_RADIUS: f32 = 0.075;
const REAR_AXLE: f32 = 0.19;
const FRONT_AXLE: f32 = 0.80;
/// How far the arch stands off the tyre.
const ARCH_CLEARANCE: f32 = 0.014;
/// The sill: the body's bottom edge between the wheels.
const SILL: f32 = 0.09;

/// An arch over a wheel at `axle`, from back to front: half a circle (a little more)
/// standing off the tyre.
fn arch(axle: f32) -> impl Iterator<Item = Point> {
    let r = WHEEL_RADIUS + ARCH_CLEARANCE;
    let cy = WHEEL_RADIUS;
    // From where the arch meets the sill (behind) over the top to the front.
    let meet = ((SILL - cy) / r).asin();
    let (start, end) = (PI - meet, meet);
    const STEPS: usize = 14;
    (0..=STEPS).map(move |i| {
        let a = start + (end - start) * i as f32 / STEPS as f32;
        (axle + r * a.cos(), cy + r * a.sin())
    })
}

/// The hatchback coupe.
pub fn hatchback() -> Silhouette {
    // Round the top, from the tail's bottom to the nose's bottom.
    let top = [
        (0.02, 0.10),   // the rear bumper's bottom
        (0.01, 0.17),   // the tail
        (0.03, 0.215),  // the hatch's end
        (0.10, 0.228),  // a small lip
        (0.31, 0.335),  // the hatch rises to the roof
        (0.55, HEIGHT), // a flat roof
        (0.69, 0.248),  // the windscreen down to the cowl
        (0.97, 0.205),  // a long bonnet, falling a little
        (0.995, 0.17),  // the nose
        (0.99, 0.10),   // the front bumper's bottom
    ];
    // Back along the bottom: over the front wheel, the sill, over the rear wheel.
    let mut body: Vec<Point> = top.to_vec();
    body.extend(arch(FRONT_AXLE).collect::<Vec<_>>().into_iter().rev());
    body.extend(arch(REAR_AXLE).collect::<Vec<_>>().into_iter().rev());
    body.push(top[0]);

    let windows = vec![
        (0.15, 0.236),
        (0.32, 0.316),
        (0.54, 0.328),
        (0.665, 0.252),
        (0.15, 0.236),
    ];
    let pillar = vec![(0.47, 0.326), (0.465, 0.243)];
    let door = vec![(0.665, 0.252), (0.66, 0.11), (0.43, 0.10), (0.44, 0.241)];
    let crease = vec![(0.02, 0.165), (0.985, 0.16)];
    let wheel = |x: f32| Wheel {
        centre: (x, WHEEL_RADIUS),
        radius: WHEEL_RADIUS,
    };
    Silhouette {
        lines: vec![body, windows, pillar, door, crease],
        wheels: [wheel(REAR_AXLE), wheel(FRONT_AXLE)],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_fits_a_long_low_box() {
        let car = hatchback();
        for line in &car.lines {
            for &(x, y) in line {
                assert!((0.0..=1.0).contains(&x), "x {x}");
                assert!((0.0..=0.36).contains(&y), "y {y}");
            }
        }
        // Low and long: more than three times as long as tall.
        let tallest = car
            .lines
            .iter()
            .flatten()
            .map(|p| p.1)
            .fold(0.0_f32, f32::max);
        assert!(tallest < 1.0 / 2.8);
    }

    #[test]
    fn the_roof_is_its_height() {
        let tallest = hatchback()
            .lines
            .iter()
            .flatten()
            .map(|p| p.1)
            .fold(0.0_f32, f32::max);
        assert_eq!(tallest, HEIGHT);
    }

    #[test]
    fn a_placement_fits_its_box() {
        // A wide box: the car is as tall as the box, at its right end.
        let wide = Placement::bottom_right(100.0, 50.0, 800.0, 69.0);
        assert!((wide.length - 200.0).abs() < 1e-3);
        assert_eq!(wide.at((1.0, 0.0)), (900.0, 119.0));
        let (_, roof) = wide.at((0.5, HEIGHT));
        assert!((roof - 50.0).abs() < 1e-3);
        // A tall box: the car is as long as the box is wide.
        let tall = Placement::bottom_right(0.0, 0.0, 300.0, 600.0);
        assert_eq!((tall.left, tall.ground, tall.length), (0.0, 600.0, 300.0));
        // No room: no car.
        assert_eq!(Placement::bottom_right(0.0, 0.0, -5.0, 10.0).length, 0.0);
    }

    #[test]
    fn the_body_is_closed() {
        let car = hatchback();
        let body = &car.lines[0];
        assert_eq!(body.first(), body.last());
        assert!(body.len() > 30);
    }

    #[test]
    fn the_wheels_stand_on_the_ground_under_their_arches() {
        let car = hatchback();
        let body = &car.lines[0];
        for w in car.wheels {
            assert!((w.centre.1 - w.radius).abs() < 1e-6, "on the ground");
            // No point of the body cuts into a tyre.
            for &(x, y) in body {
                let d = ((x - w.centre.0).powi(2) + (y - w.centre.1).powi(2)).sqrt();
                assert!(d > w.radius, "the body at ({x}, {y}) is in a wheel");
            }
        }
        assert!(car.wheels[0].centre.0 < car.wheels[1].centre.0);
    }

    #[test]
    fn the_arches_meet_the_sill() {
        for axle in [REAR_AXLE, FRONT_AXLE] {
            let points: Vec<Point> = arch(axle).collect();
            let (first, last) = (points[0], points[points.len() - 1]);
            assert!((first.1 - SILL).abs() < 1e-4 && (last.1 - SILL).abs() < 1e-4);
            assert!(first.0 < axle && last.0 > axle);
            let top = points.iter().map(|p| p.1).fold(0.0_f32, f32::max);
            assert!(top > 2.0 * WHEEL_RADIUS);
        }
    }
}
