//! Draws the little robot's eyes (the easter egg; hud-core's `robot`) peeking over the
//! bottom of the 3D view, in the strip the key legend leaves free, clipped to it.

use hud_app::paint::ToColor;
use hud_core::{
    robot::{self, Form, PeekPose, Strip, Xy},
    theme::Theme,
};
use raylib::prelude::*;

/// The eyes at most this tall, and at least.
const PEEK_H: f32 = 130.0;
const PEEK_MIN_H: f32 = 48.0;
/// Space kept round them.
const GAP: f32 = 16.0;

/// Where the eyes may peek: on the window's bottom edge between `left` (the key legend's
/// right side) and `right` (the 3D view's). `None` when there's no room.
pub fn corner(left: f32, right: f32, h: f32) -> Option<Rectangle> {
    let (x, width) = (left + GAP, right - left - 2.0 * GAP);
    let fits = |height: f32| robot::peek_width(f64::from(height)) <= f64::from(width);
    let mut height = PEEK_H;
    while height >= PEEK_MIN_H {
        if fits(height) {
            return Some(Rectangle::new(x, h - height, width, height));
        }
        height -= 8.0;
    }
    None
}

/// The eyes in `r`, as `pose` says.
pub fn peek(d: &mut RaylibDrawHandle, theme: &Theme, r: Rectangle, pose: PeekPose) {
    let colours = theme.robot();
    let strip = Strip::new(
        f64::from(r.x),
        f64::from(r.y),
        f64::from(r.width),
        f64::from(r.height),
    );
    let pieces = robot::peek(strip, strip.x + strip.width / 2.0, pose);
    let mut s = d.begin_scissor_mode(r.x as i32, r.y as i32, r.width as i32, r.height as i32);
    let v = |p: Xy| Vector2::new(p.x as f32, p.y as f32);
    let edge = colours.edge.rl();
    for piece in &pieces {
        let colour = colours.of(piece.part).rl();
        match &piece.form {
            Form::Polygon(points) => {
                let pts: Vec<Vector2> = points.iter().map(|&p| v(p)).collect();
                for i in 1..pts.len().saturating_sub(1) {
                    // raylib fills one winding only: draw both.
                    s.draw_triangle(pts[0], pts[i], pts[i + 1], colour);
                    s.draw_triangle(pts[0], pts[i + 1], pts[i], colour);
                }
                for (i, &a) in pts.iter().enumerate() {
                    s.draw_line_ex(a, pts[(i + 1) % pts.len()], 1.0, edge);
                }
            }
            Form::Circle { centre, radius } => {
                s.draw_circle_v(v(*centre), *radius as f32 + 1.0, edge);
                s.draw_circle_v(v(*centre), *radius as f32, colour);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_corner_sits_between_the_legend_and_the_panel() {
        let r = corner(510.0, 1140.0, 900.0).expect("room");
        assert!(r.x >= 510.0 + GAP && r.x + r.width <= 1140.0 - GAP);
        assert!((r.y + r.height - 900.0).abs() < 1e-3);
        assert!(corner(510.0, 600.0, 900.0).is_none(), "too narrow");
    }
}
