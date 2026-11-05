//! Draws the little robot (the easter egg; hud-core's `robot`): its eyes peeking over
//! the bottom edge, or the robot driving across the bottom lane. Only inside the room
//! the page left free ([`Room`]), clipped to it.

use hud_app::paint::ToColor;
use hud_core::{
    robot::{self, Form, PeekPose, Piece, Show, Strip, Xy},
    theme::{RobotColours, Theme},
};
use raylib::prelude::*;

use crate::ui::Room;

/// The tallest the robot drives, in pixels.
const DRIVE_H: f32 = 110.0;
/// Shorter than this, it doesn't drive (the lane is too thin).
const DRIVE_MIN_H: f32 = 20.0;

/// Draws what `show` says, in `room`.
pub fn draw(d: &mut RaylibDrawHandle, theme: &Theme, show: Show, room: &Room) {
    let colours = theme.robot();
    match show {
        Show::Nothing => {}
        Show::Peek(pose) => {
            if let Some(r) = room.corner {
                peek(d, &colours, r, pose);
            }
        }
        Show::Drive { progress } => {
            if let Some(r) = room.lane {
                drive(d, &colours, r, progress);
            }
        }
    }
}

fn peek(d: &mut RaylibDrawHandle, colours: &RobotColours, r: Rectangle, pose: PeekPose) {
    let strip = Strip::new(
        f64::from(r.x),
        f64::from(r.y),
        f64::from(r.width),
        f64::from(r.height),
    );
    let pieces = robot::peek(strip, strip.x + strip.width / 2.0, pose);
    let mut s = d.begin_scissor_mode(r.x as i32, r.y as i32, r.width as i32, r.height as i32);
    fill(&mut s, colours, &pieces);
}

fn drive(d: &mut RaylibDrawHandle, colours: &RobotColours, r: Rectangle, progress: f64) {
    let h = (r.height - 4.0).min(DRIVE_H);
    if h < DRIVE_MIN_H {
        return;
    }
    let width = robot::side_width(f64::from(h));
    let start = f64::from(r.x) - width / 2.0;
    let x = start + progress * (f64::from(r.width) + width);
    let ground = Xy::new(x, f64::from(r.y + r.height - 2.0));
    let pieces = robot::side_view(ground, f64::from(h), x - start, true);
    let mut s = d.begin_scissor_mode(r.x as i32, r.y as i32, r.width as i32, r.height as i32);
    fill(&mut s, colours, &pieces);
}

/// Fills each piece in order, with a thin dark edge.
fn fill(d: &mut impl RaylibDraw, colours: &RobotColours, pieces: &[Piece<Xy>]) {
    let v = |p: Xy| Vector2::new(p.x as f32, p.y as f32);
    let edge = colours.edge.rl();
    for piece in pieces {
        let colour = colours.of(piece.part).rl();
        match &piece.form {
            Form::Polygon(points) => {
                let pts: Vec<Vector2> = points.iter().map(|&p| v(p)).collect();
                for i in 1..pts.len().saturating_sub(1) {
                    // raylib fills one winding only: draw both.
                    d.draw_triangle(pts[0], pts[i], pts[i + 1], colour);
                    d.draw_triangle(pts[0], pts[i + 1], pts[i], colour);
                }
                for (i, &a) in pts.iter().enumerate() {
                    d.draw_line_ex(a, pts[(i + 1) % pts.len()], 1.0, edge);
                }
            }
            Form::Circle { centre, radius } => {
                d.draw_circle_v(v(*centre), *radius as f32 + 1.0, edge);
                d.draw_circle_v(v(*centre), *radius as f32, colour);
            }
        }
    }
}
