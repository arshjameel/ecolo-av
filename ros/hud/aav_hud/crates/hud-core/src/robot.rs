//! The little robot: the screens' easter egg. Our own drawing, a tribute to a film robot
//! with big round eyes: a boxy body on two tank treads, a neck, and two round
//! binocular eyes. It is made only of shapes (polygons and circles), so it scales with
//! the window and takes the theme's colours ([`crate::theme::RobotColours`]).
//!
//! | What | Where it shows | Here |
//! |---|---|---|
//! | the car on the map, from above, a little tilted (2.5D) | the map window, `car_marker: robot` | [`top_view`], [`Odometer`] |
//! | two eyes peeking over the bottom edge | the hardware panel, the debug screen | [`peek`], [`PeekPose`] |
//! | the robot driving across the bottom | the hardware panel | [`side_view`] |
//! | when each shows, and when it hides | both | [`EasterEgg`], [`Watch`], [`Show`] |
//!
//! The drawing code only fills what these functions return, in order (painter's order:
//! the first piece is at the back). Everything here is plain numbers: no raylib, no
//! clock. Time comes in as an [`Instant`] or a [`Duration`], so the tests are exact.

use std::{
    f64::consts::{PI, TAU},
    time::{Duration, Instant},
};

/// A point on a flat picture: screen pixels (y down) or the robot's own frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Xy {
    pub x: f64,
    pub y: f64,
}

impl Xy {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// A point with a height: the robot's frame from above (x forward, y left, z up,
/// metres, REP-103), or the map frame plus a height.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Xyz {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Xyz {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// Which part of the robot a piece is: it picks the piece's colour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Part {
    /// A tread's belt.
    Tread,
    /// A tread's cleats (from above), or its wheels (from the side).
    Cleat,
    /// The box's top and front: the robot's main colour.
    Body,
    /// The box's sides, in shade.
    BodySide,
    /// The neck, the bridge between the eyes, the hatch line.
    Neck,
    /// An eye's round shell; also its lid when it blinks.
    Eye,
    /// An eye's dark glass.
    Lens,
    /// The light caught in the glass.
    Glint,
}

impl Part {
    /// Every part, for the tests.
    pub const ALL: [Part; 8] = [
        Part::Tread,
        Part::Cleat,
        Part::Body,
        Part::BodySide,
        Part::Neck,
        Part::Eye,
        Part::Lens,
        Part::Glint,
    ];
}

/// A shape to fill.
#[derive(Debug, Clone, PartialEq)]
pub enum Form<P> {
    /// A convex polygon: its corners, in order round it.
    Polygon(Vec<P>),
    /// A circle (from above: one lying flat, which a tilt from above keeps round).
    Circle { centre: P, radius: f64 },
}

/// One shape of the robot, and its part.
#[derive(Debug, Clone, PartialEq)]
pub struct Piece<P> {
    pub part: Part,
    pub form: Form<P>,
}

impl<P: Copy> Piece<P> {
    fn polygon(part: Part, points: Vec<P>) -> Self {
        Self {
            part,
            form: Form::Polygon(points),
        }
    }

    fn circle(part: Part, centre: P, radius: f64) -> Self {
        Self {
            part,
            form: Form::Circle { centre, radius },
        }
    }

    /// The same piece somewhere else: each point moved by `to`, each radius times
    /// `scale`.
    pub fn map<Q>(&self, to: impl Fn(P) -> Q, scale: f64) -> Piece<Q> {
        let form = match &self.form {
            Form::Polygon(points) => Form::Polygon(points.iter().map(|&p| to(p)).collect()),
            Form::Circle { centre, radius } => Form::Circle {
                centre: to(*centre),
                radius: radius * scale,
            },
        };
        Piece {
            part: self.part,
            form,
        }
    }

    /// Every point of the piece (a circle's centre only).
    pub fn points(&self) -> Vec<P> {
        match &self.form {
            Form::Polygon(points) => points.clone(),
            Form::Circle { centre, .. } => vec![*centre],
        }
    }
}

// ---------------------------------------------------------------------------------
// From above (the map's car).
// ---------------------------------------------------------------------------------

/// How far up the screen a point one metre high is drawn, per metre of the map's
/// scale: the 2.5D's slight tilt (an oblique view, as from a little behind the screen's
/// bottom edge). Flat shapes stay their true size and place; only height moves up.
pub const TILT: f64 = 0.45;

/// The robot from above, in its own frame (x forward, y left, z up, metres; the origin
/// at the middle of its footprint), `length` by `width` on the ground.
///
/// The two treads fill the footprint exactly, so the robot covers the car's true
/// outline. The body sits between them, the neck behind the eyes, the eyes at the front,
/// facing forward. `travelled`: metres the robot has rolled; it moves the cleats.
#[derive(Debug, Clone, PartialEq)]
pub struct TopView {
    /// The right tread (y < 0), then the left (y > 0), each with its cleats. The caller
    /// draws the one further from the viewer first, then [`TopView::middle`], then the
    /// nearer one.
    pub treads: [Vec<Piece<Xyz>>; 2],
    /// The body, the neck and the eyes, back to front.
    pub middle: Vec<Piece<Xyz>>,
}

/// A tread's width, as a share of the robot's.
const TREAD_SHARE: f64 = 0.27;
/// A tread's height, as a share of the robot's width.
const TREAD_HIGH: f64 = 0.18;
/// Cleats along a tread's length.
const CLEATS: usize = 7;

/// The robot from above: see [`TopView`].
pub fn top_view(length: f64, width: f64, travelled: f64) -> TopView {
    let (hl, hw) = (length / 2.0, width / 2.0);
    let tread_w = TREAD_SHARE * width;
    let tread_h = TREAD_HIGH * width;
    let spacing = length / CLEATS as f64;
    let cleat = 0.32 * spacing;
    let tread = |y0: f64, y1: f64| {
        let mut out: Vec<Piece<Xyz>> = block(
            [-hl, hl],
            [y0, y1],
            [0.0, tread_h],
            Part::Tread,
            Part::Tread,
        )
        .into();
        // The top run moves forward against the body as the robot drives forward.
        let shift = travelled.rem_euclid(spacing);
        let z = tread_h;
        out.extend((0..=CLEATS).filter_map(|i| {
            let mid = -hl + i as f64 * spacing + shift;
            let (a, b) = ((mid - cleat / 2.0).max(-hl), (mid + cleat / 2.0).min(hl));
            let inset = 0.12 * tread_w;
            (b - a > 1e-9).then(|| {
                Piece::polygon(
                    Part::Cleat,
                    vec![
                        Xyz::new(a, y0 + inset, z),
                        Xyz::new(b, y0 + inset, z),
                        Xyz::new(b, y1 - inset, z),
                        Xyz::new(a, y1 - inset, z),
                    ],
                )
            })
        }));
        out
    };
    let treads = [tread(-hw, -hw + tread_w), tread(hw - tread_w, hw)];

    // The body: a box between the treads (a little over them), its top in the light.
    let body_x = [-0.36 * length, 0.18 * length];
    let body_y = [-0.25 * width, 0.25 * width];
    let body_z = [0.06 * width, 0.62 * width];
    let mut middle: Vec<Piece<Xyz>> =
        block(body_x, body_y, body_z, Part::BodySide, Part::Body).into();
    // A hatch line across the top.
    let top = body_z[1];
    let hatch_x = body_x[0] + 0.3 * (body_x[1] - body_x[0]);
    middle.push(Piece::polygon(
        Part::Neck,
        vec![
            Xyz::new(hatch_x - 0.01 * length, body_y[0] * 0.8, top),
            Xyz::new(hatch_x + 0.01 * length, body_y[0] * 0.8, top),
            Xyz::new(hatch_x + 0.01 * length, body_y[1] * 0.8, top),
            Xyz::new(hatch_x - 0.01 * length, body_y[1] * 0.8, top),
        ],
    ));
    // The neck, up from the top's front, behind the eyes.
    let eye_z = 0.92 * width;
    let neck_x = [0.08 * length, 0.15 * length];
    let neck_y = [-0.04 * width, 0.04 * width];
    middle.extend(block(neck_x, neck_y, [top, eye_z], Part::Neck, Part::Neck));
    // The eyes, at the front, looking forward: a shell, its glass, the light in it.
    let eye_x = 0.22 * length;
    let eye_r = 0.19 * width;
    let apart = 0.2 * width;
    middle.push(Piece::polygon(
        Part::Neck,
        vec![
            Xyz::new(eye_x - 0.3 * eye_r, -apart, eye_z),
            Xyz::new(eye_x + 0.3 * eye_r, -apart, eye_z),
            Xyz::new(eye_x + 0.3 * eye_r, apart, eye_z),
            Xyz::new(eye_x - 0.3 * eye_r, apart, eye_z),
        ],
    ));
    for side in [-1.0, 1.0] {
        let y = side * apart;
        middle.push(Piece::circle(Part::Eye, Xyz::new(eye_x, y, eye_z), eye_r));
        let lens = Xyz::new(eye_x + 0.3 * eye_r, y, eye_z + 0.01);
        middle.push(Piece::circle(Part::Lens, lens, 0.62 * eye_r));
        let glint = Xyz::new(lens.x + 0.22 * eye_r, y + 0.18 * eye_r, eye_z + 0.02);
        middle.push(Piece::circle(Part::Glint, glint, 0.16 * eye_r));
    }
    TopView { treads, middle }
}

/// A box: its four sides (in `side`'s colour), then its top (in `top`'s).
fn block(x: [f64; 2], y: [f64; 2], z: [f64; 2], side: Part, top: Part) -> [Piece<Xyz>; 5] {
    let corner = |i: usize, z: f64| {
        // Round the footprint: back right, front right, front left, back left.
        let (cx, cy) = [(x[0], y[0]), (x[1], y[0]), (x[1], y[1]), (x[0], y[1])][i];
        Xyz::new(cx, cy, z)
    };
    let wall = |i: usize| {
        let j = (i + 1) % 4;
        Piece::polygon(
            side,
            vec![
                corner(i, z[0]),
                corner(j, z[0]),
                corner(j, z[1]),
                corner(i, z[1]),
            ],
        )
    };
    [
        wall(0),
        wall(1),
        wall(2),
        wall(3),
        Piece::polygon(top, (0..4).map(|i| corner(i, z[1])).collect()),
    ]
}

/// Metres rolled, from the car's positions frame by frame: what turns the treads.
/// A jump (a relocalization, a new map) doesn't count as rolling.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Odometer {
    last: Option<Xy>,
    /// Metres rolled so far, forward positive.
    pub travelled: f64,
}

impl Odometer {
    /// More than this between two frames is a jump, not a roll.
    pub const JUMP: f64 = 5.0;

    /// The car is at `at` (metres), facing `yaw` (radians): adds the step since the
    /// last call, forward positive. Returns [`Odometer::travelled`].
    pub fn roll(&mut self, at: Xy, yaw: f64) -> f64 {
        if let Some(last) = self.last {
            let (dx, dy) = (at.x - last.x, at.y - last.y);
            let step = dx.hypot(dy);
            if step <= Self::JUMP {
                let forward = dx * yaw.cos() + dy * yaw.sin();
                self.travelled += step.copysign(forward);
            }
        }
        self.last = Some(at);
        self.travelled
    }
}

// ---------------------------------------------------------------------------------
// Peeking over the bottom edge.
// ---------------------------------------------------------------------------------

/// A box on the screen, in pixels: its top-left corner and its size.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Strip {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

impl Strip {
    pub const fn new(x: f64, y: f64, width: f64, height: f64) -> Self {
        Self {
            x,
            y,
            width,
            height,
        }
    }

    /// Its bottom edge's y.
    pub fn bottom(&self) -> f64 {
        self.y + self.height
    }

    /// Whether `p` is inside (edges included).
    pub fn contains(&self, p: Xy) -> bool {
        p.x >= self.x && p.x <= self.x + self.width && p.y >= self.y && p.y <= self.bottom()
    }
}

/// How the peeking eyes look, at one moment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PeekPose {
    /// 0 = still under the edge, 1 = fully up.
    pub rise: f64,
    /// Where they look: -1 left, 0 ahead, 1 right.
    pub look: f64,
    /// The lids: 0 open, 1 shut.
    pub lid: f64,
}

/// An eye's radius in a strip `height` high: the eyes, fully up, just fit.
pub fn peek_radius(height: f64) -> f64 {
    height / PEEK_HEIGHTS
}

/// The eyes' whole width, for a strip `height` high: what the strip must leave free.
pub fn peek_width(height: f64) -> f64 {
    2.0 * (EYES_APART + 1.0) * peek_radius(height)
}

/// Eye radii from the bottom edge to the top of an eye, fully up.
const PEEK_HEIGHTS: f64 = 2.4;
/// Eye centres from the middle, in eye radii.
const EYES_APART: f64 = 1.08;
/// Points round a lid's curve.
const LID_POINTS: usize = 16;

/// The two eyes rising over `strip`'s bottom edge, their middle at `centre_x`. Clip to
/// the strip when drawing: what is under the edge is hidden.
pub fn peek(strip: Strip, centre_x: f64, pose: PeekPose) -> Vec<Piece<Xy>> {
    let r = peek_radius(strip.height);
    let rise = pose.rise.clamp(0.0, 1.0);
    // Fully down: the eyes' tops sit on the edge; fully up: an eye's top at the strip's.
    let cy = strip.bottom() + r - rise * (PEEK_HEIGHTS * r);
    let look = pose.look.clamp(-1.0, 1.0);
    let lid = pose.lid.clamp(0.0, 1.0);
    let mut out = Vec::new();
    // The neck, from under the edge up between the eyes, and the bridge.
    let neck = 0.22 * r;
    out.push(Piece::polygon(
        Part::Neck,
        vec![
            Xy::new(centre_x - neck, cy),
            Xy::new(centre_x + neck, cy),
            Xy::new(centre_x + neck, strip.bottom() + r),
            Xy::new(centre_x - neck, strip.bottom() + r),
        ],
    ));
    out.push(Piece::polygon(
        Part::Neck,
        vec![
            Xy::new(centre_x - EYES_APART * r, cy - 0.18 * r),
            Xy::new(centre_x + EYES_APART * r, cy - 0.18 * r),
            Xy::new(centre_x + EYES_APART * r, cy + 0.22 * r),
            Xy::new(centre_x - EYES_APART * r, cy + 0.22 * r),
        ],
    ));
    for side in [-1.0, 1.0] {
        let c = Xy::new(centre_x + side * EYES_APART * r, cy);
        out.push(Piece::circle(Part::Eye, c, r));
        let lens = Xy::new(c.x + look * 0.24 * r, c.y + 0.06 * r);
        out.push(Piece::circle(Part::Lens, lens, 0.62 * r));
        let glint = Xy::new(lens.x - 0.2 * r, lens.y - 0.22 * r);
        out.push(Piece::circle(Part::Glint, glint, 0.16 * r));
        if let Some(cap) = lid_cap(c, r * 1.02, lid) {
            out.push(Piece::polygon(Part::Eye, cap));
        }
    }
    out
}

/// The part of a circle above a line `shut` of the way down it (screen y down): the lid.
/// `None` when open.
fn lid_cap(c: Xy, r: f64, shut: f64) -> Option<Vec<Xy>> {
    if shut <= 0.0 {
        return None;
    }
    // sin(angle) below s: the circle's points above the line y = c.y + s r.
    let s = (2.0 * shut - 1.0).clamp(-1.0, 1.0);
    let alpha = s.asin();
    let (from, to) = (PI - alpha, TAU + alpha);
    Some(
        (0..=LID_POINTS)
            .map(|i| {
                let a = from + (to - from) * i as f64 / LID_POINTS as f64;
                Xy::new(c.x + r * a.cos(), c.y + r * a.sin())
            })
            .collect(),
    )
}

// ---------------------------------------------------------------------------------
// Driving across, from the side.
// ---------------------------------------------------------------------------------

/// The robot from the side, `height` pixels tall, standing on `ground` (its middle's x,
/// the ground's y; screen y down), facing right or left. `rolled`: pixels it has driven;
/// it turns the wheels. Its outline fits [`side_width`] by `height`.
pub fn side_view(ground: Xy, height: f64, rolled: f64, facing_right: bool) -> Vec<Piece<Xy>> {
    let h = height;
    let dir = if facing_right { 1.0 } else { -1.0 };
    // In units of h: x forward from the middle, up from the ground.
    let at = |x: f64, up: f64| Xy::new(ground.x + dir * x * h, ground.y - up * h);
    let mut out = Vec::new();
    // The tread: a long rounded loop, its wheels, a dot on each that turns.
    let (tread_r, tread_len) = (0.15, 0.36);
    let mut belt: Vec<Xy> = (0..=10)
        .map(|i| {
            let a = -PI / 2.0 + PI * f64::from(i) / 10.0;
            at(tread_len + tread_r * a.cos(), tread_r - tread_r * a.sin())
        })
        .collect();
    belt.extend((0..=10).map(|i| {
        let a = PI / 2.0 + PI * f64::from(i) / 10.0;
        at(-tread_len + tread_r * a.cos(), tread_r - tread_r * a.sin())
    }));
    out.push(Piece::polygon(Part::Tread, belt));
    let wheel_r = 0.1;
    let turned = rolled / (wheel_r * h);
    for x in [-tread_len, 0.0, tread_len] {
        let c = at(x, tread_r);
        out.push(Piece::circle(Part::Cleat, c, wheel_r * h));
        let spoke = Xy::new(
            c.x + 0.55 * wheel_r * h * turned.cos(),
            c.y + 0.55 * wheel_r * h * turned.sin(),
        );
        out.push(Piece::circle(Part::Lens, spoke, 0.3 * wheel_r * h));
    }
    // The body: its side lit, its front in shade, a hatch line.
    let (back, front, low, high) = (-0.38, 0.3, 0.26, 0.7);
    let rect = |part, x0: f64, x1: f64, y0: f64, y1: f64| {
        Piece::polygon(part, vec![at(x0, y0), at(x1, y0), at(x1, y1), at(x0, y1)])
    };
    out.push(rect(Part::Body, back, front - 0.1, low, high));
    out.push(rect(Part::BodySide, front - 0.1, front, low, high));
    out.push(rect(Part::Neck, back, front - 0.1, 0.5, 0.53));
    // The neck, then the eyes: the far one a little behind and higher, then the near
    // one; each a round shell, its glass looking ahead.
    out.push(rect(Part::Neck, 0.08, 0.15, high, 0.84));
    let eye_r = 0.155;
    for (x, up) in [(0.1, 1.0 - eye_r + 0.0), (0.17, 1.0 - eye_r - 0.03)] {
        out.push(Piece::circle(Part::Eye, at(x, up), eye_r * h));
        let lens = at(x + 0.055, up);
        out.push(Piece::circle(Part::Lens, lens, 0.62 * eye_r * h));
        let glint = Xy::new(lens.x + dir * 0.04 * h, lens.y - 0.035 * h);
        out.push(Piece::circle(Part::Glint, glint, 0.16 * eye_r * h));
    }
    out
}

/// The side view's whole width, for a robot `height` tall.
pub fn side_width(height: f64) -> f64 {
    2.0 * 0.51 * height
}

// ---------------------------------------------------------------------------------
// When it shows.
// ---------------------------------------------------------------------------------

/// The easter egg's settings, from the screen's yaml.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EggSettings {
    /// `easter_egg`: false = never.
    pub on: bool,
    /// `easter_egg_after`: quiet this long (no input, nothing changed, the car still)
    /// before the eyes peek.
    pub idle_after: Duration,
}

/// What the screen sees this frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Watch {
    /// A key, a click, the mouse moved, the wheel.
    pub input: bool,
    /// The car is standing still (or no car is heard).
    pub still: bool,
    /// An e-stop is on, or a warning shows.
    pub alarm: bool,
    /// A summary of what the screen shows (the page, its words' states): any change
    /// hides the robot and starts the quiet time again.
    pub scene: u64,
}

/// What to draw this frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Show {
    Nothing,
    /// The eyes over the bottom edge.
    Peek(PeekPose),
    /// The robot driving across: `progress` 0 (off the left edge) to 1 (off the right).
    Drive {
        progress: f64,
    },
}

/// How long a drive across takes.
pub const DRIVE_TIME: Duration = Duration::from_secs(5);
/// A drive asked for waits for this much quiet first (the start-up's first news).
pub const DRIVE_SETTLE: Duration = Duration::from_secs(2);
/// A drive still waiting after this is dropped.
pub const DRIVE_WAIT: Duration = Duration::from_secs(20);
/// The eyes' rise.
const RISE: Duration = Duration::from_millis(1200);
/// One look round: ahead, left, right, ahead again.
const LOOK_ROUND: f64 = 10.0;
/// A blink.
const BLINK: f64 = 0.2;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Act {
    Hidden,
    Peeking { since: Instant },
    Driving { since: Instant },
}

/// The easter egg's state: when the eyes peek, when the robot drives across, and when
/// it hides. Feed it one [`Watch`] a frame.
///
/// - It never shows while the car moves, an e-stop is on or a warning shows, and hides
///   the moment anything changes or a person touches a key or the mouse.
/// - The eyes peek after [`EggSettings::idle_after`] of quiet; they blink now and then.
/// - [`EasterEgg::drive_across`] asks for one drive; it goes after [`DRIVE_SETTLE`] of
///   quiet, or never if none comes within [`DRIVE_WAIT`].
#[derive(Debug, Clone, PartialEq)]
pub struct EasterEgg {
    settings: EggSettings,
    act: Act,
    quiet_since: Instant,
    scene: Option<u64>,
    drive_asked: Option<Instant>,
    /// Blinks differ from one peek to the next.
    peeks: u64,
}

impl EasterEgg {
    pub fn new(settings: EggSettings, now: Instant) -> Self {
        Self {
            settings,
            act: Act::Hidden,
            quiet_since: now,
            scene: None,
            drive_asked: None,
            peeks: 0,
        }
    }

    /// Asks for one drive across (start-up, a lock-to-lock check passed).
    pub fn drive_across(&mut self, now: Instant) {
        if self.settings.on {
            self.drive_asked = Some(now);
        }
    }

    /// One frame: what to draw.
    pub fn update(&mut self, now: Instant, watch: Watch) -> Show {
        let changed = self.scene.is_some_and(|s| s != watch.scene);
        self.scene = Some(watch.scene);
        if !self.settings.on {
            self.act = Act::Hidden;
            return Show::Nothing;
        }
        if watch.input || changed || !watch.still || watch.alarm {
            self.act = Act::Hidden;
            self.quiet_since = now;
            return Show::Nothing;
        }
        if self
            .drive_asked
            .is_some_and(|at| now.saturating_duration_since(at) > DRIVE_WAIT)
        {
            self.drive_asked = None;
        }
        let quiet = now.saturating_duration_since(self.quiet_since);
        if self.drive_asked.is_some() && quiet >= DRIVE_SETTLE {
            self.drive_asked = None;
            self.act = Act::Driving { since: now };
        }
        match self.act {
            Act::Driving { since } => {
                let t = now.saturating_duration_since(since);
                if t < DRIVE_TIME {
                    return Show::Drive {
                        progress: t.as_secs_f64() / DRIVE_TIME.as_secs_f64(),
                    };
                }
                self.act = Act::Hidden;
                self.quiet_since = now;
                Show::Nothing
            }
            Act::Peeking { since } => {
                Show::Peek(peek_pose(now.saturating_duration_since(since), self.peeks))
            }
            Act::Hidden if self.drive_asked.is_none() && quiet >= self.settings.idle_after => {
                self.peeks += 1;
                self.act = Act::Peeking { since: now };
                Show::Peek(peek_pose(Duration::ZERO, self.peeks))
            }
            Act::Hidden => Show::Nothing,
        }
    }
}

/// The eyes `t` after they started to peek: rise, look ahead, left, right, ahead; blink
/// now and then (`seed` picks when).
pub fn peek_pose(t: Duration, seed: u64) -> PeekPose {
    let secs = t.as_secs_f64();
    let rise = ease((secs / RISE.as_secs_f64()).min(1.0));
    // Ahead 0-2 s, to the left by 3, right by 5.5, ahead again by 8.
    let round = secs % LOOK_ROUND;
    let keys = [
        (0.0, 0.0),
        (2.0, 0.0),
        (3.0, -1.0),
        (4.5, -1.0),
        (5.5, 1.0),
        (7.0, 1.0),
        (8.0, 0.0),
        (LOOK_ROUND, 0.0),
    ];
    let look = keys.windows(2).find(|w| round < w[1].0).map_or(0.0, |w| {
        let (t0, a) = w[0];
        let (t1, b) = w[1];
        a + (b - a) * ease((round - t0) / (t1 - t0))
    });
    PeekPose {
        rise,
        look,
        lid: lid_at(secs, seed),
    }
}

/// The lids at `secs` into a peek: shut for [`BLINK`] once every 2.5 to 6 s.
fn lid_at(secs: f64, seed: u64) -> f64 {
    let mut start = 1.5;
    let mut k = 0;
    while start <= secs {
        let into = secs - start;
        if into < BLINK {
            // Down then up.
            return 1.0 - (2.0 * into / BLINK - 1.0).abs();
        }
        start += 2.5 + 3.5 * unit(seed, k);
        k += 1;
    }
    0.0
}

/// A number from 0 to 1, the same each time for the same seed and step (splitmix64).
fn unit(seed: u64, step: u64) -> f64 {
    let mut z = seed
        .wrapping_mul(0x9e37_79b9_7f4a_7c15)
        .wrapping_add(step.wrapping_add(1).wrapping_mul(0xbf58_476d_1ce4_e5b9));
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^= z >> 31;
    (z >> 11) as f64 / (1u64 << 53) as f64
}

/// Smooth in and out, 0 to 1.
fn ease(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SETTINGS: EggSettings = EggSettings {
        on: true,
        idle_after: Duration::from_secs(60),
    };

    fn calm(scene: u64) -> Watch {
        Watch {
            input: false,
            still: true,
            alarm: false,
            scene,
        }
    }

    fn secs(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    #[test]
    fn the_treads_fill_the_footprint_exactly() {
        let v = top_view(2.0, 1.0, 0.0);
        let ground: Vec<Xyz> = v
            .treads
            .iter()
            .flatten()
            .flat_map(Piece::points)
            .filter(|p| p.z == 0.0)
            .collect();
        let min_x = ground.iter().map(|p| p.x).fold(f64::MAX, f64::min);
        let max_x = ground.iter().map(|p| p.x).fold(f64::MIN, f64::max);
        let min_y = ground.iter().map(|p| p.y).fold(f64::MAX, f64::min);
        let max_y = ground.iter().map(|p| p.y).fold(f64::MIN, f64::max);
        assert_eq!((min_x, max_x, min_y, max_y), (-1.0, 1.0, -0.5, 0.5));
        // Right tread first, then left.
        assert!(v.treads[0]
            .iter()
            .flat_map(Piece::points)
            .all(|p| p.y < 0.0));
        assert!(v.treads[1]
            .iter()
            .flat_map(Piece::points)
            .all(|p| p.y > 0.0));
    }

    #[test]
    fn everything_stays_inside_the_footprint() {
        let v = top_view(2.0, 1.0, 0.37);
        for piece in v.treads.iter().flatten().chain(&v.middle) {
            for p in piece.points() {
                assert!(
                    p.x.abs() <= 1.0 + 1e-9 && p.y.abs() <= 0.5 + 1e-9,
                    "{piece:?}"
                );
            }
        }
    }

    #[test]
    fn the_eyes_face_forward_and_sit_on_top() {
        let v = top_view(2.0, 1.0, 0.0);
        let eyes: Vec<&Piece<Xyz>> = v.middle.iter().filter(|p| p.part == Part::Eye).collect();
        assert_eq!(eyes.len(), 2);
        let top = v
            .middle
            .iter()
            .flat_map(Piece::points)
            .fold(0.0, |m, p| p.z.max(m));
        for eye in eyes {
            let Form::Circle { centre, .. } = eye.form else {
                panic!("an eye is round");
            };
            assert!(centre.x > 0.0, "the eyes are at the front");
            assert!(
                (centre.z - top).abs() < 0.05,
                "the eyes are the highest part"
            );
        }
        // Each lens is ahead of its shell's middle: looking forward.
        let lenses = v.middle.iter().filter(|p| p.part == Part::Lens);
        for (lens, eye) in lenses.zip(v.middle.iter().filter(|p| p.part == Part::Eye)) {
            assert!(lens.points()[0].x > eye.points()[0].x);
        }
    }

    #[test]
    fn the_cleats_roll_with_the_robot() {
        let cleats = |travelled| -> Vec<f64> {
            top_view(2.0, 1.0, travelled).treads[0]
                .iter()
                .filter(|p| p.part == Part::Cleat)
                .map(|p| p.points()[0].x)
                .collect()
        };
        let still = cleats(0.0);
        let moved = cleats(0.05);
        assert_ne!(still, moved);
        // A whole spacing on, they look the same again.
        let again = cleats(2.0 / CLEATS as f64);
        for (a, b) in still.iter().zip(&again) {
            assert!((a - b).abs() < 1e-9);
        }
    }

    #[test]
    fn the_odometer_counts_rolling_not_jumps() {
        let mut o = Odometer::default();
        assert_eq!(o.roll(Xy::new(0.0, 0.0), 0.0), 0.0);
        assert!((o.roll(Xy::new(1.0, 0.0), 0.0) - 1.0).abs() < 1e-12);
        // Backwards counts down.
        assert!((o.roll(Xy::new(0.5, 0.0), 0.0) - 0.5).abs() < 1e-12);
        // A jump of 100 m: a new position, not a roll.
        assert!((o.roll(Xy::new(100.0, 0.0), 0.0) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn the_eyes_rise_over_the_edge_and_stay_in_the_strip() {
        let strip = Strip::new(100.0, 500.0, 400.0, 120.0);
        let down = peek(
            strip,
            300.0,
            PeekPose {
                rise: 0.0,
                look: 0.0,
                lid: 0.0,
            },
        );
        let up = peek(
            strip,
            300.0,
            PeekPose {
                rise: 1.0,
                look: 0.0,
                lid: 0.0,
            },
        );
        let r = peek_radius(strip.height);
        let eye_tops = |pieces: &[Piece<Xy>]| -> Vec<f64> {
            pieces
                .iter()
                .filter_map(|p| match p.form {
                    Form::Circle { centre, radius } if p.part == Part::Eye => {
                        Some(centre.y - radius)
                    }
                    _ => None,
                })
                .collect()
        };
        for top in eye_tops(&down) {
            assert!(top >= strip.bottom() - 1e-9, "down: hidden under the edge");
        }
        for top in eye_tops(&up) {
            assert!(top >= strip.y - 1e-9, "up: inside the strip");
            assert!(top < strip.bottom() - r, "up: well over the edge");
        }
        // Side to side, inside the width peek_width says.
        let half = peek_width(strip.height) / 2.0;
        for p in up.iter().filter(|p| p.part == Part::Eye) {
            if let Form::Circle { centre, radius } = p.form {
                assert!((centre.x - 300.0).abs() + radius <= half + 1e-9);
            }
        }
    }

    #[test]
    fn the_eyes_look_where_told() {
        let strip = Strip::new(0.0, 0.0, 400.0, 120.0);
        let lens_x = |look| {
            peek(
                strip,
                200.0,
                PeekPose {
                    rise: 1.0,
                    look,
                    lid: 0.0,
                },
            )
            .iter()
            .find(|p| p.part == Part::Lens)
            .map(|p| p.points()[0].x)
            .expect("a lens")
        };
        assert!(lens_x(-1.0) < lens_x(0.0) && lens_x(0.0) < lens_x(1.0));
    }

    #[test]
    fn a_lid_closes_from_the_top() {
        let c = Xy::new(0.0, 0.0);
        assert!(lid_cap(c, 10.0, 0.0).is_none());
        let half = lid_cap(c, 10.0, 0.5).expect("half shut");
        // Half shut: the top half, down to the middle.
        assert!(half.iter().all(|p| p.y <= 1e-9));
        assert!(half.iter().any(|p| (p.y + 10.0).abs() < 0.5));
        let shut = lid_cap(c, 10.0, 1.0).expect("shut");
        assert!(shut.iter().any(|p| p.y > 9.5), "shut: down to the bottom");
    }

    #[test]
    fn the_side_view_fits_its_box() {
        for facing_right in [true, false] {
            let pieces = side_view(Xy::new(500.0, 300.0), 80.0, 12.0, facing_right);
            let half = side_width(80.0) / 2.0;
            for piece in &pieces {
                let r = match piece.form {
                    Form::Circle { radius, .. } => radius,
                    Form::Polygon(_) => 0.0,
                };
                for p in piece.points() {
                    assert!((p.x - 500.0).abs() + r <= half + 1e-9, "{piece:?}");
                    assert!(
                        p.y + r <= 300.0 + 1e-9 && p.y - r >= 220.0 - 1e-9,
                        "{piece:?}"
                    );
                }
            }
            // The eye's glass is on the side it faces.
            let eye = pieces
                .iter()
                .find(|p| p.part == Part::Eye)
                .unwrap()
                .points()[0];
            let lens = pieces.iter().rfind(|p| p.part == Part::Lens).unwrap();
            assert_eq!(lens.points()[0].x > eye.x, facing_right);
        }
    }

    #[test]
    fn the_eyes_peek_after_the_quiet_time() {
        let t0 = Instant::now();
        let mut egg = EasterEgg::new(SETTINGS, t0);
        assert_eq!(egg.update(t0, calm(1)), Show::Nothing);
        assert_eq!(egg.update(t0 + secs(59.0), calm(1)), Show::Nothing);
        assert!(matches!(
            egg.update(t0 + secs(60.0), calm(1)),
            Show::Peek(_)
        ));
        // And they stay while nothing happens.
        assert!(matches!(
            egg.update(t0 + secs(90.0), calm(1)),
            Show::Peek(_)
        ));
    }

    #[test]
    fn anything_hides_them_at_once() {
        let hiders = [
            Watch {
                input: true,
                ..calm(1)
            },
            Watch {
                still: false,
                ..calm(1)
            },
            Watch {
                alarm: true,
                ..calm(1)
            },
            calm(2),
        ];
        for hider in hiders {
            let t0 = Instant::now();
            let mut egg = EasterEgg::new(SETTINGS, t0);
            egg.update(t0, calm(1));
            assert!(matches!(
                egg.update(t0 + secs(61.0), calm(1)),
                Show::Peek(_)
            ));
            assert_eq!(
                egg.update(t0 + secs(62.0), hider),
                Show::Nothing,
                "{hider:?}"
            );
            // The quiet time starts again.
            let scene = hider.scene;
            assert_eq!(egg.update(t0 + secs(100.0), calm(scene)), Show::Nothing);
            assert!(matches!(
                egg.update(t0 + secs(122.0), calm(scene)),
                Show::Peek(_)
            ));
        }
    }

    #[test]
    fn never_while_moving_or_warning() {
        let t0 = Instant::now();
        let mut egg = EasterEgg::new(SETTINGS, t0);
        egg.drive_across(t0);
        for s in 0..200 {
            let moving = Watch {
                still: false,
                ..calm(1)
            };
            assert_eq!(egg.update(t0 + secs(f64::from(s)), moving), Show::Nothing);
        }
        let mut egg = EasterEgg::new(SETTINGS, t0);
        for s in 0..200 {
            let warning = Watch {
                alarm: true,
                ..calm(1)
            };
            assert_eq!(egg.update(t0 + secs(f64::from(s)), warning), Show::Nothing);
        }
    }

    #[test]
    fn off_means_never() {
        let t0 = Instant::now();
        let mut egg = EasterEgg::new(
            EggSettings {
                on: false,
                ..SETTINGS
            },
            t0,
        );
        egg.drive_across(t0);
        for s in 0..300 {
            assert_eq!(egg.update(t0 + secs(f64::from(s)), calm(1)), Show::Nothing);
        }
    }

    #[test]
    fn a_drive_waits_for_quiet_then_crosses() {
        let t0 = Instant::now();
        let mut egg = EasterEgg::new(SETTINGS, t0);
        egg.drive_across(t0);
        assert_eq!(egg.update(t0, calm(1)), Show::Nothing);
        // The first news changes the screen: the wait starts again.
        assert_eq!(egg.update(t0 + secs(1.0), calm(2)), Show::Nothing);
        assert_eq!(egg.update(t0 + secs(2.5), calm(2)), Show::Nothing);
        let Show::Drive { progress } = egg.update(t0 + secs(3.0), calm(2)) else {
            panic!("drives after 2 s of quiet");
        };
        assert_eq!(progress, 0.0);
        let Show::Drive { progress } = egg.update(t0 + secs(5.5), calm(2)) else {
            panic!("still driving");
        };
        assert!((progress - 0.5).abs() < 1e-9);
        assert_eq!(egg.update(t0 + secs(8.0), calm(2)), Show::Nothing);
        // Once only.
        assert_eq!(egg.update(t0 + secs(20.0), calm(2)), Show::Nothing);
    }

    #[test]
    fn a_drive_that_never_gets_quiet_is_dropped() {
        let t0 = Instant::now();
        let mut egg = EasterEgg::new(SETTINGS, t0);
        egg.drive_across(t0);
        for s in 0..25 {
            egg.update(t0 + secs(f64::from(s)), calm(u64::try_from(s).unwrap()));
        }
        assert_eq!(egg.update(t0 + secs(26.0), calm(24)), Show::Nothing);
        assert_eq!(egg.update(t0 + secs(30.0), calm(24)), Show::Nothing);
    }

    #[test]
    fn the_eyes_rise_look_round_and_blink() {
        assert_eq!(peek_pose(Duration::ZERO, 1).rise, 0.0);
        assert_eq!(peek_pose(secs(2.0), 1).rise, 1.0);
        assert!(peek_pose(secs(3.5), 1).look < -0.99, "left");
        assert!(peek_pose(secs(6.0), 1).look > 0.99, "right");
        assert!(peek_pose(secs(9.0), 1).look.abs() < 1e-9, "ahead");
        // The first blink at 1.5 s: shut half way through it.
        assert!((peek_pose(secs(1.6), 1).lid - 1.0).abs() < 1e-9);
        assert_eq!(peek_pose(secs(1.0), 1).lid, 0.0);
        // Blinks keep coming, 2.5 to 6 s apart, and the same seed blinks the same.
        let blinks = |seed| -> Vec<u32> {
            let shut: Vec<bool> = (0..3000)
                .map(|i| peek_pose(secs(f64::from(i) * 0.01), seed).lid > 0.0)
                .collect();
            (1..3000u32)
                .filter(|&i| shut[i as usize] && !shut[i as usize - 1])
                .collect()
        };
        let starts = blinks(7);
        assert!(starts.len() >= 5, "{starts:?}");
        assert!(starts
            .windows(2)
            .all(|w| (249..=601).contains(&(w[1] - w[0]))));
        assert_eq!(starts, blinks(7));
    }

    #[test]
    fn random_steps_stay_in_zero_to_one() {
        for seed in 0..20 {
            for step in 0..50 {
                let u = unit(seed, step);
                assert!((0.0..1.0).contains(&u));
            }
        }
        assert_ne!(unit(1, 0), unit(2, 0));
    }

    #[test]
    fn strips_hold_their_points() {
        let s = Strip::new(10.0, 20.0, 100.0, 50.0);
        assert_eq!(s.bottom(), 70.0);
        assert!(s.contains(Xy::new(10.0, 70.0)));
        assert!(!s.contains(Xy::new(9.0, 30.0)));
    }
}
