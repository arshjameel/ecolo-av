//! Passing a parked obstacle on the lane line (Ben, 16 Nov: "stick to the right side of
//! the lane unless we need to for obstacle avoidance"). Autoware's static obstacle
//! avoidance, on our lane line: the path stays the lane's centre, and only the blocked
//! stretch moves left, by the least that clears it, then back.
//!
//! ```text
//!                 ramp in    hold (the blocked stretch + margins)   ramp out
//!   lane line ═══════╗ ╭──────────────────────────────────────╮ ╔══════════════ ▶
//!                    ╚═╯               ▓ the box               ╚═╝
//!            (left by `shift` m: the smallest that fits, plus the clearance)
//! ```
//!
//! | Step | What |
//! |---|---|
//! | blocked | each lane-line pose where the car's padded outline doesn't fit (the search's own check) |
//! | stretches | the blocked poses in runs, `margin` m added each side; runs closer than `merge` m are one |
//! | shift | for each stretch, the smallest left step (every `step` m) whose whole ramp-hold-ramp fits; then `clearance` more if that fits too. No more than the room to the left kerb |
//! | ramps | a half cosine, at least `comfort` times the turning radius's curvature away (shorter only where the car is already close: never sharper than the radius) |
//! | can't | the goal inside a stretch, or no shift up to the kerb fits (the other lane is taken too): [`NoRoom`], the caller's fallback |
//!
//! The yaw of each moved pose turns by the ramp's slope. Poses outside every bump are the
//! lane line's own, unchanged.

use std::f64::consts::PI;

use crate::Pose;

/// How a pass is shaped.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PassSettings {
    /// m added before and after each blocked run, at the full shift.
    pub margin: f64,
    /// m between the shifts tried.
    pub step: f64,
    /// m more than the least shift that fits, when that fits too.
    pub clearance: f64,
    /// A ramp's sharpest curvature is the turning radius's over this (2: twice the radius).
    pub comfort: f64,
    /// m: blocked runs closer than this are passed in one.
    pub merge: f64,
}

impl Default for PassSettings {
    fn default() -> Self {
        Self {
            margin: 1.0,
            step: 0.25,
            clearance: 0.3,
            comfort: 2.0,
            merge: 15.0,
        }
    }
}

/// No pass fits: the stretch is blocked to the kerb, or the goal is inside it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NoRoom;

/// One stretch passed: held `shift` m left from `start` to `end` (m along the line),
/// eased in over `ramp_in` m before and out over `ramp_out` m after.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bump {
    /// m along the line where the full shift begins.
    pub start: f64,
    /// m along the line where it ends.
    pub end: f64,
    /// m left of the lane line.
    pub shift: f64,
    /// m of ramp before `start`.
    pub ramp_in: f64,
    /// m of ramp after `end`.
    pub ramp_out: f64,
}

impl Bump {
    /// The shift at `s` m along, and its slope (m sideways a m along).
    #[must_use]
    pub fn at(&self, s: f64) -> (f64, f64) {
        let half_cos = |u: f64, len: f64| {
            (
                0.5 * self.shift * (1.0 - (PI * u).cos()),
                0.5 * self.shift * PI / len * (PI * u).sin(),
            )
        };
        if s >= self.start && s <= self.end {
            (self.shift, 0.0)
        } else if s < self.start && s > self.start - self.ramp_in {
            half_cos(
                (s - (self.start - self.ramp_in)) / self.ramp_in,
                self.ramp_in,
            )
        } else if s > self.end && s < self.end + self.ramp_out {
            let (o, slope) = half_cos(
                (self.end + self.ramp_out - s) / self.ramp_out,
                self.ramp_out,
            );
            (o, -slope)
        } else {
            (0.0, 0.0)
        }
    }

    /// m along where it begins and ends, ramps and all.
    #[must_use]
    pub fn span(&self) -> (f64, f64) {
        (self.start - self.ramp_in, self.end + self.ramp_out)
    }
}

/// The shift at `s` from every bump: the largest, and its slope.
fn offset_at(bumps: &[Bump], s: f64) -> (f64, f64) {
    bumps
        .iter()
        .map(|b| b.at(s))
        .fold((0.0, 0.0), |best, o| if o.0 > best.0 { o } else { best })
}

/// m of ramp for a `shift` at turning radius `radius`: a half cosine whose sharpest
/// curvature (shift x pi² / 2 L²) is 1 / (radius x `comfort`).
#[must_use]
pub fn ramp_length(shift: f64, radius: f64, comfort: f64) -> f64 {
    PI * (shift.abs() * radius * comfort / 2.0).sqrt()
}

/// The blocked runs of `blocked` (one flag a pose, `along` its metres), each widened by
/// `margin` and merged when closer than `merge`: (start, end) m along.
#[must_use]
pub fn stretches(along: &[f64], blocked: &[bool], margin: f64, merge: f64) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::new();
    let mut run: Option<(f64, f64)> = None;
    for (&s, &b) in along.iter().zip(blocked) {
        run = match (run, b) {
            (None, true) => Some((s, s)),
            (Some((a, _)), true) => Some((a, s)),
            (Some(r), false) => {
                out.push(r);
                None
            }
            (None, false) => None,
        };
    }
    out.extend(run);
    let mut merged: Vec<(f64, f64)> = Vec::new();
    for (a, b) in out.into_iter().map(|(a, b)| (a - margin, b + margin)) {
        match merged.last_mut() {
            Some(last) if a - last.1 < merge => last.1 = b,
            _ => merged.push((a, b)),
        }
    }
    merged
}

/// `line` moved left by `bumps` (`along`: its metres): each pose sideways by the shift,
/// its yaw turned by the slope. Poses outside every bump are unchanged.
#[must_use]
pub fn shifted(line: &[Pose], along: &[f64], bumps: &[Bump]) -> Vec<Pose> {
    line.iter()
        .zip(along)
        .map(|(p, &s)| {
            let (o, slope) = offset_at(bumps, s);
            if o <= 0.0 {
                return *p;
            }
            let (sin, cos) = p.yaw.sin_cos();
            Pose::new(p.x - o * sin, p.y + o * cos, p.yaw + slope.atan())
        })
        .collect()
}

/// A pass: the moved line, and each stretch's bump.
#[derive(Debug, Clone)]
pub struct Pass {
    /// The lane line, moved left round every blocked stretch.
    pub line: Vec<Pose>,
    /// One a stretch.
    pub bumps: Vec<Bump>,
}

impl Pass {
    /// m: the largest shift.
    #[must_use]
    pub fn shift(&self) -> f64 {
        self.bumps.iter().map(|b| b.shift).fold(0.0, f64::max)
    }
}

/// Pass whatever blocks `line` (a pose every few cm, driving order; `along` its metres):
/// `None` when nothing does. `fits`: does the car fit at a pose; `room`: m left of the
/// lane line the car may go at a pose (to the kerb, less half the car and a margin);
/// `radius`: the turning radius (m).
///
/// # Errors
///
/// [`NoRoom`] when the goal (the line's end) is blocked, or no shift up to `room` fits.
pub fn plan(
    line: &[Pose],
    along: &[f64],
    radius: f64,
    settings: &PassSettings,
    fits: &dyn Fn(&Pose) -> bool,
    room: &dyn Fn(&Pose) -> f64,
) -> Result<Option<Pass>, NoRoom> {
    let blocked: Vec<bool> = line.iter().map(|p| !fits(p)).collect();
    if !blocked.iter().any(|&b| b) {
        return Ok(None);
    }
    if blocked.last().copied().unwrap_or(false) {
        return Err(NoRoom);
    }
    let total = along.last().copied().unwrap_or(0.0);
    let mut bumps: Vec<Bump> = Vec::new();
    for (start, end) in stretches(along, &blocked, settings.margin, settings.merge) {
        let mid = along.partition_point(|&s| s < 0.5 * (start + end));
        let most = room(&line[mid.min(line.len() - 1)]);
        let bump = |shift: f64| {
            let want = ramp_length(shift, radius, settings.comfort);
            let least = ramp_length(shift, radius, 1.0);
            // Where the car is already near: a sharper ramp, never past the radius.
            let ramp_in = if start > 0.0 {
                want.min(start.max(least))
            } else {
                want
            };
            Bump {
                start,
                end,
                shift,
                ramp_in,
                ramp_out: want,
            }
        };
        let clear = |b: Bump| {
            let mut with = bumps.clone();
            with.push(b);
            let (a, z) = b.span();
            let moved = shifted(line, along, &with);
            moved
                .iter()
                .zip(along)
                .filter(|(_, &s)| s >= a && s <= z.min(total))
                .all(|(p, _)| fits(p))
        };
        let steps = (most / settings.step).floor().max(0.0) as usize;
        let least = (1..=steps)
            .map(|k| k as f64 * settings.step)
            .find(|&d| clear(bump(d)))
            .ok_or(NoRoom)?;
        let wider = (least + settings.clearance).min(most);
        let chosen = if wider > least && clear(bump(wider)) {
            wider
        } else {
            least
        };
        bumps.push(bump(chosen));
    }
    Ok(Some(Pass {
        line: shifted(line, along, &bumps),
        bumps,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// East along y = 0, a pose every 0.25 m for `metres`.
    fn straight(metres: f64) -> (Vec<Pose>, Vec<f64>) {
        let n = (metres / 0.25) as usize + 1;
        let line: Vec<Pose> = (0..n)
            .map(|i| Pose::new(i as f64 * 0.25, 0.0, 0.0))
            .collect();
        let along = line.iter().map(|p| p.x).collect();
        (line, along)
    }

    /// The car (2 m long from 0.2 behind base_link, 1 m wide) against a box from
    /// x0..x1, y0..y1: fits if they don't overlap (heading ignored: a straight road).
    fn boxed(x0: f64, x1: f64, y0: f64, y1: f64) -> impl Fn(&Pose) -> bool {
        move |p: &Pose| {
            let (cx0, cx1, cy0, cy1) = (p.x - 0.2, p.x + 1.8, p.y - 0.5, p.y + 0.5);
            cx1 <= x0 || cx0 >= x1 || cy1 <= y0 || cy0 >= y1
        }
    }

    #[test]
    fn a_clear_line_is_left_alone() {
        let (line, along) = straight(60.0);
        let fits = |_: &Pose| true;
        let pass = plan(&line, &along, 3.6, &PassSettings::default(), &fits, &|_| {
            4.0
        });
        assert!(matches!(pass, Ok(None)));
    }

    #[test]
    fn a_box_on_the_line_is_passed_by_the_least_shift_and_back() {
        let (line, along) = straight(80.0);
        // A 1 m box on the line at x 30-31.
        let fits = boxed(30.0, 31.0, -0.5, 0.5);
        let settings = PassSettings::default();
        let pass = plan(&line, &along, 3.6, &settings, &fits, &|_| 4.0)
            .expect("room")
            .expect("a pass");
        // The least that clears it: 1.0 m (0.5 + 0.5), then the clearance.
        assert_eq!(pass.bumps.len(), 1);
        assert!((pass.shift() - 1.3).abs() < 1e-9, "{}", pass.shift());
        assert!(pass.line.iter().all(&fits));
        // Back on the lane line well before the end, and on it before the ramp.
        let b = pass.bumps[0];
        let (a, z) = b.span();
        for (p, &s) in pass.line.iter().zip(&along) {
            if s < a || s > z {
                assert!(p.y.abs() < 1e-12 && p.yaw.abs() < 1e-12, "{s}: {p:?}");
            }
        }
        assert!(z < 60.0, "{z}");
        // Never sharper than the turning radius: the yaw turns at most 0.25 / 3.6 a step.
        let sharpest = pass
            .line
            .windows(2)
            .map(|w| (w[1].yaw - w[0].yaw).abs())
            .fold(0.0, f64::max);
        assert!(sharpest < 0.25 / 3.6, "{sharpest}");
    }

    #[test]
    fn a_box_half_in_the_lane_needs_less() {
        let (line, along) = straight(80.0);
        // It reaches 0.3 m into the car (y -0.8 to -0.2): 0.5 m clears it (a 0.25 m step), 0.8
        // with the clearance.
        let fits = boxed(30.0, 31.0, -0.8, -0.2);
        let pass = plan(&line, &along, 3.6, &PassSettings::default(), &fits, &|_| {
            4.0
        })
        .expect("room")
        .expect("a pass");
        assert!((pass.shift() - 0.8).abs() < 1e-9, "{}", pass.shift());
    }

    #[test]
    fn the_other_lane_taken_or_the_goal_blocked_is_no_room() {
        let (line, along) = straight(80.0);
        // The box and something in the other lane beside it: a 4 m wall.
        let wall = boxed(28.0, 33.0, -0.5, 3.5);
        let pass = plan(&line, &along, 3.6, &PassSettings::default(), &wall, &|_| {
            3.5
        });
        assert_eq!(pass.err(), Some(NoRoom));
        let (short, along) = straight(31.0);
        let at_goal = boxed(30.0, 31.0, -0.5, 0.5);
        let pass = plan(
            &short,
            &along,
            3.6,
            &PassSettings::default(),
            &at_goal,
            &|_| 4.0,
        );
        assert_eq!(pass.err(), Some(NoRoom));
    }

    #[test]
    fn close_runs_merge_far_ones_dont() {
        let along: Vec<f64> = (0..400).map(|i| f64::from(i) * 0.25).collect();
        let blocked: Vec<bool> = along
            .iter()
            .map(|&s| {
                (10.0..12.0).contains(&s) || (20.0..21.0).contains(&s) || (70.0..71.0).contains(&s)
            })
            .collect();
        let runs = stretches(&along, &blocked, 1.0, 15.0);
        assert_eq!(runs.len(), 2, "{runs:?}");
        assert!((runs[0].0 - 9.0).abs() < 1e-9 && (runs[0].1 - 21.75).abs() < 1e-9);
        assert!((runs[1].0 - 69.0).abs() < 1e-9);
    }

    #[test]
    fn a_car_already_beside_it_holds_the_shift_from_the_start() {
        let (line, along) = straight(60.0);
        let fits = boxed(0.5, 1.5, -0.5, 0.5);
        let pass = plan(&line, &along, 3.6, &PassSettings::default(), &fits, &|_| {
            4.0
        })
        .expect("room")
        .expect("a pass");
        assert!((pass.line[0].y - pass.shift()).abs() < 1e-9);
    }
}
