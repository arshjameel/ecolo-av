//! The squiggly line (Ben, 7 Nov): where the car WILL go, what the planner WANTS, and where
//! it stops if it brakes now. Shared by the in-car HUD and the developer window.
//!
//! ```text
//!   faint line  /ego/plan_ahead   what the planner wants      (aav_ego look_ahead)
//!   bold line   /ego/trace        what the car will do, 2 s   (ego, the car model)
//!   stop bar    /ego/stop_point   where a full brake stops it (ego, the car model)
//!   numbers     /ego/ahead        the gap, the room, the goal (aav_ego look_ahead)
//! ```
//!
//! Everything is in base_link: x forward, y left, metres, the car at (0, 0). The numbers
//! are worked out once, in ROS (look_ahead.py); screens only draw them. No ROS, no raylib.

use std::time::Duration;

/// A point in base_link, metres.
pub type Point = (f64, f64);

/// How well the trace keeps to the plan: `/ego/ahead`'s level.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Grade {
    /// Within 0.3 m, or no plan to keep to: green.
    #[default]
    Ok,
    /// Within 0.6 m: amber.
    Warn,
    /// Further: red.
    Bad,
}

impl Grade {
    /// From diagnostic_msgs/DiagnosticStatus's level: 0 OK, 1 WARN, 2 ERROR (3 STALE).
    #[must_use]
    pub fn from_level(level: u8) -> Self {
        match level {
            0 => Self::Ok,
            1 => Self::Warn,
            _ => Self::Bad,
        }
    }
}

/// Which plan the faint line is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PlanSource {
    /// The path follower's own future (`/drivers/nav/local_plan`): MPPI's.
    Local,
    /// Nav2's whole plan (`/plan`), cut ahead of the car.
    Global,
    /// No plan: a person drives.
    #[default]
    None,
}

impl PlanSource {
    fn parse(text: &str) -> Self {
        match text {
            "local" => Self::Local,
            "global" => Self::Global,
            _ => Self::None,
        }
    }

    /// A short word for the screen.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::Local => "MPPI",
            Self::Global => "PLAN",
            Self::None => "NO PLAN",
        }
    }
}

/// `/ego/ahead`'s numbers. `None`: unknown.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Summary {
    pub grade: Grade,
    /// One line for people.
    pub message: String,
    pub plan: PlanSource,
    /// m: the most the trace strays from the plan.
    pub gap: Option<f64>,
    /// m: how far a full brake now takes the car.
    pub stop: Option<f64>,
    /// m from the stop point to the plan's end: + short of the goal, - past it.
    pub goal: Option<f64>,
    /// m base_link goes before the front bumper meets an obstacle.
    pub obstacle: Option<f64>,
    /// m between the stop and that obstacle: below 0, a full brake now still hits it.
    pub room: Option<f64>,
    /// The body from base_link (vehicle.yaml, through look_ahead): the front bumper ahead
    /// (m), the rear one (m, below 0), half the width (m).
    pub body: Option<Body>,
    /// The road ahead (`/world/road_ahead`) where the car is: `None` without one.
    pub road: Option<Road>,
}

/// What the road ahead allows where the car is, as look_ahead sends it.
#[derive(Debug, Clone, PartialEq)]
pub struct Road {
    /// m/s: the speed the road ahead allows here (0: none).
    pub limit: f64,
    /// m to its end (the nearest stop, or as far as it reaches).
    pub room: f64,
    /// Why it ends there: `sign`, `obstacle`, `fence`, `goal`; empty: just its length.
    pub why: String,
}

/// The car's outline from base_link, as look_ahead sends it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Body {
    pub front: f64,
    pub rear: f64,
    pub half_width: f64,
}

impl Summary {
    /// From a DiagnosticStatus's level, message and key-value pairs ('nan': unknown).
    pub fn from_status<'a>(
        level: u8,
        message: &str,
        values: impl IntoIterator<Item = (&'a str, &'a str)>,
    ) -> Self {
        let mut s = Self {
            grade: Grade::from_level(level),
            message: message.to_owned(),
            ..Self::default()
        };
        let (mut front, mut rear, mut half_width) = (None, None, None);
        let (mut road_limit, mut road_room, mut why) = (None, None, String::new());
        for (key, value) in values {
            let number = value.parse::<f64>().ok().filter(|v| v.is_finite());
            match key {
                "plan" => s.plan = PlanSource::parse(value),
                "gap_m" => s.gap = number,
                "stop_m" => s.stop = number,
                "goal_m" => s.goal = number,
                "obstacle_m" => s.obstacle = number,
                "room_m" => s.room = number,
                "front_m" => front = number,
                "rear_m" => rear = number,
                "half_width_m" => half_width = number,
                "road_mps" => road_limit = number,
                "road_m" => road_room = number,
                "road_why" => why = value.to_owned(),
                _ => {}
            }
        }
        if let (Some(front), Some(rear), Some(half_width)) = (front, rear, half_width) {
            s.body = Some(Body {
                front,
                rear,
                half_width,
            });
        }
        if let (Some(limit), Some(room)) = (road_limit, road_room) {
            s.road = Some(Road { limit, room, why });
        }
        s
    }
}

/// Where a full brake stops the car, in base_link.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct StopMark {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

/// Everything a screen draws for the squiggly line.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Ahead {
    /// The bold line: ego's trace.
    pub trace: Vec<Point>,
    /// The faint line: the plan ahead.
    pub plan: Vec<Point>,
    pub stop: Option<StopMark>,
    pub summary: Option<Summary>,
}

impl Ahead {
    /// The line the car will drive past the trace's end: the trace, then straight on
    /// from its last heading, `total` m long in all. Where the obstacle and the stop sit.
    #[must_use]
    pub fn point_along(&self, s: f64) -> Option<Point> {
        point_along(&self.trace, s)
    }

    /// Where the front bumper stops: the stop point carried `front` m on along its yaw.
    #[must_use]
    pub fn bumper_stop(&self) -> Option<StopMark> {
        let stop = self.stop?;
        let front = self.summary.as_ref()?.body?.front;
        let (s, c) = stop.yaw.sin_cos();
        Some(StopMark {
            x: stop.x + front * c,
            y: stop.y + front * s,
            yaw: stop.yaw,
        })
    }
}

/// The point `s` m along a polyline, carried on straight past its end. None: fewer than
/// two points, or a polyline with no length.
#[must_use]
pub fn point_along(points: &[Point], s: f64) -> Option<Point> {
    let mut walked = 0.0;
    let mut last = None;
    for pair in points.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let seg = (b.0 - a.0).hypot(b.1 - a.1);
        if seg <= 0.0 {
            continue;
        }
        if walked + seg >= s {
            let u = (s - walked).max(0.0) / seg;
            return Some((a.0 + u * (b.0 - a.0), a.1 + u * (b.1 - a.1)));
        }
        walked += seg;
        last = Some((a, b, seg));
    }
    let (a, b, seg) = last?;
    let more = s - walked;
    Some((
        b.0 + more * (b.0 - a.0) / seg,
        b.1 + more * (b.1 - a.1) / seg,
    ))
}

/// What the words under the drawing say, and how each is coloured.
#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    pub text: String,
    pub tone: Tone,
}

/// A line's colour, by meaning: the screen picks the theme's colour.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tone {
    Plain,
    Good,
    Warn,
    Bad,
    Dim,
}

impl From<Grade> for Tone {
    fn from(g: Grade) -> Self {
        match g {
            Grade::Ok => Self::Good,
            Grade::Warn => Self::Warn,
            Grade::Bad => Self::Bad,
        }
    }
}

fn metres(m: f64) -> String {
    crate::format::metres(m.abs())
}

/// The words for a driver, short: how far the stop is, whether the car keeps to the plan,
/// and the room left before the goal or an obstacle.
pub fn lines(summary: Option<&Summary>) -> [Line; 3] {
    let Some(s) = summary else {
        let dim = |t: &str| Line {
            text: t.into(),
            tone: Tone::Dim,
        };
        return [
            dim("NO LOOK-AHEAD"),
            dim("ego or look_ahead silent"),
            dim(""),
        ];
    };
    let stop = Line {
        text: s
            .stop
            .map_or("STOP --".into(), |m| format!("STOPS IN {}", metres(m))),
        tone: Tone::Plain,
    };
    let plan = match (s.plan, s.gap) {
        (PlanSource::None, _) | (_, None) => Line {
            text: "NO PLAN".into(),
            tone: Tone::Dim,
        },
        (_, Some(gap)) => Line {
            text: match s.grade {
                Grade::Ok => format!("ON PLAN {}", metres(gap)),
                Grade::Warn => format!("OFF PLAN {}", metres(gap)),
                Grade::Bad => format!("FAR OFF {}", metres(gap)),
            },
            tone: s.grade.into(),
        },
    };
    // An obstacle within a metre of the stop point comes first, then the road ahead.
    let near = s.room.is_some_and(|r| r < 1.0);
    let road = s.road.as_ref().filter(|_| !near);
    let room = road
        .map(road_line)
        .unwrap_or_else(|| match (s.room, s.goal) {
            (Some(r), _) if r < 0.0 => Line {
                text: format!("HITS: {} SHORT", metres(r)),
                tone: Tone::Bad,
            },
            (Some(r), _) if r < 1.0 => Line {
                text: format!("OBSTACLE +{}", metres(r)),
                tone: Tone::Warn,
            },
            (_, Some(g)) if g < 0.0 => Line {
                text: format!("GOAL: {} PAST", metres(g)),
                tone: Tone::Warn,
            },
            (_, Some(g)) => Line {
                text: format!("GOAL: {} SHORT", metres(g)),
                tone: Tone::Plain,
            },
            (Some(r), None) => Line {
                text: format!("OBSTACLE +{}", metres(r)),
                tone: Tone::Plain,
            },
            (None, None) => Line {
                text: "NOTHING AHEAD".into(),
                tone: Tone::Dim,
            },
        });
    [stop, plan, room]
}

/// The road ahead in words: "ROAD 0.6 m/s, SIGN IN 12 m" (`stop at` its end: 0 m/s).
#[must_use]
pub fn road_line(road: &Road) -> Line {
    let end = if road.why.is_empty() {
        "END".to_owned()
    } else {
        road.why.to_uppercase()
    };
    if road.limit <= 0.0 {
        return Line {
            text: format!("ROAD: STOP ({end})"),
            tone: Tone::Warn,
        };
    }
    Line {
        text: format!(
            "ROAD {:.1} m/s, {end} IN {}",
            road.limit,
            metres(road.room.max(0.0))
        ),
        tone: Tone::Plain,
    }
}

/// base_link to a panel: forward is up, the car's rear axle near the bottom, centred.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct View {
    /// The panel: (x, y, width, height) in the screen's units.
    pub rect: (f32, f32, f32, f32),
    /// Metres from the rear axle to the panel's top.
    pub ahead: f64,
    /// Metres below the rear axle still in the panel (the car's body).
    pub behind: f64,
}

impl View {
    /// Screen units per metre.
    #[must_use]
    pub fn scale(&self) -> f32 {
        self.rect.3 / (self.ahead + self.behind) as f32
    }

    /// A base_link point on the panel.
    #[must_use]
    pub fn to_panel(&self, (x, y): Point) -> (f32, f32) {
        let k = self.scale();
        let (rx, ry, rw, rh) = self.rect;
        let axle = (rx + rw / 2.0, ry + rh - self.behind as f32 * k);
        (axle.0 - y as f32 * k, axle.1 - x as f32 * k)
    }

    /// Whether a base_link point falls inside the panel.
    #[must_use]
    pub fn shows(&self, p: Point) -> bool {
        let (px, py) = self.to_panel(p);
        let (rx, ry, rw, rh) = self.rect;
        px >= rx && px <= rx + rw && py >= ry && py <= ry + rh
    }

    /// A polyline clipped to the points the panel shows: a line that leaves the panel
    /// stops at the last point inside it (good enough for a few metres of line).
    #[must_use]
    pub fn polyline(&self, points: &[Point]) -> Vec<(f32, f32)> {
        points
            .iter()
            .take_while(|p| self.shows(**p))
            .map(|p| self.to_panel(*p))
            .collect()
    }

    /// The stop bar's two ends: across the car's path at the stop point, `half_width` m
    /// each side.
    #[must_use]
    pub fn stop_bar(&self, stop: StopMark, half_width: f64) -> ((f32, f32), (f32, f32)) {
        let (s, c) = stop.yaw.sin_cos();
        let left = (stop.x - half_width * s, stop.y + half_width * c);
        let right = (stop.x + half_width * s, stop.y - half_width * c);
        (self.to_panel(left), self.to_panel(right))
    }
}

/// Pretend data for `--demo` and the dev window's pretend feed: a 12 s loop that speeds
/// up, bends left round something, then brakes for its goal, so the trace grows, curves
/// and shrinks, and the gap goes green, amber, green.
pub fn pretend(t: Duration) -> Ahead {
    const LOOP: f64 = 12.0;
    let t = t.as_secs_f64() % LOOP;
    // Speed: up to 1.8 m/s by 4 s, held to 8 s, braking to 0 at 11 s.
    let v = if t < 4.0 {
        0.45 * t
    } else if t < 8.0 {
        1.8
    } else {
        (1.8 * (11.0 - t) / 3.0).max(0.0)
    };
    // Curvature: a left bend from 3 s to 8 s, a touch late against the plan from 5 s.
    let bend = if (3.0..8.0).contains(&t) {
        0.18 * ((t - 3.0) * std::f64::consts::PI / 5.0).sin()
    } else {
        0.0
    };
    let late = if (5.0..7.0).contains(&t) { 0.6 } else { 1.0 };
    let arc = |k: f64, s: f64| -> Point {
        if k.abs() < 1e-9 {
            (s, 0.0)
        } else {
            ((k * s).sin() / k, (1.0 - (k * s).cos()) / k)
        }
    };
    // The trace: 21 poses over 2 s, slowing as a braking car does.
    let decel = if t >= 8.0 { 0.6 } else { 0.0 };
    let trace: Vec<Point> = (0..=20)
        .map(|i| {
            let dt = f64::from(i) * 0.1;
            let s = (v * dt - 0.5 * decel * dt * dt).max(0.0);
            let s = if decel > 0.0 {
                s.min(v * v / (2.0 * decel))
            } else {
                s
            };
            arc(bend * late, s)
        })
        .collect();
    let plan: Vec<Point> = (0..=40).map(|i| arc(bend, f64::from(i) * 0.25)).collect();
    // A full brake: about 0.2 s of the brake's lag, then 3 m/s^2.
    let stop_s = v * 0.2 + v * v / 6.0;
    let (sx, sy) = arc(bend * late, stop_s);
    let stop = StopMark {
        x: sx,
        y: sy,
        yaw: bend * late * stop_s,
    };
    let gap = trace
        .iter()
        .map(|p| {
            plan.iter()
                .map(|q| (p.0 - q.0).hypot(p.1 - q.1))
                .fold(f64::INFINITY, f64::min)
        })
        .fold(0.0, f64::max);
    let grade = if gap > 0.6 {
        Grade::Bad
    } else if gap > 0.3 {
        Grade::Warn
    } else {
        Grade::Ok
    };
    let goal = (11.0 - t) * 0.9 + 0.6 - stop_s;
    let obstacle = (t < 8.0).then_some(7.5 - 0.4 * t);
    let summary = Summary {
        grade,
        message: String::new(),
        plan: PlanSource::Local,
        gap: Some(gap),
        stop: Some(stop_s),
        goal: Some(goal),
        obstacle,
        room: obstacle.map(|o| o - stop_s),
        body: Some(Body {
            front: 1.7,
            rear: -0.3,
            half_width: 0.5,
        }),
        // The road ahead ends at the goal: its limit falls as the room does.
        road: Some(Road {
            limit: (0.5 * (goal + stop_s).max(0.0)).sqrt().min(1.0),
            room: goal + stop_s,
            why: "goal".into(),
        }),
    };
    Ahead {
        trace,
        plan,
        stop: Some(stop),
        summary: Some(summary),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn summary_reads_the_status() {
        let s = Summary::from_status(
            1,
            "OFF PLAN 0.40 m - stops in 1.2 m",
            [
                ("gap_m", "0.40"),
                ("plan", "local"),
                ("stop_m", "1.20"),
                ("goal_m", "-0.5"),
                ("obstacle_m", "nan"),
                ("room_m", "nan"),
                ("other", "x"),
            ],
        );
        assert_eq!(s.grade, Grade::Warn);
        assert_eq!(s.plan, PlanSource::Local);
        assert_eq!(s.gap, Some(0.4));
        assert_eq!(s.stop, Some(1.2));
        assert_eq!(s.goal, Some(-0.5));
        assert_eq!(s.obstacle, None);
        assert_eq!(s.room, None);
    }

    #[test]
    fn body_and_the_bumpers_stop() {
        let s = Summary::from_status(
            0,
            "",
            [
                ("front_m", "1.70"),
                ("rear_m", "-0.30"),
                ("half_width_m", "0.50"),
            ],
        );
        let body = s.body.unwrap();
        assert_eq!((body.front, body.rear, body.half_width), (1.7, -0.3, 0.5));
        let a = Ahead {
            stop: Some(StopMark {
                x: 1.0,
                y: 0.0,
                yaw: std::f64::consts::FRAC_PI_2,
            }),
            summary: Some(s),
            ..Ahead::default()
        };
        let b = a.bumper_stop().unwrap();
        assert!((b.x - 1.0).abs() < 1e-9 && (b.y - 1.7).abs() < 1e-9);
        assert_eq!(Summary::from_status(0, "", [("front_m", "1")]).body, None);
    }

    #[test]
    fn levels() {
        assert_eq!(Grade::from_level(0), Grade::Ok);
        assert_eq!(Grade::from_level(2), Grade::Bad);
        assert_eq!(Grade::from_level(3), Grade::Bad);
    }

    #[test]
    fn lines_say_the_stop_the_plan_and_the_room() {
        let s = Summary {
            grade: Grade::Ok,
            plan: PlanSource::Global,
            gap: Some(0.12),
            stop: Some(1.4),
            goal: Some(0.8),
            ..Summary::default()
        };
        let [stop, plan, room] = lines(Some(&s));
        assert_eq!(stop.text, "STOPS IN 1.4 m");
        assert_eq!(
            (plan.text.as_str(), plan.tone),
            ("ON PLAN 0.12 m", Tone::Good)
        );
        assert_eq!(room.text, "GOAL: 0.80 m SHORT");
        let hit = Summary {
            room: Some(-0.3),
            ..s.clone()
        };
        assert_eq!(lines(Some(&hit))[2].tone, Tone::Bad);
        let past = Summary {
            goal: Some(-0.5),
            ..s
        };
        assert_eq!(lines(Some(&past))[2].text, "GOAL: 0.50 m PAST");
        assert_eq!(lines(None)[0].tone, Tone::Dim);
    }

    #[test]
    fn the_road_ahead_takes_the_third_line_unless_an_obstacle_is_near() {
        let values = [
            ("plan", "global"),
            ("gap_m", "0.10"),
            ("stop_m", "1.20"),
            ("room_m", "nan"),
            ("road_mps", "0.62"),
            ("road_m", "12.30"),
            ("road_why", "sign"),
        ];
        let s = Summary::from_status(0, "", values);
        assert_eq!(
            s.road,
            Some(Road {
                limit: 0.62,
                room: 12.3,
                why: "sign".into()
            })
        );
        assert_eq!(lines(Some(&s))[2].text, "ROAD 0.6 m/s, SIGN IN 12 m");
        let near = Summary {
            room: Some(0.4),
            ..s.clone()
        };
        assert_eq!(lines(Some(&near))[2].tone, Tone::Warn);
        assert!(lines(Some(&near))[2].text.starts_with("OBSTACLE"));
        let stop = Road {
            limit: 0.0,
            room: 0.0,
            why: String::new(),
        };
        assert_eq!(road_line(&stop).text, "ROAD: STOP (END)");
        let none = Summary::from_status(0, "", [("road_mps", "nan"), ("road_m", "nan")]);
        assert_eq!(none.road, None);
    }

    #[test]
    fn no_plan_is_dim_not_red() {
        let s = Summary {
            stop: Some(0.0),
            ..Summary::default()
        };
        assert_eq!(lines(Some(&s))[1].tone, Tone::Dim);
    }

    #[test]
    fn view_puts_forward_up_and_left_left() {
        let v = View {
            rect: (0.0, 0.0, 100.0, 200.0),
            ahead: 9.0,
            behind: 1.0,
        };
        assert_eq!(v.scale(), 20.0);
        assert_eq!(v.to_panel((0.0, 0.0)), (50.0, 180.0));
        assert_eq!(v.to_panel((9.0, 0.0)), (50.0, 0.0));
        let (lx, _) = v.to_panel((0.0, 1.0));
        assert!(lx < 50.0);
        assert!(v.shows((5.0, 0.0)) && !v.shows((12.0, 0.0)));
        assert_eq!(
            v.polyline(&[(0.0, 0.0), (5.0, 0.0), (20.0, 0.0), (1.0, 0.0)])
                .len(),
            2
        );
    }

    #[test]
    fn stop_bar_is_across_the_way() {
        let v = View {
            rect: (0.0, 0.0, 100.0, 200.0),
            ahead: 9.0,
            behind: 1.0,
        };
        let (a, b) = v.stop_bar(
            StopMark {
                x: 2.0,
                y: 0.0,
                yaw: 0.0,
            },
            0.5,
        );
        assert_eq!(a.1, b.1);
        assert!((a.0 - b.0).abs() > 19.0);
    }

    #[test]
    fn point_along_runs_on_past_the_end() {
        let line = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)];
        assert_eq!(point_along(&line, 0.5), Some((0.5, 0.0)));
        assert_eq!(point_along(&line, 1.5), Some((1.0, 0.5)));
        assert_eq!(point_along(&line, 3.0), Some((1.0, 2.0)));
        assert_eq!(point_along(&[(0.0, 0.0)], 1.0), None);
    }

    #[test]
    fn pretend_grows_bends_and_shrinks() {
        let at = |s: f64| pretend(Duration::from_secs_f64(s));
        let len = |a: &Ahead| a.trace.last().map_or(0.0, |p| p.0.hypot(p.1));
        assert!(len(&at(6.0)) > len(&at(1.0)));
        assert!(len(&at(10.5)) < len(&at(6.0)));
        assert!(at(6.0).trace.last().unwrap().1 > 0.05);
        let grades: Vec<Grade> = (0..24)
            .map(|i| at(f64::from(i) * 0.5).summary.unwrap().grade)
            .collect();
        assert!(grades.contains(&Grade::Ok));
        assert!(grades.iter().any(|g| *g != Grade::Ok));
        assert_eq!(at(11.5).summary.unwrap().stop, Some(0.0));
    }
}
