//! Control barrier functions for the plan: a keep-out circle the car may come near, and
//! never enter (22 Nov; the package's README, "Barriers").
//!
//! Research, separate from the capstone: Ben's own work, not a team deliverable, and
//! nothing in the stack turns it on.
//!
//! The idea is A. M. Ali, C. Shen and H. A. Hashim's ("A linear MPC with control barrier
//! functions for differential drive robots", IET Control Theory & Applications, 2024):
//! instead of "stay R away", which does nothing until the obstacle is inside the horizon,
//! ask that the barrier **H shrinks by at most a fraction a tick**:
//!
//! ```text
//!   H(next) >= (1 - gamma) H(now)        H > 0: safe, H = 0: the edge, H < 0: inside
//! ```
//!
//! So H can get close to 0 and never crosses it: by induction, H(0) >= 0 gives H(k) >= 0
//! for every k ([`decays_no_faster`], tested below).
//!
//! Ours differs from the paper's in the model: theirs is a unicycle made linear by
//! dynamic feedback linearization; ours is the lag-aware bicycle in the path's frame,
//! linearised each tick about the last plan. So here H is linearised too ([`Circle::gradient`]),
//! and the rule holds for the plan's model, not yet for the car: the paper's proofs don't
//! carry over as they are. No ROS here.
//!
//! | In the path's frame | |
//! |---|---|
//! | the circle | its centre at (`s`, `e_y`), `radius` m: the object, the car's own half-width and a margin |
//! | the car | one point, `ahead` m in front of base_link along its heading (its body's middle) |
//! | H | (distance from that point to the centre)² - radius² |

use crate::path::Path;

/// A keep-out circle in the path's frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Circle {
    /// m along the path to its centre.
    pub s: f64,
    /// m left of the path to its centre.
    pub e_y: f64,
    /// m: the object's radius, the car's own (its body's point to its corner) and a margin.
    pub radius: f64,
}

/// How the barriers are held.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rules {
    /// The share of H a tick may use up, 0..1: small = it turns away early and gently;
    /// 1 = only "never inside", as late as a distance limit.
    pub gamma: f64,
    /// m from base_link to the body's point the circle is measured from.
    pub ahead: f64,
    /// The cost on a tick's slack (the rule broken in the plan, m²): high, so a slack is
    /// used only when no steering or braking can hold the rule.
    pub weight: f64,
    /// m/s²: how hard the car slows once a stop is asked (the spine's hold brake): the
    /// stop rule's own number ([`stop_distance`]). Measure it on the car; lower is safer.
    pub stop_decel: f64,
}

impl Default for Rules {
    fn default() -> Self {
        Rules {
            gamma: 0.05,
            ahead: 0.8,
            weight: 1.0e4,
            stop_decel: 0.8,
        }
    }
}

impl Rules {
    /// Whether the numbers can be used: gamma in (0, 1], ahead finite, a weight and a
    /// stop above 0.
    #[must_use]
    pub fn valid(&self) -> bool {
        self.gamma > 0.0
            && self.gamma <= 1.0
            && self.ahead.is_finite()
            && self.weight > 0.0
            && self.stop_decel > 0.0
    }
}

impl Circle {
    /// A circle at a point of the world, put in the path's frame: `(x, y)` its centre.
    ///
    /// The path's frame bends with the path, so this is right where the path is straight
    /// beside the circle, and close where `radius` is small against the bend's radius.
    #[must_use]
    pub fn from_world(path: &Path, x: f64, y: f64, radius: f64) -> Self {
        let foot = path.project((x, y), 0, f64::INFINITY);
        Circle {
            s: foot.s,
            e_y: foot.cross,
            radius,
        }
    }

    /// The body's point, from the circle's centre: (along the path, to its left), m.
    fn offset(&self, e_y: f64, e_th: f64, s: f64, ahead: f64) -> (f64, f64) {
        (
            s + ahead * e_th.cos() - self.s,
            e_y + ahead * e_th.sin() - self.e_y,
        )
    }

    /// H, m²: above 0 outside the circle, 0 on its edge, below 0 inside.
    #[must_use]
    pub fn h(&self, e_y: f64, e_th: f64, s: f64, ahead: f64) -> f64 {
        let (along, left) = self.offset(e_y, e_th, s, ahead);
        along * along + left * left - self.radius * self.radius
    }

    /// How H changes with (e_y, e_theta, s): the plan's linear H is `h + gradient . change`.
    #[must_use]
    pub fn gradient(&self, e_y: f64, e_th: f64, s: f64, ahead: f64) -> [f64; 3] {
        let (along, left) = self.offset(e_y, e_th, s, ahead);
        [
            2.0 * left,
            2.0 * ahead * (left * e_th.cos() - along * e_th.sin()),
            2.0 * along,
        ]
    }

    /// m straight on, along the car's heading, from the body's point to the circle's edge:
    /// `None` if that line misses the circle or leads away from it; 0 inside it.
    #[must_use]
    pub fn reach(&self, e_y: f64, e_th: f64, s: f64, ahead: f64) -> Option<f64> {
        let (along, left) = self.offset(e_y, e_th, s, ahead);
        let h = along * along + left * left - self.radius * self.radius;
        if h <= 0.0 {
            return Some(0.0);
        }
        // |offset + t u|² = radius², u the heading: t² + 2 b t + h = 0.
        let b = along * e_th.cos() + left * e_th.sin();
        let disc = b * b - h;
        (b < 0.0 && disc >= 0.0).then(|| -b - disc.sqrt())
    }

    /// m from the body's point to the circle's edge: below 0 inside.
    #[must_use]
    pub fn clearance(&self, e_y: f64, e_th: f64, s: f64, ahead: f64) -> f64 {
        let (along, left) = self.offset(e_y, e_th, s, ahead);
        along.hypot(left) - self.radius
    }
}

/// m the car needs to stand still from `v` m/s, once the tracker asks a stop: it keeps
/// its speed for `wait` s (the dead time, and the tick the answer takes), then while the
/// set speed ramps down at `ramp` m/s² (ego's), and slows at `decel` m/s² at the least.
/// Each part is the longest it can be, so the sum is on the safe side.
#[must_use]
pub fn stop_distance(v: f64, wait: f64, ramp: f64, decel: f64) -> f64 {
    let v = v.max(0.0);
    v * (wait + v / ramp) + v * v / (2.0 * decel)
}

/// The barrier's rule for one tick: `next >= (1 - gamma) now`, within `tolerance`.
#[must_use]
pub fn decays_no_faster(now: f64, next: f64, gamma: f64, tolerance: f64) -> bool {
    next >= (1.0 - gamma) * now - tolerance
}

#[cfg(test)]
mod tests {
    use super::*;

    const C: Circle = Circle {
        s: 10.0,
        e_y: 0.5,
        radius: 2.0,
    };

    #[test]
    fn h_is_zero_on_the_edge_and_its_sign_says_which_side() {
        // The body's point 2 m short of the centre, on its line.
        assert!(C.h(0.5, 0.0, 7.2, 0.8).abs() < 1e-12);
        assert!(C.h(0.5, 0.0, 6.0, 0.8) > 0.0);
        assert!(C.h(0.5, 0.0, 8.0, 0.8) < 0.0);
        assert!((C.clearance(0.5, 0.0, 6.2, 0.8) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn the_gradient_is_hs_slope() {
        let (e_y, e_th, s, ahead) = (-0.3, 0.2, 8.1, 0.8);
        let g = C.gradient(e_y, e_th, s, ahead);
        let eps = 1e-6;
        let h0 = C.h(e_y, e_th, s, ahead);
        let slopes = [
            (C.h(e_y + eps, e_th, s, ahead) - h0) / eps,
            (C.h(e_y, e_th + eps, s, ahead) - h0) / eps,
            (C.h(e_y, e_th, s + eps, ahead) - h0) / eps,
        ];
        for (a, b) in g.iter().zip(slopes) {
            assert!((a - b).abs() < 1e-4, "{a} against {b}");
        }
    }

    #[test]
    fn the_rule_kept_every_tick_never_reaches_the_inside() {
        // The proof's induction, run: the worst the rule allows, 1000 ticks on.
        for gamma in [0.01, 0.05, 0.5, 1.0] {
            let mut h = 3.0_f64;
            for _ in 0..1000 {
                let next = (1.0 - gamma) * h;
                assert!(decays_no_faster(h, next, gamma, 0.0));
                assert!(next >= 0.0);
                h = next;
            }
        }
        // And one tick that breaks it is seen.
        assert!(!decays_no_faster(1.0, 0.9, 0.05, 1e-9));
    }

    #[test]
    fn from_inside_the_rule_asks_the_way_out() {
        // H < 0: (1 - gamma) H is nearer 0 than H, so the next H must be higher.
        let (now, gamma) = (-1.0, 0.1);
        assert!(!decays_no_faster(now, now, gamma, 1e-12));
        assert!(decays_no_faster(now, -0.9, gamma, 1e-12));
    }

    #[test]
    fn the_reach_is_the_way_straight_on_to_the_edge() {
        // Heading at the centre from 5 m short of it: 3 m to the edge.
        let reach = C.reach(0.5, 0.0, 4.2, 0.8).expect("it is ahead");
        assert!((reach - 3.0).abs() < 1e-12);
        // Past it, beside it by more than its radius, or turned away: nothing ahead.
        assert!(C.reach(0.5, 0.0, 13.0, 0.8).is_none());
        assert!(C.reach(3.0, 0.0, 4.2, 0.8).is_none());
        assert!(C.reach(0.5, 1.5, 4.2, 0.8).is_none());
        assert_eq!(C.reach(0.5, 0.0, 9.0, 0.8), Some(0.0));
    }

    #[test]
    fn the_stop_distance_grows_with_speed_and_is_nothing_at_rest() {
        assert_eq!(stop_distance(0.0, 0.15, 2.0, 0.8), 0.0);
        // 1 m/s: 0.15 s waiting, 0.5 s of ramp, then 1 / 1.6 m of braking.
        assert!((stop_distance(1.0, 0.15, 2.0, 0.8) - (0.65 + 0.625)).abs() < 1e-12);
        assert!(stop_distance(2.0, 0.15, 2.0, 0.8) > 2.0 * stop_distance(1.0, 0.15, 2.0, 0.8));
    }

    #[test]
    fn a_circle_of_the_world_lands_in_the_paths_frame() {
        let line: Vec<_> = (0..200).map(|i| (f64::from(i) * 0.1, 0.0, 0.0)).collect();
        let path = Path::new(&line).expect("a path");
        let c = Circle::from_world(&path, 12.0, -0.7, 1.5);
        assert!((c.s - 12.0).abs() < 1e-6 && (c.e_y + 0.7).abs() < 1e-6);
    }
}
