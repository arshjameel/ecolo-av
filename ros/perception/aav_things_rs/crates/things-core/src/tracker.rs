//! A small tracker: one constant-velocity Kalman filter per thing.
//!
//! AB3DMOT's finding (Weng et al., 2020): a plain Kalman filter per object and a simple
//! association is a strong baseline. Each thing's state is where it is and how fast it
//! moves over the ground, in odom:
//!
//! ```text
//! state  [x, y, vx, vy]          predict   x += vx dt, y += vy dt  (speed kept)
//! lidar  measures x, y           radar     measures the speed along its ray
//! ```
//!
//! Association is greedy nearest first inside a gate: the closest (thing, group) pair is
//! matched, then the next closest, until none is closer than `gate`. A group nobody
//! claims starts a new thing; a thing nobody saw coasts on its speed.
//!
//! Each pair must also pass three checks (TODO 222: a row of thin posts became things
//! moving 1.5-11 m/s, the match hopping from post to post):
//!
//! | Check | A pair is refused when |
//! |---|---|
//! | where it should be | its Mahalanobis distance² is over `gate_chi2` (9.21: 99 %, two dimensions) |
//! | its speed, if small | a thing and group both no wider than `small_size` would have moved faster than `small_speed` since last seen, give or take `small_slack` m |
//! | its shape | the footprints differ by more than `shape_ratio` times and by more than `shape_slack` m |
//!
//! Memory: a thing seen `confirm` times is kept, coasting on its speed (parked when slower
//! than `still_speed`), for `keep` s after it was last seen; `keep_blind` s inside
//! `blind_range` m of the car, where the lidar can't see it. A thing seen fewer times is
//! dropped after `keep_tentative` s.
//!
//! **A motion model per kind** (`per_kind`, TODO 217, a challenger; Fast-Poly, Poly-MOT
//! and OC-SORT's ideas, MIT): the state stays [x, y, vx, vy], but how it may change
//! depends on what the cameras named it.
//!
//! | Kind | Its acceleration's noise |
//! |---|---|
//! | a vehicle (car, truck, bus, motorcycle) | a bicycle's: `car_along` along its heading (speed up, brake), `car_across` across it (it turns, it never slides) |
//! | a person | `person_accel`, any way: people stop and turn on the spot |
//! | unknown | `accel_noise`, as the champion |
//!
//! And a thing found again after `refind_gap` s unseen re-fits its speed from its last
//! `sightings` places (OC-SORT's observation-centric re-update), not from where it
//! coasted to.

use std::cmp::Ordering;

use ego_core::hypot;
use nalgebra::{Matrix2, Matrix4, RowVector4, SMatrix, Vector2, Vector4};

use crate::{groups::Group, looks::VEHICLES, UNKNOWN};

/// The tracker's settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackerShape {
    /// m: a group farther than this from a thing is another thing.
    pub gate: f64,
    /// m: one sigma, a group's middle as the lidar sees it.
    pub position_noise: f64,
    /// m/s²: one sigma, how hard a thing may change its speed.
    pub accel_noise: f64,
    /// m/s: one sigma, the radar's speed along its ray.
    pub radar_noise: f64,
    /// m: a radar cluster this close to a thing gives it a speed.
    pub radar_gate: f64,
    /// Times seen before a thing is kept through gaps.
    pub confirm: u32,
    /// s: a confirmed thing stays this long unseen.
    pub keep: f64,
    /// s: ... and this long while it is inside `blind_range`.
    pub keep_blind: f64,
    /// m from the car: where the lidar can't see low.
    pub blind_range: f64,
    /// s: a thing seen fewer than `confirm` times.
    pub keep_tentative: f64,
    /// m/s: how little a new thing's speed is known.
    pub start_speed_sigma: f64,
    /// m/s: an unseen thing slower than this is taken as parked.
    pub still_speed: f64,
    /// Mahalanobis distance², over: not this thing (99 %, 2 dof).
    pub gate_chi2: f64,
    /// A footprint this many times larger or smaller ...
    pub shape_ratio: f64,
    /// m: ... and this much: another thing.
    pub shape_slack: f64,
    /// m: a thing no wider is small (a post, a person) ...
    pub small_size: f64,
    /// m/s: ... and never moves faster than this.
    pub small_speed: f64,
    /// m: a small thing's middle may wander this much a look.
    pub small_slack: f64,
    /// A motion model per kind, and speeds re-fitted on re-finding (TODO 217); false: one
    /// constant-velocity model for all, the champion.
    pub per_kind: bool,
    /// m/s²: one sigma, a vehicle's acceleration along its heading.
    pub car_along: f64,
    /// m/s²: ... and across it.
    pub car_across: f64,
    /// m/s: slower, a vehicle's heading isn't known: `car_along` every way.
    pub car_heading_speed: f64,
    /// m/s²: one sigma, a person's acceleration, any way.
    pub person_accel: f64,
    /// s unseen: found again after this, its speed is re-fitted from its sightings.
    pub refind_gap: f64,
    /// Places kept a thing, for the re-fit.
    pub sightings: usize,
}

impl Default for TrackerShape {
    fn default() -> Self {
        Self {
            gate: 2.0,
            position_noise: 0.15,
            accel_noise: 1.5,
            radar_noise: 0.3,
            radar_gate: 2.0,
            confirm: 3,
            keep: 3.0,
            keep_blind: 30.0,
            blind_range: 4.0,
            keep_tentative: 0.3,
            start_speed_sigma: 3.0,
            still_speed: 0.5,
            gate_chi2: 9.21,
            shape_ratio: 2.5,
            shape_slack: 0.6,
            small_size: 0.8,
            small_speed: 3.5,
            small_slack: 0.3,
            per_kind: false,
            car_along: 2.0,
            car_across: 0.5,
            car_heading_speed: 1.0,
            person_accel: 2.0,
            refind_gap: 0.5,
            sightings: 5,
        }
    }
}

/// One thing, followed.
#[derive(Clone, Debug, PartialEq)]
pub struct Track {
    /// Kept while it is listed.
    pub id: u32,
    /// [x, y, vx, vy] in odom (m, m/s).
    pub x: Vector4<f64>,
    /// Its covariance.
    pub p: Matrix4<f64>,
    /// m along x (odom).
    pub length: f64,
    /// m along y.
    pub width: f64,
    /// m.
    pub height: f64,
    /// s: when it was last in a cloud.
    pub last_seen: f64,
    /// Times seen.
    pub seen: u32,
    /// The radar gave its speed.
    pub radar: bool,
    /// Not in the last cloud: moved on its speed.
    pub coasting: bool,
    /// Its name from the cameras, or [`UNKNOWN`].
    pub kind: String,
    /// Its last places seen: (s, x, y), the oldest first (`TrackerShape::sightings`).
    pub sightings: Vec<[f64; 3]>,
}

impl Track {
    /// m: one sigma of where it is (the mean of x's and y's variances).
    #[must_use]
    pub fn position_sigma(&self) -> f64 {
        ((self.p[(0, 0)] + self.p[(1, 1)]) / 2.0).max(0.0).sqrt()
    }

    /// 0..1: how often it was seen (up to 2 × confirm), less as it goes unseen.
    #[must_use]
    pub fn confidence(&self, now: f64, shape: &TrackerShape) -> f64 {
        let seen = (f64::from(self.seen) / (2.0 * f64::from(shape.confirm))).min(1.0);
        let unseen = (now - self.last_seen).max(0.0);
        seen * (1.0 - unseen / shape.keep_blind).max(0.0)
    }

    /// Where it will be `dt` s on, at its speed.
    fn moved(&self, dt: f64) -> [f64; 2] {
        [self.x[0] + self.x[2] * dt, self.x[1] + self.x[3] * dt]
    }
}

/// The state and covariance moved on by `dt` (s) at constant speed (white acceleration
/// noise: the discrete constant-velocity model).
#[must_use]
pub fn predict(
    x: &Vector4<f64>,
    p: &Matrix4<f64>,
    dt: f64,
    accel_noise: f64,
) -> (Vector4<f64>, Matrix4<f64>) {
    let mut f = Matrix4::identity();
    f[(0, 2)] = dt;
    f[(1, 3)] = dt;
    let a2 = accel_noise.powf(2.0);
    let (q00, q01, q11) = (
        dt.powf(4.0) / 4.0 * a2,
        dt.powf(3.0) / 2.0 * a2,
        dt.powf(2.0) * a2,
    );
    let mut q = Matrix4::zeros();
    for (a, b) in [(0, 2), (1, 3)] {
        q[(a, a)] = q00;
        q[(a, b)] = q01;
        q[(b, a)] = q01;
        q[(b, b)] = q11;
    }
    (f * x, f * p * f.transpose() + q)
}

/// The acceleration's covariance [xx, xy, yy] (m²/s⁴) for a thing of `kind` moving at
/// (vx, vy): a vehicle's long along its heading and narrow across it, a person's round
/// and wide, an unknown's the champion's (TODO 217).
#[must_use]
pub fn accel_covariance(kind: &str, vx: f64, vy: f64, shape: &TrackerShape) -> [f64; 3] {
    if VEHICLES.contains(&kind) {
        let along = shape.car_along * shape.car_along;
        let speed = hypot(vx, vy);
        if speed < shape.car_heading_speed {
            return [along, 0.0, along];
        }
        let across = shape.car_across * shape.car_across;
        let (c, s) = (vx / speed, vy / speed);
        return [
            along * c * c + across * s * s,
            (along - across) * c * s,
            along * s * s + across * c * c,
        ];
    }
    let a = if kind == "person" {
        shape.person_accel
    } else {
        shape.accel_noise
    };
    [a * a, 0.0, a * a]
}

/// As [`predict`], with the acceleration's covariance `a` ([xx, xy, yy]) for any shape of
/// noise (TODO 217).
#[must_use]
pub fn predict_with(
    x: &Vector4<f64>,
    p: &Matrix4<f64>,
    dt: f64,
    a: [f64; 3],
) -> (Vector4<f64>, Matrix4<f64>) {
    let mut f = Matrix4::identity();
    f[(0, 2)] = dt;
    f[(1, 3)] = dt;
    let (k00, k01, k11) = (dt * dt * dt * dt / 4.0, dt * dt * dt / 2.0, dt * dt);
    let [axx, axy, ayy] = a;
    let block = Matrix2::new(axx, axy, axy, ayy);
    let mut q = Matrix4::zeros();
    q.fixed_view_mut::<2, 2>(0, 0).copy_from(&(block * k00));
    q.fixed_view_mut::<2, 2>(0, 2).copy_from(&(block * k01));
    q.fixed_view_mut::<2, 2>(2, 0).copy_from(&(block * k01));
    q.fixed_view_mut::<2, 2>(2, 2).copy_from(&(block * k11));
    (f * x, f * p * f.transpose() + q)
}

/// The speed (vx, vy) through `sightings` (s, x, y) by least squares, and its variance
/// a direction for places known to `position_noise` m; `None` under 3 or all at once.
#[must_use]
pub fn fitted_speed(sightings: &[[f64; 3]], position_noise: f64) -> Option<([f64; 2], f64)> {
    if sightings.len() < 3 {
        return None;
    }
    let n = sightings.len() as f64;
    let (mt, mx, my) = sightings.iter().fold((0.0, 0.0, 0.0), |(t, x, y), s| {
        (t + s[0] / n, x + s[1] / n, y + s[2] / n)
    });
    let (mut stt, mut stx, mut sty) = (0.0, 0.0, 0.0);
    for s in sightings {
        let dt = s[0] - mt;
        stt += dt * dt;
        stx += dt * (s[1] - mx);
        sty += dt * (s[2] - my);
    }
    if stt <= 1e-9 {
        return None;
    }
    Some((
        [stx / stt, sty / stt],
        position_noise * position_noise / stt,
    ))
}

/// The state and covariance after a linear measurement `z = h x` (noise `r`); `None` if
/// the innovation's covariance can't be inverted. Joseph's form: stays symmetric and
/// positive.
fn update<const M: usize>(
    x: &Vector4<f64>,
    p: &Matrix4<f64>,
    z: &SMatrix<f64, M, 1>,
    h: &SMatrix<f64, M, 4>,
    r: &SMatrix<f64, M, M>,
) -> Option<(Vector4<f64>, Matrix4<f64>)> {
    let s = h * p * h.transpose() + r;
    let k = p * h.transpose() * s.try_inverse()?;
    let x = x + k * (z - h * x);
    let i_kh = Matrix4::identity() - k * h;
    Some((x, i_kh * p * i_kh.transpose() + k * r * k.transpose()))
}

/// One radar cluster, in odom: where, and its speed over the ground along the ray.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RadarCluster {
    /// m.
    pub x: f64,
    /// m.
    pub y: f64,
    /// The unit vector from the radar to the cluster, in odom.
    pub ray_x: f64,
    /// See `ray_x`.
    pub ray_y: f64,
    /// m/s over the ground along the ray (+ away from the radar).
    pub speed: f64,
}

/// A radar speed over the ground along its ray, from the speed it measured.
///
/// The radar measures ray · (v_thing − v_car) (negative while closing); adding ray · v_car
/// leaves ray · v_thing. The car's turning is left out, as the radar stand-in leaves it.
#[must_use]
pub fn ground_radial_speed(measured: f64, ray: [f64; 2], car_velocity: [f64; 2]) -> f64 {
    measured + ray[0] * car_velocity[0] + ray[1] * car_velocity[1]
}

/// Every thing near the car, in odom.
#[derive(Clone, Debug)]
pub struct Tracker {
    /// Its settings.
    pub shape: TrackerShape,
    /// The things, the oldest first.
    pub tracks: Vec<Track>,
    /// s: the last cloud's stamp, `None` before the first.
    pub time: Option<f64>,
    next_id: u32,
}

impl Tracker {
    /// No things yet.
    #[must_use]
    pub fn new(shape: TrackerShape) -> Self {
        Self {
            shape,
            tracks: Vec::new(),
            time: None,
            next_id: 1,
        }
    }

    /// Take one cloud's groups (in odom) seen at `now` (s), the car at `car` (x, y, odom).
    ///
    /// A cloud older than the last is dropped: a Kalman filter can't go back in time.
    pub fn step(&mut self, now: f64, groups: &[Group], car: [f64; 2]) {
        if self.time.is_some_and(|t| now < t) {
            return;
        }
        self.advance(now);
        let sh = self.shape;
        let r = Matrix2::identity() * sh.position_noise.powf(2.0);
        let h = position_h();
        let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
        for (ti, t) in self.tracks.iter().enumerate() {
            for (gi, g) in groups.iter().enumerate() {
                if self.fits(t, g, &h, &r, now) {
                    pairs.push((hypot(t.x[0] - g.x, t.x[1] - g.y), ti, gi));
                }
            }
        }
        pairs.sort_by(|a, b| {
            a.0.partial_cmp(&b.0)
                .unwrap_or(Ordering::Equal)
                .then(a.1.cmp(&b.1))
                .then(a.2.cmp(&b.2))
        });
        let mut used_t = vec![false; self.tracks.len()];
        let mut used_g = vec![false; groups.len()];
        for (distance, ti, gi) in pairs {
            if distance > sh.gate {
                break;
            }
            if used_t[ti] || used_g[gi] {
                continue;
            }
            used_t[ti] = true;
            used_g[gi] = true;
            let (t, g) = (&mut self.tracks[ti], &groups[gi]);
            if let Some((x, p)) = update(&t.x, &t.p, &Vector2::new(g.x, g.y), &h, &r) {
                (t.x, t.p) = (x, p);
            }
            let gap = now - t.last_seen;
            t.sightings.push([now, g.x, g.y]);
            if t.sightings.len() > sh.sightings {
                t.sightings.remove(0);
            }
            // Found again: its speed from where it was seen, not where it coasted to.
            if sh.per_kind && gap >= sh.refind_gap {
                if let Some((v, var)) = fitted_speed(&t.sightings, sh.position_noise) {
                    t.x[2] = v[0];
                    t.x[3] = v[1];
                    let mut p = Matrix4::zeros();
                    p.fixed_view_mut::<2, 2>(0, 0)
                        .copy_from(&t.p.fixed_view::<2, 2>(0, 0));
                    p[(2, 2)] = var;
                    p[(3, 3)] = var;
                    t.p = p;
                }
            }
            // The outline: a little of each new look, so a half-seen side doesn't jump it.
            t.length += 0.3 * (g.length - t.length);
            t.width += 0.3 * (g.width - t.width);
            t.height = g.height.max(0.7 * t.height + 0.3 * g.height);
            t.last_seen = now;
            t.seen += 1;
            t.coasting = false;
        }
        for (t, _) in self.tracks.iter_mut().zip(&used_t).filter(|(_, &u)| !u) {
            t.coasting = true;
            // The lidar's view of a parked thing shifts as the car moves, which reads as a
            // little speed; coasting on it would walk the thing away.
            if hypot(t.x[2], t.x[3]) < sh.still_speed {
                t.x[2] = 0.0;
                t.x[3] = 0.0;
                // Parked, its speed known to be about 0 (TODO 222): otherwise the
                // uncertainty grows while unseen until a neighbour fits its gate.
                if t.seen >= sh.confirm {
                    let mut p = Matrix4::zeros();
                    p.fixed_view_mut::<2, 2>(0, 0)
                        .copy_from(&t.p.fixed_view::<2, 2>(0, 0));
                    p[(2, 2)] = (0.5 * sh.still_speed).powf(2.0);
                    p[(3, 3)] = p[(2, 2)];
                    t.p = p;
                }
            }
        }
        for (g, _) in groups.iter().zip(&used_g).filter(|(_, &u)| !u) {
            let track = self.start(g, now);
            self.tracks.push(track);
        }
        self.expire(now, car);
    }

    /// Give each thing its name: `name(track)` (the cameras' hook, Q22).
    pub fn rename(&mut self, mut name: impl FnMut(&Track) -> String) {
        for t in &mut self.tracks {
            t.kind = name(t);
        }
    }

    /// Whether group `g` may be thing `t`: where it should be, a like shape.
    fn fits(
        &self,
        t: &Track,
        g: &Group,
        h: &SMatrix<f64, 2, 4>,
        r: &Matrix2<f64>,
        now: f64,
    ) -> bool {
        let sh = &self.shape;
        let nu = Vector2::new(g.x - t.x[0], g.y - t.x[1]);
        let s = h * t.p * h.transpose() + r;
        match s.lu().solve(&nu) {
            Some(solved) if nu.dot(&solved) <= sh.gate_chi2 => {}
            _ => return false,
        }
        let (mine, its) = (t.length.max(t.width), g.length.max(g.width));
        if mine.max(its) <= sh.small_size {
            // From where it was last seen: the prediction has already moved it on.
            let since = (now - t.last_seen).max(0.0);
            let seen_at = t.moved(-since);
            let moved = hypot(g.x - seen_at[0], g.y - seen_at[1]);
            if moved > sh.small_speed * since + sh.small_slack {
                return false;
            }
        }
        let (bigger, smaller) = (mine.max(its), mine.min(its).max(1e-3));
        bigger / smaller <= sh.shape_ratio || bigger - smaller <= sh.shape_slack
    }

    /// Give each nearest confirmed thing within `radar_gate` its radar speed along the ray.
    pub fn radar(&mut self, now: f64, clusters: &[RadarCluster]) {
        let Some(time) = self.time else {
            return;
        };
        let sh = self.shape;
        let r = SMatrix::<f64, 1, 1>::new(sh.radar_noise.powf(2.0));
        let dt = (now - time).max(0.0);
        for c in clusters {
            // The last of the nearest wins a tie, as the Python's `<=`.
            let mut best: Option<usize> = None;
            let mut best_d = sh.radar_gate;
            for (i, t) in self.tracks.iter().enumerate() {
                if t.seen < sh.confirm {
                    continue;
                }
                let at = t.moved(dt);
                let d = hypot(at[0] - c.x, at[1] - c.y);
                if d <= best_d {
                    best = Some(i);
                    best_d = d;
                }
            }
            if let Some(t) = best.map(|i| &mut self.tracks[i]) {
                let h = RowVector4::new(0.0, 0.0, c.ray_x, c.ray_y);
                let z = SMatrix::<f64, 1, 1>::new(c.speed);
                if let Some((x, p)) = update(&t.x, &t.p, &z, &h, &r) {
                    (t.x, t.p) = (x, p);
                }
                t.radar = true;
            }
        }
    }

    /// Drop the things unseen for too long (the blind zone remembers longer).
    pub fn expire(&mut self, now: f64, car: [f64; 2]) {
        let sh = self.shape;
        let dt = (now - self.time.unwrap_or(now)).max(0.0);
        self.tracks.retain(|t| {
            let unseen = now - t.last_seen;
            if t.seen < sh.confirm {
                return unseen <= sh.keep_tentative;
            }
            let at = t.moved(dt);
            if hypot(at[0] - car[0], at[1] - car[1]) <= sh.blind_range {
                unseen <= sh.keep_blind
            } else {
                unseen <= sh.keep
            }
        });
    }

    /// The confirmed things as they will be at `now`, without changing the tracker.
    ///
    /// The places move on their speed from the last cloud: the list a screen or planner
    /// reads between clouds.
    #[must_use]
    pub fn at(&self, now: f64) -> Vec<Track> {
        let dt = self.time.map_or(0.0, |t| (now - t).max(0.0));
        self.tracks
            .iter()
            .filter(|t| t.seen >= self.shape.confirm)
            .map(|t| {
                let (x, p) = if dt > 0.0 {
                    self.predict_track(t, dt)
                } else {
                    (t.x, t.p)
                };
                Track {
                    x,
                    p,
                    coasting: t.coasting || dt > 0.5,
                    ..t.clone()
                }
            })
            .collect()
    }

    fn advance(&mut self, now: f64) {
        if let Some(time) = self.time {
            let dt = now - time;
            if dt > 0.0 {
                for i in 0..self.tracks.len() {
                    let moved = self.predict_track(&self.tracks[i], dt);
                    (self.tracks[i].x, self.tracks[i].p) = moved;
                }
            }
        }
        self.time = Some(now);
    }

    /// `t` moved on `dt` s: the champion's model, or its kind's (`per_kind`).
    fn predict_track(&self, t: &Track, dt: f64) -> (Vector4<f64>, Matrix4<f64>) {
        if self.shape.per_kind {
            let a = accel_covariance(&t.kind, t.x[2], t.x[3], &self.shape);
            predict_with(&t.x, &t.p, dt, a)
        } else {
            predict(&t.x, &t.p, dt, self.shape.accel_noise)
        }
    }

    fn start(&mut self, g: &Group, now: f64) -> Track {
        let sh = &self.shape;
        let (pos, speed) = (sh.position_noise.powf(2.0), sh.start_speed_sigma.powf(2.0));
        let track = Track {
            id: self.next_id,
            x: Vector4::new(g.x, g.y, 0.0, 0.0),
            p: Matrix4::from_diagonal(&Vector4::new(pos, pos, speed, speed)),
            length: g.length,
            width: g.width,
            height: g.height,
            last_seen: now,
            seen: 1,
            radar: false,
            coasting: false,
            kind: UNKNOWN.to_string(),
            sightings: vec![[now, g.x, g.y]],
        };
        self.next_id += 1;
        track
    }
}

impl Default for Tracker {
    /// No things, the default settings.
    fn default() -> Self {
        Self::new(TrackerShape::default())
    }
}

/// The lidar's measurement: x and y of the state.
fn position_h() -> SMatrix<f64, 2, 4> {
    SMatrix::<f64, 2, 4>::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
}
