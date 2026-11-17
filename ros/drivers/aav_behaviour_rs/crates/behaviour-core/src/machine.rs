//! The behaviour states: why the car should slow, and the speed cap that slows it
//! (`behaviour.py`'s `Behaviour`).
//!
//! ```text
//!  DRIVE ─► APPROACH ─► STOPPED ─► WAIT ─► GO ─► DRIVE
//!   (no cap)  (cap falls)  (lock on,  (lock on)  (cap rises)
//!                          still)        │
//!                                        └─► ASK (a person is told)
//! ```
//!
//! The cap slows the car. **It can't stop it**: a lock or Nav2's goal check makes the final
//! stop. So a cap is 0.0 (Nav2: no limit) or at least the floor, never in between, and this
//! module never clears a lock, a latch or the e-stop.
//!
//! Something moving near (`World::moving`, from `paths::moving_near`) holds the car: WAIT,
//! reason `moving`, from any state but the e-stop's and ASK's, until `release_hold` s after
//! it has gone or stood still (Ben, 16 Nov: "if we see a car people or stuff moving around
//! us we should likely just stop"). The node ends the road ahead at the car meanwhile.

use std::collections::BTreeSet;
use std::fmt;
use std::str::FromStr;

use ego_core::fmt_g;
use ego_core::stop_law::StopCurve;

use crate::{py_max, py_min};

/// m/s: the lowest cap sent (`stop_law.FLOOR`).
pub const FLOOR: f64 = 0.3;

/// The machine's state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// No stop binds: no cap.
    Drive,
    /// A stop binds: the cap falls with the room left.
    Approach,
    /// Its lock holds the car, standing: WAIT next tick.
    Stopped,
    /// A lock holds the car.
    Wait,
    /// Every lock off: the cap rises from the floor.
    Go,
    /// Held too long, the goal aborted or the fence NEAR: a person is told.
    Ask,
}

impl State {
    /// The name the Python uses (`DRIVE`...).
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            State::Drive => "DRIVE",
            State::Approach => "APPROACH",
            State::Stopped => "STOPPED",
            State::Wait => "WAIT",
            State::Go => "GO",
            State::Ask => "ASK",
        }
    }

    /// A lock holds the car in this state (STOPPED, WAIT, ASK): the cap is the floor at once.
    #[must_use]
    pub fn held(self) -> bool {
        matches!(self, State::Stopped | State::Wait | State::Ask)
    }
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Why a stop is there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StopReason {
    /// A stop sign: camera range less the watcher's stop distance.
    Sign,
    /// A thing in the lane.
    Obstacle,
    /// Where the geofence would close NEAR.
    Fence,
    /// The plan's end.
    Goal,
    /// Something moving near holds the car: the road ahead's stop at 0 m (never a
    /// stop the machine is given).
    Moving,
}

/// A stop reason that isn't one of the four.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownReason(pub String);

impl fmt::Display for UnknownReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}: not sign, obstacle, fence, goal or moving", self.0)
    }
}

impl std::error::Error for UnknownReason {}

impl StopReason {
    /// The name the Python uses.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            StopReason::Sign => "sign",
            StopReason::Obstacle => "obstacle",
            StopReason::Fence => "fence",
            StopReason::Goal => "goal",
            StopReason::Moving => "moving",
        }
    }

    /// The lock that makes this reason's final stop (`LOCK_OF`); the goal's is Nav2's check.
    #[must_use]
    pub fn lock(self) -> Option<Lock> {
        match self {
            StopReason::Sign => Some(Lock::StopSign),
            StopReason::Obstacle => Some(Lock::StopZone),
            StopReason::Fence => Some(Lock::Geofence),
            StopReason::Goal | StopReason::Moving => None,
        }
    }

    /// Its release waits for the stop to stop binding (obstacle, fence, goal; not a sign).
    fn lets_go(self) -> bool {
        !matches!(self, StopReason::Sign)
    }
}

impl FromStr for StopReason {
    type Err = UnknownReason;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "sign" => Ok(StopReason::Sign),
            "obstacle" => Ok(StopReason::Obstacle),
            "fence" => Ok(StopReason::Fence),
            "goal" => Ok(StopReason::Goal),
            "moving" => Ok(StopReason::Moving),
            other => Err(UnknownReason(other.to_string())),
        }
    }
}

impl fmt::Display for StopReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A lock (twist_mux's names). In the order of their names, as the Python sorts them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Lock {
    /// `/safety/estop`.
    Estop,
    /// `/safety/geofence/lock`.
    Geofence,
    /// `/safety/sensor_watchdog/lock`.
    Sensors,
    /// `/perception/stop_sign`.
    StopSign,
    /// `/safety/stop_hold/lock`.
    StopZone,
}

impl Lock {
    /// Every lock, in name order.
    pub const ALL: [Lock; 5] = [
        Lock::Estop,
        Lock::Geofence,
        Lock::Sensors,
        Lock::StopSign,
        Lock::StopZone,
    ];

    /// twist_mux's name.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Lock::Estop => "estop",
            Lock::Geofence => "geofence",
            Lock::Sensors => "sensors",
            Lock::StopSign => "stop_sign",
            Lock::StopZone => "stop_zone",
        }
    }

    /// The topic the node reads it from (`LOCK_TOPICS`).
    #[must_use]
    pub fn topic(self) -> &'static str {
        match self {
            Lock::Estop => "/safety/estop",
            Lock::Geofence => "/safety/geofence/lock",
            Lock::Sensors => "/safety/sensor_watchdog/lock",
            Lock::StopSign => "/perception/stop_sign",
            Lock::StopZone => "/safety/stop_hold/lock",
        }
    }
}

impl FromStr for Lock {
    type Err = UnknownReason;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Lock::ALL
            .into_iter()
            .find(|l| l.as_str() == s)
            .ok_or_else(|| UnknownReason(s.to_string()))
    }
}

/// The locks on, joined as the Python does (`', '.join(sorted(locks))`).
fn joined(locks: &BTreeSet<Lock>) -> String {
    locks
        .iter()
        .map(|l| l.as_str())
        .collect::<Vec<_>>()
        .join(", ")
}

/// Why the machine is in its state.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum Reason {
    /// None (DRIVE with no stop).
    #[default]
    Nothing,
    /// A stop's reason.
    Stop(StopReason),
    /// Something moving near holds the car (`moving`).
    Moving,
    /// Words: the locks on, the e-stop, an aborted goal, a long hold.
    Said(String),
}

impl Reason {
    fn stop(&self) -> Option<StopReason> {
        match self {
            Reason::Stop(r) => Some(*r),
            _ => None,
        }
    }

    /// Whether the text is empty, as the Python's `''`.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        match self {
            Reason::Nothing => true,
            Reason::Stop(_) | Reason::Moving => false,
            Reason::Said(s) => s.is_empty(),
        }
    }
}

impl fmt::Display for Reason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Reason::Nothing => Ok(()),
            Reason::Stop(r) => f.write_str(r.as_str()),
            Reason::Moving => f.write_str(StopReason::Moving.as_str()),
            Reason::Said(s) => f.write_str(s),
        }
    }
}

/// The node's parameters. m/s, m/s², s, m.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// m/s: Nav2's own top speed (the cap is clamped to it).
    pub base: f64,
    /// m/s: the lowest cap sent (0.0 would mean no limit).
    pub floor: f64,
    /// m/s²: how fast the cap rises in GO.
    pub accel: f64,
    /// m/s²: how fast the cap may fall.
    pub fall: f64,
    /// m/s: slower counts as standing still.
    pub still: f64,
    /// s a lock other than the sign's may hold before ASK.
    pub ask_after: f64,
    /// s without odometry: the cap goes to the floor.
    pub odom_stale: f64,
    /// s without a sign point in APPROACH for the sign: DRIVE.
    pub sign_lost: f64,
    /// m past a sign's stop with no lock: DRIVE, and a warning.
    pub passed: f64,
    /// s an obstacle must stay gone before APPROACH lets go of it (TODO 189).
    pub release_hold: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            base: 1.0,
            floor: FLOOR,
            accel: 0.5,
            fall: 2.0,
            still: 0.05,
            ask_after: 30.0,
            odom_stale: 0.5,
            sign_lost: 5.0,
            passed: 1.0,
            release_hold: 1.0,
        }
    }
}

/// The settings can't make a cap.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SettingsError {
    /// m/s.
    pub floor: f64,
    /// m/s.
    pub base: f64,
}

impl fmt::Display for SettingsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "the floor must be above 0 and at most the base speed (floor {}, base {})",
            self.floor, self.base
        )
    }
}

impl std::error::Error for SettingsError {}

/// One reason to stop: why, and the metres left before its final stop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stop {
    /// Why.
    pub reason: StopReason,
    /// m before its final stop.
    pub left: f64,
}

impl Stop {
    /// A stop `left` m ahead.
    #[must_use]
    pub fn new(reason: StopReason, left: f64) -> Self {
        Stop { reason, left }
    }
}

/// What the node saw this tick.
#[derive(Debug, Clone, PartialEq)]
pub struct World {
    /// s.
    pub now: f64,
    /// m/s; `None`: no odometry yet.
    pub speed: Option<f64>,
    /// s since the last odometry.
    pub odom_age: f64,
    /// The locks on.
    pub locks: BTreeSet<Lock>,
    /// The geofence says NEAR.
    pub near: bool,
    /// Every stop in view this tick.
    pub stops: Vec<Stop>,
    /// s since the last sign point.
    pub sign_age: f64,
    /// A goal is running.
    pub goal_active: bool,
    /// This tick's news: the goal was aborted.
    pub goal_aborted: bool,
    /// Something moving near (`paths::moving_near`): hold.
    pub moving: bool,
}

impl World {
    /// Nothing seen at `now`: no odometry, no locks, no stops.
    #[must_use]
    pub fn at(now: f64) -> Self {
        World {
            now,
            speed: None,
            odom_age: f64::INFINITY,
            locks: BTreeSet::new(),
            near: false,
            stops: Vec::new(),
            sign_age: f64::INFINITY,
            goal_active: false,
            goal_aborted: false,
            moving: false,
        }
    }
}

/// What the node publishes this tick.
#[derive(Debug, Clone, PartialEq)]
pub struct Decision {
    /// The state.
    pub state: State,
    /// Why.
    pub reason: Reason,
    /// m/s for Nav2: 0.0 = no limit.
    pub cap: f64,
    /// m to the binding stop (inf: none).
    pub left: f64,
    /// This tick's warnings.
    pub warnings: Vec<String>,
}

/// The state topic's key-value pairs (all text), as `diagnostic()`.
#[must_use]
pub fn diagnostic(d: &Decision) -> [(&'static str, String); 5] {
    let reason = d.reason.to_string();
    [
        ("state", d.state.as_str().to_string()),
        (
            "reason",
            if reason.is_empty() {
                "-".into()
            } else {
                reason
            },
        ),
        ("cap_mps", format!("{:.3}", d.cap)),
        (
            "left_m",
            if d.left.is_finite() {
                format!("{:.2}", d.left)
            } else {
                "inf".into()
            },
        ),
        (
            "warnings",
            if d.warnings.is_empty() {
                "-".into()
            } else {
                d.warnings.join("; ")
            },
        ),
    ]
}

/// The state machine. [`Behaviour::step`] once a tick; it returns the [`Decision`].
#[derive(Debug, Clone)]
pub struct Behaviour {
    curve: StopCurve,
    s: Settings,
    state: State,
    reason: Reason,
    /// s: when the current state began.
    since: f64,
    /// m/s: the last cap sent (0.0: no limit).
    cap: f64,
    /// s: the last tick's time.
    last: Option<f64>,
    /// The stop_sign lock, last tick.
    sign_lock: bool,
    /// s: when the stop_sign lock last went off.
    served: f64,
    /// s: APPROACH(obstacle): when the obstacle stopped binding.
    gone_since: Option<f64>,
    /// s: when something moving near was last seen.
    moving_seen: f64,
    /// This tick: held for something moving near.
    holding: bool,
}

impl Behaviour {
    /// A machine in DRIVE, sending 0.0 (a start or a restart).
    ///
    /// # Errors
    ///
    /// [`SettingsError`] unless 0 < floor <= base.
    pub fn new(curve: StopCurve, settings: Settings) -> Result<Self, SettingsError> {
        if !(0.0 < settings.floor && settings.floor <= settings.base) {
            return Err(SettingsError {
                floor: settings.floor,
                base: settings.base,
            });
        }
        Ok(Behaviour {
            curve,
            s: settings,
            state: State::Drive,
            reason: Reason::Nothing,
            since: f64::NEG_INFINITY,
            cap: 0.0,
            last: None,
            sign_lock: false,
            served: f64::NEG_INFINITY,
            gone_since: None,
            moving_seen: f64::NEG_INFINITY,
            holding: false,
        })
    }

    /// The state now.
    #[must_use]
    pub fn state(&self) -> State {
        self.state
    }

    /// Held for something moving near, last tick (WAIT, reason `moving`): the node ends
    /// the road ahead at the car.
    #[must_use]
    pub fn holding(&self) -> bool {
        self.holding
    }

    /// The last cap sent (m/s; 0.0: no limit).
    #[must_use]
    pub fn cap(&self) -> f64 {
        self.cap
    }

    /// Nav2's cap for a stop `left` m ahead: 0.0, or floor..base (`stop_law.nav2_cap`).
    #[must_use]
    pub fn cap_for(&self, left: f64) -> f64 {
        nav2_cap(left, &self.curve, self.s.base, self.s.floor)
    }

    /// The stop whose cap binds (the lowest), or `None` if none does.
    #[must_use]
    pub fn binding<'a>(&self, stops: impl IntoIterator<Item = &'a Stop>) -> Option<&'a Stop> {
        let mut best: Option<&Stop> = None;
        let mut best_cap = f64::INFINITY;
        for stop in stops {
            let cap = self.cap_for(stop.left);
            if cap > 0.0 && cap < best_cap {
                best = Some(stop);
                best_cap = cap;
            }
        }
        best
    }

    fn go(&mut self, state: State, reason: Reason, now: f64) {
        if state != self.state || reason != self.reason {
            self.since = now;
        }
        self.state = state;
        self.reason = reason;
    }

    /// The cap to send: `want`, but falling no faster than `fall` allows.
    fn limit(&self, want: f64, dt: f64) -> f64 {
        if want == 0.0 {
            return 0.0;
        }
        // From no limit: start at the base speed and fall from there.
        let previous = if self.cap == 0.0 {
            self.s.base
        } else {
            self.cap
        };
        if want < previous {
            py_max(py_max(want, previous - self.s.fall * dt), self.s.floor)
        } else {
            want
        }
    }

    /// One tick.
    pub fn step(&mut self, w: &World) -> Decision {
        let s = self.s;
        let dt = match self.last {
            None => 0.0,
            Some(last) => py_max(0.0, w.now - last),
        };
        self.last = Some(w.now);
        let mut warnings = Vec::new();
        let locks = &w.locks;
        let stops = self.unserved(w, locks.contains(&Lock::StopSign));
        let still = w.speed.is_some_and(|v| v.abs() < s.still);
        let stop = self.binding(&stops).copied();
        let mut left = stop.map_or(f64::INFINITY, |x| x.left);
        // Something moving near, or gone or still for under release_hold s: hold.
        if w.moving {
            self.moving_seen = w.now;
        }
        let held = w.moving || w.now - self.moving_seen < s.release_hold;

        // Any state: the e-stop and an aborted goal.
        if locks.contains(&Lock::Estop) {
            self.go(State::Wait, Reason::Said("e-stop".into()), w.now);
        } else if w.goal_aborted {
            self.go(
                State::Ask,
                Reason::Said("the goal was aborted".into()),
                w.now,
            );
        } else if held
            && matches!(
                self.state,
                State::Drive | State::Approach | State::Stopped | State::Go
            )
        {
            self.gone_since = None;
            self.go(State::Wait, Reason::Moving, w.now);
        } else {
            match self.state {
                State::Drive => {
                    if !locks.is_empty() && still {
                        self.go(State::Wait, Reason::Said(joined(locks)), w.now);
                    } else if let Some(stop) = stop {
                        self.go(State::Approach, Reason::Stop(stop.reason), w.now);
                    }
                }
                State::Approach => self.approach(w, &stops, still, stop, &mut warnings),
                State::Stopped => self.go(State::Wait, self.reason.clone(), w.now),
                State::Wait => {
                    let only_sign = locks.len() == 1 && locks.contains(&Lock::StopSign);
                    if held && locks.is_empty() {
                        self.go(State::Wait, Reason::Moving, w.now);
                    } else if locks.is_empty() {
                        self.go(State::Go, self.reason.clone(), w.now);
                    } else if self.reason == Reason::Moving {
                        self.go(State::Wait, Reason::Said(joined(locks)), w.now);
                    } else if w.near && still {
                        self.go(
                            State::Ask,
                            Reason::Said("the geofence is NEAR: only a person can clear it".into()),
                            w.now,
                        );
                    } else if !only_sign && w.now - self.since > s.ask_after {
                        let why = format!("held over {} s: {}", fmt_g(s.ask_after), joined(locks));
                        self.go(State::Ask, Reason::Said(why), w.now);
                    }
                }
                State::Go => {
                    if !locks.is_empty() {
                        self.go(State::Wait, Reason::Said(joined(locks)), w.now);
                    } else if let Some(stop) = stop {
                        if self.cap_for(stop.left) <= py_max(self.cap, s.floor) {
                            self.go(State::Approach, Reason::Stop(stop.reason), w.now);
                        }
                    }
                }
                State::Ask => {
                    if held && locks.is_empty() {
                        self.go(State::Wait, Reason::Moving, w.now);
                    } else if locks.is_empty() {
                        self.go(State::Go, self.reason.clone(), w.now);
                    }
                }
            }
        }
        self.holding = self.state == State::Wait && self.reason == Reason::Moving;

        // The cap for the state.
        let mut want = match self.state {
            State::Drive => stop.map_or(0.0, |_| self.cap_for(left)),
            State::Approach if self.gone_since.is_some() => {
                // Holding the release (TODO 189): keep the cap it had; a new stop still binds.
                let held = py_max(self.cap, s.floor);
                match stop {
                    Some(_) => py_min(held, py_max(self.cap_for(left), s.floor)),
                    None => held,
                }
            }
            State::Approach => {
                let mine = self.reason.stop();
                let ahead = stops
                    .iter()
                    .filter(|x| Some(x.reason) == mine)
                    .map(|x| x.left)
                    .reduce(py_min)
                    .unwrap_or(left);
                let nearest = py_min(ahead, left);
                let mut want = py_max(self.cap_for(nearest), s.floor);
                // The sign's cap never rises in APPROACH: a new picture can't speed the car up.
                if mine == Some(StopReason::Sign) && self.cap > 0.0 {
                    want = py_min(want, self.cap);
                }
                left = nearest;
                want
            }
            State::Stopped | State::Wait | State::Ask => s.floor,
            State::Go => 0.0,
        };
        if self.state == State::Go {
            // From where the cap was (the floor after a wait), rising from the next tick.
            let rise = if self.since == w.now {
                0.0
            } else {
                s.accel * dt
            };
            let rising = py_max(self.cap, s.floor) + rise;
            let bound = stop.map_or(0.0, |_| self.cap_for(left));
            want = if bound > 0.0 {
                py_min(rising, bound)
            } else {
                rising
            };
            if want >= s.base {
                self.go(State::Drive, Reason::Nothing, w.now);
                want = 0.0;
            }
        }
        // Held by a lock, the car stands: the floor at once. Moving, the cap falls gently.
        let mut cap = if self.state.held() {
            want
        } else {
            self.limit(want, dt)
        };

        // Any state: stale odometry sends the floor, and says so.
        if w.speed.is_none() || w.odom_age > s.odom_stale {
            warnings.push(format!("odometry stale ({:.1} s): the floor", w.odom_age));
            cap = s.floor;
        }
        self.cap = cap;
        Decision {
            state: self.state,
            reason: self.reason.clone(),
            cap,
            left,
            warnings,
        }
    }

    /// The stops without a sign already stopped for.
    ///
    /// Once the sign's lock goes off, its last point (from before the stop) is behind the
    /// car's stop: only a point newer than the lock's end is a new sign.
    fn unserved(&mut self, w: &World, sign_lock: bool) -> Vec<Stop> {
        if self.sign_lock && !sign_lock {
            self.served = w.now;
        }
        self.sign_lock = sign_lock;
        if self.served.is_finite() && w.now - w.sign_age <= self.served {
            w.stops
                .iter()
                .filter(|x| x.reason != StopReason::Sign)
                .copied()
                .collect()
        } else {
            w.stops.clone()
        }
    }

    fn approach(
        &mut self,
        w: &World,
        stops: &[Stop],
        still: bool,
        stop: Option<Stop>,
        warnings: &mut Vec<String>,
    ) {
        let s = self.s;
        let reason = self.reason.stop();
        let mut mine: Vec<Stop> = stops
            .iter()
            .filter(|x| Some(x.reason) == reason)
            .copied()
            .collect();
        let lets_go = reason.is_some_and(StopReason::lets_go);
        if lets_go && self.binding(&mine).is_none() {
            // No longer binds: a new goal's long path, or the obstacle moved further off.
            mine.clear();
        }
        if !mine.is_empty() || reason != Some(StopReason::Obstacle) {
            self.gone_since = None;
        }
        let lock = reason.and_then(StopReason::lock);
        if lock.is_some_and(|l| w.locks.contains(&l)) && still {
            self.go(State::Stopped, self.reason.clone(), w.now);
        } else if !w.locks.is_empty() && still {
            self.go(State::Wait, Reason::Said(joined(&w.locks)), w.now);
        } else if reason == Some(StopReason::Sign)
            && mine
                .iter()
                .map(|x| x.left)
                .reduce(py_min)
                .is_some_and(|m| m < -s.passed)
        {
            warnings.push("sign passed, no lock".into());
            self.go(State::Drive, Reason::Nothing, w.now);
        } else if reason == Some(StopReason::Sign) && mine.is_empty() && w.sign_age > s.sign_lost {
            warnings.push("sign lost".into());
            self.go(State::Drive, Reason::Nothing, w.now);
        } else if reason == Some(StopReason::Goal) && !w.goal_active {
            self.go(State::Drive, Reason::Nothing, w.now);
        } else if lets_go && mine.is_empty() {
            // What we slowed for has gone (the obstacle moved, a new plan): speed up again;
            // an obstacle only once it has stayed gone release_hold s (TODO 189).
            if reason == Some(StopReason::Obstacle) && stop.is_none() {
                let since = *self.gone_since.get_or_insert(w.now);
                if w.now - since < s.release_hold {
                    return;
                }
            }
            self.gone_since = None;
            match stop {
                None => self.go(State::Go, self.reason.clone(), w.now),
                Some(stop) => self.go(State::Approach, Reason::Stop(stop.reason), w.now),
            }
        }
    }
}

/// The speed limit to send Nav2 for a stop `left` m ahead: m/s, never in (0, floor)
/// (`stop_law.nav2_cap`). Nav2 reads 0.0 as no limit.
#[must_use]
pub fn nav2_cap(left: f64, curve: &StopCurve, base: f64, floor: f64) -> f64 {
    let v = curve.cap(left);
    if !v.is_finite() || v >= base {
        return 0.0;
    }
    py_min(base, py_max(floor, v))
}

#[cfg(test)]
mod tests {
    //! `test_behaviour.py`, ported: one test per row of the design's event table.

    use super::*;

    const FAR: f64 = 100.0;

    fn curve() -> StopCurve {
        StopCurve::from_car_model(
            &ego_core::car_model::ActuatorConfig::default(),
            2.0,
            2.0,
            ego_core::stop_law::INPUT_DELAY,
            Some(&ego_core::car_model::SpeedLoopConfig::default()),
        )
        .unwrap()
    }

    fn machine(s: Settings) -> Behaviour {
        Behaviour::new(curve(), s).unwrap()
    }

    fn fresh() -> Behaviour {
        machine(Settings::default())
    }

    struct W(World);

    fn world(now: f64) -> W {
        W(World {
            speed: Some(1.0),
            odom_age: 0.05,
            ..World::at(now)
        })
    }

    impl W {
        fn speed(mut self, v: f64) -> Self {
            self.0.speed = Some(v);
            self
        }
        fn lock(mut self, l: Lock) -> Self {
            self.0.locks.insert(l);
            self
        }
        fn stop(mut self, r: StopReason, left: f64) -> Self {
            self.0.stops.push(Stop::new(r, left));
            self
        }
        fn sign_age(mut self, a: f64) -> Self {
            self.0.sign_age = a;
            self
        }
        fn goal(mut self, on: bool) -> Self {
            self.0.goal_active = on;
            self
        }
        fn near(mut self) -> Self {
            self.0.near = true;
            self
        }
        fn aborted(mut self) -> Self {
            self.0.goal_aborted = true;
            self
        }
        fn moving(mut self) -> Self {
            self.0.moving = true;
            self
        }
    }

    fn step(b: &mut Behaviour, w: W) -> Decision {
        b.step(&w.0)
    }

    use StopReason::{Goal, Obstacle, Sign};

    fn at_sign(b: &mut Behaviour, now: f64, left: f64) {
        let d = step(b, world(now).stop(Sign, left).sign_age(0.0));
        assert_eq!((d.state, d.reason), (State::Approach, Reason::Stop(Sign)));
    }

    #[test]
    fn starts_in_drive_sending_no_limit() {
        let d = step(&mut fresh(), world(0.0));
        assert_eq!((d.state, d.cap), (State::Drive, 0.0));
    }

    #[test]
    fn a_far_stop_doesnt_bind_and_a_near_one_starts_the_approach() {
        let mut b = fresh();
        assert_eq!(
            step(&mut b, world(0.0).stop(Goal, FAR).goal(true)).state,
            State::Drive
        );
        let d = step(&mut b, world(0.1).stop(Goal, 1.0).goal(true));
        assert_eq!((d.state, d.reason), (State::Approach, Reason::Stop(Goal)));
    }

    #[test]
    fn the_cap_is_never_between_zero_and_the_floor_nor_above_the_base() {
        let mut b = fresh();
        let mut t = 0.0;
        for i in 0..260 {
            t += 0.05;
            let left = 20.0 - 0.1 * f64::from(i);
            let d = step(&mut b, world(t).speed(0.5).stop(Sign, left).sign_age(0.0));
            assert!(d.cap == 0.0 || (FLOOR..=1.0).contains(&d.cap), "{}", d.cap);
        }
    }

    #[test]
    fn the_cap_falls_no_faster_than_the_car_can_slow() {
        let mut b = machine(Settings {
            base: 3.0,
            fall: 2.0,
            ..Settings::default()
        });
        step(&mut b, world(0.0).speed(3.0));
        let d = step(&mut b, world(0.05).speed(3.0).stop(Obstacle, 0.5));
        assert!((d.cap - (3.0 - 0.1)).abs() < 1e-12);
    }

    #[test]
    fn approach_to_stopped_when_its_lock_comes_and_the_car_is_still() {
        let mut b = fresh();
        at_sign(&mut b, 0.0, 1.0);
        let d = step(
            &mut b,
            world(1.0)
                .speed(0.0)
                .lock(Lock::StopSign)
                .stop(Sign, 0.1)
                .sign_age(0.0),
        );
        assert_eq!(d.state, State::Stopped);
        let d = step(&mut b, world(1.1).speed(0.0).lock(Lock::StopSign));
        assert_eq!(d.state, State::Wait);
    }

    #[test]
    fn approach_to_wait_when_another_lock_holds_a_still_car() {
        let mut b = fresh();
        at_sign(&mut b, 0.0, 1.0);
        let d = step(
            &mut b,
            world(1.0)
                .speed(0.0)
                .lock(Lock::Sensors)
                .stop(Sign, 3.0)
                .sign_age(0.0),
        );
        assert_eq!(
            (d.state, d.reason),
            (State::Wait, Reason::Said("sensors".into()))
        );
    }

    #[test]
    fn a_sign_passed_with_no_lock_goes_back_to_drive_and_warns() {
        let mut b = fresh();
        at_sign(&mut b, 0.0, 1.0);
        let d = step(&mut b, world(1.0).stop(Sign, -1.5).sign_age(0.0));
        assert_eq!(d.state, State::Drive);
        assert!(d.warnings.contains(&"sign passed, no lock".to_string()));
    }

    #[test]
    fn a_sign_lost_for_5_s_goes_back_to_drive_and_warns() {
        let mut b = fresh();
        at_sign(&mut b, 0.0, 1.0);
        assert_eq!(
            step(&mut b, world(3.0).sign_age(3.0)).state,
            State::Approach
        );
        let d = step(&mut b, world(6.0).sign_age(6.0));
        assert_eq!(d.state, State::Drive);
        assert!(d.warnings.contains(&"sign lost".to_string()));
    }

    #[test]
    fn a_goal_that_ends_ends_its_approach() {
        let mut b = fresh();
        step(&mut b, world(0.0).stop(Goal, 1.0).goal(true));
        assert_eq!(step(&mut b, world(0.1)).state, State::Drive);
    }

    #[test]
    fn a_new_goal_far_away_ends_the_last_goals_approach() {
        let mut b = fresh();
        step(&mut b, world(0.0).stop(Goal, 1.0).goal(true));
        assert_eq!(b.state(), State::Approach);
        let d = step(&mut b, world(0.1).stop(Goal, 15.0).goal(true));
        assert!(matches!(d.state, State::Go | State::Drive));
    }

    #[test]
    fn an_obstacle_that_goes_away_lets_the_car_speed_up_again() {
        let mut b = fresh();
        for i in 0..10 {
            step(&mut b, world(0.1 * f64::from(i)).stop(Obstacle, 1.0));
        }
        assert!(b.state() == State::Approach && b.cap() < 1.0);
        let held = b.cap();
        let d = step(&mut b, world(1.0));
        assert_eq!(d.state, State::Approach);
        assert!((d.cap - held).abs() < 1e-12);
        let d = step(&mut b, world(2.05));
        assert!(d.state == State::Go && d.cap >= FLOOR);
    }

    #[test]
    fn an_obstacle_flickering_at_the_lanes_edge_doesnt_flap_the_state() {
        let mut b = fresh();
        let states: Vec<State> = (0..60)
            .map(|i| {
                let w = world(0.1 * f64::from(i));
                let w = if (i / 3) % 2 == 0 {
                    w.stop(Obstacle, 1.0)
                } else {
                    w
                };
                step(&mut b, w).state
            })
            .collect();
        let first = states.iter().position(|s| *s == State::Approach).unwrap();
        assert!(states[first..].iter().all(|s| *s == State::Approach));
    }

    #[test]
    fn a_new_stop_still_binds_while_the_release_is_held() {
        let mut b = fresh();
        for i in 0..10 {
            step(&mut b, world(0.1 * f64::from(i)).stop(Obstacle, 1.0));
        }
        let held = step(&mut b, world(1.0)).cap;
        let d = step(&mut b, world(1.1).stop(Goal, 0.5));
        assert_eq!(
            (d.state, d.reason.clone()),
            (State::Approach, Reason::Stop(Goal))
        );
        assert!(d.cap <= held);
    }

    #[test]
    fn wait_to_go_when_every_lock_is_off_then_drive_at_the_base() {
        let mut b = fresh();
        at_sign(&mut b, 0.0, 1.0);
        step(
            &mut b,
            world(1.0)
                .speed(0.0)
                .lock(Lock::StopSign)
                .stop(Sign, 0.0)
                .sign_age(0.0),
        );
        step(&mut b, world(1.1).speed(0.0).lock(Lock::StopSign));
        let d = step(&mut b, world(5.0).speed(0.0));
        assert!(d.state == State::Go && d.cap >= FLOOR);
        let caps: Vec<f64> = (1..30)
            .map(|i| step(&mut b, world(5.0 + 0.1 * f64::from(i)).speed(0.3)).cap)
            .collect();
        assert!(caps[0] < caps[5] && b.state() == State::Drive && caps[28] == 0.0);
    }

    #[test]
    fn a_sign_stopped_for_never_brings_the_approach_back() {
        let mut b = fresh();
        at_sign(&mut b, 0.0, 1.5);
        step(
            &mut b,
            world(1.0)
                .speed(0.0)
                .lock(Lock::StopSign)
                .stop(Sign, -0.5)
                .sign_age(0.0),
        );
        step(
            &mut b,
            world(1.1)
                .speed(0.0)
                .lock(Lock::StopSign)
                .stop(Sign, -0.5)
                .sign_age(0.1),
        );
        assert_eq!(b.state(), State::Wait);
        let states: Vec<State> = (0..30)
            .map(|i| {
                let i = f64::from(i);
                step(
                    &mut b,
                    world(4.0 + 0.1 * i)
                        .speed(0.3)
                        .stop(Sign, -0.5)
                        .sign_age(3.0 + 0.1 * i),
                )
                .state
            })
            .collect();
        assert!(!states.contains(&State::Approach));
        assert_eq!((states[0], states[29]), (State::Go, State::Drive));
        let d = step(&mut b, world(8.0).stop(Sign, 1.0).sign_age(0.0));
        assert_eq!((d.state, d.reason), (State::Approach, Reason::Stop(Sign)));
    }

    #[test]
    fn a_long_hold_that_isnt_the_signs_asks_a_person() {
        let mut b = fresh();
        step(&mut b, world(0.0).speed(0.0).lock(Lock::StopZone));
        assert_eq!(b.state(), State::Wait);
        let d = step(&mut b, world(29.0).speed(0.0).lock(Lock::StopZone));
        assert_eq!(d.state, State::Wait);
        let d = step(&mut b, world(31.0).speed(0.0).lock(Lock::StopZone));
        assert_eq!((d.state, d.cap), (State::Ask, FLOOR));
        assert_eq!(d.reason, Reason::Said("held over 30 s: stop_zone".into()));
        let d = step(&mut b, world(40.0).speed(0.0).lock(Lock::StopZone));
        assert_eq!(d.state, State::Ask);
        assert_eq!(step(&mut b, world(41.0).speed(0.0)).state, State::Go);
    }

    #[test]
    fn the_signs_own_hold_never_asks() {
        let mut b = fresh();
        step(&mut b, world(0.0).speed(0.0).lock(Lock::StopSign));
        let d = step(&mut b, world(60.0).speed(0.0).lock(Lock::StopSign));
        assert_eq!(d.state, State::Wait);
    }

    #[test]
    fn geofence_near_at_a_standstill_asks_at_once() {
        let mut b = fresh();
        step(&mut b, world(0.0).speed(0.0).lock(Lock::Geofence).near());
        let d = step(&mut b, world(0.1).speed(0.0).lock(Lock::Geofence).near());
        assert_eq!(d.state, State::Ask);
    }

    #[test]
    fn go_to_wait_on_a_lock_and_go_to_approach_on_a_new_sign() {
        let mut b = fresh();
        step(&mut b, world(0.0).speed(0.0).lock(Lock::StopZone));
        step(&mut b, world(1.0).speed(0.0));
        assert_eq!(b.state(), State::Go);
        let d = step(&mut b, world(1.1).speed(0.2).lock(Lock::StopZone));
        assert_eq!(d.state, State::Wait);
        step(&mut b, world(1.2).speed(0.0));
        assert_eq!(b.state(), State::Go);
        let d = step(&mut b, world(1.3).speed(0.3).stop(Sign, 0.2).sign_age(0.0));
        assert_eq!(d.state, State::Approach);
    }

    #[test]
    fn any_state_an_aborted_goal_asks_and_the_estop_waits() {
        assert_eq!(step(&mut fresh(), world(0.0).aborted()).state, State::Ask);
        let mut b = fresh();
        at_sign(&mut b, 0.0, 1.0);
        let d = step(&mut b, world(0.5).lock(Lock::Estop));
        assert_eq!(
            (d.state, d.reason),
            (State::Wait, Reason::Said("e-stop".into()))
        );
    }

    #[test]
    fn stale_odometry_sends_the_floor_and_stays() {
        let d = fresh().step(&World {
            speed: Some(1.0),
            odom_age: 0.8,
            ..World::at(0.0)
        });
        assert_eq!((d.state, d.cap), (State::Drive, FLOOR));
        assert_eq!(
            d.warnings,
            vec!["odometry stale (0.8 s): the floor".to_string()]
        );
    }

    #[test]
    fn a_person_driving_changes_nothing() {
        let (mut a, mut b) = (fresh(), fresh());
        for t in 0..5 {
            let t = f64::from(t);
            assert_eq!(
                step(&mut a, world(t).speed(2.0)),
                step(&mut b, world(t).speed(0.1))
            );
        }
    }

    #[test]
    fn the_signs_cap_never_rises_in_approach() {
        let mut b = fresh();
        at_sign(&mut b, 0.0, 0.9);
        let caps: Vec<f64> = [0.8, 0.6, 4.0, 0.4]
            .into_iter()
            .enumerate()
            .map(|(i, left)| {
                step(
                    &mut b,
                    world(0.1 * (i + 1) as f64).stop(Sign, left).sign_age(0.0),
                )
                .cap
            })
            .collect();
        assert!(caps.windows(2).all(|w| w[1] <= w[0]));
    }

    #[test]
    fn the_state_topics_values() {
        let d = step(&mut fresh(), world(0.0));
        let values = diagnostic(&d);
        assert_eq!(values[0], ("state", "DRIVE".to_string()));
        assert_eq!(values[1], ("reason", "-".to_string()));
        assert_eq!(values[2], ("cap_mps", "0.000".to_string()));
        assert_eq!(values[3], ("left_m", "inf".to_string()));
    }

    #[test]
    fn a_floor_above_the_base_is_refused() {
        let s = Settings {
            floor: 2.0,
            ..Settings::default()
        };
        assert!(Behaviour::new(curve(), s).is_err());
        let s = Settings {
            floor: 0.0,
            ..Settings::default()
        };
        assert!(Behaviour::new(curve(), s).is_err());
    }

    #[test]
    fn the_names_read_back() {
        for l in Lock::ALL {
            assert_eq!(l.as_str().parse::<Lock>(), Ok(l));
        }
        assert_eq!("fence".parse::<StopReason>(), Ok(StopReason::Fence));
        assert!("wall".parse::<StopReason>().is_err());
    }

    #[test]
    fn something_moving_near_holds_the_car_until_it_has_gone_release_hold_s() {
        let mut b = fresh();
        step(&mut b, world(0.0));
        let d = step(&mut b, world(0.05).moving());
        assert_eq!(
            (d.state, d.reason, d.cap),
            (State::Wait, Reason::Moving, FLOOR)
        );
        assert!(b.holding());
        assert_eq!(step(&mut b, world(0.6).speed(0.0)).state, State::Wait);
        assert_eq!(
            step(&mut b, world(0.7).speed(0.0).moving()).state,
            State::Wait
        );
        assert_eq!(step(&mut b, world(1.6).speed(0.0)).state, State::Wait);
        let d = step(&mut b, world(1.75).speed(0.0));
        assert_eq!((d.state, d.reason), (State::Go, Reason::Moving));
        assert!(!b.holding());
    }

    #[test]
    fn a_hold_for_something_moving_never_outranks_the_estop_and_keeps_a_lock_waiting() {
        let mut b = fresh();
        let d = step(&mut b, world(0.0).lock(Lock::Estop).moving());
        assert_eq!(d.reason, Reason::Said("e-stop".into()));
        assert!(!b.holding());
        let d = step(&mut b, world(0.1).speed(0.0).moving());
        assert_eq!((d.state, d.reason), (State::Wait, Reason::Moving));
        let d = step(&mut b, world(0.2).speed(0.0).lock(Lock::StopZone).moving());
        assert_eq!(d.reason, Reason::Said("stop_zone".into()));
        let d = step(&mut b, world(0.3).speed(0.0).moving());
        assert_eq!((d.state, d.reason), (State::Wait, Reason::Moving));
        let mut b = fresh();
        assert_eq!(step(&mut b, world(0.0).aborted()).state, State::Ask);
        let d = step(&mut b, world(0.1).speed(0.0).moving());
        assert_eq!((d.state, d.reason), (State::Wait, Reason::Moving));
    }
}
