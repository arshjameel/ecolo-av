//! Picking a destination: the search box's steps, and whether a place can be Nav2's goal.
//!
//! ```text
//!   Closed --/ or a click--> Typing --Enter or a click--> Picked --Enter--> Confirming
//!     ^                        |  ^                         |                 |
//!     +-------- Esc -----------+  +---------- Esc ----------+  <---- Esc -----+
//!     ^                                                                       |
//!     +------------------------- Enter: the goal is sent ---------------------+
//! ```
//!
//! Two presses to send a goal (DRIVE HERE, then CONFIRM), so a stray tap never moves the
//! car. A place outside Nav2's map, or with no car position, can't get past Picked: the
//! screen says why ([`Refusal`]).

use std::fmt;

use crate::{
    geom::{Metres, Point, Pose},
    grid::GridInfo,
    places::Place,
};

/// Why a place can't be sent to Nav2 as a goal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Refusal {
    /// No `/map` yet: nobody knows where Nav2 can plan.
    NoMap,
    /// Off Nav2's map: it can't plan there. How far from the map's edge.
    OutsideMap { from_edge: Metres },
    /// No position for the car: no way to say which way to face, or that it's safe.
    NoCar,
}

impl fmt::Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoMap => write!(f, "No Nav2 map yet: can't send a goal"),
            Self::OutsideMap { from_edge } => {
                write!(f, "Outside Nav2's map: {} away", far(*from_edge))
            }
            Self::NoCar => write!(f, "No car position: can't send a goal"),
        }
    }
}

/// "850 m", "3.2 km".
fn far(d: Metres) -> String {
    if d.0 >= 1000.0 {
        format!("{:.1} km", d.0 / 1000.0)
    } else {
        format!("{:.0} m", d.0)
    }
}

/// The goal for a place at `place` (map frame): facing from the car toward it. Refused
/// with no map, outside the map, or with no car.
pub fn goal_for(place: Point, car: Option<Point>, map: Option<&GridInfo>) -> Result<Pose, Refusal> {
    let map = map.ok_or(Refusal::NoMap)?;
    let from_edge = map.distance_outside(place);
    if from_edge.0 > 0.0 {
        return Err(Refusal::OutsideMap { from_edge });
    }
    let car = car.ok_or(Refusal::NoCar)?;
    let d = place - car;
    let yaw = if d.length() > 1e-6 {
        d.y.atan2(d.x)
    } else {
        0.0
    };
    Ok(Pose {
        position: place,
        yaw,
    })
}

/// Where the search box is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Step {
    /// Shut: the map's own keys (F, S, + -) work.
    #[default]
    Closed,
    /// Typing a name; the results under it.
    Typing,
    /// One place chosen: on the map, with DRIVE HERE.
    Picked,
    /// DRIVE HERE was pressed: CONFIRM sends the goal.
    Confirming,
}

/// A key, as the search box sees it. The window turns raylib's keys into these.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Key {
    /// `/`, or a click on the box.
    Open,
    Char(char),
    /// Removes the last letter (a whole letter: é is one, though two bytes).
    Backspace,
    /// Empties the box (the on-screen keyboard's Clear).
    Clear,
    Up,
    Down,
    Enter,
    Escape,
}

/// What the window must do after a key.
#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    Nothing,
    /// The words changed: search again, then [`Finder::set_results`].
    Search,
    /// A place was chosen: centre the map on it.
    Show(Place),
    /// CONFIRM: send this place as the goal.
    Send(Place),
}

/// The search box: what's typed, the results, and the step.
#[derive(Debug, Clone, Default)]
pub struct Finder {
    query: String,
    results: Vec<Place>,
    selected: usize,
    step: Step,
    picked: Option<Place>,
    /// Why the last search failed, if it did.
    problem: Option<String>,
}

impl Finder {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    #[must_use]
    pub fn step(&self) -> Step {
        self.step
    }

    #[must_use]
    pub fn query(&self) -> &str {
        &self.query
    }

    #[must_use]
    pub fn results(&self) -> &[Place] {
        &self.results
    }

    /// The highlighted result (Up and Down move it).
    #[must_use]
    pub fn selected(&self) -> usize {
        self.selected
    }

    /// The chosen place, while it's on the map (Picked or Confirming).
    #[must_use]
    pub fn picked(&self) -> Option<&Place> {
        match self.step {
            Step::Picked | Step::Confirming => self.picked.as_ref(),
            Step::Closed | Step::Typing => None,
        }
    }

    #[must_use]
    pub fn problem(&self) -> Option<&str> {
        self.problem.as_deref()
    }

    /// Whether keys are letters for the box, not the map's own keys.
    #[must_use]
    pub fn is_typing(&self) -> bool {
        self.step == Step::Typing
    }

    /// Whether the box takes Enter and Esc (it's open).
    #[must_use]
    pub fn is_open(&self) -> bool {
        self.step != Step::Closed
    }

    /// Starts with these words (`--search`), open, ready to search.
    pub fn prefill(&mut self, query: &str) -> Action {
        self.query = query.to_owned();
        self.step = Step::Typing;
        Action::Search
    }

    /// The results of the last [`Action::Search`].
    pub fn set_results(&mut self, results: Result<Vec<Place>, String>) {
        match results {
            Ok(r) => {
                self.results = r;
                self.problem = None;
            }
            Err(why) => {
                self.results.clear();
                self.problem = Some(why);
            }
        }
        self.selected = 0;
    }

    /// A click on result `i`, or `--pick`: chooses it.
    pub fn choose(&mut self, i: usize) -> Action {
        if self.step != Step::Typing || i >= self.results.len() {
            return Action::Nothing;
        }
        self.selected = i;
        self.key(Key::Enter, false)
    }

    /// One key. `can_drive`: whether the picked place can be a goal now
    /// ([`goal_for`] says yes); without it, DRIVE HERE and CONFIRM do nothing.
    pub fn key(&mut self, key: Key, can_drive: bool) -> Action {
        match (self.step, key) {
            (Step::Closed, Key::Open) | (Step::Picked, Key::Open) => {
                self.step = Step::Typing;
                Action::Search
            }
            (Step::Closed, _) => Action::Nothing,

            (Step::Typing, Key::Char(c)) if !c.is_control() => {
                self.query.push(c);
                Action::Search
            }
            (Step::Typing, Key::Backspace) => {
                if self.query.pop().is_some() {
                    Action::Search
                } else {
                    Action::Nothing
                }
            }
            (Step::Typing, Key::Clear) => {
                if self.query.is_empty() {
                    Action::Nothing
                } else {
                    self.query.clear();
                    Action::Search
                }
            }
            (Step::Typing, Key::Up) => {
                self.selected = self.selected.saturating_sub(1);
                Action::Nothing
            }
            (Step::Typing, Key::Down) => {
                self.selected = (self.selected + 1).min(self.results.len().saturating_sub(1));
                Action::Nothing
            }
            (Step::Typing, Key::Enter) => match self.results.get(self.selected) {
                Some(place) => {
                    self.picked = Some(place.clone());
                    self.step = Step::Picked;
                    Action::Show(place.clone())
                }
                None => Action::Nothing,
            },
            (Step::Typing, Key::Escape) => {
                self.step = Step::Closed;
                Action::Nothing
            }
            (Step::Typing, _) => Action::Nothing,

            (Step::Picked, Key::Enter) if can_drive => {
                self.step = Step::Confirming;
                Action::Nothing
            }
            (Step::Picked, Key::Escape) => {
                self.step = Step::Typing;
                Action::Nothing
            }
            (Step::Picked, _) => Action::Nothing,

            (Step::Confirming, Key::Enter) if can_drive => {
                self.step = Step::Closed;
                self.picked.take().map_or(Action::Nothing, Action::Send)
            }
            // The car or the map went away meanwhile: back to the place, with the reason.
            (Step::Confirming, Key::Enter) | (Step::Confirming, Key::Escape) => {
                self.step = Step::Picked;
                Action::Nothing
            }
            (Step::Confirming, _) => Action::Nothing,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{earth::LatLon, places::Match};

    fn place(title: &str) -> Place {
        Place {
            id: 1,
            title: title.into(),
            kind: "place".into(),
            address: None,
            at: LatLon::new(45.0, -75.0),
            matched: Match::Prefix,
            distance: None,
        }
    }

    fn lot() -> GridInfo {
        // 80 m x 60 m from (-10, -10), as the test lot.
        GridInfo {
            width: 320,
            height: 240,
            resolution: 0.25,
            origin: Pose::new(-10.0, -10.0, 0.0),
        }
    }

    #[test]
    fn a_goal_faces_away_from_the_car() {
        let g = goal_for(
            Point::new(10.0, 10.0),
            Some(Point::new(0.0, 0.0)),
            Some(&lot()),
        )
        .expect("inside");
        assert_eq!(g.position, Point::new(10.0, 10.0));
        assert!((g.yaw - std::f64::consts::FRAC_PI_4).abs() < 1e-12);
        // The car on the spot: facing east.
        let g = goal_for(
            Point::new(1.0, 1.0),
            Some(Point::new(1.0, 1.0)),
            Some(&lot()),
        );
        assert_eq!(g.map(|p| p.yaw), Ok(0.0));
    }

    #[test]
    fn goals_off_the_map_or_without_a_car_are_refused() {
        let car = Some(Point::new(0.0, 0.0));
        assert_eq!(
            goal_for(Point::new(0.0, 0.0), car, None),
            Err(Refusal::NoMap)
        );
        let far = goal_for(Point::new(3270.0, 0.0), car, Some(&lot()));
        assert_eq!(
            far,
            Err(Refusal::OutsideMap {
                from_edge: Metres(3200.0)
            })
        );
        assert_eq!(
            far.expect_err("refused").to_string(),
            "Outside Nav2's map: 3.2 km away"
        );
        assert_eq!(
            Refusal::OutsideMap {
                from_edge: Metres(42.4)
            }
            .to_string(),
            "Outside Nav2's map: 42 m away"
        );
        assert_eq!(
            goal_for(Point::new(5.0, 5.0), None, Some(&lot())),
            Err(Refusal::NoCar)
        );
    }

    #[test]
    fn type_pick_drive_here_confirm() {
        let mut f = Finder::new();
        assert_eq!(
            f.key(Key::Char('x'), true),
            Action::Nothing,
            "closed: not typing"
        );
        assert_eq!(f.key(Key::Open, true), Action::Search);
        assert!(f.is_typing());
        assert_eq!(f.key(Key::Char('c'), true), Action::Search);
        assert_eq!(f.key(Key::Char('a'), true), Action::Search);
        assert_eq!(f.query(), "ca");
        f.set_results(Ok(vec![place("Carleton"), place("Carleton U")]));
        f.key(Key::Down, true);
        f.key(Key::Down, true);
        assert_eq!(f.selected(), 1, "stops at the last");
        f.key(Key::Up, true);
        f.key(Key::Down, true);
        assert_eq!(f.key(Key::Enter, true), Action::Show(place("Carleton U")));
        assert_eq!(f.step(), Step::Picked);
        assert_eq!(f.picked().map(|p| p.title.as_str()), Some("Carleton U"));
        assert_eq!(f.key(Key::Enter, true), Action::Nothing);
        assert_eq!(f.step(), Step::Confirming);
        assert_eq!(f.key(Key::Enter, true), Action::Send(place("Carleton U")));
        assert_eq!(f.step(), Step::Closed);
        assert_eq!(f.picked(), None);
    }

    #[test]
    fn escape_steps_back_and_a_refused_place_stays_picked() {
        let mut f = Finder::new();
        f.prefill("carl");
        f.set_results(Ok(vec![place("Carleton")]));
        assert_eq!(f.choose(5), Action::Nothing);
        assert_eq!(f.choose(0), Action::Show(place("Carleton")));
        // Outside the map: DRIVE HERE does nothing.
        assert_eq!(f.key(Key::Enter, false), Action::Nothing);
        assert_eq!(f.step(), Step::Picked);
        f.key(Key::Enter, true);
        assert_eq!(f.step(), Step::Confirming);
        f.key(Key::Escape, true);
        assert_eq!(f.step(), Step::Picked);
        f.key(Key::Enter, true);
        // The car's position went away before CONFIRM: back to Picked, nothing sent.
        assert_eq!(f.key(Key::Enter, false), Action::Nothing);
        assert_eq!(f.step(), Step::Picked);
        f.key(Key::Escape, true);
        assert_eq!(f.step(), Step::Typing);
        assert_eq!(f.key(Key::Backspace, true), Action::Search);
        assert_eq!(f.query(), "car");
        f.key(Key::Escape, true);
        assert_eq!(f.step(), Step::Closed);
        assert!(!f.is_open());
    }

    #[test]
    fn a_failed_search_says_why() {
        let mut f = Finder::new();
        f.prefill("x");
        f.set_results(Err("no place database".into()));
        assert_eq!(f.problem(), Some("no place database"));
        assert_eq!(f.key(Key::Enter, true), Action::Nothing);
        f.set_results(Ok(vec![]));
        assert_eq!(f.problem(), None);
        // Backspace on nothing: nothing to search.
        f.key(Key::Backspace, true);
        assert_eq!(f.key(Key::Backspace, true), Action::Nothing);
    }

    #[test]
    fn backspace_takes_a_whole_letter_and_clear_empties() {
        let mut f = Finder::new();
        f.prefill("Côté");
        assert_eq!(f.key(Key::Backspace, true), Action::Search);
        assert_eq!(f.query(), "Côt");
        f.key(Key::Backspace, true);
        assert_eq!(f.query(), "Cô");
        f.key(Key::Backspace, true);
        assert_eq!(f.query(), "C");
        assert_eq!(f.key(Key::Clear, true), Action::Search);
        assert_eq!(f.query(), "");
        assert_eq!(f.key(Key::Clear, true), Action::Nothing, "already empty");
        // Closed, Clear does nothing.
        f.key(Key::Escape, true);
        assert_eq!(f.key(Key::Clear, true), Action::Nothing);
    }
}
