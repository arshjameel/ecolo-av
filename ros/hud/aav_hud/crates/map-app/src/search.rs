//! The place search in the window: the database, what a key or a click does, and sending
//! the goal. The steps themselves (typing, picked, confirming) are map-core's
//! `destination::Finder`; the search is map-core's `places`.

use std::time::{Duration, Instant};

use map_core::{
    destination::{goal_for, Action, Finder, Key, Refusal},
    earth::{Datum, LatLon},
    geom::{Metres, Point, Pose},
    places::{Place, Places},
    state::MapState,
    view::View,
};

use crate::source::DataSource;

/// How long "Goal sent" stays on the screen.
const NOTICE_FOR: Duration = Duration::from_secs(6);

/// A line under the search box for a few seconds: the goal went, or why not.
#[derive(Debug, Clone)]
pub struct Notice {
    pub text: String,
    pub good: bool,
    at: Instant,
}

/// The search box and its database.
pub struct Search {
    /// The database, or why there isn't one ("No place database: pixi run get places").
    places: Result<Places, String>,
    pub finder: Finder,
    datum: Datum,
    notice: Option<Notice>,
}

impl Search {
    pub fn new(places: Result<Places, String>, datum: Datum) -> Self {
        Self {
            places,
            finder: Finder::new(),
            datum,
            notice: None,
        }
    }

    /// A place on the map (map frame, metres), as navsat_transform would put it.
    pub fn on_map(&self, place: &Place) -> Point {
        self.datum.to_map(place.at)
    }

    /// How far a place is from the car, when the car's position is known.
    pub fn distance_from_car(&self, place: &Place, state: &MapState) -> Option<Metres> {
        Some(state.car_position()?.distance(self.on_map(place)))
    }

    /// The goal for a place now, or why not.
    pub fn goal(&self, place: &Place, state: &MapState) -> Result<Pose, Refusal> {
        goal_for(
            self.on_map(place),
            state.car_position(),
            state.grid.as_ref().map(|g| &g.info),
        )
    }

    /// The picked place's goal, or why not; None when nothing is picked.
    pub fn picked_goal(&self, state: &MapState) -> Option<Result<Pose, Refusal>> {
        self.finder.picked().map(|p| self.goal(p, state))
    }

    pub fn notice(&self, now: Instant) -> Option<&Notice> {
        self.notice
            .as_ref()
            .filter(|n| now.duration_since(n.at) < NOTICE_FOR)
    }

    /// Where results are ranked from: the car, or before it's known, the map's datum.
    fn near(&self, state: &MapState) -> LatLon {
        state
            .car_position()
            .map_or(self.datum.origin, |c| self.datum.to_lat_lon(c))
    }

    /// One key or click, as the search box sees it.
    pub fn key(
        &mut self,
        key: Key,
        state: &MapState,
        view: &mut View,
        source: &mut dyn DataSource,
        now: Instant,
    ) {
        let can_drive = matches!(self.picked_goal(state), Some(Ok(_)));
        let action = self.finder.key(key, can_drive);
        self.act(action, state, view, source, now);
    }

    /// Starts with these words in the box (`--search`).
    pub fn prefill(
        &mut self,
        query: &str,
        state: &MapState,
        view: &mut View,
        source: &mut dyn DataSource,
        now: Instant,
    ) {
        let action = self.finder.prefill(query);
        self.act(action, state, view, source, now);
    }

    /// A click on result `i`, or `--pick`.
    pub fn choose(
        &mut self,
        i: usize,
        state: &MapState,
        view: &mut View,
        source: &mut dyn DataSource,
        now: Instant,
    ) {
        let action = self.finder.choose(i);
        self.act(action, state, view, source, now);
    }

    fn act(
        &mut self,
        action: Action,
        state: &MapState,
        view: &mut View,
        source: &mut dyn DataSource,
        now: Instant,
    ) {
        match action {
            Action::Nothing => {}
            Action::Search => {
                let results = self.find(self.finder.query(), state);
                self.finder.set_results(results);
            }
            Action::Show(place) => view.look_at(self.on_map(&place)),
            Action::Send(place) => {
                let (text, good) = match self.send(&place, state, source) {
                    Ok(()) => (format!("Goal sent to Nav2: {}", place.title), true),
                    Err(why) => (why, false),
                };
                self.notice = Some(Notice {
                    text,
                    good,
                    at: now,
                });
            }
        }
    }

    fn find(&self, query: &str, state: &MapState) -> Result<Vec<Place>, String> {
        let places = self.places.as_ref().map_err(Clone::clone)?;
        places
            .search(query, Some(self.near(state)))
            .map_err(|e| e.to_string())
    }

    /// Checks the place again (the car or the map may have changed), then publishes.
    fn send(
        &self,
        place: &Place,
        state: &MapState,
        source: &mut dyn DataSource,
    ) -> Result<(), String> {
        let goal = self.goal(place, state).map_err(|r| r.to_string())?;
        source
            .send_goal(goal, state.frame())
            .map_err(|e| format!("Couldn't send the goal: {e:#}"))
    }

    /// `--send-goal NAME`: searches, and sends the first result Nav2 can take, once.
    /// Says what it did on the terminal (for tests: nobody can type into the window).
    pub fn send_first(
        &mut self,
        query: &str,
        state: &MapState,
        source: &mut dyn DataSource,
        now: Instant,
    ) -> Result<String, String> {
        let results = self.find(query, state)?;
        let mut refused = Vec::new();
        for place in &results {
            match self.goal(place, state) {
                Ok(goal) => {
                    self.send(place, state, source)?;
                    let text = format!(
                        "sent {:?} ({}) as the goal: x {:.2} m, y {:.2} m, yaw {:.2} rad, frame {}",
                        place.title,
                        place.detail(),
                        goal.position.x,
                        goal.position.y,
                        goal.yaw,
                        state.frame()
                    );
                    self.notice = Some(Notice {
                        text: format!("Goal sent to Nav2: {}", place.title),
                        good: true,
                        at: now,
                    });
                    return Ok(text);
                }
                Err(why) => refused.push(format!("{}: {why}", place.title)),
            }
        }
        if results.is_empty() {
            Err(format!("no place found for {query:?}"))
        } else {
            Err(format!("no result Nav2 can take: {}", refused.join("; ")))
        }
    }
}
