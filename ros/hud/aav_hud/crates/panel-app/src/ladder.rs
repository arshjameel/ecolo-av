//! The LADDER tab's state (wrap-up step 3, the test bench): the test ladder in the
//! hardware page, so the hardware, its graphs, the small steps by hand and the automated
//! tests are one screen (Ben, 25 Nov).
//!
//! Every rule is `tools/ladder.py`'s, and every decision about a button is `ladder-core`'s
//! (the crate the Bevy `ladder_window` uses): this only finds the tool, ticks the session
//! while it matters, and keeps which row and which graph a person chose. Like
//! `ladder_window`, the tab starts simulator runs only: ladder-core has no function that
//! returns a car test, and the car's command is only shown.
//!
//! | Where it runs | The tab shows |
//! |---|---|
//! | `--demo` | ladder-core's pretend data: nothing runs, nothing is recorded |
//! | in the repository (or installed inside it) | the tool's ladder, read every 2 s while the tab shows |
//! | no `tools/ladder.py` above the folder or the program | why, in words: no tool here |

use std::{
    path::{Path, PathBuf},
    time::Instant,
};

use ladder_core::{command::TOOL, Answer, Rung, Session, Source};

/// One of the answer's views, under its tabs: a graph with its numbers, or the faults.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum View {
    /// `answer.graphs[i]`.
    Graph(usize),
    /// The faults tried on purpose.
    Faults,
}

/// The views an answer has, in order: its graphs, then its faults.
pub fn views(answer: &Answer) -> Vec<View> {
    (0..answer.graphs.len())
        .map(View::Graph)
        .chain(answer.faults.as_ref().map(|_| View::Faults))
        .collect()
}

/// The tab: the ladder's session, or why there is none.
#[derive(Debug)]
pub struct LadderTab {
    reach: Result<Session, String>,
    /// The answer's view shown: an index into [`views`].
    view: usize,
}

impl LadderTab {
    /// `pretend`: the panel's demo car, so the ladder's sample data. Otherwise the tool is
    /// looked for above the current folder, then above the program itself.
    pub fn open(pretend: bool) -> Self {
        if pretend {
            return Self::with(Ok(Source::Pretend));
        }
        let starts = [std::env::current_dir().ok(), std::env::current_exe().ok()];
        let found = find_root(starts.iter().flatten().map(PathBuf::as_path));
        Self::with(found.map(|root| Source::Tool { root }))
    }

    fn with(source: Result<Source, String>) -> Self {
        Self {
            reach: source.map(Session::new),
            view: 0,
        }
    }

    /// The session; `None` when the tool is not reachable here.
    pub fn session(&self) -> Option<&Session> {
        self.reach.as_ref().ok()
    }

    /// Why there is no ladder here, in words.
    pub fn missing(&self) -> Option<&str> {
        self.reach.as_ref().err().map(String::as_str)
    }

    /// Once a frame. The tool is asked only while the tab shows, or while a run it
    /// started goes on (its lines and its end must be read); a hidden tab costs nothing.
    pub fn tick(&mut self, now: Instant, shown: bool) {
        if let Ok(session) = &mut self.reach {
            if shown || session.running().is_some() {
                session.tick(now);
            }
        }
    }

    /// The rungs as the table lists them: the top rung first, the sensors' last.
    pub fn rows(&self) -> Vec<&Rung> {
        self.session()
            .and_then(Session::ladder)
            .map(|l| l.top_first().collect())
            .unwrap_or_default()
    }

    /// The row of the rung chosen.
    pub fn selected_row(&self) -> Option<usize> {
        let id = &self.session()?.selected()?.id;
        self.rows().iter().position(|r| &r.id == id)
    }

    /// A row was clicked.
    pub fn select_row(&mut self, row: usize) {
        let id = self.rows().get(row).map(|r| r.id.clone());
        if let (Some(id), Ok(session)) = (id, &mut self.reach) {
            if session.selected().map(|r| &r.id) != Some(&id) {
                self.view = 0;
            }
            session.select(&id);
        }
    }

    /// Up (-1) or Down (+1) through the rows.
    pub fn select_next(&mut self, step: isize) {
        let last = self.rows().len().saturating_sub(1);
        let row = self
            .selected_row()
            .map_or(0, |r| r.saturating_add_signed(step).min(last));
        self.select_row(row);
    }

    /// The selected rung's answer, if its session has one.
    pub fn answer(&self) -> Option<&Answer> {
        self.session()?.report()?.answer.as_ref()
    }

    /// The answer's view shown, if it has any.
    pub fn view(&self) -> Option<View> {
        let views = views(self.answer()?);
        views.get(self.view).or(views.first()).copied()
    }

    /// One of the answer's tabs was clicked.
    pub fn show_view(&mut self, i: usize) {
        if self.answer().is_some_and(|a| i < views(a).len()) {
            self.view = i;
        }
    }

    /// The index of the view shown.
    pub fn view_index(&self) -> usize {
        let count = self.answer().map_or(0, |a| views(a).len());
        if self.view < count {
            self.view
        } else {
            0
        }
    }

    /// RUN: ladder-core starts it only if its button is on.
    pub fn press_run(&mut self) {
        if let Ok(session) = &mut self.reach {
            session.press_run();
        }
    }

    /// STOP: the run's process group is asked to end; pressed again, killed.
    pub fn press_stop(&mut self) {
        if let Ok(session) = &mut self.reach {
            session.press_stop();
        }
    }

    /// The simulated car's jacks: only when ladder-core's button is on (a simulator here).
    pub fn press_jacks(&mut self) {
        if let Ok(session) = &mut self.reach {
            session.press_jacks();
        }
    }
}

/// The repository: the first folder that holds `tools/ladder.py`, above any of `starts`.
fn find_root<'a>(starts: impl Iterator<Item = &'a Path>) -> Result<PathBuf, String> {
    let mut looked = Vec::new();
    for start in starts {
        if let Some(root) = start.ancestors().find(|dir| dir.join(TOOL).is_file()) {
            return Ok(root.to_owned());
        }
        looked.push(start.display().to_string());
    }
    Err(format!(
        "No {TOOL} here (looked above {}): the ladder needs the repository. Start this \
         program inside a checkout (pixi run hw-panel), or use pixi run ladder-window there.",
        if looked.is_empty() {
            "nothing".to_owned()
        } else {
            looked.join(" and ")
        }
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ladder_core::{RunButton, WhyNot};

    #[test]
    fn pretend_data_opens_on_rung_1_and_runs_nothing() {
        let mut tab = LadderTab::open(true);
        assert!(tab.missing().is_none());
        assert_eq!(tab.rows().len(), 10);
        let row = tab.selected_row().expect("a rung is chosen");
        assert_eq!(tab.rows()[row].id, "stands");
        tab.tick(Instant::now(), true);
        let session = tab.session().expect("pretend");
        assert_eq!(
            session.run_button(),
            Some(RunButton::Off(WhyNot::Pretend)),
            "ladder-core keeps Run off"
        );
        tab.press_run();
        assert!(tab.session().and_then(Session::running).is_none());
        // Rung 1's answer: three graphs and the faults.
        let answer = tab.answer().expect("the pretend answer");
        assert_eq!(
            views(answer),
            [View::Graph(0), View::Graph(1), View::Graph(2), View::Faults]
        );
        tab.show_view(3);
        assert_eq!(tab.view(), Some(View::Faults));
        tab.show_view(9);
        assert_eq!(tab.view_index(), 3, "a view it does not have is ignored");
    }

    #[test]
    fn a_hidden_tab_asks_nothing() {
        let mut tab = LadderTab::open(true);
        tab.tick(Instant::now(), false);
        assert!(tab.answer().is_none(), "not read while hidden");
        tab.tick(Instant::now(), true);
        assert!(tab.answer().is_some());
    }

    #[test]
    fn up_and_down_stay_in_the_table_and_a_new_rung_shows_its_first_view() {
        let mut tab = LadderTab::open(true);
        tab.tick(Instant::now(), true);
        tab.show_view(2);
        let start = tab.selected_row().expect("chosen");
        tab.select_next(-1);
        assert_eq!(tab.selected_row(), Some(start - 1));
        assert_eq!(tab.view_index(), 0);
        for _ in 0..20 {
            tab.select_next(-1);
        }
        assert_eq!(tab.selected_row(), Some(0));
        for _ in 0..20 {
            tab.select_next(1);
        }
        assert_eq!(tab.selected_row(), Some(tab.rows().len() - 1));
    }

    #[test]
    fn the_tool_is_found_from_the_repository_and_missing_elsewhere_says_why() {
        let here = Path::new(env!("CARGO_MANIFEST_DIR"));
        let root = find_root([here].into_iter()).expect("this checkout has the tool");
        assert!(root.join(TOOL).is_file());
        let why = find_root([Path::new("/")].into_iter()).expect_err("no tool at /");
        assert!(
            why.contains("tools/ladder.py") && why.contains("repository"),
            "{why}"
        );
        let tab = LadderTab::with(Err(why));
        assert!(tab.session().is_none() && tab.rows().is_empty());
        assert!(tab.missing().is_some());
    }
}
