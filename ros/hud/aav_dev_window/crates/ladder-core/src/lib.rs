//! The test ladder's window (`pixi run ladder-window`), without the window: no Bevy, no
//! egui, no ROS.
//!
//! `tools/ladder.py` holds every rule and is the only writer of the progress file. This
//! crate only reads what it prints (`--json`), decides what a cell and a button show, and
//! builds the tool's own commands. Nothing here can start a car test: there is no function
//! that returns one.
//!
//! | Module | Holds |
//! |---|---|
//! | [`ladder`] | the tool's JSON as types: [`Ladder`], [`Rung`], [`Side`], [`State`], [`Place`] |
//! | [`show`] | what a cell says, whether the run button is on and why not, the car's side |
//! | [`command`] | the argument lists for `tools/ladder.py`: the table, a run, a record |
//! | [`process`] | the tool as a child: its last lines, stopped by its process group |
//! | [`session`] | the window's state: the ladder read every 2 s, one run at a time, the form |
//! | [`pretend`] | sample data, for `--pretend` |

pub mod command;
pub mod ladder;
pub mod pretend;
pub mod process;
pub mod session;
pub mod show;

pub use command::{Outcome, RecordForm, ToolCommand};
pub use ladder::{Ladder, ParseError, Place, Rung, Side, State};
pub use session::{Session, Source};
pub use show::{CarSide, Cell, RunButton, WhyNot};
