//! The map editor's logic (TODO 251, phase E of `docs/testing-plan.md`): no ROS, no
//! window. The window is `map_edit` in `ros/hud/aav_dev_window`; the page is
//! `docs/map-editor.md`.
//!
//! ```text
//!  <drive>/map/ ─▶ MapFolder::load ─▶ Editor ─ apply(Edit) / undo ─▶ save ─▶ <drive>/map-edit-1/
//!   (map-drive)     never changed       │                              new folder, edits.yaml,
//!                                       ▼                              chunks cut again
//!                         layers: HeightImage, grid_rgba, chunk_outlines
//! ```
//!
//! | Edit | Does |
//! |---|---|
//! | `Align` | moves the whole map (x, y, yaw) onto the street map; written into the YAML and the log |
//! | `Delete` | a box (turned or not) and a height band: NDT's points go, the 2D cells become unknown, free or stay |
//! | `Label`, `Relabel`, `Unlabel` | a box with a class: tree, building, parked car, pole, other |
//!
//! Points never move silently: the map in memory stays as read, the move is one logged
//! edit, and the saved YAML says it (`placed: aligned_by_hand`, `edited:`).

pub mod editor;
pub mod folder;
pub mod geom;
pub mod labels;
pub mod layers;
pub mod save;

pub use editor::{CellFill, Edit, EditError, Editor, Effect};
pub use folder::{Cell, Grid2d, LoadError, MapFolder, Point3, TumPose};
pub use geom::{Align, HeightBand, Point, Region};
pub use labels::{Label, LabelClass, LabelSource, LabelsFile};
pub use layers::{chunk_outlines, grid_rgba, height_rgb, HeightImage};
pub use save::{next_free, EditLog, SaveError, Saved, CHUNK_SIZE};
