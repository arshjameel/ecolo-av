//! The edits, and the map as they leave it. The map read from disk never changes: the
//! editor keeps the list of edits and works out the map from them, so undo is "drop the
//! last edit and work it out again".
//!
//! Boxes in edits are in the **source map's frame** (before any hand alignment): the
//! window turns a box drawn on the screen back into it ([`crate::Region::unaligned`]).

use std::fmt;

use serde::{Deserialize, Serialize};

use crate::folder::{Cell, Grid2d, MapFolder, Point3};
use crate::geom::{Align, HeightBand, Point, Region};
use crate::labels::{Label, LabelClass, LabelSource};

/// What box-delete does to the 2D map's cells inside the box.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CellFill {
    /// Unknown: the planner treats them with care (the default).
    #[default]
    Unknown,
    /// Free: the lidar saw through, the thing is gone.
    Free,
    /// Leave the 2D map alone: only NDT's points go.
    Keep,
}

impl CellFill {
    /// Every choice, in the order the editor lists them.
    pub const ALL: [Self; 3] = [Self::Unknown, Self::Free, Self::Keep];

    /// The words on the screen.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Unknown => "cells unknown",
            Self::Free => "cells free",
            Self::Keep => "cells kept",
        }
    }
}

/// One thing a person did. The edit log (`edits.yaml`) lists them in order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "action", rename_all = "snake_case")]
pub enum Edit {
    /// Move the whole map by hand onto the street map (replaces any earlier alignment).
    Align {
        /// The whole move, from the source map.
        to: Align,
    },
    /// Remove NDT's points in a box (and a height band), and set the 2D cells in it.
    Delete {
        /// The box.
        region: Region,
        /// Only these heights; all if left out.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        band: Option<HeightBand>,
        /// What the 2D cells inside become.
        #[serde(default)]
        cells: CellFill,
    },
    /// Add a label.
    Label {
        /// What is there.
        class: LabelClass,
        /// Where.
        region: Region,
        /// Which heights; all if left out.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        band: Option<HeightBand>,
    },
    /// Change a label's class ("tree, not wall").
    Relabel {
        /// Which label.
        id: u32,
        /// Its new class.
        class: LabelClass,
    },
    /// Remove a label.
    Unlabel {
        /// Which label.
        id: u32,
    },
}

/// What an edit changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct Effect {
    /// NDT's points it removed.
    pub points_removed: usize,
    /// 2D cells it changed.
    pub cells_changed: usize,
    /// The label it made, changed or removed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<u32>,
}

/// Why an edit was refused. Nothing changes.
#[derive(Debug, Clone, PartialEq)]
pub enum EditError {
    /// A box with no area, or not numbers.
    BadRegion,
    /// A band whose bottom is above its top, or not numbers.
    BadBand,
    /// An alignment that isn't numbers.
    BadAlign,
    /// No label has this number.
    NoSuchLabel(u32),
}

impl fmt::Display for EditError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadRegion => write!(f, "the box has no area"),
            Self::BadBand => write!(f, "the height band's bottom is above its top"),
            Self::BadAlign => write!(f, "the alignment must be numbers"),
            Self::NoSuchLabel(id) => write!(f, "there is no label {id}"),
        }
    }
}

impl std::error::Error for EditError {}

/// A map being edited.
#[derive(Debug, Clone)]
pub struct Editor {
    map: MapFolder,
    edits: Vec<Edit>,
    effects: Vec<Effect>,
    align: Align,
    kept: Vec<bool>,
    cells: Vec<Cell>,
    labels: Vec<Label>,
}

impl Editor {
    /// Starts editing `map`, with no edits.
    #[must_use]
    pub fn new(map: MapFolder) -> Self {
        let mut editor = Self {
            kept: vec![true; map.points.len()],
            cells: map.grid.cells.clone(),
            labels: map.labels.clone(),
            map,
            edits: Vec::new(),
            effects: Vec::new(),
            align: Align::IDENTITY,
        };
        editor.replay();
        editor
    }

    /// The map as it was read: never changed.
    #[must_use]
    pub fn source(&self) -> &MapFolder {
        &self.map
    }

    /// Every edit so far, in order.
    #[must_use]
    pub fn edits(&self) -> &[Edit] {
        &self.edits
    }

    /// What each edit changed, in the same order.
    #[must_use]
    pub fn effects(&self) -> &[Effect] {
        &self.effects
    }

    /// The hand alignment now.
    #[must_use]
    pub fn align(&self) -> Align {
        self.align
    }

    /// The labels now, in the source map's frame.
    #[must_use]
    pub fn labels(&self) -> &[Label] {
        &self.labels
    }

    /// The 2D map's cells now, in the source map's frame (on its grid).
    #[must_use]
    pub fn cells(&self) -> &[Cell] {
        &self.cells
    }

    /// Whether each of NDT's points is still there, in the source's order.
    #[must_use]
    pub fn kept(&self) -> &[bool] {
        &self.kept
    }

    /// NDT's points still there, in the source map's frame.
    pub fn kept_points(&self) -> impl Iterator<Item = &Point3> + Clone {
        self.map
            .points
            .iter()
            .zip(&self.kept)
            .filter_map(|(p, &k)| k.then_some(p))
    }

    /// How many of NDT's points are gone.
    #[must_use]
    pub fn points_removed(&self) -> usize {
        self.kept.iter().filter(|&&k| !k).count()
    }

    /// True if nothing has been done.
    #[must_use]
    pub fn is_unchanged(&self) -> bool {
        self.edits.is_empty()
    }

    /// Does an edit, and returns what it changed.
    ///
    /// # Errors
    ///
    /// [`EditError`] if the edit makes no sense; nothing changes then.
    pub fn apply(&mut self, edit: Edit) -> Result<Effect, EditError> {
        self.check(&edit)?;
        let effect = self.run(&edit);
        self.edits.push(edit);
        self.effects.push(effect);
        Ok(effect)
    }

    /// Takes back the last edit; returns it, or `None` if there was none.
    pub fn undo(&mut self) -> Option<Edit> {
        let last = self.edits.pop();
        if last.is_some() {
            self.replay();
        }
        last
    }

    /// The 2D map now, after the alignment (new cells if it moved: [`Grid2d::aligned`]).
    #[must_use]
    pub fn output_grid(&self) -> Grid2d {
        Grid2d {
            cells: self.cells.clone(),
            ..self.map.grid.clone()
        }
        .aligned(&self.align)
    }

    /// NDT's points now, after the alignment.
    #[must_use]
    pub fn output_points(&self) -> Vec<Point3> {
        let a = self.align;
        self.kept_points()
            .map(|p| {
                let q = a.apply(Point::new(p.x, p.y));
                Point3::new(q.x, q.y, p.z)
            })
            .collect()
    }

    /// The labels now, after the alignment.
    #[must_use]
    pub fn output_labels(&self) -> Vec<Label> {
        self.labels
            .iter()
            .map(|l| Label {
                region: l.region.aligned(&self.align),
                ..l.clone()
            })
            .collect()
    }

    fn check(&self, edit: &Edit) -> Result<(), EditError> {
        let band_ok = |band: &Option<HeightBand>| band.as_ref().is_none_or(HeightBand::is_valid);
        match edit {
            Edit::Align { to } if !to.is_finite() => Err(EditError::BadAlign),
            Edit::Delete { region, .. } | Edit::Label { region, .. } if !region.is_valid() => {
                Err(EditError::BadRegion)
            }
            Edit::Delete { band, .. } | Edit::Label { band, .. } if !band_ok(band) => {
                Err(EditError::BadBand)
            }
            Edit::Relabel { id, .. } | Edit::Unlabel { id }
                if !self.labels.iter().any(|l| l.id == *id) =>
            {
                Err(EditError::NoSuchLabel(*id))
            }
            _ => Ok(()),
        }
    }

    /// Works the map out again from the source and the edits.
    fn replay(&mut self) {
        self.kept = vec![true; self.map.points.len()];
        self.cells = self.map.grid.cells.clone();
        self.labels = self.map.labels.clone();
        self.align = Align::IDENTITY;
        let edits = std::mem::take(&mut self.edits);
        self.effects = edits.iter().map(|e| self.run(e)).collect();
        self.edits = edits;
    }

    fn run(&mut self, edit: &Edit) -> Effect {
        match edit {
            Edit::Align { to } => {
                self.align = *to;
                Effect::default()
            }
            Edit::Delete {
                region,
                band,
                cells,
            } => self.delete(region, band.as_ref(), *cells),
            Edit::Label {
                class,
                region,
                band,
            } => {
                let id = self.labels.iter().map(|l| l.id).max().unwrap_or(0) + 1;
                self.labels.push(Label {
                    id,
                    class: *class,
                    region: *region,
                    band: *band,
                    source: LabelSource::Person,
                    confidence: None,
                    seen_in: Vec::new(),
                });
                Effect {
                    label: Some(id),
                    ..Effect::default()
                }
            }
            Edit::Relabel { id, class } => {
                if let Some(label) = self.labels.iter_mut().find(|l| l.id == *id) {
                    label.class = *class;
                    // A person checked it: it is theirs now.
                    label.source = LabelSource::Person;
                    label.confidence = None;
                }
                Effect {
                    label: Some(*id),
                    ..Effect::default()
                }
            }
            Edit::Unlabel { id } => {
                self.labels.retain(|l| l.id != *id);
                Effect {
                    label: Some(*id),
                    ..Effect::default()
                }
            }
        }
    }

    fn delete(&mut self, region: &Region, band: Option<&HeightBand>, fill: CellFill) -> Effect {
        let (lo, hi) = region.bounds();
        let mut removed = 0;
        for (p, kept) in self.map.points.iter().zip(self.kept.iter_mut()) {
            let flat = Point::new(p.x, p.y);
            if *kept
                && (lo.x..=hi.x).contains(&p.x)
                && (lo.y..=hi.y).contains(&p.y)
                && region.contains(flat)
                && band.is_none_or(|b| b.contains(p.z))
            {
                *kept = false;
                removed += 1;
            }
        }
        let new = match fill {
            CellFill::Unknown => Cell::Unknown,
            CellFill::Free => Cell::Free,
            CellFill::Keep => {
                return Effect {
                    points_removed: removed,
                    ..Effect::default()
                }
            }
        };
        let grid = &self.map.grid;
        let (cols, rows) = grid.span(lo, hi);
        let mut changed = 0;
        for row in rows {
            for col in cols.clone() {
                let index = row * grid.width + col;
                if region.contains(grid.centre(index)) && self.cells[index] != new {
                    self.cells[index] = new;
                    changed += 1;
                }
            }
        }
        Effect {
            points_removed: removed,
            cells_changed: changed,
            label: None,
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::path::PathBuf;

    /// A small map: a row of points along y = 5 at heights 0.5 and 2.5, and a 20 x 20 m
    /// grid of 0.5 m cells, occupied along y = 5.
    pub(crate) fn sample() -> MapFolder {
        let mut points = Vec::new();
        for i in 0..20 {
            for z in [0.5, 2.5] {
                points.push(Point3::new(f64::from(i) + 0.5, 5.0, z));
            }
        }
        let (width, height) = (40, 40);
        let mut cells = vec![Cell::Free; width * height];
        for col in 0..width {
            cells[10 * width + col] = Cell::Occupied;
        }
        MapFolder {
            dir: PathBuf::from("/nowhere/map"),
            points,
            ndt_yaml: serde_yaml_ng::Mapping::new(),
            grid: Grid2d {
                origin: Point::new(0.0, 0.0),
                resolution: 0.5,
                width,
                height,
                cells,
            },
            grid_yaml: serde_yaml_ng::Mapping::new(),
            path: Vec::new(),
            labels: Vec::new(),
        }
    }

    fn a_box() -> Region {
        Region::from_corners(Point::new(2.0, 4.0), Point::new(6.0, 6.0))
    }

    #[test]
    fn box_delete_takes_points_and_cells() {
        let mut ed = Editor::new(sample());
        let effect = ed
            .apply(Edit::Delete {
                region: a_box(),
                band: None,
                cells: CellFill::Unknown,
            })
            .expect("done");
        // x 2.5, 3.5, 4.5, 5.5: four places, two heights.
        assert_eq!(effect.points_removed, 8);
        assert_eq!(ed.points_removed(), 8);
        // 8 x 4 cells in the box: free ones and occupied ones all unknown now.
        assert_eq!(effect.cells_changed, 32);
        assert_eq!(
            ed.cells().iter().filter(|&&c| c == Cell::Unknown).count(),
            32
        );
        // The source is untouched.
        assert!(ed.source().grid.cells.iter().all(|&c| c != Cell::Unknown));
        assert_eq!(ed.source().points.len(), 40);
    }

    #[test]
    fn a_height_band_keeps_the_rest() {
        let mut ed = Editor::new(sample());
        let effect = ed
            .apply(Edit::Delete {
                region: a_box(),
                band: Some(HeightBand::new(0.0, 1.0)),
                cells: CellFill::Keep,
            })
            .expect("done");
        assert_eq!(effect.points_removed, 4);
        assert_eq!(effect.cells_changed, 0);
        assert!(ed
            .kept_points()
            .filter(|p| p.x < 6.0 && p.x > 2.0)
            .all(|p| p.z > 2.0));
    }

    #[test]
    fn a_turned_box_deletes_only_inside() {
        let mut ed = Editor::new(sample());
        // A thin box across the row at 45 degrees, through (10, 5).
        let region = Region {
            centre_m: [10.0, 5.0],
            size_m: [3.0, 0.8],
            yaw_rad: std::f64::consts::FRAC_PI_4,
        };
        let effect = ed
            .apply(Edit::Delete {
                region,
                band: None,
                cells: CellFill::Free,
            })
            .expect("done");
        // Only x 9.5 and 10.5 are within 0.4 m across the box's long side.
        assert_eq!(effect.points_removed, 4);
        assert!(ed.cells().iter().all(|&c| c != Cell::Unknown));
    }

    #[test]
    fn undo_puts_it_back() {
        let mut ed = Editor::new(sample());
        let first = Edit::Delete {
            region: a_box(),
            band: None,
            cells: CellFill::Unknown,
        };
        ed.apply(first.clone()).expect("done");
        ed.apply(Edit::Align {
            to: Align::new(1.0, 2.0, 0.1),
        })
        .expect("done");
        assert_eq!(
            ed.undo(),
            Some(Edit::Align {
                to: Align::new(1.0, 2.0, 0.1)
            })
        );
        assert_eq!(ed.align(), Align::IDENTITY);
        assert_eq!(ed.points_removed(), 8);
        assert_eq!(ed.undo(), Some(first));
        assert_eq!(ed.points_removed(), 0);
        assert_eq!(ed.cells(), ed.source().grid.cells.as_slice());
        assert_eq!(ed.undo(), None);
        assert!(ed.is_unchanged());
    }

    #[test]
    fn labels_add_change_and_go() {
        let mut ed = Editor::new(sample());
        let made = ed
            .apply(Edit::Label {
                class: LabelClass::Building,
                region: a_box(),
                band: None,
            })
            .expect("done");
        let id = made.label.expect("a label");
        ed.apply(Edit::Relabel {
            id,
            class: LabelClass::Tree,
        })
        .expect("done");
        assert_eq!(ed.labels()[0].class, LabelClass::Tree);
        assert_eq!(
            ed.apply(Edit::Relabel {
                id: 99,
                class: LabelClass::Pole
            }),
            Err(EditError::NoSuchLabel(99))
        );
        ed.apply(Edit::Unlabel { id }).expect("done");
        assert!(ed.labels().is_empty());
        ed.undo();
        assert_eq!(ed.labels()[0].class, LabelClass::Tree);
        // A refused edit is not in the list.
        assert_eq!(ed.edits().len(), 2);
    }

    #[test]
    fn bad_edits_are_refused() {
        let mut ed = Editor::new(sample());
        let flat = Region::from_corners(Point::new(1.0, 1.0), Point::new(1.0, 3.0));
        assert_eq!(
            ed.apply(Edit::Delete {
                region: flat,
                band: None,
                cells: CellFill::Unknown
            }),
            Err(EditError::BadRegion)
        );
        assert_eq!(
            ed.apply(Edit::Label {
                class: LabelClass::Pole,
                region: a_box(),
                band: Some(HeightBand::new(3.0, 1.0))
            }),
            Err(EditError::BadBand)
        );
        assert_eq!(
            ed.apply(Edit::Align {
                to: Align::new(f64::NAN, 0.0, 0.0)
            }),
            Err(EditError::BadAlign)
        );
        assert!(ed.is_unchanged());
    }

    #[test]
    fn alignment_moves_the_output_only() {
        let mut ed = Editor::new(sample());
        let a = Align::new(100.0, -50.0, 0.0);
        ed.apply(Edit::Align { to: a }).expect("done");
        let out = ed.output_points();
        assert!((out[0].x - 100.5).abs() < 1e-9 && (out[0].y + 45.0).abs() < 1e-9);
        // The points in memory are where they were.
        assert!((ed.source().points[0].x - 0.5).abs() < 1e-12);
        assert_eq!(ed.output_grid().origin, Point::new(100.0, -50.0));
    }

    #[test]
    fn the_log_reads_back() {
        let edits = vec![
            Edit::Align {
                to: Align::new(1.0, 2.0, 0.01),
            },
            Edit::Delete {
                region: a_box(),
                band: Some(HeightBand::new(0.0, 2.0)),
                cells: CellFill::Free,
            },
            Edit::Label {
                class: LabelClass::ParkedCar,
                region: a_box(),
                band: None,
            },
            Edit::Relabel {
                id: 1,
                class: LabelClass::Pole,
            },
            Edit::Unlabel { id: 1 },
        ];
        let text = serde_yaml_ng::to_string(&edits).expect("writes");
        assert!(text.contains("action: delete"));
        let back: Vec<Edit> = serde_yaml_ng::from_str(&text).expect("reads");
        assert_eq!(back, edits);
    }
}
