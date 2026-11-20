//! Labels: a box on the map that says what is there, saved as `labels.yaml`.
//!
//! ```yaml
//! version: 1
//! frame: map
//! labels:
//! - id: 1
//!   class: parked_car
//!   region: {centre_m: [51.0, 15.0], size_m: [3.0, 32.0], yaw_rad: 0.0}
//!   band: {min_m: 0.0, max_m: 2.0}      # optional: all heights if left out
//!   source: person                       # person; passes (map-area); camera later
//! ```
//!
//! Room for what comes next, read now and written only when set:
//! - `source: camera` and `confidence` (0-1): the camera's labels on the lidar points
//!   (TODO 252) arrive as labels too, and a person keeps, fixes or drops them here;
//! - `seen_in`: the drives that saw it. Occupancy across passes (TODO 259, `pixi run
//!   map-area`) removes what one drive saw and another saw through, and writes a label
//!   with `source: passes` and `seen_in: [<drive>]` for each thing only one drive saw.

use serde::{Deserialize, Serialize};

use crate::geom::{HeightBand, Region};

/// What a label says is there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LabelClass {
    /// A tree: it changes with the seasons.
    Tree,
    /// A building: it stays.
    Building,
    /// A parked car: it may be gone next time.
    ParkedCar,
    /// A pole, a sign, a light.
    Pole,
    /// Anything else.
    Other,
}

impl LabelClass {
    /// Every class, in the order the editor lists them.
    pub const ALL: [Self; 5] = [
        Self::Tree,
        Self::Building,
        Self::ParkedCar,
        Self::Pole,
        Self::Other,
    ];

    /// The words on the screen.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Tree => "tree",
            Self::Building => "building",
            Self::ParkedCar => "parked car",
            Self::Pole => "pole",
            Self::Other => "other",
        }
    }

    /// Its colour on the map (r, g, b).
    #[must_use]
    pub fn rgb(self) -> [u8; 3] {
        match self {
            Self::Tree => [60, 200, 80],
            Self::Building => [230, 160, 60],
            Self::ParkedCar => [80, 170, 255],
            Self::Pole => [240, 240, 90],
            Self::Other => [210, 120, 220],
        }
    }
}

/// Who made a label.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LabelSource {
    /// A person, in the editor.
    #[default]
    Person,
    /// The camera's detector on the lidar points (TODO 252; nothing makes these yet).
    Camera,
    /// Occupancy across passes (`map-area`, TODO 259): seen by one drive only.
    Passes,
}

/// One label.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Label {
    /// Its number, unique in its file.
    pub id: u32,
    /// What it says is there.
    pub class: LabelClass,
    /// Where.
    pub region: Region,
    /// Which heights; all of them if left out.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub band: Option<HeightBand>,
    /// Who made it.
    #[serde(default)]
    pub source: LabelSource,
    /// How sure a model was (0-1); a person's labels have none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub confidence: Option<f64>,
    /// The drives that saw it (TODO 259).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub seen_in: Vec<String>,
}

/// `labels.yaml`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LabelsFile {
    /// The format's version: 1.
    pub version: u32,
    /// The frame the boxes are in: `map`.
    pub frame: String,
    /// The labels.
    #[serde(default)]
    pub labels: Vec<Label>,
}

impl LabelsFile {
    /// The format's version now.
    pub const VERSION: u32 = 1;

    /// A file of these labels, in the map frame.
    #[must_use]
    pub fn new(labels: Vec<Label>) -> Self {
        Self {
            version: Self::VERSION,
            frame: "map".into(),
            labels,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_file_round_trips_and_reads_the_future_fields() {
        let text = "version: 1\nframe: map\nlabels:\n\
                    - id: 3\n  class: parked_car\n  region: {centre_m: [1, 2], size_m: [3, 4]}\n\
                    - id: 4\n  class: tree\n  region: {centre_m: [0, 0], size_m: [1, 1], yaw_rad: 0.5}\n  \
                    source: camera\n  confidence: 0.8\n  seen_in: [drive_a]\n";
        let file: LabelsFile = serde_yaml_ng::from_str(text).expect("reads");
        assert_eq!(file.labels[0].class, LabelClass::ParkedCar);
        assert_eq!(file.labels[0].source, LabelSource::Person);
        assert_eq!(file.labels[1].confidence, Some(0.8));
        let again: LabelsFile =
            serde_yaml_ng::from_str(&serde_yaml_ng::to_string(&file).expect("writes"))
                .expect("reads");
        assert_eq!(again, file);
        // A person's label writes no empty future fields.
        let written = serde_yaml_ng::to_string(&LabelsFile::new(vec![file.labels[0].clone()]))
            .expect("writes");
        assert!(!written.contains("confidence") && !written.contains("seen_in"));
    }

    #[test]
    fn map_areas_labels_read() {
        let text = "version: 1\nframe: map\nlabels:\n- id: 1\n  class: other\n  \
                    region: {centre_m: [22.1, -5.4], size_m: [4.5, 0.5], yaw_rad: 0.0}\n  \
                    band: {min_m: 0.3, max_m: 2.0}\n  source: passes\n  \
                    seen_in: [2026-11-19_180523_simbox]\n";
        let file: LabelsFile = serde_yaml_ng::from_str(text).expect("reads");
        assert_eq!(file.labels[0].source, LabelSource::Passes);
        assert_eq!(
            file.labels[0].seen_in,
            vec!["2026-11-19_180523_simbox".to_string()]
        );
    }

    #[test]
    fn an_unknown_class_is_refused() {
        let text = "version: 1\nframe: map\nlabels:\n- id: 1\n  class: dragon\n  \
                    region: {centre_m: [0, 0], size_m: [1, 1]}\n";
        assert!(serde_yaml_ng::from_str::<LabelsFile>(text).is_err());
    }
}
