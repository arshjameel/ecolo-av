//! Approve and save: the edited map as a **new** folder, never over the one it came from.
//!
//! ```text
//!  <drive>/map/            ─ the source, read only
//!  <drive>/map-edit-1/     ─ written whole into map-edit-1.partial/, then renamed
//!     ndt.pcd, ndt.yaml    NDT's map; the YAML says it was edited, and how it was moved
//!     map2d.pgm, .yaml     the 2D map
//!     poses.tum            the path, moved with the map
//!     labels.yaml          the labels (map frame)
//!     edits.yaml           every edit, in order, and what each changed
//!     chunks/              64 m chunks, cut as map-drive cuts them (map-chunks, ndt-core)
//! ```

use std::fmt;
use std::fs::{self, File};
use std::io::{self, BufWriter};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use map_chunks::{Grid, WholeMap};
use serde::{Deserialize, Serialize};
use serde_yaml_ng::{Mapping, Value};

use crate::editor::{Edit, Editor, Effect};
use crate::folder::{Grid2d, Point3};
use crate::geom::{Align, Point};
use crate::labels::LabelsFile;

/// The chunks' side (m), as map-drive cuts them.
pub const CHUNK_SIZE: f64 = 64.0;
/// Files copied as they are, if the source has them.
const COPIED: [&str; 2] = ["closures.yaml", "poses_slam.tum"];

/// Why the map couldn't be saved. The source is never touched.
#[derive(Debug)]
pub enum SaveError {
    /// The folder is already there: nothing is ever written over.
    Exists(PathBuf),
    /// The folder is the source.
    IsSource(PathBuf),
    /// A file couldn't be written.
    Io(PathBuf, io::Error),
    /// A YAML file couldn't be written.
    Yaml(String),
    /// The chunks couldn't be cut.
    Chunks(String),
}

impl fmt::Display for SaveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Exists(p) => write!(f, "{} is already there: pick a new folder", p.display()),
            Self::IsSource(p) => {
                write!(f, "{} is the map being edited: save elsewhere", p.display())
            }
            Self::Io(p, e) => write!(f, "{}: {e}", p.display()),
            Self::Yaml(e) => write!(f, "writing YAML: {e}"),
            Self::Chunks(e) => write!(f, "cutting the chunks: {e}"),
        }
    }
}

impl std::error::Error for SaveError {}

/// What a save wrote.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Saved {
    /// The new folder.
    pub dir: PathBuf,
    /// NDT's points in it.
    pub points: usize,
    /// NDT's points left out.
    pub points_removed: usize,
    /// Chunks written: fine, coarse (the 2D map), NDT's.
    pub chunks: [usize; 3],
}

/// One line of the edit log: the edit and what it changed.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Record {
    /// The edit.
    #[serde(flatten)]
    pub edit: Edit,
    /// What it changed.
    #[serde(flatten)]
    pub effect: Effect,
}

/// `edits.yaml`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EditLog {
    /// The folder the edits were made on.
    pub source: String,
    /// When it was saved (s since 1970, UTC).
    pub saved_unix_s: u64,
    /// The hand alignment in the end (the boxes below are in the source's frame, before it).
    pub align: Align,
    /// NDT's points before and after.
    pub points_before: usize,
    /// NDT's points after.
    pub points_after: usize,
    /// Every edit, in order.
    pub edits: Vec<Record>,
}

/// The first `<source>-edit-<n>` beside the source that isn't there yet.
#[must_use]
pub fn next_free(source: &Path) -> PathBuf {
    let name = source
        .file_name()
        .map_or_else(|| "map".into(), |n| n.to_string_lossy().into_owned());
    let parent = source.parent().unwrap_or(Path::new("."));
    (1..)
        .map(|n| parent.join(format!("{name}-edit-{n}")))
        .find(|p| !p.exists() && !partial(p).exists())
        .unwrap_or_else(|| parent.join(format!("{name}-edit")))
}

fn partial(out: &Path) -> PathBuf {
    let mut name = out.file_name().unwrap_or_default().to_os_string();
    name.push(".partial");
    out.with_file_name(name)
}

fn same_place(a: &Path, b: &Path) -> bool {
    match (a.canonicalize(), b.canonicalize()) {
        (Ok(a), Ok(b)) => a == b,
        _ => a == b,
    }
}

impl Editor {
    /// Approves the edits and writes the edited map to `out`, a folder that must not be
    /// there yet. Everything goes into `<out>.partial` first, renamed at the end, so a
    /// failed save leaves no half map under the real name.
    ///
    /// # Errors
    ///
    /// [`SaveError`]: `out` is there already or is the source, or a file can't be written.
    pub fn save(&self, out: &Path) -> Result<Saved, SaveError> {
        let source = &self.source().dir;
        if same_place(out, source) {
            return Err(SaveError::IsSource(out.to_path_buf()));
        }
        if out.exists() {
            return Err(SaveError::Exists(out.to_path_buf()));
        }
        let work = partial(out);
        if work.exists() {
            fs::remove_dir_all(&work).map_err(|e| SaveError::Io(work.clone(), e))?;
        }
        fs::create_dir_all(&work).map_err(|e| SaveError::Io(work.clone(), e))?;
        let saved = self.write_all(&work);
        if saved.is_err() {
            // Best effort: the error below is the one that matters.
            let _ = fs::remove_dir_all(&work);
        }
        let saved = saved?;
        fs::rename(&work, out).map_err(|e| SaveError::Io(out.to_path_buf(), e))?;
        Ok(Saved {
            dir: out.to_path_buf(),
            ..saved
        })
    }

    fn write_all(&self, dir: &Path) -> Result<Saved, SaveError> {
        let source = self.source();
        let align = self.align();
        let points = self.output_points();
        let grid = self.output_grid();
        let placed = placed(source.ndt_yaml.get("placed"), &align);
        let edited = self.edited_block();

        let pcd = dir.join("ndt.pcd");
        let file = File::create(&pcd).map_err(|e| SaveError::Io(pcd.clone(), e))?;
        ndt_core::pcd::write(BufWriter::new(file), &points).map_err(|e| SaveError::Io(pcd, e))?;
        let mut ndt = source.ndt_yaml.clone();
        ndt.insert("points".into(), points.len().into());
        if let Some(b) = bounds_m(&points) {
            ndt.insert("bounds_m".into(), b);
        }
        if let Some(p) = &placed {
            ndt.insert("placed".into(), p.clone());
        }
        ndt.insert("edited".into(), edited.clone());
        write_yaml(
            &dir.join("ndt.yaml"),
            "# NDT's map, edited in the map editor (pixi run map-edit)\n",
            &ndt,
        )?;

        write_grid(dir, &grid, &source.grid_yaml, placed.as_ref(), &edited)?;

        if !source.path.is_empty() {
            let text: String = source
                .path
                .iter()
                .map(|p| p.aligned(&align).line() + "\n")
                .collect();
            write_file(&dir.join("poses.tum"), text.as_bytes())?;
        }
        for name in COPIED {
            let from = source.dir.join(name);
            if from.exists() {
                let to = dir.join(name);
                fs::copy(&from, &to).map_err(|e| SaveError::Io(to, e))?;
            }
        }
        let labels = serde_yaml_ng::to_string(&LabelsFile::new(self.output_labels()))
            .map_err(|e| SaveError::Yaml(e.to_string()))?;
        write_file(
            &dir.join("labels.yaml"),
            format!("# What is where, in the map frame (docs/map-editor.md)\n{labels}").as_bytes(),
        )?;
        let log = EditLog {
            source: source.dir.display().to_string(),
            saved_unix_s: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_or(0, |d| d.as_secs()),
            align,
            points_before: source.points.len(),
            points_after: points.len(),
            edits: self
                .edits()
                .iter()
                .zip(self.effects())
                .map(|(edit, effect)| Record {
                    edit: edit.clone(),
                    effect: *effect,
                })
                .collect(),
        };
        let log = serde_yaml_ng::to_string(&log).map_err(|e| SaveError::Yaml(e.to_string()))?;
        write_file(
            &dir.join("edits.yaml"),
            format!(
                "# Every edit, in order. Boxes are in the source map's frame, before `align`.\n{log}"
            )
            .as_bytes(),
        )?;

        let chunks = write_chunks(dir, &grid, &points)?;
        Ok(Saved {
            dir: dir.to_path_buf(),
            points: points.len(),
            points_removed: self.points_removed(),
            chunks,
        })
    }

    fn edited_block(&self) -> Value {
        let a = self.align();
        let mut m = Mapping::new();
        m.insert(
            "from".into(),
            self.source().dir.display().to_string().into(),
        );
        m.insert("edits".into(), self.edits().len().into());
        m.insert("points_removed".into(), self.points_removed().into());
        m.insert("aligned_by_hand".into(), align_block(&a));
        m.insert("log".into(), "edits.yaml".into());
        Value::Mapping(m)
    }
}

/// An alignment in the `placed:` block's style: `x_m`, `y_m`, `yaw_deg`.
fn align_block(a: &Align) -> Value {
    let mut m = Mapping::new();
    m.insert("x_m".into(), round(a.x_m, 4).into());
    m.insert("y_m".into(), round(a.y_m, 4).into());
    m.insert("yaw_deg".into(), round(a.yaw_rad.to_degrees(), 4).into());
    Value::Mapping(m)
}

fn round(v: f64, places: i32) -> f64 {
    let k = 10f64.powi(places);
    (v * k).round() / k
}

/// The source's `placed:` block after a hand move: `aligned_by_hand` added, and
/// `slam_to_map` (SLAM's frame to the map) followed by the move.
fn placed(source: Option<&Value>, align: &Align) -> Option<Value> {
    let mut placed = source?.as_mapping()?.clone();
    if align.is_identity() {
        return Some(Value::Mapping(placed));
    }
    if let Some(s2m) = placed.get("slam_to_map").and_then(Value::as_mapping) {
        let get = |k: &str| s2m.get(k).and_then(Value::as_f64).unwrap_or(0.0);
        let before = Align::new(get("x_m"), get("y_m"), get("yaw_deg").to_radians());
        placed.insert("slam_to_map".into(), align_block(&before.then(align)));
    }
    placed.insert("aligned_by_hand".into(), align_block(align));
    Some(Value::Mapping(placed))
}

fn bounds_m(points: &[Point3]) -> Option<Value> {
    let first = points.first()?;
    let (lo, hi) = points.iter().fold((*first, *first), |(lo, hi), p| {
        (
            Point3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z)),
            Point3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z)),
        )
    });
    let seq = |p: Point3| Value::Sequence([p.x, p.y, p.z].map(|v| round(v, 2).into()).to_vec());
    let mut m = Mapping::new();
    m.insert("min".into(), seq(lo));
    m.insert("max".into(), seq(hi));
    Some(Value::Mapping(m))
}

fn write_grid(
    dir: &Path,
    grid: &Grid2d,
    source: &Mapping,
    placed: Option<&Value>,
    edited: &Value,
) -> Result<(), SaveError> {
    let mut pgm = format!("P5\n{} {}\n255\n", grid.width, grid.height).into_bytes();
    pgm.extend(grid.pgm_pixels());
    write_file(&dir.join("map2d.pgm"), &pgm)?;
    let mut yaml = source.clone();
    let threshold =
        |k: &str, default: f64| source.get(k).and_then(Value::as_f64).unwrap_or(default);
    yaml.insert("image".into(), "map2d.pgm".into());
    yaml.insert("mode".into(), "trinary".into());
    yaml.insert("resolution".into(), grid.resolution.into());
    yaml.insert(
        "origin".into(),
        Value::Sequence(vec![grid.origin.x.into(), grid.origin.y.into(), 0.0.into()]),
    );
    yaml.insert("negate".into(), 0.into());
    yaml.insert(
        "occupied_thresh".into(),
        threshold("occupied_thresh", 0.65).into(),
    );
    yaml.insert("free_thresh".into(), threshold("free_thresh", 0.25).into());
    if let Some(p) = placed {
        yaml.insert("placed".into(), p.clone());
    }
    yaml.insert("edited".into(), edited.clone());
    write_yaml(
        &dir.join("map2d.yaml"),
        "# The 2D map, edited in the map editor (pixi run map-edit): a map server's trinary map\n",
        &yaml,
    )
}

/// The chunks, as `map-drive` cuts them (chunk-map, ndt-chunks): on the 2D map's origin.
fn write_chunks(dir: &Path, grid: &Grid2d, points: &[Point3]) -> Result<[usize; 3], SaveError> {
    let chunk_grid =
        Grid::new(grid.origin, CHUNK_SIZE).map_err(|e| SaveError::Chunks(e.to_string()))?;
    let data: Vec<i8> = grid.cells.iter().map(|c| c.to_ros()).collect();
    let whole = WholeMap {
        origin: Point::new(grid.origin.x, grid.origin.y),
        resolution: grid.resolution,
        width: grid.width,
        height: grid.height,
        data: &data,
    };
    let folder = dir.join("chunks");
    let (fine, coarse) = map_chunks::files::write_chunks(&chunk_grid, &whole, &folder)
        .map_err(|e| SaveError::Chunks(e.to_string()))?;
    let ndt = ndt_core::chunks::write_chunks(&folder.join("ndt"), &chunk_grid, points)
        .map_err(|e| SaveError::Chunks(e.to_string()))?;
    Ok([fine, coarse, ndt])
}

fn write_yaml(path: &Path, header: &str, yaml: &Mapping) -> Result<(), SaveError> {
    let text = serde_yaml_ng::to_string(yaml).map_err(|e| SaveError::Yaml(e.to_string()))?;
    write_file(path, format!("{header}{text}").as_bytes())
}

fn write_file(path: &Path, bytes: &[u8]) -> Result<(), SaveError> {
    fs::write(path, bytes).map_err(|e| SaveError::Io(path.to_path_buf(), e))
}
