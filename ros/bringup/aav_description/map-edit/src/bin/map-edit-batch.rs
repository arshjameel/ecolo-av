//! map-edit-batch: the editor's edits from the command line, with no window, for scripts
//! and the proof (`docs/map-editor.md`). The same `Editor` and save as the window.
//!
//! ```text
//! cargo run --release --bin map-edit-batch -- <map folder> [--out DIR]
//!     [--align X,Y,YAW_DEG] [--delete X0,Y0,X1,Y1] [--label CLASS:X0,Y0,X1,Y1]
//!     [--band LOW,HIGH] [--cells unknown|free|keep]
//! ```
//!
//! Boxes are corners in the source map's frame (m). `--band` and `--cells` apply to every
//! `--delete`. Edits run in the order given. With no `--out`: `<map>-edit-<n>` beside it.

use std::path::PathBuf;
use std::process::ExitCode;

use map_edit::{
    next_free, Align, CellFill, Edit, Editor, HeightBand, LabelClass, MapFolder, Point, Region,
};

const USAGE: &str = "usage: map-edit-batch <map folder> [--out DIR] [--align X,Y,YAW_DEG] \
                     [--delete X0,Y0,X1,Y1] [--label CLASS:X0,Y0,X1,Y1] [--band LOW,HIGH] \
                     [--cells unknown|free|keep]";

fn numbers(text: &str, n: usize) -> Result<Vec<f64>, String> {
    let v: Vec<f64> = text
        .split(',')
        .map(|s| s.trim().parse::<f64>())
        .collect::<Result<_, _>>()
        .map_err(|_| format!("{text}: not numbers"))?;
    if v.len() == n {
        Ok(v)
    } else {
        Err(format!("{text}: {n} numbers, comma between"))
    }
}

fn region(text: &str) -> Result<Region, String> {
    let v = numbers(text, 4)?;
    Ok(Region::from_corners(
        Point::new(v[0], v[1]),
        Point::new(v[2], v[3]),
    ))
}

fn class(name: &str) -> Result<LabelClass, String> {
    LabelClass::ALL
        .into_iter()
        .find(|c| c.name().replace(' ', "_") == name)
        .ok_or_else(|| format!("{name}: one of tree, building, parked_car, pole, other"))
}

fn run(args: Vec<String>) -> Result<(), String> {
    let mut args = args.into_iter();
    let source = PathBuf::from(args.next().ok_or(USAGE)?);
    let mut out = None;
    let mut band = None;
    let mut cells = CellFill::Unknown;
    // Boxes wait for --band and --cells, wherever they are given.
    enum Step {
        Align(Align),
        Delete(Region),
        Label(LabelClass, Region),
    }
    let mut steps = Vec::new();
    while let Some(arg) = args.next() {
        let value = args
            .next()
            .ok_or_else(|| format!("{arg} needs a value\n{USAGE}"))?;
        match arg.as_str() {
            "--out" => out = Some(PathBuf::from(value)),
            "--align" => {
                let v = numbers(&value, 3)?;
                steps.push(Step::Align(Align::new(v[0], v[1], v[2].to_radians())));
            }
            "--delete" => steps.push(Step::Delete(region(&value)?)),
            "--label" => {
                let (name, corners) = value
                    .split_once(':')
                    .ok_or_else(|| format!("--label {value}: CLASS:X0,Y0,X1,Y1"))?;
                steps.push(Step::Label(class(name)?, region(corners)?));
            }
            "--band" => {
                let v = numbers(&value, 2)?;
                band = Some(HeightBand::new(v[0], v[1]));
            }
            "--cells" => {
                cells = match value.as_str() {
                    "unknown" => CellFill::Unknown,
                    "free" => CellFill::Free,
                    "keep" => CellFill::Keep,
                    _ => return Err(format!("--cells {value}: unknown, free or keep")),
                };
            }
            other => return Err(format!("unknown option {other}\n{USAGE}")),
        }
    }
    let map = MapFolder::load(&source).map_err(|e| e.to_string())?;
    println!(
        "{}: {} points, 2D map {} x {} cells",
        source.display(),
        map.points.len(),
        map.grid.width,
        map.grid.height
    );
    let mut editor = Editor::new(map);
    for step in steps {
        let edit = match step {
            Step::Align(to) => Edit::Align { to },
            Step::Delete(region) => Edit::Delete {
                region,
                band,
                cells,
            },
            Step::Label(class, region) => Edit::Label {
                class,
                region,
                band: None,
            },
        };
        let effect = editor.apply(edit.clone()).map_err(|e| e.to_string())?;
        println!(
            "{edit:?}: {} points removed, {} cells changed",
            effect.points_removed, effect.cells_changed
        );
    }
    let out = out.unwrap_or_else(|| next_free(&source));
    let saved = editor.save(&out).map_err(|e| e.to_string())?;
    println!(
        "saved {}: {} points ({} removed); chunks: {} fine, {} coarse, {} NDT",
        saved.dir.display(),
        saved.points,
        saved.points_removed,
        saved.chunks[0],
        saved.chunks[1],
        saved.chunks[2]
    );
    Ok(())
}

fn main() -> ExitCode {
    match run(std::env::args().skip(1).collect()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(why) => {
            eprintln!("map-edit-batch: {why}");
            ExitCode::from(2)
        }
    }
}
