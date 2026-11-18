//! The diffusion planner's teachers (`pixi run diffusion data`, `tools/diffusion_data.py`):
//! our global costmap and our planner as the stack runs them, without ROS. It replaced
//! Nav2's map_server and planner_server there when Nav2 went (17 Nov, TODO 242).
//!
//! `cargo run --release -p lattice-planner --example teach`
//!
//! One JSON request a line on stdin, one JSON answer a line on stdout:
//!
//! | Request | Answer |
//! |---|---|
//! | `{"teachers": {name: settings}}`: each planner's block, as `vehicle.planner_params` writes it | `{"teachers": [names]}` |
//! | `{"map": "<map .yaml>", "costmap": block}`: the global block of `vehicle.costmap_params` | `{"width", "height", "origin": [x, y], "resolution", "costmap": [...]}`: the costmap, row by row from the bottom, in OccupancyGrid's units (0-100, -1 unknown), as `/drivers/nav/costmap/global` sends it |
//! | `{"plan": name, "start": [x, y, yaw], "goal": [x, y, yaw]}` on that map | `{"poses": [[x, y, yaw], ...], "ms": the plan's wall time}` |
//!
//! Anything wrong: `{"error": "..."}`, and it goes on to the next line.
//!
//! The map is read as our map server reads it (`map-chunks`' `read_map`), then made into the
//! global costmap by our costmap's own update (the static layer and the inflation; no
//! sensors), and each plan is our planner node's (`planner_from_json`, `steps::plan_toward`).

use std::collections::BTreeMap;
use std::io::{BufRead, Write};
use std::path::Path;

use costmap::cost::occupancy;
use costmap::{footprint, Costmap, Grid, Pose2, Settings, StaticMap};
use lattice_planner::{planner_from_json, steps, Planner, Pose};
use serde_json::{json, Value};

/// What a line asks.
enum Request {
    Teachers(BTreeMap<String, Value>),
    Map {
        yaml: String,
        block: Value,
    },
    Plan {
        teacher: String,
        start: Pose,
        goal: Pose,
    },
}

fn parse(line: &str) -> Result<Request, String> {
    let v: Value = serde_json::from_str(line).map_err(|e| format!("not JSON: {e}"))?;
    if let Some(t) = v["teachers"].as_object() {
        return Ok(Request::Teachers(
            t.iter().map(|(k, b)| (k.clone(), b.clone())).collect(),
        ));
    }
    if let Some(yaml) = v["map"].as_str() {
        return Ok(Request::Map {
            yaml: yaml.to_owned(),
            block: v["costmap"].clone(),
        });
    }
    if let Some(teacher) = v["plan"].as_str() {
        return Ok(Request::Plan {
            teacher: teacher.to_owned(),
            start: pose(&v["start"]).ok_or("start: [x, y, yaw]")?,
            goal: pose(&v["goal"]).ok_or("goal: [x, y, yaw]")?,
        });
    }
    Err("a line asks for teachers, map or plan".into())
}

fn pose(v: &Value) -> Option<Pose> {
    Some(Pose::new(v[0].as_f64()?, v[1].as_f64()?, v[2].as_f64()?))
}

/// The global costmap's settings from vehicle.costmap_params' block (the costmap node's
/// own reading of it, less the sensor layers: there are no sensors here).
fn costmap_settings(b: &Value) -> Result<Settings, String> {
    let num = |k: &str| b[k].as_f64().ok_or(format!("costmap.{k}: a number"));
    let flag = |k: &str| b[k].as_bool().ok_or(format!("costmap.{k}: true or false"));
    let outline: Vec<(f64, f64)> = b["footprint"]
        .as_array()
        .and_then(|c| {
            c.iter()
                .map(|p| Some((p[0].as_f64()?, p[1].as_f64()?)))
                .collect()
        })
        .ok_or("costmap.footprint: a list of [x, y]")?;
    let inflation = match &b["inflation"] {
        Value::Null => None,
        i => Some((
            i["radius"].as_f64().ok_or("costmap.inflation.radius")?,
            i["scaling"].as_f64().ok_or("costmap.inflation.scaling")?,
        )),
    };
    Ok(Settings {
        rolling: flag("rolling")?,
        width: num("width")?,
        height: num("height")?,
        resolution: num("resolution")?,
        track_unknown: flag("track_unknown")?,
        footprint: footprint::padded(&outline, num("footprint_padding")?),
        inflation,
        static_layer: flag("static_layer")?,
    })
}

/// The map file as the global costmap makes it.
fn global_costmap(yaml: &str, block: &Value) -> Result<Grid, String> {
    let settings = costmap_settings(block)?;
    if settings.rolling {
        return Err("costmap: the global block, not a rolling one".into());
    }
    let map = map_chunks::files::read_map(Path::new(yaml)).map_err(|e| format!("{yaml}: {e}"))?;
    let [x, y, _] = map.yaml.origin;
    let track = settings.track_unknown;
    let static_map = StaticMap::from_occupancy(
        (x, y),
        map.yaml.resolution,
        map.width,
        map.height,
        &map.data,
        track,
    )
    .ok_or(format!("{yaml}: its size and data disagree"))?;
    let mut costmap = Costmap::new(settings, Vec::new());
    costmap.set_map(static_map);
    // The global costmap is in map: the map's own frame.
    let here = Pose2::new(0.0, 0.0, 0.0);
    let _ = costmap.update(&here, Some(&here), &[]);
    Ok(costmap.grid().clone())
}

struct Teacher {
    teachers: BTreeMap<String, Planner>,
    grid: Option<Grid>,
}

impl Teacher {
    fn answer(&mut self, request: Request) -> Result<Value, String> {
        match request {
            Request::Teachers(blocks) => {
                self.teachers = blocks
                    .into_iter()
                    .map(|(name, b)| {
                        planner_from_json(&b)
                            .map(|p| (name.clone(), p))
                            .map_err(|e| format!("{name}: {e}"))
                    })
                    .collect::<Result<_, _>>()?;
                Ok(json!({ "teachers": self.teachers.keys().collect::<Vec<_>>() }))
            }
            Request::Map { yaml, block } => {
                let g = global_costmap(&yaml, &block)?;
                let cells: Vec<i8> = g.data.iter().map(|&c| occupancy(c)).collect();
                let answer = json!({
                    "width": g.width, "height": g.height,
                    "origin": [g.origin_x, g.origin_y], "resolution": g.resolution,
                    "costmap": cells,
                });
                self.grid = Some(g);
                Ok(answer)
            }
            Request::Plan {
                teacher,
                start,
                goal,
            } => {
                let planner = self
                    .teachers
                    .get(&teacher)
                    .ok_or(format!("no teacher {teacher}"))?;
                let grid = self.grid.as_ref().ok_or("no map yet")?;
                let (plan, cut) = steps::plan_toward(planner, grid, &start, &goal, &|| false)
                    .map_err(|e| e.to_string())?;
                if cut {
                    return Err("the goal is off the costmap".into());
                }
                let poses: Vec<[f64; 3]> = plan.poses.iter().map(|p| [p.x, p.y, p.yaw]).collect();
                Ok(json!({ "poses": poses, "ms": plan.total_time * 1e3 }))
            }
        }
    }
}

fn main() -> std::io::Result<()> {
    let mut teacher = Teacher {
        teachers: BTreeMap::new(),
        grid: None,
    };
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    for line in std::io::stdin().lock().lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let answer = parse(&line)
            .and_then(|r| teacher.answer(r))
            .unwrap_or_else(|e| json!({ "error": e }));
        writeln!(out, "{answer}")?;
        out.flush()?;
    }
    Ok(())
}
