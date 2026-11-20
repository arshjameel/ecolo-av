//! How long the things take (TODO 195): one cloud through the node's pipeline, and one
//! read of the NDT chunks' 3 × 3 background.
//!
//! ```sh
//! cd ros/perception/aav_things_rs
//! cargo run --release -p things-core --example timing -- \
//!     [maps/campus/chunks/ndt [maps/campus/campus.yaml]]
//! ```
//!
//! The clouds are `tests/vectors.json`'s runs (made up and recorded in Ravensim). With a
//! chunk folder, every 8th chunk with a file is read as the car crossing into it.

use std::{error::Error, fs, path::Path, time::Instant};

use serde_json::Value;
use things_core::{
    background::{new_only, Background},
    group, to_frame, ChunkBackground, GroupShape, Pose2, Tracker,
};

const RUNS: u32 = 50;

fn numbers(v: &Value) -> Vec<f64> {
    v.as_array()
        .map(|a| a.iter().map(|x| x.as_f64().unwrap_or(f64::NAN)).collect())
        .unwrap_or_default()
}

fn pose(v: &Value) -> Pose2 {
    let n = numbers(v);
    Pose2 {
        x: n[0],
        y: n[1],
        yaw: n[2],
    }
}

fn background(spec: &Value) -> Option<Background> {
    let (w, h) = (
        spec["width"].as_u64()? as usize,
        spec["height"].as_u64()? as usize,
    );
    let mut data = vec![0i8; w * h];
    for i in spec["walls"].as_array()? {
        data[i.as_u64()? as usize] = 100;
    }
    let origin = numbers(&spec["origin"]);
    Background::from_grid(
        &data,
        w,
        h,
        spec["resolution"].as_f64()?,
        [origin[0], origin[1]],
        spec["slack"].as_f64()?,
    )
}

/// One cloud: its cubes, the car in odom, the car in map, its stamp.
type Frame = (Vec<[f64; 3]>, Pose2, Option<Pose2>, f64);

fn clouds() -> Result<(), Box<dyn Error>> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/vectors.json");
    let all: Value = serde_json::from_str(&fs::read_to_string(path)?)?;
    for run in all["runs"].as_array().ok_or("runs")? {
        let bg = run.get("map").filter(|m| !m.is_null()).and_then(background);
        let below = run.get("below").and_then(Value::as_f64);
        let frames: Vec<Frame> = run["frames"]
            .as_array()
            .ok_or("frames")?
            .iter()
            .map(|f| {
                let cubes = numbers(&f["cubes"]).as_chunks::<3>().0.to_vec();
                let map_pose = f.get("map_pose").filter(|p| !p.is_null()).map(pose);
                (
                    cubes,
                    pose(&f["pose"]),
                    map_pose,
                    f["t"].as_f64().unwrap_or(0.0),
                )
            })
            .collect();
        let shape = GroupShape::default();
        let (mut all_ms, mut group_ms) = (0.0, 0.0);
        for _ in 0..RUNS {
            let mut tracker = Tracker::default();
            for (cubes, odom, map_pose, t) in &frames {
                let start = Instant::now();
                let points = match (&bg, map_pose) {
                    (Some(bg), Some(p)) => new_only(cubes, *p, bg, below),
                    _ => cubes.clone(),
                };
                let grouped = Instant::now();
                let groups = group(&points, &shape);
                group_ms += grouped.elapsed().as_secs_f64() * 1e3;
                let groups = to_frame(&groups, *odom);
                tracker.step(*t, &groups, [odom.x, odom.y]);
                std::hint::black_box(tracker.at(*t));
                all_ms += start.elapsed().as_secs_f64() * 1e3;
            }
        }
        let n = f64::from(RUNS) * frames.len() as f64;
        let cubes = frames.iter().map(|f| f.0.len()).max().unwrap_or(0);
        println!(
            "{}: {} clouds, up to {cubes} cubes: {:.4} ms a cloud ({:.4} ms grouping), mean of {RUNS}",
            run["name"].as_str().unwrap_or("?"),
            frames.len(),
            all_ms / n,
            group_ms / n,
        );
    }
    Ok(())
}

fn chunks(folder: &Path) -> Result<(), Box<dyn Error>> {
    let parent = folder.canonicalize()?;
    let grid = map_chunks::files::read_grid(parent.parent().ok_or("no parent")?)?;
    let mut keys: Vec<(i32, i32)> = fs::read_dir(folder)?
        .filter_map(|e| {
            let name = e.ok()?.file_name().into_string().ok()?;
            let (i, j) = name.strip_suffix(".pcd")?.split_once('_')?;
            Some((i.parse().ok()?, j.parse().ok()?))
        })
        .collect();
    keys.sort_unstable();
    let picked: Vec<_> = keys.iter().step_by(8).collect();
    let mut ms = Vec::new();
    for &&(i, j) in &picked {
        let corner = grid.corner(map_chunks::ChunkKey::new(i, j));
        let half = grid.size() / 2.0;
        let mut bg = ChunkBackground::open(folder, 0.25, 0.3)?;
        let start = Instant::now();
        std::hint::black_box(bg.update(corner.x + half, corner.y + half)?);
        ms.push(start.elapsed().as_secs_f64() * 1e3);
    }
    let mean = ms.iter().sum::<f64>() / ms.len().max(1) as f64;
    let most = ms.iter().copied().fold(0.0, f64::max);
    println!(
        "{}: {} chunk crossings: {mean:.2} ms a 3 x 3 read (most {most:.2} ms)",
        folder.display(),
        ms.len()
    );
    Ok(())
}

/// The whole `/map` as the background, as the node makes it when `/map` comes.
fn whole_map(yaml: &Path) -> Result<(), Box<dyn Error>> {
    let map = map_chunks::files::read_map(yaml)?;
    let origin = [map.yaml.origin[0], map.yaml.origin[1]];
    let start = Instant::now();
    let runs = 5;
    for _ in 0..runs {
        std::hint::black_box(Background::from_grid(
            &map.data,
            map.width,
            map.height,
            map.yaml.resolution,
            origin,
            0.25,
        ));
    }
    println!(
        "{}: {} x {} cells: {:.1} ms to grow its walls, mean of {runs}",
        yaml.display(),
        map.width,
        map.height,
        start.elapsed().as_secs_f64() * 1e3 / f64::from(runs)
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    clouds()?;
    let mut args = std::env::args().skip(1);
    if let Some(folder) = args.next() {
        // Twice: the first warms the disk cache.
        chunks(Path::new(&folder))?;
        chunks(Path::new(&folder))?;
    }
    if let Some(yaml) = args.next() {
        whole_map(Path::new(&yaml))?;
    }
    Ok(())
}
