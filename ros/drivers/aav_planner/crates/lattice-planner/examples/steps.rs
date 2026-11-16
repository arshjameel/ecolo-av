//! Far goals in steps on the campus's chunks, no ROS (TODO 175): each leg of the campus
//! and University Drive tours planned on our global costmap made of the chunks round the
//! car and along the route (as `costmap_chunks:=auto` holds them), the car put at a point
//! 30 m along each plan, again until it is there. Beside it, the whole map's plan.
//!
//! ```sh
//! pixi run python3 -c "import json; from aav_description import lane_map; \
//!   print(json.dumps(lane_map.costmap_block(lane_map.load('maps/campus/lanes.yaml'))))" \
//!   > /tmp/campus_lanes.json
//! cargo run --release -p lattice-planner --example steps -- /tmp/campus_lanes.json \
//!   ../../../maps/campus/chunks
//! ```

use std::path::Path;

use costmap::{footprint, ChunkSource, Costmap, Pose2, Settings};
use lattice_planner::lane::{roads_from_json, How, LaneRouter, LaneSettings};
use lattice_planner::{length, scenes, Pose};

const LEGS: [(&str, f64, f64, f64); 6] = [
    ("campus: out of the lot", 73.0, -38.0, 45.0),
    ("campus: along the road", 165.0, -111.0, -40.0),
    ("campus: round the bend", 139.0, -190.0, 109.0),
    ("university: onto University Drive", 87.7, -72.6, -67.4),
    ("university: down University Drive", 139.9, -196.4, -67.0),
    ("university: round the bend south", 169.0, -337.4, -129.0),
];
/// m the pretend car drives along each plan before the next.
const DRIVE: f64 = 30.0;

fn settings() -> Settings {
    let outline = footprint::parse("[[1.8, 0.5], [1.8, -0.5], [-0.2, -0.5], [-0.2, 0.5]]");
    Settings {
        rolling: false,
        width: 0.0,
        height: 0.0,
        resolution: 0.25,
        track_unknown: true,
        footprint: footprint::padded(&outline.expect("an outline"), 0.05),
        inflation: Some((3.0, 1.0)),
        static_layer: true,
    }
}

/// The pose `metres` along `poses` (the last if shorter).
fn along(poses: &[Pose], metres: f64) -> Pose {
    let mut left = metres;
    for w in poses.windows(2) {
        let d = (w[1].x - w[0].x).hypot(w[1].y - w[0].y);
        if d >= left {
            return w[1];
        }
        left -= d;
    }
    poses[poses.len() - 1]
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [lanes, chunks] = args.as_slice() else {
        return Err("usage: steps <lanes block .json> <chunks folder>".into());
    };
    let text = std::fs::read_to_string(lanes).map_err(|e| format!("{lanes}: {e}"))?;
    let block: serde_json::Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    let router = LaneRouter::new(roads_from_json(&block)?, LaneSettings::default());
    let source = ChunkSource::open(Path::new(chunks), 100.0, 1).map_err(|e| e.to_string())?;
    let planner = scenes::planner(true);
    let never = || false;
    let mut costmap = Costmap::new(settings(), Vec::new());
    costmap.set_chunks(source).map_err(|e| e.to_string())?;
    let mut from = scenes::CAMPUS_START;
    for (i, (name, x, y, deg)) in LEGS.iter().enumerate() {
        if i == 3 {
            from = scenes::CAMPUS_START;
        }
        let goal = Pose::new(*x, *y, deg.to_radians());
        println!("{name}");
        let mut car = from;
        let mut route: Vec<(f64, f64)> = Vec::new();
        for step in 0..40 {
            costmap.set_ahead(&route);
            costmap
                .follow_chunks((car.x, car.y))
                .map_err(|e| e.to_string())?;
            costmap.update(&Pose2::new(car.x, car.y, car.yaw), None, &[]);
            let (w, n) = costmap.chunk_window().ok_or("no window")?;
            match router.plan(&planner, costmap.grid(), &car, &goal, &never) {
                Ok(p) => {
                    let how = match p.how {
                        How::Lane(m) => format!("lane {m:.0} m"),
                        How::Pass(m, d) => format!("lane {m:.0} m, passing {d:.2} m left"),
                        How::Lattice(why) => format!("lattice ({})", why.word()),
                    };
                    let end = p.plan.poses[p.plan.poses.len() - 1];
                    println!(
                        "  step {step}: car ({:.0}, {:.0}), {n} chunks from ({}, {}); {how}, \
                         {:.0} m{}, {:.0} ms; route {:.0} m",
                        car.x,
                        car.y,
                        w.i0,
                        w.j0,
                        length(&p.plan.poses),
                        if p.cut { ", cut" } else { "" },
                        p.plan.total_time * 1e3,
                        length(&p.route)
                    );
                    route = p.route.iter().map(|q| (q.x, q.y)).collect();
                    if !p.cut {
                        break;
                    }
                    car = along(&p.plan.poses, DRIVE);
                    let _ = end;
                }
                Err(e) => {
                    println!(
                        "  step {step}: car ({:.0}, {:.0}): no path: {e}",
                        car.x, car.y
                    );
                    break;
                }
            }
        }
        from = goal;
    }
    Ok(())
}
