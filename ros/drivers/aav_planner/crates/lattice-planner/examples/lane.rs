//! The lane-centre planner (`path:=lane`) on the campus, no ROS: each leg of the campus
//! and University Drive tours, lane or lattice (and why), its time and length.
//!
//! The lanes come as the launch hands them over, a JSON lanes block:
//!
//! ```sh
//! pixi run python3 -c "import json; from aav_description import lane_map; \
//!   print(json.dumps(lane_map.costmap_block(lane_map.load('maps/campus/lanes.yaml'))))" \
//!   > /tmp/campus_lanes.json
//! cargo run --release -p lattice-planner --example lane -- /tmp/campus_lanes.json
//! ```

use lattice_planner::lane::{roads_from_json, How, LaneRouter, LaneSettings};
use lattice_planner::{length, scenes, Pose};

/// (name, x, y, heading degrees): the legs, each from the last goal (the first from P4).
const LEGS: [(&str, f64, f64, f64); 6] = [
    ("campus: out of the lot", 73.0, -38.0, 45.0),
    ("campus: along the road", 165.0, -111.0, -40.0),
    ("campus: round the bend", 139.0, -190.0, 109.0),
    ("university: onto University Drive", 87.7, -72.6, -67.4),
    ("university: down University Drive", 139.9, -196.4, -67.0),
    ("university: round the bend south", 169.0, -337.4, -129.0),
];

fn main() -> Result<(), String> {
    let path = std::env::args()
        .nth(1)
        .ok_or("usage: lane <lanes block .json>")?;
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{path}: {e}"))?;
    let block: serde_json::Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    let router = LaneRouter::new(roads_from_json(&block)?, LaneSettings::default());
    let grid = scenes::campus_area((-60.0, -420.0), (260.0, 60.0))?;
    let planner = scenes::planner(true);
    let never = || false;
    let mut from = scenes::CAMPUS_START;
    for (i, (name, x, y, deg)) in LEGS.iter().enumerate() {
        if i == 3 {
            from = scenes::CAMPUS_START;
        }
        let goal = Pose::new(*x, *y, deg.to_radians());
        match router.plan(&planner, &grid, &from, &goal, &never) {
            Ok(p) => {
                let how = match p.how {
                    How::Lane(m) => format!("lane {m:.0} m"),
                    How::Pass(m, d) => format!("lane {m:.0} m, passing {d:.2} m left"),
                    How::Lattice(why) => format!("lattice ({})", why.word()),
                };
                println!(
                    "{name:36} {how:24} {:6.1} m {:6.1} ms",
                    length(&p.plan.poses),
                    p.plan.total_time * 1e3
                );
            }
            Err(e) => println!("{name:36} no path: {e}"),
        }
        from = goal;
    }
    Ok(())
}
