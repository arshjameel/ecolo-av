//! The lane-centre planner on a Lanelet2 map (`lane_graph:=lanelet2`, TODO 175): a hand-made
//! road with a stop line, and the campus's own map against its `lanes.yaml` (when
//! `pixi run campus` has made both; skipped otherwise, as on CI).

use std::path::PathBuf;

use costmap::Grid;
use lanelet_map::{LaneletMap, Origin};
use lattice_planner::lane::{roads_from_json, How, LaneRouter, LaneSettings};
use lattice_planner::{scenes, Pose};

fn never() -> bool {
    false
}

/// Two lanelets east along y = 0 (3.5 m wide), 0-50 m and 50-100 m; a stop sign at x = 50.
fn road() -> LaneletMap {
    let node = |id: i64, x: f64, y: f64| {
        format!(
            r#"<node id="{id}" lat="0" lon="0"><tag k="local_x" v="{x}"/><tag k="local_y" v="{y}"/></node>"#
        )
    };
    let way = |id: i64, nodes: [i64; 2], kind: &str, subtype: &str| {
        format!(
            r#"<way id="{id}"><nd ref="{}"/><nd ref="{}"/><tag k="type" v="{kind}"/><tag k="subtype" v="{subtype}"/></way>"#,
            nodes[0], nodes[1]
        )
    };
    let mut osm = String::from("<osm>");
    for (id, x, y) in [
        (1, 0.0, -1.75),
        (2, 50.0, -1.75),
        (3, 100.0, -1.75),
        (4, 0.0, 1.75),
        (5, 50.0, 1.75),
        (6, 100.0, 1.75),
        (7, 50.0, -3.0),
        (8, 50.0, -3.5),
    ] {
        osm += &node(id, x, y);
    }
    osm += &way(11, [1, 2], "line_thin", "solid");
    osm += &way(12, [2, 3], "line_thin", "solid");
    osm += &way(13, [4, 5], "line_thin", "solid");
    osm += &way(14, [5, 6], "line_thin", "solid");
    osm += &way(15, [2, 5], "stop_line", "");
    osm += &way(16, [7, 8], "traffic_sign", "stop");
    osm += r#"<relation id="21"><member type="way" role="left" ref="13"/><member type="way" role="right" ref="11"/><member type="relation" role="regulatory_element" ref="30"/><tag k="type" v="lanelet"/></relation>"#;
    osm += r#"<relation id="22"><member type="way" role="left" ref="14"/><member type="way" role="right" ref="12"/><tag k="type" v="lanelet"/></relation>"#;
    osm += r#"<relation id="30"><member type="way" role="refers" ref="16"/><member type="way" role="ref_line" ref="15"/><tag k="type" v="regulatory_element"/><tag k="subtype" v="traffic_sign"/></relation>"#;
    osm += "</osm>";
    LaneletMap::from_osm(&osm, Origin::default()).expect("the road")
}

#[test]
fn along_the_lanelets_and_the_stop_line_found() {
    let router = LaneRouter::from_lanelets(road(), LaneSettings::default());
    assert_eq!(router.describe(), "2 lanelets (Lanelet2)");
    let grid = Grid::new((-20.0, -20.0), scenes::RESOLUTION, 560, 160, 0);
    let planner = scenes::planner(false);
    let start = Pose::new(2.0, 0.0, 0.0);
    let goal = Pose::new(95.0, 0.0, 0.0);
    let got = router
        .plan(&planner, &grid, &start, &goal, &never)
        .expect("a path");
    let How::Lane(metres) = got.how else {
        panic!("the lattice's: {:?}", got.how);
    };
    assert!(metres > 80.0, "{metres}");
    assert!(
        got.plan.poses.iter().all(|p| p.y.abs() < 0.05),
        "on the centre"
    );
    assert_eq!(got.stops.len(), 1, "{:?}", got.stops);
    let (x, y) = got.stops[0];
    assert!((x - 50.0).abs() < 1e-6 && y.abs() < 1e-6, "{x} {y}");
    // Against the one way: no lane, the lattice's.
    let back = router
        .plan(&planner, &grid, &goal, &start, &never)
        .expect("the lattice");
    assert!(matches!(back.how, How::Lattice(_)), "{:?}", back.how);
    assert!(back.stops.is_empty());
    // Stopping short of the line: none met.
    let short = router
        .lane_line(&start, &Pose::new(40.0, 0.0, 0.0), 5.0)
        .expect("a line");
    assert!(short.last().is_some_and(|p| (p.x - 40.0).abs() < 0.3));
}

/// The campus's legs (lattice-planner's `lane` example): each from the last goal, the
/// first and the fourth from lot P4.
const LEGS: [(f64, f64, f64); 6] = [
    (73.0, -38.0, 45.0),
    (165.0, -111.0, -40.0),
    (139.0, -190.0, 109.0),
    (87.7, -72.6, -67.4),
    (139.9, -196.4, -67.0),
    (169.0, -337.4, -129.0),
];

fn campus_files() -> Option<(PathBuf, PathBuf)> {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../../../maps/campus");
    let (osm, lanes) = (dir.join("campus.osm"), dir.join("lanes.yaml"));
    (osm.is_file() && lanes.is_file()).then_some((osm, lanes))
}

fn length(line: &[Pose]) -> f64 {
    line.windows(2)
        .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
        .sum()
}

#[test]
fn on_the_campus_lanelet2_routes_as_lanes_yaml_does() {
    let Some((osm, lanes)) = campus_files() else {
        eprintln!("no maps/campus/campus.osm and lanes.yaml (pixi run campus): skipped");
        return;
    };
    let map = LaneletMap::from_osm(
        &std::fs::read_to_string(&osm).expect("campus.osm"),
        Origin::default(),
    )
    .expect("the campus's Lanelet2 map");
    let stop_lines = map.lanelets().filter(|l| !l.rules.is_empty()).count();
    let lanelets = LaneRouter::from_lanelets(map, LaneSettings::default());
    let yaml: serde_json::Value =
        serde_yaml_ng::from_str(&std::fs::read_to_string(&lanes).expect("lanes.yaml"))
            .expect("lanes.yaml");
    let roads = LaneRouter::new(
        roads_from_json(&yaml).expect("roads"),
        LaneSettings::default(),
    );
    let radius = scenes::planner(true).turning_radius();
    let mut from = scenes::CAMPUS_START;
    for (i, &(x, y, deg)) in LEGS.iter().enumerate() {
        if i == 3 {
            from = scenes::CAMPUS_START;
        }
        let goal = Pose::new(x, y, deg.to_radians());
        let a = lanelets.lane_line(&from, &goal, radius);
        let b = roads.lane_line(&from, &goal, radius);
        let (la, lb) = (a.as_deref().map(length), b.as_deref().map(length));
        eprintln!("leg {i}: Lanelet2 {la:?} m, lanes.yaml {lb:?} m");
        // Where lanes.yaml has a way, Lanelet2 has it too (it has more roads: the service
        // roads, which lanes.yaml, the painted roads only, leaves out).
        assert!(lb.is_none() || la.is_some(), "leg {i}: Lanelet2 has no way");
        let (Some(la), Some(lb)) = (la, lb) else {
            from = goal;
            continue;
        };
        assert!(
            (la - lb).abs() <= 0.15 * lb + 15.0,
            "leg {i}: {la} against {lb}"
        );
        // The line is a lane's centre: never far from where lanes.yaml puts the right lane.
        let line = a.expect("a line");
        let theirs = b.expect("a line");
        let worst = line
            .iter()
            .map(|p| {
                theirs
                    .iter()
                    .map(|q| (p.x - q.x).hypot(p.y - q.y))
                    .fold(f64::INFINITY, f64::min)
            })
            .fold(0.0, f64::max);
        assert!(worst < 6.0, "leg {i}: {worst:.1} m from lanes.yaml's line");
        from = goal;
    }
    // The stop signs (OpenStreetMap's highway=stop), each a stop line on every lane
    // ending there: tools/campus_lanelet.py.
    eprintln!("{stop_lines} lanelets with a stop line");
    assert!(stop_lines > 50, "{stop_lines}");
}
