//! A two-lane road, both lanes east, split at x = 20, a stop sign at x = 40 on the right lane:
//!
//! ```text
//!  y = 6  7 ────────── 8 ────────── 9      top: solid
//!         │ 201 (L1)   │ 202 (L2)   │
//!  y = 3  4 - - - - -  5 - - - - -  6      middle: dashed (or solid, `solid_middle`)
//!         │ 101 (R1)   │ 102 (R2)   ┃ stop line 30, sign 31 (de206)
//!  y = 0  1 ────────── 2 ────────── 3      bottom: solid
//!        x=0          x=20         x=40
//! ```

use super::*;

fn road(middle: &str) -> String {
    let node = |id: i64, x: f64, y: f64| {
        format!(
            r#"<node id="{id}" lat="0" lon="0"><tag k="local_x" v="{x}"/><tag k="local_y" v="{y}"/></node>"#
        )
    };
    let way = |id: i64, nodes: &[i64], kind: &str, subtype: &str| {
        let nds: String = nodes
            .iter()
            .map(|n| format!(r#"<nd ref="{n}"/>"#))
            .collect();
        format!(
            r#"<way id="{id}">{nds}<tag k="type" v="{kind}"/><tag k="subtype" v="{subtype}"/></way>"#
        )
    };
    let lanelet = |id: i64, left: i64, right: i64, rules: &[i64]| {
        let reg: String = rules
            .iter()
            .map(|r| format!(r#"<member type="relation" role="regulatory_element" ref="{r}"/>"#))
            .collect();
        format!(
            r#"<relation id="{id}"><member type="way" role="left" ref="{left}"/><member type="way" role="right" ref="{right}"/>{reg}<tag k="type" v="lanelet"/><tag k="speed_limit" v="18"/></relation>"#
        )
    };
    let mut osm = String::from("<osm>");
    for (id, x, y) in [
        (1, 0.0, 0.0),
        (2, 20.0, 0.0),
        (3, 40.0, 0.0),
        (4, 0.0, 3.0),
        (5, 20.0, 3.0),
        (6, 40.0, 3.0),
        (7, 0.0, 6.0),
        (8, 20.0, 6.0),
        (9, 40.0, 6.0),
        (11, 41.0, -1.0),
        (12, 41.0, -2.0),
    ] {
        osm += &node(id, x, y);
    }
    osm += &way(21, &[1, 2], "line_thin", "solid");
    osm += &way(22, &[2, 3], "line_thin", "solid");
    osm += &way(23, &[4, 5], "line_thin", middle);
    osm += &way(24, &[5, 6], "line_thin", middle);
    osm += &way(25, &[7, 8], "line_thin", "solid");
    osm += &way(26, &[8, 9], "line_thin", "solid");
    osm += &way(30, &[3, 6], "stop_line", "");
    osm += &way(31, &[11, 12], "traffic_sign", "de206");
    osm += &lanelet(101, 23, 21, &[]);
    osm += &lanelet(102, 24, 22, &[300]);
    osm += &lanelet(201, 25, 23, &[]);
    osm += &lanelet(202, 26, 24, &[]);
    osm += r#"<relation id="300"><member type="way" role="refers" ref="31"/><member type="way" role="ref_line" ref="30"/><tag k="type" v="regulatory_element"/><tag k="subtype" v="traffic_sign"/></relation>"#;
    osm + "</osm>"
}

fn map(middle: &str) -> LaneletMap {
    LaneletMap::from_osm(&road(middle), Origin::default()).unwrap()
}

#[test]
fn the_parts_are_read() {
    let m = map("dashed");
    assert_eq!(m.lanelets().count(), 4);
    let r1 = m.lanelet(101).unwrap();
    assert_eq!((r1.left, r1.right), (23, 21));
    assert!((r1.speed_limit.unwrap() - 5.0).abs() < 1e-9); // 18 km/h
    assert_eq!(m.line(23).unwrap().kind, LineKind::Paint { dashed: true });
    assert_eq!(m.line(30).unwrap().kind, LineKind::StopLine);
    assert_eq!(m.rule(300).unwrap().kind, RuleKind::Sign("de206".into()));
    assert!(m.rule(300).unwrap().is_stop());
}

#[test]
fn the_centre_line_runs_down_the_middle_of_the_lane() {
    let m = map("dashed");
    let r1 = m.lanelet(101).unwrap();
    assert_eq!(r1.centre.first(), Some(&Point::new(0.0, 1.5)));
    assert_eq!(r1.centre.last(), Some(&Point::new(20.0, 1.5)));
    assert!(r1.centre.iter().all(|p| (p.y - 1.5).abs() < 1e-9));
    assert!((r1.length() - 20.0).abs() < 1e-9);
}

#[test]
fn lanes_follow_and_sit_beside_each_other() {
    let m = map("dashed");
    assert_eq!(m.following(101), &[102]);
    assert_eq!(m.following(201), &[202]);
    assert!(m.following(102).is_empty());
    assert_eq!(m.beside(101), &[201]);
    assert_eq!(m.beside(202), &[102]);
}

#[test]
fn a_solid_line_is_never_crossed() {
    let m = map("solid");
    assert!(m.beside(101).is_empty());
    assert_eq!(m.route(101, 102), Some(vec![101, 102]));
    assert_eq!(m.route(101, 202), None);
}

#[test]
fn a_route_changes_lanes_over_a_dashed_line() {
    let m = map("dashed");
    let route = m.route(101, 202).unwrap();
    assert_eq!(route.len(), 3, "{route:?}");
    assert_eq!((route[0], route[2]), (101, 202));
    assert_eq!(m.route(102, 101), None); // never backwards
}

#[test]
fn the_stop_line_on_the_way_is_found() {
    let m = map("dashed");
    assert_eq!(
        m.stops_on(&[101, 102]),
        vec![(102, 300, Point::new(40.0, 1.5))]
    );
    assert!(m.stops_on(&[201, 202]).is_empty());
}

#[test]
fn the_nearest_lane_is_the_one_you_are_in() {
    let m = map("dashed");
    assert_eq!(m.nearest(10.0, 4.5), Some(201));
    assert_eq!(m.nearest(30.0, 1.0), Some(102));
}

#[test]
fn lat_lon_points_go_on_the_map_round_the_origin() {
    let osm = r#"<osm>
        <node id="1" lat="45.386" lon="-75.696"/>
        <node id="2" lat="45.387" lon="-75.695"/>
    </osm>"#;
    let origin = Origin {
        lat: 45.386,
        lon: -75.696,
    };
    let m = LaneletMap::from_osm(osm, origin).unwrap();
    assert_eq!(m.lanelets().count(), 0);
    // As tools/campus_world.py's geodetic_to_enu puts it: 78.316 m east, 111.139 m north.
    let p = origin.project(45.387, -75.695);
    assert!((p.x - 78.3156).abs() < 0.01, "{p:?}");
    assert!((p.y - 111.1393).abs() < 0.01, "{p:?}");
}

#[test]
fn a_broken_file_says_what_is_wrong() {
    let bad = r#"<osm><way id="5"><nd ref="99"/></way></osm>"#;
    assert_eq!(
        LaneletMap::from_osm(bad, Origin::default()).unwrap_err(),
        MapError::Unknown {
            element: "way",
            id: 5,
            refers: 99
        }
    );
    assert!(matches!(
        LaneletMap::from_osm("<osm", Origin::default()),
        Err(MapError::Xml(_))
    ));
}
