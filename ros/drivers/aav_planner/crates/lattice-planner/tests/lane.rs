//! The lane-centre planner (`path:=lane`) with the real lattice for its joins: on a free
//! map with an L of road, the path follows the right lane's centre between the joins and
//! starts and ends where asked; off the lane map, blocked, or too short, the lattice's.

use costmap::cost::LETHAL;
use costmap::lanes::EDGE_INSET;
use costmap::{Grid, Road};
use lattice_planner::lane::{Fallback, How, LaneRouter, LaneSettings};
use lattice_planner::{scenes, Pose};

/// A free 140 m x 140 m map (0.25 m cells, origin (-20, -20)).
fn free() -> Grid {
    Grid::new((-20.0, -20.0), scenes::RESOLUTION, 560, 560, 0)
}

/// East along y = 0 to x = 100, then north along x = 100; two-way, 7 m.
fn ell() -> LaneRouter {
    let road = |name: &str, points: Vec<(f64, f64)>| Road {
        name: name.to_owned(),
        points,
        width: 7.0,
        lanes: 0,
        oneway: false,
        edge_inset: EDGE_INSET,
    };
    LaneRouter::new(
        vec![
            road("east", vec![(0.0, 0.0), (100.0, 0.0)]),
            road("north", vec![(100.0, 0.0), (100.0, 100.0)]),
        ],
        LaneSettings::default(),
    )
}

fn never() -> bool {
    false
}

#[test]
fn along_the_lane_between_the_joins() {
    let planner = scenes::planner(false);
    let grid = free();
    let start = Pose::new(5.0, 0.5, 0.0);
    let goal = Pose::new(101.5, 70.0, std::f64::consts::FRAC_PI_2);
    let got = ell()
        .plan(&planner, &grid, &start, &goal, &never)
        .expect("a path");
    let How::Lane(metres) = got.how else {
        panic!("the lattice's: {:?}", got.how);
    };
    assert!(metres > 100.0, "{metres}");
    let poses = &got.plan.poses;
    let first = poses[0];
    let last = poses[poses.len() - 1];
    assert!(
        (first.x - start.x).hypot(first.y - start.y) < 0.3,
        "{first:?}"
    );
    assert!((last.x - goal.x).hypot(last.y - goal.y) < 0.3, "{last:?}");
    // On the east road past the join: the right lane's centre, y = -1.5.
    let east: Vec<&Pose> = poses.iter().filter(|p| p.x > 20.0 && p.x < 90.0).collect();
    assert!(!east.is_empty());
    assert!(east.iter().all(|p| (p.y + 1.5).abs() < 0.05), "{east:?}");
    // No big jumps between poses: the joins meet the line.
    let gap = poses
        .windows(2)
        .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
        .fold(0.0, f64::max);
    assert!(gap < 1.0, "{gap}");
}

#[test]
fn off_the_map_short_or_blocked_is_the_lattices() {
    let planner = scenes::planner(false);
    let mut grid = free();
    let lanes = ell();
    let how = |grid: &Grid, s: Pose, g: Pose| {
        lanes
            .plan(&planner, grid, &s, &g, &never)
            .expect("a path")
            .how
    };
    // The lot: 50 m off every road.
    assert_eq!(
        how(
            &grid,
            Pose::new(10.0, 50.0, 0.0),
            Pose::new(60.0, -1.5, 0.0)
        ),
        How::Lattice(Fallback::OffMap)
    );
    // 15 m of road: no room for two 8 m joins.
    assert_eq!(
        how(
            &grid,
            Pose::new(10.0, -1.5, 0.0),
            Pose::new(25.0, -1.5, 0.0)
        ),
        How::Lattice(Fallback::Short)
    );
    // A box on the right lane at x = 50: the lattice goes round.
    for x in 276..284 {
        for y in 72..76 {
            grid.set(x, y, LETHAL);
        }
    }
    assert_eq!(
        how(&grid, Pose::new(5.0, -1.5, 0.0), Pose::new(90.0, -1.5, 0.0)),
        How::Lattice(Fallback::Blocked)
    );
}
