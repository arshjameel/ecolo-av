//! The chunks' rules, checked on random cars and routes (proptest, TODO 204).

use map_chunks::{wanted, ChunkKey, Detail, Grid, Point};
use proptest::prelude::*;

fn campus() -> Grid {
    Grid::new(Point::new(-900.0, -900.0), 64.0).expect("a good grid")
}

fn place() -> impl Strategy<Value = Point> {
    (-800.0..800.0f64, -800.0..800.0f64).prop_map(|(x, y)| Point::new(x, y))
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2000))]

    /// A point lies in the chunk `key_at` names: from its corner, less than one size on.
    #[test]
    fn a_point_is_inside_its_chunk(p in place()) {
        let grid = campus();
        let c = grid.corner(grid.key_at(p));
        prop_assert!(c.x <= p.x && p.x < c.x + grid.size());
        prop_assert!(c.y <= p.y && p.y < c.y + grid.size());
    }

    /// The car's chunk and the 8 round it are always wanted at full detail, and no other
    /// chunk is wanted at full detail.
    #[test]
    fn the_three_by_three_is_always_fine(car in place(), route in prop::collection::vec(place(), 0..8),
                                         ahead in 0.0..300.0f64) {
        let grid = campus();
        let want = wanted(&grid, car, &route, ahead);
        let home = grid.key_at(car);
        for key in home.around(1) {
            prop_assert_eq!(want.get(&key), Some(&Detail::Fine));
        }
        let fine: Vec<ChunkKey> = want.iter().filter(|(_, d)| **d == Detail::Fine).map(|(k, _)| *k).collect();
        prop_assert_eq!(fine.len(), 9);
    }

    /// Every route corner within `ahead` m (along the route, from the corner nearest the
    /// car) has its chunk wanted.
    #[test]
    fn the_route_ahead_is_wanted(route in prop::collection::vec(place(), 2..8),
                                 ahead in 0.0..600.0f64) {
        let grid = campus();
        let car = route[0];
        let want = wanted(&grid, car, &route, ahead);
        let mut along = 0.0;
        prop_assert!(want.contains_key(&grid.key_at(route[0])));
        for pair in route.windows(2) {
            along += (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y);
            if along <= ahead {
                prop_assert!(want.contains_key(&grid.key_at(pair[1])),
                             "the corner {:?}, {:.0} m along, is not wanted", pair[1], along);
            }
        }
    }
}
