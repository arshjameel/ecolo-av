//! The lidar's waves: flat ground that sits a little higher than the road.
//!
//! Each lidar ring that hits flat ground draws a circle of cubes round the car. On the
//! road they are ground already. Where the ground is raised (the campus's grass and
//! pavements, a kerb up from the road: `tools/campus_world.py`), they land in the
//! lowest height band and fill the screen with rings (Ben, 17 Nov: "those circles don't
//! help anymore"). The voxels node is right to stop for them; the screens only hide them.
//!
//! The rule, for each cube lower than [`FlatRule::top`]:
//!
//! | Near it, seen from above (within `reach`) | It is | Why |
//! |---|---|---|
//! | a taller cube | standing | the foot of a wall, a post, a person |
//! | the road | standing | the kerb: where the road ends |
//! | neither | the ground | flat raised ground: a wave |
//!
//! Cubes at or above `top` always stand. Both screens use it: the debug HUD on its own
//! cubes, the developer window on the voxels node's.

use std::collections::HashMap;

use crate::cubes::{Band, Cube, Metres};

/// What makes a low cube flat ground.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlatRule {
    /// Cubes lower than this (above base_link) may be flat ground.
    pub top: Metres,
    /// How near the road or a taller cube makes a low cube stand, seen from above.
    pub reach: Metres,
}

impl Default for FlatRule {
    /// 0.45 m: over the campus's 0.30 m kerb and its grass, under a car's bonnet.
    /// 0.5 m: two of the voxels' 0.25 m cubes.
    fn default() -> Self {
        Self {
            top: Metres(0.45),
            reach: Metres(0.5),
        }
    }
}

/// Points seen from above, filed by square so the near ones are quick to find.
struct Near {
    reach: f32,
    squares: HashMap<(i32, i32), Vec<[f32; 2]>>,
}

impl Near {
    fn new(reach: f32, points: impl IntoIterator<Item = [f32; 2]>) -> Self {
        let mut squares: HashMap<(i32, i32), Vec<[f32; 2]>> = HashMap::new();
        for p in points {
            squares.entry(square(p, reach)).or_default().push(p);
        }
        Self { reach, squares }
    }

    /// Is any point within `reach` of `p`?
    fn any_within(&self, p: [f32; 2]) -> bool {
        let (i, j) = square(p, self.reach);
        let reach2 = self.reach * self.reach;
        (-1..=1)
            .flat_map(|di| (-1..=1).map(move |dj| (i.saturating_add(di), j.saturating_add(dj))))
            .filter_map(|key| self.squares.get(&key))
            .flatten()
            .any(|q| {
                let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
                dx * dx + dy * dy <= reach2
            })
    }
}

/// Which square of `reach` a point is in. `as` saturates far out.
fn square(p: [f32; 2], reach: f32) -> (i32, i32) {
    ((p[0] / reach).floor() as i32, (p[1] / reach).floor() as i32)
}

fn from_above(c: &Cube) -> [f32; 2] {
    [c.centre[0], c.centre[1]]
}

/// Is this cube flat raised ground (`rule`), given the road and the taller cubes?
fn is_flat(c: &Cube, rule: FlatRule, road: &Near, tall: &Near) -> bool {
    c.band != Band::Ground
        && f64::from(c.centre[2]) < rule.top.0
        && !road.any_within(from_above(c))
        && !tall.any_within(from_above(c))
}

/// Moves the flat raised ground among `cubes` into [`Band::Ground`].
///
/// `road`: more ground, such as the voxels node's road cloud. The ground cubes among
/// `cubes` count as road too. Heights are the cubes' `z`, which is height above
/// base_link in odom (the EKF is 2D). Returns how many cubes became ground.
pub fn ground_the_flat(cubes: &mut [Cube], road: &[Cube], rule: FlatRule) -> usize {
    let reach = rule.reach.0 as f32;
    let road_near = Near::new(
        reach,
        road.iter()
            .chain(cubes.iter().filter(|c| c.band == Band::Ground))
            .map(from_above),
    );
    let tall = Near::new(
        reach,
        cubes
            .iter()
            .filter(|c| f64::from(c.centre[2]) >= rule.top.0)
            .map(from_above),
    );
    let flat: Vec<usize> = cubes
        .iter()
        .enumerate()
        .filter(|(_, c)| is_flat(c, rule, &road_near, &tall))
        .map(|(i, _)| i)
        .collect();
    for &i in &flat {
        cubes[i].band = Band::Ground;
    }
    flat.len()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube(x: f32, y: f32, z: f32) -> Cube {
        let band = Band::of(Metres(f64::from(z)), Metres(0.25));
        Cube {
            centre: [x, y, z],
            band,
        }
    }

    /// A lidar ring of radius `r` on flat ground `z` high, a cube every 0.25 m.
    fn ring(r: f32, z: f32) -> Vec<Cube> {
        let n = (std::f32::consts::TAU * r / 0.25) as usize;
        (0..n)
            .map(|k| {
                let a = std::f32::consts::TAU * k as f32 / n as f32;
                cube(r * a.cos(), r * a.sin(), z)
            })
            .collect()
    }

    #[test]
    fn rings_on_raised_grass_become_ground() {
        // The campus: grass a 0.30 m kerb up from the road, no road near.
        let mut cubes: Vec<Cube> = [6.0, 9.0, 14.0]
            .iter()
            .flat_map(|&r| ring(r, 0.31))
            .collect();
        let n = cubes.len();
        assert!(cubes.iter().all(|c| c.band == Band::Knee));
        assert_eq!(ground_the_flat(&mut cubes, &[], FlatRule::default()), n);
        assert!(cubes.iter().all(|c| c.band == Band::Ground));
    }

    #[test]
    fn the_kerb_beside_the_road_stands() {
        // The road ends at x = 5; the kerb's top at x = 5.2, then grass from x = 6.
        let road: Vec<Cube> = (0..20).map(|k| cube(4.8, k as f32 * 0.25, 0.0)).collect();
        let mut cubes = vec![cube(5.2, 1.0, 0.31), cube(7.0, 1.0, 0.31)];
        assert_eq!(ground_the_flat(&mut cubes, &road, FlatRule::default()), 1);
        assert_eq!(cubes[0].band, Band::Knee, "the kerb");
        assert_eq!(cubes[1].band, Band::Ground, "the grass behind it");
    }

    #[test]
    fn the_screens_own_ground_cubes_count_as_road() {
        // The debug HUD has no road cloud: its own ground cubes are the road.
        let mut cubes = vec![cube(3.0, 0.0, 0.05), cube(3.3, 0.0, 0.3)];
        assert_eq!(ground_the_flat(&mut cubes, &[], FlatRule::default()), 0);
        assert_eq!(cubes[1].band, Band::Knee);
    }

    #[test]
    fn the_feet_of_tall_things_stand() {
        // A person on the grass: their feet stay with them.
        let mut cubes = vec![
            cube(8.0, 2.0, 0.31),
            cube(8.1, 2.0, 1.0),
            cube(8.0, 2.1, 1.6),
        ];
        assert_eq!(ground_the_flat(&mut cubes, &[], FlatRule::default()), 0);
        assert_eq!(cubes[0].band, Band::Knee);
    }

    #[test]
    fn tall_cubes_always_stand() {
        let mut cubes = vec![cube(20.0, 0.0, 0.6), cube(20.0, 5.0, 2.0)];
        assert_eq!(ground_the_flat(&mut cubes, &[], FlatRule::default()), 0);
        assert_eq!(cubes[0].band, Band::Knee);
        assert_eq!(cubes[1].band, Band::High);
    }

    #[test]
    fn far_from_the_origin_still_works() {
        // Odom far from where the car started.
        let road = [cube(1.0e6, 1.0e6, 0.0)];
        let mut cubes = vec![cube(1.0e6 + 0.3, 1.0e6, 0.3)];
        assert_eq!(ground_the_flat(&mut cubes, &road, FlatRule::default()), 0);
    }
}
