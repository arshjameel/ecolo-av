//! The step finder on the nine recorded clouds (`python_vectors.txt`): the ramp, the
//! crowned road and an empty cloud have no step. The post, walls and cars do (their feet
//! are steps); those clouds only print.

mod vectors;

use voxels::{steps, usable, StepShape};

#[test]
fn the_recorded_ground_has_no_steps() {
    let (shape, clouds) = vectors::read(include_str!("python_vectors.txt"));
    for c in &clouds {
        let base: Vec<_> = c.lidar.iter().map(|p| c.mount.to_base(p)).collect();
        let found = steps(&usable(&base, &shape), &StepShape::default());
        println!("{}: {} step points", c.name, found.len());
        if ["a 10 degree ramp", "a crowned road", "empty"].contains(&c.name.as_str()) {
            assert!(found.is_empty(), "{}: {found:?}", c.name);
        }
    }
}
