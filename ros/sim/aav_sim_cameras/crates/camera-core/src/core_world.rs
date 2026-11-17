//! The core simulator's world (`sim_core::world::World`, read from Gazebo's SDF files) as
//! the cameras' [`SimWorld`]: the same shapes the rays hit, drawn.
//!
//! | SDF | Here |
//! |---|---|
//! | `<box>`, `<cylinder>`, `<plane>` | `Cuboid`, `Cylinder`, `Plane`: one to one |
//! | `<mesh>` (OBJ) | `Mesh`, its file and scale; the picture coordinates from the file |
//! | `<diffuse>`, `<albedo_map>` | the look's colour and picture |
//! | `<scene><background>`, `<ambient>`, the directional `<light>` | the sky, ambient, sun |
//!
//! Light: Gazebo's ogre2 and Bevy add light up differently, so SDF's numbers are scaled by
//! [`LIGHT`], the factor that made the road world's test copy look like Gazebo's pictures
//! by eye (ambient 0.5 → 0.3, the sun's 0.9 → 0.55).

use std::collections::HashMap;

use sim_core::world::{Geometry as CoreGeometry, Shape as CoreShape, World};

use crate::geom::{Pose, Rotation, Vec3};
use crate::world::{Geometry, Look, Rgb, Shape, SimWorld, Sun};

/// SDF's light numbers to ours (see the module's header).
pub const LIGHT: f32 = 0.6;
/// SDF's defaults where the file says nothing: `<background>` 0.7 grey, `<ambient>` 0.4.
const SKY: f64 = 0.7;
const AMBIENT: f64 = 0.4;
/// A shape with no `<diffuse>`: Gazebo draws it light grey.
const PLAIN: f64 = 0.7;

fn rgb(c: [f64; 4]) -> Rgb {
    Rgb::new(c[0] as f32, c[1] as f32, c[2] as f32)
}

fn grey(c: [f64; 4]) -> f32 {
    ((c[0] + c[1] + c[2]) / 3.0) as f32
}

fn pose(p: &sim_core::math::Pose) -> Pose {
    let m = &p.rotation.m;
    let column = |c: usize| Vec3::new(m[0][c], m[1][c], m[2][c]);
    Pose::new(
        Vec3::new(p.position.x, p.position.y, p.position.z),
        Rotation::from_axes(column(0), column(1), column(2)),
    )
}

fn geometry(g: &CoreGeometry) -> Geometry {
    match g {
        CoreGeometry::Box { size } => Geometry::Cuboid {
            x: size.x,
            y: size.y,
            z: size.z,
        },
        CoreGeometry::Cylinder { radius, length } => Geometry::Cylinder {
            radius: *radius,
            length: *length,
        },
        CoreGeometry::Plane { x, y } => Geometry::Plane { x: *x, y: *y },
        CoreGeometry::Mesh { file, scale } => Geometry::Mesh {
            file: file.clone(),
            scale: [scale.x, scale.y, scale.z],
        },
    }
}

/// One of the core's shapes as ours, moved by `at` (a placed model's pose in the map).
fn convert(s: &CoreShape, name: String, at: &Pose) -> Shape {
    Shape {
        name,
        geometry: geometry(&s.geometry),
        pose: at.then(&pose(&s.pose)),
        look: Look {
            colour: rgb(s.colour.unwrap_or([PLAIN, PLAIN, PLAIN, 1.0])),
            texture: s.texture.clone(),
        },
    }
}

/// A model placed while the world runs (sim-core `World::model_shapes`, in its own
/// frame), as shapes in the map: each named `<name>/<shape>`.
#[must_use]
pub fn placed(name: &str, shapes: &[CoreShape], at: &Pose) -> Vec<Shape> {
    shapes
        .iter()
        .enumerate()
        .map(|(i, s)| convert(s, format!("{name}/{i}"), at))
        .collect()
}

impl SimWorld {
    /// The core's world, as the cameras draw it. No car body: add it with `with_car`.
    #[must_use]
    pub fn from_core(world: &World) -> Self {
        let light = &world.lighting;
        let mut seen: HashMap<&str, usize> = HashMap::new();
        let shapes = world
            .shapes
            .iter()
            .map(|s| {
                // Names must be unique: a second "road/line" becomes "road/line#2".
                let n = seen.entry(s.name.as_str()).or_insert(0);
                *n += 1;
                let name = if *n == 1 {
                    s.name.clone()
                } else {
                    format!("{}#{n}", s.name)
                };
                convert(s, name, &Pose::IDENTITY)
            })
            .collect();
        SimWorld {
            sky: rgb(light.background.unwrap_or([SKY, SKY, SKY, 1.0])),
            ambient: LIGHT * grey(light.ambient.unwrap_or([AMBIENT, AMBIENT, AMBIENT, 1.0])),
            sun: light.sun.map(|sun| Sun {
                direction: Vec3::new(sun.direction.x, sun.direction.y, sun.direction.z),
                strength: LIGHT * grey(sun.diffuse),
            }),
            shapes,
            car: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::{Path, PathBuf};

    use sim_core::world::Surfaces;

    use super::*;

    fn gazebo() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../aav_sim_models")
    }

    fn road() -> SimWorld {
        let world = World::load(
            &gazebo().join("worlds/road.sdf"),
            &[gazebo().join("models")],
            Surfaces::Visual,
        )
        .unwrap();
        SimWorld::from_core(&world)
    }

    #[test]
    fn the_road_world_as_the_test_copy_has_it() {
        let w = road();
        let test = SimWorld::test_world();
        assert_eq!(w.sky, test.sky);
        assert!((w.ambient - test.ambient).abs() < 1e-6);
        let sun = w.sun.expect("the sun");
        assert!((sun.strength - 0.54).abs() < 1e-6);
        // The lines: where the test copy has them, the same colour.
        let line = w
            .shapes
            .iter()
            .find(|s| s.name == "road/left_line")
            .expect("the left line");
        assert_eq!(
            line.geometry,
            Geometry::Cuboid {
                x: 120.0,
                y: 0.15,
                z: 0.004
            }
        );
        assert!((line.pose.translation.y - 1.75).abs() < 1e-9);
        assert_eq!(line.look.colour, looks_line());
    }

    fn looks_line() -> Rgb {
        crate::world::looks::LINE
    }

    #[test]
    fn the_stop_sign_keeps_its_mesh_and_picture_and_faces_the_traffic() {
        let w = road();
        let plate = w
            .shapes
            .iter()
            .find(|s| s.name == "stop_sign/plate")
            .expect("the plate");
        assert!(matches!(&plate.geometry, Geometry::Mesh { file, .. } if file.is_file()));
        assert!(plate.look.texture.as_ref().is_some_and(|t| t.is_file()));
        // Turned half round: its face (+x) looks west, at the car.
        let face = plate.pose.rotation.apply(Vec3::new(1.0, 0.0, 0.0));
        assert!(face.x < -0.999, "{face:?}");
        assert!((plate.pose.translation.z - 2.0).abs() < 1e-9);
    }

    #[test]
    fn a_placed_box_sits_where_it_was_put() {
        let file = gazebo().join("models/obstacle/model.sdf");
        let shapes =
            World::model_shapes(&file, &[gazebo().join("models")], Surfaces::Visual).unwrap();
        let at = Pose::new(Vec3::new(15.5, 0.0, 0.0), Rotation::from_yaw(0.0));
        let got = placed("obstacle", &shapes, &at);
        assert_eq!(got.len(), 1);
        // The model's link is 0.5 m up: the box's centre at (15.5, 0, 0.5).
        let c = got[0].pose.translation;
        assert!((c.x - 15.5).abs() < 1e-9 && c.y.abs() < 1e-9 && (c.z - 0.5).abs() < 1e-9);
        assert!(matches!(got[0].geometry, Geometry::Cuboid { x, .. } if (x - 1.0).abs() < 1e-9));
    }

    #[test]
    fn names_stay_unique() {
        let w = road();
        let mut names: Vec<_> = w.shapes.iter().map(|s| s.name.as_str()).collect();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), w.shapes.len());
    }
}
