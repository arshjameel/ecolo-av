//! What the cameras see: the shared world type the simulator's core hands the renderer.
//!
//! Deliberately small: SDF's visual shapes (plane, box, cylinder) plus an octagon for stop
//! signs, each with a pose in the map frame and one look (a colour, maybe a picture on
//! it). Shapes follow SDF's conventions: a box and a cylinder are centred on their pose,
//! a cylinder's axis is its z, a plane's normal is its z. Colours are linear, 0 to 1, as
//! Gazebo reads SDF's `<diffuse>` and `<background>` (its sky 0.49 comes out as 186 of
//! 255 in the picture). Nothing here knows about Bevy or ROS.

use std::{fmt, path::PathBuf};

use serde::Deserialize;

use crate::geom::{Pose, Rotation, Vec3};

/// A linear colour, each part 0 to 1 (as SDF's).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rgb {
    pub r: f32,
    pub g: f32,
    pub b: f32,
}

impl Rgb {
    #[must_use]
    pub const fn new(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b }
    }

    /// As a picture's bytes (sRGB, 0 to 255).
    #[must_use]
    pub fn to_srgb8(self) -> [u8; 3] {
        let encode = |c: f32| {
            let c = c.clamp(0.0, 1.0);
            let s = if c <= 0.003_130_8 {
                12.92 * c
            } else {
                1.055 * c.powf(1.0 / 2.4) - 0.055
            };
            (s * 255.0).round() as u8
        };
        [encode(self.r), encode(self.g), encode(self.b)]
    }
}

/// A shape's geometry, in metres.
#[derive(Clone, Debug, PartialEq)]
pub enum Geometry {
    /// Flat, facing up its z, `x` by `y`.
    Plane { x: f64, y: f64 },
    /// A box centred on its pose.
    Cuboid { x: f64, y: f64, z: f64 },
    /// Centred on its pose, its axis along z.
    Cylinder { radius: f64, length: f64 },
    /// A flat octagon, its face looking along its +x: a stop sign's plate. `across` is
    /// flat side to flat side. A picture on it fills the square around it, its right edge
    /// to the octagon's +y (as seen from in front).
    Octagon { across: f64, thickness: f64 },
    /// An OBJ file (`crate::obj`), scaled along each axis, in its own frame: the campus's
    /// meshes, the stop sign's plate. Its picture coordinates come from the file.
    Mesh { file: PathBuf, scale: [f64; 3] },
}

/// How a shape looks.
#[derive(Clone, Debug, PartialEq)]
pub struct Look {
    pub colour: Rgb,
    /// A picture (PNG) laid over it, multiplied by the colour.
    pub texture: Option<PathBuf>,
}

impl Look {
    #[must_use]
    pub fn colour(r: f32, g: f32, b: f32) -> Self {
        Self {
            colour: Rgb::new(r, g, b),
            texture: None,
        }
    }
}

/// One thing in the world.
#[derive(Clone, Debug, PartialEq)]
pub struct Shape {
    /// Unique in its world: the simulator moves or removes things by name.
    pub name: String,
    pub geometry: Geometry,
    /// In the map frame (or in base_link, for the car's own body).
    pub pose: Pose,
    pub look: Look,
}

/// The sun: a directional light.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sun {
    /// Which way the light travels (SDF's `<direction>`), map frame.
    pub direction: Vec3,
    /// How bright a white surface facing it looks from the sun alone, 0 to 1: a number
    /// any renderer can match, unlike lux.
    pub strength: f32,
}

/// Everything the cameras draw.
#[derive(Clone, Debug, PartialEq)]
pub struct SimWorld {
    /// The background colour, as SDF's `<scene><background>`.
    pub sky: Rgb,
    /// Light from everywhere, 0 to 1 (SDF's `<scene><ambient>`, its grey level).
    pub ambient: f32,
    pub sun: Option<Sun>,
    /// Fixed in the map frame.
    pub shapes: Vec<Shape>,
    /// The car's own body, in base_link: only cameras with `sees_car` draw it.
    pub car: Vec<Shape>,
}

/// The road world's colours (`aav_gazebo/worlds/road.sdf`) and `docs/campus.md`'s looks.
pub mod looks {
    use super::Rgb;

    pub const SKY: Rgb = Rgb::new(0.49, 0.69, 0.80);
    pub const GRASS: Rgb = Rgb::new(0.31, 0.51, 0.27);
    pub const TARMAC: Rgb = Rgb::new(0.37, 0.37, 0.37);
    pub const LINE: Rgb = Rgb::new(0.92, 0.92, 0.92);
    pub const CONCRETE: Rgb = Rgb::new(0.70, 0.70, 0.66);
    pub const BRICK: Rgb = Rgb::new(0.30, 0.09, 0.05);
    pub const POST: Rgb = Rgb::new(0.55, 0.55, 0.55);
    /// The stop sign's red (`tools/make_stop_sign.py`'s sRGB 193, 39, 45).
    pub const STOP_RED: Rgb = Rgb::new(0.54, 0.022, 0.027);
    pub const CAR_BODY: Rgb = Rgb::new(0.02, 0.2, 0.05);
    pub const TYRE: Rgb = Rgb::new(0.05, 0.05, 0.05);
}

fn shape(name: &str, geometry: Geometry, xyz: [f64; 3], yaw: f64, look: Look) -> Shape {
    Shape {
        name: name.to_owned(),
        geometry,
        pose: Pose::new(Vec3::new(xyz[0], xyz[1], xyz[2]), Rotation::from_yaw(yaw)),
        look,
    }
}

fn plain(c: Rgb) -> Look {
    Look {
        colour: c,
        texture: None,
    }
}

/// A stop sign as `aav_gazebo/models/stop_sign`: a 0.75 m plate with its centre 2 m up,
/// on a grey post. `foot` is the post's foot; `yaw` turns the face (0 = facing +x).
#[must_use]
pub fn stop_sign(name: &str, foot: [f64; 2], yaw: f64, texture: Option<PathBuf>) -> Vec<Shape> {
    let r = Rotation::from_yaw(yaw);
    let at = |dx: f64, z: f64| {
        let d = r.apply(Vec3::new(dx, 0.0, 0.0));
        [foot[0] + d.x, foot[1] + d.y, z]
    };
    vec![
        shape(
            &format!("{name}/plate"),
            Geometry::Octagon {
                across: 0.75,
                thickness: 0.02,
            },
            at(0.0, 2.0),
            yaw,
            Look {
                colour: if texture.is_some() {
                    Rgb::new(1.0, 1.0, 1.0)
                } else {
                    looks::STOP_RED
                },
                texture,
            },
        ),
        shape(
            &format!("{name}/post"),
            Geometry::Cylinder {
                radius: 0.03,
                length: 2.0,
            },
            at(-0.03, 1.0),
            0.0,
            plain(looks::POST),
        ),
    ]
}

impl SimWorld {
    /// A small world to test with, like the road world: grass, 120 m of tarmac with its two
    /// lines, a 0.3 m concrete kerb along its right edge, a 1 m box on the left, and a red
    /// stop sign 25 m on at the right, facing the car. The car starts at the origin
    /// facing +x, as in Gazebo.
    #[must_use]
    pub fn test_world() -> Self {
        use looks::*;
        let mut shapes = vec![
            shape(
                "ground",
                Geometry::Plane { x: 400.0, y: 400.0 },
                [0.0, 0.0, 0.0],
                0.0,
                plain(GRASS),
            ),
            shape(
                "tarmac",
                Geometry::Cuboid {
                    x: 120.0,
                    y: 4.5,
                    z: 0.004,
                },
                [40.0, 0.0, 0.002],
                0.0,
                plain(TARMAC),
            ),
            shape(
                "left_line",
                Geometry::Cuboid {
                    x: 120.0,
                    y: 0.15,
                    z: 0.004,
                },
                [40.0, 1.75, 0.006],
                0.0,
                plain(LINE),
            ),
            shape(
                "right_line",
                Geometry::Cuboid {
                    x: 120.0,
                    y: 0.15,
                    z: 0.004,
                },
                [40.0, -1.75, 0.006],
                0.0,
                plain(LINE),
            ),
            shape(
                "kerb",
                Geometry::Cuboid {
                    x: 120.0,
                    y: 0.15,
                    z: 0.30,
                },
                [40.0, -2.325, 0.15],
                0.0,
                plain(CONCRETE),
            ),
            shape(
                "box",
                Geometry::Cuboid {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
                [12.0, 3.5, 0.5],
                0.0,
                plain(BRICK),
            ),
        ];
        // Facing west, at the traffic, as in road.sdf. Plain red: tests don't need a file.
        shapes.extend(stop_sign(
            "stop_sign",
            [25.0, -3.0],
            std::f64::consts::PI,
            None,
        ));
        Self {
            sky: SKY,
            ambient: 0.3,
            sun: Some(Sun {
                direction: Vec3::new(-0.5, 0.1, -0.9),
                strength: 0.55,
            }),
            shapes,
            car: Vec::new(),
        }
    }

    /// The car's body for the chase camera, from vehicle.yaml: a box the body's size from
    /// 0.25 m up, and four tyres.
    pub fn with_car(mut self, vehicle_yaml: &str) -> Result<Self, CarError> {
        self.car = car_body(vehicle_yaml)?;
        Ok(self)
    }
}

/// Why the car's body can't be read from vehicle.yaml.
#[derive(Debug)]
pub struct CarError(serde_yaml_ng::Error);

impl fmt::Display for CarError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "vehicle.yaml's vehicle.body: {}", self.0)
    }
}

impl std::error::Error for CarError {}

#[derive(Deserialize)]
struct Body {
    length: f64,
    width: f64,
    height: f64,
    rear_overhang: f64,
}

#[derive(Deserialize)]
struct Vehicle {
    wheelbase: f64,
    track: f64,
    wheel_diameter: f64,
    wheel_width: f64,
    body: Body,
}

#[derive(Deserialize)]
struct VehicleFile {
    vehicle: Vehicle,
}

/// The car's body in base_link (on the ground under the rear axle).
pub fn car_body(vehicle_yaml: &str) -> Result<Vec<Shape>, CarError> {
    let v = serde_yaml_ng::from_str::<VehicleFile>(vehicle_yaml)
        .map_err(CarError)?
        .vehicle;
    let b = &v.body;
    let floor = 0.25;
    let mut out = vec![shape(
        "car/body",
        Geometry::Cuboid {
            x: b.length,
            y: b.width,
            z: b.height - floor,
        },
        [
            b.length / 2.0 - b.rear_overhang,
            0.0,
            floor + (b.height - floor) / 2.0,
        ],
        0.0,
        plain(looks::CAR_BODY),
    )];
    let r = v.wheel_diameter / 2.0;
    for (i, x) in [0.0, v.wheelbase].into_iter().enumerate() {
        for (j, y) in [v.track / 2.0, -v.track / 2.0].into_iter().enumerate() {
            out.push(Shape {
                name: format!("car/tyre_{i}{j}"),
                geometry: Geometry::Cylinder {
                    radius: r,
                    length: v.wheel_width,
                },
                // The axle along y: the cylinder's z turned onto y.
                pose: Pose::new(
                    Vec3::new(x, y, r),
                    Rotation::from_rpy(std::f64::consts::FRAC_PI_2, 0.0, 0.0),
                ),
                look: plain(looks::TYRE),
            });
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gazebos_sky_in_the_picture() {
        assert_eq!(looks::SKY.to_srgb8(), [186, 216, 231]);
        assert_eq!(Rgb::new(0.0, 1.0, 0.5).to_srgb8(), [0, 255, 188]);
    }

    #[test]
    fn test_world_names_are_unique() {
        let w = SimWorld::test_world();
        let mut names: Vec<_> = w.shapes.iter().map(|s| s.name.as_str()).collect();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), w.shapes.len());
    }

    #[test]
    fn the_car_from_vehicle_yaml() {
        let yaml = include_str!("../../../../../bringup/aav_description/config/vehicle.yaml");
        let car = car_body(yaml).unwrap();
        assert_eq!(car.len(), 5);
        // The body reaches 1.8 m ahead of the rear axle: 2.0 long, 0.2 behind it.
        let Geometry::Cuboid { x, .. } = car[0].geometry else {
            panic!("the body is a box")
        };
        assert!((car[0].pose.translation.x + x / 2.0 - 1.8).abs() < 1e-9);
    }
}
