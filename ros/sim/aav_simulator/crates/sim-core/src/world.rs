//! One world description, read from the files Gazebo reads: `worlds/*.sdf`, the models
//! they include (`model://stop_sign`), and the campus that `pixi run campus` makes.
//!
//! What it keeps: every shape as triangles (boxes, cylinders, planes, OBJ meshes, at their
//! model, link and shape poses), and the datum (`<spherical_coordinates>`). By default the
//! shapes are the visuals, because Gazebo's lidars render what you see; the collisions are
//! there for a footprint check. Spheres, heightmaps and DAE meshes are skipped, with a
//! warning (none of our worlds has one).

use std::path::{Path, PathBuf};

use roxmltree::{Document, Node};

use crate::geo::Datum;
use crate::math::{Pose, Vec3};
use crate::scene::{Mesh, Scene, Triangle};

/// Which of a link's shapes to read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Surfaces {
    /// What you see, and what Gazebo's lidars and cameras see.
    Visual,
    /// What the physics touches.
    Collision,
}

impl Surfaces {
    fn tag(self) -> &'static str {
        match self {
            Surfaces::Visual => "visual",
            Surfaces::Collision => "collision",
        }
    }
}

/// Why a world couldn't be read.
#[derive(Debug, Clone, PartialEq)]
pub enum WorldError {
    Read { path: PathBuf, why: String },
    Xml { path: PathBuf, why: String },
    NoWorld(PathBuf),
    NoModel { uri: String, searched: Vec<PathBuf> },
    Mesh { path: PathBuf, why: String },
}

impl std::fmt::Display for WorldError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WorldError::Read { path, why } => write!(f, "can't read {}: {why}", path.display()),
            WorldError::Xml { path, why } => write!(f, "{} isn't SDF: {why}", path.display()),
            WorldError::NoWorld(path) => write!(f, "{} has no <world>", path.display()),
            WorldError::NoModel { uri, searched } => {
                write!(f, "no {uri} in any of {searched:?} (GZ_SIM_RESOURCE_PATH)")
            }
            WorldError::Mesh { path, why } => write!(f, "mesh {}: {why}", path.display()),
        }
    }
}

impl std::error::Error for WorldError {}

/// One shape as the world file draws it, before it becomes triangles: for a renderer
/// (the cameras' `camera-core` `SimWorld` takes these one to one: boxes, cylinders, planes;
/// a mesh by its file).
#[derive(Debug, Clone, PartialEq)]
pub struct Shape {
    /// The model's name, then the shape's (`wall_south/visual`).
    pub name: String,
    pub geometry: Geometry,
    /// Where it is in the world (the map frame).
    pub pose: Pose,
    /// SDF's `<diffuse>` (linear RGBA, 0..1), if the shape has one.
    pub colour: Option<[f64; 4]>,
    /// SDF's `<pbr><metal><albedo_map>`: the picture laid over it, if it has one.
    pub texture: Option<PathBuf>,
}

/// How the world is lit and its sky, as SDF's `<scene>` and its directional `<light>`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Lighting {
    /// `<scene><background>` (linear RGBA).
    pub background: Option<[f64; 4]>,
    /// `<scene><ambient>` (linear RGBA).
    pub ambient: Option<[f64; 4]>,
    /// The first directional `<light>`: the sun.
    pub sun: Option<Sunlight>,
}

/// A directional light: which way it shines, and its `<diffuse>` colour.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sunlight {
    pub direction: Vec3,
    pub diffuse: [f64; 4],
}

/// A shape's geometry, as SDF says it.
#[derive(Debug, Clone, PartialEq)]
pub enum Geometry {
    /// `size` across, centred on the pose.
    Box { size: Vec3 },
    /// Upright, centred on the pose.
    Cylinder { radius: f64, length: f64 },
    /// Flat, facing up, `x` by `y`.
    Plane { x: f64, y: f64 },
    /// An OBJ file, scaled.
    Mesh { file: PathBuf, scale: Vec3 },
}

/// A world, ready for rays.
#[derive(Debug, Clone)]
pub struct World {
    /// Everything a ray can hit.
    pub scene: Scene,
    /// The world's origin on the Earth; Carleton's if the file has none.
    pub datum: Datum,
    /// The models read, by name, in order.
    pub models: Vec<String>,
    /// What was skipped, in words.
    pub warnings: Vec<String>,
    /// Every shape read, as the file draws it: for a renderer (the cameras).
    pub shapes: Vec<Shape>,
    /// The sky and the light, for a renderer.
    pub lighting: Lighting,
}

impl World {
    /// Nothing but flat ground everywhere: for tests.
    #[must_use]
    pub fn flat(size: f64) -> World {
        World {
            scene: Scene::new(Mesh::plane(size, size).triangles),
            datum: Datum::CARLETON,
            models: vec!["ground_plane".into()],
            warnings: Vec::new(),
            shapes: vec![Shape {
                name: "ground_plane".into(),
                geometry: Geometry::Plane { x: size, y: size },
                pose: Pose::IDENTITY,
                colour: None,
                texture: None,
            }],
            lighting: Lighting::default(),
        }
    }

    /// Read a world file. `models`: where `model://name` may be (aav_gazebo's
    /// `models/`, then GZ_SIM_RESOURCE_PATH's folders).
    pub fn load(path: &Path, models: &[PathBuf], surfaces: Surfaces) -> Result<World, WorldError> {
        let text = read(path)?;
        let doc = parse(path, &text)?;
        let world = doc
            .descendants()
            .find(|n| n.has_tag_name("world"))
            .ok_or_else(|| WorldError::NoWorld(path.to_path_buf()))?;
        let mut reader = Reader {
            models,
            surfaces,
            triangles: Vec::new(),
            names: Vec::new(),
            warnings: Vec::new(),
            shapes: Vec::new(),
        };
        let folder = path.parent().unwrap_or(Path::new("."));
        for node in world.children().filter(Node::is_element) {
            match node.tag_name().name() {
                "model" => reader.model(node, &Pose::IDENTITY, folder, None)?,
                "include" => reader.include(node, &Pose::IDENTITY)?,
                _ => {}
            }
        }
        Ok(World {
            scene: Scene::new(reader.triangles),
            datum: datum(world).unwrap_or(Datum::CARLETON),
            models: reader.names,
            warnings: reader.warnings,
            shapes: reader.shapes,
            lighting: lighting(world),
        })
    }

    /// One model file's shapes as SDF draws them, in the model's own frame: for a renderer
    /// to draw a model placed while the world runs.
    pub fn model_shapes(
        path: &Path,
        models: &[PathBuf],
        surfaces: Surfaces,
    ) -> Result<Vec<Shape>, WorldError> {
        let text = read(path)?;
        let doc = parse(path, &text)?;
        let model = doc
            .descendants()
            .find(|n| n.has_tag_name("model"))
            .ok_or_else(|| WorldError::NoWorld(path.to_path_buf()))?;
        let mut reader = Reader {
            models,
            surfaces,
            triangles: Vec::new(),
            names: vec![model.attribute("name").unwrap_or("model").to_string()],
            warnings: Vec::new(),
            shapes: Vec::new(),
        };
        let folder = path.parent().unwrap_or(Path::new("."));
        reader.links(model, &Pose::IDENTITY, folder)?;
        Ok(reader.shapes)
    }

    /// One model file's shapes (`models/post/model.sdf`), in the model's own frame: for
    /// placing it while the world runs.
    pub fn model_mesh(
        path: &Path,
        models: &[PathBuf],
        surfaces: Surfaces,
    ) -> Result<Mesh, WorldError> {
        let text = read(path)?;
        let doc = parse(path, &text)?;
        let model = doc
            .descendants()
            .find(|n| n.has_tag_name("model"))
            .ok_or_else(|| WorldError::NoWorld(path.to_path_buf()))?;
        let mut reader = Reader {
            models,
            surfaces,
            triangles: Vec::new(),
            names: Vec::new(),
            warnings: Vec::new(),
            shapes: Vec::new(),
        };
        let folder = path.parent().unwrap_or(Path::new("."));
        // Its own pose doesn't count: the caller places it.
        reader.links(model, &Pose::IDENTITY, folder)?;
        Ok(Mesh {
            triangles: reader.triangles,
        })
    }
}

fn read(path: &Path) -> Result<String, WorldError> {
    std::fs::read_to_string(path).map_err(|e| WorldError::Read {
        path: path.to_path_buf(),
        why: e.to_string(),
    })
}

fn parse<'a>(path: &Path, text: &'a str) -> Result<Document<'a>, WorldError> {
    Document::parse(text).map_err(|e| WorldError::Xml {
        path: path.to_path_buf(),
        why: e.to_string(),
    })
}

fn child<'a, 'i>(node: Node<'a, 'i>, tag: &str) -> Option<Node<'a, 'i>> {
    node.children().find(|n| n.has_tag_name(tag))
}

fn text<'a>(node: Node<'a, '_>, tag: &str) -> Option<&'a str> {
    child(node, tag).and_then(|n| n.text()).map(str::trim)
}

fn numbers(s: &str) -> Vec<f64> {
    s.split_whitespace()
        .filter_map(|w| w.parse().ok())
        .collect()
}

/// A `<pose>`: x y z roll pitch yaw (missing numbers are 0).
fn pose_of(node: Node) -> Pose {
    let n = text(node, "pose").map(numbers).unwrap_or_default();
    let at = |i: usize| n.get(i).copied().unwrap_or(0.0);
    Pose::from_xyz_rpy([at(0), at(1), at(2)], [at(3), at(4), at(5)])
}

fn colour(node: Node, tag: &str) -> Option<[f64; 4]> {
    text(node, tag)
        .map(numbers)
        .filter(|n| n.len() >= 3)
        .map(|n| [n[0], n[1], n[2], n.get(3).copied().unwrap_or(1.0)])
}

fn lighting(world: Node) -> Lighting {
    let scene = child(world, "scene");
    let sun = world
        .children()
        .filter(|n| n.has_tag_name("light"))
        .find(|n| n.attribute("type") == Some("directional"))
        .and_then(|light| {
            let d = text(light, "direction").map(numbers).unwrap_or_default();
            (d.len() >= 3).then(|| Sunlight {
                direction: Vec3::new(d[0], d[1], d[2]),
                diffuse: colour(light, "diffuse").unwrap_or([1.0; 4]),
            })
        });
    Lighting {
        background: scene.and_then(|s| colour(s, "background")),
        ambient: scene.and_then(|s| colour(s, "ambient")),
        sun,
    }
}

fn datum(world: Node) -> Option<Datum> {
    let sc = child(world, "spherical_coordinates")?;
    let number = |tag| text(sc, tag).and_then(|t| t.parse::<f64>().ok());
    Some(Datum {
        latitude: number("latitude_deg")?,
        longitude: number("longitude_deg")?,
        elevation: number("elevation").unwrap_or(0.0),
    })
}

struct Reader<'m> {
    models: &'m [PathBuf],
    surfaces: Surfaces,
    triangles: Vec<Triangle>,
    names: Vec<String>,
    warnings: Vec<String>,
    shapes: Vec<Shape>,
}

impl Reader<'_> {
    /// `<include><uri>model://name</uri><pose/></include>`.
    fn include(&mut self, node: Node, parent: &Pose) -> Result<(), WorldError> {
        let uri = text(node, "uri").unwrap_or_default().to_string();
        let folder = self.model_folder(&uri)?;
        let file = folder.join("model.sdf");
        let source = read(&file)?;
        let doc = parse(&file, &source)?;
        let Some(model) = doc.descendants().find(|n| n.has_tag_name("model")) else {
            self.warnings.push(format!("{uri}: no <model>"));
            return Ok(());
        };
        // The include's pose and name replace the model's own.
        let pose = if child(node, "pose").is_some() {
            pose_of(node)
        } else {
            pose_of(model)
        };
        let name = text(node, "name").map(str::to_string);
        self.model(
            model,
            &parent.then(&pose),
            &folder,
            name.or(Some(String::new())),
        )
    }

    /// Where `model://name` lives.
    fn model_folder(&self, uri: &str) -> Result<PathBuf, WorldError> {
        let name = uri.strip_prefix("model://").unwrap_or(uri);
        self.models
            .iter()
            .map(|dir| dir.join(name))
            .find(|p| p.join("model.sdf").is_file())
            .ok_or_else(|| WorldError::NoModel {
                uri: uri.to_string(),
                searched: self.models.to_vec(),
            })
    }

    /// A `<model>`: its pose (unless `placed` already holds it), then its links.
    fn model(
        &mut self,
        node: Node,
        parent: &Pose,
        folder: &Path,
        placed: Option<String>,
    ) -> Result<(), WorldError> {
        let (pose, name) = match placed {
            Some(name) => (*parent, name),
            None => (parent.then(&pose_of(node)), String::new()),
        };
        let name = if name.is_empty() {
            node.attribute("name").unwrap_or("").to_string()
        } else {
            name
        };
        self.names.push(name);
        self.links(node, &pose, folder)?;
        // Models inside models.
        for inner in node.children().filter(|n| n.has_tag_name("model")) {
            self.model(inner, &pose, folder, None)?;
        }
        Ok(())
    }

    fn links(&mut self, model: Node, pose: &Pose, folder: &Path) -> Result<(), WorldError> {
        for link in model.children().filter(|n| n.has_tag_name("link")) {
            let at = pose.then(&pose_of(link));
            let tag = self.surfaces.tag();
            for shape in link.children().filter(|n| n.has_tag_name(tag)) {
                let Some(geometry) = child(shape, "geometry") else {
                    continue;
                };
                if let Some((mesh, kind)) = self.geometry(geometry, folder)? {
                    let placed = at.then(&pose_of(shape));
                    self.triangles.extend(mesh.transformed(&placed).triangles);
                    let material = child(shape, "material");
                    let colour = material.and_then(|m| colour(m, "diffuse"));
                    let texture = match material
                        .and_then(|m| child(m, "pbr"))
                        .and_then(|p| child(p, "metal"))
                        .and_then(|m| text(m, "albedo_map"))
                    {
                        Some(uri) => Some(self.mesh_path(uri, folder)?),
                        None => None,
                    };
                    let model = self.names.last().cloned().unwrap_or_default();
                    let name = shape.attribute("name").unwrap_or("");
                    self.shapes.push(Shape {
                        name: format!("{model}/{name}"),
                        geometry: kind,
                        pose: placed,
                        colour,
                        texture,
                    });
                }
            }
        }
        Ok(())
    }

    /// A `<geometry>` as triangles, and as SDF said it.
    fn geometry(&mut self, g: Node, folder: &Path) -> Result<Option<(Mesh, Geometry)>, WorldError> {
        let Some(shape) = g.children().find(Node::is_element) else {
            return Ok(None);
        };
        let vector = |tag: &str| {
            let n = text(shape, tag).map(numbers).unwrap_or_default();
            (n.len() >= 3).then(|| Vec3::new(n[0], n[1], n[2]))
        };
        let number = |tag: &str| text(shape, tag).and_then(|t| t.parse::<f64>().ok());
        Ok(match shape.tag_name().name() {
            "box" => vector("size").map(|size| (Mesh::cuboid(size), Geometry::Box { size })),
            "cylinder" => match (number("radius"), number("length")) {
                (Some(radius), Some(length)) => Some((
                    Mesh::cylinder(radius, length, 16),
                    Geometry::Cylinder { radius, length },
                )),
                _ => None,
            },
            "plane" => {
                let n = text(shape, "size").map(numbers).unwrap_or_default();
                let normal = vector("normal").unwrap_or(Vec3::new(0.0, 0.0, 1.0));
                if (normal.z - 1.0).abs() > 1e-9 {
                    self.warnings.push(format!(
                        "a plane facing {normal:?}: only flat ground is read"
                    ));
                    None
                } else {
                    let (x, y) = (
                        n.first().copied().unwrap_or(100.0),
                        n.get(1).copied().unwrap_or(100.0),
                    );
                    Some((Mesh::plane(x, y), Geometry::Plane { x, y }))
                }
            }
            "mesh" => {
                let uri = text(shape, "uri").unwrap_or_default();
                let path = self.mesh_path(uri, folder)?;
                if path.extension().and_then(|e| e.to_str()) != Some("obj") {
                    self.warnings
                        .push(format!("{}: only OBJ meshes are read", path.display()));
                    return Ok(None);
                }
                let mesh = Mesh::from_obj(&read(&path)?).map_err(|why| WorldError::Mesh {
                    path: path.clone(),
                    why,
                })?;
                let scale = vector("scale").unwrap_or(Vec3::new(1.0, 1.0, 1.0));
                Some((mesh.scaled(scale), Geometry::Mesh { file: path, scale }))
            }
            other => {
                self.warnings.push(format!("<{other}> isn't read"));
                None
            }
        })
    }

    /// A mesh's file: `model://name/...`, `file://...`, or relative to the SDF file.
    fn mesh_path(&self, uri: &str, folder: &Path) -> Result<PathBuf, WorldError> {
        if let Some(rest) = uri.strip_prefix("model://") {
            let (name, inside) = rest.split_once('/').unwrap_or((rest, ""));
            return Ok(self.model_folder(name)?.join(inside));
        }
        let plain = uri.strip_prefix("file://").unwrap_or(uri);
        let p = Path::new(plain);
        Ok(if p.is_absolute() {
            p.to_path_buf()
        } else {
            folder.join(p)
        })
    }
}
