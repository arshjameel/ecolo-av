//! The worlds Gazebo reads, read by us: aav_sim_models's test lot and road, and the campus if
//! `pixi run campus` has made it.

use std::path::{Path, PathBuf};
use std::time::Instant;

use sim_core::lidar::{LidarSpec, Scan};
use sim_core::math::{Pose, Rot3, Vec3};
use sim_core::noise::Noise;
use sim_core::scene::Ray;
use sim_core::world::{Surfaces, World};

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../..")
}

fn gazebo() -> PathBuf {
    repo().join("ros/sim/aav_sim_models")
}

fn load(name: &str) -> World {
    let models = [gazebo().join("models")];
    World::load(
        &gazebo().join(format!("worlds/{name}.sdf")),
        &models,
        Surfaces::Visual,
    )
    .unwrap_or_else(|e| panic!("{name}: {e}"))
}

fn distance(world: &World, from: Vec3, to: Vec3) -> Option<f64> {
    let d = to - from;
    let ray = Ray {
        origin: from,
        direction: d * (1.0 / d.length()),
    };
    world.scene.cast(&ray, 200.0)
}

#[test]
fn the_test_lot_has_its_walls_and_building() {
    let w = load("test_lot");
    assert!(w.warnings.is_empty(), "{:?}", w.warnings);
    assert_eq!(w.models.len(), 8);
    assert_eq!(w.datum, sim_core::geo::Datum::CARLETON);
    // From the origin, 1 m up: the south wall's face at y = -9.0, the west one at x = -9.0.
    let at = Vec3::new(0.0, 0.0, 1.0);
    let south = distance(&w, at, Vec3::new(0.0, -50.0, 1.0)).expect("south wall");
    assert!((south - 9.0).abs() < 1e-6, "{south}");
    let west = distance(&w, at, Vec3::new(-50.0, 0.0, 1.0)).expect("west wall");
    assert!((west - 9.0).abs() < 1e-6, "{west}");
    // The building's south face is at y = 20, between x = 20 and 40.
    let building =
        distance(&w, Vec3::new(30.0, 0.0, 1.0), Vec3::new(30.0, 50.0, 1.0)).expect("building");
    assert!((building - 20.0).abs() < 1e-6, "{building}");
    // Straight down: the ground.
    let ground = distance(&w, at, Vec3::new(0.0, 0.0, -1.0)).expect("ground");
    assert!((ground - 1.0).abs() < 1e-9);
}

#[test]
fn the_shapes_for_a_renderer_keep_their_kind_place_and_colour() {
    use sim_core::world::Geometry;
    let w = load("test_lot");
    assert_eq!(w.shapes.len(), 8);
    let south = w
        .shapes
        .iter()
        .find(|s| s.name == "wall_south/visual")
        .expect("the wall");
    assert_eq!(
        south.geometry,
        Geometry::Box {
            size: Vec3::new(80.0, 1.0, 1.5)
        }
    );
    assert_eq!(south.pose.position, Vec3::new(30.0, -9.5, 0.75));
    assert_eq!(south.colour, Some([0.6, 0.3, 0.2, 1.0]));
    let road = load("road");
    let plate = road
        .shapes
        .iter()
        .find(|s| s.name == "stop_sign/plate")
        .expect("the plate");
    assert!(
        matches!(&plate.geometry, Geometry::Mesh { file, .. } if file.ends_with("stop_sign.obj"))
    );
    // Its picture, through model://, for the cameras.
    assert!(plate
        .texture
        .as_ref()
        .is_some_and(|t| t.ends_with("materials/textures/stop_sign.png") && t.is_file()));
}

#[test]
fn the_sky_and_the_sun_as_the_file_says() {
    let road = load("road").lighting;
    assert_eq!(road.background, Some([0.49, 0.69, 0.80, 1.0]));
    assert_eq!(road.ambient, Some([0.5, 0.5, 0.5, 1.0]));
    let sun = road.sun.expect("the sun");
    assert_eq!(sun.direction, Vec3::new(-0.5, 0.1, -0.9));
    assert_eq!(sun.diffuse, [0.9, 0.9, 0.9, 1.0]);
}

#[test]
fn the_road_has_its_stop_sign_from_model_paths() {
    let w = load("road");
    assert!(w.models.iter().any(|m| m == "stop_sign"), "{:?}", w.models);
    // The sign's post: centred at (25.03, -2) after the include's half turn, 0.03 m round.
    let post = distance(&w, Vec3::new(20.0, -2.0, 1.0), Vec3::new(30.0, -2.0, 1.0)).expect("post");
    assert!((post - 5.0).abs() < 0.01, "{post}");
    // The tarmac is a picture 4 mm thick, its top 4 mm up: a ray meets it, not the grass.
    let tarmac =
        distance(&w, Vec3::new(10.0, 0.0, 1.0), Vec3::new(10.0, 0.0, -1.0)).expect("tarmac");
    assert!((tarmac - 0.996).abs() < 1e-6, "{tarmac}");
}

#[test]
fn the_roof_lidar_in_the_test_lot_is_quick() {
    let w = load("test_lot");
    let lidar = Pose::new(Vec3::new(0.49, 0.0, 1.52), Rot3::IDENTITY);
    let t = Instant::now();
    let scan = Scan::cast(&LidarSpec::ROOF, &w.scene, &lidar, &mut Noise::new(1), 4);
    let took = t.elapsed();
    let hits = scan.hits().count();
    // The ground all round for the low rings, and the walls for many level ones.
    assert!(hits > 10_000, "{hits} hits");
    println!("{hits} hits in {took:?}");
    assert!(
        took.as_millis() < 100,
        "a scan took {took:?}: it must fit in 0.1 s"
    );
}

#[test]
fn a_model_file_for_placing() {
    let models = [gazebo().join("models")];
    let post = World::model_mesh(
        &gazebo().join("models/post/model.sdf"),
        &models,
        Surfaces::Visual,
    )
    .expect("post");
    assert_eq!(post.triangles.len(), 12);
    let top = post
        .triangles
        .iter()
        .map(|t| t.a.z.max(t.b.z).max(t.c.z))
        .fold(0.0, f64::max);
    assert!((top - 1.8).abs() < 1e-9);
}

#[test]
fn the_campus_if_made() {
    let path = repo().join("maps/campus/campus.sdf");
    // A worktree has no maps/: look in the main checkout too.
    let path = if path.is_file() {
        path
    } else {
        match std::env::var("AAV_CAMPUS_SDF") {
            Ok(p) => PathBuf::from(p),
            Err(_) => {
                println!("no campus: pixi run campus makes it");
                return;
            }
        }
    };
    let t = Instant::now();
    let w = World::load(&path, &[], Surfaces::Visual).expect("campus");
    println!(
        "campus: {} triangles in {:?}",
        w.scene.triangle_count(),
        t.elapsed()
    );
    assert!(w.scene.triangle_count() > 100_000);
    let lidar = Pose::new(Vec3::new(0.49, 0.0, 1.52), Rot3::IDENTITY);
    let t = Instant::now();
    let scan = Scan::cast(&LidarSpec::ROOF, &w.scene, &lidar, &mut Noise::new(1), 4);
    println!(
        "a campus scan: {} hits in {:?}",
        scan.hits().count(),
        t.elapsed()
    );
}
