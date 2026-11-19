//! A map folder on disk, as map-drive writes it: loaded, edited, saved to a new folder,
//! and the new folder loaded again.

use std::fs;
use std::path::{Path, PathBuf};

use map_edit::{
    next_free, Align, Cell, CellFill, Edit, EditLog, Editor, HeightBand, LabelClass, LabelsFile,
    MapFolder, Point, Point3, Region, SaveError,
};

/// A fresh folder under the system's temporary folder.
fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("map-edit-test-{}-{name}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).expect("a scratch folder");
    dir
}

/// A small map-drive folder: a wall along y = 10 from x = 0 to 40, a "parked car" box at
/// x 20-22, y 2-6, a 2D map of 0.25 m cells over (-4, -4)..(44, 16), and a path.
fn write_source(dir: &Path) {
    let mut points = Vec::new();
    for i in 0..400 {
        let x = f64::from(i) * 0.1;
        for z in [0.5, 1.0, 1.4] {
            points.push(Point3::new(x, 10.0, z));
        }
    }
    for i in 0..20 {
        for j in 0..40 {
            points.push(Point3::new(
                20.0 + f64::from(i) * 0.1,
                2.0 + f64::from(j) * 0.1,
                1.0,
            ));
        }
    }
    let file = fs::File::create(dir.join("ndt.pcd")).expect("pcd");
    ndt_core::pcd::write(std::io::BufWriter::new(file), &points).expect("written");
    fs::write(
        dir.join("ndt.yaml"),
        "frame: map\nplaced:\n  method: gps\n  slam_to_map:\n    x_m: 0.5\n    y_m: 0.0\n    \
         yaw_deg: 0.0\n  datum:\n  - 45.386\n  - -75.696\npoints: 2000\nvoxel_m: 0.1\n",
    )
    .expect("ndt.yaml");
    let (width, height) = (192usize, 80usize);
    let mut pixels = vec![254u8; width * height];
    // Occupied: the wall's row (y = 10 is row 56 from the bottom), and the car's cells.
    let mut set = |col: usize, row: usize| pixels[(height - 1 - row) * width + col] = 0;
    (16..176).for_each(|col| set(col, 56));
    for col in 96..104 {
        for row in 24..40 {
            set(col, row);
        }
    }
    let mut pgm = format!("P5\n{width} {height}\n255\n").into_bytes();
    pgm.extend(pixels);
    fs::write(dir.join("map2d.pgm"), pgm).expect("pgm");
    fs::write(
        dir.join("map2d.yaml"),
        "# a test map\nimage: map2d.pgm\nmode: trinary\nresolution: 0.25\norigin: [-4.0, -4.0, 0.0]\n\
         negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
    )
    .expect("map2d.yaml");
    fs::write(
        dir.join("poses.tum"),
        "0.0 0.0 0.0 0.0 0 0 0 1\n1.0 5.0 0.0 0.0 0 0 0 1\n",
    )
    .expect("poses");
    fs::write(dir.join("closures.yaml"), "closures: []\n").expect("closures");
}

fn the_car() -> Region {
    Region::from_corners(Point::new(19.5, 1.5), Point::new(22.5, 6.5))
}

#[test]
fn delete_a_car_save_and_load_again() {
    let root = scratch("car");
    let source = root.join("map");
    fs::create_dir_all(&source).expect("source");
    write_source(&source);
    let before = fs::read(source.join("ndt.pcd")).expect("read");

    let map = MapFolder::load(&source).expect("loads");
    assert_eq!(map.points.len(), 2000);
    assert_eq!(map.path.len(), 2);
    assert_eq!(map.datum(), Some([45.386, -75.696]));
    assert_eq!(map.placed_by().as_deref(), Some("gps"));
    let mut ed = Editor::new(map);
    let gone = ed
        .apply(Edit::Delete {
            region: the_car(),
            band: Some(HeightBand::new(0.0, 2.0)),
            cells: CellFill::Unknown,
        })
        .expect("deleted");
    assert_eq!(gone.points_removed, 800);
    ed.apply(Edit::Label {
        class: LabelClass::Building,
        region: Region::from_corners(Point::new(0.0, 9.5), Point::new(40.0, 10.5)),
        band: None,
    })
    .expect("labelled");

    let out = next_free(&source);
    assert_eq!(out, root.join("map-edit-1"));
    let saved = ed.save(&out).expect("saved");
    assert_eq!(saved.points, 1200);
    assert_eq!(saved.points_removed, 800);
    assert_eq!(saved.chunks[0], 1, "one fine chunk: the map is 48 m wide");
    assert!(out.join("chunks/chunks.yaml").exists());
    assert!(out.join("chunks/ndt/0_0.pcd").exists());
    assert!(out.join("closures.yaml").exists());
    assert!(!root.join("map-edit-1.partial").exists());
    // The source is untouched.
    assert_eq!(fs::read(source.join("ndt.pcd")).expect("read"), before);
    assert!(!source.join("edits.yaml").exists());

    // The new folder loads, without the car, with its cells unknown and the label.
    let again = MapFolder::load(&out).expect("loads again");
    assert_eq!(again.points.len(), 1200);
    assert!(again
        .points
        .iter()
        .all(|p| !(19.9..22.1).contains(&p.x) || (p.y - 10.0).abs() < 1e-6));
    let car_cell = again
        .grid
        .index_at(Point::new(21.0, 4.0))
        .expect("on the map");
    assert_eq!(again.grid.cells[car_cell], Cell::Unknown);
    let wall_cell = again
        .grid
        .index_at(Point::new(10.0, 10.1))
        .expect("on the map");
    assert_eq!(again.grid.cells[wall_cell], Cell::Occupied);
    assert_eq!(again.labels.len(), 1);
    assert_eq!(again.labels[0].class, LabelClass::Building);
    assert_eq!(again.datum(), Some([45.386, -75.696]));

    // The log lists both edits and what they did.
    let log: EditLog =
        serde_yaml_ng::from_str(&fs::read_to_string(out.join("edits.yaml")).expect("log"))
            .expect("reads");
    assert_eq!(log.edits.len(), 2);
    assert_eq!(log.edits[0].effect.points_removed, 800);
    assert_eq!((log.points_before, log.points_after), (2000, 1200));
    let ndt_yaml = fs::read_to_string(out.join("ndt.yaml")).expect("ndt.yaml");
    assert!(ndt_yaml.contains("edited:") && ndt_yaml.contains("points: 1200"));

    // Nothing is written over: not the new folder, not the source.
    assert!(matches!(ed.save(&out), Err(SaveError::Exists(_))));
    assert!(matches!(ed.save(&source), Err(SaveError::IsSource(_))));
    assert_eq!(next_free(&source), root.join("map-edit-2"));
    let _ = fs::remove_dir_all(&root);
}

#[test]
fn an_aligned_save_moves_everything_together() {
    let root = scratch("align");
    let source = root.join("map");
    fs::create_dir_all(&source).expect("source");
    write_source(&source);
    let mut ed = Editor::new(MapFolder::load(&source).expect("loads"));
    let align = Align::new(3.3, -1.2, 2f64.to_radians());
    ed.apply(Edit::Align { to: align }).expect("aligned");
    ed.apply(Edit::Label {
        class: LabelClass::ParkedCar,
        region: the_car(),
        band: None,
    })
    .expect("labelled");
    let out = root.join("aligned");
    ed.save(&out).expect("saved");
    let again = MapFolder::load(&out).expect("loads");

    // The 2D map's origin is on whole metres, for NDT's 1 m cubes in the chunks.
    assert_eq!(again.grid.origin.x.fract(), 0.0);
    assert_eq!(again.grid.origin.y.fract(), 0.0);
    // A wall point and its cell moved the same way.
    let wall = align.apply(Point::new(10.0, 10.0));
    assert!(again
        .points
        .iter()
        .any(|p| (p.x - wall.x).abs() < 1e-3 && (p.y - wall.y).abs() < 1e-3));
    // New cells take the old cell under their middle: the wall is within a cell.
    let near_wall = (-1..=1)
        .flat_map(|dx| (-1..=1).map(move |dy| (f64::from(dx) * 0.25, f64::from(dy) * 0.25)))
        .filter_map(|(dx, dy)| again.grid.index_at(Point::new(wall.x + dx, wall.y + dy)))
        .any(|i| again.grid.cells[i] == Cell::Occupied);
    assert!(near_wall);
    // The path and the label too.
    let start = align.apply(Point::new(5.0, 0.0));
    assert!((again.path[1].xyz[0] - start.x).abs() < 1e-3);
    assert!((again.path[1].yaw() - align.yaw_rad).abs() < 1e-6);
    let labels: LabelsFile =
        serde_yaml_ng::from_str(&fs::read_to_string(out.join("labels.yaml")).expect("labels"))
            .expect("reads");
    let c = align.apply(the_car().centre());
    assert!((labels.labels[0].region.centre_m[0] - c.x).abs() < 1e-9);
    // The YAML says how it moved; slam_to_map follows (0.5 m east, then the move).
    let ndt: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(&fs::read_to_string(out.join("ndt.yaml")).expect("yaml"))
            .expect("reads");
    let by_hand = &ndt["placed"]["aligned_by_hand"];
    assert_eq!(by_hand["x_m"].as_f64(), Some(3.3));
    assert_eq!(by_hand["yaw_deg"].as_f64(), Some(2.0));
    let s2m = align.apply(Point::new(0.5, 0.0));
    let got = ndt["placed"]["slam_to_map"]["x_m"].as_f64().expect("x");
    assert!((got - s2m.x).abs() < 1e-3);
    let _ = fs::remove_dir_all(&root);
}

#[test]
fn a_missing_map_says_which_file() {
    let root = scratch("missing");
    let err = MapFolder::load(&root).expect_err("no files");
    assert!(err.to_string().contains("ndt.pcd"));
    let _ = fs::remove_dir_all(&root);
}
