//! `aav_things`' tests, ported: test_things.py, test_background.py, test_naming.py,
//! test_tf_wait.py, test_looks.py. Their random runs use a small generator of our own;
//! the Python's exact runs are in `tests/vectors.rs`.

use std::{collections::HashMap, collections::HashSet, f64::consts::PI, fs, path::PathBuf};

use crate::{
    background::{new_only, Background, ChunkBackground},
    groups::{group, to_frame, Group, GroupShape},
    looks,
    naming::{best_label, iou, project, thing_box, Axes, Camera, CameraMount, Lens, Namer, Seen},
    pairwise_sum,
    radar::{clusters, RadarPoint},
    tf_wait::{decide, Look, Step, WaitShape, Waiting},
    tracker::{
        accel_covariance, fitted_speed, ground_radial_speed, predict, RadarCluster, Tracker,
        TrackerShape,
    },
    Pose2, UNKNOWN,
};
use nalgebra::{Matrix4, Vector4};

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

/// A small, fixed random number generator (SplitMix64), so the runs repeat.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        (z >> 11) as f64 / (1u64 << 53) as f64
    }

    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next()
    }

    /// Roughly normal: the sum of 12 uniforms, less 6.
    fn normal(&mut self, sigma: f64) -> f64 {
        sigma * ((0..12).map(|_| self.next()).sum::<f64>() - 6.0)
    }
}

/// The cube centres aav_voxels would send for an upright box.
fn cubes(x: f64, y: f64, length: f64, width: f64, height: f64) -> Vec<[f64; 3]> {
    let size = 0.25;
    let range = |from: f64, to: f64| {
        (0..)
            .map(move |i| from + f64::from(i) * size)
            .take_while(move |v| *v < to)
    };
    let mut out = Vec::new();
    for a in range(x - length / 2.0 + size / 2.0, x + length / 2.0) {
        for b in range(y - width / 2.0 + size / 2.0, y + width / 2.0) {
            for c in range(0.3 + size / 2.0, height) {
                out.push([a, b, c]);
            }
        }
    }
    out
}

fn unit_box(x: f64, y: f64) -> Vec<[f64; 3]> {
    cubes(x, y, 1.0, 1.0, 1.0)
}

fn seen(x: f64, y: f64) -> Group {
    Group {
        x,
        y,
        length: 1.0,
        width: 1.0,
        height: 1.0,
        cubes: 16,
    }
}

const HERE: [f64; 2] = [0.0, 0.0];

// --- grouping ---------------------------------------------------------------------------

#[test]
fn two_boxes_apart_are_two_groups() {
    let found = group(
        &[unit_box(5.0, 0.0), unit_box(5.0, 3.0)].concat(),
        &GroupShape::default(),
    );
    assert_eq!(found.len(), 2);
    assert!(close(found[0].x, 5.0, 1e-9) && close(found[0].y, 0.0, 1e-9));
    assert!(close(found[1].x, 5.0, 1e-9) && close(found[1].y, 3.0, 1e-9));
    assert!(found
        .iter()
        .all(|g| close(g.length, 1.0, 1e-9) && close(g.width, 1.0, 1e-9)));
}

#[test]
fn boxes_closer_than_the_gap_are_one() {
    let shape = GroupShape::default();
    let found = group(&[unit_box(5.0, 0.0), unit_box(5.0, 1.25)].concat(), &shape);
    assert_eq!(found.len(), 1);
    assert!(close(found[0].width, 2.25, 1e-9));
    let apart = group(&[unit_box(5.0, 0.0), unit_box(5.0, 1.5)].concat(), &shape);
    assert_eq!(apart.len(), 2);
}

#[test]
fn a_lone_cube_is_noise_and_a_wall_is_not_a_thing() {
    let mut points = cubes(10.0, 0.0, 0.25, 20.0, 1.0);
    points.push([3.0, 3.0, 0.5]);
    assert!(group(&points, &GroupShape::default()).is_empty());
}

#[test]
fn nothing_and_nans_give_nothing() {
    let shape = GroupShape::default();
    assert!(group(&[], &shape).is_empty());
    assert!(group(&[[f64::NAN, 0.0, 0.5], [1.0, f64::INFINITY, 0.5]], &shape).is_empty());
}

#[test]
fn to_frame_turns_the_middle_and_the_outline() {
    let g = Group {
        x: 2.0,
        y: 0.0,
        length: 2.0,
        width: 1.0,
        height: 1.0,
        cubes: 4,
    };
    let moved = to_frame(
        &[g],
        Pose2 {
            x: 10.0,
            y: 5.0,
            yaw: PI / 2.0,
        },
    )[0];
    assert!(close(moved.x, 10.0, 1e-12) && close(moved.y, 7.0, 1e-12));
    assert!(close(moved.length, 1.0, 1e-12) && close(moved.width, 2.0, 1e-12));
}

// --- the filter -------------------------------------------------------------------------

#[test]
fn predict_moves_on_its_speed_and_grows_the_doubt() {
    let (x, p) = predict(
        &Vector4::new(0.0, 0.0, 1.0, -2.0),
        &(Matrix4::identity() * 0.01),
        0.5,
        1.0,
    );
    assert_eq!(x, Vector4::new(0.5, -1.0, 1.0, -2.0));
    assert!(p[(0, 0)] > 0.01 && p[(2, 2)] > 0.01);
}

#[test]
fn a_moving_thing_gets_its_speed() {
    let mut rng = Rng(1);
    let mut tracker = Tracker::default();
    for k in 0..40 {
        let t = f64::from(k) * 0.1;
        let (nx, ny) = (rng.normal(0.1), rng.normal(0.1));
        tracker.step(t, &[seen(5.0 + 1.2 * t + nx, -1.0 + 0.4 * t + ny)], HERE);
    }
    let things = tracker.at(3.9);
    assert_eq!(things.len(), 1);
    let thing = &things[0];
    assert!(
        close(thing.x[2], 1.2, 0.15) && close(thing.x[3], 0.4, 0.15),
        "{:?}",
        thing.x
    );
    assert_eq!(
        (thing.id, thing.seen, thing.kind.as_str()),
        (1, 40, UNKNOWN)
    );
    let later = &tracker.at(4.4)[0];
    assert!(close(later.x[0] - thing.x[0], 0.6, 0.1));
}

#[test]
fn two_things_keep_their_ids() {
    let mut tracker = Tracker::default();
    for k in 0..30 {
        let t = f64::from(k) * 0.1;
        tracker.step(t, &[seen(10.0 - t, 2.0), seen(10.0 + t, -2.0)], HERE);
    }
    let things: HashMap<u32, _> = tracker.at(2.9).into_iter().map(|t| (t.id, t)).collect();
    assert_eq!(things.len(), 2);
    assert!(close(things[&1].x[1], 2.0, 0.1));
    assert!(close(things[&1].x[2], -1.0, 0.15));
    assert!(close(things[&2].x[2], 1.0, 0.15));
}

#[test]
fn a_flash_is_never_listed_and_soon_forgotten() {
    let mut tracker = Tracker::default();
    tracker.step(0.0, &[seen(5.0, 0.0)], HERE);
    assert!(tracker.at(0.0).is_empty());
    tracker.step(0.5, &[], HERE);
    assert!(tracker.tracks.is_empty());
}

#[test]
fn memory_far_away_lasts_keep_and_in_the_blind_zone_longer() {
    let mut tracker = Tracker::new(TrackerShape {
        keep: 3.0,
        keep_blind: 30.0,
        blind_range: 4.0,
        ..TrackerShape::default()
    });
    for k in 0..10 {
        tracker.step(f64::from(k) * 0.1, &[seen(2.0, 0.0), seen(15.0, 5.0)], HERE);
    }
    for k in 10..60 {
        tracker.step(f64::from(k) * 0.1, &[], HERE);
    }
    let left = tracker.at(5.9);
    assert_eq!(left.len(), 1);
    assert!(left[0].coasting);
    assert!(close(left[0].x[0], 2.0, 0.05) && close(left[0].x[1], 0.0, 0.05));
    tracker.expire(32.0, HERE);
    assert!(tracker.tracks.is_empty());
}

#[test]
fn a_coasting_mover_keeps_moving() {
    let mut tracker = Tracker::default();
    for k in 0..30 {
        let t = f64::from(k) * 0.1;
        tracker.step(t, &[seen(10.0 + 2.0 * t, 0.0)], HERE);
    }
    tracker.step(3.5, &[], HERE);
    let thing = &tracker.at(4.0)[0];
    assert!(thing.coasting);
    assert!(close(thing.x[0], 18.0, 0.3));
}

#[test]
fn an_older_cloud_is_dropped() {
    let mut tracker = Tracker::default();
    tracker.step(1.0, &[seen(5.0, 0.0)], HERE);
    tracker.step(0.9, &[seen(50.0, 0.0)], HERE);
    assert_eq!(tracker.tracks.len(), 1);
    assert_eq!(tracker.time, Some(1.0));
}

// --- the radar --------------------------------------------------------------------------

#[test]
fn ground_radial_speed_takes_the_car_off() {
    assert!(close(
        ground_radial_speed(-2.0, [1.0, 0.0], [2.0, 0.0]),
        0.0,
        1e-12
    ));
    let ray = [0.5, 3.0_f64.sqrt() / 2.0];
    assert!(close(
        ground_radial_speed(-1.0, ray, [2.0, 0.0]),
        0.0,
        1e-12
    ));
}

#[test]
fn radar_speeds_up_a_new_things_speed() {
    let (mut with_radar, mut without) = (Tracker::default(), Tracker::default());
    for k in 0..4 {
        let t = f64::from(k) * 0.1;
        for tracker in [&mut with_radar, &mut without] {
            tracker.step(t, &[seen(10.0 + 3.0 * t, 0.0)], HERE);
        }
        with_radar.radar(
            t,
            &[RadarCluster {
                x: 10.0 + 3.0 * t,
                y: 0.0,
                ray_x: 1.0,
                ray_y: 0.0,
                speed: 3.0,
            }],
        );
    }
    let (fast, slow) = (&with_radar.at(0.3)[0], &without.at(0.3)[0]);
    assert!(fast.radar && !slow.radar);
    assert!((fast.x[2] - 3.0).abs() < (slow.x[2] - 3.0).abs());
    assert!(close(fast.x[2], 3.0, 0.2));
}

#[test]
fn the_naming_hook() {
    let mut tracker = Tracker::default();
    for k in 0..3 {
        tracker.step(f64::from(k) * 0.1, &[seen(5.0, 0.0)], HERE);
        tracker.rename(|t| if t.height < 2.0 { "person" } else { UNKNOWN }.to_string());
    }
    assert_eq!(tracker.at(0.2)[0].kind, "person");
}

#[test]
fn radar_clusters_are_their_points_middles_in_odom() {
    let p = |x, y, velocity, cluster| RadarPoint {
        x,
        y,
        velocity,
        cluster,
    };
    let points = [
        p(10.0, 1.0, -2.0, 4.0),
        p(10.0, -1.0, -2.0, 4.0),
        p(0.01, 0.0, 0.0, 1.0), // on the radar: no ray
        p(f64::NAN, 0.0, 0.0, 2.0),
    ];
    let radar = Pose2 {
        x: 1.0,
        y: 2.0,
        yaw: PI / 2.0,
    };
    let found = clusters(&points, radar, [0.0, 2.0]);
    assert_eq!(found.len(), 1);
    let c = found[0];
    assert!(close(c.x, 1.0, 1e-12) && close(c.y, 12.0, 1e-12));
    assert!(close(c.ray_x, 0.0, 1e-12) && close(c.ray_y, 1.0, 1e-12));
    assert!(close(c.speed, 0.0, 1e-12)); // the car's 2 m/s along the ray taken off
}

/// The ids that moved 0.3 m/s or more for 1.5 s on end (the behaviour's hold).
fn held_tracks(tracker: &mut Tracker, frames: &[(f64, Vec<Group>, [f64; 2])]) -> HashSet<u32> {
    let mut since: HashMap<u32, f64> = HashMap::new();
    let mut held = HashSet::new();
    for (now, groups, car) in frames {
        tracker.step(*now, groups, *car);
        for t in tracker.at(*now) {
            if t.x[2].hypot(t.x[3]) >= 0.3 {
                if now - *since.entry(t.id).or_insert(*now) >= 1.5 {
                    held.insert(t.id);
                }
            } else {
                since.remove(&t.id);
            }
        }
    }
    held
}

#[test]
fn a_row_of_thin_posts_never_moves() {
    // TODO 222: posts 1.5 m apart, each missed a third of the time, the car passing at
    // 2 m/s. The gates keep each post still.
    for seed in 1..=10 {
        let mut rng = Rng(seed * 7919);
        let posts: Vec<(f64, f64)> = (0..12).map(|i| (5.0 + 1.5 * f64::from(i), 4.0)).collect();
        let frames: Vec<_> = (0..200)
            .map(|k| {
                let (now, car) = (0.1 * f64::from(k), [0.2 * f64::from(k) - 5.0, 0.0]);
                let groups = posts
                    .iter()
                    .filter_map(|&(px, py)| {
                        let g = Group {
                            x: px + rng.uniform(-0.15, 0.15),
                            y: py + rng.uniform(-0.15, 0.15),
                            length: 0.25 + if rng.next() < 0.5 { 0.0 } else { 0.25 },
                            width: 0.25 + if rng.next() < 0.5 { 0.0 } else { 0.25 },
                            height: 1.0,
                            cubes: 2,
                        };
                        let near = (px - car[0]).hypot(py - car[1]) <= 25.0;
                        (near && rng.next() > 0.35).then_some(g)
                    })
                    .collect();
                (now, groups, car)
            })
            .collect();
        assert!(
            held_tracks(&mut Tracker::default(), &frames).is_empty(),
            "seed {seed}"
        );
    }
}

#[test]
fn a_walker_and_a_car_still_get_their_speed() {
    let mut rng = Rng(5);
    let frames: Vec<_> = (0..60)
        .map(|k| {
            let now = 0.1 * f64::from(k);
            let mut groups = Vec::new();
            if rng.next() > 0.3 {
                groups.push(Group {
                    x: 10.0,
                    y: -5.0 + 1.4 * now,
                    length: 0.5,
                    width: 0.5,
                    height: 1.7,
                    cubes: 4,
                });
            }
            if rng.next() > 0.3 {
                groups.push(Group {
                    x: -20.0 + 8.0 * now,
                    y: 8.0,
                    length: 4.5,
                    width: 2.0,
                    height: 1.5,
                    cubes: 30,
                });
            }
            (now, groups, HERE)
        })
        .collect();
    let mut tracker = Tracker::default();
    assert_eq!(held_tracks(&mut tracker, &frames).len(), 2);
    let mut speeds: Vec<f64> = tracker
        .at(6.0)
        .iter()
        .map(|t| t.x[2].hypot(t.x[3]))
        .collect();
    speeds.sort_by(f64::total_cmp);
    assert!(close(speeds[0], 1.4, 0.3) && close(speeds[1], 8.0, 0.3));
}

// --- the background ---------------------------------------------------------------------

/// A 20 × 20 m map at 0.25 m from (-10, -10): free, with a wall along x = 5.
fn lot() -> Background {
    let mut data = vec![0i8; 80 * 80];
    for row in 0..80 {
        data[row * 80 + 60] = 100;
    }
    Background::from_grid(&data, 80, 80, 0.25, [-10.0, -10.0], 0.25).unwrap()
}

const AT_ZERO: Pose2 = Pose2 {
    x: 0.0,
    y: 0.0,
    yaw: 0.0,
};

#[test]
fn a_cube_on_the_wall_drops_and_one_in_the_open_stays() {
    let kept = new_only(&[[5.1, 0.0, 1.0], [2.0, 0.0, 1.0]], AT_ZERO, &lot(), None);
    assert_eq!(kept, vec![[2.0, 0.0, 1.0]]);
}

#[test]
fn the_slack_takes_a_small_pose_error() {
    let bg = lot();
    assert!(new_only(&[[4.85, 3.0, 1.0]], AT_ZERO, &bg, None).is_empty());
    assert_eq!(new_only(&[[4.4, 3.0, 1.0]], AT_ZERO, &bg, None).len(), 1);
}

#[test]
fn the_car_pose_moves_the_cubes_onto_the_map() {
    let bg = lot();
    let cube = [[0.0, -2.1, 1.0]];
    let facing = |x| Pose2 {
        x,
        y: 0.0,
        yaw: PI / 2.0,
    };
    assert!(new_only(&cube, facing(3.0), &bg, None).is_empty());
    assert_eq!(new_only(&cube, facing(0.0), &bg, None).len(), 1);
}

#[test]
fn off_the_map_is_never_background() {
    let bg = lot();
    assert_eq!(new_only(&[[50.0, 50.0, 1.0]], AT_ZERO, &bg, None).len(), 1);
    assert!(new_only(&[], AT_ZERO, &bg, None).is_empty());
    assert!(Background::from_grid(&[0; 3], 2, 2, 0.25, [0.0, 0.0], 0.25).is_none());
}

#[test]
fn points_make_a_background_with_slack() {
    let wall: Vec<[f64; 2]> = (0..60).map(|i| [5.1, -3.0 + 0.1 * f64::from(i)]).collect();
    let bg = Background::from_points(&wall, [-10.0, -10.0], 20.0, 0.25, 0.25);
    let points = [
        [5.1, 0.0, 1.0],
        [4.85, 1.0, 1.0],
        [4.3, 0.0, 1.0],
        [5.1, 5.0, 1.0],
    ];
    let kept = new_only(&points, AT_ZERO, &bg, None);
    assert_eq!(kept, vec![[4.3, 0.0, 1.0], [5.1, 5.0, 1.0]]);
}

#[test]
fn walls_grow_by_a_square_of_reach_cells_to_the_edges() {
    // TODO 195: the growing goes row by row on many cores; the answer is the square
    // round each wall cell, checked here cell by cell on a random grid.
    let (w, h, reach) = (37, 23, 2_i64);
    let mut seed = 12_345_u64;
    let data: Vec<i8> = (0..w * h)
        .map(|_| {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            if (seed >> 33).is_multiple_of(17) {
                100
            } else {
                0
            }
        })
        .collect();
    let bg = Background::from_grid(&data, w, h, 0.25, [0.0, 0.0], 0.5).expect("a grid");
    for r in 0..h as i64 {
        for c in 0..w as i64 {
            let want = (r - reach..=r + reach).any(|rr| {
                (c - reach..=c + reach).any(|cc| {
                    (0..h as i64).contains(&rr)
                        && (0..w as i64).contains(&cc)
                        && data[rr as usize * w + cc as usize] >= 65
                })
            });
            let (x, y) = (0.25 * (c as f64 + 0.5), 0.25 * (r as f64 + 0.5));
            assert_eq!(bg.on_map(x, y), want, "cell ({r}, {c})");
        }
    }
}

#[test]
fn below_drops_only_the_low_cubes_on_the_map() {
    let points = [[5.1, 0.0, 0.4], [5.1, 0.0, 1.2], [2.0, 0.0, 0.4]];
    let kept = new_only(&points, AT_ZERO, &lot(), Some(0.7));
    assert_eq!(kept, vec![[5.1, 0.0, 1.2], [2.0, 0.0, 0.4]]);
}

fn write_pcd(path: &std::path::Path, points: &[[f32; 3]]) {
    let header = format!(
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n\
         WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n",
        n = points.len()
    );
    let mut bytes = header.into_bytes();
    for p in points {
        for v in p {
            bytes.extend(v.to_le_bytes());
        }
    }
    fs::write(path, bytes).unwrap();
}

#[test]
fn chunks_load_round_the_car_and_again_in_a_new_chunk() {
    let top: PathBuf = std::env::temp_dir().join(format!("things-chunks-{}", std::process::id()));
    let _ = fs::remove_dir_all(&top);
    let folder = top.join("ndt");
    fs::create_dir_all(&folder).unwrap();
    fs::write(top.join("chunks.yaml"), "origin: [-64, -64]\nsize: 64\n").unwrap();
    // A wall in chunk (1, 1), a far wall in chunk (4, 1), and the ground.
    let mut near: Vec<[f32; 3]> = (0..20).map(|y| [10.0, y as f32, 1.0]).collect();
    near.push([30.0, 5.0, 0.0]);
    write_pcd(&folder.join("1_1.pcd"), &near);
    write_pcd(&folder.join("4_1.pcd"), &[[200.0, 5.0, 1.0]]);
    let mut chunks = ChunkBackground::open(&folder, 0.25, 0.3).unwrap();
    let points = [[10.0, 5.0, 1.0], [30.0, 5.0, 1.0], [200.0, 5.0, 1.0]];
    let kept = new_only(&points, AT_ZERO, chunks.update(5.0, 5.0).unwrap(), None);
    assert_eq!(kept, vec![[30.0, 5.0, 1.0], [200.0, 5.0, 1.0]]);
    chunks.update(20.0, 30.0).unwrap(); // the same chunk
    assert_eq!(chunks.loads, 1);
    let bg = chunks.update(170.0, 5.0).unwrap(); // chunk (3, 1)
    let kept = new_only(&points, AT_ZERO, bg, None);
    assert_eq!(chunks.loads, 2);
    assert_eq!(kept, vec![[10.0, 5.0, 1.0], [30.0, 5.0, 1.0]]);
    let _ = fs::remove_dir_all(&top);
}

// --- names ------------------------------------------------------------------------------

const LENS: Lens = Lens {
    fx: 500.0,
    fy: 500.0,
    cx: 320.0,
    cy: 240.0,
    width: 640,
    height: 480,
};
const FRONT: CameraMount = CameraMount {
    position: [1.75, 0.0, 1.55],
    rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    axes: Axes::Body,
};
const OPTICAL: CameraMount = CameraMount {
    position: [1.75, 0.0, 1.55],
    rotation: [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
    axes: Axes::Optical,
};

fn box_seen(label: &str, area: [f64; 4]) -> Seen {
    Seen {
        label: label.to_string(),
        score: 0.8,
        area,
    }
}

#[test]
fn a_point_ahead_lands_where_the_pinhole_says() {
    let [u, v] = project(&LENS, &FRONT, [6.0, 0.0, 1.55]).unwrap();
    assert!(close(u, 320.0, 1e-9) && close(v, 240.0, 1e-9));
    assert!(project(&LENS, &FRONT, [6.0, 1.0, 1.55]).unwrap()[0] < 320.0);
    assert!(project(&LENS, &FRONT, [1.0, 0.0, 1.0]).is_none());
}

#[test]
fn an_optical_frame_projects_the_same() {
    for p in [[6.0, 0.5, 0.2], [10.0, -2.0, 1.0]] {
        let (a, b) = (
            project(&LENS, &OPTICAL, p).unwrap(),
            project(&LENS, &FRONT, p).unwrap(),
        );
        assert!(close(a[0], b[0], 1e-9) && close(a[1], b[1], 1e-9));
    }
    assert_eq!(Axes::of_frame("camera_optical_frame"), Axes::Optical);
    assert_eq!(Axes::of_frame("camera_left_link"), Axes::Body);
}

#[test]
fn a_person_ahead_is_named_by_the_box_round_them() {
    let person = thing_box(&LENS, &FRONT, 6.0, 0.0, 0.5, 0.5, 1.8).unwrap();
    let [u0, v0, u1, v1] = person;
    assert!(u0 < 320.0 && 320.0 < u1 && v0 < 240.0 && 240.0 < v1);
    let seen = [
        box_seen("car", [500.0, 300.0, 640.0, 480.0]),
        box_seen("person", [u0 + 3.0, v0 - 5.0, u1 + 2.0, v1 + 4.0]),
    ];
    assert_eq!(best_label(&person, &seen, 0.3, None), Some("person"));
    assert_eq!(best_label(&person, &seen[..1], 0.3, None), None);
}

#[test]
fn intersection_over_union() {
    assert!(close(
        iou(&[0.0, 0.0, 10.0, 10.0], &[0.0, 0.0, 10.0, 10.0]),
        1.0,
        1e-12
    ));
    assert!(close(
        iou(&[0.0, 0.0, 10.0, 10.0], &[5.0, 0.0, 15.0, 10.0]),
        1.0 / 3.0,
        1e-12
    ));
    assert_eq!(iou(&[0.0, 0.0, 1.0, 1.0], &[2.0, 2.0, 3.0, 3.0]), 0.0);
}

#[test]
fn a_name_is_held_a_while_then_let_go() {
    let mut namer = Namer::default();
    let area = thing_box(&LENS, &FRONT, 6.0, 0.0, 0.5, 0.5, 1.8).unwrap();
    namer.set_camera(
        "front",
        Camera {
            lens: LENS,
            mount: FRONT,
            seen: vec![box_seen("person", area)],
            stamp: 10.0,
        },
    );
    assert_eq!(namer.name(7, 6.0, 0.0, 0.5, 0.5, 1.8, 10.1), "person");
    assert_eq!(namer.name(7, 6.0, 0.0, 0.5, 0.5, 1.8, 12.0), "person");
    assert_eq!(namer.name(7, 6.0, 0.0, 0.5, 0.5, 1.8, 13.5), UNKNOWN);
    assert_eq!(namer.name(8, 6.0, 4.0, 0.5, 0.5, 1.8, 10.1), UNKNOWN);
}

#[test]
fn a_stop_sign_over_the_lidars_ceiling_is_named_on_its_bearing() {
    let pole = thing_box(&LENS, &FRONT, 10.0, -2.0, 0.25, 0.25, 2.125).unwrap();
    let corners: Vec<[f64; 2]> = [-0.375, 0.375]
        .iter()
        .flat_map(|dy| [1.6, 2.4].map(|z| project(&LENS, &FRONT, [10.0, -2.0 + dy, z]).unwrap()))
        .collect();
    let lo = |i: usize| corners.iter().map(|c| c[i]).fold(f64::INFINITY, f64::min);
    let hi = |i: usize| {
        corners
            .iter()
            .map(|c| c[i])
            .fold(f64::NEG_INFINITY, f64::max)
    };
    let plate = [lo(0), lo(1), hi(0), hi(1)];
    let seen = vec![box_seen("stop sign", plate)];
    assert!(iou(&pole, &plate) < 0.3);
    assert_eq!(best_label(&pole, &seen, 0.3, None), None);
    assert_eq!(best_label(&pole, &seen, 0.3, Some(0.5)), Some("stop sign"));
    let mut namer = Namer::default();
    namer.set_camera(
        "front",
        Camera {
            lens: LENS,
            mount: FRONT,
            seen,
            stamp: 10.0,
        },
    );
    assert_eq!(
        namer.name(1, 10.0, -2.0, 0.25, 0.25, 2.125, 10.1),
        "stop sign"
    );
    assert_eq!(namer.name(2, 10.0, -2.0, 0.25, 0.25, 1.5, 10.1), UNKNOWN);
    assert_eq!(namer.name(3, 10.0, 1.0, 0.25, 0.25, 2.125, 10.1), UNKNOWN);
}

#[test]
fn a_lens_needs_its_focal_lengths() {
    let k = [343.0, 0.0, 320.0, 0.0, 343.0, 180.0, 0.0, 0.0, 1.0];
    assert_eq!(Lens::from_k(&k, 640, 360).unwrap().cy, 180.0);
    assert!(Lens::from_k(&[0.0; 9], 640, 360).is_none());
}

// --- clouds waiting for TF --------------------------------------------------------------

const WAIT: WaitShape = WaitShape {
    tolerance: 0.1,
    max_wait: 1.5,
    size: 3,
};

fn look(found: bool, newest: Option<f64>) -> Look {
    Look { found, newest }
}

#[test]
fn a_cloud_with_its_transform_is_used_at_its_stamp() {
    assert_eq!(decide(10.0, look(true, None), 10.0, &WAIT), Step::AtStamp);
}

#[test]
fn a_hair_newer_than_tf_uses_the_newest_transform() {
    assert_eq!(
        decide(10.05, look(false, Some(10.0)), 10.05, &WAIT),
        Step::Newest
    );
}

#[test]
fn the_seen_warning_waits_then_drops() {
    let l = look(false, Some(90.32));
    assert_eq!(decide(91.5, l, 91.5, &WAIT), Step::Wait);
    assert_eq!(decide(91.5, l, 92.9, &WAIT), Step::Wait);
    assert_eq!(decide(91.5, l, 93.1, &WAIT), Step::Drop);
}

#[test]
fn no_transform_yet_waits_then_drops() {
    assert_eq!(decide(5.0, look(false, None), 5.5, &WAIT), Step::Wait);
    assert_eq!(decide(5.0, look(false, None), 7.0, &WAIT), Step::Drop);
}

#[test]
fn older_than_the_buffer_is_dropped_at_once() {
    assert_eq!(
        decide(1.0, look(false, Some(20.0)), 20.0, &WAIT),
        Step::Drop
    );
}

#[test]
fn clouds_come_out_oldest_first_once_tf_catches_up() {
    let mut waiting = Waiting::new(WAIT);
    for stamp in [1.0, 1.1, 1.2] {
        waiting.add(stamp, format!("cloud {stamp}"));
    }
    let tf = |until: f64| move |stamp: f64, _: &String| look(stamp <= until, Some(until));
    assert!(waiting.take(tf(0.5), 1.2).is_empty());
    assert_eq!(waiting.len(), 3);
    let ready = waiting.take(tf(1.1), 1.3);
    assert_eq!(
        ready,
        vec![
            ("cloud 1".to_string(), Step::AtStamp),
            ("cloud 1.1".to_string(), Step::AtStamp),
            ("cloud 1.2".to_string(), Step::Newest),
        ]
    );
    assert!(waiting.is_empty() && waiting.dropped == 0);
}

#[test]
fn a_full_queue_drops_its_oldest_and_counts() {
    let mut waiting = Waiting::new(WAIT);
    for stamp in [1.0, 1.1, 1.2, 1.3] {
        waiting.add(stamp, stamp);
    }
    assert_eq!(waiting.stamps().collect::<Vec<_>>(), vec![1.1, 1.2, 1.3]);
    assert_eq!(waiting.dropped, 1);
}

#[test]
fn a_stale_cloud_is_dropped_and_the_next_still_waits() {
    let mut waiting = Waiting::new(WAIT);
    waiting.add(1.0, "old");
    waiting.add(2.5, "new");
    let ready = waiting.take(|_, _| look(false, Some(0.0)), 2.6);
    assert!(ready.is_empty() && waiting.dropped == 1);
    assert_eq!(waiting.stamps().collect::<Vec<_>>(), vec![2.5]);
}

// --- looks ------------------------------------------------------------------------------

#[test]
fn an_unknown_thing_shows_only_its_id() {
    assert_eq!(looks::caption(4, "unknown", 1.0, 0.0, true), "4 1.0 m/s R");
    assert_eq!(looks::caption(4, "", 0.0, 0.0, false), "4 0.0 m/s");
}

#[test]
fn a_named_thing_shows_its_name_first() {
    assert_eq!(
        looks::caption(3, "person", 0.6, 0.8, false),
        "person 3 1.0 m/s"
    );
    assert_eq!(
        looks::caption(7, "stop sign", 0.0, 0.0, false),
        "stop sign 7 0.0 m/s"
    );
}

#[test]
fn each_kind_has_its_colour() {
    assert_eq!(looks::colour("person", false), looks::PERSON);
    assert_eq!(looks::colour("Stop Sign", false), looks::STOP_SIGN);
    for v in looks::VEHICLES {
        assert_eq!(looks::colour(v, false), looks::VEHICLE);
    }
    assert_eq!(looks::colour("bicycle", false), looks::OTHER);
    assert_eq!(looks::colour("unknown", false), looks::UNNAMED);
    assert_eq!(looks::colour("", false), looks::UNNAMED);
    assert_eq!(looks::colour("person", true), looks::COASTING);
}

// --- numpy's sum ------------------------------------------------------------------------

#[test]
fn the_pairwise_sum_adds_as_numpy_does() {
    // Few: one after the other. Many: eight running sums, then the rest.
    assert_eq!(pairwise_sum(&[]), -0.0);
    assert_eq!(pairwise_sum(&[1.0, 2.0, 3.0]), 6.0);
    let many: Vec<f64> = (0..300).map(|i| f64::from(i) * 0.1).collect();
    assert!(close(pairwise_sum(&many), 4485.0, 1e-9));
}

// --- a motion model per kind (TODO 217) ---

fn per_kind() -> TrackerShape {
    TrackerShape {
        per_kind: true,
        ..TrackerShape::default()
    }
}

#[test]
fn a_cars_noise_is_long_along_its_heading_and_narrow_across() {
    let sh = per_kind();
    // Driving along x at 5 m/s: 2² along, 0.5² across.
    let [xx, xy, yy] = accel_covariance("car", 5.0, 0.0, &sh);
    assert!(close(xx, 4.0, 1e-12) && close(xy, 0.0, 1e-12) && close(yy, 0.25, 1e-12));
    // Along the diagonal: the same, turned.
    let [xx, xy, yy] = accel_covariance("truck", 3.0, 3.0, &sh);
    assert!(close(xx, 2.125, 1e-12) && close(xy, 1.875, 1e-12) && close(yy, 2.125, 1e-12));
    // Too slow for a heading: round. A person: round and wider; unknown: the champion's.
    assert_eq!(accel_covariance("car", 0.5, 0.0, &sh), [4.0, 0.0, 4.0]);
    assert_eq!(accel_covariance("person", 1.0, 0.0, &sh), [4.0, 0.0, 4.0]);
    assert_eq!(accel_covariance(UNKNOWN, 1.0, 0.0, &sh), [2.25, 0.0, 2.25]);
}

#[test]
fn a_coasting_car_stays_in_its_lane_and_a_person_spreads_round() {
    let mut cars = Tracker::new(per_kind());
    for k in 0..30 {
        let t = f64::from(k) * 0.1;
        cars.step(t, &[seen(10.0 + 5.0 * t, 0.0)], [0.0, 0.0]);
        cars.rename(|_| "car".to_string());
    }
    let car = &cars.at(4.0)[0];
    // Unseen for a second: sure of its place across the road, less along it.
    assert!(car.p[(1, 1)] < 0.5 * car.p[(0, 0)], "{:?}", car.p);
    let mut people = Tracker::new(per_kind());
    for k in 0..30 {
        let t = f64::from(k) * 0.1;
        people.step(t, &[seen(10.0 + 1.2 * t, 0.0)], [0.0, 0.0]);
        people.rename(|_| "person".to_string());
    }
    let person = &people.at(4.0)[0];
    assert!(
        close(person.p[(1, 1)] / person.p[(0, 0)], 1.0, 0.2),
        "{:?}",
        person.p
    );
}

#[test]
fn a_thing_found_again_refits_its_speed_from_its_sightings() {
    // Seen moving at 1 m/s, then hidden 1 s while it turned: found again moving along y.
    let mut tracker = Tracker::new(per_kind());
    for k in 0..10 {
        let t = f64::from(k) * 0.1;
        tracker.step(t, &[seen(10.0 + t, 0.0)], [0.0, 0.0]);
    }
    for k in 0..3 {
        let t = 2.0 + f64::from(k) * 0.1;
        tracker.step(t, &[seen(11.0, 0.5 + (t - 2.0))], [0.0, 0.0]);
    }
    let thing = &tracker.tracks[0];
    assert_eq!(thing.sightings.len(), 5);
    // The fit runs through the last 5 places; its speed points up the y axis now.
    let (v, var) = fitted_speed(&thing.sightings, 0.15).expect("a fit");
    assert!(var > 0.0 && v[1] > 0.0);
    // And without per_kind the champion keeps its Kalman speed (no re-fit).
    let mut champion = Tracker::default();
    for k in 0..10 {
        let t = f64::from(k) * 0.1;
        champion.step(t, &[seen(10.0 + t, 0.0)], [0.0, 0.0]);
    }
    champion.step(2.0, &[seen(11.0, 0.5)], [0.0, 0.0]);
    let mut refit = Tracker::new(per_kind());
    for k in 0..10 {
        let t = f64::from(k) * 0.1;
        refit.step(t, &[seen(10.0 + t, 0.0)], [0.0, 0.0]);
    }
    refit.step(2.0, &[seen(11.0, 0.5)], [0.0, 0.0]);
    let (a, b) = (&champion.tracks[0], &refit.tracks[0]);
    let fit = fitted_speed(&b.sightings, 0.15).expect("a fit").0;
    assert!(close(b.x[2], fit[0], 1e-12) && close(b.x[3], fit[1], 1e-12));
    assert!((a.x[2] - b.x[2]).abs() > 1e-6 || (a.x[3] - b.x[3]).abs() > 1e-6);
}

#[test]
fn a_fit_needs_three_places_at_different_times() {
    assert!(fitted_speed(&[[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]], 0.15).is_none());
    assert!(fitted_speed(&[[1.0, 0.0, 0.0]; 3], 0.15).is_none());
    let (v, _) =
        fitted_speed(&[[0.0, 0.0, 0.0], [1.0, 2.0, 1.0], [2.0, 4.0, 2.0]], 0.15).expect("a fit");
    assert!(close(v[0], 2.0, 1e-12) && close(v[1], 1.0, 1e-12));
}
