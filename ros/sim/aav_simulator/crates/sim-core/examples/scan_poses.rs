//! The roof lidar's scan at each pose in a file, with no ROS: Ravensim's own rays on a world
//! file, for offline tools (`tools/label_drive.py --sim-drive`: a drive made of poses).
//!
//! ```sh
//! cargo run --release --example scan_poses -- WORLD.sdf POSES.csv OUT.bin \
//!     [--models DIR:DIR] [--mount x,y,z,roll,pitch,yaw] [--seed N]
//! ```
//!
//! POSES.csv: one `x,y,yaw` a line (m, m, rad, the world frame: base_link on the ground),
//! `#` starts a note: the same file `sim_cameras --poses` reads. `--mount`: the lidar in
//! base_link (vehicle.yaml's `lidar_link`; that is the default).
//!
//! OUT.bin, little-endian, per pose: a u32 count n, then n points of four f32: x, y, z (m,
//! the lidar's frame) and the ring. As the simulator's `/sensors/lidar/points` would carry.

use std::io::Write as _;
use std::path::{Path, PathBuf};

use sim_core::lidar::{LidarSpec, Scan};
use sim_core::math::{Pose, Rot3, Vec3};
use sim_core::noise::Noise;
use sim_core::world::{Surfaces, World};

/// vehicle.yaml's lidar_link.
const MOUNT: [f64; 6] = [0.49, 0.0, 1.52, 0.0, 0.0, 0.0];

fn numbers(text: &str) -> Result<Vec<f64>, String> {
    text.split(',')
        .map(|v| v.trim().parse::<f64>().map_err(|e| format!("{v:?}: {e}")))
        .collect()
}

fn poses(text: &str) -> Result<Vec<[f64; 3]>, String> {
    text.lines()
        .map(|l| l.split('#').next().unwrap_or("").trim())
        .filter(|l| !l.is_empty())
        .map(|l| match numbers(l)?.as_slice() {
            &[x, y, yaw] => Ok([x, y, yaw]),
            other => Err(format!("{l:?}: {} numbers, not x,y,yaw", other.len())),
        })
        .collect()
}

fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let (Some(world), Some(pose_file), Some(out)) = (args.next(), args.next(), args.next()) else {
        return Err(
            "usage: scan_poses WORLD.sdf POSES.csv OUT.bin [--models DIR:DIR] \
                    [--mount x,y,z,r,p,y] [--seed N]"
                .into(),
        );
    };
    let (mut models, mut mount, mut seed) = (Vec::<PathBuf>::new(), MOUNT.to_vec(), 7u64);
    while let Some(flag) = args.next() {
        let value = args.next().ok_or(format!("{flag} needs a value"))?;
        match flag.as_str() {
            "--models" => models = std::env::split_paths(&value).collect(),
            "--mount" => mount = numbers(&value)?,
            "--seed" => seed = value.parse().map_err(|e| format!("--seed: {e}"))?,
            other => return Err(format!("unknown option {other}")),
        }
    }
    let &[x, y, z, roll, pitch, yaw] = mount.as_slice() else {
        return Err("--mount takes six numbers".into());
    };
    let mount = Pose::from_xyz_rpy([x, y, z], [roll, pitch, yaw]);
    let world = World::load(Path::new(&world), &models, Surfaces::Visual)
        .map_err(|e| format!("{world}: {e}"))?;
    let text = std::fs::read_to_string(&pose_file).map_err(|e| format!("{pose_file}: {e}"))?;
    let poses = poses(&text)?;
    let mut noise = Noise::new(seed);
    let mut bytes = Vec::new();
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get().min(8));
    let start = std::time::Instant::now();
    for &[x, y, yaw] in &poses {
        let car = Pose::new(Vec3::new(x, y, 0.0), Rot3::from_yaw(yaw));
        let scan = Scan::cast(
            &LidarSpec::ROOF,
            &world.scene,
            &car.then(&mount),
            &mut noise,
            threads,
        );
        let hits: Vec<(Vec3, u16)> = scan.hits().collect();
        bytes.extend((hits.len() as u32).to_le_bytes());
        for (p, ring) in hits {
            for v in [p.x as f32, p.y as f32, p.z as f32, f32::from(ring)] {
                bytes.extend(v.to_le_bytes());
            }
        }
    }
    std::fs::File::create(&out)
        .and_then(|mut f| f.write_all(&bytes))
        .map_err(|e| format!("{out}: {e}"))?;
    eprintln!(
        "scan_poses: {} poses in {:.1} s ({} triangles)",
        poses.len(),
        start.elapsed().as_secs_f64(),
        world.scene.triangle_count()
    );
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("scan_poses: {e}");
        std::process::exit(1);
    }
}
