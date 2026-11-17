//! The pictures themselves, drawn by Bevy on this computer's GPU: each shape of the test
//! world is where the lens says, in its colour. Skips, saying so, on a computer with no GPU.
//!
//! One test, so Bevy starts once (a few seconds).

use std::time::{Duration, Instant};

use camera_core::{world::looks, CameraRenderer, Frame, Pose, Rgb, Rig, SimWorld, Vec3};
use sim_cameras::{Adapter, BevyRenderer, Graphics, RenderError};

const CAMERAS: &str = include_str!("../config/cameras.yaml");
const VEHICLE: &str = include_str!("../../../bringup/aav_description/config/vehicle.yaml");

fn srgb(c: Rgb) -> [f32; 3] {
    c.to_srgb8().map(f32::from)
}

/// The pixel where `p` (map) lands in camera `cam`'s picture.
fn at(rig: &Rig, frame: &Frame, car: &Pose, p: Vec3) -> [u8; 3] {
    let spec = &rig.cameras[frame.camera];
    let px = spec
        .project(car, p)
        .and_then(|px| px.index(frame.width, frame.height))
        .unwrap_or_else(|| panic!("{p:?} isn't in {}'s picture", spec.name));
    frame.pixel(px.0, px.1).unwrap()
}

/// The plate faces away from the sun, as in Gazebo's road world: a dark red.
fn is_red(c: [u8; 3]) -> bool {
    c[0] >= 50 && u16::from(c[0]) > 3 * u16::from(c[1]) && u16::from(c[0]) > 3 * u16::from(c[2])
}

fn is_grey(c: [u8; 3]) -> bool {
    let (lo, hi) = (c.iter().min().unwrap(), c.iter().max().unwrap());
    hi - lo <= 12
}

#[test]
fn the_test_world_as_the_lens_says() {
    let rig = Rig::from_yaml(CAMERAS, VEHICLE).unwrap();
    let world = SimWorld::test_world().with_car(VEHICLE).unwrap();
    let start = Instant::now();
    let mut renderer = match BevyRenderer::new(&rig, &world) {
        Ok(r) => r,
        Err(RenderError::NoGpu(why)) => {
            eprintln!("SKIPPED: no GPU on this computer ({why})");
            return;
        }
        Err(e) => panic!("{e}"),
    };
    eprintln!(
        "Bevy started and warmed up in {:.1} s",
        start.elapsed().as_secs_f64()
    );

    let car = Pose::IDENTITY;
    let all: Vec<usize> = (0..rig.cameras.len()).collect();
    let frames = renderer
        .render(&car, Duration::from_millis(1234), &all)
        .unwrap();
    assert_eq!(frames.len(), 4);
    let out = std::env::temp_dir();
    for f in &frames {
        let name = &rig.cameras[f.camera].name;
        assert_eq!(
            (f.width, f.height, f.rgb.len()),
            (640, 360, 640 * 360 * 3),
            "{name}"
        );
        assert_eq!(f.stamp, Duration::from_millis(1234));
        let path = out.join(format!("sim_cameras_{name}.png"));
        image::save_buffer(
            &path,
            &f.rgb,
            f.width,
            f.height,
            image::ExtendedColorType::Rgb8,
        )
        .unwrap();
        eprintln!("saved {}", path.display());
    }
    let front = &frames[0];

    // The sky, at the top, is exactly the world's background colour.
    let sky = front.pixel(320, 5).unwrap();
    let want = srgb(looks::SKY);
    for k in 0..3 {
        assert!(
            (f32::from(sky[k]) - want[k]).abs() <= 2.0,
            "sky {sky:?}, want {want:?}"
        );
    }

    // The stop sign's plate, 25 m on: red.
    let plate = at(&rig, front, &car, Vec3::new(24.98, -3.0, 2.0));
    assert!(is_red(plate), "plate {plate:?}");
    // Its post, below the plate: grey.
    let post = at(&rig, front, &car, Vec3::new(24.95, -3.0, 0.8));
    assert!(is_grey(post), "post {post:?}");

    // The road ahead: grey tarmac; the grass beside it: green.
    let road = at(&rig, front, &car, Vec3::new(8.0, 0.5, 0.004));
    assert!(is_grey(road), "road {road:?}");
    let grass = at(&rig, front, &car, Vec3::new(10.0, 5.0, 0.0));
    assert!(
        grass[1] > grass[0] + 20 && grass[1] > grass[2] + 20,
        "grass {grass:?}"
    );
    // The kerb's top, at the right: lighter grey than the road.
    let kerb = at(&rig, front, &car, Vec3::new(8.0, -2.325, 0.30));
    assert!(
        is_grey(kerb) && kerb[0] > road[0] + 30,
        "kerb {kerb:?}, road {road:?}"
    );
    // The box's side facing the car: brick red-brown.
    let brick = at(&rig, front, &car, Vec3::new(11.49, 3.5, 0.5));
    assert!(brick[0] > brick[1] + 20, "box {brick:?}");

    // The rear camera looks back down the road (it starts 20 m behind), grass beside it;
    // the chase camera sees the car's body.
    let rear = &frames[2];
    let behind = at(&rig, rear, &car, Vec3::new(-6.0, 0.0, 0.004));
    assert!(is_grey(behind), "behind {behind:?}");
    let beside = at(&rig, rear, &car, Vec3::new(-8.0, 5.0, 0.0));
    assert!(beside[1] > beside[0] + 20, "beside {beside:?}");
    let chase = &frames[3];
    let body = at(&rig, chase, &car, Vec3::new(0.8, 0.0, 1.5));
    let green = srgb(looks::CAR_BODY);
    assert!(
        f32::from(body[1]) > f32::from(body[0]) && green[1] > green[0],
        "body {body:?}"
    );
    // The front cameras don't see the car's own body: the bottom of the picture is road.
    let below = front.pixel(320, 359).unwrap();
    assert!(is_grey(below), "below {below:?}");

    // A fresh picture each time: 10 m on, the plate is bigger and still where the lens says.
    let nearer = Pose::planar(10.0, 0.0, 0.0);
    let f = renderer
        .render(&nearer, Duration::from_secs(2), &[0])
        .unwrap();
    let plate = at(&rig, &f[0], &nearer, Vec3::new(24.98, -3.0, 2.0));
    assert!(is_red(plate), "plate from 15 m {plate:?}");
    // The projection test on the picture: the plate's red runs, along the row through its
    // centre, between where the lens puts its two side flats (y = -3 +- 0.375 m), to a pixel.
    let spec = &rig.cameras[0];
    let centre = spec.project(&nearer, Vec3::new(24.99, -3.0, 2.0)).unwrap();
    let left = spec
        .project(&nearer, Vec3::new(24.99, -2.625, 2.0))
        .unwrap();
    let right = spec
        .project(&nearer, Vec3::new(24.99, -3.375, 2.0))
        .unwrap();
    let row = centre.v as u32;
    let red: Vec<u32> = (0..640)
        .filter(|&u| is_red(f[0].pixel(u, row).unwrap()))
        .collect();
    let (first, last) = (red[0], red[red.len() - 1]);
    eprintln!(
        "plate: lens {:.1}..{:.1} px, picture {first}..={last}",
        left.u, right.u
    );
    assert!(
        (f64::from(first) - left.u).abs() <= 1.0,
        "left edge {first} vs {:.2}",
        left.u
    );
    assert!(
        (f64::from(last + 1) - right.u).abs() <= 1.0,
        "right edge {last} vs {:.2}",
        right.u
    );
    let was_at = rig.cameras[0]
        .project(&car, Vec3::new(24.98, -3.0, 2.0))
        .unwrap();
    assert!(
        rig.cameras[0]
            .project(&nearer, Vec3::new(24.98, -3.0, 2.0))
            .unwrap()
            .u
            > was_at.u + 10.0
    );

    // Turned left 90 degrees, the front camera no longer sees the plate where it was.
    let turned = Pose::planar(10.0, 0.0, std::f64::consts::FRAC_PI_2);
    let f = renderer
        .render(&turned, Duration::from_secs(3), &[0])
        .unwrap();
    let old = f[0].pixel(was_at.u as u32, was_at.v as u32).unwrap();
    assert!(!is_red(old), "still red after turning: {old:?}");

    // Pretty (TODO 244): grain, a sky, haze far off, but the same things in the same
    // colours where the checks look: the plate red, the road grey, the grass green.
    drop(renderer);
    let mut renderer =
        BevyRenderer::with_graphics(&rig, &world, Adapter::Any, Some(Graphics::Pretty)).unwrap();
    assert_eq!(renderer.graphics(), Graphics::Pretty);
    let f = renderer.render(&car, Duration::ZERO, &[0]).unwrap();
    let front = &f[0];
    let plate = at(&rig, front, &car, Vec3::new(24.98, -3.0, 2.0));
    assert!(is_red(plate), "pretty plate {plate:?}");
    let road = at(&rig, front, &car, Vec3::new(8.0, 0.5, 0.004));
    let (lo, hi) = (road.iter().min().unwrap(), road.iter().max().unwrap());
    assert!(hi - lo <= 20, "pretty road {road:?}");
    let grass = at(&rig, front, &car, Vec3::new(10.0, 5.0, 0.0));
    assert!(
        grass[1] > grass[0] + 20 && grass[1] > grass[2] + 20,
        "pretty grass {grass:?}"
    );
    // The sky: bluer overhead than at the horizon.
    let (top, horizon) = (front.pixel(320, 5).unwrap(), front.pixel(320, 170).unwrap());
    assert!(
        top[2] > top[0] + 40 && horizon[2] - horizon[0] < top[2] - top[0],
        "pretty sky {top:?} over {horizon:?}"
    );
}
