//! Reads `python_vectors.txt` (and `--bench` files): the clouds `test/capture_python_vectors.py`
//! played through `aav_voxels`' `cubes.py`, and its answers. Shared by the parity test and
//! the timing example. A test helper: a bad file is a bug, so it panics.

#![allow(dead_code)] // the timing example reads the clouds, not the answers

use voxels::{Body, CubeShape, Mount, Point};

/// One recorded cloud and what the Python made of it.
pub struct Recorded {
    pub name: String,
    pub mount: Mount,
    /// The points in the lidar's frame, float32 as a lidar sends them.
    pub lidar: Vec<Point>,
    pub fitted: bool,
    pub candidates: usize,
    /// The road's plane: a, b, c.
    pub plane: [f64; 3],
    pub counted: usize,
    /// The obstacle cubes' points, as their numbers in `lidar`.
    pub obstacles: Vec<usize>,
    /// The same, cut on a flat road (z = 0).
    pub flat_obstacles: Vec<usize>,
    /// The ground squares' centres, x and y.
    pub ground: Vec<[f64; 2]>,
}

struct Lines<'a>(std::iter::Peekable<Box<dyn Iterator<Item = &'a str> + 'a>>);

impl<'a> Lines<'a> {
    fn next(&mut self) -> &'a str {
        self.0.next().expect("the file ends too soon")
    }

    /// The words after `label` on the next line.
    fn field(&mut self, label: &str) -> Vec<&'a str> {
        let line = self.next();
        line.strip_prefix(label)
            .and_then(|rest| rest.strip_prefix(' '))
            .unwrap_or_else(|| panic!("expected {label}, got {line}"))
            .split(' ')
            .collect()
    }

    fn count(&mut self, label: &str) -> usize {
        self.field(label)[0].parse().expect("a count")
    }

    fn numbers(&mut self) -> Vec<f64> {
        numbers(&self.next().split(' ').collect::<Vec<_>>())
    }
}

fn numbers(words: &[&str]) -> Vec<f64> {
    words
        .iter()
        .map(|w| w.parse().unwrap_or_else(|_| panic!("not a number: {w}")))
        .collect()
}

fn three(v: &[f64]) -> [f64; 3] {
    [v[0], v[1], v[2]]
}

/// The shape the clouds were cut with (the default if the file doesn't say), and the clouds.
pub fn read(text: &str) -> (CubeShape, Vec<Recorded>) {
    let all: Box<dyn Iterator<Item = &str>> =
        Box::new(text.lines().filter(|l| !l.starts_with('#')));
    let mut lines = Lines(all.peekable());
    let mut shape = CubeShape::default();
    if lines.0.peek().is_some_and(|l| l.starts_with("shape ")) {
        let v = numbers(&lines.field("shape"));
        shape = CubeShape {
            size: v[0],
            min_height: v[1],
            max_height: v[2],
            range_max: v[3],
            body: Body {
                back: v[4],
                front: v[5],
                half_width: v[6],
            },
            ..shape
        };
    }
    let mut clouds = Vec::new();
    while lines.0.peek().is_some() {
        let name = lines.field("cloud").join(" ");
        let mount = numbers(&lines.field("mount"));
        let n = lines.count("points");
        let lidar = (0..n)
            .map(|_| {
                let bits: Vec<f64> = lines
                    .next()
                    .split(' ')
                    .map(|h| {
                        let raw = u32::from_str_radix(h, 16).expect("8 hex digits");
                        f64::from(f32::from_bits(raw))
                    })
                    .collect();
                Point::new(bits[0], bits[1], bits[2])
            })
            .collect();
        let road = lines.field("road");
        let (fitted, candidates) = (road[0] == "1", road[1].parse().expect("a count"));
        let plane = three(&numbers(&road[2..]));
        let counted = lines.count("counted");
        let mut indices = |label: &str| -> Vec<usize> {
            let n = lines.count(label);
            (0..n)
                .map(|_| lines.next().parse().expect("a point's number"))
                .collect()
        };
        let obstacles = indices("obstacles");
        let flat_obstacles = indices("flat_obstacles");
        let n = lines.count("ground");
        let ground = (0..n)
            .map(|_| {
                let v = lines.numbers();
                [v[0], v[1]]
            })
            .collect();
        clouds.push(Recorded {
            name,
            mount: Mount::new(three(&mount[..3]), three(&mount[3..])),
            lidar,
            fitted,
            candidates,
            plane,
            counted,
            obstacles,
            flat_obstacles,
            ground,
        });
    }
    (shape, clouds)
}
