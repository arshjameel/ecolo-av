//! What rays can hit: triangles in a bounding volume tree, and a few movable objects.
//!
//! Gazebo's lidars render what you see (the visuals), so every shape a world draws becomes
//! triangles here: boxes, cylinders, planes and OBJ meshes. The world's own shapes go in
//! one tree, built once; objects spawned and removed while it runs (the obstacle, the
//! post) stay in a short list beside it, so a change never rebuilds the tree.

use crate::math::{Pose, Vec3};

/// One triangle, in the world frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Triangle {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
}

impl Triangle {
    #[must_use]
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Self {
        Self { a, b, c }
    }

    fn transformed(&self, pose: &Pose) -> Triangle {
        Triangle::new(pose.apply(self.a), pose.apply(self.b), pose.apply(self.c))
    }

    fn bounds(&self) -> Aabb {
        Aabb {
            min: self.a.min(self.b).min(self.c),
            max: self.a.max(self.b).max(self.c),
        }
    }

    fn centroid(&self) -> Vec3 {
        (self.a + self.b + self.c) * (1.0 / 3.0)
    }

    /// Möller-Trumbore: the distance along the ray to this triangle, either side.
    fn hit(&self, ray: &Ray) -> Option<f64> {
        const EPSILON: f64 = 1e-12;
        let e1 = self.b - self.a;
        let e2 = self.c - self.a;
        let p = ray.direction.cross(e2);
        let det = e1.dot(p);
        if det.abs() < EPSILON {
            return None;
        }
        let inv = 1.0 / det;
        let s = ray.origin - self.a;
        let u = s.dot(p) * inv;
        if !(0.0..=1.0).contains(&u) {
            return None;
        }
        let q = s.cross(e1);
        let v = ray.direction.dot(q) * inv;
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        let t = e2.dot(q) * inv;
        (t > 0.0).then_some(t)
    }
}

/// A shape made of triangles, in its own frame.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Mesh {
    pub triangles: Vec<Triangle>,
}

impl Mesh {
    /// A box centred on the origin, `size` across.
    #[must_use]
    pub fn cuboid(size: Vec3) -> Mesh {
        let (x, y, z) = (size.x / 2.0, size.y / 2.0, size.z / 2.0);
        let c = |i: usize| {
            Vec3::new(
                if i & 1 == 0 { -x } else { x },
                if i & 2 == 0 { -y } else { y },
                if i & 4 == 0 { -z } else { z },
            )
        };
        // Each face as two triangles: its four corners, by their index bits.
        const FACES: [[usize; 4]; 6] = [
            [0, 2, 6, 4],
            [1, 5, 7, 3],
            [0, 4, 5, 1],
            [2, 3, 7, 6],
            [0, 1, 3, 2],
            [4, 6, 7, 5],
        ];
        let triangles = FACES
            .iter()
            .flat_map(|f| {
                [
                    Triangle::new(c(f[0]), c(f[1]), c(f[2])),
                    Triangle::new(c(f[0]), c(f[2]), c(f[3])),
                ]
            })
            .collect();
        Mesh { triangles }
    }

    /// An upright cylinder centred on the origin, as `sides` flat faces and two caps.
    #[must_use]
    pub fn cylinder(radius: f64, length: f64, sides: usize) -> Mesh {
        let h = length / 2.0;
        let rim = |i: usize, z: f64| {
            let a = std::f64::consts::TAU * i as f64 / sides as f64;
            Vec3::new(radius * a.cos(), radius * a.sin(), z)
        };
        let (top, bottom) = (Vec3::new(0.0, 0.0, h), Vec3::new(0.0, 0.0, -h));
        let triangles = (0..sides)
            .flat_map(|i| {
                let j = (i + 1) % sides;
                [
                    Triangle::new(rim(i, -h), rim(j, -h), rim(j, h)),
                    Triangle::new(rim(i, -h), rim(j, h), rim(i, h)),
                    Triangle::new(top, rim(i, h), rim(j, h)),
                    Triangle::new(bottom, rim(j, -h), rim(i, -h)),
                ]
            })
            .collect();
        Mesh { triangles }
    }

    /// A flat rectangle on z = 0, `x` by `y`, facing up (SDF's `<plane>` with normal 0 0 1).
    #[must_use]
    pub fn plane(x: f64, y: f64) -> Mesh {
        let (hx, hy) = (x / 2.0, y / 2.0);
        let c = [
            Vec3::new(-hx, -hy, 0.0),
            Vec3::new(hx, -hy, 0.0),
            Vec3::new(hx, hy, 0.0),
            Vec3::new(-hx, hy, 0.0),
        ];
        Mesh {
            triangles: vec![
                Triangle::new(c[0], c[1], c[2]),
                Triangle::new(c[0], c[2], c[3]),
            ],
        }
    }

    /// A Wavefront OBJ file's faces: `v` and `f` lines; polygons as fans. Normals,
    /// textures and materials don't matter to a ray.
    pub fn from_obj(text: &str) -> Result<Mesh, String> {
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for (n, line) in text.lines().enumerate() {
            let mut words = line.split_whitespace();
            match words.next() {
                Some("v") => {
                    let xyz: Vec<f64> = words
                        .take(3)
                        .map(str::parse)
                        .collect::<Result<_, _>>()
                        .map_err(|e| format!("line {}: {e}", n + 1))?;
                    if xyz.len() != 3 {
                        return Err(format!("line {}: a vertex needs x y z", n + 1));
                    }
                    vertices.push(Vec3::new(xyz[0], xyz[1], xyz[2]));
                }
                Some("f") => {
                    let corners = words
                        .map(|w| obj_index(w, vertices.len()))
                        .collect::<Option<Vec<_>>>()
                        .ok_or_else(|| format!("line {}: a face's vertex is missing", n + 1))?;
                    triangles.extend((1..corners.len().saturating_sub(1)).map(|i| {
                        Triangle::new(
                            vertices[corners[0]],
                            vertices[corners[i]],
                            vertices[corners[i + 1]],
                        )
                    }));
                }
                _ => {}
            }
        }
        Ok(Mesh { triangles })
    }

    /// The mesh scaled along each axis (SDF's `<scale>`).
    #[must_use]
    pub fn scaled(mut self, s: Vec3) -> Mesh {
        let k = |v: Vec3| Vec3::new(v.x * s.x, v.y * s.y, v.z * s.z);
        for t in &mut self.triangles {
            *t = Triangle::new(k(t.a), k(t.b), k(t.c));
        }
        self
    }

    /// The mesh moved into its parent's frame.
    #[must_use]
    pub fn transformed(&self, pose: &Pose) -> Mesh {
        Mesh {
            triangles: self.triangles.iter().map(|t| t.transformed(pose)).collect(),
        }
    }
}

/// An OBJ vertex reference ("7", "7/2", "7/2/3", or negative: from the end) as an index.
fn obj_index(word: &str, count: usize) -> Option<usize> {
    let i: i64 = word.split('/').next()?.parse().ok()?;
    let index = if i < 0 { count as i64 + i } else { i - 1 };
    (0..count as i64).contains(&index).then_some(index as usize)
}

/// A ray: where it starts, and its direction (a unit vector).
#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}

#[derive(Debug, Clone, Copy)]
struct Aabb {
    min: Vec3,
    max: Vec3,
}

impl Aabb {
    const EMPTY: Aabb = Aabb {
        min: Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        max: Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
    };

    fn grow(&self, o: &Aabb) -> Aabb {
        Aabb {
            min: self.min.min(o.min),
            max: self.max.max(o.max),
        }
    }

    /// Where the ray enters the box, if it does before `limit` (slab test).
    fn entry(&self, origin: Vec3, inverse: Vec3, limit: f64) -> Option<f64> {
        let (mut near, mut far) = (0.0f64, limit);
        for axis in 0..3 {
            let o = origin.axis(axis);
            let k = inverse.axis(axis);
            let t0 = (self.min.axis(axis) - o) * k;
            let t1 = (self.max.axis(axis) - o) * k;
            // NaN (0 x inf: a ray in the slab's plane) leaves the bounds as they were.
            let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
            near = if lo > near { lo } else { near };
            far = if hi < far { hi } else { far };
            if near > far {
                return None;
            }
        }
        Some(near)
    }
}

/// A node of the tree: a box, and either two children or a run of triangles.
#[derive(Debug, Clone, Copy)]
struct Node {
    bounds: Aabb,
    /// Leaf: the first triangle. Inner: the second child (the first is the next node).
    start: u32,
    /// Leaf: how many triangles; 0 for an inner node.
    count: u32,
}

const LEAF_SIZE: usize = 4;

/// The world's fixed shapes, in a bounding volume tree for fast rays.
#[derive(Debug, Clone, Default)]
pub struct Bvh {
    triangles: Vec<Triangle>,
    nodes: Vec<Node>,
}

impl Bvh {
    pub fn new(triangles: Vec<Triangle>) -> Self {
        let mut bvh = Bvh {
            triangles,
            nodes: Vec::new(),
        };
        if !bvh.triangles.is_empty() {
            let mut centroids: Vec<Vec3> = bvh.triangles.iter().map(Triangle::centroid).collect();
            let n = bvh.triangles.len();
            bvh.build(&mut centroids, 0, n);
        }
        bvh
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.triangles.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    /// Build the subtree over triangles[start..end]; return its node's index.
    fn build(&mut self, centroids: &mut [Vec3], start: usize, end: usize) -> usize {
        let bounds = self.triangles[start..end]
            .iter()
            .fold(Aabb::EMPTY, |b, t| b.grow(&t.bounds()));
        let index = self.nodes.len();
        self.nodes.push(Node {
            bounds,
            start: start as u32,
            count: (end - start) as u32,
        });
        if end - start <= LEAF_SIZE {
            return index;
        }
        // Split at the median along the centroids' longest side.
        let spread = centroids[start..end]
            .iter()
            .fold(Aabb::EMPTY, |b, c| b.grow(&Aabb { min: *c, max: *c }));
        let size = spread.max - spread.min;
        let axis = if size.x >= size.y && size.x >= size.z {
            0
        } else if size.y >= size.z {
            1
        } else {
            2
        };
        let mid = (start + end) / 2;
        let mut order: Vec<usize> = (start..end).collect();
        order.select_nth_unstable_by(mid - start, |&a, &b| {
            centroids[a].axis(axis).total_cmp(&centroids[b].axis(axis))
        });
        let tris: Vec<Triangle> = order.iter().map(|&i| self.triangles[i]).collect();
        let cents: Vec<Vec3> = order.iter().map(|&i| centroids[i]).collect();
        self.triangles[start..end].copy_from_slice(&tris);
        centroids[start..end].copy_from_slice(&cents);

        self.build(centroids, start, mid);
        let second = self.build(centroids, mid, end);
        self.nodes[index].start = second as u32;
        self.nodes[index].count = 0;
        index
    }

    /// The nearest hit closer than `limit`, along the ray.
    #[must_use]
    pub fn cast(&self, ray: &Ray, limit: f64) -> Option<f64> {
        if self.nodes.is_empty() {
            return None;
        }
        let inverse = Vec3::new(
            1.0 / ray.direction.x,
            1.0 / ray.direction.y,
            1.0 / ray.direction.z,
        );
        let mut best = limit;
        let mut found = false;
        let mut stack = [0usize; 64];
        let mut top = 1;
        while top > 0 {
            top -= 1;
            let index = stack[top];
            let node = self.nodes[index];
            if node.bounds.entry(ray.origin, inverse, best).is_none() {
                continue;
            }
            if node.count > 0 {
                let first = node.start as usize;
                for t in &self.triangles[first..first + node.count as usize] {
                    if let Some(d) = t.hit(ray) {
                        if d < best {
                            best = d;
                            found = true;
                        }
                    }
                }
            } else if top + 2 <= stack.len() {
                // The first child is built right after its parent; `start` is the second.
                stack[top] = node.start as usize;
                stack[top + 1] = index + 1;
                top += 2;
            }
        }
        found.then_some(best)
    }
}

/// An object placed while the world runs, by name.
#[derive(Debug, Clone)]
struct Placed {
    name: String,
    triangles: Vec<Triangle>,
    bounds: Aabb,
}

/// Everything a ray can hit: the world's fixed shapes and the objects placed since.
#[derive(Debug, Clone, Default)]
pub struct Scene {
    fixed: Bvh,
    placed: Vec<Placed>,
}

impl Scene {
    #[must_use]
    pub fn new(triangles: Vec<Triangle>) -> Self {
        Self {
            fixed: Bvh::new(triangles),
            placed: Vec::new(),
        }
    }

    /// How many triangles the fixed world has.
    #[must_use]
    pub fn triangle_count(&self) -> usize {
        self.fixed.len()
    }

    /// Put a shape in the world at this pose, replacing any of the same name.
    pub fn place(&mut self, name: &str, mesh: &Mesh, pose: &Pose) {
        self.remove(name);
        let triangles = mesh.transformed(pose).triangles;
        let bounds = triangles
            .iter()
            .fold(Aabb::EMPTY, |b, t| b.grow(&t.bounds()));
        self.placed.push(Placed {
            name: name.to_string(),
            triangles,
            bounds,
        });
    }

    /// Take a placed object away; false if there was none of that name.
    pub fn remove(&mut self, name: &str) -> bool {
        let before = self.placed.len();
        self.placed.retain(|p| p.name != name);
        self.placed.len() != before
    }

    /// The names of the objects placed while running.
    pub fn placed(&self) -> impl Iterator<Item = &str> {
        self.placed.iter().map(|p| p.name.as_str())
    }

    /// The distance to the nearest surface along the ray, if one is closer than `limit`.
    #[must_use]
    pub fn cast(&self, ray: &Ray, limit: f64) -> Option<f64> {
        self.cast_named(ray, limit).map(|(d, _)| d)
    }

    /// As [`Scene::cast`], with the placed object's name if the ray hit one (`None`: the
    /// fixed world).
    #[must_use]
    pub fn cast_named(&self, ray: &Ray, limit: f64) -> Option<(f64, Option<&str>)> {
        let mut best = self.fixed.cast(ray, limit).map(|d| (d, None));
        let inverse = Vec3::new(
            1.0 / ray.direction.x,
            1.0 / ray.direction.y,
            1.0 / ray.direction.z,
        );
        for p in &self.placed {
            let reach = best.map_or(limit, |(d, _)| d);
            if p.bounds.entry(ray.origin, inverse, reach).is_none() {
                continue;
            }
            for t in &p.triangles {
                if let Some(d) = t.hit(ray) {
                    if d < best.map_or(limit, |(b, _)| b) {
                        best = Some((d, Some(p.name.as_str())));
                    }
                }
            }
        }
        best
    }
}
