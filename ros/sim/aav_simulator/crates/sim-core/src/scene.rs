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
    /// The first corner.
    pub a: Vec3,
    /// The second corner.
    pub b: Vec3,
    /// The third corner.
    pub c: Vec3,
}

impl Triangle {
    /// A triangle from its three corners.
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
    /// Its triangles, in its own frame.
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
    ///
    /// # Errors
    ///
    /// A message with the line number if a `v` or `f` line can't be read.
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
    /// Where the ray starts.
    pub origin: Vec3,
    /// Which way it goes: a unit vector.
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
    /// The tree over these triangles.
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

    /// How many triangles it holds.
    #[must_use]
    pub fn len(&self) -> usize {
        self.triangles.len()
    }

    /// True if it holds no triangles.
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
    /// A scene of these fixed triangles, nothing placed yet.
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Rot3;

    fn ray(origin: [f64; 3], direction: [f64; 3]) -> Ray {
        let d = Vec3::new(direction[0], direction[1], direction[2]);
        Ray {
            origin: Vec3::new(origin[0], origin[1], origin[2]),
            direction: d * (1.0 / d.length()),
        }
    }

    fn close(a: Option<f64>, b: f64) -> bool {
        a.is_some_and(|a| (a - b).abs() < 1e-9)
    }

    #[test]
    fn a_triangle_is_hit_from_either_side_and_only_ahead() {
        let t = Triangle::new(
            Vec3::new(2.0, -1.0, -1.0),
            Vec3::new(2.0, 1.0, -1.0),
            Vec3::new(2.0, 0.0, 1.0),
        );
        assert!(close(t.hit(&ray([0.0; 3], [1.0, 0.0, 0.0])), 2.0));
        assert!(close(t.hit(&ray([4.0, 0.0, 0.0], [-1.0, 0.0, 0.0])), 2.0));
        assert_eq!(
            t.hit(&ray([4.0, 0.0, 0.0], [1.0, 0.0, 0.0])),
            None,
            "behind"
        );
        assert_eq!(
            t.hit(&ray([0.0, 5.0, 0.0], [1.0, 0.0, 0.0])),
            None,
            "beside"
        );
        assert_eq!(
            t.hit(&ray([0.0; 3], [0.0, 1.0, 0.0])),
            None,
            "along its plane"
        );
    }

    #[test]
    fn a_box_is_twelve_triangles_hit_on_each_face() {
        let cube = Mesh::cuboid(Vec3::new(2.0, 4.0, 6.0));
        assert_eq!(cube.triangles.len(), 12);
        let scene = Scene::new(cube.triangles);
        for (from, dir, d) in [
            ([5.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 4.0),
            ([-5.0, 0.0, 0.0], [1.0, 0.0, 0.0], 4.0),
            ([0.0, 5.0, 0.0], [0.0, -1.0, 0.0], 3.0),
            ([0.0, -5.0, 0.0], [0.0, 1.0, 0.0], 3.0),
            ([0.0, 0.0, 5.0], [0.0, 0.0, -1.0], 2.0),
            ([0.0, 0.0, -5.0], [0.0, 0.0, 1.0], 2.0),
        ] {
            assert!(close(scene.cast(&ray(from, dir), 100.0), d), "{from:?}");
        }
        // From inside, a ray meets the wall it faces.
        assert!(close(
            scene.cast(&ray([0.0; 3], [1.0, 0.0, 0.0]), 100.0),
            1.0
        ));
    }

    #[test]
    fn a_cylinder_and_a_plane_are_where_they_should_be() {
        let post = Scene::new(Mesh::cylinder(0.5, 2.0, 32).triangles);
        // Off the seam at y = 0: a ray exactly through an edge two faces share can slip
        // between them by rounding (Möller-Trumbore's edges; 1 in billions of real rays).
        let d = post.cast(&ray([3.0, 0.05, 0.0], [-1.0, 0.0, 0.0]), 10.0);
        // A flat face of 32 sides sits a little inside the circle.
        assert!(d.is_some_and(|d| (2.5..2.51).contains(&d)), "{d:?}");
        assert!(close(
            // Off the cap's centre, the corner its triangles share.
            post.cast(&ray([0.1, 0.05, 5.0], [0.0, 0.0, -1.0]), 10.0),
            4.0
        ));
        assert_eq!(
            post.cast(&ray([3.0, 0.0, 1.5], [-1.0, 0.0, 0.0]), 10.0),
            None
        );
        let ground = Scene::new(Mesh::plane(10.0, 10.0).triangles);
        assert!(close(
            ground.cast(&ray([1.0, 2.0, 3.0], [0.0, 0.0, -1.0]), 10.0),
            3.0
        ));
        assert_eq!(
            ground.cast(&ray([6.0, 0.0, 3.0], [0.0, 0.0, -1.0]), 10.0),
            None
        );
    }

    #[test]
    fn nothing_beyond_the_limit_is_hit() {
        let scene = Scene::new(Mesh::cuboid(Vec3::new(1.0, 1.0, 1.0)).triangles);
        let r = ray([10.0, 0.0, 0.0], [-1.0, 0.0, 0.0]);
        assert_eq!(scene.cast(&r, 9.0), None);
        assert!(close(scene.cast(&r, 9.6), 9.5));
        assert_eq!(Scene::default().cast(&r, 100.0), None, "an empty world");
    }

    #[test]
    fn an_obj_file_reads_its_faces_as_fans() {
        let text = "# a square and a triangle\n\
                    v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n\
                    vn 0 0 1\n\
                    f 1/1/1 2/2/1 3/3/1 4/4/1\n\
                    f -1 -2 -3\n";
        let mesh = Mesh::from_obj(text).expect("a good file");
        assert_eq!(mesh.triangles.len(), 3);
        assert_eq!(
            mesh.triangles[0],
            Triangle::new(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0)
            )
        );
        assert_eq!(
            mesh.triangles[2].a,
            Vec3::new(0.0, 1.0, 0.0),
            "-1: the last"
        );
    }

    #[test]
    fn a_broken_obj_file_says_which_line() {
        let bad_vertex = Mesh::from_obj("v 0 0 0\nv 1 x 0\n").expect_err("x");
        assert!(bad_vertex.starts_with("line 2"), "{bad_vertex}");
        let short = Mesh::from_obj("v 0 0\n").expect_err("two numbers");
        assert!(short.contains("x y z"), "{short}");
        let missing = Mesh::from_obj("v 0 0 0\nf 1 2 3\n").expect_err("no vertex 2");
        assert!(missing.starts_with("line 2"), "{missing}");
    }

    #[test]
    fn scaled_and_moved_meshes_land_where_sdf_puts_them() {
        let mesh = Mesh::cuboid(Vec3::new(1.0, 1.0, 1.0)).scaled(Vec3::new(2.0, 1.0, 1.0));
        let pose = Pose::new(Vec3::new(10.0, 0.0, 0.0), Rot3::from_yaw(PI_2));
        let scene = Scene::new(mesh.transformed(&pose).triangles);
        // Turned a quarter: its 2 m side lies along y now.
        assert!(close(
            scene.cast(&ray([10.0, 5.0, 0.0], [0.0, -1.0, 0.0]), 10.0),
            4.0
        ));
        assert!(close(
            scene.cast(&ray([5.0, 0.0, 0.0], [1.0, 0.0, 0.0]), 10.0),
            4.5
        ));
    }

    const PI_2: f64 = std::f64::consts::FRAC_PI_2;

    /// A repeatable spread of numbers in [0, 1) (a linear congruential generator).
    fn numbers(seed: u64) -> impl Iterator<Item = f64> {
        let mut s = seed;
        std::iter::repeat_with(move || {
            s = s
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (s >> 11) as f64 / (1u64 << 53) as f64
        })
    }

    #[test]
    fn the_tree_finds_what_trying_every_triangle_finds() {
        let mut n = numbers(7);
        let mut next = move || n.next().unwrap_or(0.5);
        let triangles: Vec<Triangle> = (0..500)
            .map(|_| {
                let c = Vec3::new(next() * 40.0 - 20.0, next() * 40.0 - 20.0, next() * 4.0);
                let mut corner = || c + Vec3::new(next() - 0.5, next() - 0.5, next() - 0.5);
                Triangle::new(corner(), corner(), corner())
            })
            .collect();
        let tree = Bvh::new(triangles.clone());
        assert_eq!(tree.len(), 500);
        let mut hits = 0;
        for _ in 0..2000 {
            let a = next() * std::f64::consts::TAU;
            let r = ray([0.0, 0.0, 2.0], [a.cos(), a.sin(), next() * 0.2 - 0.1]);
            let brute = triangles
                .iter()
                .filter_map(|t| t.hit(&r))
                .filter(|d| *d < 30.0)
                .fold(None, |best: Option<f64>, d| {
                    Some(best.map_or(d, |b| b.min(d)))
                });
            assert_eq!(tree.cast(&r, 30.0), brute);
            hits += usize::from(brute.is_some());
        }
        assert!(hits > 100, "the rays hit something: {hits}");
    }

    #[test]
    fn placed_objects_come_and_go_by_name() {
        let wall = Mesh::cuboid(Vec3::new(1.0, 10.0, 2.0));
        let mut scene = Scene::new(
            wall.transformed(&Pose::new(Vec3::new(20.0, 0.0, 0.0), Rot3::IDENTITY))
                .triangles,
        );
        let r = ray([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
        assert_eq!(scene.cast_named(&r, 50.0), Some((19.5, None)));
        let post = Mesh::cuboid(Vec3::new(1.0, 1.0, 2.0));
        scene.place(
            "post",
            &post,
            &Pose::new(Vec3::new(5.0, 0.0, 0.0), Rot3::IDENTITY),
        );
        assert_eq!(scene.cast_named(&r, 50.0), Some((4.5, Some("post"))));
        // The same name again: moved, not doubled.
        scene.place(
            "post",
            &post,
            &Pose::new(Vec3::new(8.0, 0.0, 0.0), Rot3::IDENTITY),
        );
        assert_eq!(scene.placed().collect::<Vec<_>>(), ["post"]);
        assert_eq!(scene.cast_named(&r, 50.0), Some((7.5, Some("post"))));
        // Behind the wall, a placed thing is hidden by it.
        scene.place(
            "far",
            &post,
            &Pose::new(Vec3::new(30.0, 0.0, 0.0), Rot3::IDENTITY),
        );
        assert!(scene.remove("post"));
        assert!(!scene.remove("post"), "already gone");
        assert_eq!(scene.cast_named(&r, 50.0), Some((19.5, None)));
        assert_eq!(scene.triangle_count(), 12, "the fixed world never changes");
    }
}
