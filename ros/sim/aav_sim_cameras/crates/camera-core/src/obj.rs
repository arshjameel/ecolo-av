//! A Wavefront OBJ file as a triangle soup a renderer can draw: each corner's position,
//! normal and picture coordinate. Only what our worlds' meshes use: `v`, `vt`, `vn` and `f`
//! (polygons are fanned into triangles). Materials come from the SDF, not from `mtllib`.

use std::fmt;

/// Triangles, three corners each, in the mesh's own frame.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TriMesh {
    /// Three corners for each triangle.
    pub positions: Vec<[f32; 3]>,
    /// One per corner: the file's, or the face's own if the file has none.
    pub normals: Vec<[f32; 3]>,
    /// One per corner: the file's `vt` (0 where it has none). OBJ's v grows upwards.
    pub uvs: Vec<[f32; 2]>,
}

/// Why an OBJ file can't be read.
#[derive(Clone, Debug, PartialEq)]
pub struct ObjError {
    /// The file's line, from 1.
    pub line: usize,
    /// What's wrong with it.
    pub why: String,
}

impl fmt::Display for ObjError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "line {}: {}", self.line, self.why)
    }
}

impl std::error::Error for ObjError {}

/// The first `N` numbers on a line.
fn floats<'a, const N: usize>(
    words: impl Iterator<Item = &'a str>,
    line: usize,
) -> Result<[f32; N], ObjError> {
    let mut out = [0.0; N];
    let mut n = 0;
    for (slot, word) in out.iter_mut().zip(words) {
        *slot = word.parse().map_err(|e| ObjError {
            line,
            why: format!("{word}: {e}"),
        })?;
        n += 1;
    }
    if n < N {
        return Err(ObjError {
            line,
            why: format!("needs {N} numbers"),
        });
    }
    Ok(out)
}

/// An OBJ index (1-based, or negative from the end) into a list `count` long.
fn index(word: Option<&str>, count: usize) -> Option<usize> {
    let i: i64 = word.filter(|w| !w.is_empty())?.parse().ok()?;
    let i = if i < 0 { count as i64 + i } else { i - 1 };
    (0..count as i64).contains(&i).then_some(i as usize)
}

/// One corner of a face: its vertex, and maybe its picture coordinate and normal.
type Corner = (usize, Option<usize>, Option<usize>);

impl TriMesh {
    /// Reads an OBJ file's text.
    ///
    /// # Errors
    ///
    /// [`ObjError`] with the line if a line can't be read.
    pub fn from_obj(text: &str) -> Result<Self, ObjError> {
        let (mut v, mut vt, mut vn) = (Vec::new(), Vec::new(), Vec::new());
        let mut mesh = TriMesh::default();
        for (n, line) in text.lines().enumerate() {
            let line_no = n + 1;
            let mut words = line.split_whitespace();
            match words.next() {
                Some("v") => v.push(floats::<3>(words, line_no)?),
                Some("vt") => vt.push(floats::<2>(words, line_no)?),
                Some("vn") => vn.push(floats::<3>(words, line_no)?),
                Some("f") => {
                    let corners = words
                        .map(|w| {
                            let mut parts = w.split('/');
                            let p = index(parts.next(), v.len()).ok_or(ObjError {
                                line: line_no,
                                why: format!("{w}: no such vertex"),
                            })?;
                            Ok((
                                p,
                                index(parts.next(), vt.len()),
                                index(parts.next(), vn.len()),
                            ))
                        })
                        .collect::<Result<Vec<Corner>, ObjError>>()?;
                    for i in 1..corners.len().saturating_sub(1) {
                        let tri = [corners[0], corners[i], corners[i + 1]];
                        let flat = face_normal(tri.map(|c| v[c.0]));
                        for (p, t, nn) in tri {
                            mesh.positions.push(v[p]);
                            mesh.uvs.push(t.map_or([0.0, 0.0], |t| vt[t]));
                            mesh.normals.push(nn.map_or(flat, |nn| vn[nn]));
                        }
                    }
                }
                _ => {}
            }
        }
        Ok(mesh)
    }

    /// How many triangles.
    #[must_use]
    pub fn triangles(&self) -> usize {
        self.positions.len() / 3
    }

    /// Scaled along each axis (SDF's `<scale>`); the normals follow.
    #[must_use]
    pub fn scaled(mut self, s: [f32; 3]) -> Self {
        for p in &mut self.positions {
            *p = [p[0] * s[0], p[1] * s[1], p[2] * s[2]];
        }
        for n in &mut self.normals {
            *n = normalise([n[0] / s[0], n[1] / s[1], n[2] / s[2]]);
        }
        self
    }
}

fn normalise(v: [f32; 3]) -> [f32; 3] {
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if l > f32::EPSILON {
        [v[0] / l, v[1] / l, v[2] / l]
    } else {
        [0.0, 0.0, 1.0]
    }
}

/// Counter-clockwise corners face their normal (OBJ's rule).
fn face_normal([a, b, c]: [[f32; 3]; 3]) -> [f32; 3] {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let w = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    normalise([
        u[1] * w[2] - u[2] * w[1],
        u[2] * w[0] - u[0] * w[2],
        u[0] * w[1] - u[1] * w[0],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_square_becomes_two_triangles_facing_up() {
        let m = TriMesh::from_obj(
            "# a square\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n\
             vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nf 1/1 2/2 3/3 4/4\n",
        )
        .unwrap();
        assert_eq!(m.triangles(), 2);
        assert!(m.normals.iter().all(|n| *n == [0.0, 0.0, 1.0]));
        assert_eq!(m.uvs[2], [1.0, 1.0]);
        assert_eq!(m.positions[5], [0.0, 1.0, 0.0]);
    }

    #[test]
    fn the_files_normals_win_and_negative_indices_count_back() {
        let m = TriMesh::from_obj("v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 -1\nf -3//1 -2//1 -1//1\n")
            .unwrap();
        assert_eq!(m.normals, vec![[0.0, 0.0, -1.0]; 3]);
        assert_eq!(m.uvs, vec![[0.0, 0.0]; 3]);
    }

    #[test]
    fn a_missing_vertex_says_where() {
        let e = TriMesh::from_obj("v 0 0 0\nf 1 2 3\n").unwrap_err();
        assert_eq!(e.line, 2);
    }

    #[test]
    fn the_stop_signs_plate() {
        let text = include_str!("../../../../aav_gazebo/models/stop_sign/meshes/stop_sign.obj");
        let m = TriMesh::from_obj(text).unwrap();
        // Two faces of eight triangles each, at least.
        assert!(m.triangles() >= 16, "{}", m.triangles());
        assert!(m
            .uvs
            .iter()
            .all(|uv| (0.0..=1.0).contains(&uv[0]) && (0.0..=1.0).contains(&uv[1])));
    }
}
