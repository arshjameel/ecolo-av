//! Group the lidar's cubes into things: plain Euclidean clustering on a flat grid.
//!
//! `aav_voxels` sends the centre of every 0.25 m cube that holds an obstacle point. Seen
//! from above, cubes closer than `gap` belong to the same thing:
//!
//! ```text
//!     · · ■ ■ · · · · ■          two things: the gap between them is wider than `gap`
//!     · · ■ ■ ■ · · · ■
//!     · · · ■ · · · · ■
//! ```
//!
//! A group with too few cubes is noise; one larger than `max_size` across is a wall or a
//! building, which the map holds, not the list of things.

use std::collections::HashMap;

use crate::Pose2;

/// How cubes become groups.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroupShape {
    /// m: one cube's edge (`aav_voxels`' size).
    pub size: f64,
    /// m: cubes this close (centre to centre, flat) join a group.
    pub gap: f64,
    /// Fewer cubes is noise.
    pub min_cubes: usize,
    /// m: a group longer than this is a wall, not a thing.
    pub max_size: f64,
}

impl Default for GroupShape {
    fn default() -> Self {
        Self {
            size: 0.25,
            gap: 0.6,
            min_cubes: 2,
            max_size: 8.0,
        }
    }
}

/// One group of cubes, in the frame the cubes came in.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Group {
    /// m: the middle of its outline.
    pub x: f64,
    /// m.
    pub y: f64,
    /// m along x: the outline of the cubes' centres, plus one cube.
    pub length: f64,
    /// m along y.
    pub width: f64,
    /// m: the highest cube's top.
    pub height: f64,
    /// How many cubes.
    pub cubes: usize,
}

/// The groups among `points` (each cube's centre, x y z in m), in the order their first
/// cube comes.
///
/// Cubes join when their flat distance is at most `gap`: a flood fill over the grid cells
/// within reach, so the cost grows with the cube count, not the area. Points with a NaN or
/// infinity are left out.
#[must_use]
pub fn group(points: &[[f64; 3]], shape: &GroupShape) -> Vec<Group> {
    let points: Vec<[f64; 3]> = points
        .iter()
        .copied()
        .filter(|p| p.iter().all(|v| v.is_finite()))
        .collect();
    // Each grid cell's cubes, the cells in the order they first come.
    let cell_of = |p: &[f64; 3]| {
        (
            (p[0] / shape.size).floor() as i64,
            (p[1] / shape.size).floor() as i64,
        )
    };
    let mut slot: HashMap<(i64, i64), usize> = HashMap::new();
    let mut cells: Vec<((i64, i64), Vec<usize>)> = Vec::new();
    for (i, p) in points.iter().enumerate() {
        let key = cell_of(p);
        let at = *slot.entry(key).or_insert_with(|| {
            cells.push((key, Vec::new()));
            cells.len() - 1
        });
        cells[at].1.push(i);
    }
    let reach = ((shape.gap / shape.size).ceil() as i64).max(1);
    let steps: Vec<(i64, i64)> = (-reach..=reach)
        .flat_map(|dx| (-reach..=reach).map(move |dy| (dx, dy)))
        .filter(|&d| d != (0, 0))
        .collect();
    let gap2 = shape.gap * shape.gap;
    let first = |c: usize| points[cells[c].1[0]];
    let mut label: Vec<Option<usize>> = vec![None; cells.len()];
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for start in 0..cells.len() {
        if label[start].is_some() {
            continue;
        }
        let id = groups.len();
        label[start] = Some(id);
        let (mut members, mut todo) = (vec![start], vec![start]);
        while let Some(cell) = todo.pop() {
            let (key, here) = (cells[cell].0, first(cell));
            for (dx, dy) in &steps {
                let Some(&near) = slot.get(&(key.0 + dx, key.1 + dy)) else {
                    continue;
                };
                if label[near].is_some() {
                    continue;
                }
                let there = first(near);
                let d2 = (here[0] - there[0]).powi(2) + (here[1] - there[1]).powi(2);
                if d2 <= gap2 {
                    label[near] = Some(id);
                    members.push(near);
                    todo.push(near);
                }
            }
        }
        groups.push(members);
    }
    groups
        .iter()
        .filter_map(|members| {
            let rows: Vec<&[f64; 3]> = members
                .iter()
                .flat_map(|&c| cells[c].1.iter().map(|&i| &points[i]))
                .collect();
            if rows.len() < shape.min_cubes {
                return None;
            }
            let fold = |init: f64, pick: fn(f64, f64) -> f64, axis: usize| {
                rows.iter().map(|p| p[axis]).fold(init, pick)
            };
            let low = [
                fold(f64::INFINITY, f64::min, 0),
                fold(f64::INFINITY, f64::min, 1),
            ];
            let high = [
                fold(f64::NEG_INFINITY, f64::max, 0),
                fold(f64::NEG_INFINITY, f64::max, 1),
            ];
            let (length, width) = (high[0] - low[0] + shape.size, high[1] - low[1] + shape.size);
            if length.max(width) > shape.max_size {
                return None;
            }
            Some(Group {
                x: (low[0] + high[0]) / 2.0,
                y: (low[1] + high[1]) / 2.0,
                length,
                width,
                height: fold(f64::NEG_INFINITY, f64::max, 2) + shape.size / 2.0,
                cubes: rows.len(),
            })
        })
        .collect()
}

/// The groups moved into another frame, given the cubes' frame's pose in it.
///
/// Only the middle turns; length and width stay along the new frame's axes as the outline
/// of the turned box (an upright box that holds the turned one).
#[must_use]
pub fn to_frame(groups: &[Group], pose: Pose2) -> Vec<Group> {
    let (s, c) = pose.yaw.sin_cos();
    groups
        .iter()
        .map(|g| Group {
            x: pose.x + c * g.x - s * g.y,
            y: pose.y + s * g.x + c * g.y,
            length: c.abs() * g.length + s.abs() * g.width,
            width: s.abs() * g.length + c.abs() * g.width,
            ..*g
        })
        .collect()
}
