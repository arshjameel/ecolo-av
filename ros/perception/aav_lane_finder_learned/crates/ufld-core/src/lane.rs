//! The lines' points onto the road, and the lane: lane-core's fit and `lane_from`, so
//! `LaneGeometry` means the same from either finder.

use lane_core::{fit_line, lane_from, rotation, Lane, Lens, Line, Mount, Paint, Settings};

use crate::{Anchor, Meta, Result};

/// Where a picture's pixel (u, v) meets the flat road (z = 0), in base_link (m): the
/// classic finder's bird's-eye `project`, run backwards. None above the horizon.
///
/// A lens with distortion: the pixel is undistorted first (OpenCV's `undistortPoints`
/// fixed point, 20 steps).
pub fn ground(u: f64, v: f64, lens: &Lens, mount: &Mount) -> Option<[f64; 2]> {
    let (xd, yd) = ((u - lens.cx) / lens.fx, (v - lens.cy) / lens.fy);
    let (x, y) = match &lens.distortion {
        None => (xd, yd),
        Some(d) => (0..20).fold((xd, yd), |(x, y), _| {
            let (bx, by) = d.apply(x, y);
            (x + xd - bx, y + yd - by)
        }),
    };
    // The optical frame (x right, y down, z ahead) into the camera's axes, then base_link's.
    let c = [1.0, -x, -y];
    let r = rotation(mount.rpy);
    let d: [f64; 3] = std::array::from_fn(|i| (0..3).map(|j| r[i][j] * c[j]).sum());
    if d[2] > -1e-9 {
        return None;
    }
    let t = -mount.xyz[2] / d[2];
    Some([mount.xyz[0] + t * d[0], mount.xyz[1] + t * d[1]])
}

/// The learned finder's lane, one picture at a time. Remembers the lane's width as the
/// classic `LaneFinder` does.
#[derive(Debug, Clone)]
pub struct Learned {
    meta: Meta,
    settings: Settings,
    /// m: the lane's width, smoothed over the pictures that saw both lines.
    pub width: f64,
}

impl Learned {
    /// `settings`: the classic finder's (`lane_finder.yaml`): `min_confidence`, `fit_scale`,
    /// `min_span`, the lane widths, `lookahead` and `wheelbase` mean the same here.
    pub fn new(meta: Meta, settings: Settings) -> Self {
        Self {
            width: settings.lane_width,
            meta,
            settings,
        }
    }

    pub fn meta(&self) -> &Meta {
        &self.meta
    }

    /// One line from its points: on the ground, between `fit_x`, fitted as lane-core fits.
    /// Its confidence: the fit's (how close the points lie to it) times the model's mean
    /// chance that the line is there. Also returns the confidence for a line not trusted.
    pub fn line(&self, points: &[Anchor], lens: &Lens, mount: &Mount) -> (Option<Line>, f64) {
        let (sx, sy) = (
            lens.width as f64 / self.meta.width as f64,
            lens.height as f64 / self.meta.height as f64,
        );
        let [lo, hi] = self.meta.fit_x;
        let on_road: Vec<([f64; 2], f64)> = points
            .iter()
            .filter_map(|a| ground(a.u * sx, a.v * sy, lens, mount).map(|g| (g, a.p)))
            .filter(|(g, _)| (lo..=hi).contains(&g[0]))
            .collect();
        let n = on_road.len();
        let x: Vec<f64> = on_road.iter().map(|(g, _)| g[0]).collect();
        let y: Vec<f64> = on_road.iter().map(|(g, _)| g[1]).collect();
        // hits = windows: the fit's confidence is then its scatter alone.
        let s = Settings {
            windows: n.max(1),
            ..self.settings
        };
        let Some(line) = fit_line(&x, &y, n, &s).filter(|_| n >= 4) else {
            return (None, 0.0);
        };
        let p = on_road.iter().map(|(_, p)| p).sum::<f64>() / n as f64;
        let confidence = line.confidence * p;
        let line = Line { confidence, ..line };
        (
            (confidence >= self.settings.min_confidence).then_some(line),
            confidence,
        )
    }

    /// The lane from the model's decoded lines (`decode`'s answer), as the classic finder
    /// reports it: two lines too far apart or too near keep only the surer one.
    ///
    /// `paints`: each line's paint (`paints`' answer, the model's order); empty for a model
    /// without the colour head. A line not kept stays Unknown, as in the classic finder.
    pub fn lane(
        &mut self,
        lines: &[Vec<Anchor>],
        paints: &[Paint],
        lens: &Lens,
        mount: &Mount,
    ) -> Result<Lane> {
        let s = self.settings;
        let paint = |name: &str, line: Option<Line>| -> Result<Paint> {
            let k = self.meta.slot(name)?;
            Ok(line.and(paints.get(k).copied()).unwrap_or(Paint::Unknown))
        };
        let pick = |name: &str| -> Result<(Option<Line>, f64)> {
            let k = self.meta.slot(name)?;
            Ok(lines
                .get(k)
                .map_or((None, 0.0), |pts| self.line(pts, lens, mount)))
        };
        let (mut left, mut lc) = pick("left")?;
        let (mut right, mut rc) = pick("right")?;
        if let (Some(l), Some(r)) = (left, right) {
            let apart = l.y(s.lookahead) - r.y(s.lookahead);
            if !(s.min_lane_width..=s.max_lane_width).contains(&apart) {
                if l.confidence >= r.confidence {
                    right = None;
                    rc = 0.0;
                } else {
                    left = None;
                    lc = 0.0;
                }
            }
        }
        let lane = Lane {
            left_paint: paint("left", left)?,
            right_paint: paint("right", right)?,
            ..lane_from(left, right, self.width, &s, lc, rc)
        };
        if left.is_some() && right.is_some() {
            self.width += 0.2 * (lane.width - self.width);
        }
        Ok(lane)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::meta::example;
    use lane_core::{project, Distortion};

    fn lens() -> Lens {
        Lens::pinhole(640, 360, 343.0)
    }

    const MOUNT: Mount = Mount {
        xyz: [1.75, 0.29, 1.55],
        rpy: [0.0, 0.0, 0.0],
    };

    #[test]
    fn ground_undoes_project() {
        let points = [[5.0, 1.0, 0.0], [12.0, -2.0, 0.0], [30.0, 0.5, 0.0]];
        let tilted = Mount {
            rpy: [0.02, 0.05, -0.03],
            ..MOUNT
        };
        let bent = Lens {
            distortion: Some(
                Distortion::from_coefficients(&[-0.1, 0.02, 0.001, -0.001, 0.0]).unwrap(),
            ),
            ..lens()
        };
        for (lens, mount) in [(lens(), MOUNT), (lens(), tilted), (bent, MOUNT)] {
            for (p, uv) in points.iter().zip(project(&points, &lens, &mount)) {
                let [u, v] = uv.unwrap();
                let g = ground(u, v, &lens, &mount).unwrap();
                assert!(
                    (g[0] - p[0]).abs() < 1e-6 && (g[1] - p[1]).abs() < 1e-6,
                    "{g:?} {p:?}"
                );
            }
        }
        assert!(ground(320.0, 100.0, &lens(), &MOUNT).is_none(), "the sky");
    }

    /// The anchors of a straight line y = `y` on the road, as a perfect model would answer.
    fn anchors(meta: &Meta, y: f64, p: f64) -> Vec<Anchor> {
        let pts: Vec<[f64; 3]> = (0..400).map(|i| [3.0 + i as f64 * 0.1, y, 0.0]).collect();
        let uv: Vec<[f64; 2]> = project(&pts, &lens(), &MOUNT)
            .into_iter()
            .flatten()
            .collect();
        meta.rows
            .iter()
            .filter_map(|&v| {
                let w = uv.windows(2).find(|w| (w[1][1]..=w[0][1]).contains(&v))?;
                let t = (w[0][1] - v) / (w[0][1] - w[1][1]);
                Some(Anchor {
                    u: w[0][0] + t * (w[1][0] - w[0][0]),
                    v,
                    p,
                })
            })
            .collect()
    }

    #[test]
    fn a_straight_lane_gives_the_classic_finders_numbers() {
        let meta = example();
        let mut finder = Learned::new(meta.clone(), Settings::default());
        let lines = vec![
            Vec::new(),
            anchors(&meta, 1.55, 0.9),
            anchors(&meta, -1.95, 0.9),
            Vec::new(),
        ];
        let lane = finder.lane(&lines, &[], &lens(), &MOUNT).unwrap();
        let (l, r) = (lane.left.unwrap(), lane.right.unwrap());
        assert!((l.c - 1.55).abs() < 1e-3 && l.b.abs() < 1e-4 && l.a.abs() < 1e-5);
        assert!((r.c + 1.95).abs() < 1e-3);
        assert!((lane.width - 3.5).abs() < 1e-3);
        assert!(
            (lane.offset - 0.2).abs() < 1e-3,
            "the car 0.2 m left of the centre"
        );
        assert!((lane.left_confidence - 0.9).abs() < 1e-3);
        assert_eq!(
            (lane.left_paint, lane.right_paint),
            (Paint::Unknown, Paint::Unknown),
            "no colour head, no paint"
        );
    }

    #[test]
    fn each_kept_line_carries_its_paint() {
        let meta = example();
        let mut finder = Learned::new(meta.clone(), Settings::default());
        let lines = vec![
            Vec::new(),
            anchors(&meta, 1.55, 0.9),
            anchors(&meta, -1.95, 0.9),
            Vec::new(),
        ];
        let paints = [Paint::White, Paint::Yellow, Paint::White, Paint::Yellow];
        let lane = finder.lane(&lines, &paints, &lens(), &MOUNT).unwrap();
        assert_eq!(
            (lane.left_paint, lane.right_paint),
            (Paint::Yellow, Paint::White)
        );
        // The left line gone: its paint is Unknown, whatever the head said.
        let lines = vec![
            Vec::new(),
            Vec::new(),
            anchors(&meta, -1.95, 0.9),
            Vec::new(),
        ];
        let lane = finder.lane(&lines, &paints, &lens(), &MOUNT).unwrap();
        assert_eq!(
            (lane.left_paint, lane.right_paint),
            (Paint::Unknown, Paint::White)
        );
    }

    #[test]
    fn unsure_or_too_few_points_are_not_a_line() {
        let meta = example();
        let finder = Learned::new(meta.clone(), Settings::default());
        let (line, c) = finder.line(&anchors(&meta, 1.75, 0.2), &lens(), &MOUNT);
        assert!(line.is_none() && (c - 0.2).abs() < 1e-3);
        let few = &anchors(&meta, 1.75, 0.9)[..3];
        assert_eq!(finder.line(few, &lens(), &MOUNT), (None, 0.0));
    }

    #[test]
    fn lines_too_near_keep_the_surer() {
        let meta = example();
        let mut finder = Learned::new(meta.clone(), Settings::default());
        let lines = vec![
            Vec::new(),
            anchors(&meta, 0.5, 0.7),
            anchors(&meta, -0.5, 0.9),
            Vec::new(),
        ];
        let lane = finder.lane(&lines, &[], &lens(), &MOUNT).unwrap();
        assert!(lane.left.is_none() && lane.right.is_some());
        assert_eq!(
            finder.width, 3.5,
            "the width is only learned from both lines"
        );
    }
}
