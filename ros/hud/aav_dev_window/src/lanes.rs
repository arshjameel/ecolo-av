//! The lane map (`--lanes`, a Lanelet2 .osm: `tools/campus_lanelet.py` writes the campus's)
//! as lines to draw. The reading is lanelet-map's; this only picks each line's look.

use lanelet_map::{LaneletMap, LineKind, Origin};
use map_core::geom::Point;

use crate::panels::{LaneLine, LaneLook};

/// Read a Lanelet2 map's painted lines and stop lines, on the map frame.
pub fn read(path: &str) -> Result<Vec<LaneLine>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let map = LaneletMap::from_osm(&text, Origin::default()).map_err(|e| e.to_string())?;
    Ok(map
        .drawn_lines()
        .into_iter()
        .map(|line| LaneLine {
            points: line.points.iter().map(|p| Point::new(p.x, p.y)).collect(),
            look: match line.kind {
                LineKind::Paint { dashed: true } => LaneLook::Dashed,
                LineKind::StopLine => LaneLook::Stop,
                LineKind::Curb => LaneLook::Curb,
                _ => LaneLook::Solid,
            },
        })
        .collect())
}
