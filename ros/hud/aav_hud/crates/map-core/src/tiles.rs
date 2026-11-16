//! Map tiles: the squares the street map comes in (Web Mercator, z/x/y, as on
//! openstreetmap.org), which ones the window shows, and at which zoom.

use crate::{
    earth::{from_web_mercator, to_web_mercator, Datum, LatLon},
    geom::Point,
    grid::Bounds,
    view::PixelsPerMetre,
};

/// One tile: zoom `z` cuts the world into 2^z x 2^z squares; x goes east, y south.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TileId {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

/// The street map's deepest zoom (the extract's `--maxzoom`); closer views stretch it.
pub const MAX_ZOOM: u8 = 15;
/// How many screen pixels a tile should cover: vector tiles are made for 512.
const TILE_PX: f64 = 512.0;
/// The Earth's equator, in metres (WGS 84).
const EQUATOR_M: f64 = 40_075_016.686;
/// Never more tiles than this in one view: a huge window zoomed far out shows fewer.
pub const MAX_TILES: usize = 64;

impl TileId {
    #[must_use]
    pub const fn new(z: u8, x: u32, y: u32) -> Self {
        Self { z, x, y }
    }

    /// The tile one zoom out that holds this one; None at zoom 0.
    #[must_use]
    pub fn parent(self) -> Option<Self> {
        (self.z > 0).then(|| Self::new(self.z - 1, self.x / 2, self.y / 2))
    }

    /// Where a point inside the tile is on the Earth: `(u, v)` from 0 to 1 across it,
    /// v down (the tile's own pixels, divided by its extent).
    #[must_use]
    pub fn lat_lon(self, u: f64, v: f64) -> LatLon {
        let n = f64::from(1u32 << self.z);
        from_web_mercator((f64::from(self.x) + u) / n, (f64::from(self.y) + v) / n)
    }
}

/// The zoom whose tiles come out about [`TILE_PX`] wide at this scale, at latitude `lat`
/// (degrees), from 0 to [`MAX_ZOOM`].
#[must_use]
pub fn zoom_for(scale: PixelsPerMetre, lat: f64) -> u8 {
    let tile_m_at_zoom_0 = EQUATOR_M * lat.to_radians().cos();
    let z = (tile_m_at_zoom_0 * scale.0 / TILE_PX).log2().round();
    if z.is_finite() {
        z.clamp(0.0, f64::from(MAX_ZOOM)) as u8
    } else {
        0
    }
}

/// The tiles at zoom `z` that cover a box on the map, the middle ones first. At most
/// [`MAX_TILES`]: past that, the outer ones are left out.
pub fn covering(datum: &Datum, seen: Bounds, z: u8) -> Vec<TileId> {
    let n = 1u32 << z;
    let corners = [
        seen.min,
        seen.max,
        Point::new(seen.min.x, seen.max.y),
        Point::new(seen.max.x, seen.min.y),
    ]
    .map(|p| to_web_mercator(datum.to_lat_lon(p)));
    let fold = |f: fn(f64, f64) -> f64, pick: fn(&(f64, f64)) -> f64, start: f64| {
        corners.iter().map(pick).fold(start, f)
    };
    let to_tile = |w: f64| ((w * f64::from(n)).floor().max(0.0) as u32).min(n - 1);
    let (x0, x1) = (
        to_tile(fold(f64::min, |c| c.0, f64::INFINITY)),
        to_tile(fold(f64::max, |c| c.0, f64::NEG_INFINITY)),
    );
    let (y0, y1) = (
        to_tile(fold(f64::min, |c| c.1, f64::INFINITY)),
        to_tile(fold(f64::max, |c| c.1, f64::NEG_INFINITY)),
    );
    let (cx, cy) = (f64::from(x0 + x1) / 2.0, f64::from(y0 + y1) / 2.0);
    let mut tiles: Vec<TileId> = (y0..=y1)
        .flat_map(|y| (x0..=x1).map(move |x| TileId::new(z, x, y)))
        .collect();
    tiles.sort_by(|a, b| {
        let d = |t: &TileId| (f64::from(t.x) - cx).hypot(f64::from(t.y) - cy);
        d(a).total_cmp(&d(b))
    });
    tiles.truncate(MAX_TILES);
    tiles
}

#[cfg(test)]
mod tests {
    use super::*;

    fn datum() -> Datum {
        Datum::new(LatLon::new(45.3860, -75.6960)).expect("valid")
    }

    #[test]
    fn carleton_is_in_the_right_tile() {
        // openstreetmap.org: 45.3835 N 75.6974 W is in tile 15/9493/11737.
        let d = datum();
        let here = d.to_map(LatLon::new(45.3835, -75.6974));
        let seen = Bounds {
            min: here,
            max: here,
        };
        assert_eq!(covering(&d, seen, 15), [TileId::new(15, 9493, 11737)]);
        let t = TileId::new(15, 9493, 11737);
        let (nw, se) = (t.lat_lon(0.0, 0.0), t.lat_lon(1.0, 1.0));
        assert!(nw.lat > 45.3835 && se.lat < 45.3835);
        assert!(nw.lon < -75.6974 && se.lon > -75.6974);
        assert_eq!(t.parent(), Some(TileId::new(14, 4746, 5868)));
        assert_eq!(TileId::new(0, 0, 0).parent(), None);
    }

    #[test]
    fn the_zoom_follows_the_scale() {
        // Close in (60 m across 1200 px): the deepest tiles.
        assert_eq!(zoom_for(PixelsPerMetre(20.0), 45.4), MAX_ZOOM);
        // A tile at zoom 13 is about 3.4 km across at 45 N; 512 px of it is ~0.15 px/m.
        assert_eq!(zoom_for(PixelsPerMetre(0.15), 45.4), 13);
        assert_eq!(zoom_for(PixelsPerMetre(1e-9), 45.4), 0);
        assert_eq!(zoom_for(PixelsPerMetre(f64::NAN), 45.4), 0);
    }

    #[test]
    fn a_view_gets_the_tiles_around_it_middle_first() {
        let d = datum();
        let seen = Bounds {
            min: Point::new(-1500.0, -1000.0),
            max: Point::new(1500.0, 1000.0),
        };
        let tiles = covering(&d, seen, 15);
        // A zoom-15 tile is ~860 m across here: 4 or 5 columns, 3 or 4 rows.
        assert!((12..=20).contains(&tiles.len()), "{}", tiles.len());
        let middle = covering(
            &d,
            Bounds {
                min: Point::default(),
                max: Point::default(),
            },
            15,
        )[0];
        assert!(tiles[..4].contains(&middle));
        // Far out, the count is capped.
        let huge = Bounds {
            min: Point::new(-50_000.0, -50_000.0),
            max: Point::new(50_000.0, 50_000.0),
        };
        assert_eq!(covering(&d, huge, 15).len(), MAX_TILES);
    }
}
