//! The street map under our map: OpenStreetMap's vector tiles (`pixi run get street-map`),
//! read once at the start for the map's area and placed by its datum (map-core).

use std::path::Path;

use map_core::{
    earth::{Datum, LatLon},
    geom::Point,
    grid::Bounds,
    pmtiles::Archive,
    streets::{self, TileShapes},
    tiles::{covering, MAX_ZOOM},
};

/// How far past our map the street map reaches (m).
pub const MARGIN_M: f64 = 300.0;

/// The tiles round `bounds` (map frame, metres from `datum`), decoded.
///
/// # Errors
///
/// Why there are none: no file, a bad file, a bad datum.
pub fn load(path: &Path, datum: [f64; 2], bounds: Bounds) -> Result<Vec<TileShapes>, String> {
    let datum = Datum::new(LatLon::new(datum[0], datum[1])).map_err(|e| e.to_string())?;
    let bytes = std::fs::read(path).map_err(|e| {
        format!(
            "{}: {e} (pixi run get street-map; or --streets FILE)",
            path.display()
        )
    })?;
    let archive = Archive::new(bytes).map_err(|e| format!("{}: {e}", path.display()))?;
    let seen = Bounds {
        min: bounds.min + Point::new(-MARGIN_M, -MARGIN_M),
        max: bounds.max + Point::new(MARGIN_M, MARGIN_M),
    };
    Ok(covering(&datum, seen, MAX_ZOOM)
        .into_iter()
        .filter_map(|id| {
            let mvt = archive.tile(id).ok().flatten()?;
            streets::decode(id, mvt, &datum).ok()
        })
        .collect())
}
