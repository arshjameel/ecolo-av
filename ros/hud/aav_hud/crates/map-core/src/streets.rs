//! One street-map tile, ready to draw: its areas as triangles and its lines as points,
//! all in map metres.
//!
//! A tile is a Mapbox Vector Tile from Protomaps' basemap (layers `water`, `landuse`,
//! `landcover`, `buildings`, `roads`; their `kind`s are listed at
//! <https://docs.protomaps.com/basemaps/layers>). `mvt-reader` (MIT) decodes it; `earcut`
//! (MIT or Apache-2.0) cuts each area into triangles. Points go from the tile's own
//! pixels to the map frame through [`TileProjection`].

use mvt_reader::{feature::Value, Reader};
use thiserror::Error;

use crate::{earth::Datum, geom::Point, tiles::TileId};

/// A filled area, in the order they are drawn (later ones on top).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Area {
    /// Parks, woods, grass, sports fields, cemeteries.
    Green,
    /// Universities, schools, hospitals: their grounds.
    Campus,
    Water,
    Building,
}

impl Area {
    pub const ALL: [Area; 4] = [Area::Green, Area::Campus, Area::Water, Area::Building];
}

/// What a road is, from the least to the most important (the drawing order).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum RoadClass {
    /// Footways, cycleways, paths, steps.
    Path,
    Rail,
    /// Driveways, parking aisles, alleys.
    Service,
    /// Residential and unclassified streets.
    Minor,
    /// Tertiary roads.
    Medium,
    /// Primary and secondary roads: Bronson Avenue, Colonel By Drive.
    Major,
    /// Motorways and trunk roads: the 417.
    Highway,
}

impl RoadClass {
    pub const ALL: [RoadClass; 7] = [
        RoadClass::Path,
        RoadClass::Rail,
        RoadClass::Service,
        RoadClass::Minor,
        RoadClass::Medium,
        RoadClass::Major,
        RoadClass::Highway,
    ];

    /// About how wide the real road is, in metres: the line's width when zoomed in.
    pub fn width_m(self) -> f64 {
        match self {
            Self::Path => 2.0,
            Self::Rail => 2.0,
            Self::Service => 5.0,
            Self::Minor => 8.0,
            Self::Medium => 10.0,
            Self::Major => 13.0,
            Self::Highway => 18.0,
        }
    }

    /// From Protomaps' `kind` and `kind_detail`; None for what isn't drawn (ferries).
    fn from_kind(kind: &str, detail: &str) -> Option<Self> {
        Some(match kind {
            "highway" => Self::Highway,
            "major_road" => Self::Major,
            "medium_road" => Self::Medium,
            "minor_road" if detail == "service" => Self::Service,
            "minor_road" => Self::Minor,
            "path" => Self::Path,
            "rail" => Self::Rail,
            _ => return None,
        })
    }
}

/// What a line is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LineKind {
    Road(RoadClass),
    /// Rivers, streams, canals drawn as lines (the Rideau Canal is also an area).
    Waterway,
}

/// A line: a road or a stream.
#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    pub kind: LineKind,
    /// Map metres, x east, y north.
    pub points: Vec<[f32; 2]>,
}

/// A street's name, to write along it.
#[derive(Debug, Clone, PartialEq)]
pub struct Label {
    pub text: String,
    pub class: RoadClass,
    /// The middle of the street (along it), in map metres.
    pub at: [f32; 2],
    /// The street's direction there, radians counter-clockwise from east, turned so
    /// the text never reads upside down (between -90 and +90 degrees).
    pub angle: f32,
    /// How long the street is in this tile, metres: a name longer than that is left out.
    pub length_m: f32,
}

/// One tile, decoded.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TileShapes {
    /// Per kind of area: triangles, three points each, map metres.
    pub areas: Vec<(Area, Vec<[f32; 2]>)>,
    pub lines: Vec<Line>,
    pub labels: Vec<Label>,
}

impl TileShapes {
    /// The triangles of one kind of area.
    pub fn triangles(&self, area: Area) -> &[[f32; 2]] {
        self.areas
            .iter()
            .find(|(a, _)| *a == area)
            .map_or(&[], |(_, t)| t.as_slice())
    }

    /// How many points it holds: what it costs to keep and to draw.
    pub fn points(&self) -> usize {
        self.areas.iter().map(|(_, t)| t.len()).sum::<usize>()
            + self.lines.iter().map(|l| l.points.len()).sum::<usize>()
    }
}

/// Why a tile can't be decoded.
#[derive(Debug, Error)]
pub enum StreetError {
    #[error("not a vector tile: {0}")]
    Mvt(#[from] mvt_reader::error::ParserError),
}

/// Lattice cells per tile side: the exact conversion at (N+1)^2 points, straight lines
/// between them. At zoom 15 a cell is about 110 m, and the error under a millimetre.
const LATTICE: usize = 8;

/// From a tile's own pixels to map metres: exact at a lattice of points, interpolated
/// between them (thousands of times faster than the exact conversion, and as good).
#[derive(Debug, Clone)]
pub struct TileProjection {
    corners: Vec<Point>,
}

impl TileProjection {
    pub fn new(id: TileId, datum: &Datum) -> Self {
        let step = 1.0 / LATTICE as f64;
        let corners = (0..=LATTICE)
            .flat_map(|j| (0..=LATTICE).map(move |i| (i, j)))
            .map(|(i, j)| datum.to_map(id.lat_lon(i as f64 * step, j as f64 * step)))
            .collect();
        Self { corners }
    }

    /// `(u, v)` from 0 to 1 across the tile (v down), a little past the edges allowed.
    pub fn to_map(&self, u: f64, v: f64) -> Point {
        let n = LATTICE as f64;
        let (fu, fv) = (u * n, v * n);
        let (i, j) = (
            (fu.floor() as i64).clamp(0, LATTICE as i64 - 1) as usize,
            (fv.floor() as i64).clamp(0, LATTICE as i64 - 1) as usize,
        );
        let (s, t) = (fu - i as f64, fv - j as f64);
        let at = |i: usize, j: usize| self.corners[j * (LATTICE + 1) + i];
        let top = at(i, j) + (at(i + 1, j) - at(i, j)) * s;
        let bottom = at(i, j + 1) + (at(i + 1, j + 1) - at(i, j + 1)) * s;
        top + (bottom - top) * t
    }
}

fn text<'a>(props: &'a std::collections::HashMap<String, Value>, key: &str) -> &'a str {
    match props.get(key) {
        Some(Value::String(s)) => s,
        _ => "",
    }
}

/// The area a polygon in this layer fills, if it's drawn.
fn area_of(layer: &str, kind: &str) -> Option<Area> {
    match layer {
        "water" => Some(Area::Water),
        "buildings" => Some(Area::Building),
        "landuse" | "landcover" => match kind {
            "park" | "forest" | "wood" | "grass" | "grassland" | "nature_reserve" | "cemetery"
            | "golf_course" | "garden" | "meadow" | "playground" | "pitch"
            | "recreation_ground" | "village_green" | "scrub" | "wetland" | "dog_park"
            | "national_park" | "protected_area" | "zoo" => Some(Area::Green),
            "university" | "college" | "school" | "hospital" | "kindergarten" => Some(Area::Campus),
            _ => None,
        },
        _ => None,
    }
}

/// Decodes one tile's bytes (unzipped MVT) into shapes in map metres.
pub fn decode(id: TileId, mvt: Vec<u8>, datum: &Datum) -> Result<TileShapes, StreetError> {
    let reader = Reader::new(mvt)?;
    let proj = TileProjection::new(id, datum);
    let mut areas: Vec<(Area, Vec<[f32; 2]>)> =
        Area::ALL.iter().map(|&a| (a, Vec::new())).collect();
    let mut lines = Vec::new();
    let mut labels = Vec::new();
    let mut earcut = earcut::Earcut::new();
    let mut scratch = Triangulation::default();
    for layer in reader.get_layer_metadata()? {
        let extent = f64::from(layer.extent.max(1));
        let to_map = |c: geo_types::Coord<f32>| -> [f32; 2] {
            let p = proj.to_map(f64::from(c.x) / extent, f64::from(c.y) / extent);
            [p.x as f32, p.y as f32]
        };
        for f in reader.get_features(layer.layer_index)? {
            let empty = std::collections::HashMap::new();
            let props = f.properties.as_ref().unwrap_or(&empty);
            let kind = text(props, "kind");
            match &f.geometry {
                geo_types::Geometry::Polygon(p) => {
                    if let Some(a) = area_of(&layer.name, kind) {
                        let out = &mut areas[a as usize].1;
                        scratch.polygon(&mut earcut, p, &to_map, out);
                    }
                }
                geo_types::Geometry::MultiPolygon(mp) => {
                    if let Some(a) = area_of(&layer.name, kind) {
                        let out = &mut areas[a as usize].1;
                        for p in mp {
                            scratch.polygon(&mut earcut, p, &to_map, out);
                        }
                    }
                }
                geo_types::Geometry::LineString(ls) => {
                    line(
                        &layer.name,
                        props,
                        std::slice::from_ref(ls),
                        &to_map,
                        &mut lines,
                        &mut labels,
                    );
                }
                geo_types::Geometry::MultiLineString(mls) => {
                    line(&layer.name, props, &mls.0, &to_map, &mut lines, &mut labels);
                }
                _ => {}
            }
        }
    }
    areas.retain(|(_, t)| !t.is_empty());
    // Draw the minor roads first, so the major ones cross over them.
    lines.sort_by_key(|l: &Line| match l.kind {
        LineKind::Waterway => None,
        LineKind::Road(c) => Some(c),
    });
    Ok(TileShapes {
        areas,
        lines,
        labels,
    })
}

/// A road or a waterway, and its name.
fn line(
    layer: &str,
    props: &std::collections::HashMap<String, Value>,
    parts: &[geo_types::LineString<f32>],
    to_map: &impl Fn(geo_types::Coord<f32>) -> [f32; 2],
    lines: &mut Vec<Line>,
    labels: &mut Vec<Label>,
) {
    let kind = match layer {
        "roads" => match RoadClass::from_kind(text(props, "kind"), text(props, "kind_detail")) {
            Some(c) => LineKind::Road(c),
            None => return,
        },
        "water" => LineKind::Waterway,
        _ => return,
    };
    let name = text(props, "name");
    for part in parts {
        let points: Vec<[f32; 2]> = part.0.iter().map(|&c| to_map(c)).collect();
        if points.len() < 2 {
            continue;
        }
        if let (LineKind::Road(class), false) = (kind, name.is_empty()) {
            if class >= RoadClass::Minor {
                labels.extend(label(name, class, &points));
            }
        }
        lines.push(Line { kind, points });
    }
}

/// Where a street's name goes: halfway along it, turned along the street there.
fn label(name: &str, class: RoadClass, points: &[[f32; 2]]) -> Option<Label> {
    let seg = |w: &[[f32; 2]]| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]);
    let length: f32 = points.windows(2).map(seg).sum();
    let mut left = length / 2.0;
    let w = points.windows(2).find(|w| {
        let s = seg(w);
        if left <= s {
            true
        } else {
            left -= s;
            false
        }
    })?;
    let s = seg(w).max(1e-6);
    let k = left / s;
    let (dx, dy) = (w[1][0] - w[0][0], w[1][1] - w[0][1]);
    let mut angle = dy.atan2(dx);
    if angle > std::f32::consts::FRAC_PI_2 {
        angle -= std::f32::consts::PI;
    } else if angle < -std::f32::consts::FRAC_PI_2 {
        angle += std::f32::consts::PI;
    }
    Some(Label {
        text: name.to_owned(),
        class,
        at: [w[0][0] + dx * k, w[0][1] + dy * k],
        angle,
        length_m: length,
    })
}

/// Buffers kept between polygons, so triangulating thousands of buildings allocates
/// little.
#[derive(Default)]
struct Triangulation {
    points: Vec<[f64; 2]>,
    holes: Vec<u32>,
    triangles: Vec<u32>,
}

impl Triangulation {
    /// Cuts a polygon (its outside ring and its holes) into triangles, appended to `out`
    /// as map points.
    fn polygon(
        &mut self,
        earcut: &mut earcut::Earcut<f64>,
        p: &geo_types::Polygon<f32>,
        to_map: &impl Fn(geo_types::Coord<f32>) -> [f32; 2],
        out: &mut Vec<[f32; 2]>,
    ) {
        self.points.clear();
        self.holes.clear();
        for (i, ring) in std::iter::once(p.exterior())
            .chain(p.interiors())
            .enumerate()
        {
            if i > 0 {
                self.holes.push(self.points.len() as u32);
            }
            // The ring's last point repeats its first: leave it out.
            let n = ring.0.len().saturating_sub(1);
            self.points.extend(
                ring.0[..n]
                    .iter()
                    .map(|&c| to_map(c))
                    .map(|[x, y]| [f64::from(x), f64::from(y)]),
            );
        }
        earcut.earcut(
            self.points.iter().copied(),
            &self.holes,
            &mut self.triangles,
        );
        out.extend(self.triangles.iter().map(|&i| {
            let [x, y] = self.points[i as usize];
            [x as f32, y as f32]
        }));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::earth::LatLon;

    fn datum() -> Datum {
        Datum::new(LatLon::new(45.3860, -75.6960)).expect("valid")
    }

    #[test]
    fn the_lattice_is_as_good_as_the_exact_conversion() {
        let d = datum();
        for id in [TileId::new(15, 9493, 11737), TileId::new(12, 1186, 1467)] {
            let proj = TileProjection::new(id, &d);
            let worst = (0..=40)
                .flat_map(|j| (0..=40).map(move |i| (f64::from(i) / 40.0, f64::from(j) / 40.0)))
                .map(|(u, v)| proj.to_map(u, v).distance(d.to_map(id.lat_lon(u, v))).0)
                .fold(0.0, f64::max);
            // Zoom 15: under a millimetre. Zoom 12 (7 km across): under 5 cm.
            let limit = if id.z == 15 { 0.001 } else { 0.05 };
            assert!(worst < limit, "{id:?}: {worst} m");
        }
    }

    #[test]
    fn roads_by_class() {
        assert_eq!(
            RoadClass::from_kind("highway", "motorway"),
            Some(RoadClass::Highway)
        );
        assert_eq!(
            RoadClass::from_kind("major_road", "primary"),
            Some(RoadClass::Major)
        );
        assert_eq!(
            RoadClass::from_kind("minor_road", "service"),
            Some(RoadClass::Service)
        );
        assert_eq!(
            RoadClass::from_kind("minor_road", "residential"),
            Some(RoadClass::Minor)
        );
        assert_eq!(RoadClass::from_kind("ferry", ""), None);
        assert!(RoadClass::Highway.width_m() > RoadClass::Minor.width_m());
        assert_eq!(area_of("landuse", "park"), Some(Area::Green));
        assert_eq!(area_of("landuse", "university"), Some(Area::Campus));
        assert_eq!(area_of("landuse", "residential"), None);
        assert_eq!(area_of("buildings", "building"), Some(Area::Building));
    }

    #[test]
    fn a_label_sits_halfway_and_reads_left_to_right() {
        // A street drawn westward: the text still reads from left to right.
        let pts = [[100.0, 0.0], [0.0, 0.0]];
        let l = label("Bronson Avenue", RoadClass::Major, &pts).expect("a label");
        assert_eq!(l.at, [50.0, 0.0]);
        assert!(l.angle.abs() < 1e-6);
        assert_eq!(l.length_m, 100.0);
        let up = label(
            "x",
            RoadClass::Minor,
            &[[0.0, 0.0], [0.0, 10.0], [0.0, 30.0]],
        )
        .expect("a label");
        assert_eq!(up.at, [0.0, 15.0]);
        assert!((up.angle - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }

    /// The real tile at Carleton, when `pixi run get street-map` has downloaded it:
    /// buildings, green, water (the Rideau River and Canal), and named major roads.
    #[test]
    fn carleton_decodes() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../../../maps/streets/ottawa.pmtiles"
        );
        let Ok(bytes) = std::fs::read(path) else {
            eprintln!("skipped: no {path} (pixi run get street-map)");
            return;
        };
        let archive = crate::pmtiles::Archive::new(bytes).expect("the extract reads");
        let id = TileId::new(15, 9493, 11737);
        let mvt = archive.tile(id).expect("readable").expect("in the extract");
        let d = datum();
        let t = decode(id, mvt, &d).expect("decodes");
        for a in [Area::Building, Area::Green, Area::Water] {
            assert!(t.triangles(a).len() >= 3, "{a:?}");
            assert_eq!(t.triangles(a).len() % 3, 0);
        }
        assert!(t
            .lines
            .iter()
            .any(|l| l.kind == LineKind::Road(RoadClass::Major)));
        // Everything is near Carleton: within the tile (~860 m) of the datum, give or
        // take the tile's buffer.
        let all = t
            .areas
            .iter()
            .flat_map(|(_, v)| v)
            .chain(t.lines.iter().flat_map(|l| &l.points));
        assert!(all
            .into_iter()
            .all(|p| p[0].abs() < 2000.0 && p[1].abs() < 2000.0));
        assert!(t
            .labels
            .iter()
            .any(|l| l.text.contains("Bronson") || l.text.contains("Colonel By")));
    }
}
