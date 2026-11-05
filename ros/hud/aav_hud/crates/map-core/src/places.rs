//! The place search: names and addresses around Ottawa, from `ottawa.db`.
//!
//! `ottawa.db` is SQLite, made by the 2025-26 team from OpenStreetMap (© OpenStreetMap
//! contributors, ODbL): one table,
//! `places(id, osm_id, name, type, house_number, street, city, lat, lon)`, with indexes on
//! `name` and `street`. 606,628 rows; over half are no use as a destination (street
//! lamps, crossings, benches...), and 300,000 are addresses with no name.
//!
//! How a search goes (about 1 to 30 ms on a laptop):
//!
//! | Step | SQL | Uses |
//! |---|---|---|
//! | 1. names that start with the words | `name >= 'Carl' AND name < 'Carl\u{10FFFF}'`, for a few spellings (carl, Carl, CARL) | the name index |
//! | 2. an address, "580 rideau" | `street >= 'Rideau' ... AND house_number = '580'` | the street index |
//! | 3. only if 1 and 2 found too few: names with the words inside | `name LIKE '%carl%'` | a scan of the table (~20 ms) |
//!
//! The index is case-sensitive (SQLite's LIKE can't use it), so step 1 tries the
//! spellings people type and OpenStreetMap uses; step 3 catches the rest ("McDonald's"
//! for "mcdonald"). Then: junk types out ([`NOT_DESTINATIONS`]), the same place once
//! (a street is many pieces: the nearest piece stays), ranked exact > starts with >
//! a word starts with > inside, then nearest first, and at most [`MAX_RESULTS`].

use std::{
    cmp::Ordering,
    collections::HashMap,
    path::{Path, PathBuf},
};

use rusqlite::{Connection, OpenFlags, Row};
use thiserror::Error;

use crate::{earth::LatLon, geom::Metres};

/// The most results a search gives: what fits under the search box.
pub const MAX_RESULTS: usize = 8;

/// Fewer letters than this: no search (one letter matches thousands of places).
pub const MIN_QUERY_CHARS: usize = 2;

/// What every screen that shows these places must say (ODbL).
pub const CREDIT: &str = "© OpenStreetMap contributors";

/// OpenStreetMap types that are never a destination, left out of every search.
///
/// A deny-list, not an allow-list: the file has over 300 types (every kind of shop), and
/// a new one should show up, not vanish. These are street furniture, bits of road, and
/// paths a car can't drive.
pub const NOT_DESTINATIONS: [&str; 41] = [
    // Street furniture and signs.
    "street_lamp",
    "bench",
    "post_box",
    "waste_basket",
    "waste_disposal",
    "recycling",
    "vending_machine",
    "telephone",
    "drinking_water",
    "grit_bin",
    "compressed_air",
    "bicycle_repair_station",
    "bicycle_parking",
    "public_bookcase",
    "information",
    "shelter",
    "elevator",
    "trolley_bay",
    "speed_display",
    "speed_camera",
    // Bits of road, and junctions.
    "crossing",
    "stop",
    "give_way",
    "traffic_signals",
    "turning_circle",
    "motorway_junction",
    "motorway_link",
    "trunk_link",
    "primary_link",
    "secondary_link",
    "tertiary_link",
    "parking_space",
    "parking_entrance",
    // Not for a car, or not there yet.
    "footway",
    "path",
    "steps",
    "cycleway",
    "track",
    "service",
    "construction",
    "proposed",
];

/// How a place's name matches what was typed: the better first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Match {
    /// The whole name.
    Exact,
    /// The name starts with it.
    Prefix,
    /// A word in the name starts with it ("univ" in "Carleton University").
    WordStart,
    /// Somewhere inside.
    Contains,
}

impl Match {
    /// How `name` matches `query`; both already lower case. None: it doesn't.
    pub fn of(name: &str, query: &str) -> Option<Self> {
        if query.is_empty() {
            return None;
        }
        if name == query {
            return Some(Self::Exact);
        }
        if name.starts_with(query) {
            return Some(Self::Prefix);
        }
        let word_start = name
            .match_indices(query)
            .any(|(i, _)| name[..i].ends_with(|c: char| !c.is_alphanumeric()));
        if word_start {
            Some(Self::WordStart)
        } else if name.contains(query) {
            Some(Self::Contains)
        } else {
            None
        }
    }
}

/// One place found.
#[derive(Debug, Clone, PartialEq)]
pub struct Place {
    /// The row's id in `places`.
    pub id: i64,
    /// What to call it: its name, or for an address with no name, "580 Rideau Street".
    pub title: String,
    /// OpenStreetMap's type ("university", "restaurant"), or "address".
    pub kind: String,
    /// "1125 Colonel By Drive", when the row has a street and isn't only an address.
    pub address: Option<String>,
    pub at: LatLon,
    pub matched: Match,
    /// From the place the search was asked near, when one was given.
    pub distance: Option<Metres>,
}

impl Place {
    /// The line under the title: "university · 1125 Colonel By Drive".
    pub fn detail(&self) -> String {
        let kind = self.kind.replace('_', " ");
        match &self.address {
            Some(a) => format!("{kind} · {a}"),
            None => kind,
        }
    }

    /// Ranking: the match, then the distance (unknown last), then the row, so a search
    /// always gives the same order.
    fn rank(&self, other: &Self) -> Ordering {
        let far = |p: &Self| p.distance.map_or(f64::INFINITY, |d| d.0);
        self.matched
            .cmp(&other.matched)
            .then(far(self).total_cmp(&far(other)))
            .then(self.id.cmp(&other.id))
    }
}

/// Why the place file can't be used.
#[derive(Debug, Error)]
pub enum PlacesError {
    #[error("no place database at {0}: pixi run get places")]
    Missing(PathBuf),
    #[error("can't open {path}: {source}")]
    Open {
        path: PathBuf,
        source: rusqlite::Error,
    },
    #[error("{path} has no places table: {source}")]
    NotPlaces {
        path: PathBuf,
        source: rusqlite::Error,
    },
    #[error("searching: {0}")]
    Query(#[from] rusqlite::Error),
}

/// The place file, opened read-only.
pub struct Places {
    conn: Connection,
}

impl std::fmt::Debug for Places {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Places").finish_non_exhaustive()
    }
}

/// The columns every query reads, in this order (see [`Candidate::from_row`]).
const COLUMNS: &str = "id, name, type, house_number, street, lat, lon";
/// Rows looked at per spelling, before ranking. Enough for the nearest to be in it.
const CANDIDATES: usize = 300;

/// One row, before it is ranked.
struct Candidate {
    id: i64,
    name: String,
    kind: String,
    house_number: String,
    street: String,
    at: LatLon,
}

impl Candidate {
    fn from_row(row: &Row<'_>) -> rusqlite::Result<Self> {
        let text = |i: usize| -> rusqlite::Result<String> {
            Ok(row.get::<_, Option<String>>(i)?.unwrap_or_default())
        };
        Ok(Self {
            id: row.get(0)?,
            name: text(1)?.trim().to_owned(),
            kind: text(2)?,
            house_number: text(3)?.trim().to_owned(),
            street: text(4)?.trim().to_owned(),
            at: LatLon::new(row.get(5)?, row.get(6)?),
        })
    }

    /// "580 Rideau Street", or the street alone, or None.
    fn address(&self) -> Option<String> {
        match (self.house_number.is_empty(), self.street.is_empty()) {
            (_, true) => None,
            (true, false) => Some(self.street.clone()),
            (false, false) => Some(format!("{} {}", self.house_number, self.street)),
        }
    }

    /// The place, if it matches `query` (lower case).
    fn into_place(self, query: &str, near: Option<LatLon>) -> Option<Place> {
        let address = self.address();
        let (title, kind, address) = if self.name.is_empty() {
            (address?, "address".to_owned(), None)
        } else {
            (self.name, self.kind, address)
        };
        let by_title = Match::of(&title.to_lowercase(), query);
        let by_address = address
            .as_deref()
            .and_then(|a| Match::of(&a.to_lowercase(), query));
        let matched = by_title.into_iter().chain(by_address).min()?;
        Some(Place {
            id: self.id,
            title,
            kind,
            address,
            at: self.at,
            matched,
            distance: near.map(|n| great_circle(n, self.at)),
        })
    }
}

impl Places {
    /// Opens the file read-only (it is never changed), and checks it has the table.
    pub fn open(path: &Path) -> Result<Self, PlacesError> {
        if !path.is_file() {
            return Err(PlacesError::Missing(path.to_path_buf()));
        }
        let flags = OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX;
        let conn =
            Connection::open_with_flags(path, flags).map_err(|source| PlacesError::Open {
                path: path.to_path_buf(),
                source,
            })?;
        conn.prepare(&format!("SELECT {COLUMNS} FROM places LIMIT 0"))
            .map_err(|source| PlacesError::NotPlaces {
                path: path.to_path_buf(),
                source,
            })?;
        Ok(Self { conn })
    }

    /// The best [`MAX_RESULTS`] places for `query`, nearest to `near` first among
    /// equal matches. Fewer than [`MIN_QUERY_CHARS`] letters: none.
    pub fn search(&self, query: &str, near: Option<LatLon>) -> Result<Vec<Place>, PlacesError> {
        let query = query.split_whitespace().collect::<Vec<_>>().join(" ");
        if query.chars().count() < MIN_QUERY_CHARS {
            return Ok(Vec::new());
        }
        let lower = query.to_lowercase();
        let mut found = Found::default();

        // 1. Names that start with it, through the name index.
        let by_name = format!(
            "SELECT {COLUMNS} FROM places WHERE name >= ?1 AND name < ?2 AND {} LIMIT {CANDIDATES}",
            not_junk()
        );
        for spelling in spellings(&query) {
            let upper = format!("{spelling}\u{10FFFF}");
            self.collect(&by_name, (&spelling, &upper), &lower, near, &mut found)?;
        }

        // 2. An address: a house number, then the street.
        if let Some((number, street)) = address_query(&query) {
            let by_address = format!(
                "SELECT {COLUMNS} FROM places WHERE street >= ?1 AND street < ?2 \
                 AND house_number = ?3 LIMIT {CANDIDATES}"
            );
            for spelling in spellings(street) {
                let upper = format!("{spelling}\u{10FFFF}");
                self.collect(
                    &by_address,
                    (&spelling, &upper, number),
                    &lower,
                    near,
                    &mut found,
                )?;
            }
        }

        // 3. Too few: the words anywhere in the name (a scan, ASCII case-insensitive).
        if found.len() < MAX_RESULTS {
            let inside = format!(
                "SELECT {COLUMNS} FROM places WHERE name LIKE ?1 ESCAPE '\\' AND {} \
                 LIMIT {CANDIDATES}",
                not_junk()
            );
            let pattern = format!("%{}%", escape_like(&query));
            self.collect(&inside, (&pattern,), &lower, near, &mut found)?;
        }
        Ok(found.best())
    }

    fn collect(
        &self,
        sql: &str,
        params: impl rusqlite::Params,
        query: &str,
        near: Option<LatLon>,
        found: &mut Found,
    ) -> Result<(), PlacesError> {
        let mut stmt = self.conn.prepare_cached(sql)?;
        let rows = stmt.query_map(params, Candidate::from_row)?;
        for row in rows {
            if let Some(place) = row?.into_place(query, near) {
                found.add(place);
            }
        }
        Ok(())
    }
}

/// The places found so far, each once.
#[derive(Default)]
struct Found {
    /// By title and address, lower case: a street's pieces, or a shop's two rows, are
    /// one place. The better one stays.
    places: HashMap<(String, String), Place>,
}

impl Found {
    fn len(&self) -> usize {
        self.places.len()
    }

    fn add(&mut self, place: Place) {
        let key = (
            place.title.to_lowercase(),
            place.address.as_deref().unwrap_or_default().to_lowercase(),
        );
        match self.places.get(&key) {
            Some(old) if old.rank(&place).is_le() => {}
            _ => {
                self.places.insert(key, place);
            }
        }
    }

    fn best(self) -> Vec<Place> {
        let mut all: Vec<Place> = self.places.into_values().collect();
        all.sort_by(Place::rank);
        all.truncate(MAX_RESULTS);
        all
    }
}

/// `type NOT IN (...)`, from [`NOT_DESTINATIONS`] (fixed words, never what was typed).
fn not_junk() -> String {
    let list = NOT_DESTINATIONS
        .iter()
        .map(|t| format!("'{t}'"))
        .collect::<Vec<_>>()
        .join(", ");
    format!("(type IS NULL OR type NOT IN ({list}))")
}

/// The spellings tried against the case-sensitive index: as typed, lower, Title Case,
/// UPPER, and the first letter up. Each once.
fn spellings(query: &str) -> Vec<String> {
    let title_word = |w: &str| {
        let mut c = w.chars();
        c.next().map_or_else(String::new, |f| {
            f.to_uppercase()
                .chain(c.flat_map(char::to_lowercase))
                .collect()
        })
    };
    let title = query
        .split(' ')
        .map(title_word)
        .collect::<Vec<_>>()
        .join(" ");
    let mut rest = query.chars();
    let first_up: String = rest
        .next()
        .map(|f| f.to_uppercase().chain(rest).collect())
        .unwrap_or_default();
    let mut out: Vec<String> = Vec::new();
    for s in [
        query.to_owned(),
        query.to_lowercase(),
        title,
        query.to_uppercase(),
        first_up,
    ] {
        if !out.contains(&s) {
            out.push(s);
        }
    }
    out
}

/// "580 rideau" -> ("580", "rideau"); "12b main st" -> ("12b", "main st").
fn address_query(query: &str) -> Option<(&str, &str)> {
    let (number, street) = query.split_once(' ')?;
    let starts_with_digit = number.chars().next().is_some_and(|c| c.is_ascii_digit());
    (starts_with_digit && !street.is_empty()).then_some((number, street))
}

/// What was typed, safe inside a LIKE pattern with `ESCAPE '\'`.
fn escape_like(s: &str) -> String {
    s.chars()
        .flat_map(|c| match c {
            '%' | '_' | '\\' => vec!['\\', c],
            _ => vec![c],
        })
        .collect()
}

/// The distance along the Earth's surface (a sphere of the mean radius: within 0.5 %,
/// plenty to rank places across a city).
pub fn great_circle(a: LatLon, b: LatLon) -> Metres {
    const R: f64 = 6_371_008.8;
    let (p1, p2) = (a.lat.to_radians(), b.lat.to_radians());
    let dp = p2 - p1;
    let dl = (b.lon - a.lon).to_radians();
    let h = (dp / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    Metres(2.0 * R * h.sqrt().min(1.0).asin())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering as AtomicOrdering};

    /// A tiny place file, written to the temp folder, with the real table and indexes.
    struct TinyDb(PathBuf);

    impl TinyDb {
        fn new(rows: &[(&str, &str, &str, &str, f64, f64)]) -> Self {
            static N: AtomicU32 = AtomicU32::new(0);
            let path = std::env::temp_dir().join(format!(
                "aav-places-{}-{}.db",
                std::process::id(),
                N.fetch_add(1, AtomicOrdering::SeqCst)
            ));
            let _ = std::fs::remove_file(&path);
            let conn = Connection::open(&path).expect("a temp file");
            conn.execute_batch(
                "CREATE TABLE places (id INTEGER PRIMARY KEY AUTOINCREMENT, osm_id INTEGER,
                   name TEXT, type TEXT, house_number TEXT, street TEXT, city TEXT,
                   lat REAL, lon REAL);
                 CREATE INDEX idx_name ON places(name);
                 CREATE INDEX idx_street ON places(street);",
            )
            .expect("the schema");
            for &(name, kind, number, street, lat, lon) in rows {
                let none = |s: &str| (!s.is_empty()).then(|| s.to_owned());
                conn.execute(
                    "INSERT INTO places (name, type, house_number, street, lat, lon)
                     VALUES (?1, ?2, ?3, ?4, ?5, ?6)",
                    (none(name), kind, none(number), none(street), lat, lon),
                )
                .expect("a row");
            }
            Self(path)
        }

        fn open(&self) -> Places {
            Places::open(&self.0).expect("opens")
        }
    }

    impl Drop for TinyDb {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }

    /// Near Carleton.
    const HERE: LatLon = LatLon::new(45.3860, -75.6960);

    fn sample() -> TinyDb {
        TinyDb::new(&[
            (
                "Carleton University",
                "university",
                "1125",
                "Colonel By Drive",
                45.3849,
                -75.6961,
            ),
            ("Carleton", "place", "", "", 45.3856, -75.6959),
            (
                "Carleton Grill",
                "restaurant",
                "150",
                "Albert Street",
                45.4209,
                -75.6976,
            ),
            (
                "Carleton Heights",
                "neighbourhood",
                "",
                "",
                45.3572,
                -75.7110,
            ),
            ("Old Carleton Mill", "place", "", "", 45.3000, -75.9000),
            (
                "Carleton street lamp",
                "street_lamp",
                "",
                "",
                45.3860,
                -75.6960,
            ),
            ("Bank Street", "secondary", "", "", 45.4100, -75.6900),
            ("Bank Street", "secondary", "", "", 45.3900, -75.6800),
            ("", "place", "580", "Rideau Street", 45.4326, -75.6763),
            ("", "place", "582", "Rideau Street", 45.4327, -75.6762),
            ("McDonald's", "fast_food", "", "", 45.4000, -75.7000),
            ("50% Off", "clothes", "", "", 45.4000, -75.7000),
        ])
    }

    fn titles(found: &[Place]) -> Vec<&str> {
        found.iter().map(|p| p.title.as_str()).collect()
    }

    #[test]
    fn exact_then_prefix_then_a_word_then_inside_nearest_first() {
        let db = sample();
        let found = db.open().search("carleton", Some(HERE)).expect("searches");
        assert_eq!(
            titles(&found),
            [
                "Carleton",
                "Carleton University",
                "Carleton Heights",
                "Carleton Grill",
                "Old Carleton Mill",
            ]
        );
        assert_eq!(found[0].matched, Match::Exact);
        assert_eq!(found[1].matched, Match::Prefix);
        assert_eq!(found[4].matched, Match::WordStart);
        // The distance is from HERE: Carleton U is ~120 m south.
        let d = found[1].distance.expect("a distance").0;
        assert!((100.0..140.0).contains(&d), "{d}");
        assert_eq!(found[1].detail(), "university · 1125 Colonel By Drive");
    }

    #[test]
    fn any_case_and_extra_spaces() {
        let db = sample();
        let places = db.open();
        for q in [
            "CARLETON UNIV",
            "carleton   univ",
            "Carleton Univ",
            "cArLeToN uNiV",
        ] {
            let found = places.search(q, None).expect("searches");
            assert_eq!(titles(&found), ["Carleton University"], "{q}");
        }
        // Not ASCII-only lower or Title Case: found by the scan.
        let found = places.search("mcdonald", None).expect("searches");
        assert_eq!(titles(&found), ["McDonald's"]);
    }

    #[test]
    fn junk_types_are_left_out() {
        let db = sample();
        let found = db.open().search("carleton street", None).expect("searches");
        assert!(found.is_empty(), "{:?}", titles(&found));
    }

    #[test]
    fn a_street_in_pieces_is_one_result_the_nearest_piece() {
        let db = sample();
        let near = LatLon::new(45.3900, -75.6800);
        let found = db.open().search("bank st", Some(near)).expect("searches");
        assert_eq!(titles(&found), ["Bank Street"]);
        assert_eq!(found[0].at, near);
    }

    #[test]
    fn an_address_by_its_number_and_street() {
        let db = sample();
        let found = db.open().search("580 rideau", None).expect("searches");
        assert_eq!(titles(&found), ["580 Rideau Street"]);
        assert_eq!(found[0].kind, "address");
        assert_eq!(found[0].detail(), "address");
        // The street alone finds no address with no name: a number is needed.
        assert!(db.open().search("rideau", None).expect("ok").is_empty());
    }

    #[test]
    fn short_queries_and_like_wildcards_find_nothing_odd() {
        let db = sample();
        let places = db.open();
        assert!(places.search("c", None).expect("ok").is_empty());
        assert!(places.search("   ", None).expect("ok").is_empty());
        // % and _ are letters here, not wildcards.
        assert_eq!(
            titles(&places.search("50%", None).expect("ok")),
            ["50% Off"]
        );
        assert!(places.search("c_r", None).expect("ok").is_empty());
    }

    #[test]
    fn at_most_eight() {
        let rows: Vec<(String, f64)> = (0..20)
            .map(|i| (format!("Shop {i:02}"), 45.0 + f64::from(i) * 0.001))
            .collect();
        let rows: Vec<_> = rows
            .iter()
            .map(|(n, lat)| (n.as_str(), "shop", "", "", *lat, -75.0))
            .collect();
        let db = TinyDb::new(&rows);
        let found = db
            .open()
            .search("shop", Some(LatLon::new(45.0, -75.0)))
            .expect("searches");
        assert_eq!(found.len(), MAX_RESULTS);
        assert_eq!(found[0].title, "Shop 00");
    }

    #[test]
    fn the_file_is_read_only_and_must_be_a_place_file() {
        let db = sample();
        let places = db.open();
        assert!(places.conn.execute("DELETE FROM places", ()).is_err());
        assert!(matches!(
            Places::open(Path::new("/no/such/places.db")),
            Err(PlacesError::Missing(_))
        ));
        let other = TinyDb::new(&[]);
        Connection::open(&other.0)
            .and_then(|c| c.execute_batch("DROP TABLE places; CREATE TABLE x (y);"))
            .expect("a file with another table");
        assert!(matches!(
            Places::open(&other.0),
            Err(PlacesError::NotPlaces { .. })
        ));
    }

    #[test]
    fn matches_rank_in_order() {
        assert_eq!(Match::of("carleton", "carleton"), Some(Match::Exact));
        assert_eq!(Match::of("carleton u", "carl"), Some(Match::Prefix));
        assert_eq!(Match::of("old carleton", "carl"), Some(Match::WordStart));
        assert_eq!(Match::of("st-laurent", "laur"), Some(Match::WordStart));
        assert_eq!(Match::of("mcdonald's", "donald"), Some(Match::Contains));
        assert_eq!(Match::of("bank", "carl"), None);
        assert!(Match::Exact < Match::Prefix && Match::WordStart < Match::Contains);
    }

    #[test]
    fn spellings_cover_how_people_type() {
        assert_eq!(
            spellings("bank st"),
            ["bank st", "Bank St", "BANK ST", "Bank st"]
        );
        assert_eq!(address_query("580 rideau"), Some(("580", "rideau")));
        assert_eq!(address_query("rideau 580"), None);
        assert_eq!(escape_like(r"a%b_c\"), r"a\%b\_c\\");
    }

    #[test]
    fn the_deny_list_is_plain_words() {
        // They go into SQL as text: only lower-case letters and _.
        assert!(NOT_DESTINATIONS
            .iter()
            .all(|t| t.chars().all(|c| c.is_ascii_lowercase() || c == '_')));
    }

    #[test]
    fn great_circle_distances() {
        // Carleton to Parliament Hill: about 4.0 km.
        let d = great_circle(HERE, LatLon::new(45.4236, -75.7009)).0;
        assert!((4150.0..4250.0).contains(&d), "{d}");
        assert_eq!(great_circle(HERE, HERE).0, 0.0);
    }

    /// The real file, when it's there (pixi run get places): fast enough to search as
    /// someone types. The limit is for a release build (the window's); a debug build's
    /// SQLite is unoptimised, and tests run side by side, so it gets more room.
    #[test]
    fn the_real_file_is_quick() {
        let limit_ms = if cfg!(debug_assertions) { 1000 } else { 50 };
        let path =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../../maps/places/ottawa.db");
        let Ok(places) = Places::open(&path) else {
            eprintln!("skipped: no {}", path.display());
            return;
        };
        for q in ["carleton", "580 rideau", "bank street", "xqzv", "ca"] {
            let start = std::time::Instant::now();
            let found = places.search(q, Some(HERE)).expect("searches");
            let took = start.elapsed();
            eprintln!("{q:?}: {} results in {took:?}", found.len());
            assert!(took.as_millis() < limit_ms, "{q:?} took {took:?}");
        }
        let found = places
            .search("carleton university", Some(HERE))
            .expect("ok");
        assert_eq!(found[0].title, "Carleton University");
    }
}
