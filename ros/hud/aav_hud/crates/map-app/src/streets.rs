//! The street map under Nav2's map: OpenStreetMap's water, parks, buildings and roads,
//! from the offline Protomaps extract (`pixi run get street-map`).
//!
//! A thread reads the file and decodes tiles (map-core's `streets`), the ones the view
//! wants, the middle first; the window draws what's ready, a frame at a time, so the
//! map never waits for a tile. Drawing is raylib's rlgl: one transform from map metres
//! to the screen, and every triangle of one colour in one batch.

use std::{
    collections::VecDeque,
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver, Sender},
        Arc, Condvar, Mutex,
    },
    thread,
};

use hud_app::paint::ToColor;
use hud_core::theme::StreetColours;
use map_core::{
    earth::Datum,
    pmtiles::Archive,
    streets::{self, Area, LineKind, RoadClass, TileShapes},
    tilecache::TileCache,
    tiles::{self, TileId},
    view::{View, Viewport},
};
use raylib::prelude::*;

/// What the street map is doing.
#[derive(Debug, Clone, PartialEq)]
pub enum Status {
    /// No file: the window says how to get it.
    Missing(PathBuf),
    Loading,
    Ready,
    /// The file is there but can't be read.
    Failed(String),
}

/// What the loader thread sends.
enum News {
    Opened,
    Failed(String),
    Tile(TileId, Option<TileShapes>),
}

/// The loader's queue: the tiles wanted, a bell to wake it, and whether the window
/// has closed.
#[derive(Default)]
struct Queue {
    ids: Mutex<VecDeque<TileId>>,
    bell: Condvar,
    closed: AtomicBool,
}

/// The street map: its loader, its tiles, and whether it's shown.
pub struct StreetMap {
    pub datum: Datum,
    pub status: Status,
    /// S turns it off and on.
    pub shown: bool,
    cache: TileCache,
    queue: Arc<Queue>,
    news: Option<Receiver<News>>,
}

impl StreetMap {
    /// Starts reading `file` on its own thread. A missing file is no error: the window
    /// shows a hint and runs without streets.
    pub fn open(file: &Path, datum: Datum) -> Self {
        let mut map = Self {
            datum,
            status: Status::Loading,
            shown: true,
            cache: TileCache::default(),
            queue: Arc::new(Queue::default()),
            news: None,
        };
        if !file.is_file() {
            map.status = Status::Missing(file.to_path_buf());
            return map;
        }
        let (tx, rx) = mpsc::channel();
        let queue = Arc::clone(&map.queue);
        let file = file.to_path_buf();
        let spawned = thread::Builder::new()
            .name("street-tiles".into())
            .spawn(move || load(&file, datum, &queue, &tx));
        match spawned {
            Ok(_) => map.news = Some(rx),
            Err(e) => map.status = Status::Failed(format!("no loader thread: {e}")),
        }
        map
    }

    /// Whether streets are drawn this frame.
    pub fn visible(&self) -> bool {
        self.shown && matches!(self.status, Status::Ready | Status::Loading)
    }

    /// Each frame: takes in decoded tiles, asks for the ones this view needs, and gives
    /// back the tiles to draw.
    pub fn frame(&mut self, view: &View, vp: &Viewport) -> Vec<Arc<TileShapes>> {
        if let Some(rx) = &self.news {
            for news in rx.try_iter() {
                match news {
                    News::Opened => self.status = Status::Ready,
                    News::Failed(why) => self.status = Status::Failed(why),
                    News::Tile(id, shapes) => self.cache.insert(id, shapes),
                }
            }
        }
        if !self.visible() {
            return Vec::new();
        }
        let z = tiles::zoom_for(view.scale, self.datum.origin.lat);
        let wanted = tiles::covering(&self.datum, view.visible(vp), z);
        let Ok(mut queue) = self.queue.ids.lock() else {
            return Vec::new();
        };
        let draw = self.cache.frame(&wanted, &mut queue);
        if !queue.is_empty() {
            self.queue.bell.notify_one();
        }
        draw
    }
}

impl Drop for StreetMap {
    /// Stops the loader.
    fn drop(&mut self) {
        // Under the queue's lock, so the loader can't miss the bell.
        let _ids = self.queue.ids.lock();
        self.queue.closed.store(true, Ordering::Relaxed);
        self.queue.bell.notify_all();
    }
}

/// The loader thread: opens the file, then decodes tiles from the queue until the
/// window closes.
fn load(file: &Path, datum: Datum, queue: &Queue, tx: &Sender<News>) {
    let archive = match std::fs::read(file)
        .map_err(|e| e.to_string())
        .and_then(|bytes| Archive::new(bytes).map_err(|e| e.to_string()))
    {
        Ok(a) => a,
        Err(e) => {
            let _ = tx.send(News::Failed(format!("{}: {e}", file.display())));
            return;
        }
    };
    if tx.send(News::Opened).is_err() {
        return;
    }
    loop {
        let id = {
            let Ok(mut ids) = queue.ids.lock() else {
                return;
            };
            loop {
                if queue.closed.load(Ordering::Relaxed) {
                    return;
                }
                if let Some(id) = ids.pop_front() {
                    break id;
                }
                match queue.bell.wait(ids) {
                    Ok(guard) => ids = guard,
                    Err(_) => return,
                }
            }
        };
        // Not in the file (outside the extract) or broken: drawn as nothing.
        let shapes = archive
            .tile(id)
            .ok()
            .flatten()
            .and_then(|mvt| streets::decode(id, mvt, &datum).ok());
        if tx.send(News::Tile(id, shapes)).is_err() {
            return;
        }
    }
}

// The street map's colours come from the theme (hud_core::theme): quiet, so Nav2's map,
// the plan and the car stay the brightest things on the screen.

fn area_colour(s: &StreetColours, a: Area) -> Color {
    match a {
        Area::Green => s.green,
        Area::Campus => s.campus,
        Area::Water => s.water,
        Area::Building => s.building,
    }
    .rl()
}

fn road_colour(s: &StreetColours, c: RoadClass) -> Color {
    match c {
        RoadClass::Path => s.path,
        RoadClass::Rail => s.rail,
        RoadClass::Service => s.service,
        RoadClass::Minor => s.minor,
        RoadClass::Medium => s.medium,
        RoadClass::Major => s.major,
        RoadClass::Highway => s.highway,
    }
    .rl()
}

/// A road's width on the screen: its real width, but never thinner than this.
fn min_px(c: RoadClass) -> f32 {
    match c {
        RoadClass::Path | RoadClass::Rail | RoadClass::Service => 1.0,
        RoadClass::Minor => 1.4,
        RoadClass::Medium => 1.8,
        RoadClass::Major => 2.4,
        RoadClass::Highway => 3.0,
    }
}

/// Names are written only this close in (px per metre): about 1.5 km across the window.
const LABELS_FROM: f64 = 0.8;

/// Draws the tiles: the areas, then the roads (casings under the major ones), then the
/// names, in the theme's `colours`. `u` is the window's scale (1 at 1600 x 900).
#[allow(clippy::too_many_arguments)]
pub fn draw<D: RaylibDraw>(
    d: &mut D,
    colours: &StreetColours,
    font: &impl AsRef<raylib::ffi::Font>,
    measure: impl Fn(&str, f32) -> f32,
    tiles: &[Arc<TileShapes>],
    view: &View,
    vp: &Viewport,
    u: f32,
) {
    d.draw_rectangle(
        vp.x as i32,
        vp.y as i32,
        vp.width as i32,
        vp.height as i32,
        colours.land.rl(),
    );
    if tiles.is_empty() {
        return;
    }
    let k = view.scale.0 as f32;
    let c = vp.centre();
    // Map metres to pixels: x right, y down (north up). Triangles from the two flips
    // wind both ways, so culling is off while the streets draw.
    d.rl_disable_backface_culling();
    {
        let mut m = d.rl_push_matrix();
        m.rl_translatef(c.x as f32, c.y as f32, 0.0);
        m.rl_scalef(k, -k, 1.0);
        m.rl_translatef(-view.centre.x as f32, -view.centre.y as f32, 0.0);
        for area in Area::ALL {
            m.rl_draw(DrawMode::Triangles, |v| {
                v.color4ub(area_colour(colours, area));
                for t in tiles {
                    for p in t.triangles(area) {
                        v.vertex2f(p[0], p[1]);
                    }
                }
            });
        }
        // Waterways, then each road class from the least: one batch per colour.
        let width = |px: f32| px / k;
        lines(
            &mut *m,
            tiles,
            LineKind::Waterway,
            width(1.5),
            colours.waterway.rl(),
        );
        for class in [RoadClass::Major, RoadClass::Highway] {
            let w = (class.width_m() as f32 * k).max(min_px(class)) + 2.0;
            lines(
                &mut *m,
                tiles,
                LineKind::Road(class),
                width(w),
                colours.casing.rl(),
            );
        }
        for class in RoadClass::ALL {
            let w = (class.width_m() as f32 * k).max(min_px(class))
                * if class == RoadClass::Path { 0.5 } else { 1.0 };
            lines(
                &mut *m,
                tiles,
                LineKind::Road(class),
                width(w.min(80.0)),
                road_colour(colours, class),
            );
        }
    }
    d.rl_enable_backface_culling();
    if view.scale.0 >= LABELS_FROM {
        labels(d, colours, font, measure, tiles, view, vp, u);
    }
}

/// Every line of one kind as quads, `w` wide (map metres), each segment stretched by
/// half its width at both ends, so bends have no gaps.
fn lines(
    d: &mut impl RaylibDraw,
    tiles: &[Arc<TileShapes>],
    kind: LineKind,
    w: f32,
    colour: Color,
) {
    let h = w / 2.0;
    d.rl_draw(DrawMode::Triangles, |v| {
        v.color4ub(colour);
        let segments = tiles
            .iter()
            .flat_map(|t| &t.lines)
            .filter(|l| l.kind == kind)
            .flat_map(|l| l.points.windows(2));
        for s in segments {
            let (a, b) = (s[0], s[1]);
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let len = dx.hypot(dy);
            if len <= f32::EPSILON {
                continue;
            }
            let (ux, uy) = (dx / len * h, dy / len * h);
            // Along (ux, uy) and across (-uy, ux), each half the width.
            let p = [
                [a[0] - ux - uy, a[1] - uy + ux],
                [a[0] - ux + uy, a[1] - uy - ux],
                [b[0] + ux + uy, b[1] + uy - ux],
                [b[0] + ux - uy, b[1] + uy + ux],
            ];
            for i in [0, 1, 2, 0, 2, 3] {
                v.vertex2f(p[i][0], p[i][1]);
            }
        }
    });
}

/// Street names, along their streets, where they fit and don't crowd each other.
#[allow(clippy::too_many_arguments)]
fn labels(
    d: &mut impl RaylibDraw,
    colours: &StreetColours,
    font: &impl AsRef<raylib::ffi::Font>,
    measure: impl Fn(&str, f32) -> f32,
    tiles: &[Arc<TileShapes>],
    view: &View,
    vp: &Viewport,
    u: f32,
) {
    let size = 15.0 * u;
    let spacing = size / 20.0;
    let mut placed: Vec<(Vector2, f32, &str)> = Vec::new();
    let mut all: Vec<&streets::Label> = tiles.iter().flat_map(|t| &t.labels).collect();
    // The most important streets get their names first.
    all.sort_by(|a, b| {
        b.class
            .cmp(&a.class)
            .then(b.length_m.total_cmp(&a.length_m))
    });
    for l in all {
        let at = view.to_screen(
            map_core::geom::Point::new(f64::from(l.at[0]), f64::from(l.at[1])),
            vp,
        );
        let at = Vector2::new(at.x as f32, at.y as f32);
        let inset = 20.0 * u;
        if !(vp.x as f32 + inset..(vp.x + vp.width) as f32 - inset).contains(&at.x)
            || !(vp.y as f32 + inset..(vp.y + vp.height) as f32 - inset).contains(&at.y)
        {
            continue;
        }
        let text_w = measure(&l.text, size);
        if text_w > l.length_m * view.scale.0 as f32 * 0.9 {
            continue;
        }
        // Not on top of another name, and the same name not again close by.
        let r = text_w / 2.0;
        let crowded = placed.iter().any(|&(p, pr, name)| {
            let gap = (p - at).length();
            gap < (pr + r) * 0.8 || (name == l.text && gap < 300.0 * u)
        });
        if crowded {
            continue;
        }
        let origin = Vector2::new(text_w / 2.0, size / 2.0);
        let rotation = -l.angle.to_degrees();
        for (dx, dy) in [(-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)] {
            let halo = at + Vector2::new(dx, dy) * (1.5 * u);
            d.draw_text_pro(
                font,
                &l.text,
                halo,
                origin,
                rotation,
                size,
                spacing,
                colours.label_halo.rl(),
            );
        }
        d.draw_text_pro(
            font,
            &l.text,
            at,
            origin,
            rotation,
            size,
            spacing,
            colours.label.rl(),
        );
        placed.push((at, r, &l.text));
    }
}
