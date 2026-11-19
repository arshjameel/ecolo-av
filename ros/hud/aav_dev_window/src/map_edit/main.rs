//! The map editor (TODO 251, `docs/map-editor.md`): a map from `pixi run map-drive` over
//! the street map, to check, line up, clean and label, then save as a new folder. Bevy and
//! egui, like the developer window; no ROS. The edits are `map-edit`'s
//! (`ros/bringup/aav_description/map-edit`): this file only draws them and passes them on.
//!
//! ```text
//! map_edit <map folder> [--streets FILE] [--select X0,Y0,X1,Y1] [--seconds S] [--screenshot FILE]
//! ```
//!
//! `--streets`: OpenStreetMap's tiles (default `maps/streets/ottawa.pmtiles`). `--select`
//! starts with a box drawn (map metres). `--seconds` exits after S seconds; `--screenshot`
//! saves one picture halfway through: the window seen without a person.

mod streets;
mod tool;

use std::path::PathBuf;

use bevy::{
    app::AppExit,
    prelude::*,
    render::view::screenshot::{save_to_disk, Screenshot},
};
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use map_core::{
    geom::Point as ViewPoint,
    grid::Bounds,
    streets::{Area, LineKind, RoadClass, TileShapes},
    view::{Mode, PixelsPerMetre, ScreenPoint, View, Viewport},
};
use map_edit::{
    chunk_outlines, grid_rgba, next_free, Align, CellFill, Edit, Editor, HeightBand, HeightImage,
    LabelClass, MapFolder, Point, Region, CHUNK_SIZE,
};

use crate::tool::{Nudge, Tool};

const USAGE: &str = "usage: map_edit <map folder> [--streets FILE] [--select X0,Y0,X1,Y1] \
                     [--seconds S] [--screenshot FILE]";
/// NDT's points from above: one cell (m), as map-drive thins them.
const POINT_CELL: f64 = 0.1;
/// The wheel's zoom: this power of e per point of scroll.
const WHEEL_ZOOM: f64 = 0.002;
/// How long the window settles before `--screenshot`, seconds.
const SETTLE: f32 = 1.0;

/// What this run was asked for.
#[derive(Debug, Clone)]
struct Options {
    map: PathBuf,
    streets: PathBuf,
    select: Option<[f64; 4]>,
    seconds: Option<f32>,
    screenshot: Option<PathBuf>,
}

impl Options {
    fn parse(mut args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut options = Self {
            map: PathBuf::new(),
            streets: PathBuf::from("maps/streets/ottawa.pmtiles"),
            select: None,
            seconds: None,
            screenshot: None,
        };
        let mut map = None;
        while let Some(arg) = args.next() {
            let mut value = || args.next().ok_or_else(|| format!("{arg} needs a value"));
            match arg.as_str() {
                "--streets" => options.streets = value()?.into(),
                "--select" => {
                    let text = value()?;
                    let v: Vec<f64> = text
                        .split(',')
                        .map(|s| s.trim().parse())
                        .collect::<Result<_, _>>()
                        .map_err(|_| format!("--select {text}: X0,Y0,X1,Y1 in metres"))?;
                    let [a, b, c, d] = v[..] else {
                        return Err(format!("--select {text}: four numbers"));
                    };
                    options.select = Some([a, b, c, d]);
                }
                "--seconds" => {
                    options.seconds =
                        Some(value()?.parse().map_err(|e| format!("--seconds: {e}"))?);
                }
                "--screenshot" => options.screenshot = Some(value()?.into()),
                other if other.starts_with("--") => return Err(format!("unknown option {other}")),
                other => map = Some(PathBuf::from(other)),
            }
        }
        options.map = map.ok_or("which map folder? (a map-drive folder: bags/<drive>/map)")?;
        Ok(options)
    }
}

/// The layers, each on or off.
#[derive(Debug, Clone, Copy)]
struct Layers {
    streets: bool,
    points: bool,
    grid: bool,
    chunks: bool,
    labels: bool,
    path: bool,
}

/// A picture of one layer, and the editor's version it was made from.
struct Picture {
    texture: egui::TextureHandle,
    /// Lower left, upper right (m, the source map's frame).
    bounds: (Point, Point),
    version: u64,
}

/// The whole window's state.
#[derive(Resource)]
struct EditorUi {
    editor: Editor,
    /// Goes up with every change to the editor: pictures older than it are made again.
    version: u64,
    streets: Vec<TileShapes>,
    view: View,
    fitted: bool,
    tool: Tool,
    layers: Layers,
    /// The alignment shown: the editor's, or one being dragged or typed.
    draft: Align,
    selection: Option<Region>,
    drag_from: Option<Point>,
    box_yaw_deg: f64,
    use_band: bool,
    band: HeightBand,
    cells: CellFill,
    label_class: LabelClass,
    out_dir: String,
    confirming: bool,
    messages: Vec<String>,
    points_picture: Option<Picture>,
    grid_picture: Option<Picture>,
    styled: bool,
}

impl EditorUi {
    fn note(&mut self, line: impl Into<String>) {
        self.messages.push(line.into());
        if self.messages.len() > 8 {
            self.messages.remove(0);
        }
    }

    /// An edit, through the editor: a typed or dragged alignment first, so a box drawn on
    /// the screen goes back into the map's own frame with the move the person sees.
    fn apply(&mut self, edit: Edit) {
        if !matches!(edit, Edit::Align { .. }) {
            self.commit_draft();
        }
        let words = describe(&edit);
        match self.editor.apply(edit) {
            Ok(effect) => {
                self.version += 1;
                self.draft = self.editor.align();
                let mut line = words;
                if effect.points_removed > 0 || effect.cells_changed > 0 {
                    line += &format!(
                        ": {} points, {} cells",
                        effect.points_removed, effect.cells_changed
                    );
                }
                self.note(line);
            }
            Err(why) => self.note(format!("refused: {why}")),
        }
    }

    fn commit_draft(&mut self) {
        if self.draft != self.editor.align() {
            self.apply(Edit::Align { to: self.draft });
        }
    }

    fn undo(&mut self) {
        match self.editor.undo() {
            Some(edit) => {
                self.version += 1;
                self.draft = self.editor.align();
                self.note(format!("undone: {}", describe(&edit)));
            }
            None => self.note("nothing to undo"),
        }
    }

    fn save(&mut self) {
        self.commit_draft();
        let out = PathBuf::from(self.out_dir.trim());
        match self.editor.save(&out) {
            Ok(saved) => {
                self.note(format!(
                    "saved {}: {} points ({} removed), {} + {} + {} chunks",
                    saved.dir.display(),
                    saved.points,
                    saved.points_removed,
                    saved.chunks[0],
                    saved.chunks[1],
                    saved.chunks[2]
                ));
                self.out_dir = next_free(&self.editor.source().dir).display().to_string();
            }
            Err(why) => self.note(format!("not saved: {why}")),
        }
    }
}

fn describe(edit: &Edit) -> String {
    match edit {
        Edit::Align { to } => format!(
            "align x {:.2} m, y {:.2} m, yaw {:.2}°",
            to.x_m,
            to.y_m,
            to.yaw_rad.to_degrees()
        ),
        Edit::Delete { region, cells, .. } => format!(
            "delete {:.1} x {:.1} m, {}",
            region.size_m[0],
            region.size_m[1],
            cells.name()
        ),
        Edit::Label { class, .. } => format!("label: {}", class.name()),
        Edit::Relabel { id, class } => format!("label {id} is a {}", class.name()),
        Edit::Unlabel { id } => format!("label {id} removed"),
    }
}

#[derive(Resource)]
struct Run {
    seconds: Option<f32>,
    screenshot: Option<PathBuf>,
    shot: bool,
}

fn main() -> AppExit {
    let options = match Options::parse(std::env::args().skip(1)) {
        Ok(options) => options,
        Err(message) => {
            eprintln!("map_edit: {message}\n{USAGE}");
            return AppExit::from_code(2);
        }
    };
    let map = match MapFolder::load(&options.map) {
        Ok(map) => map,
        Err(why) => {
            eprintln!("map_edit: {why}");
            return AppExit::from_code(1);
        }
    };
    let mut messages = vec![format!(
        "{}: {} points, 2D map {} x {} cells of {} m, placed by {}",
        options.map.display(),
        map.points.len(),
        map.grid.width,
        map.grid.height,
        map.grid.resolution,
        map.placed_by().unwrap_or_else(|| "nothing (local)".into())
    )];
    let (lo, hi) = map.grid.bounds();
    let streets = match map.datum() {
        Some(datum) => match streets::load(&options.streets, datum, bounds(lo, hi)) {
            Ok(tiles) => {
                messages.push(format!("street map: {} tiles", tiles.len()));
                tiles
            }
            Err(why) => {
                messages.push(format!("no street map: {why}"));
                Vec::new()
            }
        },
        None => {
            messages.push("no street map: the map has no datum (placed: local)".into());
            Vec::new()
        }
    };
    for line in &messages {
        println!("map_edit: {line}");
    }
    let out_dir = next_free(&map.dir).display().to_string();
    let selection = options
        .select
        .and_then(|[a, b, c, d]| tool::drag_box(Point::new(a, b), Point::new(c, d), 0.0));
    let ui = EditorUi {
        editor: Editor::new(map),
        version: 0,
        streets,
        view: View::default(),
        fitted: false,
        tool: if selection.is_some() {
            Tool::Box
        } else {
            Tool::Pan
        },
        layers: Layers {
            streets: true,
            points: true,
            grid: true,
            chunks: true,
            labels: true,
            path: true,
        },
        draft: Align::IDENTITY,
        selection,
        drag_from: None,
        box_yaw_deg: 0.0,
        use_band: false,
        band: HeightBand::new(0.0, 2.5),
        cells: CellFill::Unknown,
        label_class: LabelClass::ParkedCar,
        out_dir,
        confirming: false,
        messages,
        points_picture: None,
        grid_picture: None,
        styled: false,
    };

    App::new()
        .insert_resource(ClearColor(Color::srgb(0.06, 0.06, 0.07)))
        .insert_resource(ui)
        .insert_resource(Run {
            seconds: options.seconds,
            screenshot: options.screenshot,
            shot: false,
        })
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "AAV map editor".into(),
                resolution: (1500, 950).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_systems(Startup, |mut commands: Commands| {
            commands.spawn(Camera2d);
        })
        .add_systems(EguiPrimaryContextPass, draw)
        .add_systems(Update, timer)
        .run()
}

/// `--seconds` and `--screenshot`.
fn timer(
    mut commands: Commands,
    time: Res<Time<Real>>,
    mut run: ResMut<Run>,
    mut exit: MessageWriter<AppExit>,
) {
    let elapsed = time.elapsed_secs();
    let shot_at = SETTLE + run.seconds.unwrap_or(0.0) / 2.0;
    if !run.shot && elapsed > shot_at {
        if let Some(path) = run.screenshot.clone() {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path));
            run.shot = true;
        }
    }
    if run.seconds.is_some_and(|s| elapsed > SETTLE + s) {
        exit.write(AppExit::Success);
    }
}

fn bounds(lo: Point, hi: Point) -> Bounds {
    Bounds {
        min: ViewPoint::new(lo.x, lo.y),
        max: ViewPoint::new(hi.x, hi.y),
    }
}

fn draw(mut contexts: EguiContexts, mut state: ResMut<EditorUi>) -> Result {
    let ctx = contexts.ctx_mut()?.clone();
    let state = &mut *state;
    if !state.styled {
        ctx.set_visuals(egui::Visuals::dark());
        state.styled = true;
    }
    keys(&ctx, state);
    let mut root = egui::Ui::new(
        ctx.clone(),
        "map_edit".into(),
        egui::UiBuilder::new()
            .layer_id(egui::LayerId::background())
            .max_rect(ctx.viewport_rect()),
    );
    egui::Panel::left("tools")
        .default_size(330.0)
        .resizable(true)
        .show(&mut root, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| side_panel(ui, state));
        });
    egui::CentralPanel::default()
        .frame(egui::Frame::NONE)
        .show(&mut root, |ui| map_panel(ui, state));
    if state.confirming {
        confirm_window(&ctx, state);
    }
    Ok(())
}

fn keys(ctx: &egui::Context, state: &mut EditorUi) {
    if ctx.egui_wants_keyboard_input() {
        return;
    }
    let (undo, pan, boxes, align, delete, escape, shift) = ctx.input(|i| {
        (
            i.modifiers.command && i.key_pressed(egui::Key::Z),
            i.key_pressed(egui::Key::P),
            i.key_pressed(egui::Key::B),
            i.key_pressed(egui::Key::A),
            i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace),
            i.key_pressed(egui::Key::Escape),
            i.modifiers.shift,
        )
    });
    if undo {
        state.undo();
        return;
    }
    for (pressed, tool) in [(pan, Tool::Pan), (boxes, Tool::Box), (align, Tool::Align)] {
        if pressed {
            state.tool = tool;
        }
    }
    if escape {
        state.selection = None;
    }
    if delete {
        delete_selection(state);
    }
    if state.tool == Tool::Align {
        let pressed: Vec<Nudge> = ctx.input(|i| {
            [
                (egui::Key::ArrowLeft, Nudge::Left),
                (egui::Key::ArrowRight, Nudge::Right),
                (egui::Key::ArrowUp, Nudge::Up),
                (egui::Key::ArrowDown, Nudge::Down),
                (egui::Key::Q, Nudge::TurnLeft),
                (egui::Key::E, Nudge::TurnRight),
            ]
            .into_iter()
            .filter(|(k, _)| i.key_pressed(*k))
            .map(|(_, n)| n)
            .collect()
        });
        for nudge in pressed {
            let pivot = Point::new(state.view.centre.x, state.view.centre.y);
            let to = tool::nudged(state.draft, nudge, shift, pivot);
            state.apply(Edit::Align { to });
        }
    }
}

fn delete_selection(state: &mut EditorUi) {
    if let Some(region) = state.selection.take() {
        state.commit_draft();
        let band = state.use_band.then_some(state.band);
        let region = region.unaligned(&state.editor.align());
        state.apply(Edit::Delete {
            region,
            band,
            cells: state.cells,
        });
    }
}

fn side_panel(ui: &mut egui::Ui, state: &mut EditorUi) {
    ui.heading("Map editor");
    for line in state.messages.iter().take(1) {
        ui.label(egui::RichText::new(line).small());
    }
    ui.separator();

    ui.strong("Layers");
    let l = &mut state.layers;
    ui.horizontal_wrapped(|ui| {
        ui.toggle_value(&mut l.streets, "Streets (OSM)");
        ui.toggle_value(&mut l.points, "NDT points");
        ui.toggle_value(&mut l.grid, "2D map");
        ui.toggle_value(&mut l.chunks, "64 m chunks");
        ui.toggle_value(&mut l.labels, "Labels");
        ui.toggle_value(&mut l.path, "Path");
    });
    ui.separator();

    ui.strong("Tool");
    ui.horizontal(|ui| {
        for tool in Tool::ALL {
            ui.selectable_value(&mut state.tool, tool, tool.name());
        }
    });
    if ui.button("Whole map").clicked() {
        state.fitted = false;
    }
    ui.separator();

    ui.strong("Align our map on the street map");
    let committed = state.editor.align();
    let mut yaw_deg = state.draft.yaw_rad.to_degrees();
    egui::Grid::new("align").num_columns(2).show(ui, |ui| {
        ui.label("x (m)");
        ui.add(
            egui::DragValue::new(&mut state.draft.x_m)
                .speed(0.01)
                .max_decimals(3),
        );
        ui.end_row();
        ui.label("y (m)");
        ui.add(
            egui::DragValue::new(&mut state.draft.y_m)
                .speed(0.01)
                .max_decimals(3),
        );
        ui.end_row();
        ui.label("yaw (°)");
        ui.add(
            egui::DragValue::new(&mut yaw_deg)
                .speed(0.01)
                .max_decimals(3),
        );
        ui.end_row();
    });
    if (yaw_deg - state.draft.yaw_rad.to_degrees()).abs() > 1e-12 {
        state.draft.yaw_rad = yaw_deg.to_radians();
    }
    ui.horizontal(|ui| {
        let changed = state.draft != committed;
        if ui
            .add_enabled(changed, egui::Button::new("Apply"))
            .clicked()
        {
            state.commit_draft();
        }
        if ui
            .add_enabled(changed, egui::Button::new("Cancel"))
            .clicked()
        {
            state.draft = committed;
        }
        if ui.button("Zero").clicked() {
            state.draft = Align::IDENTITY;
            state.commit_draft();
        }
    });
    ui.label(
        egui::RichText::new(
            "Align tool: drag; arrows 5 cm (shift 50 cm); Q / E turn 0.1° (shift 1°)",
        )
        .small(),
    );
    ui.separator();

    ui.strong("Box");
    match state.selection {
        None => {
            ui.label("Box tool (B): drag on the map");
        }
        Some(region) => {
            ui.label(format!(
                "{:.1} x {:.1} m at ({:.1}, {:.1})",
                region.size_m[0], region.size_m[1], region.centre_m[0], region.centre_m[1]
            ));
        }
    }
    ui.horizontal(|ui| {
        ui.label("turn (°)");
        if ui
            .add(
                egui::DragValue::new(&mut state.box_yaw_deg)
                    .speed(0.5)
                    .range(-90.0..=90.0),
            )
            .changed()
        {
            if let Some(r) = &mut state.selection {
                r.yaw_rad = state.box_yaw_deg.to_radians();
            }
        }
    });
    ui.horizontal(|ui| {
        ui.checkbox(&mut state.use_band, "only heights");
        ui.add_enabled(
            state.use_band,
            egui::DragValue::new(&mut state.band.min_m)
                .speed(0.05)
                .suffix(" m"),
        );
        ui.label("to");
        ui.add_enabled(
            state.use_band,
            egui::DragValue::new(&mut state.band.max_m)
                .speed(0.05)
                .suffix(" m"),
        );
    });
    egui::ComboBox::from_id_salt("cells")
        .selected_text(state.cells.name())
        .show_ui(ui, |ui| {
            for fill in CellFill::ALL {
                ui.selectable_value(&mut state.cells, fill, fill.name());
            }
        });
    let has_box = state.selection.is_some();
    ui.horizontal(|ui| {
        if ui
            .add_enabled(has_box, egui::Button::new("Delete in box (Del)"))
            .clicked()
        {
            delete_selection(state);
        }
    });
    ui.horizontal(|ui| {
        egui::ComboBox::from_id_salt("class")
            .selected_text(state.label_class.name())
            .show_ui(ui, |ui| {
                for class in LabelClass::ALL {
                    ui.selectable_value(&mut state.label_class, class, class.name());
                }
            });
        if ui
            .add_enabled(has_box, egui::Button::new("Label box"))
            .clicked()
        {
            if let Some(region) = state.selection.take() {
                state.commit_draft();
                let region = region.unaligned(&state.editor.align());
                let band = state.use_band.then_some(state.band);
                state.apply(Edit::Label {
                    class: state.label_class,
                    region,
                    band,
                });
            }
        }
    });
    ui.separator();

    ui.strong(format!("Labels ({})", state.editor.labels().len()));
    let labels: Vec<(u32, LabelClass)> = state
        .editor
        .labels()
        .iter()
        .map(|l| (l.id, l.class))
        .collect();
    for (id, class) in labels {
        ui.horizontal(|ui| {
            ui.label(format!("{id}"));
            let mut chosen = class;
            egui::ComboBox::from_id_salt(("label", id))
                .selected_text(chosen.name())
                .show_ui(ui, |ui| {
                    for c in LabelClass::ALL {
                        ui.selectable_value(&mut chosen, c, c.name());
                    }
                });
            if chosen != class {
                state.apply(Edit::Relabel { id, class: chosen });
            }
            if ui.small_button("remove").clicked() {
                state.apply(Edit::Unlabel { id });
            }
        });
    }
    ui.separator();

    ui.horizontal(|ui| {
        ui.strong(format!("Edits ({})", state.editor.edits().len()));
        if ui
            .add_enabled(!state.editor.is_unchanged(), egui::Button::new("Undo (⌘Z)"))
            .clicked()
        {
            state.undo();
        }
    });
    for line in state.messages.iter().skip(1) {
        ui.label(egui::RichText::new(line).small());
    }
    ui.separator();

    ui.strong("Approve and save");
    ui.label("A new folder: the map opened is never written over.");
    ui.text_edit_singleline(&mut state.out_dir);
    if ui
        .add_enabled(
            !state.editor.is_unchanged() || state.draft != state.editor.align(),
            egui::Button::new("Approve and save…"),
        )
        .clicked()
    {
        state.confirming = true;
    }
}

fn confirm_window(ctx: &egui::Context, state: &mut EditorUi) {
    let mut open = true;
    let mut save = false;
    egui::Window::new("Approve these edits?")
        .collapsible(false)
        .resizable(false)
        .open(&mut open)
        .show(ctx, |ui| {
            let ed = &state.editor;
            ui.label(format!("{} edits", ed.edits().len()));
            ui.label(format!(
                "NDT points: {} → {}",
                ed.source().points.len(),
                ed.source().points.len() - ed.points_removed()
            ));
            let a = state.draft;
            ui.label(format!(
                "moved by hand: x {:.3} m, y {:.3} m, yaw {:.3}°",
                a.x_m,
                a.y_m,
                a.yaw_rad.to_degrees()
            ));
            ui.label(format!("labels: {}", ed.labels().len()));
            ui.label(format!("into: {}", state.out_dir));
            ui.label("Then chunks are cut again, as map-drive does.");
            ui.horizontal(|ui| {
                save = ui.button("Approve and save").clicked();
                if ui.button("Not yet").clicked() {
                    state.confirming = false;
                }
            });
        });
    if save {
        state.save();
        state.confirming = false;
    }
    if !open {
        state.confirming = false;
    }
}

fn to_view(p: Point) -> ViewPoint {
    ViewPoint::new(p.x, p.y)
}

fn map_panel(ui: &mut egui::Ui, state: &mut EditorUi) {
    let rect = ui.available_rect_before_wrap();
    let vp = Viewport {
        x: f64::from(rect.left()),
        y: f64::from(rect.top()),
        width: f64::from(rect.width()),
        height: f64::from(rect.height()),
    };
    let source = state.editor.source();
    if !state.fitted {
        let (lo, hi) = source.grid.bounds();
        state.view.fit(bounds(lo, hi), &vp);
        state.view.mode = Mode::Free;
        state.fitted = true;
    }
    let response = ui.allocate_rect(rect, egui::Sense::click_and_drag());
    gestures(ui, &response, &vp, state);

    let view = state.view;
    let to = |p: Point| {
        let s = view.to_screen(to_view(p), &vp);
        egui::pos2(s.x as f32, s.y as f32)
    };
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(16, 17, 20));
    let align = state.draft;
    let moved = |p: Point| to(align.apply(p));

    if state.layers.streets {
        paint_streets(&painter, &state.streets, view.scale, &to);
    }
    refresh_pictures(ui.ctx(), state);
    if state.layers.grid {
        if let Some(p) = &state.grid_picture {
            quad(&painter, p.texture.id(), p.bounds, &moved, 220);
        }
    }
    if state.layers.points {
        if let Some(p) = &state.points_picture {
            quad(&painter, p.texture.id(), p.bounds, &moved, 255);
        }
    }
    let source = state.editor.source();
    if state.layers.chunks {
        let (origin, w, h) = source.grid.aligned_frame(&align);
        let r = source.grid.resolution;
        let hi = Point::new(origin.x + w as f64 * r, origin.y + h as f64 * r);
        for (key, (a, b)) in chunk_outlines(origin, origin, hi, CHUNK_SIZE) {
            let corners = [a, Point::new(b.x, a.y), b, Point::new(a.x, b.y)].map(&to);
            painter.add(egui::Shape::closed_line(
                corners.to_vec(),
                egui::Stroke::new(1.5, egui::Color32::from_rgb(90, 140, 230)),
            ));
            painter.text(
                corners[3] + egui::vec2(4.0, 4.0),
                egui::Align2::LEFT_TOP,
                format!("chunk {key}"),
                egui::FontId::proportional(12.0),
                egui::Color32::from_rgb(120, 160, 240),
            );
        }
    }
    if state.layers.path && !source.path.is_empty() {
        let step = (source.path.len() / 2000).max(1);
        let line: Vec<egui::Pos2> = source
            .path
            .iter()
            .step_by(step)
            .map(|p| moved(Point::new(p.xyz[0], p.xyz[1])))
            .collect();
        painter.add(egui::Shape::line(
            line,
            egui::Stroke::new(2.0, egui::Color32::from_rgb(230, 70, 70)),
        ));
    }
    if state.layers.labels {
        for label in state.editor.labels() {
            let [r, g, b] = label.class.rgb();
            let colour = egui::Color32::from_rgb(r, g, b);
            let region = label.region.aligned(&align);
            let corners: Vec<egui::Pos2> = region.corners().iter().map(|&p| to(p)).collect();
            painter.add(egui::Shape::closed_line(
                corners,
                egui::Stroke::new(2.0, colour),
            ));
            painter.text(
                to(region.centre()),
                egui::Align2::CENTER_CENTER,
                format!("{} {}", label.id, label.class.name()),
                egui::FontId::proportional(13.0),
                colour,
            );
        }
    }
    if let Some(region) = state.selection {
        let corners: Vec<egui::Pos2> = region.corners().iter().map(|&p| to(p)).collect();
        painter.add(egui::Shape::convex_polygon(
            corners,
            egui::Color32::from_rgba_unmultiplied(255, 220, 60, 40),
            egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 220, 60)),
        ));
    }
    let a = state.draft;
    painter.text(
        rect.left_bottom() + egui::vec2(10.0, -10.0),
        egui::Align2::LEFT_BOTTOM,
        format!(
            "{}   moved by hand: x {:.2} m, y {:.2} m, yaw {:.2}°{}   NDT points {} of {}",
            state.tool.name(),
            a.x_m,
            a.y_m,
            a.yaw_rad.to_degrees(),
            if a != state.editor.align() {
                " (not applied)"
            } else {
                ""
            },
            source.points.len() - state.editor.points_removed(),
            source.points.len()
        ),
        egui::FontId::proportional(14.0),
        egui::Color32::from_gray(210),
    );
}

fn gestures(ui: &egui::Ui, response: &egui::Response, vp: &Viewport, state: &mut EditorUi) {
    let scale = state.view.scale.0;
    let pointer = response
        .interact_pointer_pos()
        .or_else(|| response.hover_pos())
        .map(|p| ScreenPoint::new(f64::from(p.x), f64::from(p.y)));
    if response.hovered() {
        let (scroll, pinch) = ui.input(|i| (i.smooth_scroll_delta.y, i.zoom_delta()));
        let factor = (f64::from(scroll) * WHEEL_ZOOM).exp() * f64::from(pinch);
        if (factor - 1.0).abs() > 1e-6 {
            state.view.zoom(factor, pointer, vp);
        }
    }
    let delta = response.drag_delta();
    let (dx, dy) = (f64::from(delta.x), f64::from(delta.y));
    if response.dragged_by(egui::PointerButton::Secondary)
        || response.dragged_by(egui::PointerButton::Middle)
    {
        state.view.pan(dx, dy);
        return;
    }
    if !response.dragged_by(egui::PointerButton::Primary)
        && !response.drag_stopped_by(egui::PointerButton::Primary)
    {
        return;
    }
    match state.tool {
        Tool::Pan => state.view.pan(dx, dy),
        Tool::Align => {
            state.draft = tool::dragged(state.draft, dx, dy, scale);
            if response.drag_stopped() {
                state.commit_draft();
            }
        }
        Tool::Box => {
            let Some(at) = pointer.map(|s| {
                let p = state.view.to_map(s, vp);
                Point::new(p.x, p.y)
            }) else {
                return;
            };
            if response.drag_started() || state.drag_from.is_none() {
                state.drag_from = Some(at);
            }
            if let Some(from) = state.drag_from {
                state.selection = tool::drag_box(from, at, state.box_yaw_deg.to_radians());
            }
            if response.drag_stopped() {
                state.drag_from = None;
            }
        }
    }
}

/// Makes the layers' pictures again when the editor has changed.
fn refresh_pictures(ctx: &egui::Context, state: &mut EditorUi) {
    let version = state.version;
    let stale = |p: &Option<Picture>| p.as_ref().is_none_or(|p| p.version != version);
    if stale(&state.points_picture) {
        state.points_picture =
            HeightImage::new(state.editor.kept_points(), POINT_CELL).map(|image| {
                let colour = egui::ColorImage::from_rgba_unmultiplied(
                    [image.width, image.height],
                    &image.rgba(),
                );
                Picture {
                    texture: ctx.load_texture("ndt_points", colour, egui::TextureOptions::NEAREST),
                    bounds: image.bounds(),
                    version,
                }
            });
    }
    if stale(&state.grid_picture) {
        let grid = &state.editor.source().grid;
        let colour = egui::ColorImage::from_rgba_unmultiplied(
            [grid.width, grid.height],
            &grid_rgba(state.editor.cells(), grid.width),
        );
        state.grid_picture = Some(Picture {
            texture: ctx.load_texture("map2d", colour, egui::TextureOptions::NEAREST),
            bounds: grid.bounds(),
            version,
        });
    }
}

/// A picture on the map, its corners placed by `place` (so it may be turned).
fn quad(
    painter: &egui::Painter,
    texture: egui::TextureId,
    (lo, hi): (Point, Point),
    place: &impl Fn(Point) -> egui::Pos2,
    alpha: u8,
) {
    let corners = [
        (Point::new(lo.x, lo.y), egui::pos2(0.0, 1.0)),
        (Point::new(hi.x, lo.y), egui::pos2(1.0, 1.0)),
        (Point::new(hi.x, hi.y), egui::pos2(1.0, 0.0)),
        (Point::new(lo.x, hi.y), egui::pos2(0.0, 0.0)),
    ];
    let mut mesh = egui::Mesh::with_texture(texture);
    for (p, uv) in corners {
        mesh.vertices.push(egui::epaint::Vertex {
            pos: place(p),
            uv,
            color: egui::Color32::from_white_alpha(alpha),
        });
    }
    mesh.add_triangle(0, 1, 2);
    mesh.add_triangle(0, 2, 3);
    painter.add(mesh);
}

fn area_colour(area: Area) -> egui::Color32 {
    match area {
        Area::Green => egui::Color32::from_rgb(28, 48, 32),
        Area::Campus => egui::Color32::from_rgb(38, 34, 48),
        Area::Water => egui::Color32::from_rgb(26, 44, 72),
        Area::Building => egui::Color32::from_rgb(78, 70, 64),
    }
}

fn road_colour(class: RoadClass) -> egui::Color32 {
    match class {
        RoadClass::Highway | RoadClass::Major => egui::Color32::from_rgb(150, 130, 80),
        RoadClass::Medium | RoadClass::Minor => egui::Color32::from_rgb(105, 105, 115),
        RoadClass::Service => egui::Color32::from_rgb(80, 80, 90),
        RoadClass::Path | RoadClass::Rail => egui::Color32::from_rgb(70, 70, 70),
    }
}

fn paint_streets(
    painter: &egui::Painter,
    tiles: &[TileShapes],
    scale: PixelsPerMetre,
    to: &impl Fn(Point) -> egui::Pos2,
) {
    let at = |p: [f32; 2]| to(Point::new(f64::from(p[0]), f64::from(p[1])));
    for area in Area::ALL {
        let colour = area_colour(area);
        let mut mesh = egui::Mesh::default();
        for tile in tiles {
            for triangle in tile.triangles(area).as_chunks::<3>().0 {
                let base = mesh.vertices.len() as u32;
                for &p in triangle {
                    mesh.colored_vertex(at(p), colour);
                }
                mesh.add_triangle(base, base + 1, base + 2);
            }
        }
        painter.add(mesh);
    }
    for class in RoadClass::ALL {
        let width = (class.width_m() * scale.0).clamp(1.0, 60.0) as f32;
        let stroke = egui::Stroke::new(width, road_colour(class));
        for line in tiles.iter().flat_map(|t| &t.lines) {
            if line.kind == LineKind::Road(class) {
                painter.add(egui::Shape::line(
                    line.points.iter().map(|&p| at(p)).collect(),
                    stroke,
                ));
            }
        }
    }
}
