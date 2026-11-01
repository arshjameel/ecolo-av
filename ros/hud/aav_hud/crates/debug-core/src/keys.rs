//! The keys, and the layers they turn on and off: one place, so the window, its key
//! legend and `--help` all say the same.

use crate::orbit::Mode;

/// Which cubes the 3D view shows (key K).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CubeView {
    /// Every cube seen in the last `keep_cubes_for` seconds, fading: the "occupancy
    /// network" look.
    #[default]
    Kept,
    /// The newest scan's cubes only.
    Scan,
}

impl CubeView {
    pub fn toggled(self) -> Self {
        match self {
            Self::Kept => Self::Scan,
            Self::Scan => Self::Kept,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Kept => "kept, fading",
            Self::Scan => "this scan only",
        }
    }
}

/// How the window is laid out (key L).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Layout {
    /// The seeing screen (docs/designs/2026-10-29_two-screens.md): the 3D cubes big on
    /// the left; on the right, NDT's map from above with every localizer's car, and
    /// NDT's match (its cells, the latest scan over them, the score).
    #[default]
    Seeing,
    /// The debug HUD as it was: the 3D view and the panel of every sensor.
    Panel,
}

impl Layout {
    pub fn toggled(self) -> Self {
        match self {
            Self::Seeing => Self::Panel,
            Self::Panel => Self::Seeing,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Seeing => "seeing: cubes, map, NDT",
            Self::Panel => "panel: every sensor",
        }
    }
}

/// What the 3D view draws besides the localizers, and the window's layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Layers {
    /// The seeing screen, or the sensor panel (key L).
    pub layout: Layout,
    pub cubes: CubeView,
    /// NDT's map as faint wireframe cubes (key M).
    pub ndt_map: bool,
    /// AMCL's particles (key A).
    pub particles: bool,
    /// The key legend itself (key H).
    pub keys: bool,
}

impl Default for Layers {
    fn default() -> Self {
        Self {
            layout: Layout::Seeing,
            cubes: CubeView::Kept,
            ndt_map: false,
            particles: true,
            keys: true,
        }
    }
}

/// What a key does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// C: chase, follow, free.
    Camera,
    /// R: the camera back behind the car.
    ResetCamera,
    /// K: kept cubes or this scan's.
    Cubes,
    /// M: NDT's map.
    NdtMap,
    /// A: AMCL's particles.
    Particles,
    /// H: the key legend.
    Keys,
    /// L: the seeing screen, or the sensor panel.
    Layout,
    /// F11: full screen.
    FullScreen,
}

impl Action {
    /// Every key, in the legend's order (keys 1-4 come after them).
    pub const ALL: [Action; 8] = [
        Action::Layout,
        Action::Cubes,
        Action::NdtMap,
        Action::Particles,
        Action::Camera,
        Action::ResetCamera,
        Action::Keys,
        Action::FullScreen,
    ];

    /// The key, as printed.
    pub fn key(self) -> &'static str {
        match self {
            Self::Camera => "C",
            Self::ResetCamera => "R",
            Self::Cubes => "K",
            Self::NdtMap => "M",
            Self::Particles => "A",
            Self::Keys => "H",
            Self::Layout => "L",
            Self::FullScreen => "F11",
        }
    }

    /// What it does, in a few words.
    pub fn what(self) -> &'static str {
        match self {
            Self::Camera => "camera",
            Self::ResetCamera => "reset the camera",
            Self::Cubes => "cubes",
            Self::NdtMap => "NDT's map",
            Self::Particles => "AMCL's particles",
            Self::Keys => "hide these keys",
            Self::Layout => "layout",
            Self::FullScreen => "full screen",
        }
    }
}

impl Layers {
    /// Takes a key's action (the camera's and the window's are the caller's).
    pub fn apply(&mut self, action: Action) {
        match action {
            Action::Cubes => self.cubes = self.cubes.toggled(),
            Action::NdtMap => self.ndt_map = !self.ndt_map,
            Action::Particles => self.particles = !self.particles,
            Action::Keys => self.keys = !self.keys,
            Action::Layout => self.layout = self.layout.toggled(),
            Action::Camera | Action::ResetCamera | Action::FullScreen => {}
        }
    }

    /// The legend: each key, what it does, and what it is now (if it has a state).
    /// `has_map`: whether NDT's map was loaded (else M says so).
    pub fn legend(&self, camera: Mode, has_map: bool) -> Vec<KeyLine> {
        let on = |b: bool| if b { "on" } else { "off" };
        let mut lines: Vec<KeyLine> = Action::ALL
            .into_iter()
            .map(|a| {
                let now = match a {
                    Action::Cubes => self.cubes.label().to_owned(),
                    Action::NdtMap if !has_map => "no map".to_owned(),
                    Action::NdtMap => on(self.ndt_map).to_owned(),
                    Action::Particles => on(self.particles).to_owned(),
                    Action::Camera => camera.label().to_owned(),
                    Action::Layout => self.layout.label().to_owned(),
                    _ => String::new(),
                };
                KeyLine {
                    key: a.key(),
                    what: a.what(),
                    now,
                }
            })
            .collect();
        lines.push(KeyLine {
            key: "1-4",
            what: "tracker (sim)",
            now: "pp, stanley, lqr, mpc".to_owned(),
        });
        lines.push(KeyLine {
            key: "mouse",
            what: "drag turn, right-drag move, wheel zoom",
            now: String::new(),
        });
        lines
    }
}

/// One line of the key legend.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KeyLine {
    pub key: &'static str,
    pub what: &'static str,
    /// Its state now, or empty.
    pub now: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_toggle_their_layers() {
        let mut l = Layers::default();
        assert_eq!(l.cubes, CubeView::Kept);
        l.apply(Action::Cubes);
        assert_eq!(l.cubes, CubeView::Scan);
        l.apply(Action::NdtMap);
        l.apply(Action::Particles);
        assert!(l.ndt_map && !l.particles);
        l.apply(Action::Camera);
        l.apply(Action::Cubes);
        assert_eq!(l.cubes, CubeView::Kept);
        assert_eq!(l.layout, Layout::Seeing);
        l.apply(Action::Layout);
        assert_eq!(l.layout, Layout::Panel);
        l.apply(Action::Layout);
        assert_eq!(l.layout, Layout::Seeing);
    }

    #[test]
    fn the_legend_says_each_key_and_its_state() {
        let l = Layers::default();
        let lines = l.legend(Mode::Chase, false);
        let m = lines.iter().find(|k| k.key == "M").expect("M is listed");
        assert_eq!(m.now, "no map");
        let k = lines.iter().find(|k| k.key == "K").expect("K is listed");
        assert_eq!(k.now, "kept, fading");
        let keys: Vec<&str> = lines.iter().map(|k| k.key).collect();
        let mut unique = keys.clone();
        unique.sort_unstable();
        unique.dedup();
        assert_eq!(unique.len(), keys.len(), "a key used twice: {keys:?}");
        let with_map = l.legend(Mode::Chase, true);
        assert!(with_map.iter().any(|k| k.key == "M" && k.now == "off"));
    }
}
