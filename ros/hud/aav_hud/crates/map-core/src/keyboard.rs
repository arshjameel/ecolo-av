//! The on-screen keyboard for the place search: the car's screen has no real keyboard.
//!
//! ```text
//!   1   2   3   4   5   6   7   8   9   0  [ Back  ]
//!   q   w   e   r   t   y   u   i   o   p   é   è
//!   a   s   d   f   g   h   j   k   l   à   ç   ê
//!   z   x   c   v   b   n   m   ô   '   -   .   &
//!  [ Close ][ Clear ][       Space       ][ Enter  ]
//! ```
//!
//! Twelve columns, five rows. Lower case only: the search ignores case. The accents are
//! the ones in Ottawa's French names (rue Élisabeth-Bruyère, chemin de la Côte...).
//!
//! This module only places the keys and says which one a tap hit; the window (map-app)
//! draws them with raylib, and the letters go to [`Finder`](crate::destination::Finder),
//! which does the editing. A mouse click is a tap, so it works the same on a Mac.

use crate::{
    destination::Key,
    view::{ScreenPoint, Viewport},
};

/// One key on the on-screen keyboard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyCap {
    Char(char),
    Space,
    Backspace,
    Clear,
    /// Searches what's typed: picks the highlighted result (the first, after typing).
    Enter,
    /// Hides the keyboard; the box and its results stay, to tap one.
    Close,
}

/// What a key does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Press {
    /// A key for the search box.
    Search(Key),
    /// Hide the keyboard.
    Close,
}

impl KeyCap {
    /// The words on the key.
    #[must_use]
    pub fn label(self) -> String {
        match self {
            Self::Char(c) => c.to_string(),
            Self::Space => "Space".to_owned(),
            Self::Backspace => "Back".to_owned(),
            Self::Clear => "Clear".to_owned(),
            Self::Enter => "Enter".to_owned(),
            Self::Close => "Close".to_owned(),
        }
    }

    /// What pressing it does.
    #[must_use]
    pub fn press(self) -> Press {
        match self {
            Self::Char(c) => Press::Search(Key::Char(c)),
            Self::Space => Press::Search(Key::Char(' ')),
            Self::Backspace => Press::Search(Key::Backspace),
            Self::Clear => Press::Search(Key::Clear),
            Self::Enter => Press::Search(Key::Enter),
            Self::Close => Press::Close,
        }
    }

    /// Whether it's an editing or command key (drawn in another colour), not a letter.
    #[must_use]
    pub fn is_command(self) -> bool {
        !matches!(self, Self::Char(_) | Self::Space)
    }
}

/// Columns in each row, in key widths.
const COLUMNS: u8 = 12;
/// Smallest key, in pixels: a fingertip.
pub const MIN_KEY_PX: f64 = 60.0;
/// A key's width at the reference size (1600 x 900, `u` = 1).
const KEY_W: f64 = 84.0;
/// A key's height, as a share of its width (at least [`MIN_KEY_PX`]).
const KEY_H_SHARE: f64 = 0.8;

/// A letter key, one column wide.
const fn ch(c: char) -> (KeyCap, u8) {
    (KeyCap::Char(c), 1)
}

/// The rows, top to bottom: each key and its width in columns. Each row is 12 wide.
const ROWS: [&[(KeyCap, u8)]; 5] = [
    &[
        ch('1'),
        ch('2'),
        ch('3'),
        ch('4'),
        ch('5'),
        ch('6'),
        ch('7'),
        ch('8'),
        ch('9'),
        ch('0'),
        (KeyCap::Backspace, 2),
    ],
    &[
        ch('q'),
        ch('w'),
        ch('e'),
        ch('r'),
        ch('t'),
        ch('y'),
        ch('u'),
        ch('i'),
        ch('o'),
        ch('p'),
        ch('é'),
        ch('è'),
    ],
    &[
        ch('a'),
        ch('s'),
        ch('d'),
        ch('f'),
        ch('g'),
        ch('h'),
        ch('j'),
        ch('k'),
        ch('l'),
        ch('à'),
        ch('ç'),
        ch('ê'),
    ],
    &[
        ch('z'),
        ch('x'),
        ch('c'),
        ch('v'),
        ch('b'),
        ch('n'),
        ch('m'),
        ch('ô'),
        ch('\''),
        ch('-'),
        ch('.'),
        ch('&'),
    ],
    &[
        (KeyCap::Close, 2),
        (KeyCap::Clear, 2),
        (KeyCap::Space, 5),
        (KeyCap::Enter, 3),
    ],
];

/// The keyboard placed in the window this frame: its back and every key's rectangle.
#[derive(Debug, Clone, PartialEq)]
pub struct Keyboard {
    back: Viewport,
    keys: Vec<(KeyCap, Viewport)>,
    key_h: f64,
}

impl Keyboard {
    /// Places the keyboard at the bottom of `window`, centred on `centre_x` (the map's
    /// middle) when it fits, else kept inside the window. `u`: the window's scale, 1 at
    /// 1600 x 900. Keys grow with `u`, shrink to fit a narrow window, and are never
    /// under [`MIN_KEY_PX`] unless the window itself is too narrow for that.
    #[must_use]
    pub fn new(window: Viewport, centre_x: f64, u: f64) -> Self {
        let gap = (6.0 * u).max(4.0);
        let pad = 2.0 * gap;
        let cols = f64::from(COLUMNS);
        let fits = (window.width - 2.0 * pad - (cols - 1.0) * gap) / cols;
        let key_w = (KEY_W * u).max(MIN_KEY_PX).min(fits).max(1.0);
        let key_h = (key_w * KEY_H_SHARE).max(MIN_KEY_PX.min(key_w));
        let rows = ROWS.len() as f64;
        let width = cols * key_w + (cols - 1.0) * gap + 2.0 * pad;
        let height = rows * key_h + (rows - 1.0) * gap + 2.0 * pad;
        let margin = (12.0 * u).max(4.0);
        let x = (centre_x - width / 2.0)
            .min(window.x + window.width - width)
            .max(window.x);
        let y = (window.y + window.height - margin - height).max(window.y);
        let back = Viewport {
            x,
            y,
            width,
            height,
        };
        let keys = ROWS
            .iter()
            .enumerate()
            .flat_map(|(row, keys)| {
                let top = y + pad + row as f64 * (key_h + gap);
                keys.iter().scan(0u8, move |col, &(cap, span)| {
                    let left = x + pad + f64::from(*col) * (key_w + gap);
                    *col += span;
                    let w = f64::from(span) * key_w + f64::from(span - 1) * gap;
                    Some((
                        cap,
                        Viewport {
                            x: left,
                            y: top,
                            width: w,
                            height: key_h,
                        },
                    ))
                })
            })
            .collect();
        Self { back, keys, key_h }
    }

    /// The keyboard's whole area, its back: taps there never reach the map.
    #[must_use]
    pub fn area(&self) -> Viewport {
        self.back
    }

    /// Whether `at` is on the keyboard (a key or the gaps between).
    #[must_use]
    pub fn contains(&self, at: ScreenPoint) -> bool {
        self.back.contains(at)
    }

    /// Every key and where it is, row by row.
    pub fn keys(&self) -> impl Iterator<Item = (KeyCap, Viewport)> + '_ {
        self.keys.iter().copied()
    }

    /// One key's height, in pixels (the window sizes the letters by it).
    #[must_use]
    pub fn key_height(&self) -> f64 {
        self.key_h
    }

    /// The key at `at`, if any: a tap between two keys hits neither.
    #[must_use]
    pub fn key_at(&self, at: ScreenPoint) -> Option<KeyCap> {
        self.keys
            .iter()
            .find(|(_, r)| r.contains(at))
            .map(|&(cap, _)| cap)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::destination::{Action, Finder};

    fn window(w: f64, h: f64) -> Viewport {
        Viewport {
            x: 0.0,
            y: 0.0,
            width: w,
            height: h,
        }
    }

    fn rect_of(kb: &Keyboard, cap: KeyCap) -> Viewport {
        kb.keys()
            .find(|&(k, _)| k == cap)
            .map(|(_, r)| r)
            .expect("the key is there")
    }

    fn middle(r: Viewport) -> ScreenPoint {
        r.centre()
    }

    #[test]
    fn every_row_is_twelve_wide() {
        for row in ROWS {
            let width: u8 = row.iter().map(|&(_, span)| span).sum();
            assert_eq!(width, COLUMNS);
        }
    }

    #[test]
    fn it_has_the_keys_the_search_needs() {
        let kb = Keyboard::new(window(1600.0, 900.0), 600.0, 1.0);
        let caps: Vec<KeyCap> = kb.keys().map(|(k, _)| k).collect();
        for c in "abcdefghijklmnopqrstuvwxyz0123456789éèàçêô'-".chars() {
            assert!(caps.contains(&KeyCap::Char(c)), "{c} is missing");
        }
        for k in [
            KeyCap::Space,
            KeyCap::Backspace,
            KeyCap::Clear,
            KeyCap::Enter,
            KeyCap::Close,
        ] {
            assert!(caps.contains(&k), "{k:?} is missing");
        }
    }

    #[test]
    fn the_key_at_a_point() {
        let kb = Keyboard::new(window(1600.0, 900.0), 600.0, 1.0);
        let q = rect_of(&kb, KeyCap::Char('q'));
        assert_eq!(kb.key_at(middle(q)), Some(KeyCap::Char('q')));
        // Its top-left corner is on it; one pixel left of it is the gap.
        assert_eq!(
            kb.key_at(ScreenPoint::new(q.x, q.y)),
            Some(KeyCap::Char('q'))
        );
        assert_eq!(kb.key_at(ScreenPoint::new(q.x - 1.0, q.y + 5.0)), None);
        assert!(kb.contains(ScreenPoint::new(q.x - 1.0, q.y + 5.0)));
        // The wide keys: anywhere on them.
        let space = rect_of(&kb, KeyCap::Space);
        let near_end = ScreenPoint::new(space.x + space.width - 2.0, space.y + 2.0);
        assert_eq!(kb.key_at(near_end), Some(KeyCap::Space));
        assert_eq!(
            kb.key_at(middle(rect_of(&kb, KeyCap::Enter))),
            Some(KeyCap::Enter)
        );
        // Off the keyboard: nothing.
        assert_eq!(kb.key_at(ScreenPoint::new(5.0, 5.0)), None);
        assert!(!kb.contains(ScreenPoint::new(5.0, 5.0)));
    }

    #[test]
    fn keys_never_overlap_and_stay_in_the_window() {
        for (w, h, u) in [
            (1600.0, 900.0, 1.0),
            (800.0, 450.0, 0.6),
            (3840.0, 2160.0, 2.4),
        ] {
            let win = window(w, h);
            let kb = Keyboard::new(win, w * 0.4, u);
            let a = kb.area();
            assert!(a.x >= 0.0 && a.x + a.width <= w + 1e-9, "inside at {w}");
            assert!(a.y >= 0.0 && a.y + a.height <= h + 1e-9);
            let keys: Vec<_> = kb.keys().collect();
            for (i, (_, r)) in keys.iter().enumerate() {
                for (_, s) in &keys[i + 1..] {
                    let apart = r.x + r.width <= s.x
                        || s.x + s.width <= r.x
                        || r.y + r.height <= s.y
                        || s.y + s.height <= r.y;
                    assert!(apart, "{r:?} overlaps {s:?}");
                }
            }
        }
    }

    #[test]
    fn the_layout_scales_with_the_window() {
        let key_w = |kb: &Keyboard| rect_of(kb, KeyCap::Char('a')).width;
        let reference = Keyboard::new(window(1600.0, 900.0), 600.0, 1.0);
        assert_eq!(key_w(&reference), KEY_W);
        let big = Keyboard::new(window(3200.0, 1800.0), 1200.0, 2.0);
        assert_eq!(key_w(&big), 2.0 * KEY_W);
        // The smallest window: keys still a fingertip wide and tall.
        let small = Keyboard::new(window(800.0, 450.0), 280.0, 0.6);
        assert!(key_w(&small) >= MIN_KEY_PX);
        assert!(small.key_height() >= MIN_KEY_PX);
        // Too narrow for 60 px keys: they shrink to fit rather than leave the window.
        let tiny = Keyboard::new(window(500.0, 400.0), 250.0, 0.6);
        assert!(tiny.area().width <= 500.0 + 1e-9);
    }

    #[test]
    fn it_sits_at_the_bottom_centred_on_the_map() {
        let kb = Keyboard::new(window(1600.0, 900.0), 600.0, 1.0);
        let a = kb.area();
        assert!((a.x + a.width / 2.0 - 600.0).abs() < 1e-9, "centred on 600");
        assert!(
            (a.y + a.height - 888.0).abs() < 1e-9,
            "12 px above the bottom"
        );
        // Centred too near the edge: pushed back inside.
        let kb = Keyboard::new(window(1600.0, 900.0), 10.0, 1.0);
        assert_eq!(kb.area().x, 0.0);
    }

    /// Taps a key, as the window does: into the search box's [`Finder`].
    fn tap(f: &mut Finder, kb: &Keyboard, cap: KeyCap) -> Option<Action> {
        let at = middle(rect_of(kb, cap));
        match kb.key_at(at)?.press() {
            Press::Search(key) => Some(f.key(key, true)),
            Press::Close => None,
        }
    }

    #[test]
    fn typing_with_taps_edits_the_box() {
        let kb = Keyboard::new(window(1600.0, 900.0), 600.0, 1.0);
        let mut f = Finder::new();
        f.key(Key::Open, true);
        for c in "rue é".chars() {
            let cap = if c == ' ' {
                KeyCap::Space
            } else {
                KeyCap::Char(c)
            };
            assert_eq!(tap(&mut f, &kb, cap), Some(Action::Search));
        }
        assert_eq!(f.query(), "rue é");
        // Backspace takes the whole é (two bytes), not half of it.
        tap(&mut f, &kb, KeyCap::Backspace);
        assert_eq!(f.query(), "rue ");
        tap(&mut f, &kb, KeyCap::Char('ç'));
        assert_eq!(f.query(), "rue ç");
        assert_eq!(tap(&mut f, &kb, KeyCap::Clear), Some(Action::Search));
        assert_eq!(f.query(), "");
        assert_eq!(tap(&mut f, &kb, KeyCap::Close), None, "Close hides, no key");
        assert!(f.is_typing(), "the box stays open");
    }

    #[test]
    fn enter_picks_the_first_result() {
        use crate::{
            earth::LatLon,
            places::{Match, Place},
        };
        let kb = Keyboard::new(window(1600.0, 900.0), 600.0, 1.0);
        let mut f = Finder::new();
        f.prefill("carl");
        let place = Place {
            id: 1,
            title: "Carleton".into(),
            kind: "university".into(),
            address: None,
            at: LatLon::new(45.38, -75.70),
            matched: Match::Prefix,
            distance: None,
        };
        f.set_results(Ok(vec![place.clone()]));
        assert_eq!(tap(&mut f, &kb, KeyCap::Enter), Some(Action::Show(place)));
        assert!(!f.is_typing(), "picked: the keyboard's job is done");
    }

    #[test]
    fn labels_and_kinds() {
        assert_eq!(KeyCap::Char('é').label(), "é");
        assert_eq!(KeyCap::Backspace.label(), "Back");
        assert!(KeyCap::Enter.is_command());
        assert!(!KeyCap::Space.is_command());
        assert_eq!(KeyCap::Space.press(), Press::Search(Key::Char(' ')));
    }
}
