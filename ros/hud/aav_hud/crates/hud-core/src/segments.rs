//! Seven-segment digits, as the arcade gauge's digital box draws them: which segments
//! each character lights, and each segment's shape (a long hexagon) in a slanted cell.
//!
//! ```text
//!    aaaa
//!   f    b
//!   f    b
//!    gggg
//!   e    c
//!   e    c
//!    dddd  .
//! ```
//!
//! The screen draws all seven of every cell dim first (the unlit glass of a real
//! display), then the lit ones bright.

/// One of the seven segments.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Segment {
    /// Top.
    A,
    /// Upper right.
    B,
    /// Lower right.
    C,
    /// Bottom.
    D,
    /// Lower left.
    E,
    /// Upper left.
    F,
    /// Middle.
    G,
}

impl Segment {
    /// All seven, a to g.
    pub const ALL: [Segment; 7] = [
        Segment::A,
        Segment::B,
        Segment::C,
        Segment::D,
        Segment::E,
        Segment::F,
        Segment::G,
    ];

    fn bit(self) -> u8 {
        1 << self as u8
    }
}

/// The segments one character lights.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Lit(u8);

impl Lit {
    /// Nothing lit: a space.
    pub const NONE: Lit = Lit(0);

    fn of_segments(segments: &[Segment]) -> Self {
        Self(segments.iter().fold(0, |bits, s| bits | s.bit()))
    }

    /// The segments for a digit, a minus, or a space. Anything else: `None`.
    pub fn of(c: char) -> Option<Self> {
        use Segment::*;
        let segments: &[Segment] = match c {
            '0' => &[A, B, C, D, E, F],
            '1' => &[B, C],
            '2' => &[A, B, G, E, D],
            '3' => &[A, B, G, C, D],
            '4' => &[F, G, B, C],
            '5' => &[A, F, G, C, D],
            '6' => &[A, F, G, E, C, D],
            '7' => &[A, B, C],
            '8' => &Segment::ALL,
            '9' => &[A, B, C, D, F, G],
            '-' => &[G],
            ' ' => &[],
            _ => return None,
        };
        Some(Self::of_segments(segments))
    }

    /// Whether this character lights `segment`.
    pub fn has(self, segment: Segment) -> bool {
        self.0 & segment.bit() != 0
    }

    /// How many segments are lit.
    pub fn count(self) -> u32 {
        self.0.count_ones()
    }
}

/// One cell of the display: its segments, and its decimal point after it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Cell {
    pub lit: Lit,
    pub point: bool,
}

/// `text` as `width` cells, right-aligned, as a display shows a number. A `.` lights
/// the point of the digit before it. Characters with no segments (letters) show blank;
/// text too long keeps its right end.
pub fn cells(text: &str, width: usize) -> Vec<Cell> {
    let mut out: Vec<Cell> = Vec::new();
    for c in text.chars() {
        match (c, out.last_mut()) {
            ('.', Some(last)) if !last.point => last.point = true,
            ('.', _) => out.push(Cell {
                lit: Lit::NONE,
                point: true,
            }),
            (c, _) => out.push(Cell {
                lit: Lit::of(c).unwrap_or(Lit::NONE),
                point: false,
            }),
        }
    }
    let blank = width.saturating_sub(out.len());
    let skip = out.len().saturating_sub(width);
    std::iter::repeat_n(Cell::default(), blank)
        .chain(out.into_iter().skip(skip))
        .collect()
}

/// A point on the screen, (x, y), y down.
pub type Point = (f32, f32);

/// Where one digit is drawn: its box, its segments' thickness, and its slant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Digit {
    /// The top left corner of the upright box, before the slant.
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
    /// How thick each segment is.
    pub thick: f32,
    /// How far right the top leans, for each unit up: 0.1 is a gentle italic.
    pub slant: f32,
}

impl Digit {
    /// The gap between two segments' ends, so each reads as its own bar.
    fn gap(&self) -> f32 {
        self.thick * 0.18
    }

    /// From the upright box (u right, v down from the top) to the screen, slanted: the
    /// bottom stays put, the top moves right.
    fn place(&self, u: f32, v: f32) -> Point {
        (self.x + u + (self.height - v) * self.slant, self.y + v)
    }

    /// The segment's shape: a hexagon, pointed at both ends, corners in order round it.
    pub fn polygon(&self, segment: Segment) -> [Point; 6] {
        let (w, h, k) = (self.width, self.height, self.thick / 2.0);
        let g = self.gap();
        let horizontal = |v: f32| {
            let (u0, u1) = (k + g, w - k - g);
            [
                (u0, v),
                (u0 + k, v - k),
                (u1 - k, v - k),
                (u1, v),
                (u1 - k, v + k),
                (u0 + k, v + k),
            ]
        };
        let vertical = |u: f32, v0: f32, v1: f32| {
            let (v0, v1) = (v0 + g, v1 - g);
            [
                (u, v0),
                (u + k, v0 + k),
                (u + k, v1 - k),
                (u, v1),
                (u - k, v1 - k),
                (u - k, v0 + k),
            ]
        };
        let (top, mid, bottom) = (k, h / 2.0, h - k);
        let (left, right) = (k, w - k);
        let corners = match segment {
            Segment::A => horizontal(top),
            Segment::G => horizontal(mid),
            Segment::D => horizontal(bottom),
            Segment::F => vertical(left, top, mid),
            Segment::B => vertical(right, top, mid),
            Segment::E => vertical(left, mid, bottom),
            Segment::C => vertical(right, mid, bottom),
        };
        corners.map(|(u, v)| self.place(u, v))
    }

    /// The decimal point: a square's corners, just right of the bottom.
    pub fn point(&self) -> [Point; 4] {
        let s = self.thick;
        let (u, v) = (self.width + s * 0.35, self.height - s);
        [(u, v), (u + s, v), (u + s, v + s), (u, v + s)].map(|(u, v)| self.place(u, v))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lit(c: char) -> Vec<Segment> {
        let l = Lit::of(c).expect("a digit");
        Segment::ALL.into_iter().filter(|s| l.has(*s)).collect()
    }

    #[test]
    fn digits_light_the_right_segments() {
        use Segment::*;
        assert_eq!(lit('8'), Segment::ALL.to_vec());
        assert_eq!(lit('1'), vec![B, C]);
        assert_eq!(lit('7'), vec![A, B, C]);
        assert_eq!(lit('-'), vec![G]);
        assert!(lit(' ').is_empty());
        // The usual counts: 0 has six, 1 two, 4 four.
        let counts: Vec<u32> = "0123456789"
            .chars()
            .filter_map(Lit::of)
            .map(Lit::count)
            .collect();
        assert_eq!(counts, vec![6, 2, 5, 5, 4, 5, 6, 3, 7, 6]);
        assert_eq!(Lit::of('x'), None);
    }

    #[test]
    fn text_becomes_right_aligned_cells_with_points() {
        let c = cells("3.6", 3);
        assert_eq!(c.len(), 3);
        assert_eq!(c[0], Cell::default());
        assert_eq!(c[1].lit, Lit::of('3').expect("3"));
        assert!(c[1].point);
        assert_eq!(c[2].lit, Lit::of('6').expect("6"));
        assert!(!c[2].point);
        // "--.-": the point after the second minus.
        let c = cells("--.-", 3);
        assert!(c.iter().all(|x| x.lit == Lit::of('-').expect("-")));
        assert!(c[1].point);
        // Too long: the right end stays.
        let c = cells("123.4", 3);
        assert_eq!(c[0].lit, Lit::of('2').expect("2"));
        assert!(c[1].point);
        // Letters show blank.
        assert_eq!(cells("A", 1)[0].lit, Lit::NONE);
    }

    fn digit(slant: f32) -> Digit {
        Digit {
            x: 10.0,
            y: 20.0,
            width: 30.0,
            height: 50.0,
            thick: 6.0,
            slant,
        }
    }

    fn centre(p: &[Point]) -> Point {
        let n = p.len() as f32;
        let (x, y) = p.iter().fold((0.0, 0.0), |(x, y), q| (x + q.0, y + q.1));
        (x / n, y / n)
    }

    #[test]
    fn segments_sit_in_their_places() {
        let d = digit(0.0);
        let at = |s| centre(&d.polygon(s));
        // a above g above d; f left of b; e left of c; f above e.
        assert!(at(Segment::A).1 < at(Segment::G).1 && at(Segment::G).1 < at(Segment::D).1);
        assert!(at(Segment::F).0 < at(Segment::B).0);
        assert!(at(Segment::E).0 < at(Segment::C).0);
        assert!(at(Segment::F).1 < at(Segment::E).1);
        // Upright: every corner inside the box.
        for s in Segment::ALL {
            for (x, y) in d.polygon(s) {
                assert!(
                    (10.0..=40.0).contains(&x) && (20.0..=70.0).contains(&y),
                    "{s:?}"
                );
            }
        }
    }

    #[test]
    fn segments_do_not_touch() {
        // Where the top bar ends and the upper right bar begins, a gap stays between
        // their tips, so each reads as its own bar.
        let d = digit(0.0);
        let a_tip = d.polygon(Segment::A)[3];
        let b_tip = d.polygon(Segment::B)[0];
        let apart = ((a_tip.0 - b_tip.0).powi(2) + (a_tip.1 - b_tip.1).powi(2)).sqrt();
        assert!(apart > 1.0, "{apart}");
        let g_tip = d.polygon(Segment::G)[0];
        let f_end = d.polygon(Segment::F)[3];
        assert!(f_end.1 < g_tip.1, "f ends above g");
    }

    #[test]
    fn the_slant_leans_the_top_right() {
        let upright = digit(0.0);
        let leaning = digit(0.12);
        let top = |d: &Digit| centre(&d.polygon(Segment::A)).0;
        let bottom = |d: &Digit| centre(&d.polygon(Segment::D)).0;
        assert!((top(&upright) - bottom(&upright)).abs() < 1e-3);
        assert!(top(&leaning) > bottom(&leaning) + 4.0);
        // The bottom bar barely moves; heights stay.
        assert!((bottom(&leaning) - bottom(&upright)).abs() < 1.0);
        let y = |d: &Digit| centre(&d.polygon(Segment::A)).1;
        assert_eq!(y(&upright), y(&leaning));
    }

    #[test]
    fn the_point_sits_bottom_right() {
        let d = digit(0.0);
        let p = centre(&d.point());
        assert!(p.0 > 40.0 && p.1 > 60.0);
    }
}
