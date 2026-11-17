//! What `graphics:=pretty` adds to the pictures, with no engine (TODO 244).
//!
//! | Part | Here | The renderer |
//! |---|---|---|
//! | [`Graphics`] | simple or pretty, and which one a computer gets | reads `--graphics` |
//! | [`Surface`] | grass, asphalt, concrete, paint or brick, guessed from a shape's name | its roughness |
//! | [`Detail`] | a small tiling picture of that surface, made from noise | lays it over the shape |
//! | [`sky`] | the sky's colour in each direction: haze at the horizon, blue above, the sun's glow | a skybox |
//! | [`haze`] | the colour far things fade into | distance fog |
//!
//! Only the cameras see any of it: the lidar and radar read the same shapes as before.
//! A detail picture keeps its surface's average colour (its [`Detail::gain`]), so a scored
//! check that looks at colours sees the same road and grass, with grain on them.

use std::{fmt, str::FromStr};

use crate::geom::Vec3;
use crate::world::{Look, Rgb};

/// How much the cameras draw.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Graphics {
    /// Plain colours, a flat sky, the sun's shadows: as Gazebo drew. Cheap in software.
    Simple,
    /// Grain on the ground, a sky with haze and the sun's glow, fog far off, longer shadows.
    Pretty,
}

impl Graphics {
    /// The choice when nobody says: pretty on a GPU, simple drawn in software (CI, no GPU),
    /// where every extra costs real time.
    #[must_use]
    pub fn for_adapter(software: bool) -> Self {
        if software {
            Self::Simple
        } else {
            Self::Pretty
        }
    }
}

impl fmt::Display for Graphics {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Simple => "simple",
            Self::Pretty => "pretty",
        })
    }
}

/// A word that isn't `simple` or `pretty`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UnknownGraphics(pub String);

impl fmt::Display for UnknownGraphics {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "graphics {:?}: say simple or pretty", self.0)
    }
}

impl std::error::Error for UnknownGraphics {}

impl FromStr for Graphics {
    type Err = UnknownGraphics;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "simple" => Ok(Self::Simple),
            "pretty" => Ok(Self::Pretty),
            other => Err(UnknownGraphics(other.to_owned())),
        }
    }
}

/// What a shape is made of, for its grain and shine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Surface {
    /// Lawn: patches of lighter and yellower green.
    Grass,
    /// Road and lot: fine grain and darker blotches.
    Asphalt,
    /// Kerbs and pavements: faint blotches, a joint every 1.5 m.
    Concrete,
    /// Painted lines: almost clean, a little wear.
    Paint,
    /// Brick walls: rows of bricks, each its own shade, dark mortar.
    Brick,
    /// Anything else: its plain colour, as before.
    Plain,
}

/// Words in a shape's name and the surface they mean, first match wins.
const NAMES: [(&str, Surface); 15] = [
    ("line", Surface::Paint),
    ("paint", Surface::Paint),
    ("stripe", Surface::Paint),
    ("grass", Surface::Grass),
    ("lawn", Surface::Grass),
    ("tarmac", Surface::Asphalt),
    ("asphalt", Surface::Asphalt),
    ("road", Surface::Asphalt),
    ("lot", Surface::Asphalt),
    ("kerb", Surface::Concrete),
    ("curb", Surface::Concrete),
    ("concrete", Surface::Concrete),
    ("pavement", Surface::Concrete),
    ("brick", Surface::Brick),
    ("wall", Surface::Brick),
];

impl Surface {
    /// Guesses from the shape's own name (its last part: `road/tarmac` is tarmac), else a
    /// green ground is grass. A shape with its own picture keeps it: [`Surface::Plain`].
    #[must_use]
    pub fn guess(name: &str, look: &Look) -> Self {
        if look.texture.is_some() {
            return Self::Plain;
        }
        let own = name
            .rsplit('/')
            .find(|part| !part.is_empty() && *part != "visual")
            .unwrap_or("")
            .to_ascii_lowercase();
        if let Some((_, surface)) = NAMES.iter().find(|(word, _)| own.contains(word)) {
            return *surface;
        }
        let Rgb { r, g, b } = look.colour;
        if own.contains("ground") && g > 1.2 * r && g > 1.2 * b {
            Self::Grass
        } else {
            Self::Plain
        }
    }

    /// How rough it is to the light (Bevy's perceptual roughness): 1 is chalk, 0 a mirror.
    #[must_use]
    pub fn roughness(self) -> f32 {
        match self {
            Self::Grass => 0.92,
            Self::Asphalt => 0.85,
            Self::Concrete | Self::Brick => 0.9,
            Self::Paint => 0.6,
            Self::Plain => 0.95,
        }
    }

    /// Metres one detail picture covers before it repeats.
    #[must_use]
    pub fn tile(self) -> f64 {
        match self {
            Self::Grass => 6.0,
            Self::Asphalt => 4.0,
            Self::Concrete => 3.0,
            Self::Paint => 2.0,
            Self::Brick | Self::Plain => 1.0,
        }
    }

    /// Its detail picture, `size` pixels square (a power of two), or none for
    /// [`Surface::Plain`]. The same picture every run.
    #[must_use]
    pub fn detail(self, size: u32) -> Option<Detail> {
        let n = size.max(8);
        let at = |f: &dyn Fn(f64, f64) -> [f64; 3]| -> Vec<[f64; 3]> {
            (0..n * n)
                .map(|i| {
                    let (x, y) = (f64::from(i % n), f64::from(i / n));
                    f((x + 0.5) / f64::from(n), (y + 0.5) / f64::from(n))
                })
                .collect()
        };
        let grain = |u: f64, v: f64, seed: u32| hash(cell(u, n), cell(v, n), seed);
        let texels = match self {
            Self::Plain => return None,
            Self::Asphalt => at(&|u, v| {
                let g = 0.10 * (grain(u, v, 1) - 0.5) * 2.0;
                let blotch = 0.08 * (fbm(u, v, 4, 4, 2) - 0.5) * 2.0;
                let stone = if grain(u, v, 3) > 0.985 { 0.18 } else { 0.0 };
                grey(1.0 + g + blotch + stone)
            }),
            Self::Grass => at(&|u, v| {
                let patch = (fbm(u, v, 3, 3, 4) - 0.5) * 2.0;
                let blade = 0.16 * (grain(u, v, 5) - 0.5) * 2.0;
                let k = 1.0 + 0.16 * patch + blade;
                // Lighter patches turn yellower, darker ones bluer.
                [k * (1.0 + 0.12 * patch), k, k * (1.0 - 0.15 * patch)]
            }),
            Self::Concrete => at(&|u, v| {
                let g = 0.04 * (grain(u, v, 6) - 0.5) * 2.0;
                let blotch = 0.06 * (fbm(u, v, 3, 3, 7) - 0.5) * 2.0;
                // A joint across every half picture: every 1.5 m.
                let joint = if (u * 2.0).fract() < 1.5 / f64::from(n) {
                    -0.3
                } else {
                    0.0
                };
                grey(1.0 + g + blotch + joint)
            }),
            Self::Paint => at(&|u, v| {
                let wear = 0.05 * (fbm(u, v, 4, 3, 8) - 0.5) * 2.0;
                grey(1.0 + wear + 0.03 * (grain(u, v, 9) - 0.5))
            }),
            Self::Brick => at(&|u, v| {
                // 12 rows a metre, 4 bricks a row, every other row half a brick along.
                let row = (v * 12.0).floor();
                let along = u * 4.0 + 0.5 * (row % 2.0);
                let (fu, fv) = (along.fract(), (v * 12.0).fract());
                if fu < 0.04 || fv < 0.12 {
                    grey(0.55)
                } else {
                    let brick = hash(along.floor() as u32 % 4, row as u32, 10);
                    grey(1.0 + 0.3 * (brick - 0.5) + 0.06 * (grain(u, v, 11) - 0.5))
                }
            }),
        };
        Some(Detail::from_multipliers(n, &texels))
    }
}

fn grey(k: f64) -> [f64; 3] {
    [k, k, k]
}

/// A tiling picture of a surface's grain, to multiply its colour by.
#[derive(Clone, Debug, PartialEq)]
pub struct Detail {
    /// Pixels across and down.
    pub size: u32,
    /// RGBA, row by row, **linear** (not sRGB): each a multiplier of the colour, 0 to 1.
    pub rgba: Vec<u8>,
    /// Multiply the colour by this too: the picture's average comes back to 1.
    pub gain: f32,
}

impl Detail {
    /// Multipliers around 1 as bytes: scaled down to fit, the scale kept as the gain.
    fn from_multipliers(size: u32, texels: &[[f64; 3]]) -> Self {
        let top = texels
            .iter()
            .flatten()
            .fold(f64::MIN_POSITIVE, |a, &b| a.max(b));
        let mean: f64 = texels.iter().flatten().sum::<f64>() / (3 * texels.len()).max(1) as f64;
        let rgba = texels
            .iter()
            .flat_map(|t| {
                let byte = |c: f64| (c / top * 255.0).round().clamp(0.0, 255.0) as u8;
                [byte(t[0]), byte(t[1]), byte(t[2]), 255]
            })
            .collect();
        Self {
            size,
            rgba,
            gain: (top / mean) as f32,
        }
    }

    /// The picture's average multiplier with its gain: 1, give or take rounding.
    #[must_use]
    pub fn mean(&self) -> f32 {
        let sum: u64 = self
            .rgba
            .as_chunks::<4>()
            .0
            .iter()
            .flat_map(|p| &p[..3])
            .map(|&b| u64::from(b))
            .sum();
        let count = (self.rgba.len() / 4 * 3).max(1) as f32;
        sum as f32 / count / 255.0 * self.gain
    }
}

/// The colour far things fade into: the sky's colour, paler, as air looks at the horizon.
#[must_use]
pub fn haze(sky: Rgb) -> Rgb {
    mix(sky, Rgb::new(0.92, 0.95, 1.0), 0.45)
}

/// The sky's colour looking along `dir` (map frame, z up; any length): [`haze`] at and
/// below the horizon, deepening to a stronger blue overhead, and a warm glow around the
/// sun when there is one. `sun` is which way its light travels (as [`crate::Sun`]).
#[must_use]
pub fn sky(dir: Vec3, base: Rgb, sun: Option<Vec3>) -> Rgb {
    let Some(d) = dir.normalized() else {
        return haze(base);
    };
    let horizon = haze(base);
    let zenith = Rgb::new(base.r * 0.55, base.g * 0.8, base.b.max(0.6));
    let up = d.z.max(0.0).sqrt() as f32;
    let mut c = mix(horizon, zenith, up);
    if let Some(towards) = sun.and_then(|s| (-s).normalized()) {
        let a = d.dot(towards).max(0.0);
        let glow = (0.5 * a.powi(64) + 0.12 * a.powi(6)) as f32;
        let disc = if a > 0.9995 { 1.0 } else { 0.0 };
        let add = (glow + disc).min(1.0);
        c = Rgb::new(
            (c.r + add).min(1.0),
            (c.g + add * 0.9).min(1.0),
            (c.b + add * 0.7).min(1.0),
        );
    }
    c
}

fn mix(a: Rgb, b: Rgb, t: f32) -> Rgb {
    Rgb::new(
        a.r + (b.r - a.r) * t,
        a.g + (b.g - a.g) * t,
        a.b + (b.b - a.b) * t,
    )
}

/// Which cell of `n` a picture coordinate (0 to 1) falls in.
fn cell(t: f64, n: u32) -> u32 {
    ((t * f64::from(n)) as u32).min(n - 1)
}

/// A number from 0 to 1 for a cell and a seed: the same every run (a 32-bit mix).
fn hash(x: u32, y: u32, seed: u32) -> f64 {
    let mut h = x
        .wrapping_mul(0x8da6_b343)
        .wrapping_add(y.wrapping_mul(0xd816_3841))
        .wrapping_add(seed.wrapping_mul(0xcb1a_b31f));
    h ^= h >> 13;
    h = h.wrapping_mul(0x5bd1_e995);
    h ^= h >> 15;
    f64::from(h) / f64::from(u32::MAX)
}

/// Smooth noise from 0 to 1 that repeats every picture: `period` cells across.
fn value_noise(u: f64, v: f64, period: u32, seed: u32) -> f64 {
    let (x, y) = (u * f64::from(period), v * f64::from(period));
    let (x0, y0) = (x.floor(), y.floor());
    let (fx, fy) = (x - x0, y - y0);
    let smooth = |t: f64| t * t * (3.0 - 2.0 * t);
    let corner =
        |dx: u32, dy: u32| hash((x0 as u32 + dx) % period, (y0 as u32 + dy) % period, seed);
    let top = corner(0, 0) + (corner(1, 0) - corner(0, 0)) * smooth(fx);
    let bottom = corner(0, 1) + (corner(1, 1) - corner(0, 1)) * smooth(fx);
    top + (bottom - top) * smooth(fy)
}

/// `octaves` of value noise, each twice as fine and half as strong: from 0 to 1.
fn fbm(u: f64, v: f64, period: u32, octaves: u32, seed: u32) -> f64 {
    let (sum, weight) = (0..octaves).fold((0.0, 0.0), |(sum, weight), k| {
        let w = 0.5f64.powi(k as i32);
        (
            sum + w * value_noise(u, v, period << k, seed + k),
            weight + w,
        )
    });
    sum / weight
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::looks;

    #[test]
    fn graphics_words() {
        assert_eq!("pretty".parse(), Ok(Graphics::Pretty));
        assert_eq!("simple".parse(), Ok(Graphics::Simple));
        assert!("fancy".parse::<Graphics>().is_err());
        assert_eq!(Graphics::Pretty.to_string(), "pretty");
        assert_eq!(Graphics::for_adapter(true), Graphics::Simple);
        assert_eq!(Graphics::for_adapter(false), Graphics::Pretty);
    }

    /// The road world's names (road.sdf) and the test world's.
    #[test]
    fn surfaces_from_names() {
        let plain = |c| Look {
            colour: c,
            texture: None,
            normal_map: None,
        };
        let guess = |name, c| Surface::guess(name, &plain(c));
        assert_eq!(guess("road/tarmac", looks::TARMAC), Surface::Asphalt);
        assert_eq!(guess("road/left_line", looks::LINE), Surface::Paint);
        assert_eq!(guess("ground_plane/grass", looks::GRASS), Surface::Grass);
        assert_eq!(guess("ground", looks::GRASS), Surface::Grass);
        assert_eq!(guess("kerb", looks::CONCRETE), Surface::Concrete);
        assert_eq!(guess("stop_sign/post", looks::POST), Surface::Plain);
        assert_eq!(guess("box", looks::BRICK), Surface::Plain);
        // A shape with its own picture keeps it.
        let textured = Look {
            colour: looks::GRASS,
            texture: Some("grass.png".into()),
            normal_map: None,
        };
        assert_eq!(Surface::guess("campus/grass", &textured), Surface::Plain);
    }

    /// A detail picture keeps its surface's average colour: a check that reads colours sees
    /// the same road.
    #[test]
    fn detail_keeps_the_average() {
        for s in [
            Surface::Grass,
            Surface::Asphalt,
            Surface::Concrete,
            Surface::Paint,
            Surface::Brick,
        ] {
            let d = s.detail(64).unwrap();
            assert_eq!(d.rgba.len(), 64 * 64 * 4);
            assert!((d.mean() - 1.0).abs() < 0.01, "{s:?}: {}", d.mean());
            // Not flat: it has grain.
            let first = d.rgba[0];
            assert!(d.rgba.chunks_exact(4).any(|p| p[0] != first), "{s:?}");
        }
        assert_eq!(Surface::Plain.detail(64), None);
        // The same picture every run.
        assert_eq!(Surface::Grass.detail(32), Surface::Grass.detail(32));
    }

    /// Value noise repeats every picture, so the tiles meet without a seam.
    #[test]
    fn noise_tiles() {
        for v in [0.0, 0.3, 0.77] {
            let a = value_noise(0.0, v, 4, 1);
            let b = value_noise(1.0 - 1e-12, v, 4, 1);
            assert!((a - b).abs() < 1e-6);
        }
    }

    #[test]
    fn sky_is_paler_at_the_horizon_and_glows_at_the_sun() {
        let base = looks::SKY;
        let sun = Vec3::new(-0.5, 0.1, -0.9);
        let horizon = sky(Vec3::new(0.0, 1.0, 0.0), base, None);
        let up = sky(Vec3::new(0.0, 0.0, 1.0), base, None);
        assert_eq!(horizon, haze(base));
        // Overhead is bluer than the horizon: less red against its blue.
        assert!(up.r / up.b < horizon.r / horizon.b);
        // Below the horizon: the haze, so the ground's far edge melts into it.
        assert_eq!(sky(Vec3::new(1.0, 0.0, -0.3), base, None), haze(base));
        let at_sun = sky(-sun, base, Some(sun));
        let away = sky(Vec3::new(0.5, -0.1, 0.3), base, Some(sun));
        assert!(at_sun.r > away.r);
        assert!((at_sun.r - 1.0).abs() < 1e-6);
    }
}
