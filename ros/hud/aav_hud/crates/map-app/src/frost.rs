//! Frosted glass for the map window (`blur: true` in config/map.yaml): the boxes over
//! the map (the search, the keyboard, the chips) show the map behind them, blurred.
//!
//! Each frame, once the map is on the screen, the screen is copied into a texture
//! ([`Frost::capture`]: a framebuffer blit on the graphics card, so the map is drawn
//! only once). That is shrunk to a quarter and blurred in two passes, across then down,
//! twice ([`Frost::blur`]): a Gaussian, nine taps a pass. Each glass box then draws its
//! piece of the blurred map, cut to its rounded corners by a second small shader
//! ([`Frost::behind`]), before its see-through tint.
//!
//! If the graphics card refuses a shader (an old OpenGL), [`Frost::new`] says so and
//! the window keeps see-through boxes without the blur.

use anyhow::{bail, Result};
use raylib::prelude::*;

/// raylib's default vertex shader is used; these are the fragment shaders (GLSL 330,
/// raylib's desktop OpenGL 3.3).
///
/// One Gaussian pass: nine taps along `texel_step`, weights from the binomial row 16.
const BLUR_FS: &str = r#"#version 330
in vec2 fragTexCoord;
in vec4 fragColor;
uniform sampler2D texture0;
uniform vec2 texel_step;
out vec4 finalColor;
const float weight[5] = float[](0.2270270270, 0.1945945946, 0.1216216216, 0.0540540541, 0.0162162162);
void main() {
    vec3 sum = texture(texture0, fragTexCoord).rgb * weight[0];
    for (int i = 1; i < 5; i++) {
        vec2 off = texel_step * float(i);
        sum += texture(texture0, fragTexCoord + off).rgb * weight[i];
        sum += texture(texture0, fragTexCoord - off).rgb * weight[i];
    }
    finalColor = vec4(sum, 1.0);
}
"#;

/// The blurred map, cut to a rounded rectangle `size` points big, corners `radius`
/// round: `uv_a` and `uv_b` are its top-left and bottom-right corners in the texture.
const CUT_FS: &str = r#"#version 330
in vec2 fragTexCoord;
in vec4 fragColor;
uniform sampler2D texture0;
uniform vec2 uv_a;
uniform vec2 uv_b;
uniform vec2 size;
uniform float radius;
out vec4 finalColor;
void main() {
    vec2 p = (fragTexCoord - uv_a) / (uv_b - uv_a) * size;
    vec2 q = abs(p - size * 0.5) - (size * 0.5 - vec2(radius));
    float d = length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - radius;
    float inside = clamp(0.5 - d, 0.0, 1.0);
    finalColor = vec4(texture(texture0, fragTexCoord).rgb, inside);
}
"#;

/// How far apart the blur's taps are, in texels of the quarter-size picture.
const SPREAD: f32 = 1.6;
/// Across-then-down passes: twice is a soft frost.
const ROUNDS: usize = 2;

/// The blur's shaders and textures.
pub struct Frost {
    blur: Shader,
    blur_step: i32,
    cut: Shader,
    cut_uv_a: i32,
    cut_uv_b: i32,
    cut_size: i32,
    cut_radius: i32,
    /// The window's size in points, and in pixels (twice on a Retina screen).
    points: (i32, i32),
    pixels: (i32, i32),
    textures: Option<Textures>,
}

struct Textures {
    /// The screen, copied: as sharp as it is.
    sharp: RenderTexture2D,
    /// A quarter of its size: the passes go back and forth between these; `soft` ends
    /// blurred.
    soft: RenderTexture2D,
    spare: RenderTexture2D,
}

/// OpenGL's names, as rlgl.h has them.
const RL_READ_FRAMEBUFFER: u32 = 0x8CA8;
const RL_DRAW_FRAMEBUFFER: u32 = 0x8CA9;
const RL_FRAMEBUFFER: u32 = 0x8D40;
const GL_COLOR_BUFFER_BIT: i32 = 0x4000;

/// Whether raylib fell back to its default shader: the card refused ours.
fn refused(shader: &Shader) -> bool {
    // SAFETY: reads the id of the default shader raylib loaded with the window.
    let default = unsafe { raylib::ffi::rlGetShaderIdDefault() };
    shader.as_ref().id == default
}

impl Frost {
    /// Loads the shaders. Needs the window open. An error when the card refuses them.
    pub fn new(rl: &mut RaylibHandle, thread: &RaylibThread) -> Result<Self> {
        let blur = rl.load_shader_from_memory(thread, None, Some(BLUR_FS));
        let cut = rl.load_shader_from_memory(thread, None, Some(CUT_FS));
        if refused(&blur) || refused(&cut) {
            bail!("the graphics card refused the blur's shaders (GLSL 330)");
        }
        Ok(Self {
            blur_step: blur.get_shader_location("texel_step"),
            cut_uv_a: cut.get_shader_location("uv_a"),
            cut_uv_b: cut.get_shader_location("uv_b"),
            cut_size: cut.get_shader_location("size"),
            cut_radius: cut.get_shader_location("radius"),
            blur,
            cut,
            points: (0, 0),
            pixels: (0, 0),
            textures: None,
        })
    }

    /// Makes the textures for the window (again, when it changes size). Before drawing
    /// starts.
    pub fn fit(&mut self, rl: &mut RaylibHandle, thread: &RaylibThread) -> Result<()> {
        let points = (rl.get_screen_width(), rl.get_screen_height());
        let pixels = (rl.get_render_width(), rl.get_render_height());
        if self.textures.is_some() && self.points == points && self.pixels == pixels {
            return Ok(());
        }
        let (w, h) = (pixels.0.max(4) as u32, pixels.1.max(4) as u32);
        let mut make = |w: u32, h: u32| -> Result<RenderTexture2D> {
            let t = rl.load_render_texture(thread, w, h)?;
            t.texture()
                .set_texture_filter(thread, TextureFilter::TEXTURE_FILTER_BILINEAR);
            Ok(t)
        };
        self.textures = Some(Textures {
            sharp: make(w, h)?,
            soft: make(w / 4, h / 4)?,
            spare: make(w / 4, h / 4)?,
        });
        self.points = points;
        self.pixels = pixels;
        Ok(())
    }

    /// Copies what the screen shows now (the map) for the blur. Outside any scissor:
    /// the copy obeys it.
    pub fn capture(&mut self, _frame: &mut RaylibDrawHandle) {
        let Some(t) = self.textures.as_ref() else {
            return;
        };
        let (w, h) = self.pixels;
        // SAFETY: rlgl calls on the main thread, inside a frame (the handle proves it):
        // draw what is batched, copy the window's framebuffer (0) into ours, then bind
        // the window's again, as raylib expects. The two are the same size, as a copy
        // from a multisampled framebuffer needs.
        unsafe {
            raylib::ffi::rlDrawRenderBatchActive();
            raylib::ffi::rlBindFramebuffer(RL_READ_FRAMEBUFFER, 0);
            raylib::ffi::rlBindFramebuffer(RL_DRAW_FRAMEBUFFER, t.sharp.id());
            raylib::ffi::rlBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT);
            raylib::ffi::rlBindFramebuffer(RL_FRAMEBUFFER, 0);
        }
    }

    /// Blurs what [`Frost::capture`] copied.
    pub fn blur(&mut self, d: &mut RaylibDrawHandle, thread: &RaylibThread) {
        let Some(t) = self.textures.as_mut() else {
            return;
        };
        let (sw, sh) = (
            t.soft.texture().width as f32,
            t.soft.texture().height as f32,
        );
        let whole = |tex: &WeakTexture2D| {
            let (w, h) = (tex.width as f32, tex.height as f32);
            // A render texture is stored upside down: a negative height flips it back.
            Rectangle::new(0.0, 0.0, w, -h)
        };
        let dest = Rectangle::new(0.0, 0.0, sw, sh);
        let mut pass =
            |from: &WeakTexture2D, to: &mut RenderTexture2D, step: Vector2, shader: &mut Shader| {
                shader.set_shader_value(self.blur_step, step);
                let mut m = d.begin_texture_mode(thread, to.as_mut());
                let mut s = m.begin_shader_mode(shader);
                s.draw_texture_pro(from, whole(from), dest, Vector2::zero(), 0.0, Color::WHITE);
            };
        let across = Vector2::new(SPREAD / sw, 0.0);
        let down = Vector2::new(0.0, SPREAD / sh);
        // The first pass also shrinks the sharp copy to a quarter: its taps as far apart.
        pass(t.sharp.texture(), &mut t.spare, across, &mut self.blur);
        pass(t.spare.texture(), &mut t.soft, down, &mut self.blur);
        for _ in 1..ROUNDS {
            pass(t.soft.texture(), &mut t.spare, across, &mut self.blur);
            pass(t.spare.texture(), &mut t.soft, down, &mut self.blur);
        }
    }

    /// Draws the blurred map behind a glass box `r` (window points), corners `radius`
    /// round. Its tint goes on top after.
    pub fn behind(&mut self, d: &mut impl RaylibDraw, r: Rectangle, radius: f32) {
        let Some(t) = self.textures.as_ref() else {
            return;
        };
        let soft = t.soft.texture();
        let (sw, sh) = (soft.width as f32, soft.height as f32);
        // Window points to the quarter-size texture.
        let k = sw / self.points.0.max(1) as f32;
        let src = Rectangle::new(
            r.x * k,
            sh - (r.y + r.height) * k,
            r.width * k,
            -r.height * k,
        );
        let uv_a = Vector2::new(r.x * k / sw, 1.0 - r.y * k / sh);
        let uv_b = Vector2::new((r.x + r.width) * k / sw, 1.0 - (r.y + r.height) * k / sh);
        let radius = radius.min(r.width.min(r.height) / 2.0);
        self.cut.set_shader_value(self.cut_uv_a, uv_a);
        self.cut.set_shader_value(self.cut_uv_b, uv_b);
        self.cut
            .set_shader_value(self.cut_size, Vector2::new(r.width, r.height));
        self.cut.set_shader_value(self.cut_radius, radius);
        let mut s = d.begin_shader_mode(&mut self.cut);
        s.draw_texture_pro(soft, src, r, Vector2::zero(), 0.0, Color::WHITE);
    }
}
