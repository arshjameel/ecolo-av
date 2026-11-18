//! Which program decodes the stream, and its command line (`decoders.py`).
//!
//! ```text
//!  decoder      runs                                   where
//!  auto         the best this computer has (choose())  the default
//!  ffmpeg       ffmpeg, in software (or hwaccel)       any Mac or Linux, Pixi's own ffmpeg
//!  gst-nvidia   gst-launch-1.0 with nvv4l2decoder      the Orin: NVIDIA's video hardware
//!  gst-soft     gst-launch-1.0 with avdec_h264         any computer with GStreamer's plugins
//!  gst-mac      gst-launch-1.0 with vtdec              a Mac: VideoToolbox, Apple's hardware
//!  opencv       OpenCV's VideoCapture, in-process      the Python node only
//! ```
//!
//! Every one here is a child program writing raw BGR pictures (width x height x 3 bytes)
//! to a pipe ([`crate::pipe`]), so the node links no GStreamer or FFmpeg: on the Orin it
//! runs JetPack's `/usr/bin/gst-launch-1.0`, whose NVIDIA plugins Pixi doesn't have.

use std::collections::BTreeMap;
use std::fmt;
use std::path::{Component, Path, PathBuf};
use std::str::FromStr;

use crate::{Error, Result};

/// The decoders, by the `decoder` parameter's names.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decoder {
    Auto,
    Ffmpeg,
    GstNvidia,
    GstSoft,
    GstMac,
    OpenCv,
}

impl Decoder {
    pub const ALL: [Decoder; 6] = [
        Decoder::Auto,
        Decoder::Ffmpeg,
        Decoder::GstNvidia,
        Decoder::GstSoft,
        Decoder::GstMac,
        Decoder::OpenCv,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Decoder::Auto => "auto",
            Decoder::Ffmpeg => "ffmpeg",
            Decoder::GstNvidia => "gst-nvidia",
            Decoder::GstSoft => "gst-soft",
            Decoder::GstMac => "gst-mac",
            Decoder::OpenCv => "opencv",
        }
    }

    /// The GStreamer preset's pipeline: rtspsrc pulls (latency=0: no jitter buffer), the
    /// depayloader and parser hand H.264 to a decoder, then the picture is scaled and
    /// turned into BGR. Any other decoder gets gst-soft's, as the Python's `GST.get`.
    pub fn preset(self) -> &'static str {
        match self {
            // nvvidconv converts and scales in NVIDIA's hardware too; disable-dpb: the
            // stream has no B-frames (baseline), so nothing waits to be reordered.
            Decoder::GstNvidia => {
                "rtspsrc location={url} protocols={protocols} latency=0 \
                 ! rtph264depay ! h264parse \
                 ! nvv4l2decoder enable-max-performance=true disable-dpb=true \
                 ! nvvidconv ! video/x-raw,format=BGRx,width={width},height={height} \
                 ! videoconvert ! video/x-raw,format=BGR ! fdsink sync=false"
            }
            Decoder::GstMac => {
                "rtspsrc location={url} protocols={protocols} latency=0 \
                 ! rtph264depay ! h264parse ! vtdec \
                 ! videoconvert ! videoscale \
                 ! video/x-raw,format=BGR,width={width},height={height} \
                 ! fdsink sync=false"
            }
            _ => {
                "rtspsrc location={url} protocols={protocols} latency=0 \
                 ! rtph264depay ! h264parse ! avdec_h264 max-threads=1 \
                 ! videoconvert ! videoscale \
                 ! video/x-raw,format=BGR,width={width},height={height} \
                 ! fdsink sync=false"
            }
        }
    }

    /// What the preset needs, checked with `gst-inspect-1.0 --exists` before auto picks it.
    pub fn elements(self) -> &'static [&'static str] {
        match self {
            Decoder::GstMac => &[
                "rtspsrc",
                "rtph264depay",
                "h264parse",
                "vtdec",
                "videoconvert",
                "videoscale",
                "fdsink",
            ],
            _ => &[
                "rtspsrc",
                "rtph264depay",
                "h264parse",
                "avdec_h264",
                "videoconvert",
                "videoscale",
                "fdsink",
            ],
        }
    }
}

impl fmt::Display for Decoder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

impl FromStr for Decoder {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self> {
        Decoder::ALL
            .into_iter()
            .find(|d| d.name() == s)
            .ok_or_else(|| {
                let names: Vec<&str> = Decoder::ALL.iter().map(|d| d.name()).collect();
                Error::Invalid(format!("decoder:={s}: use one of {}", names.join(", ")))
            })
    }
}

/// RTSP over TCP or UDP.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Transport {
    Tcp,
    Udp,
}

impl Transport {
    pub fn name(self) -> &'static str {
        match self {
            Transport::Tcp => "tcp",
            Transport::Udp => "udp",
        }
    }
}

impl FromStr for Transport {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self> {
        match s {
            "tcp" => Ok(Transport::Tcp),
            "udp" => Ok(Transport::Udp),
            _ => Err(Error::Invalid(format!("transport:={s}: use tcp or udp"))),
        }
    }
}

/// JetPack's own GStreamer, not Pixi's (which has no NVIDIA plugins).
pub const SYSTEM_GST: &str = "/usr/bin/gst-launch-1.0";
/// The gst environment's gst-launch-1.0, under the workspace's root.
pub const PIXI_GST: &str = ".pixi/envs/gst/bin/gst-launch-1.0";
/// A Mac's hardware can be missing (a virtual machine): three small test pictures through
/// VideoToolbox and vtdec, as the stream's would be.
pub const VTDEC_TEST: &str = "videotestsrc num-buffers=3 ! video/x-raw,width=64,height=64 \
     ! vtenc_h264 ! h264parse ! vtdec ! videoconvert ! video/x-raw,format=BGR ! fakesink";
pub const FFMPEG_SLOWER: &str = "ffmpeg holds back about 2 frames: 135 ms more delay";
/// What a program from outside Pixi must not inherit from it: Pixi's libraries and plugins.
pub const PIXI_ONLY: [&str; 12] = [
    "LD_LIBRARY_PATH",
    "DYLD_LIBRARY_PATH",
    "DYLD_FALLBACK_LIBRARY_PATH",
    "GST_PLUGIN_PATH",
    "GST_PLUGIN_PATH_1_0",
    "GST_PLUGIN_SYSTEM_PATH",
    "GST_PLUGIN_SYSTEM_PATH_1_0",
    "GST_PLUGIN_SCANNER",
    "GST_PLUGIN_SCANNER_1_0",
    "GST_REGISTRY",
    "GST_REGISTRY_1_0",
    "GIO_MODULE_DIR",
];

/// The decoder and transport, or what to pick.
pub fn check(decoder: &str, transport: &str) -> Result<(Decoder, Transport)> {
    Ok((decoder.parse()?, transport.parse()?))
}

/// What the stream is and how big its pictures come out.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Stream {
    pub url: String,
    pub width: u32,
    pub height: u32,
    pub transport: Transport,
}

/// The argv of an ffmpeg writing the pictures to stdout, as raw BGR.
pub fn ffmpeg_command(stream: &Stream, program: &str, hwaccel: &str) -> Vec<String> {
    let program = if program.is_empty() {
        "ffmpeg"
    } else {
        program
    };
    let mut argv: Vec<String> = [
        program,
        "-hide_banner",
        "-loglevel",
        "error",
        "-nostdin",
        // Low delay: no input buffer, no waiting to guess the format, slice threads only
        // (frame threads hold back one frame per thread).
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-probesize",
        "32768",
        "-analyzeduration",
        "0",
        "-thread_type",
        "slice",
    ]
    .map(String::from)
    .to_vec();
    if !hwaccel.is_empty() {
        argv.extend(["-hwaccel".to_string(), hwaccel.to_string()]);
    }
    argv.extend(
        [
            "-rtsp_transport",
            stream.transport.name(),
            "-i",
            &stream.url,
            "-an",
            // Each decoded picture once: no frames repeated or dropped to keep a rate.
            "-fps_mode",
            "passthrough",
            "-vf",
            &format!("scale={}:{}", stream.width, stream.height),
            "-pix_fmt",
            "bgr24",
            "-f",
            "rawvideo",
            "-",
        ]
        .map(String::from),
    );
    argv
}

/// Python's `str.format` for the four names a pipeline may hold: `{url}`, `{width}`,
/// `{height}`, `{protocols}`; `{{` and `}}` are braces.
pub fn fill(pipeline: &str, stream: &Stream) -> Result<String> {
    let mut out = String::with_capacity(pipeline.len() + 64);
    let mut chars = pipeline.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '{' if chars.peek() == Some(&'{') => {
                chars.next();
                out.push('{');
            }
            '}' if chars.peek() == Some(&'}') => {
                chars.next();
                out.push('}');
            }
            '{' => {
                let name: String = chars.by_ref().take_while(|&c| c != '}').collect();
                match name.as_str() {
                    "url" => out.push_str(&stream.url),
                    "width" => out.push_str(&stream.width.to_string()),
                    "height" => out.push_str(&stream.height.to_string()),
                    "protocols" => out.push_str(stream.transport.name()),
                    _ => {
                        return Err(Error::Invalid(format!(
                            "pipeline: {{{name}}} is not one of {{url}}, {{width}}, {{height}}, \
                             {{protocols}}"
                        )))
                    }
                }
            }
            '}' => {
                return Err(Error::Invalid(
                    "pipeline: a single '}' (write '}}')".to_string(),
                ))
            }
            _ => out.push(c),
        }
    }
    Ok(out)
}

/// Python's `shlex.split`: words split at spaces, quotes and backslashes as a POSIX
/// shell has them.
pub fn shlex_split(text: &str) -> Result<Vec<String>> {
    #[derive(PartialEq)]
    enum In {
        Space,
        Word,
        Single,
        Double,
    }
    let mut words = Vec::new();
    let mut word = String::new();
    let mut state = In::Space;
    let mut chars = text.chars();
    while let Some(c) = chars.next() {
        match state {
            In::Space | In::Word => match c {
                c if c.is_whitespace() => {
                    if state == In::Word {
                        words.push(std::mem::take(&mut word));
                    }
                    state = In::Space;
                }
                '\'' => state = In::Single,
                '"' => state = In::Double,
                '\\' => {
                    let next = chars
                        .next()
                        .ok_or_else(|| Error::Invalid("No escaped character".to_string()))?;
                    word.push(next);
                    state = In::Word;
                }
                c => {
                    word.push(c);
                    state = In::Word;
                }
            },
            In::Single => match c {
                '\'' => state = In::Word,
                c => word.push(c),
            },
            In::Double => match c {
                '"' => state = In::Word,
                '\\' => {
                    let next = chars
                        .next()
                        .ok_or_else(|| Error::Invalid("No closing quotation".to_string()))?;
                    if next != '"' && next != '\\' {
                        word.push('\\');
                    }
                    word.push(next);
                }
                c => word.push(c),
            },
        }
    }
    match state {
        In::Single | In::Double => Err(Error::Invalid("No closing quotation".to_string())),
        In::Word => {
            words.push(word);
            Ok(words)
        }
        In::Space => Ok(words),
    }
}

/// The argv of a quiet gst-launch-1.0 running this pipeline, filled in. `-q`: its progress
/// goes to stdout, where the pictures go.
pub fn gst_command(pipeline: &str, stream: &Stream, program: &str) -> Result<Vec<String>> {
    let program = if program.is_empty() {
        "gst-launch-1.0"
    } else {
        program
    };
    let mut argv = vec![program.to_string(), "-q".to_string()];
    argv.extend(shlex_split(&fill(pipeline, stream)?)?);
    Ok(argv)
}

/// The child program's argv for a pipe decoder (every one but opencv; auto is chosen
/// first).
pub fn command(
    decoder: Decoder,
    stream: &Stream,
    program: &str,
    pipeline: &str,
    hwaccel: &str,
) -> Result<Vec<String>> {
    match decoder {
        Decoder::OpenCv => Err(Error::Invalid(
            "decoder:=opencv decodes in this process: it has no command".to_string(),
        )),
        Decoder::Auto => Err(Error::Invalid(
            "decoder:=auto: choose() picks the decoder first".to_string(),
        )),
        Decoder::Ffmpeg if pipeline.is_empty() => Ok(ffmpeg_command(stream, program, hwaccel)),
        _ => {
            let program = if decoder == Decoder::GstNvidia && program.is_empty() {
                SYSTEM_GST
            } else {
                program
            };
            let pipeline = if pipeline.is_empty() {
                decoder.preset()
            } else {
                pipeline
            };
            gst_command(pipeline, stream, program)
        }
    }
}

/// Whether a file can be run.
fn executable(path: &Path) -> bool {
    use std::os::unix::fs::PermissionsExt;
    path.metadata()
        .is_ok_and(|m| m.is_file() && m.permissions().mode() & 0o111 != 0)
}

/// The gst environment's gst-launch-1.0 under the workspace root, or "".
pub fn pixi_gst(root: &str) -> String {
    if root.is_empty() {
        return String::new();
    }
    let path = Path::new(root).join(PIXI_GST);
    if executable(&path) {
        path.to_string_lossy().into_owned()
    } else {
        String::new()
    }
}

/// What the node runs: the decoder, its program ("" for the preset's), and a warning.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Choice {
    pub decoder: Decoder,
    pub program: String,
    pub warning: String,
}

/// The computers `choose` tells apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Platform {
    Mac,
    Other,
}

impl Platform {
    pub fn this() -> Self {
        if cfg!(target_os = "macos") {
            Platform::Mac
        } else {
            Platform::Other
        }
    }
}

/// The decoder the node runs.
///
/// auto picks GStreamer from the gst environment under `root` (or `program`): on a Mac
/// gst-mac (vtdec) if it decodes, else gst-soft (avdec_h264); elsewhere gst-soft. With
/// no GStreamer that works, ffmpeg, with a warning. gst-mac and gst-soft given with no
/// program run the gst environment's, if it's there. `works(program, decoder)` says why
/// that GStreamer can't run that decoder (Err), or Ok.
pub fn choose(
    decoder: Decoder,
    program: &str,
    root: &str,
    platform: Platform,
    mut works: impl FnMut(&str, Decoder) -> std::result::Result<(), String>,
) -> Choice {
    let gst = if program.is_empty() {
        pixi_gst(root)
    } else {
        program.to_string()
    };
    let choice = |decoder, program: &str, warning: String| Choice {
        decoder,
        program: program.to_string(),
        warning,
    };
    match decoder {
        Decoder::GstMac | Decoder::GstSoft => return choice(decoder, &gst, String::new()),
        Decoder::Auto => {}
        other => return choice(other, program, String::new()),
    }
    if gst.is_empty() {
        return choice(
            Decoder::Ffmpeg,
            "",
            format!(
                "decoder auto: no GStreamer, so ffmpeg ({FFMPEG_SLOWER}). On a Mac or a \
                 Linux PC, install it once: pixi install -e gst"
            ),
        );
    }
    let why = match platform {
        Platform::Mac => works(&gst, Decoder::GstMac).err(),
        Platform::Other => None,
    };
    if platform == Platform::Mac && why.is_none() {
        return choice(Decoder::GstMac, &gst, String::new());
    }
    match works(&gst, Decoder::GstSoft) {
        Ok(()) => choice(
            Decoder::GstSoft,
            &gst,
            why.map(|w| format!("decoder auto: {w}; avdec_h264 (software) instead"))
                .unwrap_or_default(),
        ),
        Err(soft) => {
            let reasons: Vec<String> = why
                .into_iter()
                .chain([soft])
                .filter(|w| !w.is_empty())
                .collect();
            choice(
                Decoder::Ffmpeg,
                "",
                format!(
                    "decoder auto: GStreamer can't decode here ({}), so ffmpeg ({FFMPEG_SLOWER})",
                    reasons.join("; ")
                ),
            )
        }
    }
}

/// Runs a program to its end; Ok(true) if it exited with 0.
pub trait Runner {
    fn run(&mut self, argv: &[String], env: &Env) -> std::io::Result<bool>;
}

/// An environment for a child program: every variable, by name.
pub type Env = BTreeMap<String, String>;

/// Whether this gst-launch-1.0 can run the decoder's preset: Ok, or why not.
///
/// Every element must exist (gst-inspect-1.0 --exists, beside the program); for gst-mac,
/// vtdec must also decode VTDEC_TEST's pictures, at the first or the second try (once on
/// 24 Oct the first try failed and the next passed).
pub fn gst_works(
    program: &str,
    decoder: Decoder,
    env: &Env,
    runner: &mut impl Runner,
) -> std::result::Result<(), String> {
    let folder = Path::new(program)
        .parent()
        .map(|p| p.to_string_lossy().into_owned())
        .unwrap_or_default();
    let inspect = Path::new(&folder)
        .join("gst-inspect-1.0")
        .to_string_lossy()
        .into_owned();
    let failed = |e: std::io::Error| format!("{program}: {e}");
    for element in decoder.elements() {
        let argv = [inspect.clone(), "--exists".to_string(), element.to_string()];
        if !runner.run(&argv, env).map_err(failed)? {
            return Err(format!("no {element} in {folder}"));
        }
    }
    if decoder == Decoder::GstMac {
        let mut argv = vec![program.to_string(), "-q".to_string()];
        argv.extend(shlex_split(VTDEC_TEST).map_err(|e| e.to_string())?);
        let mut decoded = false;
        for _ in 0..2 {
            if runner.run(&argv, env).map_err(failed)? {
                decoded = true;
                break;
            }
        }
        if !decoded {
            return Err(
                "vtdec could not decode a test picture (no VideoToolbox here?)".to_string(),
            );
        }
    }
    Ok(())
}

/// Runs programs for real, each given 20 s, as the Python's `subprocess.run(timeout=20)`.
#[derive(Debug, Default)]
pub struct SystemRunner;

impl Runner for SystemRunner {
    fn run(&mut self, argv: &[String], env: &Env) -> std::io::Result<bool> {
        use std::process::{Command, Stdio};
        use std::time::{Duration, Instant};
        let (program, args) = argv
            .split_first()
            .ok_or_else(|| std::io::Error::other("no program"))?;
        let mut child = Command::new(program)
            .args(args)
            .env_clear()
            .envs(env)
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()?;
        let start = Instant::now();
        loop {
            if let Some(status) = child.try_wait()? {
                return Ok(status.success());
            }
            if start.elapsed() > Duration::from_secs(20) {
                child.kill().ok();
                child.wait().ok();
                return Err(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    "timed out after 20 seconds",
                ));
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }
}

/// Python's `os.path.abspath`, without the file system: made absolute against `cwd`, with
/// `.`, `..` and repeated slashes taken out.
fn abspath(path: &str, cwd: &Path) -> PathBuf {
    let joined = cwd.join(path);
    let mut out = PathBuf::from("/");
    for part in joined.components() {
        match part {
            Component::ParentDir => {
                out.pop();
            }
            Component::Normal(p) => out.push(p),
            _ => {}
        }
    }
    out
}

/// The environment for the decoder program.
///
/// One from inside this Pixi environment (`prefix`) gets it all. One from outside, such as
/// JetPack's /usr/bin/gst-launch-1.0, loses Pixi's library and plugin paths, so it loads
/// its own system's.
pub fn child_env(environ: &Env, program: &str, prefix: &str) -> Env {
    let mut env = environ.clone();
    if prefix.is_empty() || !program.starts_with('/') {
        return env;
    }
    let cwd = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/"));
    let path = abspath(program, &cwd);
    let mut inside = abspath(prefix, &cwd).into_os_string();
    if inside != "/" {
        inside.push("/");
    }
    if !path
        .to_string_lossy()
        .starts_with(&*inside.to_string_lossy())
    {
        for name in PIXI_ONLY {
            env.remove(name);
        }
    }
    env
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stream() -> Stream {
        Stream {
            url: "rtsp://h:8554/front_left".into(),
            width: 1280,
            height: 720,
            transport: Transport::Tcp,
        }
    }

    #[test]
    fn the_orin_runs_jetpacks_gstreamer() {
        let argv = command(Decoder::GstNvidia, &stream(), "", "", "").unwrap_or_default();
        assert_eq!(argv[0], SYSTEM_GST);
        assert!(argv.contains(&"nvv4l2decoder".to_string()));
    }

    #[test]
    fn a_pipeline_with_an_unknown_name_is_refused() {
        assert!(fill("{nope}", &stream()).is_err());
        assert_eq!(fill("{{a}} {width}", &stream()), Ok("{a} 1280".to_string()));
    }

    #[test]
    fn opencv_and_auto_have_no_command() {
        assert!(command(Decoder::OpenCv, &stream(), "", "", "").is_err());
        assert!(command(Decoder::Auto, &stream(), "", "", "").is_err());
    }
}
