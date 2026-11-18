//! camera_stream: one camera's H.264 stream (RTSP, from MediaMTX) to pictures and their
//! (empty) detections, in Rust. A drop-in for aav_camera_stream's Python node, run with
//! `detector:=none`: the same name, topics, QoS, parameters and diagnostics.
//!
//! ```text
//!   camera Pi: MediaMTX ──RTSP H.264──> decoder ──> undistort ──> detections (none)
//!                                      (child)     (stream-core)
//! ```
//!
//! Subscribes  nothing
//! Publishes   /perception/detections               vision_msgs/Detection2DArray: one per
//!                                                  picture, no boxes (detector none); the
//!                                                  camera's frame. The sensor watchdog
//!                                                  watches it
//!             /sensors/camera/<camera>/camera_info sensor_msgs/CameraInfo: the undistorted
//!                                                  picture's lens, with each picture
//!                                                  (publish_camera_info, with a calibration)
//!             /perception/camera/<camera>/preview/compressed  sensor_msgs/CompressedImage:
//!                                                  a small JPEG every preview_period s
//!             /sensors/camera/<camera>/image_raw   sensor_msgs/Image, bgr8: every
//!                                                  undistorted picture (publish_image)
//!             /diagnostics                         diagnostic_msgs/DiagnosticArray, 1 Hz:
//!                                                  rates, drops, times, measured delays
//!             /system/source                       aav_interfaces/DataSource, latched
//! Parameters  the Python's: camera, url, frame_id, decoder (auto, ffmpeg, gst-nvidia,
//!             gst-soft, gst-mac; opencv is the Python's), pipeline, program, hwaccel,
//!             transport, width, height, fps, calibration, alpha, delay, detector (none
//!             only: the models are the Python's), model, device, confidence, labels,
//!             image_size, preview_period, preview_width, publish_image,
//!             publish_camera_info, time_code, source.
//!
//! No serving, no calls. Stamp: a picture's arrival minus `delay` (s). The logic is the
//! `stream-core` crate, with no ROS.

mod params;

use std::collections::BTreeMap;
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::{bail, Context as _, Result};
use params::Declarer;
use rclrs::*;
use ros_env::aav_interfaces::msg::{DataSource, SensorSource};
use ros_env::builtin_interfaces::msg::Time;
use ros_env::diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
use ros_env::sensor_msgs::msg::{CameraInfo, CompressedImage, Image};
use ros_env::std_msgs::msg::Header;
use ros_env::vision_msgs::msg::Detection2DArray;
use stream_core::decoders::{self, Decoder, Env, Platform, Stream, SystemRunner};
use stream_core::latest::{stamp_ns, Delays, Latest, Rate};
use stream_core::lens::{self, Lens, Undistorter};
use stream_core::models::Detector;
use stream_core::pipe::PipeSource;
use stream_core::reader::{wall_ns, Arrival, Reader, Retry};
use stream_core::{preview, timecode};

const ERROR_EVERY: Duration = Duration::from_secs(5);

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("camera_stream")?;
    let mut stream = build(&node)?;
    // Ctrl-C, or launch stopping us: stop spinning, then the decoder, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    stream.shutdown();
    Ok(())
}

/// The frame each camera's pictures are in, by default.
fn frame_of(camera: &str) -> String {
    match camera {
        "front_left" => "camera_left_link".into(),
        "front_right" => "camera_right_link".into(),
        "rear" => "camera_rear_link".into(),
        other => format!("camera_{other}_link"),
    }
}

/// /system/source's value for the source parameter.
fn source_of(source: &str) -> Result<u8> {
    Ok(match source {
        "real" => SensorSource::REAL,
        "sim" => SensorSource::SIM,
        "bench" => SensorSource::BENCH,
        "replay" => SensorSource::REPLAY,
        _ => bail!("source:={source}: use one of real, sim, bench, replay"),
    })
}

fn to_time(ns: i64) -> Time {
    Time {
        sec: i32::try_from(ns.div_euclid(1_000_000_000)).unwrap_or(i32::MAX),
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    }
}

fn camera_info_msg(header: &Header, lens: &Lens) -> CameraInfo {
    let mut k = [0.0; 9];
    k.iter_mut().zip(&lens.k).for_each(|(k, v)| *k = *v);
    CameraInfo {
        header: header.clone(),
        width: lens.width,
        height: lens.height,
        distortion_model: "plumb_bob".into(),
        d: vec![0.0; 5],
        k,
        r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p: lens::projection(lens),
        ..CameraInfo::default()
    }
}

/// The numbers the worker keeps and the report reads.
struct Stats {
    start: Instant,
    in_rate: Mutex<Rate>,
    out_rate: Mutex<Rate>,
    work_ms: Mutex<Delays>,
    detect_ms: Mutex<Delays>,
    /// Measured delays (time_code), the last minute: to the picture out of the decoder,
    /// and to its detections published.
    arrival: Mutex<Delays>,
    done: Mutex<Delays>,
    codes: AtomicU64,
}

impl Stats {
    fn new(fps: f64) -> Self {
        let minute = (60.0 * fps).max(10.0) as usize;
        Stats {
            start: Instant::now(),
            in_rate: Mutex::default(),
            out_rate: Mutex::default(),
            work_ms: Mutex::new(Delays::new(300)),
            detect_ms: Mutex::new(Delays::new(300)),
            arrival: Mutex::new(Delays::new(minute)),
            done: Mutex::new(Delays::new(minute)),
            codes: AtomicU64::new(0),
        }
    }

    /// Monotonic seconds, for the rates.
    fn now(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }
}

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// What the worker thread needs to turn a picture into messages.
struct Worker {
    camera: String,
    frame_id: String,
    width: u32,
    height: u32,
    delay: f64,
    time_code: bool,
    undistort: Option<Undistorter>,
    preview_period: f64,
    preview_width: u32,
    last_preview: Option<Instant>,
    clock: Clock,
    stats: Arc<Stats>,
    det_pub: Publisher<Detection2DArray>,
    info_pub: Option<Publisher<CameraInfo>>,
    preview_pub: Option<Publisher<CompressedImage>>,
    image_pub: Option<Publisher<Image>>,
    logger: Logger,
}

impl Worker {
    fn run(mut self, latest: &Latest<Arrival>, stopping: &AtomicBool) {
        while !stopping.load(Ordering::Relaxed) {
            let Some((frame, arrived_ns)) = latest.take(Duration::from_millis(500)) else {
                continue;
            };
            let now = self.stats.now();
            lock(&self.stats.in_rate).tick(now);
            if let Err(e) = self.process(frame, arrived_ns) {
                // A bad frame must not stop the node.
                log_error!(self.logger.throttle(ERROR_EVERY), "{}: {e}", self.camera);
            }
        }
    }

    fn process(&mut self, frame: Vec<u8>, arrived_ns: i64) -> Result<()> {
        let t0 = Instant::now();
        let sent = if self.time_code {
            timecode::read(&timecode::Frame {
                data: &frame,
                width: self.width as usize,
                height: self.height as usize,
                channels: 3,
            })
        } else {
            None
        };
        if let Some(sent) = sent {
            self.stats.codes.fetch_add(1, Ordering::Relaxed);
            lock(&self.stats.arrival).add(timecode::age_s(sent, arrived_ns as f64 / 1e9));
        }
        let frame = match &self.undistort {
            Some(u) => u.apply(&frame)?,
            None => frame,
        };
        // No model here (detector none): no boxes, and no time spent on them.
        let t1 = Instant::now();
        lock(&self.stats.detect_ms).add(1000.0 * t1.elapsed().as_secs_f64());
        let header = Header {
            stamp: to_time(stamp_ns(
                arrived_ns,
                self.clock.now().nsec,
                wall_ns(),
                self.delay,
            )),
            frame_id: self.frame_id.clone(),
        };
        self.det_pub.publish(Detection2DArray {
            header: header.clone(),
            detections: Vec::new(),
        })?;
        if let Some(sent) = sent {
            lock(&self.stats.done).add(timecode::age_s(sent, wall_ns() as f64 / 1e9));
        }
        if let (Some(publisher), Some(u)) = (&self.info_pub, &self.undistort) {
            publisher.publish(camera_info_msg(&header, u.lens()))?;
        }
        if let Some(publisher) = &self.preview_pub {
            let due = self
                .last_preview
                .is_none_or(|t| t0.duration_since(t).as_secs_f64() >= self.preview_period);
            if due {
                self.last_preview = Some(t0);
                let data = preview::jpeg(&frame, self.width, self.height, self.preview_width)?;
                publisher.publish(CompressedImage {
                    header: header.clone(),
                    format: "jpeg".into(),
                    data,
                })?;
            }
        }
        if let Some(publisher) = &self.image_pub {
            publisher.publish(Image {
                header,
                height: self.height,
                width: self.width,
                encoding: "bgr8".into(),
                is_bigendian: 0,
                step: 3 * self.width,
                data: frame,
            })?;
        }
        let now = self.stats.now();
        lock(&self.stats.out_rate).tick(now);
        lock(&self.stats.work_ms).add(1000.0 * t0.elapsed().as_secs_f64());
        Ok(())
    }
}

/// The running stream: what `main` stops at the end.
struct Running {
    reader: Arc<Mutex<Reader>>,
    stopping: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
    _keep: Vec<Box<dyn std::any::Any>>,
}

impl Running {
    fn shutdown(&mut self) {
        self.stopping.store(true, Ordering::Relaxed);
        lock(&self.reader).stop();
        if let Some(worker) = self.worker.take() {
            // The worker wakes every 0.5 s to look at `stopping`.
            worker.join().ok();
        }
    }
}

/// The diagnostics' values, in the Python's order and format.
fn report(
    stats: &Stats,
    reader: &Mutex<Reader>,
    latest: &Latest<Arrival>,
    decoder: Decoder,
    delay: f64,
    time_code: bool,
) -> (bool, f64, Vec<(String, String)>) {
    let (connected, starts, frames) = {
        let r = lock(reader);
        (r.connected(), r.starts(), r.frames())
    };
    let now = stats.now();
    let fps_in = lock(&stats.in_rate).hz(now);
    let fps_out = lock(&stats.out_rate).hz(now);
    let mut values: Vec<(String, String)> = [
        ("connected", connected.to_string()),
        ("decoder", decoder.name().to_string()),
        ("decoder_starts", starts.to_string()),
        ("frames_in", frames.to_string()),
        ("fps_in", format!("{fps_in:.1}")),
        ("fps_out", format!("{fps_out:.1}")),
        ("dropped", latest.dropped().to_string()),
        ("delay_s", format!("{delay:.3}")),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v))
    .collect();
    for (name, delays) in [("work_ms", &stats.work_ms), ("detect_ms", &stats.detect_ms)] {
        if let Some(s) = lock(delays).summary() {
            values.push((name.into(), format!("{:.1}", s.median)));
        }
    }
    if time_code {
        values.push((
            "time_codes".into(),
            stats.codes.load(Ordering::Relaxed).to_string(),
        ));
        for (name, delays) in [("arrival", &stats.arrival), ("detection", &stats.done)] {
            if let Some(s) = lock(delays).summary() {
                for (key, v) in [("median", s.median), ("p95", s.p95), ("max", s.max)] {
                    values.push((format!("{name}_{key}_ms"), format!("{:.1}", 1000.0 * v)));
                }
            }
        }
    }
    (connected, fps_out, values)
}

fn build(node: &Node) -> Result<Running> {
    let mut d = Declarer::new(node);
    let camera = d.text("camera", "front_left")?;
    let url = d.text("url", &format!("rtsp://127.0.0.1:8554/{camera}"))?;
    let frame_id = d.text("frame_id", &frame_of(&camera))?;
    let decoder = d.text("decoder", "auto")?;
    let pipeline = d.text("pipeline", "")?;
    let program = d.text("program", "")?;
    let hwaccel = d.text("hwaccel", "")?;
    let transport = d.text("transport", "tcp")?;
    let width = d.count("width", 1280)?;
    let height = d.count("height", 720)?;
    let expected_hz = d.number("fps", 15.0)?;
    let calibration = d.text("calibration", "")?;
    let alpha = d.number("alpha", 0.0)?;
    let delay = d.number("delay", 0.06)?;
    let detector = d.text("detector", "rfdetr")?;
    // The model's settings: declared so a launch file can set them; only the Python's
    // models read them.
    let _model = d.text("model", "")?;
    let _device = d.text("device", "auto")?;
    let _confidence = d.number("confidence", 0.5)?;
    let _labels = d.text("labels", "")?;
    let _image_size = d.count("image_size", 640)?;
    let preview_period = d.number("preview_period", 1.0)?;
    let preview_width = d.count("preview_width", 320)?;
    let publish_image = d.flag("publish_image", false)?;
    let publish_info = d.flag("publish_camera_info", true)?;
    let time_code = d.flag("time_code", false)?;
    let source = d.text("source", "real")?;
    let source_value = source_of(&source)?;
    let what = |e: stream_core::Error| anyhow::anyhow!("camera_stream {camera}: {e}");
    let (decoder, transport) = decoders::check(&decoder, &transport).map_err(what)?;
    if decoder == Decoder::OpenCv {
        bail!(
            "camera_stream {camera}: decoder:=opencv decodes inside the Python node: \
             camera_stream_impl:=python, or another decoder"
        );
    }
    let kind: Detector = detector.parse().map_err(what)?;
    if kind != Detector::None {
        bail!(
            "camera_stream {camera}: detector:={kind} runs aav_detector's model, in the Python \
             node: camera_stream_impl:=python, or detector:=none"
        );
    }
    if width == 0 || height == 0 {
        bail!("camera_stream {camera}: width and height must be more than 0");
    }
    let undistort = if calibration.is_empty() {
        None
    } else {
        let lens = lens::load(Path::new(&calibration))
            .and_then(|l| lens::scaled(&l, width, height))
            .and_then(|l| Undistorter::new(&l, alpha))
            .map_err(what)?;
        Some(lens)
    };
    let logger = node.logger().clone();
    let environ: Env = std::env::vars().collect();
    let prefix = std::env::var("CONDA_PREFIX").unwrap_or_default();
    let root = std::env::var("PIXI_PROJECT_ROOT").unwrap_or_else(|_| {
        std::env::current_dir()
            .map(|p| p.to_string_lossy().into_owned())
            .unwrap_or_default()
    });
    let (decoder, program) =
        if matches!(decoder, Decoder::Auto | Decoder::GstMac | Decoder::GstSoft) {
            let choice =
                decoders::choose(decoder, &program, &root, Platform::this(), |gst, preset| {
                    let env = decoders::child_env(&environ, gst, &prefix);
                    decoders::gst_works(gst, preset, &env, &mut SystemRunner)
                });
            if !choice.warning.is_empty() {
                log_warn!(&logger, "{}", choice.warning);
            }
            (choice.decoder, choice.program)
        } else {
            (decoder, program)
        };
    let stream = Stream {
        url: url.clone(),
        width,
        height,
        transport,
    };
    let argv = decoders::command(decoder, &stream, &program, &pipeline, &hwaccel).map_err(what)?;
    let env: BTreeMap<String, String> = decoders::child_env(&environ, &argv[0], &prefix);
    log_info!(&logger, "decoder: {}", argv.join(" "));

    let base = format!("/sensors/camera/{camera}");
    let det_pub =
        node.create_publisher::<Detection2DArray>("/perception/detections".keep_last(10))?;
    let info_pub = match (&undistort, publish_info) {
        (Some(_), true) => Some(node.create_publisher::<CameraInfo>(
            format!("{base}/camera_info").as_str().keep_last(10),
        )?),
        _ => None,
    };
    let preview_pub = if preview_period > 0.0 {
        Some(
            node.create_publisher::<CompressedImage>(
                format!("/perception/camera/{camera}/preview/compressed")
                    .as_str()
                    .keep_last(2),
            )?,
        )
    } else {
        None
    };
    let image_pub = if publish_image {
        Some(node.create_publisher::<Image>(format!("{base}/image_raw").as_str().keep_last(2))?)
    } else {
        None
    };
    let diag_pub = node.create_publisher::<DiagnosticArray>("/diagnostics".keep_last(10))?;
    let announce = node.create_publisher::<DataSource>(
        "/system/source".keep_last(1).reliable().transient_local(),
    )?;
    announce.publish(DataSource {
        header: Header {
            stamp: to_time(node.get_clock().now().nsec),
            frame_id: String::new(),
        },
        announcer: node.name(),
        sensors: vec![SensorSource {
            name: format!("camera_{camera}"),
            topic: if publish_image {
                format!("{base}/image_raw")
            } else {
                String::new()
            },
            source: source_value,
        }],
    })?;

    let latest: Arc<Latest<Arrival>> = Arc::new(Latest::default());
    let stats = Arc::new(Stats::new(expected_hz));
    let log = logger.clone();
    let name = camera.clone();
    let make_source = move || PipeSource::new(argv.clone(), width, height, env.clone());
    let reader = Reader::start(
        make_source,
        Arc::clone(&latest),
        Retry::default(),
        move |text| {
            log_info!(&log, "{name} {text}");
        },
    )
    .context("the reader thread")?;
    let reader = Arc::new(Mutex::new(reader));

    let undistorted_by = if calibration.is_empty() {
        "not undistorted".to_string()
    } else {
        format!("undistorted by {calibration}")
    };
    let worker = Worker {
        camera: camera.clone(),
        frame_id: frame_id.clone(),
        width,
        height,
        delay,
        time_code,
        undistort,
        preview_period,
        preview_width,
        last_preview: None,
        clock: node.get_clock(),
        stats: Arc::clone(&stats),
        det_pub,
        info_pub,
        preview_pub,
        image_pub,
        logger: logger.clone(),
    };
    let stopping = Arc::new(AtomicBool::new(false));
    let worker = {
        let latest = Arc::clone(&latest);
        let stopping = Arc::clone(&stopping);
        std::thread::Builder::new()
            .name("stream-worker".into())
            .spawn(move || worker.run(&latest, &stopping))
            .context("the worker thread")?
    };

    let timer = {
        let (reader, stats, latest) =
            (Arc::clone(&reader), Arc::clone(&stats), Arc::clone(&latest));
        let clock = node.get_clock();
        let status_name = format!("camera_stream: {camera}");
        let url = url.clone();
        node.create_timer_repeating(Duration::from_secs(1).node_time(), move || {
            let (connected, fps, values) =
                report(&stats, &reader, &latest, decoder, delay, time_code);
            let (level, message) = if !connected {
                (DiagnosticStatus::ERROR, "no pictures".to_string())
            } else if fps < 0.5 * expected_hz {
                (DiagnosticStatus::WARN, format!("slow: {fps:.1} fps"))
            } else {
                (DiagnosticStatus::OK, format!("{fps:.1} fps"))
            };
            let status = DiagnosticStatus {
                level,
                name: status_name.clone(),
                message,
                hardware_id: url.clone(),
                values: values
                    .into_iter()
                    .map(|(key, value)| KeyValue { key, value })
                    .collect(),
            };
            let out = DiagnosticArray {
                header: Header {
                    stamp: to_time(clock.now().nsec),
                    frame_id: String::new(),
                },
                status: vec![status],
            };
            diag_pub.publish(out).ok();
        })?
    };
    log_info!(
        &logger,
        "{camera}: {url} by {decoder}, {width}x{height}, {undistorted_by}; detector none \
         (Rust); stamps = arrival - {delay:.3} s; frame {frame_id}"
    );
    let mut keep: Vec<Box<dyn std::any::Any>> = vec![Box::new(timer), Box::new(announce)];
    keep.extend(d.finish());
    Ok(Running {
        reader,
        stopping,
        worker: Some(worker),
        _keep: keep,
    })
}
