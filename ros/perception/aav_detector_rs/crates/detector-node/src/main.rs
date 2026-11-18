//! detector: RF-DETR's ONNX model finds objects, such as stop signs, in camera images, in
//! Rust. A drop-in for aav_detector's Python node (detector.launch.py detector_impl:=rust):
//! the same name, topics, QoS, parameters and messages. A challenger, not the default.
//!
//! Subscribes  image              sensor_msgs/Image, rgb8 or bgr8 (the launch file maps it
//!                                to /sensors/camera/front_left/image_raw), reliable,
//!                                keep last 1: a slow model drops old pictures
//!             extra_images       more cameras for the same model, comma-separated topics
//! Publishes   detections         vision_msgs/Detection2DArray (/perception/detections), the
//!                                picture's header; each box's centre and size in pixels,
//!                                one hypothesis: its COCO label and score
//!             detections/image   sensor_msgs/Image, the picture with each box outlined in
//!                                red (no text, unlike the Python's), its own encoding;
//!                                reliable, depth 3
//! Parameters  detector (rfdetr only), model (an .onnx; empty = models/rf-detr-nano.onnx,
//!             from $PIXI_PROJECT_ROOT or here), device (auto, cpu, coreml or mps, cuda,
//!             tensorrt: ONNX Runtime's provider), confidence (0.5), labels (empty keeps
//!             all), image_size (unused: RF-DETR's is fixed), publish_image (true),
//!             compressed (false only: JPEG needs the Python), extra_images (empty).
//!
//! No serving, no calls, never TF. The model and its maths are detector-core, with no ROS.
//! Logs the frame rate every 10 s, as the Python does.

// conda's linker (pixi's) warns about every unaligned label in ONNX Runtime's prebuilt
// KleidiAI kernels: harmless (Apple's linker is quiet), and thousands of lines.
#![allow(linker_messages)]

use std::{
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{bail, Context as _, Result};
use detector_core::{normal_label, parse_labels, Canvas, Detector, Picture, PixelOrder, Provider};
use rclrs::*;
use ros_env::sensor_msgs::msg::Image;
use ros_env::vision_msgs::msg::{
    BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesis, ObjectHypothesisWithPose,
    Point2D, Pose2D,
};

/// How often the frame rate is logged, and a repeated warning may be.
const EVERY: Duration = Duration::from_secs(10);
const WARN_EVERY: Duration = Duration::from_secs(5);
const DEFAULT_MODEL: &str = "models/rf-detr-nano.onnx";

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("object_detector")?;
    let params = Params::declare(&node)?;
    let detector = params.detector.get();
    if detector.trim().to_lowercase() != "rfdetr" {
        bail!("detector:={detector}: the Rust detector runs RF-DETR only; YOLO is the Python's");
    }
    if params.compressed.get() {
        bail!("compressed:=true: the Rust detector reads raw pictures only; use the Python's");
    }
    let model = model_path(&params.model.get())?;
    let provider = Provider::from_device(&params.device.get())?;
    let start = Instant::now();
    let detector = Detector::open(&model, provider)
        .with_context(|| format!("{}: the ONNX model", model.display()))?;
    let labels = parse_labels(&params.labels.get());
    let confidence = params.confidence.get() as f32;
    let extra: Vec<String> = params
        .extra_images
        .get()
        .split(',')
        .map(str::trim)
        .filter(|t| !t.is_empty())
        .map(str::to_owned)
        .collect();
    log_info!(
        node.logger(),
        "rfdetr (Rust, ONNX Runtime): {} on {provider}, loaded in {:.1} s; reading image{}; \
         keeping {}, at confidence {confidence} or more",
        model.display(),
        start.elapsed().as_secs_f64(),
        if extra.is_empty() {
            String::new()
        } else {
            format!(", {}", extra.join(", "))
        },
        labels
            .as_ref()
            .map_or("every label".into(), |l| l.join(", "))
    );

    let state = State {
        detector,
        labels,
        confidence,
        logger: node.logger().clone(),
        detections: node.create_publisher("detections".keep_last(10).reliable())?,
        image: if params.publish_image.get() {
            Some(node.create_publisher("detections/image".keep_last(3).reliable())?)
        } else {
            None
        },
        frames: 0,
        busy: Duration::ZERO,
        since: Instant::now(),
    };
    let worker = node.create_worker(state);
    let topics: Vec<String> = std::iter::once("image".to_owned()).chain(extra).collect();
    let _subscriptions = topics
        .iter()
        .map(|topic| {
            worker.create_subscription(
                topic.as_str().keep_last(1).reliable(),
                |s: &mut State, msg: Image| s.on_image(msg),
            )
        })
        .collect::<Result<Vec<_>, _>>()?;

    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The model file: an absolute path, or one from the repository's root ($PIXI_PROJECT_ROOT,
/// else the working folder), as the Python finds its models.
fn model_path(model: &str) -> Result<PathBuf> {
    let model = if model.trim().is_empty() {
        DEFAULT_MODEL
    } else {
        model.trim()
    };
    let path = match model.strip_prefix("~/") {
        Some(rest) => PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(rest),
        None => PathBuf::from(model),
    };
    let root = std::env::var_os("PIXI_PROJECT_ROOT")
        .map(PathBuf::from)
        .map_or_else(std::env::current_dir, Ok)?;
    let path = root.join(path);
    if path.extension().is_none_or(|x| x != "onnx") {
        bail!(
            "model:={model}: the Rust detector reads ONNX (.onnx): \
             tools/export_model.py --format onnx makes it from the .pth"
        );
    }
    if !path.exists() {
        bail!(
            "{} is missing: pixi run get models, then pixi run -e ml python3 \
             tools/export_model.py --format onnx (docs/onnx.md)",
            path.display()
        );
    }
    Ok(path)
}

/// The worker's payload.
struct State {
    detector: Detector,
    labels: Option<Vec<String>>,
    confidence: f32,
    logger: Logger,
    detections: Publisher<Detection2DArray>,
    image: Option<Publisher<Image>>,
    frames: u32,
    busy: Duration,
    since: Instant,
}

impl State {
    fn on_image(&mut self, mut msg: Image) {
        let start = Instant::now();
        let order = match msg.encoding.as_str() {
            "rgb8" => PixelOrder::Rgb,
            "bgr8" => PixelOrder::Bgr,
            other => {
                log_warn!(
                    self.logger.throttle(WARN_EVERY),
                    "a {other} picture: the Rust detector reads rgb8 and bgr8"
                );
                return;
            }
        };
        let (width, height, stride) = (msg.width as usize, msg.height as usize, msg.step as usize);
        let found = Picture::new(&msg.data, width, height, stride, order)
            .map_err(anyhow::Error::from)
            .and_then(|picture| Ok(self.detector.detect(&picture, self.confidence)?));
        let found = match found {
            Ok(found) => found,
            Err(e) => {
                log_warn!(self.logger.throttle(WARN_EVERY), "detection failed: {e}");
                return;
            }
        };
        let kept: Vec<_> = found
            .into_iter()
            .map(|d| (normal_label(&d.class.to_string()), d))
            .filter(|(label, _)| self.labels.as_ref().is_none_or(|keep| keep.contains(label)))
            .collect();
        let out = Detection2DArray {
            header: msg.header.clone(),
            detections: kept
                .iter()
                .map(|(label, d)| {
                    let (cx, cy) = d.corners.centre();
                    Detection2D {
                        header: msg.header.clone(),
                        bbox: BoundingBox2D {
                            center: Pose2D {
                                position: Point2D {
                                    x: f64::from(cx),
                                    y: f64::from(cy),
                                },
                                theta: 0.0,
                            },
                            size_x: f64::from(d.corners.width()),
                            size_y: f64::from(d.corners.height()),
                        },
                        results: vec![ObjectHypothesisWithPose {
                            hypothesis: ObjectHypothesis {
                                class_id: label.clone(),
                                score: f64::from(d.score),
                            },
                            ..Default::default()
                        }],
                        ..Default::default()
                    }
                })
                .collect(),
        };
        if let Err(e) = self.detections.publish(out) {
            log_warn!(self.logger.throttle(WARN_EVERY), "detections: {e}");
        }
        if let Some(publisher) = &self.image {
            let mut canvas = Canvas {
                data: &mut msg.data,
                width,
                height,
                stride,
                order,
            };
            for (_, d) in &kept {
                canvas.outline(&d.corners, 2);
            }
            if let Err(e) = publisher.publish(msg) {
                log_warn!(self.logger.throttle(WARN_EVERY), "detections/image: {e}");
            }
        }
        self.frames += 1;
        self.busy += start.elapsed();
        if self.since.elapsed() >= EVERY {
            log_info!(
                &self.logger,
                "{} frames in 10 s, {:.1} ms each",
                self.frames,
                self.busy.as_secs_f64() * 1000.0 / f64::from(self.frames)
            );
            self.frames = 0;
            self.busy = Duration::ZERO;
            self.since = Instant::now();
        }
    }
}

/// The Python node's parameters, the same names and defaults.
struct Params {
    detector: ReadOnlyParameter<Arc<str>>,
    model: ReadOnlyParameter<Arc<str>>,
    device: ReadOnlyParameter<Arc<str>>,
    confidence: ReadOnlyParameter<f64>,
    labels: ReadOnlyParameter<Arc<str>>,
    publish_image: ReadOnlyParameter<bool>,
    compressed: ReadOnlyParameter<bool>,
    extra_images: ReadOnlyParameter<Arc<str>>,
    _image_size: ReadOnlyParameter<i64>,
}

impl Params {
    fn declare(node: &Node) -> Result<Self> {
        let text = |name: &str, default: &str| -> Result<ReadOnlyParameter<Arc<str>>> {
            node.declare_parameter(name)
                .default(Arc::from(default))
                .read_only()
                .with_context(|| format!("parameter {name} (text)"))
        };
        Ok(Self {
            detector: text("detector", "rfdetr")?,
            model: text("model", "")?,
            device: text("device", "auto")?,
            confidence: node
                .declare_parameter("confidence")
                .default(0.5)
                .read_only()
                .context("parameter confidence (0 to 1)")?,
            labels: text("labels", "")?,
            publish_image: node
                .declare_parameter("publish_image")
                .default(true)
                .read_only()
                .context("parameter publish_image")?,
            compressed: node
                .declare_parameter("compressed")
                .default(false)
                .read_only()
                .context("parameter compressed")?,
            extra_images: text("extra_images", "")?,
            _image_size: node
                .declare_parameter("image_size")
                .default(640)
                .read_only()
                .context("parameter image_size")?,
        })
    }
}
