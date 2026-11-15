//! Bevy draws the simulator's cameras: [`BevyRenderer`], a [`CameraRenderer`] with no window.
//!
//! One offscreen Bevy camera per rig camera, each into its own picture, read back from the
//! GPU. The app is stepped by hand, only inside [`CameraRenderer::render`], so nothing is
//! drawn between pictures and a camera not due costs nothing. Bevy keeps ROS's axes (z up),
//! as the developer window does: only each camera turns into Bevy's camera axes.
//!
//! Which adapter draws: a GPU if there is one, else a software one (Mesa's lavapipe, through
//! Vulkan: `tools/software_gl.sh` on Linux). [`Adapter::Software`] or
//! `WGPU_FORCE_FALLBACK_ADAPTER=1` asks for the software one even beside a GPU.
//! [`BevyRenderer::adapter`] says which one it got.
//!
//! Lighting: no tone mapping, no dithering. A colour in the world (linear, as Gazebo's) comes
//! out times the light, `ambient + sun.strength * cos(angle to the sun)`, then into sRGB.

pub mod scene;

use std::{
    fmt,
    panic::{self, AssertUnwindSafe},
    time::Duration,
};

use bevy::{
    app::PluginsState,
    asset::RenderAssetUsages,
    camera::{visibility::RenderLayers, Exposure, ImageRenderTarget, RenderTarget},
    core_pipeline::tonemapping::{DebandDither, Tonemapping},
    prelude::*,
    render::{
        gpu_readback::{Readback, ReadbackComplete},
        pipelined_rendering::PipelinedRenderingPlugin,
        render_resource::{PipelineCache, TextureFormat, TextureUsages},
        renderer::RenderAdapterInfo,
        settings::{RenderCreation, WgpuSettings},
        RenderApp, RenderPlugin,
    },
    window::ExitCondition,
};
use camera_core::{
    frame::{padded_row_bytes, rgba_rows_to_rgb, ShortCopy},
    CameraRenderer, Frame, Pose, Rig, SimWorld,
};

pub use camera_core;

/// The car's body is drawn on this layer: only cameras with `sees_car` look at it.
const CAR_LAYER: usize = 1;
/// The most app steps one picture may take to come back from the GPU.
const MAX_STEPS: usize = 120;
/// The most pictures the start may draw before every shader is ready.
const WARM_UP_RENDERS: usize = 200;

/// Why a picture can't be drawn.
#[derive(Debug)]
pub enum RenderError {
    /// Bevy found nothing to draw with: no GPU, and no software renderer either (on a Linux
    /// computer with no GPU, run it under `tools/software_gl.sh`: Mesa's lavapipe).
    NoGpu(String),
    /// The GPU didn't hand the pictures back in time.
    Timeout {
        steps: usize,
    },
    /// The shaders weren't ready after the start's warm-up.
    WarmUp,
    Copy(ShortCopy),
    /// A shape's picture couldn't be read.
    Texture(String),
    /// A shape's OBJ file couldn't be read.
    Mesh(String),
}

impl fmt::Display for RenderError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoGpu(why) => write!(
                f,
                "no GPU to draw with, and no software renderer: {why} \
                 (with no GPU on Linux: tools/software_gl.sh)"
            ),
            Self::Timeout { steps } => {
                write!(f, "no picture back from the GPU after {steps} steps")
            }
            Self::WarmUp => write!(f, "the shaders weren't ready after the warm-up"),
            Self::Copy(e) => write!(f, "{e}"),
            Self::Texture(e) => write!(f, "a shape's picture: {e}"),
            Self::Mesh(e) => write!(f, "a shape's mesh: {e}"),
        }
    }
}

impl std::error::Error for RenderError {}

/// Which adapter to ask Bevy for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Adapter {
    /// A GPU if there is one, else a software renderer if one is installed. The default.
    #[default]
    Any,
    /// A software renderer only (wgpu's fallback adapter: Mesa's lavapipe), even beside a
    /// GPU: to try CI's way on a computer with a GPU.
    Software,
}

/// The adapter Bevy drew with, for the log.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AdapterUsed {
    /// The adapter's own name, such as "Apple M4 Max" or "llvmpipe (LLVM 20.1.8, 256 bits)".
    pub name: String,
    /// Metal, Vulkan, Gl...
    pub backend: String,
    /// Drawn by the CPU, not a GPU: slow.
    pub software: bool,
}

impl fmt::Display for AdapterUsed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let kind = if self.software {
            "software, on the CPU"
        } else {
            "GPU"
        };
        write!(f, "{} ({}, {kind})", self.name, self.backend)
    }
}

/// One rig camera inside Bevy.
struct Slot {
    entity: Entity,
    image: Handle<Image>,
    mount: Pose,
    width: u32,
    height: u32,
}

/// Pictures back from the GPU, by camera index.
#[derive(Resource, Default)]
struct Arrived(Vec<(usize, Vec<u8>)>);

/// A readback in flight, for camera `0`.
#[derive(Component)]
struct Shot(usize);

/// The car's body: moved to the car's pose with each picture.
#[derive(Component)]
struct CarRoot;

/// Bevy's camera axes (x right, y up, looking along -z) in a camera link (x ahead, z up).
fn bevy_camera_in_link() -> camera_core::Rotation {
    use camera_core::Vec3;
    camera_core::Rotation::from_axes(
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(-1.0, 0.0, 0.0),
    )
}

/// A pose in ROS's axes as Bevy's transform (the same axes: no conversion but the types).
pub fn transform(pose: &Pose) -> Transform {
    let t = pose.translation;
    let [x, y, z, w] = pose.rotation.to_quaternion();
    Transform {
        translation: Vec3::new(t.x as f32, t.y as f32, t.z as f32),
        rotation: Quat::from_xyzw(x as f32, y as f32, z as f32, w as f32).normalize(),
        scale: Vec3::ONE,
    }
}

/// The renderer. Not `Send` in spirit: keep it on one thread (Bevy's own tasks run inside).
pub struct BevyRenderer {
    app: App,
    slots: Vec<Slot>,
    /// Everything spawned for the world, to replace it.
    world_entities: Vec<Entity>,
    adapter: AdapterUsed,
}

impl BevyRenderer {
    /// Starts Bevy with no window, adds one offscreen camera per rig camera and the world,
    /// and draws until every shader is ready. Any adapter: a GPU if there is one.
    pub fn new(rig: &Rig, world: &SimWorld) -> Result<Self, RenderError> {
        Self::with_adapter(rig, world, Adapter::Any)
    }

    /// As [`BevyRenderer::new`], on the adapter asked for.
    pub fn with_adapter(
        rig: &Rig,
        world: &SimWorld,
        adapter: Adapter,
    ) -> Result<Self, RenderError> {
        let app = panic::catch_unwind(AssertUnwindSafe(|| start_app(adapter))).map_err(|e| {
            let why = e
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| e.downcast_ref::<&str>().map(|s| (*s).to_owned()))
                .unwrap_or_else(|| "Bevy stopped while starting".to_owned());
            RenderError::NoGpu(why)
        })?;
        let adapter = app
            .world()
            .get_resource::<RenderAdapterInfo>()
            .map(|info| AdapterUsed {
                name: info.name.clone(),
                backend: format!("{:?}", info.backend),
                software: info.device_type == wgpu::DeviceType::Cpu,
            })
            .ok_or_else(|| RenderError::NoGpu("Bevy started with no adapter".to_owned()))?;
        let mut renderer = Self {
            app,
            slots: Vec::new(),
            world_entities: Vec::new(),
            adapter,
        };
        renderer.add_cameras(rig);
        renderer.set_world(world)?;
        renderer.warm_up()?;
        Ok(renderer)
    }

    /// The adapter it draws with: say it in the log, so a slow picture has its reason.
    pub fn adapter(&self) -> &AdapterUsed {
        &self.adapter
    }

    fn add_cameras(&mut self, rig: &Rig) {
        let world = self.app.world_mut();
        for (i, spec) in rig.cameras.iter().enumerate() {
            let (w, h) = (spec.lens.width(), spec.lens.height());
            let mut target = Image::new_target_texture(w, h, TextureFormat::Rgba8UnormSrgb, None);
            target.texture_descriptor.usage |= TextureUsages::COPY_SRC;
            target.asset_usage = RenderAssetUsages::RENDER_WORLD;
            let image = world.resource_mut::<Assets<Image>>().add(target);
            let layers = if spec.sees_car {
                RenderLayers::from_layers(&[0, CAR_LAYER])
            } else {
                RenderLayers::layer(0)
            };
            let entity = world
                .spawn((
                    Camera3d::default(),
                    Camera {
                        is_active: false,
                        order: i as isize,
                        ..default()
                    },
                    RenderTarget::Image(ImageRenderTarget::from(image.clone())),
                    Projection::Perspective(PerspectiveProjection {
                        fov: spec.lens.vertical_fov() as f32,
                        aspect_ratio: w as f32 / h as f32,
                        near: spec.near as f32,
                        far: spec.far as f32,
                        ..default()
                    }),
                    Exposure::BLENDER,
                    Tonemapping::None,
                    DebandDither::Disabled,
                    Msaa::Sample4,
                    layers,
                    Transform::default(),
                ))
                .id();
            self.slots.push(Slot {
                entity,
                image,
                mount: spec.mount,
                width: w,
                height: h,
            });
        }
    }

    /// Draws every camera until the shaders are all compiled and two pictures in a row
    /// match: on a Mac, Bevy compiles them in the background, and a shape whose shader
    /// isn't ready yet is simply missing from the picture.
    fn warm_up(&mut self) -> Result<(), RenderError> {
        let all: Vec<usize> = (0..self.slots.len()).collect();
        let car = Pose::IDENTITY;
        let mut last: Option<Vec<Frame>> = None;
        for _ in 0..WARM_UP_RENDERS {
            let frames = self.render(&car, Duration::ZERO, &all)?;
            let waiting = self
                .app
                .sub_app(RenderApp)
                .world()
                .get_resource::<PipelineCache>()
                .map_or(0, |cache| cache.waiting_pipelines().count());
            if waiting == 0 && last.as_ref() == Some(&frames) {
                return Ok(());
            }
            last = Some(frames);
        }
        Err(RenderError::WarmUp)
    }
}

fn start_app(adapter: Adapter) -> App {
    // Bevy's own defaults (and its WGPU_* variables), with the software renderer if asked.
    let settings = WgpuSettings {
        force_fallback_adapter: adapter == Adapter::Software,
        // Mesa's lavapipe lists cooperative-matrix shapes but not the feature itself, so
        // wgpu 30 offers it, Bevy asks for every feature offered, and Vulkan refuses the
        // device (VK_ERROR_FEATURE_NOT_PRESENT, 15 Nov). Nothing here uses it.
        disabled_features: Some(wgpu::Features::EXPERIMENTAL_COOPERATIVE_MATRIX),
        ..default()
    };
    let mut app = App::new();
    app.add_plugins(
        DefaultPlugins
            .set(RenderPlugin {
                render_creation: RenderCreation::Automatic(Box::new(settings)),
                ..default()
            })
            .set(WindowPlugin {
                primary_window: None,
                exit_condition: ExitCondition::DontExit,
                close_when_requested: false,
                ..default()
            })
            // Draw in the same step: a picture comes back sooner, and nothing renders
            // between our steps.
            .disable::<PipelinedRenderingPlugin>(),
    );
    app.init_resource::<Arrived>();
    app.add_observer(
        |event: On<ReadbackComplete>,
         shots: Query<&Shot>,
         mut arrived: ResMut<Arrived>,
         mut commands: Commands| {
            if let Ok(shot) = shots.get(event.entity) {
                arrived.0.push((shot.0, event.data.clone()));
                commands.entity(event.entity).despawn();
            }
        },
    );
    while app.plugins_state() == PluginsState::Adding {
        bevy::tasks::tick_global_task_pools_on_main_thread();
    }
    app.finish();
    app.cleanup();
    app.update();
    app
}

impl CameraRenderer for BevyRenderer {
    type Error = RenderError;

    fn set_world(&mut self, world: &SimWorld) -> Result<(), RenderError> {
        let ecs = self.app.world_mut();
        for e in self.world_entities.drain(..) {
            if let Ok(entity) = ecs.get_entity_mut(e) {
                entity.despawn();
            }
        }
        self.world_entities = scene::spawn(ecs, world, CAR_LAYER)?;
        Ok(())
    }

    fn render(
        &mut self,
        car: &Pose,
        stamp: Duration,
        which: &[usize],
    ) -> Result<Vec<Frame>, RenderError> {
        let car_tf = transform(car);
        let to_bevy = Pose::new(camera_core::Vec3::ZERO, bevy_camera_in_link());
        {
            let ecs = self.app.world_mut();
            let mut roots = ecs.query_filtered::<&mut Transform, With<CarRoot>>();
            for mut t in roots.iter_mut(ecs) {
                *t = car_tf;
            }
            for (i, slot) in self.slots.iter().enumerate() {
                let due = which.contains(&i);
                let pose = car.then(&slot.mount).then(&to_bevy);
                if let Ok(mut e) = ecs.get_entity_mut(slot.entity) {
                    if let Some(mut cam) = e.get_mut::<Camera>() {
                        cam.is_active = due;
                    }
                    if let Some(mut t) = e.get_mut::<Transform>() {
                        *t = transform(&pose);
                    }
                }
                if due {
                    // Not ReadbackOnce: it is removed at the start of the next step, before
                    // the render world sees it, when spawned from outside a system.
                    ecs.spawn((Readback::texture(slot.image.clone()), Shot(i)));
                }
            }
            ecs.resource_mut::<Arrived>().0.clear();
        }
        let mut steps = 0;
        while self.app.world().resource::<Arrived>().0.len() < which.len() {
            if steps == MAX_STEPS {
                return Err(RenderError::Timeout { steps });
            }
            self.app.update();
            steps += 1;
            if steps == 1 {
                // That step drew and copied each picture once: no more drawing or copies
                // while the copies come back.
                let ecs = self.app.world_mut();
                for slot in &self.slots {
                    if let Some(mut cam) = ecs.get_mut::<Camera>(slot.entity) {
                        cam.is_active = false;
                    }
                }
                let shots: Vec<Entity> = ecs
                    .query_filtered::<Entity, (With<Shot>, With<Readback>)>()
                    .iter(ecs)
                    .collect();
                for e in shots {
                    ecs.entity_mut(e).remove::<Readback>();
                }
            }
        }
        let arrived = std::mem::take(&mut self.app.world_mut().resource_mut::<Arrived>().0);
        which
            .iter()
            .map(|&i| {
                let slot = &self.slots[i];
                let data = arrived
                    .iter()
                    .find(|(c, _)| *c == i)
                    .map(|(_, d)| d.as_slice())
                    .unwrap_or_default();
                let rgb =
                    rgba_rows_to_rgb(data, slot.width, slot.height, padded_row_bytes(slot.width))
                        .map_err(RenderError::Copy)?;
                Ok(Frame {
                    camera: i,
                    stamp,
                    width: slot.width,
                    height: slot.height,
                    rgb,
                })
            })
            .collect()
    }
}
