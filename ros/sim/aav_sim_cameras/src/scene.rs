//! A [`SimWorld`] as Bevy entities: meshes, materials, the sky, the sun.
//!
//! [`Graphics::Simple`] draws as Gazebo did: plain colours, a flat sky, the sun's shadows.
//! [`Graphics::Pretty`] adds (`camera_core::pretty`): grain on the ground (a small picture
//! made in code, laid in metres), each surface's own roughness, the campus's bump maps, a
//! sky with haze and the sun's glow (a skybox), distance fog in the haze's colour, a warm
//! sun and a blue-grey ambient, and shadows further out.

use std::f32::consts::FRAC_PI_2;

use bevy::image::{ImageAddressMode, ImageSampler, ImageSamplerDescriptor};
use bevy::{
    asset::RenderAssetUsages,
    camera::{visibility::RenderLayers, Exposure},
    light::{CascadeShadowConfigBuilder, DirectionalLightShadowMap, Skybox},
    math::Affine2,
    mesh::{Indices, PrimitiveTopology, VertexAttributeValues},
    pbr::{DistanceFog, FogFalloff},
    prelude::*,
    render::render_resource::{
        Extent3d, TextureDimension, TextureFormat, TextureViewDescriptor, TextureViewDimension,
    },
};
use camera_core::{pretty, Geometry, Graphics, Look, Shape, SimWorld, Surface, TriMesh};
use std::path::Path;

use crate::{transform, CarRoot, RenderError};

/// The exposure every camera uses (Bevy's default, Blender's), as a multiplier.
fn exposure() -> f32 {
    Exposure::BLENDER.exposure()
}

/// Pixels across a detail picture and a sky face: small, cheap even in software.
const DETAIL_PIXELS: u32 = 256;
const SKY_PIXELS: u32 = 64;
/// Haze, m: none nearer than the start (a stop sign, the lane lines and anything the
/// detector reads keep their colours), then more with distance, a fifth at the car
/// cameras' 150 m edge, so it melts into the sky. Exponential fog washed out dark bricks
/// 40 m away (sRGB's dark end shows a few percent of haze).
const FOG_START: f32 = 50.0;
const FOG_END: f32 = 600.0;

/// Spawns the world; returns every entity, to remove them when the world changes.
/// `cameras` get the sky and fog with [`Graphics::Pretty`] (and lose them with simple).
pub(crate) fn spawn(
    ecs: &mut World,
    world: &SimWorld,
    car_layer: usize,
    graphics: Graphics,
    cameras: &[Entity],
) -> Result<Vec<Entity>, RenderError> {
    let pretty = graphics == Graphics::Pretty;
    let sky = Color::linear_rgb(world.sky.r, world.sky.g, world.sky.b);
    ecs.insert_resource(ClearColor(sky));
    // Ambient: a white surface comes out `ambient` grey (Bevy's ambient is in nits,
    // scaled by the camera's exposure). Pretty: a little blue, as light from the sky is.
    ecs.insert_resource(GlobalAmbientLight {
        color: if pretty {
            Color::linear_rgb(0.9, 0.96, 1.08)
        } else {
            Color::WHITE
        },
        brightness: world.ambient / exposure(),
        ..default()
    });
    // Each shadow cascade's picture, in pixels: sharper shadows when pretty.
    ecs.insert_resource(DirectionalLightShadowMap {
        size: if pretty { 4096 } else { 2048 },
    });
    sky_and_fog(ecs, world, pretty, cameras);
    let mut out = Vec::new();
    if let Some(sun) = world.sun {
        // Lambert's diffuse is albedo / pi times the light: so lux = strength * pi / exposure.
        let lux = sun.strength * std::f32::consts::PI / exposure();
        let d = sun.direction;
        let dir = Vec3::new(d.x as f32, d.y as f32, d.z as f32).normalize_or(Vec3::NEG_Z);
        // Simple: two cascades out to 60 m: shadows cost the GPU about half its time with
        // Bevy's four out to 150 m (measured on an M4 Max). Pretty: three, out to 120 m.
        let cascades = if pretty {
            CascadeShadowConfigBuilder {
                num_cascades: 3,
                first_cascade_far_bound: 12.0,
                maximum_distance: 120.0,
                ..default()
            }
        } else {
            CascadeShadowConfigBuilder {
                num_cascades: 2,
                first_cascade_far_bound: 15.0,
                maximum_distance: 60.0,
                ..default()
            }
        };
        let id = ecs
            .spawn((
                DirectionalLight {
                    // Pretty: an afternoon sun, a little warm.
                    color: if pretty {
                        Color::linear_rgb(1.0, 0.95, 0.86)
                    } else {
                        Color::WHITE
                    },
                    illuminance: lux,
                    shadow_maps_enabled: true,
                    ..default()
                },
                cascades.build(),
                // A directional light shines along its -z.
                Transform::default().looking_to(dir, Vec3::Z),
                RenderLayers::from_layers(&[0, car_layer]),
            ))
            .id();
        out.push(id);
    }
    for shape in &world.shapes {
        out.push(spawn_shape(
            ecs,
            shape,
            RenderLayers::layer(0),
            None,
            graphics,
        )?);
    }
    if !world.car.is_empty() {
        let root = ecs
            .spawn((CarRoot, Transform::default(), Visibility::default()))
            .id();
        out.push(root);
        for shape in &world.car {
            spawn_shape(
                ecs,
                shape,
                RenderLayers::layer(car_layer),
                Some(root),
                graphics,
            )?;
        }
    }
    Ok(out)
}

/// Pretty: each camera gets the sky as a skybox and fog in the haze's colour. Simple:
/// neither, the flat clear colour behind everything.
fn sky_and_fog(ecs: &mut World, world: &SimWorld, pretty: bool, cameras: &[Entity]) {
    if !pretty {
        for &camera in cameras {
            if let Ok(mut e) = ecs.get_entity_mut(camera) {
                e.remove::<(Skybox, DistanceFog)>();
            }
        }
        return;
    }
    let sun = world
        .sun
        .map(|s| camera_core::Vec3::new(s.direction.x, s.direction.y, s.direction.z));
    let cube = ecs
        .resource_mut::<Assets<Image>>()
        .add(sky_cube(world.sky, sun));
    let haze = pretty::haze(world.sky);
    let haze = Color::linear_rgb(haze.r, haze.g, haze.b);
    for &camera in cameras {
        if let Ok(mut e) = ecs.get_entity_mut(camera) {
            e.insert((
                Skybox {
                    image: Some(cube.clone()),
                    // The picture's linear colour comes out as it is.
                    brightness: 1.0 / exposure(),
                    rotation: Quat::IDENTITY,
                },
                DistanceFog {
                    color: haze,
                    directional_light_color: Color::linear_rgba(1.0, 0.92, 0.75, 0.25),
                    directional_light_exponent: 24.0,
                    falloff: FogFalloff::Linear {
                        start: FOG_START,
                        end: FOG_END,
                    },
                },
            ));
        }
    }
}

/// The sky ([`pretty::sky`]) as a cube map: six faces, each the directions it covers.
fn sky_cube(base: camera_core::Rgb, sun: Option<camera_core::Vec3>) -> Image {
    let n = SKY_PIXELS;
    // Each face's direction for picture coordinates s, t (-1 to 1), as the cube faces are
    // laid out (+x, -x, +y, -y, +z, -z). Bevy's skybox looks up (x, y, -z) for a world
    // direction (x, y, z), so the face's z is turned back.
    let faces: [fn(f64, f64) -> [f64; 3]; 6] = [
        |s, t| [1.0, -t, -s],
        |s, t| [-1.0, -t, s],
        |s, t| [s, 1.0, t],
        |s, t| [s, -1.0, -t],
        |s, t| [s, -t, 1.0],
        |s, t| [-s, -t, -1.0],
    ];
    let pixel = |i: u32| 2.0 * (f64::from(i) + 0.5) / f64::from(n) - 1.0;
    let data: Vec<u8> = faces
        .iter()
        .flat_map(|face| {
            (0..n * n).flat_map(move |i| {
                let [x, y, z] = face(pixel(i % n), pixel(i / n));
                let c = pretty::sky(camera_core::Vec3::new(x, y, -z), base, sun);
                let [r, g, b] = c.to_srgb8();
                [r, g, b, 255]
            })
        })
        .collect();
    let mut img = Image::new(
        Extent3d {
            width: n,
            height: n,
            depth_or_array_layers: 6,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    );
    img.texture_view_descriptor = Some(TextureViewDescriptor {
        dimension: Some(TextureViewDimension::Cube),
        ..default()
    });
    img
}

fn spawn_shape(
    ecs: &mut World,
    shape: &Shape,
    layers: RenderLayers,
    parent: Option<Entity>,
    graphics: Graphics,
) -> Result<Entity, RenderError> {
    let pretty = graphics == Graphics::Pretty;
    let flat = matches!(
        shape.geometry,
        Geometry::Plane { .. } | Geometry::Cuboid { .. }
    );
    // Grain only where it can be laid in metres: planes and boxes.
    let surface = if pretty && flat {
        Surface::guess(&shape.name, &shape.look)
    } else {
        Surface::Plain
    };
    let mut mesh = mesh(&shape.geometry)?;
    if surface != Surface::Plain {
        uv_in_metres(&mut mesh);
    }
    let bumps = pretty && shape.look.normal_map.is_some() && mesh.generate_tangents().is_ok();
    let mesh = ecs.resource_mut::<Assets<Mesh>>().add(mesh);
    let material = material(ecs, &shape.look, surface, bumps)?;
    let mut e = ecs.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(material),
        transform(&shape.pose),
        layers,
        Name::new(shape.name.clone()),
    ));
    if let Some(parent) = parent {
        e.insert(ChildOf(parent));
    }
    Ok(e.id())
}

/// SDF's shapes in SDF's own axes (a cylinder along z, a plane facing z).
fn mesh(geometry: &Geometry) -> Result<Mesh, RenderError> {
    Ok(match *geometry {
        Geometry::Plane { x, y } => {
            Plane3d::new(Vec3::Z, Vec2::new(x as f32 / 2.0, y as f32 / 2.0))
                .mesh()
                .build()
        }
        Geometry::Cuboid { x, y, z } => Cuboid::new(x as f32, y as f32, z as f32).mesh().build(),
        Geometry::Cylinder { radius, length } => Cylinder::new(radius as f32, length as f32)
            .mesh()
            .resolution(24)
            .build()
            .rotated_by(Quat::from_rotation_x(FRAC_PI_2)),
        Geometry::Octagon { across, thickness } => octagon(across as f32, thickness as f32),
        Geometry::Mesh { ref file, scale } => obj_mesh(file, scale)?,
    })
}

/// Picture coordinates in metres, as a box would cast them: each face takes the two axes
/// across it. The material's `uv_transform` then repeats the picture every tile.
fn uv_in_metres(mesh: &mut Mesh) {
    let (
        Some(VertexAttributeValues::Float32x3(positions)),
        Some(VertexAttributeValues::Float32x3(normals)),
    ) = (
        mesh.attribute(Mesh::ATTRIBUTE_POSITION),
        mesh.attribute(Mesh::ATTRIBUTE_NORMAL),
    )
    else {
        return;
    };
    let uvs: Vec<[f32; 2]> = positions
        .iter()
        .zip(normals)
        .map(|(p, n)| {
            let [ax, ay, az] = n.map(f32::abs);
            if az >= ax && az >= ay {
                [p[0], -p[1]]
            } else if ax >= ay {
                [p[1], -p[2]]
            } else {
                [p[0], -p[2]]
            }
        })
        .collect();
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
}

/// An OBJ file, as triangles with the file's normals and picture coordinates.
fn obj_mesh(file: &Path, scale: [f64; 3]) -> Result<Mesh, RenderError> {
    let text = std::fs::read_to_string(file)
        .map_err(|e| RenderError::Mesh(format!("{}: {e}", file.display())))?;
    let soup = TriMesh::from_obj(&text)
        .map_err(|e| RenderError::Mesh(format!("{}: {e}", file.display())))?
        .scaled(scale.map(|s| s as f32));
    // OBJ's v grows upwards; a picture's rows go down.
    let uvs: Vec<[f32; 2]> = soup.uvs.iter().map(|[u, v]| [*u, 1.0 - v]).collect();
    Ok(Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, soup.positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, soup.normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs))
}

/// A picture, tiled (the campus's road and grass repeat every few metres), with its
/// smaller copies (mipmaps), so a far-off road is its average colour, not a flicker.
/// `format`: sRGB for a colour picture, linear for bumps and detail.
fn tiled_image(picture: image::RgbaImage, format: TextureFormat) -> Image {
    let (w, h) = picture.dimensions();
    let levels = 32 - w.max(h).max(1).leading_zeros();
    let mut data = picture.as_raw().clone();
    let mut level = picture;
    for _ in 1..levels {
        let (lw, lh) = ((level.width() / 2).max(1), (level.height() / 2).max(1));
        level = image::imageops::resize(&level, lw, lh, image::imageops::FilterType::Triangle);
        data.extend_from_slice(level.as_raw());
    }
    let mut img = Image::new_uninit(
        Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        format,
        RenderAssetUsages::RENDER_WORLD,
    );
    img.data = Some(data);
    img.texture_descriptor.mip_level_count = levels;
    let mut sampler = ImageSamplerDescriptor::linear();
    sampler.set_address_mode(ImageAddressMode::Repeat);
    sampler.set_anisotropic_filter(16);
    img.sampler = ImageSampler::Descriptor(sampler);
    img
}

/// A PNG, tiled with its mipmaps, as `format`.
fn load_tiled(
    ecs: &mut World,
    path: &Path,
    format: TextureFormat,
) -> Result<Handle<Image>, RenderError> {
    let picture = image::open(path)
        .map_err(|e| RenderError::Texture(format!("{}: {e}", path.display())))?
        .to_rgba8();
    Ok(ecs
        .resource_mut::<Assets<Image>>()
        .add(tiled_image(picture, format)))
}

/// A flat octagon facing +x, its flats up, down and to the sides. The picture's square
/// fills the square around it: its right edge at +y (seen from in front, +x).
fn octagon(across: f32, thickness: f32) -> Mesh {
    let r = across / 2.0 / (std::f32::consts::PI / 8.0).cos();
    let corners: Vec<Vec2> = (0..8)
        .map(|k| {
            let a = std::f32::consts::PI / 8.0 + k as f32 * std::f32::consts::PI / 4.0;
            Vec2::new(r * a.cos(), r * a.sin()) // (y, z)
        })
        .collect();
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut uvs = Vec::new();
    let mut indices = Vec::new();
    for (x, normal) in [(thickness / 2.0, 1.0f32), (-thickness / 2.0, -1.0)] {
        let base = positions.len() as u32;
        for p in std::iter::once(Vec2::ZERO).chain(corners.iter().copied()) {
            positions.push([x, p.x, p.y]);
            normals.push([normal, 0.0, 0.0]);
            uvs.push([0.5 + p.x / across, 0.5 - p.y / across]);
        }
        for k in 0..8u32 {
            let (a, b) = (base + 1 + k, base + 1 + (k + 1) % 8);
            // Counter-clockwise seen from the side the normal points to.
            if normal > 0.0 {
                indices.extend([base, a, b]);
            } else {
                indices.extend([base, b, a]);
            }
        }
    }
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_indices(Indices::U32(indices))
}

/// A shape's material. `surface`: its grain and roughness (pretty), or
/// [`Surface::Plain`]: matte, its plain colour, as Gazebo drew. `bumps`: draw its normal map.
fn material(
    ecs: &mut World,
    look: &Look,
    surface: Surface,
    bumps: bool,
) -> Result<Handle<StandardMaterial>, RenderError> {
    let texture = match &look.texture {
        Some(path) => Some(load_tiled(ecs, path, TextureFormat::Rgba8UnormSrgb)?),
        None => None,
    };
    let normal_map = match (&look.normal_map, bumps) {
        (Some(path), true) => Some(load_tiled(ecs, path, TextureFormat::Rgba8Unorm)?),
        _ => None,
    };
    let (detail, gain) = match surface.detail(DETAIL_PIXELS) {
        Some(d) => {
            let picture = image::RgbaImage::from_raw(d.size, d.size, d.rgba)
                .ok_or_else(|| RenderError::Texture("a detail picture's size".to_owned()))?;
            let img = tiled_image(picture, TextureFormat::Rgba8Unorm);
            (Some(ecs.resource_mut::<Assets<Image>>().add(img)), d.gain)
        }
        None => (None, 1.0),
    };
    let shiny = surface != Surface::Plain || normal_map.is_some();
    let c = look.colour;
    Ok(ecs
        .resource_mut::<Assets<StandardMaterial>>()
        .add(StandardMaterial {
            // The detail picture's average is 1 / gain: the colour comes back to its own.
            base_color: Color::linear_rgb(c.r * gain, c.g * gain, c.b * gain),
            base_color_texture: texture.or(detail),
            normal_map_texture: normal_map,
            // Matte, as Gazebo's plain materials: almost no shine. Pretty: each surface's.
            perceptual_roughness: surface.roughness(),
            reflectance: if shiny { 0.3 } else { 0.05 },
            // The detail picture's coordinates are in metres: one picture each tile.
            uv_transform: if surface == Surface::Plain {
                Affine2::IDENTITY
            } else {
                Affine2::from_scale(Vec2::splat(1.0 / surface.tile() as f32))
            },
            ..default()
        }))
}
