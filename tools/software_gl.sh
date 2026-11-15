#!/usr/bin/env bash
# Run a command with Mesa's software renderers, so a computer with no GPU draws pictures:
# GitHub's runners, oirp-spss.
#
#   tools/software_gl.sh pixi run sim-check                   # ours: its cameras on lavapipe
#   tools/software_gl.sh pixi run sim-check --simulator gazebo --obstacle
#   tools/software_gl.sh --force pixi run sim-cameras --bench 100   # CI's way, beside a GPU
#
# | For | Mesa's | Through |
# |---|---|---|
# | Gazebo's cameras and lidar | llvmpipe | OpenGL (EGL) |
# | our simulator's cameras (Bevy, wgpu) | lavapipe (llvmpipe inside) | Vulkan |
#
# Why: our environment's EGL (libglvnd) and Vulkan loader have no driver of their own; they
# use the computer's. With none (no Mesa, no NVIDIA), Gazebo crashes as the world loads
# ("Unable to load Ogre Plugin[.../RenderSystem_GL3Plus]", then a segfault; 23 Oct), and
# our cameras find no adapter ("no GPU to draw with"; 15 Nov).
#
# Mesa comes from conda-forge through `pixi exec`, into pixi's cache, not into pixi.lock:
# Mesa in the default environment would also replace a Linux laptop's own GPU driver.
# A Linux computer with a GPU keeps it: wgpu picks a GPU before a software adapter.
#
# --force: the software renderer even beside a GPU (WGPU_FORCE_FALLBACK_ADAPTER=1), on Linux
# or a Mac, to try CI's way. On a Mac it also builds the cameras with Vulkan
# (feature vulkan-on-mac). Without --force a Mac just runs the command: it has Metal.
#
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -euo pipefail
MESA_GL="mesa-llvmpipe==26.2.5"   # OpenGL: llvmpipe, and EGL for it (mesa-libgallium)
MESA_VK="mesa-lavapipe==26.2.5"   # Vulkan: lavapipe
LOADER="libvulkan-loader"         # the Vulkan loader (libvulkan), to find lavapipe

force=false
if [[ "${1:-}" == --force ]]; then
  force=true
  shift
fi
system=$(uname -s)
if [[ "$system" != Linux && "$force" != true ]]; then
  exec "$@"
fi

if [[ "$system" == Linux ]]; then
  specs=(--spec "$MESA_GL" --spec "$MESA_VK" --spec "$LOADER")
else
  specs=(--spec "$MESA_VK" --spec "$LOADER")   # no EGL on a Mac: Gazebo there uses Metal
fi
prefix=$(pixi exec "${specs[@]}" -- sh -c 'echo "$CONDA_PREFIX"')
icd=$(ls "$prefix"/share/vulkan/icd.d/lvp_icd.*.json 2>/dev/null | head -n 1 || true)
if [[ -z "$icd" ]]; then
  echo "software_gl.sh: no lavapipe in $prefix ($MESA_VK)" >&2
  exit 1
fi
here=$(mktemp -d "${TMPDIR:-/tmp}/aav-software-gl.XXXXXX")

# Vulkan: lavapipe as the loader's only driver, and a folder with only the loader in it, so
# nothing else of Mesa's environment shadows our own libraries.
export VK_DRIVER_FILES="$icd"
export VK_ICD_FILENAMES="$icd"   # the same, for loaders older than 1.3.234
mkdir "$here/lib"
for lib in "$prefix"/lib/libvulkan.so.1 "$prefix"/lib/libvulkan.1.dylib "$prefix"/lib/libvulkan.dylib; do
  if [[ -e "$lib" ]]; then
    ln -s "$lib" "$here/lib/"
  fi
done

if [[ "$system" == Linux ]]; then
  if [[ ! -f "$prefix/lib/libEGL_mesa.so.0" ]]; then
    echo "software_gl.sh: no libEGL_mesa.so.0 in $prefix ($MESA_GL)" >&2
    exit 1
  fi
  # EGL's driver list, by full path: Mesa's own file names the library without a folder.
  cat > "$here/50_mesa.json" <<EOF
{"file_format_version": "1.0.0", "ICD": {"library_path": "$prefix/lib/libEGL_mesa.so.0"}}
EOF
  export __EGL_VENDOR_LIBRARY_FILENAMES="$here/50_mesa.json"
  export LD_LIBRARY_PATH="$here/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  echo "software_gl.sh: OpenGL (EGL) draws with Mesa llvmpipe, Vulkan with lavapipe ($prefix)" >&2
else
  # macOS takes DYLD_LIBRARY_PATH out of every program started through /usr/bin/env, so
  # the folder goes under its own name: tools/sim_cameras.sh and the launch put it back
  # (aav_gazebo worlds.sim_cameras_env).
  export AAV_VULKAN_LIB_DIR="$here/lib"
  export AAV_SIM_CAMERAS_FEATURES=vulkan-on-mac
  echo "software_gl.sh: Vulkan draws with Mesa lavapipe ($prefix)" >&2
fi
if [[ "$force" == true ]]; then
  export WGPU_FORCE_FALLBACK_ADAPTER=1
  echo "software_gl.sh: --force: a software adapter only, even beside a GPU" >&2
fi
exec "$@"
