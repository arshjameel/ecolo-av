# map-edit

## Overview

The map editor's logic (TODO 251, phase E of the [testing plan](../../../../docs/testing-plan.md)):
a map folder from `pixi run map-drive`, edited by a person, saved as a **new** folder. A Rust
crate with no ROS and no window. The window is `map_edit` in
[`aav_dev_window`](../../../hud/aav_dev_window/README.md); the page is
[docs/map-editor.md](../../../../docs/map-editor.md).

```text
 <drive>/map/ ─▶ MapFolder::load ─▶ Editor ── apply(Edit), undo ──▶ save ─▶ <drive>/map-edit-1/
  never changed                        │                                 edits.yaml, labels.yaml,
                                       ▼                                 chunks cut again
                    layers: HeightImage, grid_rgba, chunk_outlines
```

## Interface

| Item | What |
|---|---|
| `MapFolder::load(dir)` | `ndt.pcd`, `map2d.yaml`/`.pgm`; `ndt.yaml`, `poses.tum`, `labels.yaml` if there; `datum()`, `placed_by()` |
| `Editor::new(map)`, `apply(Edit)`, `undo()` | the edits; the map read never changes, the state is worked out from the list |
| `Edit::Align { to }` | the whole map moved by hand (x, y, yaw): one logged edit |
| `Edit::Delete { region, band, cells }` | a box (turned or not) and a height band: NDT's points go; the 2D cells become unknown, free or stay |
| `Edit::Label`, `Relabel`, `Unlabel` | a box with a class: tree, building, parked car, pole, other |
| `Editor::save(out)`, `next_free(source)` | approve: a new folder (`<map>-edit-<n>`), never over one that exists or the source |
| `HeightImage`, `grid_rgba`, `chunk_outlines` | the layers as pictures and outlines |
| `map-edit-batch` (a binary) | the same edits from the command line, for scripts |

```sh
cargo run --release --bin map-edit-batch -- bags/<drive>/map \
    --delete 49.5,-1,52.5,31 --label parked_car:49.5,-1,52.5,31     # → bags/<drive>/map-edit-1
```

## Invariants

- The source folder is only read. A save writes `<out>.partial/`, then renames it.
- Boxes in edits are in the source map's frame, before the alignment.
- An aligned save moves the points, the 2D map, the path and the labels together, and says
  so: `placed: aligned_by_hand`, `slam_to_map` followed by the move, `edited:` in both YAMLs.
- An aligned 2D map gets new cells on whole metres (NDT's chunks need 1 m cube edges).
- Chunks are cut by the same code as `map-drive` (`map-chunks`, `ndt-core`).
- The 2D map writes unknown as grey 128, as `map-drive` does (between the thresholds).

## Testing

`cargo test` here (`tools/test.sh` runs it, with `cargo fmt` and `cargo clippy`).
