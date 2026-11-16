# map-chunks

## Overview

The map cut into **fixed 64 m squares (chunks)**: step 1 of the three map levels
(`docs/designs/2026-11-15_three-levels.md`, TODO 175). A Rust crate with no ROS, shared
by NDT (`ndt-core`), our global costmap (`aav_costmap` `chunked.rs`) and the dev window.

```text
 car + route ─▶ wanted ─▶ { chunk: Fine | Coarse }  ─▶ Store::update ─▶ loaded / dropped
 whole map   ─▶ cut    ─▶ { chunk: OccupancyChunk }
```

| Level | Chunks | Cells |
|---|---|---|
| Fine | the 3 x 3 round the car | 0.25 m (256 x 256 per chunk) |
| Coarse | those the route crosses in the next 100 m | 1 m (64 x 64) |

## Interface

| Item | What |
|---|---|
| `Grid::new(origin, size)` | squares of `size` m (a whole number) from the map's origin |
| `Grid::key_at(p)`, `corner(key)`, `cells_per_side(detail)` | a point's chunk; a chunk's corner; its cells |
| `ChunkKey { i, j }`, `around(reach)`, `Display` (`i_j`) | one chunk; its neighbours; its file name |
| `Detail::{Coarse, Fine}`, `cell()` | 1 m or 0.25 m |
| `wanted(grid, car, route, ahead)` | the chunks the car needs now, at what detail |
| `Store<T>::update(want, load)` | load the new chunks, drop the rest; a failed load changes nothing |
| `OccupancyChunk` (`unknown`, `at`, `mark`) | one chunk's cells, -1 unknown, 0-100 as ROS counts |
| `cut(grid, WholeMap, detail)` | a whole map cut into chunks, each cell the worst under it |
| `files::read_map`, `write_chunks`, `read_chunk`, `read_grid` | a map server's `.yaml` + `.pgm` in; `<out>/chunks.yaml`, `fine/i_j.pgm`, `coarse/i_j.pgm` out (trinary colours, any viewer shows them), and back |
| `chunk-map <map.yaml> <out> [size]` (a binary) | the same from the command line: the campus, 625 chunks a detail, 1.5 s |

## Testing

`cargo test` here (`tools/test.sh` runs it, with `cargo fmt` and `cargo clippy`).
