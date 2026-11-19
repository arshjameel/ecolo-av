# Provenance

| What | From | Licence |
|---|---|---|
| The selection rule in `mux-state-core` (`mux.rs`): expiry, locks, the lock priority, the winner | twist_mux 4.5.0 (the Jazzy release), `include/twist_mux/topic_handle.hpp` and `src/twist_mux.cpp` (`getLockPriority`, `hasPriority`), read in `vendor/ros2-src/ecosystem/twist_mux` | Apache-2.0 |

No code was copied: the rule was rewritten in Rust, in our style, from reading the C++.

## Changes since

| Date | Change |
|---|---|
| 19 Nov 2026 | Written (TODO 247), Ben Woodhead |
