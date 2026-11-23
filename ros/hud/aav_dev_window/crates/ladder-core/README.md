# ladder-core

## Overview

The test ladder's window (`pixi run ladder-window`), without the window: no Bevy, no
egui, no ROS, so `tools/test.sh` tests it on any laptop. `tools/ladder.py` holds every
rule and is the only writer of the progress file; this crate reads what it prints and
builds its commands ([Proving it](../../../../../docs/proving-it.md#the-window-pixi-run-ladder-window)).

```text
python3 tools/ladder.py --json ──▶ Ladder (types) ──▶ Cell, RunButton, CarSide ──▶ the window draws
        ▲                                                   │ a press
        └── run <rung> [--only <test>] · record <rung> pass|fail ... ◀──────┘ (Session)
python3 tools/ladder.py session <rung> --json ──▶ Report: steps, numbers, Graph, Fit
```

## Interface

| Module | Holds |
|---|---|
| `ladder` | the tool's JSON as types: `Ladder::parse`, `Rung` (with its `tests`: `SingleTest`, and `after`), `Side`; `State` (pass, fail, not run) and `Place` (sim, car) are enums |
| `show` | `Cell` (the words in a cell: `PASS 2026-11-22`, `not run, shut`), `run_button` and `test_button` (on, or off and `WhyNot`), `RunsIn` (the simulator that is open, or one of its own), `CarSide` (a command to copy, or the page), `CAR_RULE` |
| `command` | `ToolCommand`: `table()`, `session(rung)`, `record(rung, form)`; a run comes only from `run_button` or `test_button`. `RecordForm`, `Outcome` |
| `report` | a session as the tool prints it (`Report::parse`): `StepRow` (`Status`: PASS, FAIL, SKIPPED, NOT RUN; its `vehicle.yaml` lines, its `Number`s), `Graph` (commanded against measured: `time`, `values`, `Span::ticks`, the legend's words, `widest_gap`), `Fit` (mean ± sd, trusted or why not); `current_step` (which step a run is on, from its lines); `tick_label` |
| `process` | `ask` (a short call, answered on a thread), `Running` (a run in its own process group: `new_lines`, `ended`, `stop`) |
| `session` | `Session`: the ladder read every 2 s and after each action, one run at a time (a rung, or one test of it), the selected rung's newest session and the chosen step, the console's last 200 lines, the form; `Source` (the tool, or pretend) |
| `jacks` | "The simulated car: on stands / on the ground": `Ask` (`tools/ros.sh --local python3 tools/jacks.py <status, up or down> --sim --json`), `Answer` (its JSON), `Jacks` (the state, and the one button it allows) |
| `pretend` | sample data for `--pretend`: every kind of cell, and two real sessions from Ravensim (22 Nov) for the table and the graph |

## Invariants

- **Nothing here starts a car test.** The only runs are `tools/ladder.py run <rung>` and
  `run <rung> --only <test>`, which the tool runs in the simulator; `run_button` and
  `test_button` take no place. The car's command is text.
- It never opens the progress file or a report's files, and decides no rule: a shut rung's button is off
  because the tool's JSON says `blocked_by`, and a refusal is shown in the tool's words.
- The jacks switch is the simulator's only: every command carries `--sim` and `--local`,
  the button is on only when the tool heard a simulator on this computer, and the tool
  refuses when anything says REAL. Ravensim's own refusal (wheels spinning) is shown word for word.
- `--pretend` runs nothing and records nothing.

## Side Effects

- `Session` starts `python3 tools/ladder.py` in the repository's root: `--json` every 2 s,
  `session <rung> --json` when a rung is chosen or a run ends, `run` and `record` on a
  press. A run is the leader of its own process group; Stop, and
  the window closing, signal that group only.
- `Session` starts `tools/ros.sh --local python3 tools/jacks.py status --sim --json` 5 s
  after each answer, and `up` or `down` on a press.

## Verification

```sh
cd ros/hud/aav_dev_window && cargo test -p ladder-core    # also in tools/test.sh
```

`tests/ladder.json` is the real tool's output; `tools/tests/test_ladder.py` checks the tool
still prints those fields. `tests/session.json` and `tests/session-stands.json` are the
tool's `session --json` after two real runs in Ravensim (22 Nov; `--pretend` shows them). `tests/session.rs` drives the session against a stand-in tool
(`tests/fake_ladder.py`): it needs `python3` and a Unix.
