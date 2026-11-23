# ladder-core

## Overview

The test ladder's window (`pixi run ladder-window`), without the window: no Bevy, no
egui, no ROS, so `tools/test.sh` tests it on any laptop. `tools/ladder.py` holds every
rule and is the only writer of the progress file; this crate reads what it prints and
builds its commands ([Proving it](../../../../../docs/proving-it.md#the-window-pixi-run-ladder-window)).

```text
python3 tools/ladder.py --json ──▶ Ladder (types) ──▶ Cell, RunButton, CarSide ──▶ the window draws
        ▲                                                   │ a press
        └── run <rung> · record <rung> pass|fail ... ◀──────┘ (Session)
```

## Interface

| Module | Holds |
|---|---|
| `ladder` | the tool's JSON as types: `Ladder::parse`, `Rung`, `Side`; `State` (pass, fail, not run) and `Place` (sim, car) are enums |
| `show` | `Cell` (the words in a cell: `PASS 2026-11-22`, `not run, shut`), `run_button` (on, or off and `WhyNot`), `CarSide` (a command to copy, or the page), `CAR_RULE` |
| `command` | `ToolCommand`: `table()`, `record(rung, form)`; a run comes only from `run_button`. `RecordForm`, `Outcome` |
| `process` | `ask` (a short call, answered on a thread), `Running` (a run in its own process group: `new_lines`, `ended`, `stop`) |
| `session` | `Session`: the ladder read every 2 s and after each action, one run at a time, the console's last 200 lines, the form; `Source` (the tool, or pretend) |
| `jacks` | "The simulated car: on stands / on the ground": `Ask` (`tools/ros.sh --local python3 tools/jacks.py <status, up or down> --sim --json`), `Answer` (its JSON), `Jacks` (the state, and the one button it allows) |
| `pretend` | sample data for `--pretend`: every kind of cell |

## Invariants

- **Nothing here starts a car test.** The only run is `tools/ladder.py run <rung>`, which
  the tool runs in the simulator; `run_button` takes no place. The car's command is text.
- It never opens the progress file, and decides no rule: a shut rung's button is off
  because the tool's JSON says `blocked_by`, and a refusal is shown in the tool's words.
- The jacks switch is the simulator's only: every command carries `--sim` and `--local`,
  the button is on only when the tool heard a simulator on this computer, and the tool
  refuses when anything says REAL. Ravensim's own refusal (wheels spinning) is shown word for word.
- `--pretend` runs nothing and records nothing.

## Side Effects

- `Session` starts `python3 tools/ladder.py` in the repository's root: `--json` every 2 s,
  `run` and `record` on a press. A run is the leader of its own process group; Stop, and
  the window closing, signal that group only.
- `Session` starts `tools/ros.sh --local python3 tools/jacks.py status --sim --json` 5 s
  after each answer, and `up` or `down` on a press.

## Verification

```sh
cd ros/hud/aav_dev_window && cargo test -p ladder-core    # also in tools/test.sh
```

`tests/ladder.json` is the real tool's output; `tools/tests/test_ladder.py` checks the tool
still prints those fields. `tests/session.rs` drives the session against a stand-in tool
(`tests/fake_ladder.py`): it needs `python3` and a Unix.
