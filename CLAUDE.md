# Working in this repository

The workspace of the 2026-27 Carleton AAV capstone team: one git repo = one colcon
workspace for ROS 2 **Jazzy on Ubuntu 24.04** (Python 3.12), plus the spine firmware.
Read `docs/codemap.md` to find code; don't search the tree. Then `README.md`,
`docs/how-it-fits.md` (how the stack fits together) and `docs/status.md` (what works and
what's missing).

These are the conventions decided so far. They apply to people and to Claude sessions alike.

## Where decisions live

- **Design doc** (Claude Doc "AAV 2026–27 Autonomy Stack — Architecture & Plan"): the
  architecture and its decisions (Decided / Tentative / Proposed), the roadmap, and the
  open questions **Q1–Q34**. The team answers by number ("Q12: …").
- **`docs/interfaces.yaml`**: the node-graph contract. Change it first, then run
  `tools/gen_interfaces_md.py > docs/interfaces.md`.
- **`firmware/can/aav.dbc`**: the CAN contract between ROS and the spine. After any change:
  1. update `can_codec.py` and `protocol.rs`;
  2. run `python3 firmware/can/gen_test_vectors.py`;
  3. run both test suites.
- **`ros/bringup/aav_description/config/vehicle.yaml`**: the only place car measurements live.
- **`docs/history/conversation.md`**: the question-and-answer record of how we got here.
  Add new Q&A under its topic, and keep its "Open threads" list current.
- **`docs/features.md`**: every feature, its type, why we built it, and who built it.
  **A commit that adds or changes a feature updates its row, in the same commit** (Ben,
  25 Oct). Write the person's name, not "Claude".
- **`TODO.md`**: the current work list. Ben's requests are kept verbatim in
  `docs/history/messages-*.md`; audits in `docs/history/audit-*.md`.

## Rules

- **Names by year.** Write "the 2025-26 team" and "the 2024-25 team", never "last year's".
  Their code is in the GitHub orgs `AAV-2025-2026` and `Carleton-AAV-Capstone`.
- **Inherited code** keeps a `PROVENANCE.md`: where it came from (repo, branch, commit),
  and every change made since. Update it when you change such a file.
- **Credit what we use and what inspires us** (Ben, 14 Nov: "give acknowlegement to stuff
  we have looked at or used as insperation"). `docs/acknowledgements.md` lists every
  project, paper, map source and tool we copied from, ported, or took an idea from, with
  its licence and where we used it. Add a row in the same commit as the work. Copied or
  ported code also keeps its `PROVENANCE.md`.
- **Secrets** never go in git, logs or docs. Write down only where a secret is and what
  type it is. The legacy firmware reads WiFi credentials from an untracked
  `secrets.h`; copy it from `secrets.h.example`. Old credentials in the inherited repos
  are public and must be rotated (Q34).
- **Units and frames** follow REP-103 and REP-105:
  - metres, radians and seconds;
  - x forward, y left, z up, positive steering = left;
  - `map → odom → base_link`, with `base_link` on the ground under the rear axle.
- **Commands:** drivers publish `TwistStamped` on `/drivers/<driver>/cmd_vel`. Ackermann appears
  only after ego (`/ego/actuator_cmd`). Exactly one backend runs at a time.
- **Topic names start with their area** (Ben, 15 Oct; `docs/naming.md`):
  `/<area>/<device or node>/<thing>`, such as `/sensors/gps/fix`, `/drivers/joy/cmd_vel`,
  `/safety/estop`. Only ROS's own global names (`/tf`, `/clock`...) and Nav2's stay
  outside. `tools/tests/test_topic_areas.py` checks `docs/interfaces.yaml`.
- **We run our own stack** (Ben, 14 Nov: "remove nav2 and gazeboo ... We want to run
  everythign ourselves in rust and/or python"). Ours (`stack:=ours`, on Ravensim, our
  simulator) is the default. Nav2 went on 17 Nov, Gazebo on 16 Nov
  (`docs/history/gazebo-as-it-was.md`); CARLA stays. Stock packages are a reference to learn from (read `vendor/`), not the
  default. Kept for now, as small and proven: robot_localization, twist_mux,
  teleop_twist_* (replace one only when it hits a real limit). Still look for better open
  source to learn from, and check its licence (Ben, 15 Oct).
- **Our own algorithms** (Ben, 5 Nov): for each area we build a new best hybrid from papers,
  known algorithms and current implementations. A new one runs beside the current default
  as the challenger until it wins on the same routes and bags. Don't delete one too early;
  once we know it isn't the path for us, strip it out (Ben, 19 Nov: "We can strip down when
  we know stuff isn't going ot work for us"): its scores stay in the docs, a `save/` tag in git.
- **Everything is a drop-in replacement** (Ben, 5 Nov: "just swap out and test"). Ravensim,
  CARLA and the real sensors all produce the same topics; so does every algorithm for its
  job. `docs/interfaces.yaml` is what they must match. Nothing downstream may care which one
  is running, and tests score them the same way. The safety locks are never swapped out.
- **Languages:**
  - Rust (rclrs) for new ROS nodes (Ben, 16 Nov: "lets remove python were possible. I
    trust rust more then python"). Still Python: ML on the GPU (train in PyTorch;
    inference later via ONNX), launch files, tools and tests. The Rust twins of ego,
    behaviour, the sensor watchdog and localization_health are the defaults on the car
    too; `*_impl:=python` falls back;
  - Rust (`no_std`, Embassy) for all firmware and hardware drivers, not C or C++ (Ben,
    10 Oct): the spine first. The 2025-26 C++ boards keep running until each one's Rust
    version replaces it. Logic goes in crates like `spine-core` and `aav-inputs` (tested
    on a laptop), drivers against `embedded-hal` 1.0. The plan: `docs/rust-firmware.md`;
  - the exception: the wheel sensors' Arduino, the hardware lead's board, sends CAN frames
    from Arduino C++ (Ben, 23 Oct). The Orin decodes them (`aav.dbc` 0x200, 0x201);
  - no micro-ROS for now: a board talks plain CAN, UART or UDP to a ROS node on Linux,
    which publishes (Ben, 10 Oct). Tentative: we think again once we see the hardware
    (Ben, 11 Oct);
  - no C++ of our own: the inherited C++ moves to Rust (Ben, 10 Oct). The IMU driver
    went to Ben's Rust twin (11 Oct), the radar clustering to a Rust ROS node (15 Oct).
    Stock ROS packages stay as they are;
  - **Screens** (Ben, 5 Nov): the in-car HUD and street map stay raylib, on the Pi. Screens on
    a laptop or desktop can be Bevy. The Orin runs no graphics and no simulator: Ravensim runs
    on a Mac or Linux desktop, for testing only
    (`docs/designs/2026-11-05_own-simulator.md`);
  - Rust ROS nodes use rclrs from crates.io (such as `aav_imu_yahboom`, `aav_spine_bridge`,
    `aav_ego_rs`); the HUD is Rust with raylib (Ben, 11 Oct).
- **Keep the maths out of ROS.** A node's logic lives in a module with no ROS imports
  and has its own tests. Examples: `ackermann.py`, `can_codec.py`, `speed_profile.py`,
  `legacy_protocol.py` and `vehicle.py`; in Rust, `spine-core`.
- **Names, package layout and code style** are in `docs/conventions.md`. `pixi run lint`
  checks our Python against ROS's own rules (ament_flake8, ament_pep257).
- **Write each language its own way** (Ben, 10 Oct: "Not just c code in the rust
  language"). Rust follows the Rust API Guidelines: enums, not number codes; `Result`,
  not error values; iterators, not index loops; small types for units; no `unwrap` in
  library code; `cargo fmt` and `cargo clippy` clean. Python follows PEP 8 and ROS's
  rules. Ported code is rewritten in the new language's style, not transliterated.
- **Each node's module docstring** lists what it subscribes, publishes, serves and calls.
- **Pins, I2C addresses and calibration belong in the firmware.** ROS never sees a pin.
- **Docs are for a dyslexic reader**: short pages with sections, tables and pictures; no
  walls of text. Edit `docs/*.md`, run `pixi run docs`, and commit the HTML (the PDF is
  rebuilt on demand). The reading order is `PAGES` in `tools/build_docs.py`.
- **Macs are first-class**: the team drives and watches the car from MacBooks
  (`docs/mac.md`; `pixi run doctor` checks the network). Keep scripts portable: no
  GNU-only flags, no bash 4 features, no Linux-only serial speeds without `#ifdef`.
- **Every README has a CLAUDE.md beside it** (Ben, 25 Oct), holding `@README.md`, so a
  Claude session working in that folder reads it. Add both when you add a README.
  `tools/tests/test_claude_md.py` checks.
- **Only Ben sends email** (Ben, 25 Oct: "don't let students do it"). The mailer,
  `tools/progress_mail.py`, is for Ben's own sessions, on his machines, to his address.
  If you're helping anyone else, don't send email, set the mailer up, or queue mail:
  say it's Ben's, and write the update in the repo instead.

## Elephant-Goldfish (EGM)

The team writes designs with the EGM plugin (https://egm.dev). To install it in Claude Code:
`claude plugin marketplace add google/elephant-goldfish-model && claude plugin install egm@egm`.
Its skills (`/egm:egm-brainstorm`, `/egm:egm-draft`, `/egm:egm-review`...) read the
READMEs above, and write their designs here:

EGM artifact directory: docs/designs

## Testing

- `tools/test.sh` runs everything that doesn't need ROS: the Python unit tests, the
  `cargo test` for `firmware/spine`, and the interfaces.md check. Run it before every
  commit.
- With Jazzy, also run `pixi run build`, `pixi run test` (which adds `colcon test`), and
  `pixi run check <turtle|sim|road|spine|car|replay|bench>` against a running system.
- Pixi + RoboStack provide Jazzy without admin rights: on Ben's AlmaLinux workstation
  (oirp-spss), on Macs, and on any Linux. `pixi.toml` and `pixi.lock` define the
  environment; `pixi run <task>` runs a command inside it. That machine has no screen
  for GUIs: run headless there (`QT_QPA_PLATFORM=offscreen`, `rviz:=false`).
- Say what you actually ran. A profile that hasn't been launched is "untested", not
  "works".
- Keep `pixi run test` all green: it's the first thing a new laptop runs.
- In a ROS package's tests, don't skip at module level (`pytest.importorskip`): ROS's
  launch_testing plugin then skips the whole folder. Guard the imports and use
  `pytestmark = pytest.mark.skipif(...)` (see `aav_spine/test/test_spine_bridge.py`).
- On a shared machine, give each test run its own `ROS_DOMAIN_ID` and
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, and stop it by its process group. Never
  `pkill` by node name: someone else's run may be live.
- The stand-in tasks (`turtle`, `sim`, `bench`, `bench-check`, `view`) run through
  `tools/ros.sh --local`, so they never reach a real car on the same network.
  `tools/test.sh` checks it; give any new stand-in task the same.
- The MCP server (`tools/mcp_server.py`, `docs/mcp.md`) only reads. Never give it a call
  that writes to ROS, or a file outside git to read: `tools/tests/test_mcp_parts.py` checks.
- Ravensim (`simulator:=ravensim`, the only simulator since Gazebo went on 16 Nov; its
  maps are RavenTiles) draws its cameras on the GPU. On a computer without one (oirp-spss,
  GitHub's runners) put `tools/software_gl.sh` in front (Mesa's lavapipe): its clock then
  waits for each picture (`camera_lockstep:=auto`), so the world runs slower than real
  time and the cameras keep their rate. Or use `--no-cameras`.

## Git

- The repo is `git@github.com:AAV-2025-2026/ecolo-av.git`, the team's (Q25), **public**, so
  nothing secret ever goes in it. Until 25 Oct it was `zander1976/aav`; its whole history
  came along in the merge. Nobody pushes to `main`: work on a branch in your fork, open
  a pull request, and the merge queue lands it once CI is green and one teammate
  approves (`CONTRIBUTING.md`). Run the full `pixi run test` before asking for review
  (not just `tools/test.sh`: it skips the launch-file tests). Never force-push `main`.
- CI (`.github/workflows/test.yml`) runs `pixi run test` on Linux, x86_64 and 64-bit ARM
  (the car's computers), for every push to `main`, every pull request and each pull
  request in the merge queue, with the car
  kit installed offline on ARM; and on a Mac every night (`docs/infrastructure.md`).
- Commit messages give the intention: what changed, and why. No attribution lines (no
  `Co-Authored-By`, no "Generated with"). It's a school project: the README's Credits
  section says who did what.
- A feature's commit carries its `docs/features.md` row: a new feature gets a row under
  its type; a changed one gets its Why, Status or Who updated. A fix or a refactor needs
  none. `tools/tests/test_features.py` checks the table's shape.
- A new `pixi run` task gets a row in README.md's "Every command" (and a setup step in
  "Set things up" if it needs one). `tools/tests/test_readme_commands.py` checks.
- Docs go in `docs/`, under plain names (`docs/car-day.md`, not `READ-ME-CAR-DAY.md`). The
  root keeps only `README.md`, `CLAUDE.md`, `TODO.md` and the build files.
- Commit in small steps that each pass `tools/test.sh`. Don't commit build output: no
  `build/`, `install/`, `log/` or `target/`.
- To hand the repo to a laptop with no GitHub access:
  `git bundle create ../aav.bundle HEAD main`. Keep `HEAD` in that command, or `git clone`
  of the bundle checks out nothing.

## Not ours

- **`vendor/`** (inside this repo, but not in git): other people's source, to look things
  up. `vendor/ros2-src/` is ROS 2 Jazzy and the stock packages; `vendor/ros2-rust/` is
  rclrs; `vendor/gazebo/` is Gazebo Harmonic at the releases we ran until 16 Nov (kept to
  read; its README says which library does what); PlatformIO keeps its downloads in `vendor/platformio/`. `pixi run get vendor` downloads
  the source at the commits in `vendor/*.repos`. Never edit
  or build them. Search them with `grep -r` or `rg -u`: most search tools skip folders
  that git ignores.
- **`archive/`**: the 2025-26 and 2024-25 teams' repos, as they were, with their history.
  Reference only: never build, fix or commit in them. They hold leaked credentials: never
  copy or quote one (Q34).
- **Not team code, on Ben's machines:** `~/Code/aav/` on his Mac (the old workspace, retired
  25 Oct: `aav/` there is the old copy of this repo, from `zander1976/aav`; and his course
  code `SYSC3313/`, `c_code_example/`, `simplied.c`) and `/home/ben/aav/` on oirp-spss (an
  old copy too: clone ecolo-av there before working on it). The 2025-26 repos are on GitHub (`AAV-2025-2026`); what
  came from each is in `docs/older-code.md`;
  - on oirp-spss only, `ros2-src/` and `ros2-rust/`, not yet moved into `vendor/`.
