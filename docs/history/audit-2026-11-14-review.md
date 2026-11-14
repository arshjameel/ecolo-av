# Audit, 14 November 2026: the whole review

Ben asked for a review of everything (13 Nov; `messages-2026-11-14.md`). This page holds
what the review found in the record: his messages, the Q&A, the audits and `TODO.md`. The
merged design it led to: [the whole car](../designs/2026-11-14_the-whole-car.md).

**How:** every `messages-*.md` and `conversation.md` read in full; `TODO.md` lines 1-215 in
full, the rest by its headings and open rows; code checked with `git log`, `gh pr list` and
greps. Nothing was launched. **Not verified:** anything about 9 Nov; whether any flash or
backup has happened since 7 Nov; `features.md` row by row.

## 1. Asked for, never done or half done

| Date | Ben's words | Exists today | Missing |
|---|---|---|---|
| 10 Oct, 15 Oct, 1 Nov | "back it up before the reflash" | a checklist in `docs/car-day.md` | no audit records a backup, or a file copied off the car |
| 10 Oct, 15 Oct | "checking for best.pt and/or just using another model" | the stock detector | `best.pt` never found |
| 15 Oct | "calculate the weight based on acceleration, and braking" | a weight model, 181 kg from his "400 pounds" | the car never weighed (TODO 14); the model never refitted (52b) |
| 15 Oct | "get carla 5 running too?" | the adapter, tested with no server | never run, though the lab PC runs CARLA (1 Nov) |
| 15 Oct | "Try to get this working in teh car" | the car kit, the runbooks | only the localization Pi has our kit |
| 28 Oct to 3 Nov | "interface for interacting with the sensors and actuators directly ... on a stand" | `hw_panel`, steering calibration, preflight | never run on the car (TODO 67) |
| 28-29 Oct | "merge the voxels with the camera feed"; "merge cubes that related like a stop sign" | cubes on the screens | no cube-to-object naming (now TODO 156) |
| 1 Nov | "login to each node, upgrade the os, and install our code" | `car_setup.sh`, a dry run | Day B not started; the first real install broke (PR #28) |
| 1 Nov | "drive 10m stragith. then mesaure how far it actually went" | `docs/know-the-car.md`, tests 4.1-4.5 | none run: every car number is still a guess |
| 3 Nov | "release note like an advertisement"; "definisions like what is ndt, mppi" | `aav-release-notes.pdf` | TODO 97, 98 open; four screenshots missing |
| 4 Nov | "Braking, clock, speed profile, tracking and CBF filter" | the behaviour planner and stop law, sim only | the brake wish (101), clocks (100), the filter; the tracker is paused |
| 5 Nov | "a single screen ... and a list of algo so you can swap" | the Bevy window | the algorithm list, SWAP, cancel a goal, the console (TODO 114) |
| 5 Nov | "A drive suggests changes, and a person accepts them" | the campus from OpenStreetMap | drive fitting, the GPS offset, approval (TODO 120) |
| 5 Nov | kerbs "1 foot tall ... the lidar should be able to see the curb" | kerbs drawn | may be invisible to Nav2; untested (TODO 119) |
| 5 Nov | screens "send heartbeats to the esp32 fsm" | `watch_screens:=`, sim only | the car refuses it; a lost Mac is undecided (TODO 116) |
| 5 Nov, 13 Nov | "why does the car occolate so much?" | two causes ruled out | no fix (TODO 124, 149) |
| 5 Nov | "test each one and find the best combination" | scores per trial | the league (TODO 150) |
| 6 Nov | "any things that would help us like bag files" | `docs/car-evening.md` | no bag recorded (now TODO 157) |
| 6-7 Nov | "send commands and read comments directly from the esp32 over wifi"; "graph it over time" | a plan | nothing built (TODO 126) |
| 13 Nov | "AlpaMayo Edge ... instead of yolo" | answered 14 Nov, in the whole car's "Not doing" | — |
| 13 Nov | "fix it so I can merge them both" | Ben's two commits on #28 | #28 and #29 both open |

## 2. What Ben keeps asking for

| Theme | Dates | Landing? |
|---|---|---|
| Get it on the car | 15 Oct; 1, 3, 5, 13 Nov | **no**: see section 5 |
| The wiggle | 5 Nov (three times), 13 Nov (three times) | **no** |
| Best of everything, as one system | 1, 4, 5, 13 Nov | designs only |
| Test the hardware directly | 28, 29 Oct; 1, 3, 6, 7 Nov | built, never on the car |
| One screen, fewer windows | 29 Oct; 1, 3, 4, 5 Nov | half: the window exists, the swap list doesn't |
| Explain it simply | 9, 10, 14, 15 Oct; 1, 3 Nov | done, but the pages keep growing |
| Python to Rust | eight times, 9 Oct to 3 Nov | mostly; no stated end point |
| The local and global EKF "don't talk" | 14 Oct, 1 Nov | explained twice; the question returns |

## 3. Drift from Ben's decisions

| Decision | Now |
|---|---|
| 22 Oct: save his messages word for word | `TODO.md` quotes him on 27 Oct, 30 Oct and 8 Nov with no messages file; 7 Nov holds only the ESP32 part; 9-10 Nov has nothing |
| 9 Oct: "I don't like reading huge documents" | `docs/car-day.md` is 1,072 lines; six designs pass 450 |
| 1 Nov: "Less is more" | 36 switches by 13 Nov |
| 14, 23 Oct: the ESP32 stays | `docs/decisions.md` still said Pico 2 or STM32, "not the ESP32" |
| Keep `decisions.md` and `status.md` current | both last changed 1 Nov; no Decided block after 29 Oct |

`TODO.md` uses the number 145 twice (the adaptive brake, and our navigator).

## 4. TODO.md's health

About 73 rows are open and 111 done. The 7-13 Nov rows are nearly all "built in the sim;
left: the car or the Orin".

| Kind | Rows |
|---|---|
| Duplicates | 124 and 149 (the wiggle); 102 and 137 (behaviour); 115, 134, 135, 141 (the simulator); 78, 83, 133 (diffusion); 73, 90, 107, 114 (one window); 74a and 82; 77 and 97 |
| Dead | 91 (PR #21 merged); 68, 77 (0.2 shipped); 99 ("Thursday: Ben picks"); the spare Pico 2; micro-ROS; snapshot pruning |
| **Buried but important** | 49b (nothing measures the driven wheels on stands); 93 (the cubes let the car into a post); 111 (the radar driver can die); 113 (the rear camera can trip the stop lock); 112 (duplicate `vehicle.yaml` keys); 119 (kerbs); 116 (a lost Mac); 14 (weigh the car); 60, 61, 64 (wheel, track, datum); 52b; `IDLE_BRAKE`; Q34 |

A new shape, proposed (TODO 160):

1. **Next**: ten rows at most.
2. **The car, waiting on a person**: the buried rows, each with a name.
3. **Safety bugs**: 93, 111, 113, 119, 116, 49b.
4. **Built in the sim, waiting for the car or the Orin**: one line each for 127-146.
5. **Ideas**: one list, with the doubles removed.

Everything from 30 Oct back (about 600 lines) moves to `docs/history/todo-2026-10.md`, and
the 0.2 sections to `docs/history/todo-2026-11-0.2.md`, after the buried rows are lifted out.

## 5. The real car

| Date | What happened | Found |
|---|---|---|
| 16 Oct | a read-only audit, one login | Humble on Ubuntu 22.04; the VLP-16 works; no IMU or GPS on the graph |
| 21 Oct | a read-only survey, 6 of 8 computers | the ROS network split; "nothing of ours runs on the car" |
| 6 Nov | drove the 2025-26 stack as is | a fuse and a board, then it moved; the Orin's CAN down; no bag |
| 6 Nov | Senura measured the IMU's noise | PRs #22, #25 |
| 9 Nov | "car day" | **no record in the repo** |
| by 11 Nov | our kit on the localization Pi | the install broke on `aav_hud`; PR #28 |
| 13 Nov | Ben: "drove and stopped well", "only had the localization update installed" | no audit, bag or notes |

What blocks the full suite, by the records:

| # | Blocker | Where |
|---|---|---|
| 1 | The car's computers run Humble; Jazzy doesn't mix | `audit-2026-10-21-car.md` |
| 2 | The Orin needs a reflash, the hardware lead's OK, and a proven restore; no backup is recorded | `docs/car-day.md` |
| 3 | The Rust ESP32 firmware isn't flashed; the C++ has no command timeout | TODO 45 |
| 4 | Which Pi is the Motor Pi (H1) | `docs/car-day.md` |
| 5 | The Orin's CAN is down, so no wheel speed | `audit-2026-11-06-car.md` |
| 6 | The people aren't named; the hardware lead's questions (H2-H5, H8-H10) are unanswered | `docs/audit-questions.md` |
| 7 | `vehicle.yaml` still holds guesses | `docs/car_measurements.md` |

## 6. Questions

| State | Which |
|---|---|
| Answered in the record, still listed open | Q11, Q16, Q18, Q19, Q22, Q23, Q31 |
| Partly answered | Q1, Q6, Q9, Q13, Q14, Q20, Q21, Q29, Q32 |
| Open | Q2-Q4, Q7, Q8, Q10, Q12, Q15, Q17, Q24, Q27, Q28, Q30, Q33, Q34 |

`conversation.md`'s "Open threads" list was brought up to date in the same commit.

## 7. The 35 designs

Seven read in full, 28 skimmed. "Built" was checked by the package or file being there, and `TODO.md`.

| State | Designs |
|---|---|
| **Built** | geofence, NDT, KISS-ICP, voxels (a trial; TODO 93 open), trials, preflight (refused on the car, 49b), debug HUD, two screens (steps 1-3), diffusion (sim), our own simulator, behaviour and speed (a sim trial) |
| **Partly** | how the car stops (the law, not the filter); one window (no algorithm list, SWAP or cancel); control plan (steps 1 and 5, in the sim); brake path (nothing chosen); research picks (1-3, not 4); the ESP32 debug brief (it says "nothing is built"; `wifi.rs` and `status-lines` exist) |
| **Not built** | one shape; knows itself; shadow mode; tracks and memory; the twelve research ideas |
| **History** | road to 0.2; the 0.2 brief; bench link (superseded by the ESP32 brief) |
| **Reviews and reports** | the drift report, the design review, naming and cleanup, the Lyrical note, the EGM review, the ESP32 second review, Fable's review of one shape |

Eleven `Status:` lines were wrong and were fixed in this commit.

**Proposed, to move to `docs/history/`:** the drift report, the design review, naming and
cleanup, the Lyrical note, the concepts brief, road to 0.2, the 0.2 brief, bench link, the
EGM review, the ESP32 second review. **A "superseded" banner:** debug HUD, one car, weak
links, research picks.

**Review findings never acted on:**

| Review | Finding | Today |
|---|---|---|
| Drift report, 25 Oct | `IDLE_BRAKE` 0.0 | unchanged; the code says it waits for the team |
| Design review, 26 Oct | `aav.dbc` DRIVE_CMD has no brake field; reversing on by default | both unchanged |
| Naming, 26 Oct | lane keeping lives under `ros/perception/`; `aav_legacy`, `aav_io_mux`, `ecolo_msgs`, `can_bridge` names | unchanged |
| EGM review, 5 Nov | cut the long designs down (TODO 109) | not cut |
| ESP32 brief, 7 Nov | rotate the WiFi password; close the C++ `/update` page | open (Q34) |

Where the designs disagree with each other: [the whole car](../designs/2026-11-14_the-whole-car.md#where-the-designs-disagree), R1-R6.
