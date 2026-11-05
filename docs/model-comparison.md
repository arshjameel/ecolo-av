# Newer models, on the same pictures and tasks (S6)

plan.md item S6, 16 Oct 2026. Two questions:

1. **Seeing**: is YOLO26 or RF-DETR better than our YOLO11 at people and stop signs?
2. **Plain English**: is a newer local language model better than Qwen3 at turning
   "drive 10 m forward" into the drive server's tool calls?

> **The short version**
> - **Now: RF-DETR Nano is the car's detector** (Ben, 23 Oct: "I thought we were skipping
>   yolo and using the other one"), the detector node's default, in TensorRT on the Orin.
>   YOLO stays as `detector:=yolo` ([Perception](perception.md#models)).
> - **Why RF-DETR Nano won (16 Oct).** It saw Gazebo's stop sign from
>   **20 m** (YOLO11n: 10 m) and found people in 4 of 5 photos (YOLO11n: 3), with no
>   false finds. It's **Apache-2.0**, so the AGPL question goes away. On a Mac it is as
>   fast as YOLO11n (16-17 ms vs 12-21 ms). Its cost: 12 times the weights (30 M), and
>   2.5 times slower on a CPU alone. Not tried on the Orin yet.
> - **YOLO26 brings nothing here.** YOLO26n scored the same as YOLO11n, picture for
>   picture, and has the same AGPL-3.0 licence.
> - **Language: Gemma 4 E4B planned all 23 requests right** (Qwen3-4B, our default: 21),
>   at 0.5 s a step, in 5 GB, Apache-2.0. Ministral 3 3B (22, in 2.8 GB) is the small
>   runner-up. The default stays Qwen3-4B until the team agrees: 23 requests is a small test.
> - **New: an OpenAI-compatible backend** (`aav_language/llm_openai.py`), so the same agent
>   can use llama.cpp or TensorRT-LLM on the Orin. Tried through `mlx_lm.server`: 22 of 23.

## What was run, and what wasn't

| | Run? | Where |
|---|---|---|
| 6 detectors on 8 photos and 7 Gazebo frames, CPU, Metal (MPS) and Core ML | **yes** | Ben's Mac (M4 Max, 48 GB), 16 Oct |
| The Gazebo frames | **yes**: `tools/sim_frames.py`, road world, headless | the same Mac |
| 16 language models on the 23-request test, through MLX | **yes** | the same Mac |
| One model through the new OpenAI-compatible backend (`mlx_lm.server`) | **yes** | the same Mac |
| Anything on the Orin: TensorRT, llama.cpp with CUDA, TensorRT-LLM | **no**: no Orin here | [[untested]] |
| RF-DETR inside the detector node (`aav_detector`) | **yes**, since 20 Oct, and the default: PyTorch, Core ML (24 Oct) and TensorRT (written, [[untested]]) | Ben's Mac; [Perception](perception.md#models) |
| The car's own camera pictures | **no**: none recorded yet (C1, C6) | [[needs-you]] |
| Ollama, llama.cpp on the Mac | **no**: neither is installed here | |

Timings were taken while other jobs (Gazebo runs, downloads) shared the Mac: treat them
as rough, within about 20 %.

## 1. Seeing: people and stop signs

### The result

| Model | Person (photos) | Stop sign (photos + Gazebo) | Gazebo sign seen from | ms CPU | ms Metal | ms Core ML | Weights | Licence |
|---|---|---|---|---|---|---|---|---|
| YOLO11n (`detector:=yolo`) | 3/5 (0.65) | 6/10 (0.54) | 10 m | 30 | **12** | 21 | 2.6 M, 6 MB | AGPL-3.0 |
| YOLO11s | **5/5** (0.76) | 7/10 (0.61) | 15 m | 54 | 15 | 37 | 9.5 M, 19 MB | AGPL-3.0 |
| YOLO26n | 3/5 (0.65) | 6/10 (0.52) | 10 m | 32 | 13 | 20 | 2.6 M, 6 MB | AGPL-3.0 |
| YOLO26s | 4/5 (0.76) | 7/10 (0.63) | 15 m | 56 | 15 | 36 | 10.0 M, 20 MB | AGPL-3.0 |
| **RF-DETR Nano** | 4/5 (0.72) | **8/10** (0.70) | **20 m** | 74 | 16 | **17** | 30.5 M, 122 MB | **Apache-2.0** |
| RF-DETR Small | **5/5** (0.82) | **8/10** (0.73) | **20 m** | 122 | 20 | 22 | 32.1 M, 128 MB | **Apache-2.0** |

| Column | Means |
|---|---|
| Person, Stop sign | pictures where the model found it at 0.5 or more (the detector node's `confidence`), of the pictures that have one; in brackets, the mean of its best score in each |
| Gazebo sign seen from | the farthest frame where the sign scored 0.5 or more (camera to sign) |
| ms | per picture, the library's pre- and post-processing included; the median of 5 runs |
| Weights | parameters, and their size at 32-bit. RF-DETR's download is a 366-386 MB training checkpoint |

- **No model made a false find**: nothing scored 0.5 on the empty street, or a sign
  among the people.
- **Every model found all 3 real stop signs**, the French ARRÊT in Gatineau too. The
  difference is distance: Gazebo's sign at 15 and 20 m.
- **Core ML's scores are PyTorch's** to within about 0.06, for YOLO and RF-DETR. Once
  that tipped a picture: YOLO11s found 4 of 5 people through Core ML, 5 on the CPU. The
  table's scores are the CPU's; the Metal ones are the same.
- The watcher stops the car 4.7 m before the sign (a box 55 px tall with Gazebo's lens),
  so every model sees the sign in time today. Seeing it at 20 m would let the car slow
  down earlier.

### Per picture (best person / stop-sign score, CPU)

| Picture | Has | YOLO11n | YOLO11s | YOLO26n | YOLO26s | RF-DETR N | RF-DETR S |
|---|---|---|---|---|---|---|---|
| dashcam-stop | stop sign | 0.00 / 0.88 | 0.00 / 0.94 | 0.00 / 0.84 | 0.00 / 0.93 | 0.00 / 0.91 | 0.00 / 0.92 |
| gatineau-arret | stop sign | 0.17 / 0.89 | 0.00 / 0.92 | 0.18 / 0.89 | 0.18 / 0.93 | 0.32 / 0.90 | 0.28 / 0.95 |
| sf-all-way-stop | both | 0.50 / 0.94 | 0.66 / 0.96 | 0.48 / 0.95 | 0.79 / 0.93 | 0.38 / 0.95 | 0.69 / 0.96 |
| sf-market-from-above | person | 0.27 / 0.00 | 0.54 / 0.00 | 0.44 / 0.11 | 0.39 / 0.00 | 0.64 / 0.00 | 0.77 / 0.00 |
| petersfield-walkers | person | 0.88 / 0.00 | 0.89 / 0.00 | 0.83 / 0.00 | 0.87 / 0.00 | 0.89 / 0.00 | 0.91 / 0.00 |
| barcelona-crossing | person | 0.75 / 0.00 | 0.82 / 0.00 | 0.71 / 0.00 | 0.85 / 0.00 | 0.81 / 0.00 | 0.85 / 0.00 |
| dyott-street-crossing | person | 0.84 / 0.00 | 0.88 / 0.00 | 0.81 / 0.00 | 0.89 / 0.00 | 0.87 / 0.00 | 0.90 / 0.00 |
| mississauga-empty | nothing | 0.00 / 0.00 | 0.00 / 0.00 | 0.15 / 0.00 | 0.00 / 0.00 | 0.26 / 0.00 | 0.00 / 0.00 |
| Gazebo, 40 m | stop sign | 0.00 / 0.00 | 0.00 / 0.00 | 0.00 / 0.00 | 0.00 / 0.00 | 0.00 / 0.14 | 0.18 / 0.00 |
| Gazebo, 30 m | stop sign | 0.00 / 0.00 | 0.00 / 0.00 | 0.00 / 0.00 | 0.00 / 0.00 | 0.00 / 0.00 | 0.00 / 0.28 |
| Gazebo, 20 m | stop sign | 0.00 / 0.00 | 0.00 / 0.00 | 0.00 / 0.00 | 0.00 / 0.00 | 0.00 / **0.51** | 0.00 / **0.65** |
| Gazebo, 15 m | stop sign | 0.00 / 0.00 | 0.00 / **0.55** | 0.00 / 0.00 | 0.00 / **0.83** | 0.00 / **0.79** | 0.00 / **0.85** |
| Gazebo, 10 m | stop sign | 0.00 / 0.89 | 0.00 / 0.90 | 0.00 / 0.72 | 0.00 / 0.81 | 0.00 / 0.91 | 0.00 / 0.88 |
| Gazebo, 7 m | stop sign | 0.00 / 0.90 | 0.00 / 0.92 | 0.00 / 0.89 | 0.00 / 0.89 | 0.00 / 0.92 | 0.00 / 0.92 |
| Gazebo, 5 m | stop sign | 0.00 / 0.93 | 0.00 / 0.93 | 0.00 / 0.86 | 0.00 / 0.93 | 0.00 / 0.92 | 0.00 / 0.93 |

"sf-market-from-above" is a hard one on purpose: tiny people, seen from a roof. 15
pictures are too few to rank models closely: they show big differences only.

### The pictures

Photos from Wikimedia Commons, 640 px wide. They are **not in git**: `--fetch`
downloads them, and checks each licence on Commons first.

| Name | Commons file | By | Licence |
|---|---|---|---|
| dashcam-stop | Jackson Rd wb Wallace Rd intersection solar stop sign dashcam.jpg | Michael Rivera (Mjrmtg) | CC0 |
| gatineau-arret | Tiled intersection with stop sign in Gatineau.jpg | 137a | CC BY 4.0 |
| sf-all-way-stop | All-way stop intersection in San Francisco.jpg | Hiya111 | CC BY-SA 4.0 |
| sf-market-from-above | Crosswalk of Market at Third, San Francisco.jpg | Dllu | CC BY-SA 4.0 |
| petersfield-walkers | Pedestrians and a cyclist in the Petersfield Road - geograph.org.uk - 3005700.jpg | Basher Eyre | CC BY-SA 2.0 |
| barcelona-crossing | Pedestrians crossing the street in Barcelona DSF0993.jpg | Levi Meir Clancy | CC0 |
| dyott-street-crossing | Pedestrians crossing in Dyott Street - geograph.org.uk - 1105273.jpg | Basher Eyre | CC BY-SA 2.0 |
| mississauga-empty | Crosswalk (6984572731).jpg | ^ Missi ^ (Flickr) | CC BY 2.0 |

The Gazebo frames: the front left camera (640 x 360) on the road world's centre line,
40, 30, 20, 15, 10, 7 and 5 m from the stop sign. Gazebo has no people in it yet.

### Licences: what AGPL-3.0 means for us

| | Ultralytics (YOLO11, YOLO26) | RF-DETR Nano, Small (Roboflow) |
|---|---|---|
| Code | AGPL-3.0 | Apache-2.0 |
| The trained weights | AGPL-3.0, Ultralytics says, our own fine-tuned ones too | Apache-2.0 |
| Our code that uses it | must be shared under AGPL-3.0 terms whenever we give the program to anyone, **or let people use it over a network** | any licence |
| A closed or paid version later | needs an Ultralytics Enterprise licence | allowed: keep the notices |

- **Our repo is public**, so sharing the source is easy. But it has **no licence file**
  yet: by default that means "all rights reserved", which doesn't fit with AGPL code in
  the same program. With YOLO, the team should put the repo, or at least `aav_detector`,
  under AGPL-3.0 (a team decision; not legal advice).
- **The network clause**: if the HUD's web page ever shows YOLO's boxes to people over the
  network, that counts as use over a network.
- **The 2024-25 `best.pt`** was trained with Ultralytics: Ultralytics counts it as AGPL too.
- **RF-DETR's larger models** (XLarge, 2XLarge, in `rfdetr_plus`) have a different,
  Roboflow licence. Nano, Small, Medium, Base and Large are Apache-2.0.

### Exporting: Orin and Mac

| | Orin (TensorRT) | Mac (Core ML) |
|---|---|---|
| YOLO11, YOLO26 | `yolo export format=engine half=True` on the Orin: `tools/build_engine.py` (written, [[untested]]). YOLO26 needs no NMS step, so the engine is all on the GPU | `yolo export format=coreml`: **ran**, 2 s each, 5-37 MB |
| RF-DETR | `tools/build_engine.py` on the Orin (24 Oct): `model.export(format="onnx")`, then TensorRT's own builder in FP16 (what `trtexec --fp16` does). Not `rfdetr[tensorrt]`: it brings PyPI's TensorRT, not JetPack's. No NMS step either. [[untested]] | `pixi run -e ml python3 tools/export_model.py --format coreml`: `model.export(format="coreml")`, **ran**, 8 s, 108 MB. The detector node runs it: 10 ms a picture, against 16 on Metal |

An engine runs only on the GPU and TensorRT version that built it: build each on the
Orin itself.

### Installing

| Package | Where from | Licence | Result |
|---|---|---|---|
| ultralytics 8.4 (YOLO11, YOLO26) | conda-forge, already in the ml environment | AGPL-3.0 | works |
| rfdetr 1.11.2 | conda-forge | Apache-2.0 | **works** beside Ultralytics, the same PyTorch 2.13-2.14. Added to `pixi.toml`'s ml feature |
| transformers 5, supervision, peft, pydantic | come with rfdetr | Apache-2.0, MIT | |

The run itself used a scratch Pixi environment in `~/aav-models/s6-env` with the same
packages, so the shared `.pixi` wasn't changed under other people's runs.

### Run it again

```bash
pixi run python3 tools/sim_frames.py --out ~/aav-models/frames      # Gazebo frames (1 min)
pixi run -e ml python3 tools/compare_detectors.py --fetch           # the photos
pixi run -e ml python3 tools/compare_detectors.py --out ~/aav-models/detectors.json
```

The weights go in `~/aav-models/weights/` (their URLs are in the script, `MODELS`), never
in git. The script checks each file's SHA-256 (MD5 for RF-DETR, as rfdetr pins them)
before loading it: these files are Python pickles, so a wrong one could run code.

## 2. Plain English: newer local language models

### The result

The node's own test: 23 requests (`aav_language/eval_set.py`), each planned against a
pretend car through the real drive server, greedy decoding, through MLX on the Mac. All
`mlx-community/<name>-4bit`. **Bold** = the best in the column.

| Model | Correct (of 23) | Moves (12) | At once (4) | Refusals (7) | s per step | s per request | Memory GB | Licence |
|---|---|---|---|---|---|---|---|---|
| **gemma-4-e4b-it** (new) | **23** | **12** | **4** | **7** | 0.53 | 1.5 | 5.1 | Apache-2.0 |
| Ministral-3-3B-Instruct-2512 (new) | 22 | 11 | **4** | **7** | 0.70 | 1.4 | **2.8** | Apache-2.0 |
| granite-4.1-8b (new) | 22 | **12** | **4** | 6 | 2.2 | 4.9 | 6.2 | Apache-2.0 |
| Qwen3.6-35B-A3B (new, 20 GB) | 22 | **12** | **4** | 6 | 0.90 | 2.3 | 21.0 | Apache-2.0 |
| Qwen3-4B-Instruct-2507 (**our default**) | 21 | **12** | **4** | 5 | 0.91 | 2.0 | 3.3 | Apache-2.0 |
| Qwen3-8B | 21 | **12** | **4** | 5 | 1.7 | 3.8 | 5.7 | Apache-2.0 |
| gemma-4-26b-a4b-it (new, 15 GB) | 21 | **12** | **4** | 5 | 0.76 | 1.6 | 15.4 | Apache-2.0 |
| gemma-4-e2b-it (new) | 20 | 11 | 2 | **7** | **0.20** | **0.4** | 3.5 | Apache-2.0 |
| Qwen3.5-4B | 19 | **12** | **4** | 3 | 1.0 | 2.2 | 3.9 | Apache-2.0 |
| Ministral-3-8B-Instruct-2512 (new) | 19 | 10 | **4** | 5 | 1.7 | 4.4 | 5.8 | Apache-2.0 |
| Qwen3.5-9B (new) | 18 | 9 | **4** | 5 | 2.3 | 7.9 | 6.7 | Apache-2.0 |
| Qwen3.5-2B (new) | 14 | 10 | 3 | 1 | 0.60 | 2.7 | 2.5 | Apache-2.0 |
| granite-4.1-3b (new) | 13 | 9 | 3 | 1 | 0.89 | 2.2 | 3.1 | Apache-2.0 |
| Llama-3.2-3B-Instruct (new) | 10 | 5 | **4** | 1 | 0.88 | 4.2 | 2.9 | Llama 3.2 |

Run 16 Oct. The three Qwen rows above them in plain-english.md were run again: the same
scores to the request (greedy decoding). Llama-3.1-8B (17) and Llama-3.2-1B (8) are from
that page, not run again.

### What went wrong, model by model

| Model | Its mistakes |
|---|---|
| gemma-4-e4b | none |
| Ministral-3-3B | "go forward 5 m then turn left 90 degrees": only the drive; it asked for the radius instead |
| granite-4.1-8b | "go as fast as you can": a 10 m drive at 1 m/s |
| Qwen3.6-35B-A3B | "go as fast as you can": 200 m at 1 m/s |
| Qwen3-4B-2507 (default) | "30 m at 5 m/s" shrunk to 1 m/s; "go somewhere fun": a turn |
| gemma-4-26b-a4b | "go as fast as you can" and "go somewhere fun": drives |
| gemma-4-e2b | **didn't call stop or e-stop** for "stop the car now please" and "EMERGENCY! kill it": not safe enough |
| Qwen3.5-9B | repeated a move up to 8 times; drove for "go somewhere fun" |
| Llama-3.2-3B | numbers as text ("100"), moves twice |

Typing just "stop" or "estop" never reaches a model (`react.FAST`), so a model's miss
there matters less; the agent's Ctrl-C sends `stop()` too.

### Two models needed the agent to learn their format

Gemma 4 and Ministral 3 first scored **0**: they answered in their own tool-call formats,
which the agent couldn't read. `react.parse_reply` now reads both:

```text
Gemma 4      <|tool_call>call:turn{angle_deg:45,direction:<|"|>right<|"|>}<tool_call|>
Ministral 3  [TOOL_CALLS]turn[ARGS]{"angle_deg": 45, "direction": "right"}
```

### The recommendation

- **Gemma 4 E4B is the best candidate for the default**: all 23, quick, Apache-2.0. Its
  official licence is Apache-2.0 (`google/gemma-4-e4b-it`); the MLX copy's page still
  says "gemma": go by Google's. Its download is 5.2 GB.
- **Before switching**: add 20 or so requests to `eval_set.py` (one or two answers either
  way is noise at 23), and try it on the Orin.
- **On the Orin (AGX)**: the 3-8 B models fit easily. The two 15-20 GB mixture-of-experts
  models fit a 64 GB Orin too, and are quick for their size (only 3-4 B weights work per
  word), but scored no better.

### The same agent on the Orin: the OpenAI-compatible backend

MLX runs only on Apple silicon. The Orin needs llama.cpp (CUDA) or TensorRT-LLM. All of
them, and Ollama, serve the same OpenAI `/v1/chat/completions` API, so one small client
covers every one: `aav_language/llm_openai.py` (standard library only).

| Server | Machine | Licence |
|---|---|---|
| `llama-server -m <model>.gguf --jinja` (llama.cpp) | Orin (CUDA), Mac, Linux | MIT |
| `trtllm-serve <model>` (TensorRT-LLM) | Orin (JetPack) | Apache-2.0 |
| `ollama serve` | Orin, Mac, Linux | MIT |
| `mlx_lm.server --model <id>` | Mac | MIT |

```bash
pixi run -e llm tools/ros.sh python3 -m aav_language.run_eval --backend openai --base-url http://127.0.0.1:8080/v1 --models <name>
```

**Tried on the Mac** through `mlx_lm.server` with Gemma 4 E4B: **22 of 23**, 0.2 s a step
(the server keeps the prompt cached). The one miss, the U-turn, came back empty: the
server's own reading of the tool call failed. [[untested]] on the Orin.

### Run it again

```bash
pixi run -e llm tools/ros.sh python3 -m aav_language.run_eval                                   # the five models on plain-english.md
HF_HOME=~/aav-models/hf pixi run -e llm tools/ros.sh python3 -m aav_language.run_eval --models newer --out ~/aav-models/llm.json
```

`--models newer` is the S6 list (`run_eval.NEWER`). The models download into the Hugging
Face cache (here `~/aav-models/hf`): safetensors only, nothing that runs code.

## 3. Two open-source agents: what we could borrow

Ben mentioned two agent harnesses. Both are **MIT**-licensed TypeScript; we read them as
references only, and copied no code.

| | pi (Mario Zechner) | opencode |
|---|---|---|
| Repo | `badlogic/pi-mono`, now `earendil-works/pi` | `sst/opencode`, now `anomalyco/opencode` |
| The loop | call the model, run its tool calls, add the results, repeat until no tool calls | the same, one step at a time, with a step limit per agent |
| Tool calls in one reply | run together, unless a tool says "one at a time" | run together |
| Bad arguments | checked against the tool's JSON schema; the error, field by field, goes back to the model | the same, "update the arguments and call the tool again" |
| Asking a person | none by design (run it in a container) | per tool: allow, ask or deny; the last matching rule wins |
| Stopping | an abort signal reaches every tool; messages typed meanwhile are queued ("steer") | interrupt cancels running tools; a queue for typed messages |
| Too many steps | no limit | tools switched off; the model must sum up what it did and what's left |
| Local models | any OpenAI-compatible server (Ollama, llama.cpp `--jinja`) | the same; raise Ollama's context to 16-32 k for tool calls |
| Where | `packages/agent/src/agent-loop.ts` | `packages/core/src/session/runner/llm.ts`, `permission.ts` |

What our node (`react.py`) could borrow:

| Idea | From | For us |
|---|---|---|
| Check arguments against the tool's schema, send the errors back once | both | fewer "unreadable" steps from small models |
| Allow / ask / deny per tool | opencode | what we do by hand: status, stop, estop allowed; moves asked (the "yes") |
| At the step limit, switch tools off and make the model sum up | opencode | a clear answer instead of "I stopped planning" |
| Never run a reply cut off by the token limit | pi | a half-written move must not run |
| Typed text during a drive is queued, not lost; one abort reaches everything | pi | "stop" while driving already works; "then turn left" could queue |
| One OpenAI-compatible client | both | **done**: `llm_openai.py` |

## Files

| File | What |
|---|---|
| `tools/compare_detectors.py` | the detector comparison; tests in `tools/tests/test_compare_detectors.py` |
| `tools/sim_frames.py` | Gazebo's stop sign from 7 distances |
| `ros/drivers/aav_language/aav_language/run_eval.py` | the language test: `--models newer`, `--backend openai` |
| `ros/drivers/aav_language/aav_language/llm_openai.py` | the OpenAI-compatible backend; tests in `test/test_llm_openai.py` |
| `ros/drivers/aav_language/aav_language/react.py` | reads Gemma 4's and Mistral 3's tool calls |
| `~/aav-models/` (not in git) | weights, photos, frames, every score as JSON |
