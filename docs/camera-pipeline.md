# The camera pipeline: Pi to detections (24 October)

Ben decided it on 23 Oct ("mediamtx on the pi to the orin seems like the best choice";
[decisions](decisions.md#decided-on-22-and-23-october)). This page is how it's built, the
settings and why, what was measured on a Mac, and what to check at the car.

!!! tip "The short version"
    - The camera Pi streams **H.264** (1280x720, 15 fps, 2 Mbit/s) with **MediaMTX**.
    - On the Orin, **one program per camera** (`camera_stream`) unpacks it, undistorts
      it and runs **RF-DETR Nano**. Raw pictures never cross the network.
    - On a Mac, the whole chain takes **27 ms** from the picture to its detections:
      GStreamer with VideoToolbox, now the default. Measured with `pixi run measure stream`.
    - Calibrating the cameras at the car: `pixi run calibrate-camera`
      ([camera calibration](camera-calibration.md)).

## The picture

```text
 camera Pi (Pi 5, no ROS)                         Orin (ROS)
 ────────────────────────                         ──────────
 ribbon camera 0 ─┐                               camera_stream_front_left
 ribbon camera 1 ─┤ MediaMTX        RTSP, TCP     ┌──────────────────────────────┐
                  │ (OpenH264,  ────────────────► │ decoder (child program)      │
                  │  software)      2 Mbit/s each │  GStreamer nvv4l2decoder     │
                  │ :8554/front_left              │ undistort (OpenCV)           │
                  │ :8554/front_right             │ RF-DETR Nano (aav_detector)      │
                  │ :8889 WebRTC ──► a browser    └──────┬───────────────────────┘
                                                         ▼
                        /perception/detections   (~0.5 kB, every picture)
                        /perception/camera/front_left/preview/compressed (4 kB, 1 Hz)
                        /sensors/camera/front_left/camera_info (with a calibration)
                        /diagnostics             (its numbers, 1 Hz)
```

The same for `front_right`, in a second program.

## What is where

| Part | File | Note |
|---|---|---|
| The Pi's settings | `ros/sources/aav_camera_stream/pi/mediamtx.yml` | install: `pi/README.md` [[untested]] on the Pi |
| The node | `aav_camera_stream/camera_stream.py` | one per camera |
| Its logic, no ROS | `decoders.py`, `reader.py`, `latest.py`, `lens.py`, `timecode.py`, `models.py`, `calibration.py` | 55 tests |
| The detector | `aav_detector.detectors.load_detector` | imported, not copied |
| Start it, the car | `car.launch.py machine:=orin`: its `camera_stream` part, set in `orin.yaml` | [below](#on-the-car-carlaunchpy) |
| Start it, by hand | `ros2 launch aav_camera_stream camera_stream.launch.py host:=<pi> decoder:=gst-nvidia` | on the Orin |
| Test it | `pixi run -e ml measure stream` | a Mac, no car |
| A Mac's decoder | the `gst` environment in `pixi.toml`: `pixi install -e gst` | [below](#decoding-on-a-mac) |
| Calibrate a camera | `pixi run calibrate-camera front_left` | [camera calibration](camera-calibration.md) |

## On the car: `car.launch.py`

```text
 orin.yaml                          car.launch.py machine:=orin
 ─────────                          ───────────────────────────
 parts:                             camera_stream_front_left  ──┐
   camera_stream: true      ──────► camera_stream_front_right ──┤
   detector: false                                              │ /perception/detections
 camera_stream:                                                 ├──► stop_sign_watcher
   host: 192.168.1.108  (CAM1-PI)                               ├──► camera_obstacles (when on)
   decoder: gst-nvidia                                          └──► sensor_watchdog:
   model: models/rf-detr-nano.engine                                 front_left_stream,
                                                                     front_right_stream
```

The Pi's address is the one the 21 Oct net map found (CAM1-PI; also seen at `.102`).
Check it at the car, and fix it on the router.

### What starts, by what the Orin has

| The Orin has | What starts |
|---|---|
| JetPack's GStreamer, PyTorch, TensorRT, the engine built | both streams, RF-DETR inside each; the watchdog watches both |
| no engine yet (**today**) | no streams; a warning says how to build it (`python3 tools/build_engine.py`) |
| `detector: none` in `camera_stream` | the streams and previews, **no stop signs**; the watchdog still watches |
| no `/usr/bin/gst-launch-1.0` | no streams; a warning |

### The rules

| Rule | Why |
|---|---|
| The detector part doesn't start for a streamed camera | the pictures stay inside the stream's node: its own detector replaces it |
| Labels: `stop sign`; with `camera_obstacles`, its road users too | the stop-sign watcher, and the costmap's camera layer |
| `camera_obstacles` gets each stream's `camera_info` | its lens; none until the cameras are calibrated (a warning says so) |
| `image_raw` only with the `lane` part | the lane finder reads it, on the same computer |
| `usb_cam` and a stream can't both give `front_left` | two publishers: the output mux would stop |

### The sensor watchdog

| Input | Watches | Wrong when | Then |
|---|---|---|---|
| `front_left_stream`, `front_right_stream` | `/perception/detections`, only its camera's `frame_id` | quiet 1 s; NaN in a box | stop Nav2 and the AI slot |

### Tried on a Mac (24 Oct)

| Run | Result |
|---|---|
| `pixi run -e ml measure stream --gst` (the watchdog as on the car) | `front_left_stream` OK in all 604 of its states over 60 s; **STALE 1.1 s** after the encoder stopped |
| `car.launch.py` itself, with a stand-in machine file: two test streams from a local MediaMTX, `decoder: ffmpeg`, RF-DETR (`.pth`) | both nodes started from the part; 15 detections a second from each camera; the watchdog watched both; the right stream's sender stopped: `front_right_stream STALE`, the left one stayed OK |
| The Orin, the Pi, the real cameras | [[untested]] |

!!! note "Why the detections, not `/diagnostics`"
    The node sends detections for **every picture**, boxes or none: 15 a second, so 1 s
    of silence is a sure sign, as for a camera's pictures. `/diagnostics` comes once a
    second, and its frame rate is over 5 s. Nothing checks for a frozen picture: an
    empty list repeats on a clear road, and a stuck Pi camera stops the stream instead.

## The settings, and why

| Setting | Value | Why |
|---|---|---|
| Codec | H.264 | about 10 times smaller than JPEG; every decoder has it |
| Encoder | MediaMTX's `softwareH264` (OpenH264) | the Pi 5 has **no** H.264 hardware |
| Size, rate | 1280x720, 15 fps | enough for the detector (it shrinks to 384 px); half the Pi's work of 1080p |
| Bitrate | 2 Mbit/s (max 2.4) | 4 Mbit/s for both: 0.4% of the Orin's cable |
| Key frame | every 15 frames = 1 s | a lost or new reader waits at most 1 s for a picture |
| Profile | baseline | no B-frames: no picture waits for a later one |
| Focus | fixed, at infinity | a lens that refocuses changes its calibration |
| Transport | RTSP over TCP | wired, so nothing is lost; a frame never arrives broken |
| Who may publish | the Pi itself only | anyone may watch |
| Decoder, Orin | GStreamer `nvv4l2decoder` | NVIDIA's hardware; JetPack's own `/usr/bin/gst-launch-1.0` |
| Decoder, Mac | GStreamer `vtdec` (VideoToolbox, Apple's hardware): the default, `decoder:=auto` | ffmpeg holds back 2 frames (below) |
| Newest only | yes | a slow detector drops frames, never falls behind |
| Stamp | arrival − `delay` (0.06 s) | an **estimate**: measure it at the car |

!!! note "Why the decoder is a child program"
    NVIDIA's GStreamer plugins come with JetPack, not Pixi. The node runs JetPack's
    `gst-launch-1.0` and reads raw pictures from its pipe (2.7 MB each, 41 MB/s:
    nothing for a pipe). So the node needs no GStreamer of its own, and the same code
    runs on a Mac with the `gst` environment's GStreamer, or ffmpeg. `decoder:=opencv` decodes inside the
    process instead, with an OpenCV built with GStreamer or FFmpeg.

## Decoding on a Mac

`decoder:=auto` (the default) picks the decoder when the node starts:

```text
 the gst environment there?  ──no──►  ffmpeg, with a warning: "pixi install -e gst"
 (.pixi/envs/gst)                       (135 ms slower)
        │ yes
        ▼
 vtdec decodes 3 test pictures? ──yes──►  gst-mac: VideoToolbox, Apple's hardware
        │ no (a virtual Mac)
        ▼
 avdec_h264 there? ──yes──►  gst-soft, with a warning   ──no──►  ffmpeg, with a warning
```

| Computer | `auto` gives | Note |
|---|---|---|
| A Mac with `pixi install -e gst` | `gst-mac` (vtdec) | measured below |
| A Mac without it | `ffmpeg` | works; 135 ms slower; the log says how to fix it |
| A Linux PC with `pixi install -e gst` | `gst-soft` (avdec_h264) | [[untested]] on Linux |
| The Orin | `orin.yaml` says `gst-nvidia` | JetPack's GStreamer, not Pixi's |

!!! note "Why its own environment"
    GStreamer's RTSP plugins aren't in the default environment, and adding them there
    means re-solving it (one re-solve on 23 Oct pulled Python 3.14). The `gst`
    environment has only GStreamer: adding it to `pixi.lock` changed no other
    environment's packages (all 22 environment-platform pairs compared, 24 Oct). The node
    runs its `gst-launch-1.0` as the child program, as on the Orin.

`/diagnostics` says which decoder runs (`decoder`); `measure stream` checks a Mac gets
`gst-mac`.

## Measured on a Mac (24 Oct)

`pixi run -e ml measure stream`, Ben's Mac (Apple silicon), other helpers' work running
(load average 5-12). Gazebo's road world, the car rolling at 0.3 m/s towards the stop
sign, 60 s each. **measured**:

| Decoder | Time code → decoded | → detections published | Detector |
|---|---|---|---|
| **`vtdec`** (VideoToolbox: the default, `auto`) | **4 ms** median, 7 p95 | **27 ms** median, 38 p95 | 21 ms |
| `avdec_h264` (software, `--gst`) | 4 ms median, 6 p95, 45 worst | 27 ms median, 32 p95, 94 worst | 22 ms |
| ffmpeg (`--decoder ffmpeg`; the default until 24 Oct) | 136 ms median, 144 p95, 173 worst | 160 ms median, 174 p95, 215 worst | 22 ms |

- **vtdec and avdec_h264 are as fast** here: the gain is over ffmpeg, about 130 ms.
  The CPU was the same too (camera_stream with its decoder: 29% and 28% of a core).
- vtdec, run twice: the same medians; the worst 309 ms in the first run, 85 ms in the
  second. Both with the load average at 8-13: other helpers' Gazebo runs.

| What | Gazebo | Noisy photo (`--pattern`) |
|---|---|---|
| On the wire | 0.13 Mbit/s (simple scenery compresses well) | **2.04 Mbit/s**: a real camera's noise fills the 2 Mbit/s |
| Frames | 15.0 fps in, 15.0 worked on, 2-3 dropped a minute | the same |
| Detections | 15 a second, 489 bytes each | the same |
| Previews | 1 a second, 4.0 kB | 5.2 kB |
| Stop sign | in 902 of 906 pictures, from **25 m** down to 7 m | in every picture |
| CPU, encoder (stands in for the Pi) | 2% of a core | 7% of a core |
| CPU, `camera_stream` + its decoder | 29% of a core | 32% |

!!! warning "ffmpeg's 135 ms"
    Measured hop by hop: OpenH264 and x264 encode in 2 ms; MediaMTX adds nothing
    measurable; **ffmpeg's decoder program** holds back about 2 frames, with every
    low-delay flag tried (`-flags low_delay`, slice threads, one thread, `-fflags
    nobuffer`). GStreamer doesn't. So the Orin uses GStreamer, and so does a Mac now
    (`decoder:=auto`). OpenCV's FFmpeg reader was worse (over 1 s behind).

The time code: a row of black and white squares in the top left corner, carrying the
time in ms, drawn before encoding and read after decoding (`timecode.py`). It survives
OpenH264 at 2 Mbit/s (a test checks).

## What it can't measure here

| Not measured | Why | Estimate |
|---|---|---|
| The camera's exposure and readout, libcamera | no Pi camera here | 20-40 ms |
| The Pi 5 encoding two streams | its CPU isn't this Mac's | 20-30% of a core each (a Pi 5 core is about 3-4 times slower) |
| The cable, Pi to Orin | one computer | under 1 ms |
| The Orin's hardware decoder | no Orin here | a few ms |
| RF-DETR Nano in TensorRT on the Orin | no Orin here | to time (PLAN T8.3) |

So on the car, camera to detections: about **50-100 ms** (a guess).

## At the car: what to check

| Check | How | Good |
|---|---|---|
| Which camera is which | `rpicam-hello --list-cameras` on the Pi | swap `rpiCameraCamID` if left is 1 |
| The Pi's CPU, two streams | `top` while the Orin reads both | under 50% of the Pi in all; `vcgencmd measure_temp` under 80 °C |
| Bitrate | `curl http://127.0.0.1:9997/v3/paths/list` on the Pi, twice, 10 s apart | about 2 Mbit/s each |
| The Orin decodes | `ros2 launch aav_camera_stream camera_stream.launch.py host:=<pi> decoder:=gst-nvidia detector:=none`, then `/diagnostics` | `fps_in` 15 |
| Delay, end to end | film a phone's stopwatch next to the Orin's screen, which shows the preview (or the stopwatch itself) | under 150 ms |
| Stamp error | the same: the stopwatch in the picture vs the detection's stamp | set `delay` to the difference |
| Clock | `chronyc tracking` on the Pi and the Orin | under 1 ms apart |
| The car's launch | `pixi run car orin`, after the engine is built; then `ros2 topic echo /safety/sensor_watchdog/state` | no `front_left_stream` or `front_right_stream` in it; pull the Pi's cable: `STALE` within 1-2 s |
| Calibrate each camera | `pixi run calibrate-camera front_left --host <pi>`, then `front_right` ([how](camera-calibration.md)) | the window's numbers green; the files committed |
| The stop-sign distance | a real stop sign 4.7 m ahead (tape measure); `stop_sign_watcher`'s log | it locks at 4.5-5 m; its log names each camera's focal length |
| A Mac decoding | on the Mac: `ros2 launch aav_camera_stream camera_stream.launch.py host:=<pi> cameras:=front_left detector:=none`; its log | `by gst-mac`, not a warning about ffmpeg |

## Still open

| Item | Note |
|---|---|
| Calibrations | the ribbon cameras have none yet: `config/cameras.yaml` leaves them empty (no undistortion). **Ready for the car:** `pixi run calibrate-camera` ([how](camera-calibration.md)). The 2024-25 `left.ini` is another camera's (Q22) |
| `car.launch.py` | done ([above](#on-the-car-carlaunchpy)): [[untested]] on the Orin. Needs the engine built there first |
| The sensor watchdog | done: watches each stream's detections. [[untested]] at the car |
| The stop sign's size | done (24 Oct): a distance, 4.7 m, from each camera's focal length ([perception](perception.md#how-far-the-boxs-height-and-the-lens-24-oct)). Needs the calibration, then a check with a real sign at the car |
| A Mac's decoder | done (24 Oct): GStreamer with VideoToolbox by default ([above](#decoding-on-a-mac)) |
| Lane finder and HUD | read `image_raw` too: `publish_image:=true` on the Orin (no network), or the preview |
| A rear camera | the QNX Pi's doesn't work today; a USB camera's `ffmpeg` example is in `pi/mediamtx.yml` |
