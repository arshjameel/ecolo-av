# Run it on a Mac or Linux

Pixi installs ROS 2 Jazzy into this folder: no Homebrew, no admin rights, and the same
steps on a Mac and on Linux. Then three commands drive a turtle like a car.

## What you need

| | Mac | Linux |
|---|---|---|
| System | macOS 14 (Sonoma) or newer; Apple Silicon or Intel | 64-bit x86 or ARM: Ubuntu, AlmaLinux … |
| Disk space | about 7 GB | about 7 GB |
| Admin rights | no | no |
| First | `xcode-select --install`: Apple's git and compiler | nothing |
| Tested | [[works]] on an Apple Silicon Mac, 10 Oct | [[works]] on oirp-spss, 9 Oct |

## 1. Install pixi (once)

```bash
curl -fsSL https://pixi.sh/install.sh | sh
```

Then open a new terminal, so that it finds `pixi`.

## 2. Get the code

The repository is on GitHub, `AAV-2025-2026/ecolo-av` (public):

```bash
git clone git@github.com:AAV-2025-2026/ecolo-av.git ecolo-av
cd ecolo-av
```

Later, `git pull` fetches new work. To send changes, use your own fork and pull requests:
[Git](git.md).

## 3. Install ROS, then build

```bash
pixi install       # the first time: downloads ROS 2 Jazzy, about 7 GB
pixi run build     # compiles our packages, in about 15 s
```

## 4. Drive the turtle

| Terminal 1 | Terminal 2 |
|---|---|
| `pixi run turtle` | `pixi run keyboard` |

| Key | Does |
|---|---|
| `i` / `,` | forward / reverse |
| `u` / `o` | forward, turning left / right |
| `k` | stop |
| `q` / `z` | faster / slower (ego caps it at 1.0 m/s) |

Hold the key down: the turtle stops about half a second after you let go. The path is
keyboard → `/drivers/keyboard/cmd_vel` → twist_mux → ego → `/ego/actuator_cmd` → output_mux → turtle.

!!! note "Is this different from the regular turtlesim?"
  No, it's the stock turtlesim. Our `turtlesim_adapter` steers it with the car's bicycle
  model, so like the car, it can't turn on the spot.

## 5. More commands

| Command | Does |
|---|---|
| `pixi run graph` | rqt_graph: the nodes and topics, live |
| `pixi run check` | compares the running graph with our contract (`pixi run check sim` for Gazebo) |
| `pixi run view` | the car model in RViz |
| `pixi run sim` | Gazebo instead of the turtle ([Simulation](simulation.md)) |
| `pixi run bench` | the real car's software, with stand-ins for the hardware ([Bench mode](bench.md)) |
| `pixi run bench-check` | starts the bench and tests the whole graph: about 2 minutes |
| `pixi run get models`, then `pixi run detect picture:=ros/perception/aav_detector/test/stop_sign.jpg` | RF-DETR Nano finds a stop sign in a picture (`detector:=yolo` for YOLO; [Perception](perception.md)) |
| `pixi run tools/ros.sh python3 tools/graph_picture.py` | saves the running graph as a picture |
| `pixi run rviz` | RViz with the driving layout |
| `pixi run estop`, `pixi run clear estop` | stop every driver; let them go again |
| `pixi run doctor` | can this computer join the car's network? What to fix |
| `pixi run test` | every test |
| `pixi run docs` | rebuilds these pages |
| `pixi install -e mcp` | the read-only MCP server's environment, for Claude or another AI app ([AI apps](mcp.md)) |
| `pixi shell`, then `source install/setup.bash` | a terminal for any `ros2` command (`setup.zsh` in a Mac's zsh) |

## On a Mac

[Your Mac](mac.md) has the Mac's own steps: copying the code over, joining the car's
network, driving and watching the car, and Gazebo.

## When something goes wrong

| Symptom | Try |
|---|---|
| `pixi: command not found` | open a new terminal, or run `~/.pixi/bin/pixi` |
| "Not built yet" | `pixi run build` |
| The turtle doesn't move | click into the keyboard terminal, and hold the key. Then see [when nothing moves](simulation.md#when-nothing-moves). |
| Odd build errors after `git pull` | delete `build/`, `install/` and `log/`, then `pixi run build` |
| Two computers can't see each other | the same `ROS_DOMAIN_ID` on both: see [Platforms](platforms.md) |
| Nodes hang at start-up, or the sim never spawns the car | `pixi run fastdds shm clean`: it removes Fast DDS's shared-memory files left by programs that died (515 of them on 15 Oct). Only with `AAV_RMW=fastdds`: Cyclone, the default since 23 Oct, leaves none. Stop every ROS program first |
| Another computer can't see my sim or bench | that's on purpose: the stand-ins stay on their computer. To share one: `AAV_DISCOVERY=SUBNET pixi run sim` |
