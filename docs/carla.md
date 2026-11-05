# CARLA: photo-real towns, and Carleton's streets

CARLA is a driving simulator built on Unreal Engine: real-looking towns, traffic and
cameras. Gazebo stays our everyday simulator. CARLA is for what Gazebo can't do well:
testing the cameras and the detectors on real-looking streets.

**Status (15 Oct):** the research is done and the ROS side is built and tested without a
server. **It has never run against a CARLA server**: there is none here, and CARLA has
no Mac version. Running it needs the team's NVIDIA Linux computer, or a cloud GPU.

## The picture

```
 NVIDIA Linux computer                          the same computer (or another Linux PC)
 +---------------------------+   TCP 2000-2001  +----------------------------------+
 | CARLA server, headless     | <-------------> | carla_adapter (CARLA's Python    |
 | (Docker or the package)    |  CARLA's own    |   client) -> our topics          |
 | Town10, or Carleton's roads|  protocol       | mux, ego, EKFs, Nav2, watchdog   |
 +---------------------------+                  +----------------------------------+
                                                     | ROS 2 (only if you share it)
                                                     v
                                                 a Mac: Foxglove / Lichtblick, RViz
```

The adapter talks to CARLA over TCP, not over ROS. So ROS can stay on one computer
while CARLA runs anywhere it can reach.

## Which CARLA: 5 (Unreal Engine 5) or 4?

Ben asked for the Unreal Engine 5 version. Both work with our adapter; they're good at
different things today.

| | CARLA 0.10.0 (Unreal Engine 5.5) | CARLA 0.9.16 (Unreal Engine 4.26) |
|---|---|---|
| Released | 19 Dec 2024: still the only UE5 release | 16 Sep 2025: the newest release of any kind |
| Looks | best: new lighting, remodelled Town10 | good |
| Maps | **Town10 and Mine01 only** | Towns 1-7, 10, 11, 12, 13, 15 |
| Weather | daylight only | all |
| Carleton's map (OpenStreetMap, OpenDRIVE) | **no**: "not supported" in 0.10 | **yes** |
| Digital Twin Tool | **no** | yes (experimental) |
| Cars | 11 | all of 0.9's, many more |
| Built-in ROS 2 | yes, but stalls on Jazzy | yes, but stalls on Jazzy |
| Python client | in the package (3.8-3.12), not on PyPI | `pip install carla==0.9.16` (3.10-3.12) |
| GPU | RTX 3070 or better, **16 GB** of video memory | less: 0.9's docs ask for 6-8 GB |
| Disk | 130 GB | much less |
| Docker | `carlasim/carla:0.10.0` (10.8 GB) | `carlasim/carla:0.9.16` |

**Our advice:** start with 0.10 for Town10's cameras (Ben's choice), and use 0.9.16 for
Carleton's streets until UE5 can import maps. The same adapter and the same command run
both: only the server changes.

## What you need

| Need | For |
|---|---|
| An NVIDIA RTX GPU: 16 GB of video memory for 0.10 (RTX 3070 is the floor); less for 0.9.16 | the server |
| Ubuntu 22.04 (or Windows 11); the `ue5-dev` branch also lists 24.04. NVIDIA driver 550 or newer | the server |
| 32 GB of RAM, 130 GB of disk (0.10) | the server |
| A Linux x86_64 computer with our pixi environment: the same one is fine | the adapter and the stack |
| CARLA's Python client, **the server's exact version** | the adapter |

Not possible: a Mac (no build, no Python client: there is no macOS or arm64 wheel), and
the car's Jetson Orin (ARM: CARLA ships only Linux x86_64 and Windows).

## 1. Start the server, headless

```bash
# CARLA 0.10 (Unreal Engine 5), from the package
./CarlaUnreal.sh -RenderOffScreen -nosound
# or Docker (check the image's own start command first: untested here)
docker run --rm --gpus all --net=host carlasim/carla:0.10.0 \
  bash CarlaUnreal.sh -RenderOffScreen -nosound
# CARLA 0.9.16 (Unreal Engine 4)
./CarlaUE4.sh -RenderOffScreen
```

It listens on TCP ports **2000 and 2001**. Open both if a firewall is in the way.

## 2. Give the adapter CARLA's Python client

Our pixi environment has no pip, and the client isn't a ROS package. The wheel is a zip
file, so unpack it beside the repo and point Python at it:

```bash
# CARLA 0.10: the wheel is in the package's PythonAPI folder (cp312 = Python 3.12)
unzip <carla-0.10>/PythonAPI/dist/carla-0.10.0-cp312-*.whl -d ~/carla-client
# CARLA 0.9.16: from PyPI
python3 -m pip download carla==0.9.16 --no-deps --only-binary=:all: \
  --python-version 3.12 --platform manylinux_2_31_x86_64 -d /tmp/carla-wheel
unzip /tmp/carla-wheel/carla-0.9.16-*.whl -d ~/carla-client
export PYTHONPATH=~/carla-client:$PYTHONPATH
```

Without it, `carla_adapter` stops with one line saying how to get it. The client and the
server must be the same version: the adapter warns when they aren't.

## 3. Run the stack

```bash
pixi run build
pixi run carla-car host:=127.0.0.1                          # the server on this computer
pixi run carla-car host:=192.168.1.50 town:=Town10HD_Opt    # another computer, Town10
pixi run carla-car host:=127.0.0.1 osm:=maps/carla/carleton.osm   # Carleton (0.9.16)
```

| Argument | Default | Does |
|---|---|---|
| `host`, `port` | 127.0.0.1, 2000 | where the server is |
| `town` | '' (keep the server's) | load one of its maps |
| `osm`, `xodr` | '' | build Carleton's roads from a file (0.9.x only) |
| `spawn` | '' (spawn point 0) | `3` = spawn point 3; `x,y,yaw` = a place in our map frame |
| `frame` | auto | `world`: CARLA's world is our map (Carleton); `spawn`: the start is (0, 0) facing east |
| `control` | ackermann | CARLA's Ackermann controller; `pedals`: our speed loop |
| `sync`, `fixed_delta` | true, 0.05 s | CARLA waits for our step, 20 steps a second |
| `realtime` | true | don't let simulated time outrun the clock |
| `cameras`, `radar`, `lidar` | true | which sensors to put on the car |
| `nav`, `rviz`, `foxglove`, `teleop` | true, false, false, false | as in `pixi run sim` |

### The network rule

`pixi run carla-car` runs through `tools/ros.sh --local`, like every stand-in: ROS
discovery stays on this computer, so the pretend car can never meet the real one. That's
safe **whether or not the server is on this computer**, because the adapter reaches CARLA
over TCP, not ROS.

To watch from a Mac, share ROS on purpose, on a network with no real car on the same
`ROS_DOMAIN_ID`:

```bash
AAV_DISCOVERY=SUBNET pixi run carla-car host:=127.0.0.1 foxglove:=true
```

The bridge listens on 127.0.0.1 only: from the Mac, `ssh -L 8765:127.0.0.1:8765
<the CARLA computer>`, then `ws://localhost:8765` ([Watch from a laptop](foxglove.md)).

## What the adapter publishes

The same topics as Gazebo, so nothing else can tell them apart
([Interfaces](interfaces.md)):

| Topic | From |
|---|---|
| `/vehicle/odom`, `/vehicle/status`, `/vehicle/wheels`, `/vehicle/steering` | the CARLA car's motion |
| `/sensors/imu/data_raw` | CARLA's IMU, at `imu_link` |
| `/sensors/gps/fix` | the antenna (`gps_link`) round our datum, plus 0.5 m of noise |
| `/sensors/camera/{front_left,front_right,rear}/image_raw` and `camera_info` | CARLA's cameras, with Gazebo's lens (640 x 360) |
| `/sensors/radar/points` | CARLA's radar, with the TI driver's fields |
| `/sensors/lidar/points` | CARLA's lidar, 16 beams like Gazebo's |
| `/clock`, `/system/source` (all SIM), `/vehicle/carla/truth` | the adapter |

Every sensor sits where `vehicle.yaml` puts ours. The CARLA car is bigger than ours, so
the steering angle is scaled to turn it on **our** car's circle.

## A digital twin of Carleton

**Is there a ready-made Ottawa or Carleton map? No.** None was found. We build our own
from OpenStreetMap: the same data as our map window (`ottawa.pmtiles`).

Two ways, both from the same download:

| | Roads only (our adapter) | CARLA's Digital Twin Tool |
|---|---|---|
| Gets | the roads, lanes and junctions; no buildings | roads, plus generated buildings and trees |
| How | `osm:=` : CARLA's `Osm2Odr` converter, then `generate_opendrive_world` | a plug-in inside the Unreal editor |
| Needs | CARLA 0.9.16 (any 0.9.x) | a source build of 0.9.16 on Linux (its map browser and local files are Linux only) |
| Time | minutes | about 10 min per 2 x 2 km, and saving can take hours |
| Status | written, untested | experimental: "not production ready"; no bridges or overpasses |

The "standalone" Digital Twin Tool (version 2) was only announced as "coming soon" with
0.9.16: it hasn't shipped. CARLA 0.10 (UE5) has neither way yet.

### Steps (roads only)

1. Download Carleton's map data (once; it stays out of git):
   ```bash
   pixi run get carla-osm          # maps/carla/carleton.osm, about 6 MB
   ```
   The box is 900 m west and south, 700 m east and north of our datum: the campus,
   Bronson Avenue, Colonel By Drive, Sunnyside. `CARLA_OSM_AROUND=w,s,e,n` (metres)
   changes it.
2. Start CARLA **0.9.16**, then:
   ```bash
   pixi run carla-car host:=127.0.0.1 osm:=maps/carla/carleton.osm spawn:=0,0,0
   ```
3. Check the line-up: drive to a corner you know and compare `/sensors/gps/fix` with the
   map window.

### Lining it up with our maps

Our map frame is fixed by the datum in `navsat.yaml` (Carleton, about 45.386, -75.696):
x east, y north. The adapter builds the CARLA map with a projection centred on that datum
(`+proj=tmerc +lat_0=<datum> +lon_0=<datum>`), and no recentring. So:

- CARLA's world origin **is** the datum, and its x points east;
- `frame:=auto` sees that (CARLA's own georeference puts the origin within 20 km of the
  datum) and uses CARLA's world as our map frame;
- the GPS, Nav2's maps, GPS goals and the map window all agree with the streets CARLA
  draws.

A Digital Twin Tool map lines up the same way if its georeference is right: the adapter
reads where CARLA puts the world origin, and shifts by that.

## Cloud GPU instead

If the NVIDIA computer is busy, a cloud GPU with 16 GB or more runs the server headless in
Docker. Rough prices (Oct 2026, not checked with each provider):

| Option | GPU | About |
|---|---|---|
| AWS g6.2xlarge | L4, 24 GB | $1 an hour |
| AWS g5.2xlarge | A10G, 24 GB | $1.20 an hour |
| Lambda | A6000 / A10 | $0.70-1.30 an hour |

Run the adapter on the same cloud machine, and watch over Foxglove. Whether Unreal's
Vulkan renderer works on each instance type is not documented: try before relying on it.

## Other ways we didn't pick

| Way | Why not now |
|---|---|
| CARLA's built-in ROS 2 (`--ros2`) | stalls for seconds at a time on Jazzy: its FastDDS 2.11 vs Jazzy's 2.14 ([issue 9551](https://github.com/carla-simulator/carla/issues/9551)). A fix (FastDDS 2.14.6, Zenoh) is on the unreleased `ue4-dev` branch |
| carla-ros-bridge | last commit July 2022; no Jazzy release |
| A Humble bridge in a container | works, but adds a second ROS and a DDS bridge to debug, for nothing the adapter doesn't give |
| bringauto's carla-ros-bridge fork | says it runs Jazzy with 0.9.16: unverified |

## What works and what's untested

| Part | State |
|---|---|
| The conversions: frames, steering, sensors, GPS round the datum | tested (`aav_carla/test`, 78 tests) |
| Every call to the server | tested against a pretend `carla` module, with a drive that checks the signs |
| The launch files and their arguments | tested (`test_carla_launch.py`) |
| Carleton's OpenStreetMap download | run, 15 Oct: 2,311 ways, 6.2 MB |
| The adapter against a real CARLA server, 0.10 or 0.9.16 | **untested** |
| The IMU's roll and pitch rates (they follow carla-ros-bridge) | **untested**: our EKFs use only the yaw rate |
| The radar's speed sign | **untested** |
| Carleton's roads in CARLA, and the GPS line-up | **untested** |

## Licences

| Part | Licence |
|---|---|
| CARLA's code | MIT |
| CARLA's assets (maps, cars) | CC-BY |
| Unreal Engine | Epic's own terms. Building 0.10 from source needs a GitHub account linked to Epic |
| OpenStreetMap data | ODbL: say "© OpenStreetMap contributors" wherever it shows |

## Sources

- Releases: [github.com/carla-simulator/carla/releases](https://github.com/carla-simulator/carla/releases);
  [0.10.0 post](https://carla.org/2024/12/19/release-0.10.0/);
  [0.9.16 post](https://carla.org/2025/09/16/release-0.9.16)
- UE5 needs: [ue5-dev README](https://github.com/carla-simulator/carla/blob/ue5-dev/README.md);
  [UE5 quick start](https://carla-ue5.readthedocs.io/en/latest/start_quickstart/)
- Python client: [pypi.org/project/carla](https://pypi.org/project/carla/#files)
- Built-in ROS 2: [ros2_native](https://carla-ue5.readthedocs.io/en/latest/ros2_native/);
  [issue 9551](https://github.com/carla-simulator/carla/issues/9551)
- Bridge: [carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge)
- Digital Twin Tool: [0.9.16 docs](https://carla.readthedocs.io/en/0.9.16/adv_digital_twin/);
  limits: Richardson and Sprinkle, *OpenTwinMap*, [arXiv 2511.21925](https://arxiv.org/abs/2511.21925)
- OpenStreetMap to CARLA: [tutorial](https://carla.readthedocs.io/en/latest/tuto_G_openstreetmap/);
  [geocoordinates](https://carla.readthedocs.io/en/latest/geocoordinates/)
