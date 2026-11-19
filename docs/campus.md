# The campus

Carleton in Ravensim (Gazebo too, until 16 Nov), on one costmap, with a guessed NDT map. It's for practice while the
hardware catches up (Ben, 5 Nov). Nothing here has met the real campus yet. Its map, cut
into 64 m chunks (`maps/campus/chunks/`), is **RavenTiles** ([Own costmap](own-costmap.md)).

```sh
pixi run campus                                       # builds maps/campus/ (about 10 s)
pixi run sim world:=campus screen_cubes:=true ndt:=true gui:=false rviz:=false
pixi run dev-window --live --ndt-map maps/ndt/campus.pcd   # in a second terminal
```

Click a road or lot on the map, then confirm, as in any world. The car starts in lot P4,
because the datum itself is on a station shelter. It drives at up to 2.5 m/s here, in the
simulator only: the real car's limit stays 1 m/s.

## What's real and what's guessed

| From OpenStreetMap | Guessed by `tools/campus_world.py` |
|---|---|
| where roads, lots, buildings and trees are | how tall most buildings are (529 of 574) |
| their shapes | where kerbs run (58 km: every edge of every road and lot, 0.30 m) |
| many road widths | other road widths, tree sizes, trees inside woods |
| | all the looks: pavement, grass, brick, stone |

The data is (c) OpenStreetMap contributors, under the ODbL, so anything showing it must say so.
`maps/` isn't in git: `pixi run campus` makes it again.

**Google:** being students doesn't change Google's terms, which limit copying their maps into
another map. So the world is built from OpenStreetMap. Your own tape measurements and points
you read by hand are for checking it.

## What it makes

| File | What it is |
|---|---|
| `maps/campus/campus.sdf`, `meshes/`, `textures/` | the world: 355,000 triangles, bump-mapped |
| `maps/campus/campus.pgm`, `.yaml` | one map (Nav2's format): 1.6 x 1.6 km at 0.25 m (6400 x 6400 cells) |
| `maps/campus/place.yaml` | where the car starts, what went in, the credit |
| `maps/campus/lanes.yaml` | the painted roads' centre lines and widths: `lane_score.py` scores keeping right by them; the lane map (`sim world:=campus behaviour:=true lane_map:=world`): the road ahead on the right lane's centre |
| `maps/ndt/campus.pcd` | NDT's map, **a guess**: points on the same shapes |

**The paint (8 Nov):** big roads (trunk to tertiary) have a dashed yellow centre line and a
solid white line 0.5 m in from each edge, so the lane finder sees both sides of its lane.
Other roads have none.

**The shape (Ben, 5 Nov):** everything that isn't pavement is a foot up, held in by a concrete
kerb, with grass on top.

**One costmap, no segments.** It works on a Mac: only roads and lots are free. Gazebo (then) held
real time (0.98-1.01). Nav2's planner loop ran at about 8 Hz, not the 20 Hz it wanted, and warned
about it in the console. Paths and grass are blocked: the car stays on roads and lots.

**Sent once.** At this size Nav2's costmap sending the whole grid every second froze the window (up to
1.66 s) and may have caused the flashing. `world:=campus` sends it once, then only changes. The
window also skips a grid it already has.

## What was measured (Gazebo, 5 Nov)

| Check | Result |
|---|---|
| Nav2 goal from P4, 40 m | reached |
| EKF against the truth | within 0.36 m |
| NDT against the truth, on the guess, grass raised | within 0.13 m (near buildings, 90 s) |
| NDT on a map recorded on that road, grass not raised | within 0.11 m |

**This doesn't show the real campus will work.** The guess is made from the same shapes as the
world, so of course it fits here.

## NDT's score floor

`ndt.yaml`'s 2.3 is the test lot's, with walls close by. On the campus, fits at the true place
score about 1.6-1.9 on the guess, and 2.00-2.31 on a recorded map. So this world uses 1.5.

**Floors.** The grass is at 0.30 m, right on the car's usual 0.3 m floor. NDT looked at it and
got lost, so here NDT looks above 0.45 m. The window's cubes look down to 0.12 m, so kerbs and
grass show as the lidar's rings. The flat scan keeps 0.3 m: untested against kerbs (TODO 119).

**Not measured:** how high a *wrong* pose scores here. Until it is, leave `ndt_ekf` off on the
campus: NDT is watched, and doesn't steer the estimate.

## Found along the way

| Finding | What to do |
|---|---|
| With `reversing` on, the car backs a long way to a goal behind it, at 0.5 m/s | reversing stays on (a must-have for 0.4-rc1): our stack reached 9 of 9 campus legs with it (TODO 167); `reversing:=false` for forward only |
| MPPI stayed near 1 m/s at any `sim_speed`: its 3 m of path was shorter than its futures | fixed: the path now grows with the speed. Campus default 2.5 m/s (mean 2.2) |
| The car wiggles: the turn rate flips about once a second | TODO 124; to be tested on its own (TODO 149) |
| Kerbs sit at the flat scan's 0.3 m floor | TODO 119: test it now this world exists |
| A lot's middle is far from anything | the lidar sees little there: NDT may lose the fit |
| Lidar cubes reach 30 m, NDT's points 45 m | so the top view can look empty in open space |

Newer runs here (8 Nov, Ravensim, 2.5 m/s): [our planner](own-planner.md#the-campus-3-legs-25-ms),
[our navigator](own-navigator.md#the-campus-25-ms-2-runs), [directions](routes.md#on-the-campus).

## Next: the real lidar drive

A drive suggests and a person accepts (Ben, 5 Nov). The tools to record and build a map exist,
and were rehearsed here:

```sh
tools/record.sh --lidar
pixi run ndt-map --out maps/ndt/<place> build <bag> --poses /localization/odom/global
```

Not built yet: fitting a drive onto the outlines, the GPS offset, and approving changes in the
window.
