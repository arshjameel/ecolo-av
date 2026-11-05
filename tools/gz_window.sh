#!/usr/bin/env bash
# Gazebo's window, for a simulator already running (pixi run tools/ros.sh tools/gz_window.sh), with its camera
# following the car, as gazebo.launch.py's follow:=true does when the window starts with
# the simulator. Run through tools/ros.sh, inside pixi.
gz sim -g &
window=$!
# Wait for the car, then keep asking until the window answers (it takes a few seconds).
for _ in $(seq 1 60); do gz model --list 2> /dev/null | grep -q -- "- aav$" && break; sleep 1; done
for _ in $(seq 1 30); do
  gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
    --timeout 2000 --req 'data: "aav"' > /dev/null 2>&1 &&
  gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
    --timeout 2000 --req 'x: -3.5, y: -2.5, z: 1.8' > /dev/null 2>&1 && break
  sleep 1
done
wait "$window"
