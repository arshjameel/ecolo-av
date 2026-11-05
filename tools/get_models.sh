#!/usr/bin/env bash
# Download the stock detector models into models/. Model files stay out of git.
#
# usage: tools/get_models.sh      (or: pixi run get models)
#
# Both know COCO's 80 everyday things, "stop sign" and "person" among them:
#   rf-detr-nano.pth  RF-DETR Nano, the detector's default (detector:=rfdetr). 366 MB: the
#                     download is a training checkpoint. Apache-2.0, from Roboflow.
#   yolo11n.pt        YOLO11 nano (detector:=yolo). 5.6 MB. AGPL-3.0, from Ultralytics.
# Model files are Python pickles: a wrong one could run code, so each is checked against
# its known checksum (MD5 for RF-DETR, as the rfdetr library pins its own) before use.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p models

sum_of() {  # sum_of <md5|sha256> <file>: portable (macOS has md5 and shasum)
  case $1 in
    md5) if command -v md5sum >/dev/null; then md5sum "$2" | cut -d' ' -f1; else md5 -q "$2"; fi ;;
    sha256) if command -v sha256sum >/dev/null; then sha256sum "$2" | cut -d' ' -f1
            else shasum -a 256 "$2" | cut -d' ' -f1; fi ;;
  esac
}

get() {  # get <file> <url> <md5|sha256> <sum>
  local file=$1 url=$2 kind=$3 sum=$4
  if [[ ! -f $file ]]; then
    curl -fL --retry 3 -o "$file.part" "$url"
    mv "$file.part" "$file"
  fi
  if [[ $(sum_of "$kind" "$file") != "$sum" ]]; then
    echo "$file isn't the expected download: delete it and run this again" >&2
    exit 1
  fi
  echo "$file: ok"
}

get models/rf-detr-nano.pth \
  https://storage.googleapis.com/rfdetr/nano_coco/checkpoint_best_regular.pth \
  md5 fb6504cce7fbdc783f7a46991f07639f
get models/yolo11n.pt \
  https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt \
  sha256 0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1
