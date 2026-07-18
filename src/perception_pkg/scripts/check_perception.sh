#!/usr/bin/env bash
set -euo pipefail

topics=(
  /sony/image_raw
  /sony/camera_info
  /perception/detections
  /perception/tracks
)

for topic in "${topics[@]}"; do
  echo "== $topic =="
  ros2 topic info "$topic" --verbose
  timeout 8 ros2 topic hz "$topic" || true
done

echo "== one track sample =="
timeout 5 ros2 topic echo /perception/tracks --once

if ros2 topic list | grep -Fxq /perception/targets_3d; then
  echo "ERROR: /perception/targets_3d exists during the pre-fusion production run" >&2
  exit 2
fi

echo "Pre-fusion topic check passed"
