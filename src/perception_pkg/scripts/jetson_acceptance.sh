#!/usr/bin/env bash
set -euo pipefail

engine="${1:-$HOME/fcr_models/yolov8n_fp16.engine}"
duration="${ACCEPTANCE_DURATION_SECONDS:-300}"

if [[ ! -s "$engine" ]]; then
  echo "ERROR: TensorRT engine does not exist or is empty: $engine" >&2
  exit 2
fi
if [[ ! -f install/setup.bash ]]; then
  echo "ERROR: run this script from the built ROS 2 workspace root." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source install/setup.bash

required_topics=(
  /sony/image_raw
  /sony/camera_info
  /perception/detections
  /perception/tracks
)
for topic in "${required_topics[@]}"; do
  if ! ros2 topic list | grep -Fxq "$topic"; then
    echo "ERROR: required topic is missing: $topic" >&2
    exit 3
  fi
done
if ros2 topic list | grep -Fxq /perception/targets_3d; then
  echo "ERROR: /perception/targets_3d must not exist before fusion." >&2
  exit 3
fi

echo "Checking message delivery and rates for ${duration}s."
for topic in "${required_topics[@]}"; do
  echo "== $topic interface =="
  ros2 topic info "$topic" --verbose
  timeout 8 ros2 topic echo "$topic" --once >/dev/null
done

echo "== sampled rates =="
timeout "$duration" ros2 topic hz /sony/image_raw >/tmp/fcr_sony_image_hz.log &
image_hz_pid=$!
timeout "$duration" ros2 topic hz /perception/tracks >/tmp/fcr_tracks_hz.log &
tracks_hz_pid=$!
wait "$image_hz_pid" || [[ $? -eq 124 ]]
wait "$tracks_hz_pid" || [[ $? -eq 124 ]]
cat /tmp/fcr_sony_image_hz.log
cat /tmp/fcr_tracks_hz.log

echo "Acceptance sampling completed. Review /diagnostics for ERROR/WARN and the"
echo "rate logs above. The 30-minute camera stability and ID-switch observations"
echo "remain operator-confirmed gates; this script does not claim them automatically."
