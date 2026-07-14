#!/usr/bin/env bash
set -euo pipefail

workspace="${1:-$PWD}"
if [[ ! -d "$workspace/src/perception_pkg" ]]; then
  echo "ERROR: workspace must contain src/perception_pkg: $workspace" >&2
  exit 2
fi
if [[ ! -f "$workspace/src/sony_camera_pkg/sdk/include/CRSDK/CameraRemote_SDK.h" ]]; then
  echo "ERROR: Sony CRSDK is not staged under src/sony_camera_pkg/sdk." >&2
  echo "Run src/sony_camera_pkg/scripts/install_crsdk.py first." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
cd "$workspace"

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to perception_pkg sony_camera_pkg \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DSONY_CRSDK_REQUIRED=ON \
    -DPERCEPTION_BUILD_LEGACY_FUSION=OFF \
    -DPERCEPTION_ENABLE_TENSORRT=ON

colcon test --packages-select perception_pkg
colcon test-result --verbose

echo "Jetson perception build and automated tests passed."
