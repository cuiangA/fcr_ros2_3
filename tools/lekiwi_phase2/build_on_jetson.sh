#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
WS_ROOT="$(cd -- "$REPO_ROOT/../.." && pwd)"

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "aarch64" ]]; then
  echo "ERROR: run this script on the Jetson aarch64 Linux host." >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found." >&2
  exit 1
fi
if [[ ! -d "$WS_ROOT/src" ]]; then
  echo "ERROR: inferred workspace $WS_ROOT does not contain src/." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash

echo "Workspace: $WS_ROOT"
echo "Repository: $REPO_ROOT"
echo "Commit: $(git -C "$REPO_ROOT" rev-parse --short HEAD)"

cd "$WS_ROOT"
echo "[1/3] Building robot_platform_pkg and its workspace dependencies"
colcon build \
  --symlink-install \
  --packages-up-to robot_platform_pkg \
  --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source "$WS_ROOT/install/setup.bash"

echo "[2/3] Running robot_platform_pkg tests"
colcon test \
  --packages-select robot_platform_pkg \
  --event-handlers console_cohesion+
colcon test-result --test-result-base build/robot_platform_pkg --verbose

echo "[3/3] Checking installed chassis executable"
ros2 pkg executables robot_platform_pkg | grep -q 'chassis_driver_node'

echo
echo "BUILD PASS"
echo "Next: run tools/lekiwi_phase2/run_acceptance.sh with the stable serial path."
