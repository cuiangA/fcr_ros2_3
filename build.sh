#!/bin/bash
# =============================================================================
# FCR ROS2 Humble Workspace Build Script
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== FCR ROS2 Workspace Build ==="
echo "Workspace: $SCRIPT_DIR"

# Source ROS2 Humble underlay
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/humble/install/setup.bash ]; then
    source /opt/ros/humble/install/setup.bash
else
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble"
    exit 1
fi

# Install dependencies
echo "--- Installing rosdep dependencies ---"
rosdep install --from-paths src --ignore-src -y --rosdistro humble 2>/dev/null || true

# Build with colcon
echo "--- Building packages ---"
colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_STANDARD=17 \
    --event-handlers console_direct+ \
    "$@"

echo ""
echo "=== Build complete ==="
echo "Source the workspace: source install/setup.bash"
