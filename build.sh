#!/bin/bash
# =============================================================================
# FCR ROS2 Humble Workspace Build Script
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

SKIP_ROSDEP=0
COLCON_ARGS=()

usage() {
    cat <<'EOF'
Usage: ./build.sh [--skip-rosdep] [--help] [colcon args...]

Options:
  --skip-rosdep   Skip rosdep dependency installation.
  --help          Show this help message.

Any remaining arguments are passed through to colcon build.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-rosdep)
            SKIP_ROSDEP=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            COLCON_ARGS+=("$1")
            shift
            ;;
    esac
done

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
if [ "$SKIP_ROSDEP" -eq 0 ]; then
    echo "--- Installing rosdep dependencies ---"
    if ! command -v rosdep >/dev/null 2>&1; then
        echo "ERROR: rosdep is not installed. Install it or rerun with --skip-rosdep."
        exit 1
    fi
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
else
    echo "--- Skipping rosdep dependency installation ---"
fi

# Build with colcon
echo "--- Building packages ---"
colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_STANDARD=17 \
    --event-handlers console_direct+ \
    "${COLCON_ARGS[@]}"

echo ""
echo "=== Build complete ==="
echo "Source the workspace: source install/setup.bash"
