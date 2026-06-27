#!/bin/bash
# =============================================================================
# FCR ROS2 Humble Workspace Build Script
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

SKIP_ROSDEP=0
SKIP_PIP=0
COLCON_ARGS=()

usage() {
    cat <<'EOF'
Usage: ./build.sh [--skip-rosdep] [--skip-pip] [--help] [colcon args...]

Options:
  --skip-rosdep   Skip rosdep dependency installation.
  --skip-pip      Skip pip dependency installation.
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
        --skip-pip)
            SKIP_PIP=1
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

# Install pip dependencies
if [ "$SKIP_PIP" -eq 0 ]; then
    echo "--- Installing pip dependencies ---"
    pip3 install --user -r requirements.txt
else
    echo "--- Skipping pip dependency installation ---"
fi

# Install rosdep dependencies (apt packages from package.xml)
if [ "$SKIP_ROSDEP" -eq 0 ]; then
    echo "--- Installing rosdep dependencies ---"

    # 如果 rosdep 未安装，尝试自动安装
    if ! command -v rosdep >/dev/null 2>&1; then
        echo "rosdep not found, attempting to install..."
        if command -v apt-get >/dev/null 2>&1; then
            sudo apt-get update && sudo apt-get install -y python3-rosdep
        else
            echo "ERROR: rosdep is not installed and apt-get is not available."
            echo "Please install rosdep manually or rerun with --skip-rosdep."
            exit 1
        fi
    fi

    # 如果 rosdep 未初始化，自动初始化
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        echo "Initializing rosdep..."
        sudo rosdep init
    fi
    rosdep update 2>/dev/null || true

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
