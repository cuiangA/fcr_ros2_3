#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./run_acceptance.sh --port /dev/serial/by-id/... --confirm-wheels-off-ground
EOF
}

PORT=""
CONFIRMED=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="${2:-}"; shift 2 ;;
    --confirm-wheels-off-ground) CONFIRMED=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "$PORT" || $CONFIRMED -ne 1 ]]; then
  usage
  echo "ERROR: a port and explicit wheel-off-ground confirmation are required." >&2
  exit 2
fi
if [[ ! -e "$PORT" ]]; then
  echo "ERROR: serial device does not exist: $PORT" >&2
  exit 1
fi
if command -v fuser >/dev/null 2>&1 && fuser "$PORT" >/dev/null 2>&1; then
  echo "ERROR: serial port is already in use: $PORT" >&2
  fuser -v "$PORT" || true
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: ROS 2 Humble is not installed." >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
WS_ROOT="$(cd -- "$REPO_ROOT/../.." && pwd)"
LOG_DIR="$WS_ROOT/log/lekiwi_phase2"
mkdir -p "$LOG_DIR"
DRIVER_LOG="$LOG_DIR/chassis_driver_$(date +%Y%m%d_%H%M%S).log"

source /opt/ros/humble/setup.bash
if [[ ! -f "$WS_ROOT/install/setup.bash" ]]; then
  echo "ERROR: workspace is not built; run build_on_jetson.sh first." >&2
  exit 1
fi
source "$WS_ROOT/install/setup.bash"

driver_pid=""
cleanup() {
  if [[ -n "$driver_pid" ]] && kill -0 "$driver_pid" 2>/dev/null; then
    kill -INT "$driver_pid" 2>/dev/null || true
    wait "$driver_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting chassis-only driver; log: $DRIVER_LOG"
ros2 run robot_platform_pkg chassis_driver_node --ros-args \
  --params-file "$REPO_ROOT/src/robot_platform_pkg/config/chassis_params.yaml" \
  -p use_sim:=false \
  -p serial_device:="$PORT" \
  -p baudrate:=1000000 \
  -p max_linear_velocity:=0.1 \
  -p max_angular_velocity:=0.5 \
  -p max_raw_wheel_velocity:=500 \
  -p command_timeout_ms:=250 \
  >"$DRIVER_LOG" 2>&1 &
driver_pid=$!

for _ in {1..50}; do
  if ! kill -0 "$driver_pid" 2>/dev/null; then
    echo "ERROR: chassis driver exited during startup." >&2
    tail -n 80 "$DRIVER_LOG" >&2
    exit 1
  fi
  if ros2 node list 2>/dev/null | grep -qx '/chassis_driver'; then
    break
  fi
  sleep 0.1
done
if ! ros2 node list 2>/dev/null | grep -qx '/chassis_driver'; then
  echo "ERROR: /chassis_driver did not become ready." >&2
  tail -n 80 "$DRIVER_LOG" >&2
  exit 1
fi

python3 "$SCRIPT_DIR/acceptance_test.py" \
  --confirm-wheels-off-ground \
  --report "$LOG_DIR/lekiwi_phase2_report.json"

echo "Stopping chassis driver safely..."
cleanup
driver_pid=""

if grep -Eqi 'fatal|checksum mismatch|response timeout|velocity (read|write) failed' "$DRIVER_LOG"; then
  echo "ERROR: driver log contains communication failures:" >&2
  grep -Ein 'fatal|checksum mismatch|response timeout|velocity (read|write) failed' "$DRIVER_LOG" >&2
  exit 1
fi

echo
echo "PHASE 2 PASS"
echo "Report: $LOG_DIR/lekiwi_phase2_report.json"
echo "Driver log: $DRIVER_LOG"
