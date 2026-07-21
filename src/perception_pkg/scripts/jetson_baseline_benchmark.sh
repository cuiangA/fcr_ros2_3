#!/usr/bin/env bash
set -euo pipefail

# T01: 2D tracking baseline benchmark for Jetson
# Records FPS, inference latency, end-to-end latency from /diagnostics and node logs.
# Output: output/baseline-v2.5/performance_baseline.json

engine="${1:-$HOME/fcr_models/yolov8n_fp16.engine}"
duration="${BENCHMARK_DURATION_SECONDS:-60}"
baseline_dir="$(cd "$(dirname "$0")/../../.." && pwd)/output/baseline-v2.5"

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

mkdir -p "$baseline_dir"

# Record engine hash
sha256sum "$engine" | tee "$baseline_dir/engine_hash.txt"

# Start perception pipeline (detection + tracking only, no control)
ros2 launch perception_pkg perception.launch.py \
  device:=tensorrt \
  model_path:="$engine" \
  enable_camera_motion_compensation:=true \
  performance_log_period:=5.0 &
perception_pid=$!
sleep 5

# Collect diagnostics
echo "Collecting benchmark data for ${duration}s."
timeout "$duration" ros2 topic echo /diagnostics \
  > /tmp/fcr_baseline_diagnostics.json 2>&1 &
diag_pid=$!

# Collect FPS from detection_node
timeout "$duration" ros2 topic hz /perception/detections \
  > /tmp/fcr_baseline_detections_hz.log 2>&1 &
det_hz_pid=$!

timeout "$duration" ros2 topic hz /perception/tracks \
  > /tmp/fcr_baseline_tracks_hz.log 2>&1 &
trk_hz_pid=$!

# Collect node logs for latency
timeout "$duration" ros2 node info /detection_node \
  > /dev/null 2>&1 || true

wait "$diag_pid" || [[ $? -eq 124 ]]
wait "$det_hz_pid" || [[ $? -eq 124 ]]
wait "$trk_hz_pid" || [[ $? -eq 124 ]]

# Extract FPS from hz logs
det_fps=$(grep -oP 'average rate: \K[0-9.]+' /tmp/fcr_baseline_detections_hz.log || echo "N/A")
trk_fps=$(grep -oP 'average rate: \K[0-9.]+' /tmp/fcr_baseline_tracks_hz.log || echo "N/A")

# Extract inference / latency from detection_node logs (parse performance_log_period messages)
# Format: "inference avg=XX.Xms max=XX.Xms p95=XX.Xms | e2e avg=XX.Xms max=XX.Xms p95=XX.Xms"
log_lines=$(ros2 node info /detection_node 2>&1 || true)
perf_log=$(grep -oP 'inference avg=[0-9.]+ms max=[0-9.]+ms p95=[0-9.]+ms \| e2e avg=[0-9.]+ms max=[0-9.]+ms p95=[0-9.]+ms' /tmp/fcr_baseline_diagnostics.json 2>/dev/null | tail -1 || echo "")

# Build JSON output
cat > "$baseline_dir/performance_baseline.json" << JSONEOF
{
  "timestamp": "$(date -Iseconds)",
  "tag": "baseline-v2.5-tracking",
  "engine": "$engine",
  "engine_hash": "$(sha256sum "$engine" | cut -d' ' -f1)",
  "benchmark_duration_seconds": $duration,
  "fps": {
    "detections": $det_fps,
    "tracks": $trk_fps
  },
  "latency_ms": {
    "inference_avg": null,
    "inference_max": null,
    "inference_p95": null,
    "e2e_avg": null,
    "e2e_max": null,
    "e2e_p95": null
  },
  "hw_platform": "jetson_orin_nano",
  "notes": "TODO: parse perf log lines for detailed latency; baseline-v2.5-track"
}
JSONEOF

echo "=== FPS summary ==="
echo "detections average rate: ${det_fps} Hz"
echo "tracks      average rate: ${trk_fps} Hz"
echo ""
echo "Baseline written to: $baseline_dir/performance_baseline.json"
echo "Engine hash written to: $baseline_dir/engine_hash.txt"
echo ""
echo "Full diagnostics: /tmp/fcr_baseline_diagnostics.json"
echo ""
echo "⚠  TODO: Review /diagnostics for ERROR/WARN."
echo "⚠  TODO: Manually observe ID switches and tracking stability."

# Cleanup
kill "$perception_pid" 2>/dev/null || true
wait "$perception_pid" 2>/dev/null || true
