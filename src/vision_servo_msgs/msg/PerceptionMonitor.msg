# PerceptionMonitor.msg — Read-only telemetry for remote visual monitoring.
#
# Latencies are measured by the observer from the source image header stamp to
# message arrival. inference_time_ms_estimate is therefore a detector-pipeline
# estimate, not TensorRT kernel profiling.
std_msgs/Header header

uint8 HEALTH_UNKNOWN=0
uint8 HEALTH_OK=1
uint8 HEALTH_WARN=2
uint8 HEALTH_ERROR=3
uint8 HEALTH_STALE=4

# Component health derived from /diagnostics.
uint8 camera_health
uint8 detector_health
uint8 tracker_health
string camera_status
string detector_status
string tracker_status
bool camera_connected
bool model_ready

# Model identity is supplied by deployment configuration; backend may also be
# refreshed from the detector diagnostic key/value stream.
string yolo_model
string inference_backend
string tensor_rt_engine

# Observer-side rates and timing.
float32 image_fps
float32 detection_fps
float32 tracking_fps
float32 detection_latency_ms
float32 tracking_latency_ms
float32 visualization_latency_ms
float32 inference_time_ms_estimate

# Latest 2D perception state.
uint32 detection_count
uint32 track_count
int32 tracking_target_id
uint8 tracking_state
string tracking_state_name

# Reserved observation fields for the post-fusion/control phase. Their valid
# flags remain false until the optional subscriptions are enabled and data is
# received; the monitor never publishes control commands.
bool target_3d_available
float32[3] target_3d_position
bool cmd_vel_available
float32[3] cmd_vel
bool gimbal_state_available
float32 gimbal_yaw
float32 gimbal_pitch

