# 2D perception contract (pre-fusion candidate v1)

This contract becomes frozen after the Jetson acceptance checklist passes. Until
then, incompatible changes must update this document and the message tests in the
same commit.

## Data chain

```text
/sony/image_raw + /sony/camera_info
  -> /perception/detections
  -> /perception/tracks
```

No production node in this phase publishes `/perception/targets_3d`.

## Image and calibration

- `/sony/image_raw` uses `sensor_msgs/Image`, BGR8, SensorDataQoS, depth 1.
- `/sony/camera_info` uses `sensor_msgs/CameraInfo` and has exactly the same
  `header.stamp`, `header.frame_id`, width, and height as its image.
- The fixed frame is `sony_camera_optical_frame`: X right, Y down, Z forward.
- A calibration is valid only for the exact live-view resolution. Missing or
  mismatched calibration is published with zero intrinsics, never rescaled or
  presented as valid silently.
- Image timestamps are ROS time and strictly increase within one driver process.

## Detection and tracking messages

- `TargetArray.header` is copied from the source Sony image. Each contained
  `Target.header` is identical to the array header.
- `bbox=[x_min,y_min,x_max,y_max]` is half-open and expressed in source-image
  pixels after letterbox reversal and clipping.
- `center=[cx,cy]`, `width`, and `height` are also source-image pixels.
- Raw detections use `id=-1`, `UNTRACKED`, and `visible=true`.
- Confirmed tracks have a process-local non-negative ID. IDs are unique only
  within one tracking-node run and may restart after the node restarts.
- `CONFIRMED + visible=true` means the current frame supplied a matching
  detection. `LOST + visible=false` means the bbox is Kalman-predicted and is
  retained for at most `max_age` missed frames.
- Tentative tracks are internal and are not published before `min_hits`
  consecutive matches.
- `TargetArray.tracking_id` is the selected confirmed track, or `-1`. Disabling
  target lock does not disable multi-object tracking.

## Deliberately invalid 3D fields

Before calibrated Sony/Orbbec fusion, every detection and track must keep
`depth_confidence=0`. `position` and `velocity` must be ignored regardless of
their default numeric values. No consumer may infer metric depth from the 2D
message alone.

## QoS

- Images and CameraInfo: best effort, volatile, keep last 1.
- Detections and tracks: reliable, volatile, keep last 1.

## Acceptance gate before freezing

- Clean Humble build and all unit/launch tests pass.
- Sony calibration, reconnect, Ctrl-C, and 30-minute stability tests pass.
- Known-image C++ YOLO results agree with the Ultralytics reference.
- Jetson TensorRT reaches at least 15 FPS with detection latency P95 below
  100 ms and track latency P95 below 150 ms.
- A five-minute single-person run has no unexplained ID switch.
- A rosbag regression reproduces timestamps, frame IDs, and comparable boxes.
