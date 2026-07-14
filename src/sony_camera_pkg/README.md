# sony_camera_pkg

ROS 2 Humble driver wrapper for Sony Camera Remote SDK live view. Sony's SDK is
not redistributed by this repository; download and accept it separately.

## Stage the ARM64 SDK locally

Extract the Sony Linux 64-bit ARMv8 archive and unpack `RemoteCli.zip`. Then run:

```bash
python3 src/sony_camera_pkg/scripts/install_crsdk.py /path/to/RemoteCli
```

The script copies public headers and runtime libraries to
`src/sony_camera_pkg/sdk/`. That directory is ignored by Git.

## Camera calibration

The driver publishes `/sony/image_raw` and `/sony/camera_info` with the same
timestamp, dimensions, and `sony_camera_optical_frame`. A calibration for a
different resolution is rejected rather than published with misleading
intrinsics. Calibrate the exact live-view resolution and checkerboard used by
your setup, for example:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 9x6 --square 0.024 \
  image:=/sony/image_raw camera:=/sony
```

Replace `9x6` and `0.024` with the inner-corner count and square size in metres.
Use `camera_info_url:=file:///home/nvidia/.ros/camera_info/sony_zv_e10_ii.yaml`
when launching so the persisted calibration is loaded on every start.

The one-command bringup exposes the same setting as `sony_camera_info_url` and
the one-based CRSDK device selection as `sony_camera_index`.

For a Jetson deployment build, require the SDK explicitly:

```bash
colcon build --packages-select sony_camera_pkg \
  --cmake-args -DSONY_CRSDK_REQUIRED=ON -DCMAKE_BUILD_TYPE=Release
```

Without the locally staged SDK, the package remains discoverable but does not
build or install `sony_camera_node`. This keeps ordinary CI independent of Sony's
proprietary download while preventing a deployment build from silently omitting
the driver.
