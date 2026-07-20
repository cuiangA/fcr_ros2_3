# Target.msg — Single detected/tracked target with 2D and 3D information
std_msgs/Header header

# Identity
int32 id                          # Unique tracking ID (-1 if not tracked)
string class_name                  # Object class name from YOLO

# 2D tracker lifecycle. Raw detections use UNTRACKED and visible=true.
uint8 TRACKING_STATE_UNTRACKED=0
uint8 TRACKING_STATE_TENTATIVE=1
uint8 TRACKING_STATE_CONFIRMED=2
uint8 TRACKING_STATE_LOST=3
uint8 tracking_state
bool visible                       # True only when matched to a detection in this frame

# 2D geometry in the source RGB image. Coordinates are always pixels, never
# normalized values. The source image and CameraInfo must have the same size.
# bbox uses the half-open convention [x_min, y_min, x_max, y_max].
float32[4] bbox                    # Pixel coordinates [x_min, y_min, x_max, y_max]
float32[2] center                  # Pixel coordinates [cx, cy]

# Confidence & size
float32 confidence                 # Detection confidence [0.0, 1.0]
float32 height                     # BBox height in pixels
float32 width                      # BBox width in pixels

# 3D spatial information. The coordinate frame is header.frame_id and follows
# the ROS optical-frame convention (X right, Y down, Z forward) for camera data.
float32[3] position                # [x, y, z] in meters
float32[3] velocity                # [vx, vy, vz] in m/s

# A value of 0 means position/velocity must not be used as valid metric data.
float32 depth_confidence           # Reliability of metric 3D data [0.0, 1.0]
