# Target.msg — Single detected/tracked target with 2D and 3D information
std_msgs/Header header

# Identity
int32 id                          # Unique tracking ID (-1 if not tracked)
string class_name                  # Object class name from YOLO

# Bounding box (normalized 0-1 or pixel coordinates)
float32[4] bbox                    # [x_min, y_min, x_max, y_max]
float32[2] center                  # [cx, cy] — centroid in image coordinates

# Confidence & size
float32 confidence                 # Detection confidence [0.0, 1.0]
float32 height                     # BBox height in pixels
float32 width                      # BBox width in pixels

# 3D spatial information (camera frame)
float32[3] position                # [x, y, z] in meters (camera optical frame)
float32[3] velocity                # [vx, vy, vz] in m/s

# Depth quality
float32 depth_confidence           # How reliable is the depth estimate [0.0, 1.0]
