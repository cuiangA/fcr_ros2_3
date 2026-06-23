# TargetState.msg - selected target state in a stable control frame
std_msgs/Header header

uint8 STATUS_LOST=0
uint8 STATUS_TRACKING=1
uint8 STATUS_PREDICTED=2

int32 target_id
string class_name

# Position and velocity are expressed in header.frame_id, normally odom.
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 speed

# Confidence values are normalized to [0, 1].
float32 position_confidence
float32 velocity_confidence

# Original image-space observation for gimbal composition control.
float32[2] image_center
float32[4] bbox

uint8 tracking_status
bool valid
