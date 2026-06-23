# ShotReference.msg - unified reference shared by follow and cinematic modes
std_msgs/Header header

uint8 MODE_STANDBY=0
uint8 MODE_FOLLOW=1
uint8 MODE_ACTION_SHOT=2

uint8 SHOT_FOLLOW=0
uint8 SHOT_PAN=1
uint8 SHOT_RECENTER=2
uint8 SHOT_DOLLY_IN=3
uint8 SHOT_DOLLY_OUT=4
uint8 SHOT_ORBIT=5
uint8 SHOT_TRUCK=6
uint8 SHOT_ARC_IN=7

uint8 STATE_IDLE=0
uint8 STATE_PRECHECK=1
uint8 STATE_ACQUIRE_TARGET=2
uint8 STATE_ALIGN_START=3
uint8 STATE_ACTIVE=4
uint8 STATE_SPEED_LIMITED=5
uint8 STATE_HOLD_SHOT=6
uint8 STATE_RECOVER_TARGET=7
uint8 STATE_FOLLOW_FALLBACK=8
uint8 STATE_SHOT_EXIT=9
uint8 STATE_COMPLETED=10
uint8 STATE_CANCELED=11
uint8 STATE_SAFE_STOP=12
uint8 STATE_FAILED=13

uint8 mode
uint8 shot_type
uint8 state
int32 target_id

# Both poses are expressed in header.frame_id, normally odom.
geometry_msgs/Pose virtual_base_pose
geometry_msgs/Point target_position
geometry_msgs/Vector3 target_velocity

float32 desired_distance
float32 orbit_radius
float32 orbit_angle
float32 desired_composition_u
float32 desired_composition_v

float32 max_base_speed
float32 max_base_yaw_rate
float32 max_gimbal_yaw_rate
float32 max_gimbal_pitch_rate

float32 target_speed
float32 required_base_speed
float32 speed_scale
float32 progress

bool base_motion_enabled
bool gimbal_tracking_enabled
bool valid
string status
