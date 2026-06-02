# GimbalCmd.msg — Gimbal velocity-level command
std_msgs/Header header

float32 yaw_rate                   # Desired yaw angular velocity (rad/s)
float32 pitch_rate                 # Desired pitch angular velocity (rad/s)
bool hold_yaw                      # If true, hold current yaw; yaw_rate ignored
bool hold_pitch                    # If true, hold current pitch; pitch_rate ignored
