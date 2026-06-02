# PlatformState.msg — Unified robot platform state (chassis + gimbal + IMU)
std_msgs/Header header

# Chassis state
float32[3] chassis_pose            # [x, y, yaw] in odom frame
float32[3] chassis_velocity        # [vx, vy, omega] current velocity
float32 battery_voltage            # Battery voltage

# Gimbal state
float32 gimbal_yaw                 # Current yaw angle (rad)
float32 gimbal_pitch               # Current pitch angle (rad)
float32 gimbal_yaw_rate            # Current yaw rate (rad/s)
float32 gimbal_pitch_rate          # Current pitch rate (rad/s)

# IMU data
float32[3] angular_velocity        # [wx, wy, wz] rad/s
float32[3] linear_acceleration     # [ax, ay, az] m/s²
geometry_msgs/Quaternion orientation # Orientation quaternion

# Status flags
bool chassis_connected
bool gimbal_connected
bool imu_connected
bool emergency_stop
uint8 system_mode                  # 0=IDLE 1=MANUAL 2=AUTO 3=SERVO 4=EMERGENCY
