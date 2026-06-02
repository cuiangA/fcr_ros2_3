# ServoState.msg — Visual servoing state (IBVS/PBVS)
std_msgs/Header header

# Operational state
uint8 IDLE=0
uint8 CONVERGING=1
uint8 TRACKING=2
uint8 LOST=3
uint8 ERROR=4
uint8 state

# Error metrics
float32[6] feature_error           # Current image feature error vector
float32 norm_error                 # L2 norm of feature error
float32 desired_norm_error         # Convergence threshold

# Jacobian
float32 condition_number           # Jacobian condition number (∞ = singular)

# Velocity command (output)
float32[6] camera_velocity         # [vx,vy,vz,wx,wy,wz] camera-frame velocity
float32[2] gimbal_velocity         # [yaw_rate, pitch_rate] rad/s
float32[3] chassis_velocity        # [vx,vy,wz] chassis twist

# Diagnostics
float32 convergence_rate           # Error reduction rate per iteration
uint32 iteration_count             # Servo iterations since activation
builtin_interfaces/Time last_update
