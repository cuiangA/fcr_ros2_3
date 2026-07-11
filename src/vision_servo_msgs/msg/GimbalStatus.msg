# GimbalStatus.msg — Runtime state and transport diagnostics for the gimbal driver
std_msgs/Header header

# Motion state
float32 yaw
float32 pitch
float32 yaw_rate
float32 pitch_rate

# Hardware/transport state
bool connected
float32 last_rx_age_sec
uint32 tx_count
uint32 rx_count
uint32 crc_error_count
uint32 can_error_count
uint32 parse_error_count

# Command watchdog state
bool command_watchdog_enabled
bool active_command
