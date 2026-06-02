# TargetArray.msg — Multi-target detection/tracking result
std_msgs/Header header

Target[] targets                    # All detected targets
int32 tracking_id                  # ID of the currently tracked target (-1 if none)
