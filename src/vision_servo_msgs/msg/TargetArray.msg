# TargetArray.msg — Multi-target detection/tracking result.
# The array header is copied from the source Sony RGB frame. Every Target in the
# array must use the same timestamp and frame_id.
std_msgs/Header header

Target[] targets                    # All detected targets
int32 tracking_id                  # ID of the currently tracked target (-1 if none)
