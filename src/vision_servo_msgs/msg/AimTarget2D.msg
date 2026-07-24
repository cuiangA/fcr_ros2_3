# AimTarget2D.msg — 单一人脸/上身/关键点瞄准点，像素级精确指示
# 所有坐标均为 Sony 原始图像像素坐标，保留源图像的 stamp 和 frame_id

std_msgs/Header header

# 源类型枚举
uint8 FACE=0               # 人脸检测框中心
uint8 KEYPOINT=1           # 身体关键点（如鼻尖）
uint8 PREDICTED_FACE=2     # 根据预测/卡尔曼滤波推得的人脸位置
uint8 UPPER_BODY=3         # 上身框中心
uint8 LOST_PREDICTION=4    # 目标丢失后的预测位置

# 关联的跟踪 ID
int32 tracking_id

# 像素坐标（Sony 原图坐标系）
float32 pixel_x
float32 pixel_y

# 置信度 [0.0, 1.0]
float32 confidence

# 数据来源类型
uint8 source

# 有效性标志（false 表示当前帧无有效观测）
bool valid
