# FCR ROS2 智能跟拍机器人项目功能与技术路线文档

本文档结合当前仓库实现和任务划分表编写，重点说明系统要实现哪些功能、每类功能采用什么技术路线，以及这些功能如何落到当前 ROS 2 工作区的包、节点、接口和控制链路中。

## 1. 项目定位

FCR ROS2 项目面向“移动机器人 + 稳定云台 + 视觉感知”的智能跟拍场景。系统通过相机识别人物或目标，持续估计目标在图像和三维空间中的状态，再协同控制底盘与 DJI RS2 云台，使目标稳定处于画面中的期望位置，并支持后续 Vlog 运镜、任务 Action、MPC 协同优化和人机交互。

项目的核心技术链可以概括为：

```text
Sony 图像输入
  -> YOLO 检测
  -> 多目标跟踪
  -> Target State Estimation
  -> IBVS/PBVS 视觉伺服
  -> Control Allocation 控制分配
  -> MPC/QP 协同优化
  -> 双轮足/移动底盘 + RS2 云台协同运动
```

当前代码仓库已经具备 ROS 2 Humble 多包结构、自定义接口、Sony CRSDK 图像驱动、YOLO ONNX/TensorRT 检测、2D 多目标跟踪、IBVS/PBVS 控制器、控制分配器、平台抽象层、Gazebo/2D/mock 仿真和 bringup 启动编排。Sony 驱动和 2D 感知代码仍待 Jetson 实机验收；奥比中光 335 读取、双相机标定与融合、语音/网页交互和 Action 运镜属于后续阶段。

## 2. 任务划分总览

| 技术类别 | 包含功能 | 共同技术要求 |
| --- | --- | --- |
| 1. ROS2 系统架构与接口层 | Topic/Service/Action、模式切换、参数配置、一键启动、节点编排 | ROS2 Humble、rclcpp/rclpy、msg/srv/action、自定义接口、QoS、launch、YAML 参数、Lifecycle 思路 |
| 2. 硬件驱动与设备抽象 | 双轮足底盘、DJI RS2 云台、Sony 相机、IMU、电池状态、奥比中光 335 深度相机 | 串口/CAN/USB/SDK、硬件抽象接口、Factory Pattern、设备状态反馈、超时检测、真实/仿真切换 |
| 3. 图像采集与视觉感知 | Sony 图像输入、YOLO 检测、人脸/人体检测、检测结果发布 | V4L2/采集卡/OpenCV、cv_bridge、image_transport、CameraInfo、YOLO、ONNX Runtime/TensorRT、NMS |
| 4. 目标跟踪与状态估计 | 多目标跟踪、目标选择、目标锁定、目标丢失、距离估计、速度估计 | SORT/ByteTrack、Kalman Filter、IoU 匹配、深度估计、bbox 估距、滤波、tf2 坐标变换 |
| 5. 底盘与云台基础控制 | 底盘遥控、云台遥控、云台回中、速度档位、手动接管 | `cmd_vel`、`GimbalCmd`、运动学、速度控制、姿态反馈、限幅、低速调试模式 |
| 6. 视觉伺服与跟随控制 | 云台自动跟随、底盘距离保持、底盘-云台协同、自动构图 | IBVS、PBVS、PID/P 控制、ControlAllocator、图像误差、距离误差、构图模板、死区、低通滤波 |
| 7. 优化控制与高级协同 | MPC、QP 控制分配、平滑跟随、云台回中策略、约束控制 | MPC、QP、OSQP/qpOASES、预测时域、代价函数、速度/加速度/jerk 约束、云台限位约束 |
| 8. 任务规划与运镜 Action | 定点摇镜、推近、拉远、环绕、侧向跟拍、半环绕推近 | ROS2 Action、轨迹生成、圆弧轨迹、S 曲线、Bezier/多项式插值、Action cancel/timeout |
| 9. 人机交互与上层指令 | 遥控、语音控制、网页/可视化控制、状态查询 | joystick、keyboard teleop、ASR、Intent Parser、Web UI、Service 调用、命令优先级仲裁 |
| 10. 安全、仿真、测试与评估 | 急停、安全层、Gazebo/2D 仿真、rosbag、RViz/Foxglove、性能指标 | Safety filter、watchdog、Gazebo、URDF/Xacro、mock 数据、rosbag2、RViz Marker、Foxglove、误差统计 |

这些任务可以按下面的实施主线组织：

```text
ROS2 架构
  -> 硬件驱动
  -> 图像感知
  -> 目标跟踪与状态估计
  -> 基础遥控
  -> 视觉伺服跟随
  -> MPC 协同优化
  -> Action 运镜
  -> 语音/遥控交互
  -> 安全、仿真、测试评估
```

## 3. 当前仓库承载关系

| 当前包 | 承载任务类别 | 主要职责 |
| --- | --- | --- |
| `vision_servo_msgs` | 1、4、5、6、8 | 定义目标、平台状态、云台命令、伺服状态、模式切换服务和视觉伺服 Action |
| `perception_pkg` | 3、4 | Sony 图像输入、YOLO ONNX 检测、多目标跟踪；深度融合留待后续阶段 |
| `remote_monitor_pkg` | 9、10 | 只读视觉标注、结构化健康/性能状态、Foxglove 局域网桥接与默认布局 |
| `servo_control_pkg` | 5、6、7、8 | MVP 跟拍、IBVS/PBVS、控制器插件、控制分配、VisualServo Action |
| `robot_platform_pkg` | 2、5、10 | 底盘、云台、IMU、里程计、平台状态聚合、真实/仿真硬件抽象 |
| `simulation_pkg` | 3、4、5、6、10 | Gazebo 模型、目标仿真、相机仿真、mock 检测、2D 闭环仿真 |
| `bringup_pkg` | 1、9、10 | 生产/仿真/调试一键启动、RViz/Foxglove 可视化入口 |

### 3.1 当前项目进度

按照当前 README、源码和配置文件的状态，项目处于 V2.5 阶段：MVP 跟拍闭环、滤波/死区/限幅、目标管理、IBVS/PBVS 控制器以及 Sony + YOLO + 2D 跟踪代码已经具备；当前感知链待 Jetson/相机验收，双相机融合、真实底盘通信、MPC/QP、运镜 Action 和上层交互仍需继续完成。

| 阶段 | 目标能力 | 当前进度 | 进度说明 |
| --- | --- | --- | --- |
| V1 MVP 跟拍 | 图像误差控制、距离控制、云台偏角补偿 | 约 90% | MVP 控制链完整，可通过 mock target 和 2D 仿真验证；YOLO 代码已接入，待真实模型与相机验证 |
| V2 稳定化 | 滤波、死区、限幅、目标锁定、目标丢失处理 | 约 85% | `mvp_follow_controller_node` 已具备低通滤波、死区、限幅和目标超时；跟踪层已有目标 ID 管理 |
| V3 混合视觉伺服 | IBVS、PBVS、底盘-云台控制分配、运行时模式切换 | 约 70% | IBVS/PBVS、`ServoManagerNode`、`ControlAllocator` 和 `SetServoMode` 已实现；HYBRID/MPC/RL 模式仍是扩展接口 |
| V4 高级优化 | MPC、QP 控制分配、RL、运镜轨迹优化 | 约 5% | 已有 MPC/RL 头文件和接口设想，尚未接入 QP 求解器和完整运镜任务 |

按任务划分来看，当前进度可归纳为：

| 技术类别 | 当前进度 | 已具备能力 | 下一步技术路线 |
| --- | --- | --- | --- |
| 1. ROS2 系统架构与接口层 | 基本完成 | 6 个 ROS 2 包、自定义 msg/srv/action、QoS 封装、分阶段 launch、YAML 参数体系 | 引入更明确的 Lifecycle/状态机管理，并为运镜 Action 和交互节点继续扩展接口 |
| 2. 硬件驱动与设备抽象 | 云台实机链路已打通，Sony代码待验收，底盘/IMU待接入 | Sony CRSDK驱动及诊断、底盘/云台/IMU抽象接口、Factory Pattern、`use_sim` 实/仿切换、PlatformState聚合；RS2 bringup见 [`rs2_gimbal_bringup.md`](rs2_gimbal_bringup.md) | 验收Sony驱动，继续接入底盘、IMU、电池和奥比中光深度相机 |
| 3. 图像采集与视觉感知 | 代码收口，待 Jetson/相机验收 | Sony CRSDK驱动、CameraInfo、YOLOv5/v8 ONNX/TensorRT、可测前后处理、按类别NMS、低延迟队列与诊断 | 完成Sony标定、已知图片对齐、ROS链路与30分钟稳定性验收 |
| 4. 目标跟踪与状态估计 | 2D代码收口，待实景验收 | Kalman+Hungarian全局IoU关联、连续确认、CONFIRMED/LOST、目标选择与失败反馈 | 先验证Sony单相机多人跟踪；双相机融合另立阶段，复杂场景再升级ByteTrack/ReID |
| 5. 底盘与云台基础控制 | 云台键盘/语音实机可控，底盘待接入 | `/cmd_vel`、`GimbalCmd`、三轮全向运动学、云台绝对位置累加控制、低速限幅和平台反馈接口 | 固化云台限位和诊断，继续接入真实底盘、手动接管和速度档位 |
| 6. 视觉伺服与跟随控制 | 核心完成 | MVP P 控制、IBVS、PBVS、50Hz ServoManager、控制分配、Action feedback | 补齐自动构图模板、HYBRID 模式、TF 驱动分配和实机闭环参数整定 |
| 7. 优化控制与高级协同 | 接口预留 | MPC/RL 头文件、ControlAllocator 优化模式设想 | 接入 OSQP/qpOASES，实现 MPC 控制器和 QP 控制分配 |
| 8. 任务规划与运镜 Action | 基础 Action 已有 | `VisualServo.action` 可表达长周期伺服任务 | 扩展推近、拉远、环绕、侧向跟拍等 Cinematic Action 和轨迹生成器 |
| 9. 人机交互与上层指令 | 尚未展开 | 当前主要通过 launch、Service 和 RViz/Foxglove 调试入口操作 | 增加 joystick/keyboard teleop、Web UI、语音意图解析和命令优先级仲裁 |
| 10. 安全、仿真、测试与评估 | 仿真基础完成 | Gazebo、URDF/Xacro、target simulator、mock target、2D sim、RViz/Foxglove 入口 | 增加 safety filter、watchdog、rosbag2 评估脚本和误差统计指标 |

当前最完整的能力链路是：

```text
mock/仿真 Target
  -> 目标跟踪或 3D target 输入
  -> MVP / IBVS / PBVS 控制
  -> ControlAllocator
  -> /cmd_vel + /cmd_gimbal
  -> 仿真平台状态反馈
```

当前最需要补齐的真实产品链路是：

```text
Sony/深度相机真实图像
  -> YOLO 推理
  -> 目标跟踪与 3D 状态估计
  -> 真实底盘 + RS2 云台通信
  -> 实机视觉伺服闭环
```

### 3.2 分层架构

项目整体可以分成“纵向业务层 + 横向公共支撑层”两部分。纵向业务层从上到下依次把用户意图转成机器人运动：人机交互与任务层负责产生任务，启动编排层负责组织节点，视觉伺服层负责计算控制量，感知层负责提供目标状态，平台层负责抽象硬件，最底层是真实或仿真的传感器与执行器。横向公共支撑层由 `vision_servo_msgs`、QoS、TF 坐标系、仿真和评估工具组成，贯穿所有功能模块。

| 层级 | 名称 | 当前主要包/节点 | 职责 |
| --- | --- | --- | --- |
| L6 | 人机交互与任务层 | 规划中的 Teleop/Web/Voice、`VisualServo.action` | 接收操作员意图，触发跟拍、模式切换和运镜任务 |
| L5 | 任务编排与启动层 | `bringup_pkg`、RViz2、Foxglove | 分阶段启动平台、感知、控制和可视化节点 |
| L4 | 视觉伺服与运动控制层 | `servo_control_pkg`、`ServoManagerNode`、IBVS/PBVS、`ControlAllocator` | 根据目标状态计算相机速度，并分配到底盘和云台 |
| L3 | 目标状态与视觉感知层 | `perception_pkg`、Detection/Tracking | 当前从 Sony 图像生成稳定 2D 目标轨迹；后续扩展深度状态 |
| L2 | 平台状态与硬件抽象层 | `robot_platform_pkg`、PlatformManager、DriverNodes | 将底盘、云台、IMU、里程计统一成 ROS 2 状态和命令接口 |
| L1 | 传感器与执行器层 | Sony/深度相机、底盘、RS2、IMU、电池、Gazebo 后端 | 真实硬件或仿真设备，执行控制命令并产生传感器数据 |
| 横向 | 公共接口与仿真评估 | `vision_servo_msgs`、QoS、TF、`simulation_pkg`、rosbag2 | 统一消息类型、通信策略、坐标系、仿真输入和指标评估 |

对应 UML 图位于 `docs/uml/09_layered_architecture.puml`。该图的主数据流是：

```text
人机/任务指令
  -> ServoManager / VisualServo Action
  -> IBVS/PBVS/MPC 控制器
  -> ControlAllocator
  -> ChassisDriver + GimbalDriver
  -> PlatformState 反馈
```

## 4. ROS2 系统架构与接口层

### 功能目标

系统架构层负责把感知、控制、平台、仿真和上层任务组织成一个可扩展的 ROS 2 系统。核心功能包括：

- 定义跨包通信接口。
- 组织 Topic、Service、Action 的数据流。
- 支持控制模式切换。
- 支持 YAML 参数配置和 launch 参数覆盖。
- 支持生产、仿真、调试等不同启动组合。
- 为后续 Lifecycle、状态机和多机器人命名空间预留结构。

### 技术路线

接口层采用独立的 `vision_servo_msgs` 包，避免功能包之间直接依赖对方内部实现。连续数据使用 Topic，短指令使用 Service，长周期任务使用 Action：

| 通信方式 | 适用功能 | 当前接口 |
| --- | --- | --- |
| Topic | 图像、检测结果、目标状态、平台状态、速度指令 | `TargetArray`、`PlatformState`、`ServoState`、`GimbalCmd` |
| Service | 模式切换、目标选择、标定触发 | `SetServoMode`、`SetTrackingTarget`、`CalibrateCamera` |
| Action | 可取消、可反馈、持续一段时间的跟拍/运镜任务 | `VisualServo.action` |

QoS 策略按数据语义划分：

- 图像和深度图使用 `SensorDataQoS`，优先低延迟。
- 检测、跟踪和目标状态使用可靠传输，保证控制输入稳定。
- 相机内参使用 `TRANSIENT_LOCAL`，保证后启动节点也能获得最新标定。
- 底盘和云台指令使用可靠传输，避免控制命令丢失。
- 平台状态使用可靠 + `TRANSIENT_LOCAL`，便于控制器和可视化节点获取当前状态。

启动层采用 Python launch。顶层 bringup 以阶段化方式组织系统：

```text
t=0s   平台驱动层
t=2s   感知管线
t=3s   伺服控制
t=4s   RViz/Foxglove 可视化
```

这种结构适合后续继续扩展语音节点、Web 控制节点、运镜任务节点和安全监控节点。

## 5. 硬件驱动与设备抽象

### 功能目标

硬件驱动层负责把真实设备封装为统一 ROS 2 接口，使控制层无需关心底层通信协议。任务划分中的硬件包括：

- 双轮足底盘或其他移动底盘。
- DJI RS2 云台。
- Sony 主相机。
- 奥比中光 335 深度相机。
- IMU。
- 电池与设备状态。

当前仓库已有 `robot_platform_pkg`，可承载底盘、云台、IMU、里程计和平台状态；相机与深度相机可作为后续独立 driver 或接入现有 perception/simulation 入口。

### 技术路线

硬件层采用“抽象接口 + 工厂函数 + 参数切换”的方式组织：

```text
DriverNode
  -> IChassisInterface / IGimbalInterface / IIMUInterface
  -> Factory: make_real_xxx() 或 make_simulated_xxx()
  -> use_sim 参数决定真实后端或仿真后端
```

底盘驱动技术路线：

- 命令入口统一为 `cmd_vel`。
- 控制量为线速度和角速度。
- 底层通信根据具体底盘选择串口、CAN 或 SDK。
- 驱动节点负责速度限幅、速度档位、低速调试模式和手动接管接口。
- 状态反馈包括当前速度、里程计、电池电压、急停状态和连接状态。

云台驱动技术路线：

- 命令入口统一为 `GimbalCmd`。
- 控制量为 yaw/pitch 角速度，也可扩展 position hold 和回中命令。
- DJI RS2 当前通过 SocketCAN `can0` 封装，host -> gimbal 使用 `0x223`，gimbal -> host 使用 `0x222`。
- 实机已验证 `0E 00` 绝对位置控制可用；`0E 01` 速度控制暂未执行，当前驱动采用“速度输入 -> 绝对目标角累加 -> 绝对位置帧”的工程方案。
- 云台状态反馈包括 yaw、pitch、yaw_rate、pitch_rate、回中状态和限位状态。
- 云台回中策略既可作为基础控制功能，也可作为高级构图和 MPC 的约束条件。

Sony 相机和奥比中光深度相机技术路线：

- Sony 图像当前通过 Sony Camera Remote SDK 的 live view 接入。
- 图像发布为 `sensor_msgs/Image`，内参发布为 `sensor_msgs/CameraInfo`。
- 奥比中光 335 的深度读取、与 Sony 的标定、时间同步和 TF 对齐留到融合阶段。
- 图像链路使用 `image_transport`，后续可接压缩传输或硬件编码。

设备状态路线：

- 所有设备状态聚合为 `PlatformState` 或后续扩展的 diagnostics 消息。
- 平台管理节点统一发布底盘、云台、IMU、电池和系统模式。
- 上层控制器只订阅平台状态，不直接访问硬件节点。

## 6. 图像采集与视觉感知

### 功能目标

视觉感知层负责把相机图像转换成候选目标列表，包括目标类别、置信度、二维 bbox 和中心点。目标检测既要支持人体检测，也要预留人脸、车辆、自定义物体等类别。

### 技术路线

图像输入链路：

```text
Sony Camera / Video Capture
  -> Image Driver
  -> /sony/image_raw
  -> /sony/camera_info
  -> DetectionNode
```

检测链路：

```text
Image
  -> cv_bridge 转 OpenCV Mat
  -> YOLO 前处理
  -> OpenCV DNN ONNX 或 TensorRT 推理
  -> NMS 后处理
  -> TargetArray 发布
```

YOLO 检测路线：

- 模型优先使用 YOLOv8 ONNX 作为通用实现。
- 在 Jetson 或 GPU 平台上使用 TensorRT engine 提升帧率。
- 前处理包括 resize、letterbox、归一化、通道转换。
- 后处理包括置信度过滤、类别过滤、NMS 和 bbox 坐标还原。
- 输出转换为 `vision_servo_msgs/TargetArray`。

检测输出的字段约定：

- `bbox[4]`: `[x_min, y_min, x_max, y_max]`。
- `center[2]`: 目标中心点。
- `class_name`: 例如 person、face、自定义目标。
- `confidence`: 检测置信度。
- `header.frame_id`: 相机坐标系。

后续扩展方向：

- 人脸检测与人体检测并行，形成“人脸优先、人体兜底”的跟拍目标。
- 加入目标类别白名单和黑名单。
- 加入检测结果可视化 overlay 或 RViz marker。

## 7. 目标跟踪与状态估计

### 功能目标

目标跟踪与状态估计层负责把每帧独立检测结果变成连续、稳定、可控制的目标状态。核心功能包括：

- 多目标 ID 维护。
- 目标选择与锁定。
- 目标短时丢失保持。
- 目标距离估计。
- 目标速度估计。
- 图像坐标、相机坐标和机器人坐标之间的转换。

### 技术路线

多目标跟踪采用 SORT/ByteTrack 风格：

```text
Detections
  -> Kalman predict
  -> IoU association
  -> Kalman correct
  -> track create/delete
  -> confirmed tracks
```

当前跟踪器使用 8 维状态：

```text
[x, y, w, h, vx, vy, vw, vh]
```

其中 `x,y,w,h` 来自检测框，速度项由卡尔曼滤波估计。目标轨迹需要满足 `min_hits` 才输出，短时未匹配目标通过 `max_age` 保留。

目标选择路线：

- 自动选择最大面积或最高置信度目标。
- 支持按 `target_id` 锁定目标。
- 支持按 `class_name` 过滤目标。
- 后续可加入人脸优先级、目标距离优先级、画面中心优先级。

距离和 3D 状态估计路线：

```text
Target bbox + CameraInfo + Depth Image
  -> 深度图中心采样
  -> bbox 尺寸估距作为回退
  -> 针孔模型反投影
  -> Target.position
```

反投影公式：

```text
X = (u - cx) / fx * Z
Y = (v - cy) / fy * Z
Z = depth
```

速度估计路线：

- 对连续帧 `Target.position` 做差分得到目标速度。
- 对速度做低通滤波，减少检测抖动。
- 后续可使用 tf2 将目标速度从 camera frame 转到 base_link 或 odom frame。

目标丢失处理路线：

- 跟踪层短时保持目标 ID。
- 控制层检测目标超时后进入保持或搜索状态。
- 重新检测到同类目标后尝试恢复锁定。

## 8. 底盘与云台基础控制

### 功能目标

基础控制层先让机器人“可控、可接管、可调试”。它不依赖完整视觉伺服，也可以单独运行，用于底盘、云台和手动控制验证。

功能包括：

- 底盘遥控。
- 云台遥控。
- 云台回中。
- 速度档位。
- 手动接管。
- 低速调试模式。
- 基础限幅和姿态反馈。

### 技术路线

底盘基础控制：

- 输入接口使用 `geometry_msgs/Twist` 或 `TwistStamped`。
- 手柄、键盘、网页或上层 planner 均转换为 `cmd_vel`。
- 底盘 driver 根据运动学模型将速度转换为底层执行器命令。
- 对线速度和角速度设置分级限幅，例如低速、中速、高速。

云台基础控制：

- 输入接口使用 `GimbalCmd`。
- 上层接口仍使用 yaw/pitch 角速度输入。
- RS2 实机当前采用驱动内部积分为绝对位置目标，再发送 `0E 00` 绝对位置控制帧执行。
- 支持 `hold_yaw` 和 `hold_pitch`。
- 支持回中服务或回中 Action。
- 云台姿态进入 `PlatformState`，供自动跟随和构图控制使用。

手动接管路线：

```text
Manual Command
  -> command arbiter
  -> chassis/gimbal command
```

命令优先级建议：

```text
E-stop > Manual override > Action cinematic task > Auto follow > Idle hold
```

这样后续接入语音、遥控器和 Web 控制时，可以避免多源命令同时控制底盘和云台。

## 9. 视觉伺服与跟随控制

### 功能目标

视觉伺服层是系统核心。它负责让目标在画面中保持稳定构图，并让底盘与目标保持期望距离。

功能包括：

- 云台自动跟随。
- 底盘距离保持。
- 底盘-云台协同。
- 自动构图。
- 目标锁定后的稳定跟拍。
- 目标移动时的平滑响应。

### 技术路线一：MVP 跟拍控制

MVP 控制采用三通道解耦：

```text
图像水平误差 ex -> 云台 yaw
图像垂直误差 ey -> 云台 pitch
距离误差 ez     -> 底盘 vx
云台 yaw 偏角   -> 底盘 wz
```

控制律：

```text
gimbal_yaw   = -Kx * e_x
gimbal_pitch = -Ky * e_y
base_vx      =  Kz * (Z - Z_desired)
base_wz      =  Kb * q_yaw
```

信号处理路线：

```text
raw error
  -> low-pass filter
  -> deadband
  -> P gain
  -> command low-pass filter
  -> clamp
  -> cmd_vel / cmd_gimbal
```

这种结构简单稳定，适合早期验证和快速调参。

### 技术路线二：IBVS

IBVS 在图像特征空间中直接控制目标位置。当前控制器使用 bbox 的 3 个点作为 6 维图像特征：

```text
s = [x_lt, y_lt, x_rb, y_rb, x_rt, y_rt]
```

控制律：

```text
v_camera = -lambda * L_plus * (s - s*)
```

技术步骤：

1. 从 `Target.bbox` 提取归一化图像点。
2. 根据目标深度计算 6x6 交互矩阵。
3. 通过 SVD 阻尼伪逆计算相机速度。
4. 对速度进行线速度和角速度限幅。
5. 将相机速度交给 `ControlAllocator`。

IBVS 适合图像中心稳定跟踪，对相机标定误差相对更鲁棒。

### 技术路线三：PBVS

PBVS 先估计目标在相机坐标系下的位置，再在三维空间中控制相对位姿。

控制律：

```text
v_trans = K_t * (P_current - P_desired)
omega   = K_r * angle * rotation_axis
```

技术步骤：

1. 优先使用 `Target.position`。
2. 如果没有 3D position，则由 bbox 和深度反投影。
3. 期望目标位置设为相机正前方 `desired_depth`。
4. 计算平移误差和视线方向误差。
5. 输出 6-DOF 相机速度。

PBVS 适合深度可靠的场景，距离控制和空间路径更直观。

### 技术路线四：底盘-云台控制分配

IBVS/PBVS 输出的是相机 6-DOF 速度，而执行器是底盘和云台。因此需要控制分配：

```text
camera velocity
  -> coordinate mapping / tf2 transform
  -> chassis twist
  -> gimbal yaw/pitch rate
```

当前优先级策略：

- 云台优先承担快速角度修正。
- 底盘承担平移和低频大角度修正。
- `allocation_ratio` 控制旋转分配比例。
- 输出经过限幅和一阶低通滤波。

后续自动构图可以在期望特征 `s*` 中体现，例如：

- 人物居中。
- 人物偏左留出运动方向空间。
- 半身/全身构图。
- 推近、拉远、侧向跟拍时动态改变期望 bbox 尺寸和位置。

## 10. 优化控制与高级协同

### 功能目标

高级协同层的目标是让跟随更平滑、更符合拍摄需求，并在底盘、云台和画面约束之间做统一优化。

功能包括：

- MPC 视觉伺服。
- QP 控制分配。
- 平滑跟随。
- 云台回中策略。
- 速度、加速度、jerk 约束。
- 云台角限位约束。
- 目标视野边界约束。

### 技术路线

MPC 控制器可以基于图像特征模型：

```text
s_{k+1} = s_k + L(s_k, Z_k) * u_k * dt
```

优化目标：

```text
min Σ ||s_k - s_ref||_Q^2
    + Σ ||u_k||_R^2
    + Σ ||Δu_k||_S^2
```

约束：

- 底盘最大线速度和角速度。
- 云台最大 yaw/pitch 角速度。
- 云台 yaw/pitch 角限位。
- 图像特征保持在画面边界内。
- 加速度和 jerk 平滑约束。

QP 控制分配路线：

```text
desired camera velocity
  -> decision variables: chassis velocity + gimbal velocity
  -> constraints: actuator limits + gimbal angle limits
  -> cost: tracking error + smoothness + gimbal recentering
  -> OSQP / qpOASES
```

云台回中策略：

- 当目标稳定居中时，底盘逐步旋转，让云台 yaw 回到中位。
- 云台只负责高频抖动和短时偏差。
- 底盘负责长期航向调整，减少云台接近限位的概率。

## 11. 任务规划与运镜 Action

### 功能目标

运镜层让机器人不仅“跟上目标”，还要“拍得像 Vlog”。任务包括：

- 定点摇镜。
- 推近。
- 拉远。
- 环绕。
- 侧向跟拍。
- 半环绕推近。
- 任务取消和超时处理。

### 技术路线

使用 ROS 2 Action 表达长周期运镜任务：

```text
Cinematic Action Goal
  -> trajectory generation
  -> desired framing / desired relative pose
  -> visual servo controller
  -> chassis + gimbal command
  -> feedback / result
```

轨迹生成方式：

- 推近/拉远：沿目标连线方向生成距离参考。
- 环绕：以目标为圆心生成圆弧轨迹。
- 侧向跟拍：保持目标深度，同时控制横向相对位移。
- 半环绕推近：圆弧轨迹和深度参考同时变化。

轨迹平滑方式：

- S 曲线速度规划。
- Bezier 曲线。
- 多项式插值。
- 限制速度、加速度和 jerk。

Action feedback 可以输出：

- 当前任务进度。
- 当前构图误差。
- 当前目标距离。
- 当前底盘/云台速度。
- 是否接近云台限位。

当前仓库已有 `VisualServo.action`，可作为自动跟随和简单运镜的基础；后续可以增加 `CinematicShot.action` 或扩展现有 action goal 字段。

## 12. 人机交互与上层指令

### 功能目标

人机交互层把操作员意图转换为系统命令，让系统从“算法演示”变成可操作产品。

功能包括：

- 遥控器控制。
- 键盘控制。
- 语音控制。
- Web UI / 可视化控制。
- 状态查询。
- 模式切换。
- 目标选择。
- 运镜任务触发。

### 技术路线

遥控和键盘：

- joystick 或 keyboard teleop 生成手动底盘/云台命令。
- 手动命令进入 command arbiter。
- 手动接管优先级高于自动跟随和运镜任务。

语音控制：

```text
audio
  -> ASR
  -> intent parser
  -> ROS2 Service / Action client
```

典型意图：

- “开始跟拍”
- “停止”
- “锁定这个人”
- “推近一点”
- “环绕拍摄”
- “云台回中”

Web UI：

- 展示视频流、目标框、系统状态和当前模式。
- 通过服务调用切换 IBVS/PBVS/MPC 模式。
- 通过 Action 发送跟拍和运镜任务。
- 展示 `ServoState`、`PlatformState` 和任务反馈。

命令仲裁：

```text
E-stop
  > 手动接管
  > 运镜 Action
  > 自动跟随
  > 空闲保持
```

这样可以让语音、Web、遥控器和自动控制共存。

## 13. 安全、仿真、测试与评估

### 功能目标

安全与评估层保证系统可验证、可展示、可量化。它贯穿硬件、感知、控制和运镜任务。

功能包括：

- 急停。
- 安全限幅。
- watchdog。
- Gazebo 仿真。
- 2D 快速仿真。
- mock 数据。
- rosbag 录制与回放。
- RViz/Foxglove 可视化。
- 误差与性能指标统计。

### 技术路线

安全过滤：

```text
raw command
  -> velocity limit
  -> acceleration limit
  -> workspace / gimbal limit
  -> watchdog timeout
  -> final command
```

仿真路线：

- Gazebo 用于完整模型、底盘运动、云台关节和传感器联调。
- 2D 仿真用于快速验证跟拍控制逻辑。
- mock target 用于固定场景测试，例如 left/right/far/near/lost/sinusoidal。

测试评估路线：

- rosbag2 录制图像、目标、控制命令和平台状态。
- 离线统计图像误差、距离误差、控制平滑度和目标丢失次数。
- RViz 显示 TF、目标位置、机器人轨迹和 marker。
- Foxglove 展示时间序列和 Web 监控界面。

远程视觉观察层由 `remote_monitor_pkg` 提供。它按图像时间戳匹配
`/sony/image_raw`、`/perception/detections` 与 `/perception/tracks`，输出保留原始
`stamp` 和 `frame_id` 的 `/perception/tracking_image`，并通过
`/perception/monitor_status` 发布相机、检测器、跟踪器健康状态以及 FPS、延迟和
当前目标信息。`foxglove_bridge` 默认监听 `0.0.0.0:8765`，只开放观察白名单，
不允许远程发布控制话题、调用服务或修改参数。无线查看优先选择
`/perception/tracking_image/compressed`。

常用指标：

- 图像中心误差均值/峰值。
- 目标距离误差均值/峰值。
- 目标丢失次数和恢复时间。
- 云台角速度峰值。
- 底盘速度和加速度峰值。
- 控制指令抖动量。
- 运镜任务完成时间和最终构图误差。

## 14. 分阶段实施路线

| 实施方向 | 对应技术类别 | 阶段目标 |
| --- | --- | --- |
| 先让硬件动起来 | 1、2、5、10 | 底盘、云台、IMU、相机可以独立运行，并能通过 ROS 2 统一控制 |
| 让机器人看见目标 | 3、4 | Sony 图像进入 YOLO，检测结果经过跟踪和深度估计形成目标状态 |
| 让机器人稳定跟随 | 4、5、6、10 | MVP、IBVS、PBVS 和控制分配形成稳定闭环 |
| 让机器人拍得更像 Vlog | 6、7、8 | 自动构图、MPC/QP 协同、推拉摇移和环绕运镜 |
| 让系统更像完整产品 | 1、8、9、10 | 加入遥控、语音、Web、状态查询、任务管理和安全仲裁 |
| 形成最终技术亮点 | 7、8、10 | 用优化控制、运镜 Action 和量化评估形成项目展示重点 |

### 阶段一：ROS 2 与硬件基础

目标是完成系统骨架和基础硬件控制：

- 自定义接口稳定。
- 平台 driver 接入底盘、云台、IMU 和电池状态。
- 相机 driver 发布图像和 CameraInfo。
- 底盘和云台支持手动遥控。
- RViz/Foxglove 能观察平台状态。

输出成果：

- 可启动的基础机器人系统。
- 可手动控制的底盘和云台。
- 可观察的图像、IMU、里程计和平台状态。

### 阶段二：视觉感知与目标状态

目标是完成“看见目标”和“锁定目标”：

- YOLO 人体/人脸检测。
- TargetArray 输出。
- 多目标跟踪和 ID 保持。
- 目标选择和目标锁定。
- 深度估计和目标 3D position。

输出成果：

- 稳定目标 ID。
- 目标距离和速度估计。
- 可供控制器直接消费的目标状态。

### 阶段三：基础跟随闭环

目标是完成稳定跟拍：

- MVP P 控制闭环。
- 云台自动跟随。
- 底盘距离保持。
- 底盘根据云台偏角修正航向。
- 加入死区、低通滤波和限幅。

输出成果：

- 目标能稳定保持在画面中心附近。
- 机器人能保持期望距离。
- mock、2D sim 和 Gazebo 中均可演示。

### 阶段四：IBVS/PBVS 与协同控制

目标是从简单 P 控制升级为视觉伺服控制：

- IBVSController 使用图像特征和交互矩阵。
- PBVSController 使用 3D 位姿误差。
- ServoManager 支持模式切换。
- ControlAllocator 协同底盘和云台。
- 自动构图模板进入期望特征或期望位姿。

输出成果：

- IBVS/PBVS 可切换。
- 底盘和云台协同更自然。
- 具备论文和答辩中的核心算法内容。

### 阶段五：MPC/QP 和运镜 Action

目标是形成高级技术亮点：

- MPC 控制器考虑预测时域。
- QP 控制分配统一底盘、云台和构图约束。
- Action 支持推近、拉远、环绕、侧向跟拍。
- 运镜轨迹平滑且可取消。

输出成果：

- 更平滑的跟拍和更明确的运镜任务。
- 可量化对比 IBVS/PBVS/MPC 的控制效果。
- 项目展示从“能跟随”升级为“会拍摄”。

### 阶段六：交互、安全与评估

目标是让系统接近完整产品形态：

- 遥控、语音和 Web UI 接入。
- 命令优先级仲裁。
- 急停和 watchdog。
- rosbag 数据采集。
- 误差、速度、平滑性等指标统计。

输出成果：

- 操作员可以自然控制系统。
- 系统状态可观察、可记录、可评估。
- 项目具备完整演示闭环。

## 15. 关键功能链路详解

### 15.1 自动跟拍链路

```text
Sony Image
  -> YOLO person detection
  -> TargetArray
  -> MultiObjectTracker
  -> selected target
  -> depth / bbox distance
  -> IBVS or PBVS
  -> ControlAllocator
  -> cmd_vel + GimbalCmd
```

功能解释：

- YOLO 负责找到人。
- 跟踪器负责保持同一个人的 ID。
- 深度估计负责给出距离。
- IBVS/PBVS 负责算出相机应该如何运动。
- 控制分配负责让底盘和云台共同完成这个运动。

### 15.2 自动构图链路

```text
composition template
  -> desired image feature / desired bbox size
  -> visual servo error
  -> camera velocity
  -> chassis + gimbal command
```

构图模板可以包括：

- 居中构图。
- 人物偏左或偏右。
- 半身构图。
- 全身构图。
- 运动方向留白。

技术上，构图模板最终会转换为 IBVS 的期望特征 `s*` 或 PBVS 的期望相对位姿。

### 15.3 运镜 Action 链路

```text
shot command
  -> action goal
  -> trajectory generator
  -> desired relative pose over time
  -> visual servo tracking
  -> feedback / result
```

例如“环绕拍摄”：

- Action goal 指定目标 ID、半径、角度、时长。
- 轨迹生成器生成相对目标的圆弧轨迹。
- PBVS/MPC 将当前相对位姿跟踪到期望轨迹。
- 控制分配器将运动拆到底盘和云台。
- Action feedback 返回进度和构图误差。

## 16. 当前代码与任务主线的对应结论

当前仓库已经覆盖任务主线中的核心中段：

```text
目标跟踪与状态估计
  -> 视觉伺服跟随
  -> 底盘/云台协同控制
  -> 仿真与可视化
```

现有代码最适合作为以下方向的基础：

- `vision_servo_msgs` 继续作为全系统接口层。
- `perception_pkg` 继续扩展 Sony 图像输入、YOLO 和深度相机融合。
- `servo_control_pkg` 继续承载 MVP、IBVS、PBVS、MPC、QP 和运镜 Action。
- `robot_platform_pkg` 继续承载双轮足底盘、RS2 云台、IMU、电池和设备状态。
- `simulation_pkg` 继续承载快速仿真、Gazebo 验证和 mock 数据生成。
- `bringup_pkg` 继续承载一键启动、调试启动和可视化启动。

最终技术重点建议集中在三条线上：

1. **视觉状态链路**：Sony 图像 -> YOLO -> 跟踪 -> 深度/速度估计。
2. **协同控制链路**：IBVS/PBVS -> ControlAllocator -> 底盘 + RS2 云台。
3. **高级拍摄链路**：MPC/QP -> 运镜 Action -> Vlog 风格构图与平滑运动。
