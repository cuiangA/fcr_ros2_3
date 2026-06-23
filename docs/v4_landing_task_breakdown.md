# FCR ROS2 V4 落地任务单元表

更新时间：2026-06-18

当前项目状态按现有仓库和规划文档估计为 V2.5：MVP 跟拍闭环、2D 可视化仿真、滤波/死区/限幅、IBVS/PBVS 基础类和控制分配框架已有；主要缺口是实机硬件适配、真实感知、V3 架构拆分、V4 优化控制、语音交互和系统化测试。

## 阶段定义

| 阶段 | 目标 | 成果边界 |
|---|---|---|
| V1 MVP | 跟拍闭环跑通 | 能产生 `/target/current`，控制器输出 `/cmd_vel` 和 `/cmd_gimbal`，仿真或实机可见动作 |
| V2 稳定版 | 展示稳定 | 目标锁定、滤波、死区、限幅、丢失保护、调参文档完整 |
| V2.5 交互增强 | 可演示增强 | Foxglove/RViz 面板、语音开始/停止/急停/回中等交互可用 |
| V3 混合视觉伺服 | 结构化算法 | 云台 IBVS、底盘 PBVS、控制分配、TF 目标坐标转换拆分清楚 |
| V4 优化协同控制 | 高级创新 | QP/MPC 原型、约束优化、对比实验和最终演示报告 |

## 总任务表

| ID | 阶段 | 分类 | 任务单元 | 主要内容 | 交付物 | 验收标准 | 依赖 |
|---|---|---|---|---|---|---|---|
| A1 | V2.5 | 项目基线 | 当前版本运行基线 | 梳理 mock、2D sim、Gazebo、Foxglove/RViz 的启动命令 | `docs/runbook_current.md` | 新成员按文档可在 30 分钟内跑通 MVP 2D 仿真 | 无 |
| A2 | V2.5 | 项目基线 | Node/Topic 总图 | 绘制当前节点、话题、服务、TF 数据流 | `docs/system_graph.md` | 图中覆盖 `/target/current`、`/platform/state`、`/cmd_vel`、`/cmd_gimbal`、`/odom`、`/tf` | A1 |
| A3 | V2.5 | 项目基线 | 参数总表 | 整理 MVP、IBVS、PBVS、硬件、仿真的参数含义和推荐范围 | `docs/parameter_reference.md` | 每个关键参数有默认值、含义、影响、推荐调参范围 | A1 |
| A4 | V2.5 | 项目基线 | 问题清单 | 记录当前 TODO、占位实现、已知风险和优先级 | `docs/open_issues.md` | 与 README 当前状态一致，能指导下一阶段任务选择 | A1 |
| A5 | V2.5 | 项目基线 | 分支和提交规范 | 制定任务分支、commit、PR 或 push 规则 | `docs/development_workflow.md` | 新成员知道如何拉取、开发、测试、提交 | 无 |
| B1 | V2.5 | 模型仿真 | URDF/xacro 结构检查 | 检查 LeKiwi 或目标底盘模型的 frame 命名、坐标轴、关节层级 | URDF 检查报告 | RViz 中 `base_link`、`camera_link`、`camera_optical_link` 方向正确 | A2 |
| B2 | V2.5 | 模型仿真 | CAD 到 URDF 尺寸对齐 | 根据 CAD/CATIA 尺寸更新底盘、云台、相机安装位置 | 更新后的 xacro 文件 | 模型尺寸与实物关键安装尺寸误差可接受 | B1 |
| B3 | V2.5 | 模型仿真 | Foxglove/RViz 布局 | 配置 3D、Topic、Plot、TF、Marker、Image 面板 | Foxglove/RViz 配置文件和说明 | 能同时观察目标、机器人、轨迹、控制量 | A2 |
| B4 | V2.5 | 模型仿真 | 2D 仿真场景扩展 | 增加圆形、直线、8 字、停止、突然丢失、速度变化目标 | 更新 `simple_robot_sim_node.py` 或配置 | 每个场景可通过 launch 参数复现 | A1 |
| B5 | V2.5 | 模型仿真 | 可视化 Marker 完善 | 增加目标轨迹、相机视线、期望距离、安全区、云台方向 | Marker 可视化更新 | Foxglove/RViz 可直观看出跟随误差和期望距离 | B3 |
| B6 | V3 | 模型仿真 | TRON2/备选底盘模型适配 | 如果切换 TRON2，新增底盘 xacro、TF 和 remap 方案 | `tron2_*` xacro 和说明 | 不影响上层 `/cmd_vel`、`/odom`、`base_link` 接口 | 底盘路线确定 |
| C1 | V2.5 | 感知 | 真实相机驱动接入 | 接入 USB/CSI/RealSense/OAK 等相机，发布图像和内参 | 相机 launch 和参数 | `/camera/image_raw`、`/camera/camera_info` 稳定输出 | 硬件到位 |
| C2 | V2.5 | 感知 | YOLO 推理补全 | 补全当前 DetectionNode 的模型加载和推理逻辑 | 可运行检测节点 | 输入图像后输出 `/perception/detections` | C1 |
| C3 | V2.5 | 感知 | Mock 与真实检测切换 | launch 参数支持 mock detector 和真实 detector 切换 | launch 更新 | 同一 bringup 可切换仿真/真实感知 | C2 |
| C4 | V2 | 感知 | 目标跟踪稳定 | 完善 SORT/ByteTrack 或轻量 ID 保持逻辑 | tracking 节点更新 | 多人场景中当前目标 ID 不频繁跳变 | C2 |
| C5 | V2 | 感知 | 深度估计稳定 | 深度异常值剔除、低通滤波、深度置信度输出 | depth 节点更新 | 距离突变不会直接导致底盘大幅速度突变 | C1 |
| C6 | V2 | 感知 | `/target/current` 统一出口 | 将检测、跟踪、深度融合成控制器直接使用的单目标输出 | target manager 或 remap 方案 | MVP 控制器无需关心感知内部结构 | C4, C5 |
| C7 | V3 | 感知 | 目标 3D 坐标输出 | 输出目标在 `camera_optical_link` 下的 3D 点 | `TargetArray.position` 完整 | 目标 3D 坐标可用于 PBVS | C5 |
| C8 | V3 | 感知 | 目标在 base 坐标系转换 | 使用 TF 将目标从 camera frame 转到 `base_link` | target transform 节点 | 输出 `target_in_base` 或等效字段 | C7, F2 |
| D1 | V2.5 | 平台硬件 | 底盘路线确认 | 确定使用 LeKiwi 自写驱动、LeRobot 桥接或 TRON2 官方驱动 | 技术路线记录 | 明确通信接口、话题、依赖和风险 | 无 |
| D2 | V2.5 | 平台硬件 | 底盘接口最小闭环 | 让实体底盘接收小速度命令并可停止 | 底盘 bringup 说明 | `/cmd_vel` 能让底盘低速运动，停车命令可靠 | D1 |
| D3 | V2.5 | 平台硬件 | `/odom` 与 TF 接入 | 确认或实现底盘里程计和 `odom -> base_link` TF | odom/TF 输出 | Foxglove/RViz 可看到实体底盘位姿变化 | D2 |
| D4 | V2.5 | 平台硬件 | 底盘安全保护 | 超时停车、急停、速度限幅、启动低速模式 | 安全逻辑和测试记录 | 控制节点退出或目标丢失时底盘停止 | D2 |
| D5 | V2.5 | 平台硬件 | 云台真实驱动路线 | 确定 DJI RS2 CAN、舵机云台或固定相机方案 | 云台路线记录 | 明确是否保留 `/cmd_gimbal` 闭环 | 无 |
| D6 | V2.5 | 平台硬件 | 云台控制最小闭环 | 让云台接收 yaw/pitch 速度或位置命令 | gimbal bringup 说明 | `/cmd_gimbal` 能控制云台低速运动并停止 | D5 |
| D7 | V2.5 | 平台硬件 | 云台状态反馈 | 发布云台 yaw/pitch 角和速度到 `/joint_states` 或 `/platform/state` | 状态反馈实现 | MVP 底盘 yaw 可基于真实 `gimbal_yaw` 工作 | D6 |
| D8 | V2.5 | 平台硬件 | IMU 接入或替代 | 接入 BNO055/底盘 IMU，或明确不用 IMU 的替代策略 | IMU bringup 或替代说明 | `/imu/data` 可用，或平台不依赖 IMU 也能运行 | D1 |
| D9 | V2.5 | 平台硬件 | 实机 MVP launch | 新建不启动仿真节点的实体 MVP launch | `mvp_real.launch.py` | 实机可启动感知、平台、MVP 控制和可视化 | C6, D2 |
| D10 | V3 | 平台硬件 | TRON2 驱动适配 | 如果使用 TRON2，完成 `/cmd_vel`、`/odom`、TF remap | TRON2 platform launch | 上层控制器无需改算法即可驱动 TRON2 | D1 |
| E1 | V2 | 稳定控制 | MVP 参数调优 | 系统调 `K_*`、deadband、filter、limit | 推荐参数 yaml | 仿真中无明显抖动和过冲 | B4 |
| E2 | V2 | 稳定控制 | 目标丢失处理 | 短时丢失保持/减速，长时丢失停车或搜索 | 控制逻辑更新 | 目标丢失时无危险运动 | C6 |
| E3 | V2 | 稳定控制 | 速度变化率限制 | 加入底盘和云台命令的加速度/变化率限制 | 控制器更新 | 命令曲线平滑，无速度突变 | E1 |
| E4 | V2 | 稳定控制 | 云台回中策略 | 增加小权重回中项，避免长期偏置 | 参数和控制逻辑 | 目标稳定时云台能逐步回中 | D7 |
| E5 | V2 | 稳定控制 | 固定相机模式 | 无云台时底盘直接基于图像水平误差转向 | 控制器参数或模式 | 固定相机也能跟随目标 | D5 |
| E6 | V2 | 稳定控制 | 展示参数包 | 为室内演示保存一套稳定参数 | `config/demo_*.yaml` | 一键启动即可稳定展示 | E1-E5 |
| F1 | V3 | 混合视觉伺服 | IBVS 云台节点拆分 | 将云台 IBVS 从 MVP/ServoManager 中拆成独立节点或清晰模块 | `ibvs_gimbal_controller` | 输入图像误差，输出云台命令 | C6 |
| F2 | V3 | 混合视觉伺服 | PBVS 底盘节点拆分 | 基于目标在 `base_link` 下的位置计算底盘速度 | `pbvs_base_controller` | 输入目标 3D 位置，输出底盘速度 | C8 |
| F3 | V3 | 混合视觉伺服 | 控制分配节点 | 统一处理云台优先、底盘慢跟、云台极限提前介入 | `control_allocation_node` | 云台接近极限时底盘参与变强 | F1, F2 |
| F4 | V3 | 混合视觉伺服 | 控制模式切换 | 支持 MVP、IBVS、PBVS、HYBRID 模式切换 | service 或参数接口 | 不重启系统可切换控制模式 | F1-F3 |
| F5 | V3 | 混合视觉伺服 | V3 仿真对比 | 对比 V2 规则控制和 V3 混合视觉伺服 | 实验记录 | 给出误差、平滑性、丢失恢复对比 | F4 |
| F6 | V3 | 混合视觉伺服 | 架构文档 | 更新 V3 节点图、topic 图、参数表 | `docs/v3_architecture.md` | 后续成员能按文档理解控制链路 | F1-F4 |
| G1 | V4 | 优化控制 | QP 控制分配数学定义 | 定义状态、控制量、代价函数、约束和权重 | `docs/qp_formulation.md` | 公式能对应到实际控制量 | F3 |
| G2 | V4 | 优化控制 | QP 原型仿真 | 用 Python 或 C++ 实现单步优化分配原型 | 原型节点或脚本 | 仿真中输出满足速度和云台约束 | G1 |
| G3 | V4 | 优化控制 | QP 集成到 ROS2 | 将 QP 原型接入 `/target/current`、`/platform/state` | `qp_allocation_node` | 可替换 V3 allocation 输出控制命令 | G2 |
| G4 | V4 | 优化控制 | MPC 简化模型 | 建立预测状态、输入、约束和时域长度 | `docs/mpc_model.md` | 模型能解释图像误差、距离误差、云台角变化 | G1 |
| G5 | V4 | 优化控制 | MPC 最小原型 | 实现短预测时域 MPC 或离线仿真 | MPC 原型 | 能在仿真中比 V2/V3 更平滑或约束更好 | G4 |
| G6 | V4 | 优化控制 | 控制器对比实验 | 对比 V2、V3、QP/MPC 的误差、速度、平滑度 | 实验报告 | 有图表、rosbag、参数和结论 | E6, F5, G3 |
| H1 | V2.5 | 测试验收 | Launch 测试清单 | 为每个 launch 写启动命令、预期 topic、常见错误 | `docs/launch_test_checklist.md` | 新成员可逐项检查系统健康 | A1 |
| H2 | V2.5 | 测试验收 | rosbag 数据采集规范 | 定义必须记录的话题和命名规则 | `docs/rosbag_recording.md` | 典型场景 bag 可复现实验 | A2 |
| H3 | V2.5 | 测试验收 | 仿真测试数据集 | 录制不同目标轨迹的 bag 和参数 | bag 文件和索引 | 可离线回放验证控制器 | B4 |
| H4 | V2 | 测试验收 | 实机安全测试 | 低速、架空、急停、目标丢失、节点退出测试 | 测试记录 | 所有危险场景均能安全停止 | D4 |
| H5 | V3 | 测试验收 | 控制性能指标 | 定义居中误差、距离误差、速度平滑度、目标恢复时间 | `docs/metrics.md` | 每次实验可量化评价 | H2 |
| H6 | V4 | 测试验收 | 最终演示脚本 | 开机、启动、语音、跟随、异常恢复、关闭流程 | `docs/final_demo_script.md` | 可按脚本稳定完成最终展示 | E6, I8 |
| I1 | V2.5 | 语音交互 | 语音需求表 | 定义开始、停止、急停、切换目标、距离调整、云台回中等口令 | `docs/voice_command_requirements.md` | 每个口令有意图、同义说法、ROS2 动作 | A2 |
| I2 | V2.5 | 语音交互 | `voice_control_pkg` 脚手架 | 新建 ROS2 Python 包，包含 config、launch、节点目录 | 包结构 | `colcon build` 通过，可启动空节点 | I1 |
| I3 | V2.5 | 语音交互 | ASR 节点 | 麦克风或文本输入转 `/voice/text` | `asr_node.py` | 能在终端或 topic 中看到识别文字 | I2 |
| I4 | V2.5 | 语音交互 | 模糊语义解析 | 关键词、同义词、编辑距离、置信度阈值 | `intent_parser_node.py` 和 yaml | “开始跟拍/跟着我/启动追踪”归一成同一 intent | I3 |
| I5 | V2.5 | 语音交互 | Intent 输出接口 | 定义 `/voice/intent` 格式，包含原文、意图、置信度 | 消息方案或 std_msgs 约定 | 下游可稳定消费 intent | I4 |
| I6 | V2.5 | 语音交互 | 语音执行节点 | 将 intent 转成服务调用、参数修改或安全命令 | `voice_executor_node.py` | 支持开始跟随、停止、急停、云台回中 | I5 |
| I7 | V2.5 | 语音交互 | 语音安全策略 | 急停最高优先级，低置信度不执行，危险操作二次确认 | 安全策略文档和逻辑 | 错识别不会导致危险运动 | I6 |
| I8 | V2.5 | 语音交互 | 语音演示流程 | 编写语音控制演示脚本和测试用例 | `docs/voice_demo.md` | 能展示开始/停止/急停/回中/远一点 | I7 |
| I9 | V3 | 语音交互 | 语音目标选择 | 将“跟左边的人/换目标”接入 target manager | 目标选择接口 | 可以通过语音切换或锁定目标 | C6, I6 |
| I10 | V3 | 语音交互 | 语音状态查询 | 支持“现在状态/有没有跟到/目标丢了吗” | 状态查询输出 | 能返回当前模式、目标状态、距离 | I6, A2 |
| J1 | V2.5 | 文档汇报 | 中期展示材料 | 整理当前功能、架构、仿真效果、待办 | PPT/Markdown | 适合组会或中期检查 | A1-A4 |
| J2 | V3 | 文档汇报 | 算法说明文档 | 说明 V1/V2/V3 控制律和升级原因 | `docs/control_algorithm.md` | 公式、变量、参数和数据流一致 | E6, F6 |
| J3 | V4 | 文档汇报 | 创新点总结 | 总结 QP/MPC、语音交互、协同控制和实验对比 | 最终报告材料 | 可用于答辩/比赛/论文撰写 | G6, I8 |

## 优先级建议

| 优先级 | 任务范围 | 原因 |
|---|---|---|
| P0 | A1-A4, B3-B5, D1-D4, C1-C3, E1-E2, H1 | 先保证当前版本可运行、可调试、可安全接硬件 |
| P1 | C4-C6, D5-D9, E3-E6, I1-I8, H2-H4 | 形成稳定实机展示版和语音交互增强 |
| P2 | F1-F6, I9-I10, H5, J2 | 完成 V3 混合视觉伺服结构化升级 |
| P3 | G1-G6, H6, J3 | 完成 V4 优化控制和最终展示亮点 |

## 推荐近期里程碑

| 里程碑 | 目标 | 必须完成任务 |
|---|---|---|
| M1 当前版本可交接 | 新成员能独立跑仿真、看懂 topic、记录问题 | A1, A2, A3, H1 |
| M2 展示级仿真 | 2D/Foxglove 可视化清楚，目标持续移动，控制稳定 | B3, B4, B5, E1 |
| M3 实机底盘闭环 | 实体底盘低速可控，安全停车，`/odom` 可用 | D1, D2, D3, D4 |
| M4 感知闭环 | 真实或半真实感知输出 `/target/current` | C1, C2, C3, C6 |
| M5 V2.5 展示版 | 跟随稳定，语音可控制开始/停止/急停/回中 | E2-E6, I1-I8 |
| M6 V3 算法版 | IBVS/PBVS/控制分配拆分，可和 V2 对比 | F1-F6, H5 |
| M7 V4 创新版 | QP/MPC 原型和实验报告完成 | G1-G6, J3 |

## 语音控制第一版意图集合

| Intent | 示例口令 | 期望动作 |
|---|---|---|
| `start_follow` | 开始跟随、开始跟拍、跟着我、启动追踪 | 进入跟随模式 |
| `stop_follow` | 停止跟随、别跟了、暂停跟拍 | 停止跟随，底盘和云台速度归零 |
| `emergency_stop` | 急停、停车、马上停、别动 | 最高优先级安全停止 |
| `gimbal_home` | 云台回中、镜头回中、看正前方 | 云台回到中位或发回中命令 |
| `distance_closer` | 近一点、靠近一点 | 减小 `desired_distance` |
| `distance_farther` | 远一点、离我远点 | 增大 `desired_distance` |
| `target_switch` | 换目标、跟左边的人、跟这个人 | 调用目标选择或锁定逻辑 |
| `status_query` | 现在状态、有没有跟到、目标丢了吗 | 输出当前模式、目标状态和距离 |

## 验收记录模板

| 日期 | 任务 ID | 测试环境 | 启动命令 | 结果 | 问题 | 下一步 |
|---|---|---|---|---|---|---|
|  |  |  |  |  |  |  |
