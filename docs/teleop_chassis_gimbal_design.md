# 底盘遥控与云台遥控技术设计

## 1. 功能定位

底盘遥控和云台遥控是系统的人工控制入口，主要用于：

- 实机调试
- 自动跟随前的硬件验证
- 拍摄过程中的人工接管
- 安全测试和急停验证
- 后续 MPC、Action 运镜、视觉伺服的基础调试

这部分不是最终的智能算法核心，但它决定了系统是否可控、可调、可救场。

## 2. 主要涉及技术

| 功能 | 涉及技术 |
|---|---|
| 底盘遥控 | ROS2 `cmd_vel`、`geometry_msgs/Twist`、键盘/手柄输入、速度档位、速度限幅、加速度限幅、急停 |
| 云台遥控 | 自定义 `GimbalCmd`、yaw/pitch 角速度控制、云台回中、锁定/解锁、死区、速度平滑、角度限位 |
| 输入设备 | Keyboard teleop、Joystick、`sensor_msgs/Joy`、Web UI 或 Foxglove 控制面板 |
| 安全保护 | deadman button、command timeout、急停优先级、零速度保护 |
| 控制仲裁 | mode manager、command mux、teleop/auto/action 控制权切换 |

## 3. 总体设计链路

推荐不要让遥控节点直接控制硬件，而是经过模式仲裁和安全过滤：

```text
键盘 / 手柄 / Web UI
        ↓
teleop_node
        ↓
/teleop/cmd_vel_raw
/teleop/gimbal_cmd_raw
        ↓
mode_manager / command_mux
        ↓
safety_filter
        ↓
/tron2/cmd_vel
/rs2/cmd_gimbal
        ↓
TRON2 底盘 / DJI RS2 云台
```

这样设计的好处是：

- 遥控、自动跟随、MPC、Action 运镜都走同一套安全出口
- 急停和限速逻辑可以集中管理
- 后续切换模式时不会出现多个控制源抢硬件
- 实机调试时更容易定位问题

## 4. 底盘遥控设计

### 4.1 推荐 Topic

```text
/teleop/cmd_vel_raw     # 遥控输入的原始底盘速度
/safety/cmd_vel_safe    # 安全过滤后的速度
/tron2/cmd_vel          # 发送给 TRON2 driver 的最终速度
```

### 4.2 输入映射

| 输入 | 输出 |
|---|---|
| 左摇杆上下 | `linear.x` 前进 / 后退 |
| 左摇杆左右 | `linear.y` 横移，如果 TRON2 支持全向移动 |
| 右摇杆左右 | `angular.z` 原地转向 |
| 按键 A/B/X | 慢速 / 普通 / 快速 |
| 肩键或扳机 | deadman，按住才允许运动 |
| 急停键 | 立即发布零速度并进入 `SAFE_STOP` |

### 4.3 关键参数

```yaml
teleop_chassis:
  slow_scale: 0.3
  normal_scale: 0.6
  fast_scale: 1.0
  v_x_max: 0.8
  v_y_max: 0.6
  w_z_max: 1.0
  accel_limit: 0.5
  angular_accel_limit: 0.8
  cmd_timeout: 0.3
  deadman_required: true
```

### 4.4 控制重点

底盘遥控要重点保证：

- 松手必须停车
- 通信断开必须停车
- 速度不能突然跳变
- 急停优先级最高
- 自动模式下遥控命令不能随便插入
- 进入 `TELEOP` 模式后才允许人工速度命令生效

## 5. 云台遥控设计

### 5.1 推荐 Topic / Service

```text
/teleop/gimbal_cmd_raw      # 遥控输入的原始云台命令
/safety/gimbal_cmd_safe     # 安全过滤后的云台命令
/rs2/cmd_gimbal             # 发送给 DJI RS2 driver 的最终命令
/rs2/gimbal_state           # 云台 yaw/pitch 状态反馈
/rs2/home                   # 云台回中服务，可选
```

### 5.2 推荐命令格式

```text
GimbalCmd:
  yaw_rate
  pitch_rate
  hold_yaw
  hold_pitch
```

### 5.3 输入映射

| 输入 | 输出 |
|---|---|
| 右摇杆左右 | 云台 yaw 左 / 右 |
| 右摇杆上下 | 云台 pitch 上 / 下 |
| 一个按键 | 云台回中 |
| 一个按键 | 云台锁定 / 解锁 |
| 速度档位 | 慢速拍摄 / 快速调试 |
| 急停键 | 云台速度归零并进入保护状态 |

### 5.4 关键参数

```yaml
teleop_gimbal:
  yaw_rate_max_slow: 0.3
  yaw_rate_max_normal: 0.8
  yaw_rate_max_fast: 1.5
  pitch_rate_max_slow: 0.2
  pitch_rate_max_normal: 0.6
  pitch_rate_max_fast: 1.0
  deadband: 0.08
  smooth_alpha: 0.4
  yaw_limit_min: -1.57
  yaw_limit_max: 1.57
  pitch_limit_min: -0.7
  pitch_limit_max: 0.5
  cmd_timeout: 0.3
```

### 5.5 控制重点

云台遥控要重点保证：

- 小摇杆输入不会造成画面抖动
- 云台不会突然高速甩动
- pitch/yaw 不超过安全角度
- 松手后进入 hold 或零速度
- 云台回中动作要平滑
- 自动跟随和手动云台控制不能同时抢控制权

## 6. 模式仲裁设计

建议系统中引入统一的 `mode_manager_node` 或 `command_mux_node`。

模式可以包括：

```text
STANDBY
TELEOP
AUTO_FOLLOW
ACTION_SHOT
TARGET_LOST
SAFE_STOP
```

控制权规则建议如下：

| 当前模式 | 底盘遥控 | 云台遥控 | 自动跟随 | Action 运镜 |
|---|---|---|---|---|
| `STANDBY` | 不生效 | 可允许低速回中 | 不生效 | 不生效 |
| `TELEOP` | 生效 | 生效 | 不生效 | 不生效 |
| `AUTO_FOLLOW` | 不生效或仅允许接管 | 可选人工微调 | 生效 | 不生效 |
| `ACTION_SHOT` | 不生效或仅允许取消 | 不生效或仅允许取消 | 由 Action 控制 | 生效 |
| `TARGET_LOST` | 可接管 | 可接管 | 暂停 | 暂停 |
| `SAFE_STOP` | 不生效 | 不生效 | 不生效 | 不生效 |

## 7. 推荐实现顺序

1. 实现键盘遥控，验证 `/cmd_vel` 和 `/cmd_gimbal` 是否能正常发布。
2. 接入手柄遥控，加入速度档位和 deadman。
3. 增加 `safety_filter`，对底盘和云台命令做限速、超时停车、急停处理。
4. 增加 `mode_manager` 或 `command_mux`，统一控制遥控、自动跟随、Action 运镜的优先级。
5. 接入真实 TRON2 和 DJI RS2 driver，完成低速实机测试。
6. 最后接 Web UI 或 Foxglove 控制面板，方便演示和调参。

## 8. 总结

底盘遥控解决“机器人怎么移动”，云台遥控解决“镜头怎么转动”。

这两个模块共同构成系统的人工接管和硬件调试入口。设计时最重要的原则是：遥控命令不要直接绕过安全层打到硬件，必须经过模式仲裁和安全过滤。
