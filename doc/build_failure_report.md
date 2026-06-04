# FCR ROS2 工作空间构建失败问题分析与修复报告

> **项目**: fcr_ros2_3 | **ROS2 版本**: Humble | **编译器**: GCC 11.4.0 | **C++ 标准**: C++17
>
> **最终状态**: 6/6 包构建成功 ✅

## 概览

首次执行 `bash ./build.sh` 时遇到 **11 个构建错误**，涉及 5 个 ROS2 包。以下按依赖顺序（构建顺序）逐一定位根因、分析影响并提供修复方案。

---

## 错误 1 — simulation_pkg: 缺少 config 目录

**错误信息**:
```
CMake Error: ament_cmake_symlink_install_directory() can't find
  '/home/angcui/fcr_ros2/fcr_ros2_3/src/simulation_pkg/config'
```

**根因**: CMakeLists.txt 第 18 行 `install(DIRECTORY urdf worlds config launch ...)` 引用了一个不存在的 `config` 目录。该包实际仅有 `urdf`、`worlds`、`launch` 三个资源目录。

**修复**: 从 `install(DIRECTORY ...)` 指令中移除 `config`。

**文件**: `src/simulation_pkg/CMakeLists.txt`

---

## 错误 2 — vision_servo_msgs: 缺少 find_package 依赖声明

**错误信息**:
```
rosidl_generate_interfaces() the passed dependency 'std_msgs' has not been
found before using find_package()
```

**根因**: `rosidl_generate_interfaces(... DEPENDENCIES std_msgs geometry_msgs ...)` 要求这些依赖包**必须**在调用前通过 `find_package()` 声明。CMakeLists.txt 仅 find 了 `ament_cmake` 和 `rosidl_default_generators`，遗漏了 5 个接口依赖包。

**修复**: 添加 5 个 `find_package()` 调用：

```cmake
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
```

**文件**: `src/vision_servo_msgs/CMakeLists.txt`

---

## 错误 3 — perception_pkg: 独立可执行文件缺少 rclcpp_components 构建依赖

**错误信息**:
```
fatal error: rclcpp_components/register_node_macro.hpp: 没有那个文件或目录
```

**根因**: `detection_node`、`tracking_node`、`depth_estimator_node` 三个独立可执行文件的源文件 `#include <rclcpp_components/register_node_macro.hpp>` 并调用 `RCLCPP_COMPONENTS_REGISTER_NODE` 宏，但在 CMakeLists.txt 中，这三个目标的 `ament_target_dependencies()` 声明了 `rclcpp` 但**遗漏**了 `rclcpp_components`。虽然顶层 `find_package(rclcpp_components REQUIRED)` 已执行，但 colcon 根据 `ament_target_dependencies` 逐目标注入 include 路径——缺少声明即找不到头文件。

**修复**: 为三个目标的 `ament_target_dependencies()` 添加 `rclcpp_components`。

**文件**: `src/perception_pkg/CMakeLists.txt`

---

## 错误 4 — perception_pkg: 缺少 OpenCV 头文件 include

**错误信息**:
```
error: 'cv' does not name a type
error: 'cv::KalmanFilter' has not been declared
```

**根因**:
- `tracking_node.hpp` 第 43 行声明了 `cv::KalmanFilter kf` 成员，但头文件未 `#include <opencv2/video/tracking.hpp>`
- `depth_estimator.hpp` 第 46 行使用了 `const cv::Mat&` 参数类型，但头文件未 `#include <opencv2/core/mat.hpp>`

**修复**: 为两个头文件补充缺失的 OpenCV include。

**文件**:
- `src/perception_pkg/include/perception_pkg/tracking_node.hpp`
- `src/perception_pkg/include/perception_pkg/depth_estimator.hpp`

---

## 错误 5 — perception_pkg: 缺少 vision_servo_msgs 服务头文件

**错误信息**:
```
error: 'srv' is not a member of 'vision_servo_msgs'
error: 'tracking_srv_' does not name a type
```

**根因**: `tracking_node.hpp` 第 91 行声明了类型为 `rclcpp::Service<vision_servo_msgs::srv::SetTrackingTarget>::SharedPtr` 的成员 `tracking_srv_`，但头文件只 include 了 `<vision_servo_msgs/msg/target_array.hpp>`（消息），未 include 对应的服务定义头文件 `<vision_servo_msgs/srv/set_tracking_target.hpp>`。

**修复**: 添加服务头文件 include。

**文件**: `src/perception_pkg/include/perception_pkg/tracking_node.hpp`

---

## 错误 6 — perception_pkg: create_service 回调签名类型推导失败

**错误信息**:
```
error: no match for 'operator=' ... variant ... lambda ...
```

**根因**: 在 `tracking_node.cpp` 和 `perception_pipeline.cpp` 中，`create_service<vision_servo_msgs::srv::SetTrackingTarget>()` 的回调使用了**泛型 lambda** 参数：
```cpp
[this](const auto& req, auto& resp) { ... }
```

ROS2 Humble 的 `rclcpp::create_service()` 内部通过 `std::variant` 和模板元编程判定回调签名是否与多个可能的回调原型之一匹配。`auto` 泛型参数导致函数签名模糊，编译器（GCC 11.4）无法将其匹配到 `variant` 中任何一个确定的 `std::function` 类型。

**修复**: 将 lambda 参数从 `auto` 替换为精确的类型：
```cpp
[this](
  const std::shared_ptr<vision_servo_msgs::srv::SetTrackingTarget::Request> req,
  std::shared_ptr<vision_servo_msgs::srv::SetTrackingTarget::Response> resp) { ... }
```

**文件**:
- `src/perception_pkg/src/tracking_node.cpp`
- `src/perception_pkg/src/perception_pipeline.cpp`

---

## 错误 7 — perception_pkg: `std::array` 与 C 数组类型不匹配

**错误信息**:
```
error: cannot convert 'std::array<float, 4>' to 'const float*'
```

**根因**: `vision_servo_msgs::msg::Target` 的 `bbox` 字段在 ROS2 IDL 代码生成后是 `std::array<float, 4>` 类型。`depth_estimator_node.cpp` 调用 `estimate_depth(depth_frame, target.bbox)` 时，函数声明 `estimate_depth(const cv::Mat&, const float bbox[4])` 期望 C 风格数组指针，而 `std::array` 不会隐式退化为原始指针。

**修复**: 将函数签名从 `const float bbox[4]` 改为 `const std::array<float, 4>& bbox`，并在声明和定义两侧同步修改。

**文件**:
- `src/perception_pkg/include/perception_pkg/depth_estimator.hpp`
- `src/perception_pkg/src/depth_estimator_node.cpp`

---

## 错误 8 — perception_pkg: 独立可执行文件缺少 main() 入口

**错误信息**:
```
undefined reference to `main'
collect2: error: ld returned 1 exit status
```

**根因**: `RCLCPP_COMPONENTS_REGISTER_NODE` 宏仅将节点类注册为可通过 `ComponentManager` 动态加载的插件，**不会**生成 `main()` 函数。四个 `.cpp` 文件 (`detection_node.cpp`、`tracking_node.cpp`、`depth_estimator_node.cpp`、`perception_pipeline.cpp`) 在 CMakeLists.txt 中通过 `add_executable()` 声明为独立可执行文件，但因缺少 `main()`，链接器找不到程序入口点。

**修复**: 为三个独立可执行文件添加标准 ROS2 main 函数：
```cpp
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::XXXNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

> `perception_pipeline` 通过 `rclcpp_components_register_node()` 宏处理（该宏额外生成 main），故无需手动添加。

**文件**:
- `src/perception_pkg/src/detection_node.cpp`
- `src/perception_pkg/src/tracking_node.cpp`
- `src/perception_pkg/src/depth_estimator_node.cpp`

---

## 错误 9 — servo_control_pkg: 控制器插件缺少默认构造函数

**错误信息**:
```
error: no matching function for call to 'servo_control_pkg::IBVSController::IBVSController()'
```

**根因**: `pluginlib::ClassLoader` 通过 `new C`（无参数构造）创建插件实例。`IBVSController` 和 `PBVSController` 仅声明了带 `const rclcpp::NodeOptions&` 参数的显式构造函数，编译器不会自动生成默认构造函数。

**修复**: 为两个控制器类添加默认构造函数，在初始化列表中调用基类构造函数并提供合理的默认节点名称和空 `NodeOptions`：
```cpp
IBVSController() : ServoControllerBase("ibvs_controller", rclcpp::NodeOptions()) {}
```

**文件**:
- `src/servo_control_pkg/include/servo_control_pkg/ibvs_controller.hpp`
- `src/servo_control_pkg/include/servo_control_pkg/pbvs_controller.hpp`

---

## 错误 10 — servo_control_pkg: 缺少头文件 include

### 10a — velocity_commander_node: 缺少 Vector3Stamped 头文件

**错误信息**:
```
error: 'Vector3Stamped' in namespace 'geometry_msgs::msg' does not name a type
```

**根因**: `velocity_commander_node.cpp` 使用了 `geometry_msgs::msg::Vector3Stamped` 类型创建发布者和订阅者，但 include 列表中仅有 `<geometry_msgs/msg/twist_stamped.hpp>`，缺少 `<geometry_msgs/msg/vector3_stamped.hpp>`。

**修复**: 添加缺失的 include。

**文件**: `src/servo_control_pkg/src/velocity_commander_node.cpp`

### 10b — servo_manager_node: 缺少 rclcpp_action 头文件

**错误信息**:
```
error: 'rclcpp_action' has not been declared
```

**根因**: `servo_manager_node.cpp` 使用了 `rclcpp_action::Server`、`rclcpp_action::GoalResponse` 等类型来实现 VisualServo 动作服务器，但 include 列表中遗漏了 `<rclcpp_action/rclcpp_action.hpp>`。

**修复**: 添加 `#include <rclcpp_action/rclcpp_action.hpp>`。

**文件**: `src/servo_control_pkg/src/servo_manager_node.cpp`

---

## 错误 11 — robot_platform_pkg: 工厂函数未实现 + 缺失头文件

### 11a — 硬件接口工厂函数找不到定义

**错误信息**:
```
undefined reference to `robot_platform_pkg::make_simulated_imu()'
undefined reference to `robot_platform_pkg::make_bno055_imu()'
undefined reference to `robot_platform_pkg::make_simulated_chassis()'
undefined reference to `robot_platform_pkg::make_lekiwi_chassis()'
undefined reference to `robot_platform_pkg::make_simulated_gimbal()'
undefined reference to `robot_platform_pkg::make_dji_rs2_gimbal()'
undefined reference to `robot_platform_pkg::make_omni_wheel_odometry()'
```

**根因**: 四个硬件抽象接口头文件中声明了工厂函数（`make_simulated_*` / `make_*`），但对应的 `.cpp` 实现文件完全缺失。这些函数被各 driver node 的构造函数调用，链接时找不到符号定义。

**修复**: 创建四个接口实现文件，每个提供两类工厂函数：
- **真实硬件工厂**：返回 `nullptr`（TODO 占位）
- **仿真工厂**：返回具体实现类的 `unique_ptr`

新建文件:
| 文件 | 提供的函数 |
|------|-----------|
| `src/robot_platform_pkg/src/chassis_interface_impl.cpp` | `make_lekiwi_chassis()`, `make_simulated_chassis()` |
| `src/robot_platform_pkg/src/gimbal_interface_impl.cpp` | `make_dji_rs2_gimbal()`, `make_simulated_gimbal()` |
| `src/robot_platform_pkg/src/imu_interface_impl.cpp` | `make_bno055_imu()`, `make_simulated_imu()` |
| `src/robot_platform_pkg/src/odometry_interface_impl.cpp` | `make_omni_wheel_odometry()` |

并修改 CMakeLists.txt 将各 impl 文件添加到对应 executable 的 `add_executable()` 源文件列表中。

### 11b — odometry_interface.hpp 缺少 sensor_msgs include

**错误信息**:
```
error: 'sensor_msgs' does not name a type
```

**根因**: `odometry_interface.hpp` 第 31 行声明了 `const sensor_msgs::msg::Imu& imu` 参数，但头文件 include 列表中只有 `<nav_msgs/msg/odometry.hpp>` 和 `<geometry_msgs/msg/twist.hpp>`，缺少 `<sensor_msgs/msg/imu.hpp>`。

**修复**: 添加 `#include <sensor_msgs/msg/imu.hpp>`。

**文件**: `src/robot_platform_pkg/include/robot_platform_pkg/hardware_interfaces/odometry_interface.hpp`

---

## 修复文件总览

| 文件 | 修改类型 |
|------|---------|
| `src/simulation_pkg/CMakeLists.txt` | 移除不存在的 config 目录引用 |
| `src/vision_servo_msgs/CMakeLists.txt` | 添加 5 个 find_package 调用 |
| `src/perception_pkg/CMakeLists.txt` | 为 3 个目标添加 rclcpp_components 依赖 |
| `src/perception_pkg/include/perception_pkg/tracking_node.hpp` | 添加 OpenCV + vision_servo_msgs srv 头文件 |
| `src/perception_pkg/include/perception_pkg/depth_estimator.hpp` | 添加 OpenCV 头文件，修正函数签名为 std::array |
| `src/perception_pkg/src/detection_node.cpp` | 添加 main() |
| `src/perception_pkg/src/tracking_node.cpp` | 添加 main()，修正服务回调签名，修正成员名 age_→age |
| `src/perception_pkg/src/depth_estimator_node.cpp` | 添加 main()，修正函数签名为 std::array |
| `src/perception_pkg/src/perception_pipeline.cpp` | 修正服务回调签名 |
| `src/servo_control_pkg/include/servo_control_pkg/ibvs_controller.hpp` | 添加默认构造函数 |
| `src/servo_control_pkg/include/servo_control_pkg/pbvs_controller.hpp` | 添加默认构造函数 |
| `src/servo_control_pkg/src/velocity_commander_node.cpp` | 添加 vector3_stamped 头文件 |
| `src/servo_control_pkg/src/servo_manager_node.cpp` | 添加 rclcpp_action 头文件 |
| `src/robot_platform_pkg/CMakeLists.txt` | 为 4 个目标添加 impl 源文件 |
| `src/robot_platform_pkg/include/robot_platform_pkg/hardware_interfaces/odometry_interface.hpp` | 添加 sensor_msgs 头文件 |
| `src/robot_platform_pkg/src/chassis_interface_impl.cpp` | 新建 — 底盘工厂函数实现 |
| `src/robot_platform_pkg/src/gimbal_interface_impl.cpp` | 新建 — 云台工厂函数实现 |
| `src/robot_platform_pkg/src/imu_interface_impl.cpp` | 新建 — IMU 工厂函数实现 |
| `src/robot_platform_pkg/src/odometry_interface_impl.cpp` | 新建 — 里程计工厂函数实现 |

## 经验教训

1. **`rosidl_generate_interfaces` 的 DEPENDENCIES 必须事先 find_package** — 这是 ROS2 构建系统的硬性要求，容易被遗漏。
2. **`install(DIRECTORY ...)` 引用的目录必须存在** — colcon 在安装阶段会检查，即使配置阶段通过。
3. **`RCLCPP_COMPONENTS_REGISTER_NODE` 不生成 main()** — 可组合节点如果同时作为独立可执行文件使用，需手动添加 main 入口。
4. **`create_service` 回调避免使用 `auto` 泛型参数** — ROS2 Humble 的 `AnyServiceCallback` 通过模板元编程匹配回调签名，`auto` 类型推导导致匹配失败。
5. **`pluginlib::ClassLoader` 要求默认构造函数** — 通过 `new C` 创建插件实例，所有插件类（如 PBVS/IBVS 控制器）必须有公开的默认构造函数。
6. **硬件抽象接口的工厂函数应及时实现** — 即使真实硬件驱动可标记为 TODO，仿真实现也必须提供，否则链接阶段失败。
