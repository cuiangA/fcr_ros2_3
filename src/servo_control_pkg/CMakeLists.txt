# ── servo_control_pkg CMakeLists.txt ────────────────────────────────
# 伺服控制包：基于 pluginlib 的可插拔视觉伺服控制器。
#
# 构建目标：
#   1. servo_qos (INTERFACE 库)           — 统一 QoS 配置
#   2. servo_controller_base (共享库)      — 控制器基类（交互矩阵、特征提取）
#   3. ibvs_controller (共享库)            — 基于图像的视觉伺服（pluginlib 插件）
#   4. pbvs_controller (共享库)            — 基于位置的视觉伺服（pluginlib 插件）
#   5. control_allocator (共享库)          — 控制分配器（相机速度 → 底盘 + 云台）
#   6. servo_manager_node (可执行文件)      — 主节点：加载插件 + 运行控制回路
#   7. velocity_commander_node (可执行文件) — 速度指令发布器
#
# 控制器通过 pluginlib 动态加载，支持运行时热切换。
# 新增控制器只需继承 ServoControllerBase 并注册到 plugins.xml。
# ────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.8)
project(servo_control_pkg)

# C++17：pluginlib + Eigen 需要
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖声明 ──────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(vision_servo_msgs REQUIRED)   # TargetArray, ServoState, PlatformState
find_package(geometry_msgs REQUIRED)        # Twist, Vector3
find_package(sensor_msgs REQUIRED)          # CameraInfo
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)               # 矩阵/线性代数（交互矩阵、伪逆）
find_package(pluginlib REQUIRED)            # 动态插件加载（运行时切换控制器）

# ── QoS 工具库（INTERFACE，仅头文件） ──────────────────────────────
add_library(servo_qos INTERFACE)
target_include_directories(servo_qos INTERFACE include)
target_link_libraries(servo_qos INTERFACE rclcpp::rclcpp)

# ── 控制器基类（共享库，所有控制器的公共基础） ────────────────────
add_library(servo_controller_base SHARED
  src/servo_controller_base.cpp
)
target_include_directories(servo_controller_base PUBLIC include)
ament_target_dependencies(servo_controller_base
  rclcpp vision_servo_msgs geometry_msgs Eigen3 pluginlib
)
target_link_libraries(servo_controller_base servo_qos)
install(TARGETS servo_controller_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# ── IBVS 控制器（共享库，通过 pluginlib 加载） ────────────────────
add_library(ibvs_controller SHARED
  src/ibvs_controller.cpp
)
target_include_directories(ibvs_controller PRIVATE include)
ament_target_dependencies(ibvs_controller
  rclcpp vision_servo_msgs geometry_msgs Eigen3 pluginlib
)
target_link_libraries(ibvs_controller servo_controller_base servo_qos)
install(TARGETS ibvs_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# ── PBVS 控制器（共享库，通过 pluginlib 加载） ────────────────────
add_library(pbvs_controller SHARED
  src/pbvs_controller.cpp
)
target_include_directories(pbvs_controller PRIVATE include)
ament_target_dependencies(pbvs_controller
  rclcpp vision_servo_msgs geometry_msgs Eigen3 pluginlib
)
target_link_libraries(pbvs_controller servo_controller_base servo_qos)
install(TARGETS pbvs_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# ── 控制分配器（共享库，相机速度 → 底盘 + 云台指令） ──────────────
add_library(control_allocator SHARED
  src/control_allocator.cpp
)
target_include_directories(control_allocator PUBLIC include)
ament_target_dependencies(control_allocator
  rclcpp vision_servo_msgs geometry_msgs Eigen3
)
target_link_libraries(control_allocator servo_qos)
install(TARGETS control_allocator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# ── 伺服管理节点（主可执行文件） ──────────────────────────────────
add_executable(servo_manager_node src/servo_manager_node.cpp)
target_include_directories(servo_manager_node PRIVATE include)
ament_target_dependencies(servo_manager_node
  rclcpp rclcpp_components vision_servo_msgs geometry_msgs
  tf2_ros tf2_geometry_msgs pluginlib
)
target_link_libraries(servo_manager_node
  servo_controller_base
  control_allocator
  servo_qos
)
install(TARGETS servo_manager_node DESTINATION lib/${PROJECT_NAME})

# ── 速度指令节点 ──────────────────────────────────────────────────
add_executable(velocity_commander_node src/velocity_commander_node.cpp)
target_include_directories(velocity_commander_node PRIVATE include)
ament_target_dependencies(velocity_commander_node
  rclcpp vision_servo_msgs geometry_msgs
)
target_link_libraries(velocity_commander_node servo_qos)
install(TARGETS velocity_commander_node DESTINATION lib/${PROJECT_NAME})

# ── Plugin XML（pluginlib 发现的必要文件） ─────────────────────────
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(servo_control_pkg plugins.xml)

# ── 头文件 ────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)

# ── 启动文件和配置 ────────────────────────────────────────────────
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
