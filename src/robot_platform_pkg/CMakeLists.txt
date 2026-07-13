# ── robot_platform_pkg CMakeLists.txt ──────────────────────────────
# 机器人平台包：LEKIWI 底盘 + DJI RS2 云台 + BNO055 IMU + 里程计
#
# 构建目标：
#   1. platform_qos (INTERFACE 库)           — 统一 QoS 配置
#   2. three_wheel_kinematics (INTERFACE 库)  — 三全向轮运动学
#   3. chassis_driver_node                    — 底盘驱动（串口/仿真，发布底盘原始反馈）
#   4. gimbal_driver_node                    — 云台驱动（CAN/仿真）
#   5. imu_driver_node                       — IMU 驱动（I2C/仿真）
#   6. odometry_node                         — 里程计融合（轮速+IMU）
#   7. platform_manager_node                 — 平台状态聚合
#
# 所有节点通过硬件抽象接口 (I*Interface) 支持真实/仿真双模式。
# ────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.8)
project(robot_platform_pkg)

# C++17：Eigen + rosidl 生成代码需要
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖声明 ──────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(vision_servo_msgs REQUIRED)   # GimbalCmd, PlatformState
find_package(geometry_msgs REQUIRED)        # Twist, Pose, Transform
find_package(sensor_msgs REQUIRED)          # Imu, Image
find_package(std_srvs REQUIRED)             # Trigger debug services
find_package(nav_msgs REQUIRED)             # Odometry
find_package(tf2_ros REQUIRED)              # TF 广播
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)               # 矩阵运算（运动学、里程计）

# ── QoS 工具库（INTERFACE，仅头文件） ──────────────────────────────
add_library(platform_qos INTERFACE)
target_include_directories(platform_qos INTERFACE include)
target_link_libraries(platform_qos INTERFACE rclcpp::rclcpp)

# ── 运动学库（INTERFACE，仅头文件） ────────────────────────────────
add_library(three_wheel_kinematics INTERFACE)
target_include_directories(three_wheel_kinematics INTERFACE include)
target_link_libraries(three_wheel_kinematics INTERFACE Eigen3::Eigen)

# ── 底盘驱动节点 ──────────────────────────────────────────────────
add_executable(chassis_driver_node
  src/chassis_driver_node.cpp
  src/chassis_interface_impl.cpp
  src/feetech_sts3215_bus.cpp
)
target_include_directories(chassis_driver_node PRIVATE include)
ament_target_dependencies(chassis_driver_node
  rclcpp geometry_msgs nav_msgs
)
target_link_libraries(chassis_driver_node three_wheel_kinematics platform_qos)
install(TARGETS chassis_driver_node DESTINATION lib/${PROJECT_NAME})

# ── 云台驱动节点 ──────────────────────────────────────────────────
add_executable(gimbal_driver_node src/gimbal_driver_node.cpp src/gimbal_interface_impl.cpp
  src/dji_rs2_protocol.cpp)
target_include_directories(gimbal_driver_node PRIVATE include)
ament_target_dependencies(gimbal_driver_node
  rclcpp rclcpp_lifecycle vision_servo_msgs sensor_msgs std_srvs
)
target_link_libraries(gimbal_driver_node platform_qos)
install(TARGETS gimbal_driver_node DESTINATION lib/${PROJECT_NAME})

# ── IMU 驱动节点 ──────────────────────────────────────────────────
add_executable(imu_driver_node src/imu_driver_node.cpp src/imu_interface_impl.cpp)
target_include_directories(imu_driver_node PRIVATE include)
ament_target_dependencies(imu_driver_node
  rclcpp sensor_msgs
)
target_link_libraries(imu_driver_node platform_qos)
install(TARGETS imu_driver_node DESTINATION lib/${PROJECT_NAME})

# ── 里程计节点 ────────────────────────────────────────────────────
add_executable(odometry_node src/odometry_node.cpp src/odometry_interface_impl.cpp)
target_include_directories(odometry_node PRIVATE include)
ament_target_dependencies(odometry_node
  rclcpp nav_msgs geometry_msgs tf2_ros sensor_msgs
)
target_link_libraries(odometry_node three_wheel_kinematics platform_qos)
install(TARGETS odometry_node DESTINATION lib/${PROJECT_NAME})

# ── 平台管理节点 ──────────────────────────────────────────────────
add_executable(platform_manager_node src/platform_manager_node.cpp)
target_include_directories(platform_manager_node PRIVATE include)
ament_target_dependencies(platform_manager_node
  rclcpp vision_servo_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros
)
target_link_libraries(platform_manager_node platform_qos)
install(TARGETS platform_manager_node DESTINATION lib/${PROJECT_NAME})

# ── 头文件 ────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)

# ── 启动文件和配置 ────────────────────────────────────────────────
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dji_rs2_protocol
    test/test_dji_rs2_protocol.cpp
    src/dji_rs2_protocol.cpp
  )
  target_include_directories(test_dji_rs2_protocol PRIVATE include)
endif()

ament_package()
