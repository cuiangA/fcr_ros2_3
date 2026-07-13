# ── perception_pkg CMakeLists.txt ──────────────────────────────────
# 感知管线包：YOLO 目标检测 → 多目标跟踪 → 深度估计
#
# 构建目标：
#   1. perception_qos (INTERFACE 库) — 统一的 QoS 配置，各节点共享
#   2. detection_node — 独立可执行文件，YOLO 目标检测
#   3. tracking_node — 独立可执行文件，卡尔曼滤波多目标跟踪
#   4. depth_estimator_node — 独立可执行文件，单目深度估计
#   5. perception_pipeline (共享库) — 可组合节点，零拷贝管线
#
# C++17 和 -Wall -Wextra -Wpedantic 用于编译期错误检测
# ────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.8)
project(perception_pkg)

# C++17：OpenCV + rosidl 生成代码需要此标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 严格的编译警告，帮助尽早发现潜在问题
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖声明 ──────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)   # 可组合节点支持（零拷贝管线）
find_package(sensor_msgs REQUIRED)
find_package(vision_servo_msgs REQUIRED)   # 视觉伺服自定义消息类型
find_package(cv_bridge REQUIRED)           # OpenCV ↔ ROS 图像桥接
find_package(image_transport REQUIRED)     # 高效的图像传输与订阅
find_package(tf2_ros REQUIRED)             # TF 坐标变换支持
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# ── QoS 工具库（INTERFACE，仅头文件） ──────────────────────────────
add_library(perception_qos INTERFACE)
target_include_directories(perception_qos INTERFACE include)
target_link_libraries(perception_qos INTERFACE rclcpp::rclcpp)

# ── 检测节点（独立可执行文件） ────────────────────────────────────
add_executable(detection_node src/detection_node.cpp)
target_include_directories(detection_node PRIVATE include)
ament_target_dependencies(detection_node
  rclcpp rclcpp_components sensor_msgs vision_servo_msgs cv_bridge image_transport OpenCV
)
target_link_libraries(detection_node perception_qos)
install(TARGETS detection_node DESTINATION lib/${PROJECT_NAME})

# ── 跟踪节点（独立可执行文件） ────────────────────────────────────
add_executable(tracking_node src/tracking_node.cpp src/multi_object_tracker.cpp)
target_include_directories(tracking_node PRIVATE include)
ament_target_dependencies(tracking_node
  rclcpp rclcpp_components sensor_msgs vision_servo_msgs cv_bridge OpenCV
)
target_link_libraries(tracking_node perception_qos)
install(TARGETS tracking_node DESTINATION lib/${PROJECT_NAME})

# ── 深度估计节点 ──────────────────────────────────────────────────
add_executable(depth_estimator_node src/depth_estimator_node.cpp)
target_include_directories(depth_estimator_node PRIVATE include)
ament_target_dependencies(depth_estimator_node
  rclcpp rclcpp_components sensor_msgs vision_servo_msgs cv_bridge image_transport tf2_ros tf2_geometry_msgs
)
target_link_libraries(depth_estimator_node perception_qos)
install(TARGETS depth_estimator_node DESTINATION lib/${PROJECT_NAME})

# ── 可组合感知管线（共享库，支持零拷贝进程内通信） ────────────────
add_library(perception_pipeline SHARED src/perception_pipeline.cpp src/multi_object_tracker.cpp src/yolo_inference.cpp)
target_include_directories(perception_pipeline PRIVATE include)
ament_target_dependencies(perception_pipeline
  rclcpp rclcpp_components sensor_msgs vision_servo_msgs cv_bridge image_transport OpenCV
)
target_link_libraries(perception_pipeline perception_qos)
# 注册为可组合节点，支持通过 ComposableNodeContainer 动态加载
rclcpp_components_register_node(perception_pipeline
  PLUGIN "perception_pkg::PerceptionPipeline"
  EXECUTABLE perception_pipeline_node
)
install(TARGETS perception_pipeline
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ── 头文件（供其他包 include） ────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)

# ── 启动文件和配置 ────────────────────────────────────────────────
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
