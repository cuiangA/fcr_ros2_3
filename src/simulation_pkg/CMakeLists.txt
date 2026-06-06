# ── simulation_pkg CMakeLists.txt ───────────────────────────────────
# 该包负责：
#   1. 编译 gimbal_controller_plugin（Gazebo 模型插件）
#   2. 安装 URDF 模型、世界文件、配置文件和启动文件到共享目录
#   3. 将 Python 脚本安装为可执行程序
# ────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.8)
project(simulation_pkg)

# C++17 for ROS2 + Gazebo compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ament_cmake 是 ROS2 中 CMake 包的最小构建依赖
find_package(ament_cmake REQUIRED)

# ── 编译依赖：gimbal_controller_plugin ────────────────────────────
# MVP mock/2D simulation does not need Gazebo. Keep the existing Gazebo plugin
# available when Gazebo dev packages are installed, but do not make it block
# lightweight builds.
option(BUILD_GAZEBO_PLUGIN "Build the legacy Gazebo gimbal controller plugin" ON)
find_package(rclcpp REQUIRED)
find_package(vision_servo_msgs REQUIRED)

if(BUILD_GAZEBO_PLUGIN)
  find_package(gazebo_dev QUIET)
  find_package(gazebo_ros QUIET)

  if(gazebo_dev_FOUND AND gazebo_ros_FOUND)
    add_library(gimbal_controller_plugin SHARED
      src/gimbal_controller_plugin.cpp
    )
    target_include_directories(gimbal_controller_plugin PRIVATE include)
    ament_target_dependencies(gimbal_controller_plugin
      gazebo_dev gazebo_ros rclcpp vision_servo_msgs
    )

    # ── 安装编译后的 Gazebo 插件 ──────────────────────────────────
    install(TARGETS gimbal_controller_plugin
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
    )
  else()
    message(WARNING
      "Gazebo dev packages not found; skipping gimbal_controller_plugin. "
      "MVP mock_target_publisher_node and simple_robot_sim_node do not require Gazebo.")
  endif()
endif()

# ── 安装资源文件 ──────────────────────────────────────────────────
# DIRECTORY 模式保留目录结构，便于 launch/urdf/worlds 文件在
# 安装后仍保持原有的相对路径关系
install(DIRECTORY urdf worlds launch
  DESTINATION share/${PROJECT_NAME}
)

# ── 安装 Python 脚本为可执行文件 ──────────────────────────────────
# PROGRAMS 模式确保脚本在安装后保留可执行权限（Linux 上为 +x）
install(PROGRAMS
  scripts/target_simulator.py
  scripts/camera_simulator.py
  scripts/mock_detector.py
  scripts/mock_target_publisher_node.py
  scripts/simple_robot_sim_node.py
  scripts/twist_stamped_to_twist.py
  DESTINATION lib/${PROJECT_NAME}
)

# 必须调用 ament_package() 以完成 ROS2 CMake 包的生成
ament_package()
