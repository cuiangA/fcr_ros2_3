# ── simulation_pkg CMakeLists.txt ───────────────────────────────────
# 该包为纯资源包（无编译目标），CMake 仅负责：
#   1. 安装 URDF 模型、世界文件、配置文件和启动文件到共享目录
#   2. 将 Python 脚本安装为可执行程序
# 无需 find_package 除 ament_cmake 之外的依赖：所有运行时依赖
# 均已在 package.xml 中声明。
# ────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.8)
project(simulation_pkg)

# ament_cmake 是 ROS2 中 CMake 包的最小构建依赖
find_package(ament_cmake REQUIRED)

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
  DESTINATION lib/${PROJECT_NAME}
)

# 必须调用 ament_package() 以完成 ROS2 CMake 包的生成
ament_package()
