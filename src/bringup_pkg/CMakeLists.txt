# bringup_pkg CMakeLists.txt
# 纯启动包 — 无编译内容，仅安装 launch/config/rviz 等运行资源

cmake_minimum_required(VERSION 3.8)
project(bringup_pkg)

# ament_cmake：ROS2 C++ 构建工具（纯资源包也需要声明）
find_package(ament_cmake REQUIRED)

# 将 launch、config、rviz 目录安装到包共享目录
# 运行时通过 FindPackageShare 引用这些资源
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
