# simulation_pkg CMakeLists.txt
# 协同控制仿真包 — 仅安装 coop_sim.py

cmake_minimum_required(VERSION 3.8)
project(simulation_pkg)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/coop_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
