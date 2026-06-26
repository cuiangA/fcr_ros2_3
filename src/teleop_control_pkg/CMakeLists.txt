cmake_minimum_required(VERSION 3.8)
project(teleop_control_pkg)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/keyboard_gimbal_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_gimbal_teleop
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
