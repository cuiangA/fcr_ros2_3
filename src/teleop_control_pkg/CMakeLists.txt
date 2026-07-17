cmake_minimum_required(VERSION 3.8)
project(teleop_control_pkg)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_servo_msgs REQUIRED)

add_library(command_mux_core src/command_mux_core.cpp)
target_compile_features(command_mux_core PUBLIC cxx_std_17)
target_include_directories(command_mux_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(command_mux_node src/command_mux_node.cpp)
target_compile_features(command_mux_node PUBLIC cxx_std_17)
target_link_libraries(command_mux_node command_mux_core)
ament_target_dependencies(command_mux_node
  geometry_msgs rclcpp std_msgs vision_servo_msgs)

install(TARGETS command_mux_core command_mux_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(PROGRAMS scripts/keyboard_gimbal_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_gimbal_teleop
)

install(PROGRAMS scripts/keyboard_platform_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_platform_teleop
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_mux_core test/test_command_mux_core.cpp)
  if(TARGET test_command_mux_core)
    target_link_libraries(test_command_mux_core command_mux_core)
  endif()
endif()

ament_package()
