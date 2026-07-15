cmake_minimum_required(VERSION 3.8)
project(remote_monitor_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(vision_servo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(perception_visualizer_node src/perception_visualizer_node.cpp)
target_include_directories(perception_visualizer_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(perception_visualizer_node
  rclcpp rcl_interfaces builtin_interfaces sensor_msgs std_msgs geometry_msgs diagnostic_msgs
  vision_servo_msgs cv_bridge)
target_link_libraries(perception_visualizer_node ${OpenCV_LIBS})

install(TARGETS perception_visualizer_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config foxglove
  DESTINATION share/${PROJECT_NAME})

ament_package()
