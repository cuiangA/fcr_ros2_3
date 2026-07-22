cmake_minimum_required(VERSION 3.8)
project(perception_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_servo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn video calib3d)
find_package(Threads REQUIRED)

option(
  PERCEPTION_BUILD_LEGACY_FUSION
  "Build the pre-calibration depth estimator and monolithic legacy pipeline"
  OFF)
option(
  PERCEPTION_ENABLE_TENSORRT
  "Build TensorRT engine support when TensorRT and CUDA runtime development files are present"
  ON)

add_library(perception_qos INTERFACE)
target_include_directories(perception_qos INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(perception_qos INTERFACE rclcpp::rclcpp)

add_library(yolo_processing src/yolo_processing.cpp)
target_include_directories(yolo_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(yolo_processing vision_servo_msgs)
target_link_libraries(yolo_processing ${OpenCV_LIBS})

add_executable(detection_node src/detection_node.cpp)
target_include_directories(detection_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(detection_node
  rclcpp rcl_interfaces diagnostic_updater diagnostic_msgs sensor_msgs std_msgs vision_servo_msgs
  cv_bridge image_transport)
target_link_libraries(detection_node
  perception_qos yolo_processing ${OpenCV_LIBS} Threads::Threads)

if(PERCEPTION_ENABLE_TENSORRT)
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    PATHS /usr/include /usr/include/aarch64-linux-gnu /usr/local/TensorRT/include)
  find_library(TENSORRT_LIBRARY nvinfer
    PATHS /usr/lib /usr/lib/aarch64-linux-gnu /usr/local/TensorRT/lib)
  find_path(CUDA_RUNTIME_INCLUDE_DIR cuda_runtime_api.h
    PATHS
      $ENV{CUDA_HOME}/include
      /usr/local/cuda/include
      /usr/local/cuda/targets/aarch64-linux/include
      /usr/local/cuda-12.5/targets/aarch64-linux/include
      /usr/include)
  find_library(CUDA_RUNTIME_LIBRARY cudart
    PATHS
      $ENV{CUDA_HOME}/lib64
      /usr/local/cuda/lib64
      /usr/local/cuda/targets/aarch64-linux/lib
      /usr/local/cuda-12.5/targets/aarch64-linux/lib
      /usr/lib/aarch64-linux-gnu)
  if(TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY AND
     CUDA_RUNTIME_INCLUDE_DIR AND CUDA_RUNTIME_LIBRARY)
    target_sources(detection_node PRIVATE src/tensorrt_backend.cpp)
    target_include_directories(detection_node PRIVATE
      ${TENSORRT_INCLUDE_DIR} ${CUDA_RUNTIME_INCLUDE_DIR})
    target_link_libraries(detection_node ${TENSORRT_LIBRARY} ${CUDA_RUNTIME_LIBRARY})
    target_compile_definitions(detection_node PRIVATE PERCEPTION_HAS_TENSORRT=1)
    message(STATUS "TensorRT detection backend enabled")
  else()
    message(STATUS
      "TensorRT development files not found; detection_node will support OpenCV ONNX only")
  endif()
endif()

add_executable(tracking_node
  src/tracking_node.cpp
  src/assignment_solver.cpp
  src/byte_tracker.cpp
  src/camera_motion_estimator.cpp
  src/multi_object_tracker.cpp
  src/target_selector.cpp)
target_include_directories(tracking_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(tracking_node
  rclcpp rcl_interfaces builtin_interfaces diagnostic_updater diagnostic_msgs sensor_msgs
  vision_servo_msgs cv_bridge)
target_link_libraries(tracking_node perception_qos ${OpenCV_LIBS})

if(PERCEPTION_BUILD_LEGACY_FUSION)
  message(WARNING
    "Building legacy depth/fusion nodes. They are not calibrated for the Sony + Orbbec pair "
    "and must not be used as the production /perception/targets_3d source.")
  add_executable(depth_estimator_node src/depth_estimator_node.cpp)
  target_include_directories(depth_estimator_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(depth_estimator_node
    rclcpp rclcpp_components sensor_msgs vision_servo_msgs cv_bridge image_transport
    tf2_ros tf2_geometry_msgs)
  target_link_libraries(depth_estimator_node perception_qos ${OpenCV_LIBS})

  add_library(perception_pipeline SHARED
    src/perception_pipeline.cpp
    src/assignment_solver.cpp
    src/multi_object_tracker.cpp)
  target_include_directories(perception_pipeline PRIVATE include ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(perception_pipeline
    rclcpp rclcpp_components sensor_msgs vision_servo_msgs cv_bridge image_transport)
  target_link_libraries(perception_pipeline perception_qos ${OpenCV_LIBS})
  rclcpp_components_register_node(perception_pipeline
    PLUGIN "perception_pkg::PerceptionPipeline"
    EXECUTABLE perception_pipeline_node)

  install(TARGETS depth_estimator_node DESTINATION lib/${PROJECT_NAME})
  install(TARGETS perception_pipeline
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endif()

add_executable(face_aim_node src/face_aim_node.cpp)
target_include_directories(face_aim_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(face_aim_node
  rclcpp sensor_msgs vision_servo_msgs cv_bridge image_transport)
target_link_libraries(face_aim_node perception_qos ${OpenCV_LIBS})

install(TARGETS
  yolo_processing
  detection_node
  tracking_node
  face_aim_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES models/README.md models/manifest.json DESTINATION share/${PROJECT_NAME}/models)
install(PROGRAMS
  scripts/download_model.py
  scripts/check_perception.sh
  scripts/jetson_build_perception.sh
  scripts/jetson_acceptance.sh
  DESTINATION lib/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models/yolov8n.onnx")
  install(FILES models/yolov8n.onnx DESTINATION share/${PROJECT_NAME}/models)
else()
  message(STATUS
    "yolov8n.onnx is not staged. Build succeeds without it; run scripts/download_model.py "
    "before launching detection or pass model_path explicitly.")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_multi_object_tracker
    test/test_multi_object_tracker.cpp
    src/assignment_solver.cpp
    src/multi_object_tracker.cpp)
  if(TARGET test_multi_object_tracker)
    target_include_directories(test_multi_object_tracker PRIVATE include ${OpenCV_INCLUDE_DIRS})
    # multi_object_tracker.cpp includes tracking_node.hpp, which also declares
    # the diagnostic_updater-backed ROS node wrapper.  The test therefore
    # needs the diagnostic headers even though it exercises only the tracker.
    ament_target_dependencies(test_multi_object_tracker
      rclcpp builtin_interfaces diagnostic_updater sensor_msgs vision_servo_msgs)
    target_link_libraries(test_multi_object_tracker ${OpenCV_LIBS})
  endif()
  ament_add_gtest(test_byte_tracker
    test/test_byte_tracker.cpp
    src/assignment_solver.cpp
    src/byte_tracker.cpp)
  if(TARGET test_byte_tracker)
    target_include_directories(test_byte_tracker PRIVATE include ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(test_byte_tracker vision_servo_msgs)
    target_link_libraries(test_byte_tracker ${OpenCV_LIBS})
  endif()
  ament_add_gtest(test_camera_motion_estimator
    test/test_camera_motion_estimator.cpp
    src/camera_motion_estimator.cpp)
  if(TARGET test_camera_motion_estimator)
    target_include_directories(test_camera_motion_estimator PRIVATE include ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(test_camera_motion_estimator vision_servo_msgs)
    target_link_libraries(test_camera_motion_estimator ${OpenCV_LIBS})
  endif()
  ament_add_gtest(test_yolo_processing test/test_yolo_processing.cpp)
  if(TARGET test_yolo_processing)
    target_include_directories(test_yolo_processing PRIVATE include ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(test_yolo_processing vision_servo_msgs)
    target_link_libraries(test_yolo_processing yolo_processing ${OpenCV_LIBS})
  endif()
  ament_add_gtest(test_target_selector
    test/test_target_selector.cpp
    src/target_selector.cpp)
  if(TARGET test_target_selector)
    target_include_directories(test_target_selector PRIVATE include)
    ament_target_dependencies(test_target_selector vision_servo_msgs)
  endif()
  add_launch_test(test/test_tracking_launch.py TIMEOUT 30)
endif()

# ── 模型文件（供 detection_params.yaml 用 find-pkg-share 解析） ───────
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

ament_package()
