cmake_minimum_required(VERSION 3.8)
project(sony_camera_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(
  SONY_CRSDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/sdk"
  CACHE PATH
  "Local Sony Camera Remote SDK staging root (contains include/CRSDK and lib)"
)
option(
  SONY_CRSDK_REQUIRED
  "Fail configuration instead of skipping the Sony driver when CRSDK is absent"
  OFF
)

set(CRSDK_INCLUDE_DIR "${SONY_CRSDK_ROOT}/include/CRSDK")
set(CRSDK_LIB_DIR "${SONY_CRSDK_ROOT}/lib")
set(CRSDK_CORE_HEADER "${CRSDK_INCLUDE_DIR}/CameraRemote_SDK.h")
set(CRSDK_CORE_LIBRARY "${CRSDK_LIB_DIR}/libCr_Core.so")

if(NOT EXISTS "${CRSDK_CORE_HEADER}" OR NOT EXISTS "${CRSDK_CORE_LIBRARY}")
  set(_crsdk_message
    "Sony CRSDK is not staged at ${SONY_CRSDK_ROOT}. "
    "Run scripts/install_crsdk.py with the extracted RemoteCli directory."
  )
  if(SONY_CRSDK_REQUIRED)
    message(FATAL_ERROR "${_crsdk_message}")
  endif()

  message(WARNING "${_crsdk_message} Skipping sony_camera_node.")
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
  install(FILES README.md DESTINATION share/${PROJECT_NAME})
  install(PROGRAMS scripts/install_crsdk.py DESTINATION share/${PROJECT_NAME}/scripts)
  ament_package()
  return()
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(Threads REQUIRED)

find_library(
  CRSDK_CORE_LIB
  NAMES Cr_Core
  PATHS "${CRSDK_LIB_DIR}"
  NO_DEFAULT_PATH
  REQUIRED
)

file(GLOB CRSDK_RUNTIME_LIBS "${CRSDK_LIB_DIR}/*.so*")
file(GLOB CRSDK_ADAPTER_LIBS "${CRSDK_LIB_DIR}/CrAdapter/*.so*")

add_library(sony_camera_streamer SHARED src/sony_camera_streamer.cpp)
target_include_directories(sony_camera_streamer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    "${CRSDK_INCLUDE_DIR}"
)
target_link_libraries(sony_camera_streamer PUBLIC "${CRSDK_CORE_LIB}" PRIVATE Threads::Threads)
target_compile_options(sony_camera_streamer PRIVATE -fsigned-char)
target_compile_definitions(sony_camera_streamer PRIVATE LINUX)

add_executable(sony_camera_node src/sony_camera_node.cpp)
target_include_directories(sony_camera_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(
  sony_camera_node
  rclcpp sensor_msgs std_msgs cv_bridge image_transport
  camera_info_manager diagnostic_updater diagnostic_msgs
)
target_link_libraries(sony_camera_node sony_camera_streamer ${OpenCV_LIBS})

set_target_properties(
  sony_camera_streamer sony_camera_node
  PROPERTIES
    BUILD_RPATH "${CRSDK_LIB_DIR};${CRSDK_LIB_DIR}/CrAdapter"
    INSTALL_RPATH "$ORIGIN;$ORIGIN/CrAdapter"
)

add_custom_command(TARGET sony_camera_node POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${CRSDK_LIB_DIR}" "$<TARGET_FILE_DIR:sony_camera_node>"
  COMMENT "Copying local Sony CRSDK runtime libraries"
)

install(TARGETS sony_camera_streamer sony_camera_node DESTINATION lib/${PROJECT_NAME})
install(FILES ${CRSDK_RUNTIME_LIBS} DESTINATION lib/${PROJECT_NAME})
if(CRSDK_ADAPTER_LIBS)
  install(FILES ${CRSDK_ADAPTER_LIBS} DESTINATION lib/${PROJECT_NAME}/CrAdapter)
endif()
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/install_crsdk.py DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
