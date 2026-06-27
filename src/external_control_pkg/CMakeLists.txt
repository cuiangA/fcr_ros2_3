# ── external_control_pkg CMakeLists.txt ────────────────────────────────
# 外部控制包：语音唤醒 + 录音 + HTTP 云端 ASR + 指令发布
#
# 构建目标：
#   1. wake_up_node — C++ 主节点（PortAudio + sherpa-onnx + libcurl）
#
# 构建依赖（需预先安装）：
#   sudo apt install portaudio19-dev libcurl4-openssl-dev nlohmann-json3-dev
#   pip3 install sherpa-onnx         （附带 C 头文件与动态库）
#   模型下载：见 README 或 package.xml 中的说明
# ────────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.8)
project(external_control_pkg)

# C++17：rosidl 代码生成 + nlohmann-json 需要
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖声明（ROS2 包） ──────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)           # Header 消息

# ── 依赖声明（系统库） ───────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
pkg_check_modules(LIBCURL REQUIRED libcurl)

# ── 查找 sherpa-onnx ─────────────────────────────────────────────────
# pip install sherpa-onnx 会将头文件与动态库安装在 Python site-packages 下，
# 常见位置是 ~/.local/lib/python3.1x/site-packages/_sherpa_onnx/lib/
# 若未找到，请在下方 SHERPA_ROOT 或 CMAKE_PREFIX_PATH 中指定。.
find_library(SHERPA_ONNX_LIB
  NAMES sherpa-onnx-c-api sherpa-onnx
  PATHS
    /usr/local/lib
    /usr/lib
    $ENV{HOME}/.local/lib
  PATH_SUFFIXES
    _sherpa_onnx/lib
  NO_DEFAULT_PATH
)
if(NOT SHERPA_ONNX_LIB)
  message(WARNING
    "sherpa-onnx 库未找到！请确保已安装:\n"
    "  pip3 install sherpa-onnx\n"
    "若已安装，请在 CMakeLists.txt 中手动设置 SHERPA_ROOT 或在 "
    "CMAKE_PREFIX_PATH 中添加路径。\n"
    "常见位置: ~/.local/lib/python3.10/site-packages/_sherpa_onnx/lib/")
endif()

find_path(SHERPA_ONNX_INCLUDE_DIR
  NAMES sherpa-onnx/c-api/c-api.h
  PATHS
    /usr/local/include
    /usr/include
    $ENV{HOME}/.local/include
    $ENV{HOME}/.local/lib
  PATH_SUFFIXES
    python3.10/site-packages/sherpa_onnx/include
    python3.11/site-packages/sherpa_onnx/include
    python3.12/site-packages/sherpa_onnx/include
  NO_DEFAULT_PATH
)

# ── 接口定义与代码生成 ───────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VoiceCommand.msg"
  DEPENDENCIES std_msgs
)

# ── 语音唤醒节点 ─────────────────────────────────────────────────────
add_executable(wake_up_node
  src/wake_up_node.cpp
)

ament_target_dependencies(wake_up_node
  rclcpp std_msgs
)

target_link_libraries(wake_up_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${PORTAUDIO_LIBRARIES}
  ${LIBCURL_LIBRARIES}
)

target_include_directories(wake_up_node PRIVATE
  ${PORTAUDIO_INCLUDE_DIRS}
  ${LIBCURL_INCLUDE_DIRS}
)

if(SHERPA_ONNX_LIB AND SHERPA_ONNX_INCLUDE_DIR)
  target_link_libraries(wake_up_node ${SHERPA_ONNX_LIB})
  target_include_directories(wake_up_node PRIVATE ${SHERPA_ONNX_INCLUDE_DIR})
  target_compile_definitions(wake_up_node PRIVATE HAS_SHERPA_ONNX=1)
else()
  target_compile_definitions(wake_up_node PRIVATE HAS_SHERPA_ONNX=0)
  message(STATUS "sherpa-onnx 未找到 — 唤醒节点将以降级模式编译（仅录音）")
endif()

# ⚠ nlohmann-json 是 header-only，依赖系统包 libnlohmann-json3-dev
#   若未安装，编译时将报找不到 <nlohmann/json.hpp>
#   安装：sudo apt install nlohmann-json3-dev

install(TARGETS wake_up_node DESTINATION lib/${PROJECT_NAME})

# ── 启动文件和模型 ───────────────────────────────────────────────────
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
