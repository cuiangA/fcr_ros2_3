# ── external_control_pkg CMakeLists.txt ────────────────────────────────
# 外部控制包：语音唤醒 + 录音 + HTTP 云端 ASR + 指令发布
#
# 构建目标：
#   1. wake_up_node — C++ 主节点（PortAudio + sherpa-onnx + libcurl）
#   2. voice_gimbal_nudge_node — 语音短指令到 /voice/cmd_gimbal 的桥接节点
#   3. voice_command_dispatcher_node — 结构化意图校验和目标分流
#   4. command_router_node — 手动/语音/自主云台命令优先级仲裁
#   5. keyboard_gimbal_control_node — 键盘云台调试控制
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
find_package(std_srvs REQUIRED)           # Trigger 云台服务
find_package(vision_servo_msgs REQUIRED)  # GimbalCmd

# ── 依赖声明（系统库） ───────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
option(BUILD_WAKE_UP_NODE "Build PortAudio/sherpa-onnx wake-up and ASR node" ON)

pkg_check_modules(PORTAUDIO portaudio-2.0)
pkg_check_modules(LIBCURL libcurl)
find_path(NLOHMANN_JSON_INCLUDE_DIR
  NAMES nlohmann/json.hpp
)

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
    python3.10/site-packages/sherpa_onnx/lib
    python3.11/site-packages/sherpa_onnx/lib
    python3.12/site-packages/sherpa_onnx/lib
    _sherpa_onnx/lib
  NO_DEFAULT_PATH
)
if(BUILD_WAKE_UP_NODE AND NOT SHERPA_ONNX_LIB)
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
  "msg/VoiceDispatchStatus.msg"
  DEPENDENCIES std_msgs
)

set(EXTERNAL_CONTROL_TARGETS)

# ── 语音唤醒节点 ─────────────────────────────────────────────────────
if(BUILD_WAKE_UP_NODE
   AND PORTAUDIO_FOUND
   AND LIBCURL_FOUND
   AND NLOHMANN_JSON_INCLUDE_DIR
   AND SHERPA_ONNX_LIB
   AND SHERPA_ONNX_INCLUDE_DIR)
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
    ${SHERPA_ONNX_LIB}
  )

  target_include_directories(wake_up_node PRIVATE
    ${PORTAUDIO_INCLUDE_DIRS}
    ${LIBCURL_INCLUDE_DIRS}
    ${NLOHMANN_JSON_INCLUDE_DIR}
    ${SHERPA_ONNX_INCLUDE_DIR}
  )

  target_compile_definitions(wake_up_node PRIVATE HAS_SHERPA_ONNX=1)
  list(APPEND EXTERNAL_CONTROL_TARGETS wake_up_node)
else()
  message(WARNING
    "wake_up_node 未构建；缺少 PortAudio/libcurl/nlohmann-json/sherpa-onnx 中至少一项，"
    "或 BUILD_WAKE_UP_NODE=OFF。\n"
    "最小语音云台桥接 voice_gimbal_nudge_node 仍会构建。\n"
    "完整唤醒节点依赖安装：sudo apt install -y pkg-config portaudio19-dev "
    "libcurl4-openssl-dev nlohmann-json3-dev && pip3 install sherpa-onnx")
endif()

# ⚠ nlohmann-json 是 header-only，依赖系统包 libnlohmann-json3-dev
#   若未安装，编译时将报找不到 <nlohmann/json.hpp>
#   安装：sudo apt install nlohmann-json3-dev

# ── 语音云台短动作桥接节点 ─────────────────────────────────────────────
add_executable(voice_gimbal_nudge_node
  src/voice_gimbal_nudge_node.cpp
)

ament_target_dependencies(voice_gimbal_nudge_node
  rclcpp std_msgs std_srvs vision_servo_msgs
)

target_link_libraries(voice_gimbal_nudge_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

list(APPEND EXTERNAL_CONTROL_TARGETS voice_gimbal_nudge_node)

# ── 结构化语音指令分发节点 ───────────────────────────────────────────
add_executable(voice_command_dispatcher_node
  src/voice_command_dispatcher_node.cpp
)

target_include_directories(voice_command_dispatcher_node PRIVATE include)

ament_target_dependencies(voice_command_dispatcher_node
  rclcpp std_msgs
)

target_link_libraries(voice_command_dispatcher_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

list(APPEND EXTERNAL_CONTROL_TARGETS voice_command_dispatcher_node)

# ── 云台命令优先级仲裁节点 ───────────────────────────────────────────
add_executable(command_router_node
  src/command_router_node.cpp
)

ament_target_dependencies(command_router_node
  rclcpp std_msgs std_srvs vision_servo_msgs
)

list(APPEND EXTERNAL_CONTROL_TARGETS command_router_node)

# ── 键盘云台控制节点 ─────────────────────────────────────────────────
add_executable(keyboard_gimbal_control_node
  src/keyboard_gimbal_control_node.cpp
)

ament_target_dependencies(keyboard_gimbal_control_node
  rclcpp vision_servo_msgs
)

list(APPEND EXTERNAL_CONTROL_TARGETS keyboard_gimbal_control_node)

install(TARGETS ${EXTERNAL_CONTROL_TARGETS}
  DESTINATION lib/${PROJECT_NAME})

# ── 启动文件和模型 ───────────────────────────────────────────────────
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voice_intent_contract
    test/test_voice_intent_contract.cpp
  )
  target_include_directories(test_voice_intent_contract PRIVATE include)
endif()

ament_package()
