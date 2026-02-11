cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# 🟢 [新增] 必须找到这几个基础包
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 你的消息定义
set(msg_files
  "msg/AudioSpeech.msg"
  "msg/RobotState.msg"
  "msg/FaceEmotion.msg"
  "msg/VisionResult.msg"
  "msg/BluetoothCommand.msg"
  "msg/NetworkStatus.msg"
)

# 你的服务定义
set(srv_files
  "srv/AskLLM.srv"
  "srv/ConnectBluetooth.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  # 🟢 [核心修复] 必须声明依赖！没有这一行，就不会生成库文件！
  DEPENDENCIES std_msgs action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()