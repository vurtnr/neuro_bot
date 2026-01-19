cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# 引入消息生成器
find_package(rosidl_default_generators REQUIRED)

# 声明我们要编译哪些文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioSpeech.msg"
  "msg/RobotState.msg"
  "msg/FaceEmotion.msg"
  "srv/AskLLM.srv"
  "msg/VisionResult.msg"
  "srv/ConnectBluetooth.srv"
  "msg/BluetoothCommand.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()