cmake_minimum_required(VERSION 3.5)
project(viso2_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisoInfo.msg"

  DEPENDENCIES
  builtin_interfaces
  std_msgs
)

ament_package()
